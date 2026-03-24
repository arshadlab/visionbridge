/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#include "yolo_tracker.h"
#include "../common/logger.h"

#ifdef DS_HAVE_OPENCV_DNN

#include <opencv2/imgproc.hpp>
#include <opencv2/core/ocl.hpp>
#include <algorithm>
#include <cctype>
#include <fstream>

// Lowercase helper for case-insensitive class name matching
static std::string to_lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c){ return std::tolower(c); });
    return s;
}

YoloTracker::YoloTracker(const DetectorConfig& cfg) : m_cfg(cfg) {}

// ---------------------------------------------------------------------------
// init — load model weights and configuration file
// ---------------------------------------------------------------------------
bool YoloTracker::init(int width, int height) {
    m_width  = width;
    m_height = height;

    const std::string& model_path  = m_cfg.yolo.model;
    const std::string& config_path = m_cfg.yolo.config;

    if (model_path.empty()) {
        DS_WARN("YoloTracker: no model path configured — YOLO disabled\n");
        return false;
    }

    try {
        if (config_path.empty()) {
            // ONNX or other self-describing format
            m_net = cv::dnn::readNet(model_path);
        } else {
            // Darknet .weights + .cfg
            m_net = cv::dnn::readNetFromDarknet(config_path, model_path);
        }
    } catch (const cv::Exception& e) {
        DS_WARN("YoloTracker: failed to load model '%s': %s — YOLO disabled\n",
                model_path.c_str(), e.what());
        return false;
    }

    if (m_net.empty()) {
        DS_WARN("YoloTracker: empty network after load — YOLO disabled\n");
        return false;
    }

    // Use CPU backend (set DNN_BACKEND_CUDA + DNN_TARGET_CUDA for GPU)
    m_net.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
    m_net.setPreferableTarget (cv::dnn::DNN_TARGET_CPU);

    // Apply configured inference backend with graceful CPU fallback.
    // opencl / opencl_fp16 work on Intel, AMD, and NVIDIA GPUs via OpenCL
    // (no special OpenCV build needed beyond the default WITH_OPENCL=ON).
    // cuda / cuda_fp16 require OpenCV built with WITH_CUDA=ON.
    const std::string& be = m_cfg.yolo.backend;
    if (be == "opencl" || be == "opencl_fp16") {
        if (!cv::ocl::haveOpenCL()) {
            DS_WARN("YoloTracker: backend=%s requested but OpenCL not available — using CPU\n",
                    be.c_str());
            m_net.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
            m_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        } else {
            const auto target = (be == "opencl_fp16")
                                ? cv::dnn::DNN_TARGET_OPENCL_FP16
                                : cv::dnn::DNN_TARGET_OPENCL;
            m_net.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
            m_net.setPreferableTarget(target);
            DS_INFO("YoloTracker: backend=%s enabled\n", be.c_str());
        }
    } else if (be == "cuda" || be == "cuda_fp16") {
        const auto target = (be == "cuda_fp16")
                            ? cv::dnn::DNN_TARGET_CUDA_FP16
                            : cv::dnn::DNN_TARGET_CUDA;
        try {
            m_net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            m_net.setPreferableTarget(target);
            DS_INFO("YoloTracker: backend=%s enabled\n", be.c_str());
        } catch (const cv::Exception& e) {
            DS_WARN("YoloTracker: backend=%s unavailable (%s) — falling back to CPU\n",
                    be.c_str(), e.what());
            m_net.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
            m_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        }
    } else {
        DS_INFO("YoloTracker: backend=cpu\n");
    }

    // Collect names of the output layers (e.g. yolo_82, yolo_94, yolo_106)
    const auto& layer_names = m_net.getLayerNames();
    const auto  unconnected = m_net.getUnconnectedOutLayers();
    m_output_layers.clear();
    for (int idx : unconnected) {
        m_output_layers.push_back(layer_names[static_cast<size_t>(idx) - 1]);
    }

    m_loaded = true;
    DS_INFO("YoloTracker: loaded '%s' output_layers=%zu\n",
            model_path.c_str(), m_output_layers.size());

    // ----- Load class names -----
    m_class_names.clear();
    if (!m_cfg.yolo.names.empty()) {
        std::ifstream nf(m_cfg.yolo.names);
        if (nf.is_open()) {
            std::string line;
            while (std::getline(nf, line)) {
                // strip trailing \r for Windows-style files
                if (!line.empty() && line.back() == '\r') line.pop_back();
                m_class_names.push_back(line);
            }
            DS_INFO("YoloTracker: loaded %zu class names from '%s'\n",
                    m_class_names.size(), m_cfg.yolo.names.c_str());
        } else {
            DS_WARN("YoloTracker: could not open names file '%s' — class filtering disabled\n",
                    m_cfg.yolo.names.c_str());
        }
    }

    // ----- Build filter set (class indices to keep) -----
    m_filter_ids.clear();
    if (!m_cfg.yolo.filter_classes.empty() && !m_class_names.empty()) {
        for (const auto& want : m_cfg.yolo.filter_classes) {
            const std::string want_lc = to_lower(want);
            for (size_t i = 0; i < m_class_names.size(); ++i) {
                if (to_lower(m_class_names[i]) == want_lc) {
                    m_filter_ids.insert(static_cast<int>(i));
                    DS_INFO("YoloTracker: filtering class '%s' (id=%zu)\n",
                            m_class_names[i].c_str(), i);
                }
            }
        }
        if (m_filter_ids.empty())
            DS_WARN("YoloTracker: none of the filter_classes matched any known class name\n");
    }

    return true;
}

// ---------------------------------------------------------------------------
// process — run inference and return NMS-filtered DsBbox list
// ---------------------------------------------------------------------------
std::vector<DsBbox> YoloTracker::process(const cv::Mat& frame) {
    if (!m_loaded || frame.empty()) return {};

    // ------------------------------------------------------------------
    // 1. Build 4D blob from frame
    //    The pipeline delivers BGRx (4-channel).  blobFromImage / the
    //    first YOLO convolution expects exactly 3 channels.
    //    Convert BGRx → BGR (zero-copy colour conversion, no resize yet).
    // ------------------------------------------------------------------
    cv::Mat bgr;
    if (frame.channels() == 4) {
        cv::cvtColor(frame, bgr, cv::COLOR_BGRA2BGR);
    } else {
        bgr = frame; // already 3-channel
    }

    cv::Mat blob;
    cv::dnn::blobFromImage(bgr,
                           blob,
                           1.0 / 255.0,
                           cv::Size(m_cfg.yolo.input_width, m_cfg.yolo.input_height),
                           cv::Scalar(0, 0, 0),
                           /*swapRB=*/true,
                           /*crop=*/false);
    m_net.setInput(blob);

    // ------------------------------------------------------------------
    // 2. Forward pass
    // ------------------------------------------------------------------
    std::vector<cv::Mat> outs;
    m_net.forward(outs, m_output_layers);

    // ------------------------------------------------------------------
    // 3. Parse detections
    // ------------------------------------------------------------------
    std::vector<int>    class_ids;
    std::vector<float>  confidences;
    std::vector<cv::Rect> boxes;

    for (const auto& out : outs) {
        // Each row: [cx, cy, w, h, obj_score, class_scores…] — normalised
        const float* data = reinterpret_cast<const float*>(out.data);
        for (int i = 0; i < out.rows; ++i, data += out.cols) {
            // class scores start at column 5
            cv::Mat scores(1, out.cols - 5, CV_32F,
                           const_cast<float*>(data + 5));
            cv::Point class_id_pt;
            double max_score = 0.0;
            cv::minMaxLoc(scores, nullptr, &max_score, nullptr, &class_id_pt);

            const float confidence = data[4] * static_cast<float>(max_score);
            if (confidence < m_cfg.yolo.conf_threshold) continue;

            // Class filter: skip if a filter is active and this class is not in it
            if (!m_filter_ids.empty() &&
                m_filter_ids.find(class_id_pt.x) == m_filter_ids.end()) continue;

            const int cx = static_cast<int>(data[0] * m_width);
            const int cy = static_cast<int>(data[1] * m_height);
            const int bw = static_cast<int>(data[2] * m_width);
            const int bh = static_cast<int>(data[3] * m_height);

            boxes.emplace_back(cx - bw / 2, cy - bh / 2, bw, bh);
            confidences.push_back(confidence);
            class_ids.push_back(class_id_pt.x);
        }
    }

    // ------------------------------------------------------------------
    // 4. Non-maximum suppression
    // ------------------------------------------------------------------
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, m_cfg.yolo.conf_threshold,
                      m_cfg.yolo.nms_threshold, indices);

    // ------------------------------------------------------------------
    // 5. Convert to DsBbox list
    // ------------------------------------------------------------------
    const int max_boxes = DS_MAX_BBOXES / 3;
    std::vector<DsBbox> result;
    result.reserve(std::min(static_cast<int>(indices.size()), max_boxes));

    for (int idx : indices) {
        if (static_cast<int>(result.size()) >= max_boxes) break;
        const cv::Rect& r = boxes[static_cast<size_t>(idx)];

        DsBbox b{};
        b.x           = std::max(r.x, 0);
        b.y           = std::max(r.y, 0);
        b.w           = std::min(r.x + r.width,  m_width)  - b.x;
        b.h           = std::min(r.y + r.height, m_height) - b.y;
        b.confidence  = confidences[static_cast<size_t>(idx)];
        b.detector_id = DS_DET_YOLO;
        result.push_back(b);
    }

    DS_TRACE("YoloTracker: %zu boxes after NMS\n", result.size());
    return result;
}

#else // DS_HAVE_OPENCV_DNN not defined

// ---------------------------------------------------------------------------
// Stub implementation when OpenCV DNN is not available at build time
// ---------------------------------------------------------------------------

YoloTracker::YoloTracker(const DetectorConfig& cfg) : m_cfg(cfg) {}

bool YoloTracker::init(int width, int height) {
    m_width  = width;
    m_height = height;
    DS_WARN("YoloTracker: built without DS_HAVE_OPENCV_DNN — YOLO disabled\n");
    return false;
}

std::vector<DsBbox> YoloTracker::process(const cv::Mat& /*frame*/) {
    return {};
}

#endif // DS_HAVE_OPENCV_DNN
