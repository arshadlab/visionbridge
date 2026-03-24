/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#ifndef VISIONBRIDGE_YOLO_TRACKER_H
#define VISIONBRIDGE_YOLO_TRACKER_H

#include "tracker.h"
#include "../common/config.h"

// ---------------------------------------------------------------------------
// YoloTracker
//
// Runs YOLOv4-tiny (or any compatible Darknet/ONNX model) through the
// OpenCV DNN module.  The DNN module is available in all standard OpenCV
// packages (libopencv-dnn-dev / libopencv-dev).
//
// Model files (not bundled — user must provide):
//   yolov4-tiny.weights  — https://github.com/AlexeyAB/darknet/releases
//   yolov4-tiny.cfg      — same repo
//   coco.names           — 80 class labels
//
// If any model file is missing or fails to load, init() returns false and
// SourceNode disables this detector gracefully (no crash).
//
// Pipeline per frame:
//   1. Resize to (input_width × input_height) blob.
//   2. Forward pass through the network.
//   3. Parse output layer detections.
//   4. Apply confidence threshold + NMS.
//   5. Return surviving boxes as DsBbox (capped at DS_MAX_BBOXES/3).
//
// detector_id = DS_DET_YOLO (rendered in red)
// ---------------------------------------------------------------------------

// Guard: the header compiles even without OpenCV DNN, but the .cpp is only
// compiled when DS_HAVE_OPENCV_DNN is defined by the build system.
#ifdef DS_HAVE_OPENCV_DNN
#  include <opencv2/dnn.hpp>
#endif

#include <set>
#include <string>
#include <vector>

class YoloTracker : public ITracker {
public:
    explicit YoloTracker(const DetectorConfig& cfg);
    ~YoloTracker() override = default;

    bool init(int width, int height) override;
    std::vector<DsBbox> process(const cv::Mat& frame) override;
    const char* name() const override { return "yolo"; }

private:
#ifdef DS_HAVE_OPENCV_DNN
    cv::dnn::Net m_net;
    std::vector<std::string> m_output_layers;
    std::vector<std::string> m_class_names;   ///< loaded from coco.names
    std::set<int>            m_filter_ids;    ///< class indices to keep (empty=all)
#endif
    DetectorConfig m_cfg;
    int m_width  = 0;
    int m_height = 0;
    bool m_loaded = false;
};

#endif // VISIONBRIDGE_YOLO_TRACKER_H
