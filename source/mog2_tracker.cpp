/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#include "mog2_tracker.h"
#include "../common/logger.h"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cinttypes>

Mog2Tracker::Mog2Tracker(const DetectorConfig& cfg) : m_cfg(cfg) {}

// ---------------------------------------------------------------------------
// init — create the MOG2 background subtractor
// ---------------------------------------------------------------------------
bool Mog2Tracker::init(int width, int height) {
    m_width  = width;
    m_height = height;

    m_bgsub = cv::createBackgroundSubtractorMOG2(
        m_cfg.mog2_history,
        m_cfg.mog2_var_threshold,
        /*detectShadows=*/false); // shadows labelled 127; we don't need them

    DS_INFO("Mog2Tracker: init %dx%d history=%d var_threshold=%.1f min_area=%d\n",
            width, height,
            m_cfg.mog2_history,
            m_cfg.mog2_var_threshold,
            m_cfg.mog2_min_area);
    return true;
}

// ---------------------------------------------------------------------------
// process — run MOG2, extract contours, return DsBbox list
// ---------------------------------------------------------------------------
std::vector<DsBbox> Mog2Tracker::process(const cv::Mat& frame, uint64_t /*seq*/) {
    if (frame.empty()) return {};

    // --- 1. Convert to grayscale ---
    cv::Mat gray;
    if (frame.channels() == 4) {
        cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
    } else if (frame.channels() == 3) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = frame;
    }

    // --- 2. MOG2 foreground mask ---
    cv::Mat fg_mask;
    m_bgsub->apply(gray, fg_mask);

    // --- 3. Morphological cleanup ---
    // Opening removes tiny noise specks; dilation fills small gaps in objects.
    const cv::Mat kernel3 = cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3});
    const cv::Mat kernel7 = cv::getStructuringElement(cv::MORPH_ELLIPSE, {7, 7});
    cv::morphologyEx(fg_mask, fg_mask, cv::MORPH_OPEN,  kernel3);
    cv::dilate      (fg_mask, fg_mask, kernel7);

    // --- 4. Find contours ---
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(fg_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // --- 5. Filter and convert to DsBbox ---
    // Sort by area descending: report the largest objects first.
    std::sort(contours.begin(), contours.end(),
        [](const auto& a, const auto& b) {
            return cv::contourArea(a) > cv::contourArea(b);
        });

    std::vector<DsBbox> result;
    const int max_boxes = DS_MAX_BBOXES / 3; // reserve equal share per detector

    for (const auto& c : contours) {
        if (static_cast<int>(result.size()) >= max_boxes) break;

        const cv::Rect r = cv::boundingRect(c);
        if (r.area() < m_cfg.mog2_min_area) continue;

        // Clamp to frame bounds
        const cv::Rect safe(
            std::max(r.x, 0), std::max(r.y, 0),
            std::min(r.x + r.width,  m_width)  - std::max(r.x, 0),
            std::min(r.y + r.height, m_height) - std::max(r.y, 0));
        if (safe.width <= 0 || safe.height <= 0) continue;

        DsBbox b{};
        b.x           = safe.x;
        b.y           = safe.y;
        b.w           = safe.width;
        b.h           = safe.height;
        // Use relative area as a simple confidence proxy (0..1)
        b.confidence  = std::min(1.0f, static_cast<float>(r.area()) /
                                       static_cast<float>(m_width * m_height));
        b.detector_id = DS_DET_MOG2;
        result.push_back(b);
    }

    return result;
}
