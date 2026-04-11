/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#ifndef VISIONBRIDGE_MOG2_TRACKER_H
#define VISIONBRIDGE_MOG2_TRACKER_H

#include "tracker.h"
#include "../common/config.h"
#include <opencv2/video/background_segm.hpp>
#include <memory>

// ---------------------------------------------------------------------------
// Mog2Tracker
//
// Uses OpenCV BackgroundSubtractorMOG2 to extract a foreground mask, then
// finds contours and converts each sufficiently large contour to a DsBbox.
//
// Algorithm:
//   1. Convert frame to grayscale (MOG2 accepts grayscale or colour; grayscale
//      is faster and sufficient for motion-based surveillance).
//   2. Apply MOG2 → binary foreground mask.
//   3. Morphological open + dilate to remove noise and fill small holes.
//   4. Find contours; filter by bounding-rect area ≥ mog2_min_area.
//   5. Emit one DsBbox per surviving contour (capped at DS_MAX_BBOXES/3).
//
// Parameters come from DetectorConfig::mog2_* fields:
//   history:        frames used to build background model (default 500)
//   var_threshold:  Mahalanobis distance² threshold (default 16)
//   min_area:       minimum bounding-rect area in px² to report (default 800)
//
// detector_id = DS_DET_MOG2 (rendered in green)
// ---------------------------------------------------------------------------
class Mog2Tracker : public ITracker {
public:
    explicit Mog2Tracker(const DetectorConfig& cfg);
    ~Mog2Tracker() override = default;

    bool init(int width, int height) override;
    std::vector<DsBbox> process(const cv::Mat& frame, uint64_t seq) override;
    const char* name() const override { return "mog2"; }

private:
    DetectorConfig m_cfg;
    cv::Ptr<cv::BackgroundSubtractorMOG2> m_bgsub;
    int m_width  = 0;
    int m_height = 0;
};

#endif // VISIONBRIDGE_MOG2_TRACKER_H
