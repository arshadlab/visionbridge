/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#include "stub_tracker.h"
#include "../common/logger.h"
#include <cmath>

bool StubTracker::init(int width, int height) {
    m_width  = width;
    m_height = height;
    DS_INFO("StubTracker: init %dx%d\n", width, height);
    return true;
}

// ---------------------------------------------------------------------------
// process — emit one drifting bounding box per frame
//
// The box centre oscillates on a Lissajous-like path so that it moves
// visibly across the frame and the overlay clearly updates each frame.
// ---------------------------------------------------------------------------
std::vector<DsBbox> StubTracker::process(const cv::Mat& /*frame*/) {
    const double t = static_cast<double>(m_seq++) * 0.033; // ~30fps → 1 s per cycle

    // Box size: ~25% of frame
    const int bw = m_width  / 4;
    const int bh = m_height / 4;

    // Centre oscillates within [bw/2 … W-bw/2] x [bh/2 … H-bh/2]
    const double cx = (m_width  - bw) * 0.5 * (1.0 + 0.5 * std::sin(t * 0.7));
    const double cy = (m_height - bh) * 0.5 * (1.0 + 0.5 * std::cos(t * 1.1));

    DsBbox b{};
    b.x           = static_cast<int32_t>(cx - bw * 0.5);
    b.y           = static_cast<int32_t>(cy - bh * 0.5);
    b.w           = bw;
    b.h           = bh;
    b.confidence  = 0.5f;
    b.detector_id = DS_DET_STUB;

    return { b };
}
