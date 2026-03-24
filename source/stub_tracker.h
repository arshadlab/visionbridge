/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#ifndef VISIONBRIDGE_STUB_TRACKER_H
#define VISIONBRIDGE_STUB_TRACKER_H

#include "tracker.h"
#include <cstdint>

// ---------------------------------------------------------------------------
// StubTracker
//
// Returns a single pseudo-random bounding box per frame, centred in the
// frame with small position jitter.  Useful for:
//   - Verifying the ZMQ bbox pipeline and SDL2 overlay without real detection
//   - Providing a visible overlay when no model files are available
//   - Load testing the bbox channel throughput
//
// The stub box slowly drifts to avoid a perfectly static rectangle that might
// be mistaken for a frozen display.
//
// detector_id = DS_DET_STUB (rendered in gray)
// ---------------------------------------------------------------------------
class StubTracker : public ITracker {
public:
    StubTracker() = default;
    ~StubTracker() override = default;

    bool init(int width, int height) override;
    std::vector<DsBbox> process(const cv::Mat& frame) override;
    const char* name() const override { return "stub"; }

private:
    int      m_width  = 1280;
    int      m_height = 720;
    uint64_t m_seq    = 0;
};

#endif // VISIONBRIDGE_STUB_TRACKER_H
