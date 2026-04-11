/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#ifndef VISIONBRIDGE_TRACKER_H
#define VISIONBRIDGE_TRACKER_H

#include "../common/protocol.h"
#include <opencv2/core.hpp>
#include <vector>

// ---------------------------------------------------------------------------
// ITracker — abstract object-detection interface
//
// Every detector (MOG2, YOLO, Stub) implements this interface.  The
// CapturePipeline calls process() on each one per captured frame, aggregates
// the results into a DsMsgBbox, and sends it via ZMQ to the render node.
//
// Lifecycle:
//   1. Construct
//   2. init() — allocate resources (background model, DNN weights, …)
//   3. process(frame) → vector<DsBbox>  — called per frame (appsink callback)
//   4. Destroy (resources freed in destructor)
//
// Thread safety: process() is called from a single GStreamer streaming thread.
// No shared state between different ITracker instances.
// ---------------------------------------------------------------------------
class ITracker {
public:
    virtual ~ITracker() = default;

    /// One-time initialisation.  Returns false on failure (e.g. model load error).
    /// @param width  — input frame width (must match CapturePipeline output)
    /// @param height — input frame height
    virtual bool init(int width, int height) = 0;

    /// Detect objects in a BGR or BGRA frame.
    /// Each DsBbox must have detector_id set by the implementation.
    /// Returns an empty vector if nothing is detected or the detector is disabled.
    virtual std::vector<DsBbox> process(const cv::Mat& frame, uint64_t seq) = 0;

    /// Human-readable detector name (used in debug output).
    virtual const char* name() const = 0;
};

#endif // VISIONBRIDGE_TRACKER_H
