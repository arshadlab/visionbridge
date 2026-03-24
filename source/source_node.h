/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#ifndef VISIONBRIDGE_SOURCE_NODE_H
#define VISIONBRIDGE_SOURCE_NODE_H

#include "../common/config.h"
#include "../common/protocol.h"
#include "capture_pipeline.h"

#include <atomic>
#include <memory>
#include <thread>

// ---------------------------------------------------------------------------
// SourceNode
//
// Top-level orchestrator for the VisionBridge source process.
//
// Responsibilities:
//   1. Initialise all enabled ITracker instances (MOG2, YOLO, Stub).
//   2. Construct CapturePipeline with those trackers.
//   3. Start the ZMQ PUSH socket bound to zmq.source_endpoint.
//   4. Register the bbox callback: ZMQ-send each DsMsgBbox produced by pipeline.
//   5. Block in run() until stop is requested.
//
// Thread model:
//   GStreamer runs its own encode/thread.
//   The appsink callback (inside CapturePipeline::on_new_sample) runs all
//   trackers synchronously and then fires the bbox_callback which calls
//   zmq_send() — all on GStreamer's streaming thread.
//   ZMQ PUSH is thread-safe: no additional locking needed.
// ---------------------------------------------------------------------------
class SourceNode {
public:
    explicit SourceNode(const DsSourceConfig& cfg);
    ~SourceNode();

    /// Initialise trackers, pipeline, and ZMQ socket.  False on any failure.
    bool init();

    /// Begin capture+detection+streaming.  Blocks until request_stop().
    int run();

    /// Graceful shutdown from any thread (signal handler, etc.)
    void request_stop();

private:
    void send_bbox(const DsMsgBbox& msg);

    DsSourceConfig m_cfg;

    std::unique_ptr<CapturePipeline> m_pipeline;

    void*  m_zmq_ctx  = nullptr;
    void*  m_zmq_push = nullptr;

    std::atomic<bool> m_stop_requested{false};
};

#endif // VISIONBRIDGE_SOURCE_NODE_H
