/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#include "source_node.h"
#include "stub_tracker.h"
#include "mog2_tracker.h"
#include "yolo_tracker.h"
#include "../common/logger.h"
#include "../common/time_utils.h"

#include <zmq.h>
#include <cinttypes>
#include <cstring>

// ---------------------------------------------------------------------------
SourceNode::SourceNode(const DsSourceConfig& cfg) : m_cfg(cfg) {}

SourceNode::~SourceNode() {
    request_stop();
    if (m_zmq_push) { zmq_close(m_zmq_push);   m_zmq_push = nullptr; }
    if (m_zmq_ctx)  { zmq_ctx_destroy(m_zmq_ctx); m_zmq_ctx = nullptr; }
}

// ---------------------------------------------------------------------------
// init
// ---------------------------------------------------------------------------
bool SourceNode::init() {
    const auto& det = m_cfg.detectors;
    const int W = static_cast<int>(m_cfg.input.width);
    const int H = static_cast<int>(m_cfg.input.height);

    // ----- Build tracker list -----
    std::vector<std::unique_ptr<ITracker>> trackers;

    if (det.enable_stub) {
        auto t = std::make_unique<StubTracker>();
        if (t->init(W, H)) {
            DS_INFO("SourceNode: StubTracker enabled\n");
            trackers.push_back(std::move(t));
        }
    }

    if (det.enable_mog2) {
        auto t = std::make_unique<Mog2Tracker>(det);
        if (t->init(W, H)) {
            DS_INFO("SourceNode: Mog2Tracker enabled\n");
            trackers.push_back(std::move(t));
        }
    }

    if (det.enable_yolo) {
        auto t = std::make_unique<YoloTracker>(det);
        if (t->init(W, H)) {
            DS_INFO("SourceNode: YoloTracker enabled\n");
            trackers.push_back(std::move(t));
        } else {
            DS_WARN("SourceNode: YoloTracker disabled (model missing or DNN unavailable)\n");
        }
    }

    if (trackers.empty())
        DS_WARN("SourceNode: no detectors initialised — bbox channel will be silent\n");

    // ----- Construct pipeline -----
    m_pipeline = std::make_unique<CapturePipeline>(m_cfg, std::move(trackers));
    m_pipeline->set_bbox_callback([this](const DsMsgBbox& msg) { send_bbox(msg); });

    // ----- ZMQ PUSH socket -----
    m_zmq_ctx = zmq_ctx_new();
    if (!m_zmq_ctx) {
        DS_ERR("SourceNode: zmq_ctx_new failed\n");
        return false;
    }

    m_zmq_push = zmq_socket(m_zmq_ctx, ZMQ_PUSH);
    if (!m_zmq_push) {
        DS_ERR("SourceNode: zmq_socket(PUSH) failed\n");
        return false;
    }

    // HWM=1: if render is slow, drop old bboxes — always deliver the latest.
    int hwm = 1;
    zmq_setsockopt(m_zmq_push, ZMQ_SNDHWM, &hwm, sizeof(hwm));
    // Non-blocking send: return immediately if full (EAGAIN → we just drop)
    // The actual drop is handled by the HWM mechanism; this timeout just
    // ensures zmq_send doesn't stall the appsink streaming thread.
    int timeout_ms = 0;
    zmq_setsockopt(m_zmq_push, ZMQ_SNDTIMEO, &timeout_ms, sizeof(timeout_ms));

    if (m_cfg.transport.multicast) {
        // Multicast: bind — render PULL connects to us
        const std::string& ep = m_cfg.zmq.source_endpoint;
        if (zmq_bind(m_zmq_push, ep.c_str()) != 0) {
            DS_ERR("SourceNode: zmq_bind(%s) failed: %s\n",
                   ep.c_str(), zmq_strerror(zmq_errno()));
            return false;
        }
        DS_INFO("SourceNode: ZMQ PUSH bound to %s\n", ep.c_str());
    } else {
        // Unicast: connect to render — render PULL binds
        const std::string& ep = m_cfg.zmq.render_endpoint;
        if (zmq_connect(m_zmq_push, ep.c_str()) != 0) {
            DS_ERR("SourceNode: zmq_connect(%s) failed: %s\n",
                   ep.c_str(), zmq_strerror(zmq_errno()));
            return false;
        }
        DS_INFO("SourceNode: ZMQ PUSH connected to %s\n", ep.c_str());
    }
    return true;
}

// ---------------------------------------------------------------------------
// run
// ---------------------------------------------------------------------------
int SourceNode::run() {
    if (!m_pipeline->start()) {
        DS_ERR("SourceNode: CapturePipeline start failed\n");
        return 1;
    }

    DS_INFO("SourceNode: running. Press Ctrl+C to stop.\n");

    while (!m_stop_requested.load() && m_pipeline->is_running()) {
        ds_sleep_ms(100);
    }

    m_pipeline->stop();
    DS_INFO("SourceNode: stopped\n");
    return 0;
}

// ---------------------------------------------------------------------------
// request_stop
// ---------------------------------------------------------------------------
void SourceNode::request_stop() {
    m_stop_requested.store(true);
    if (m_pipeline) m_pipeline->stop();
}

// ---------------------------------------------------------------------------
// send_bbox — called from CapturePipeline's appsink callback thread
// ---------------------------------------------------------------------------
void SourceNode::send_bbox(const DsMsgBbox& msg) {
    if (!m_zmq_push) return;
    // zmq_send is thread-safe; PUSH socket with HWM=1 drops oldest on overflow.
    int rc = zmq_send(m_zmq_push, &msg, sizeof(msg), ZMQ_NOBLOCK);
    if (rc < 0 && zmq_errno() != EAGAIN) {
        DS_WARN("SourceNode: zmq_send failed: %s\n", zmq_strerror(zmq_errno()));
    }
}
