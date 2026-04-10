/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#include "display_output_base.h"
#include "../common/logger.h"
#include "../common/time_utils.h"

#include <cinttypes>

// ---------------------------------------------------------------------------
BboxReceiver::BboxReceiver(const DsRenderConfig& cfg) : m_cfg(cfg) {}

// ---------------------------------------------------------------------------
bool BboxReceiver::start() {
    m_zmq_ctx    = zmq_ctx_new();
    m_zmq_socket = zmq_socket(m_zmq_ctx, ZMQ_PULL);
    if (!m_zmq_socket) {
        DS_ERR("BboxReceiver: zmq_socket failed\n");
        return false;
    }

    int hwm = 1;
    zmq_setsockopt(m_zmq_socket, ZMQ_RCVHWM, &hwm, sizeof(hwm));
    int rcvtimeo = 100; // ms — allows graceful shutdown
    zmq_setsockopt(m_zmq_socket, ZMQ_RCVTIMEO, &rcvtimeo, sizeof(rcvtimeo));

    if (m_cfg.transport.multicast) {
        const std::string ep = m_cfg.zmq.render_endpoint;
        if (zmq_connect(m_zmq_socket, ep.c_str()) != 0) {
            DS_ERR("BboxReceiver: ZMQ connect to %s failed: %s\n",
                   ep.c_str(), zmq_strerror(zmq_errno()));
            return false;
        }
        DS_INFO("BboxReceiver: ZMQ PULL connected to %s\n", ep.c_str());
    } else {
        const std::string ep = m_cfg.zmq.source_endpoint;
        if (zmq_bind(m_zmq_socket, ep.c_str()) != 0) {
            DS_ERR("BboxReceiver: ZMQ bind to %s failed: %s\n",
                   ep.c_str(), zmq_strerror(zmq_errno()));
            return false;
        }
        DS_INFO("BboxReceiver: ZMQ PULL bound to %s\n", ep.c_str());
    }

    m_running.store(true);
    m_thread = std::thread(&BboxReceiver::recv_loop, this);
    return true;
}

// ---------------------------------------------------------------------------
void BboxReceiver::stop() {
    if (!m_running.exchange(false)) return;
    if (m_thread.joinable()) m_thread.join();
    if (m_zmq_socket) { zmq_close(m_zmq_socket); m_zmq_socket = nullptr; }
    if (m_zmq_ctx)    { zmq_ctx_destroy(m_zmq_ctx); m_zmq_ctx = nullptr; }
}

// ---------------------------------------------------------------------------
void BboxReceiver::recv_loop() {
    DS_DBG("BboxReceiver: recv thread started\n");
    while (m_running.load()) {
        DsMsgBbox msg{};
        const int n = zmq_recv(m_zmq_socket, &msg, sizeof(msg), 0);
        if (n < 0) {
            if (zmq_errno() == EAGAIN || zmq_errno() == EINTR) continue;
            DS_WARN("BboxReceiver: recv error: %s\n", zmq_strerror(zmq_errno()));
            continue;
        }
        if (static_cast<size_t>(n) < sizeof(DsMsgBbox) || msg.type != DS_MSG_BBOX)
            continue;

        auto shared = std::make_shared<DsMsgBbox>(msg);
        {
            std::lock_guard<std::mutex> lk(m_mtx);
            auto& slot = m_ring[m_ring_head % kRingSize];
            slot.frame_seq = msg.frame_seq;
            slot.msg       = std::move(shared);
            ++m_ring_head;
        }
    }
    DS_DBG("BboxReceiver: recv thread exiting\n");
}

// ---------------------------------------------------------------------------
std::shared_ptr<DsMsgBbox> BboxReceiver::best_for_frame(const DecodedFrame* frame) {
    std::lock_guard<std::mutex> lk(m_mtx);

    if (!frame) return nullptr;

    const uint64_t want_seq = frame->sei_frame_seq;

    // --- No SEI: use latest available (no correlation possible) ---
    if (want_seq == 0) {
        std::shared_ptr<DsMsgBbox> latest;
        uint64_t latest_seq = 0;
        for (auto& e : m_ring) {
            if (e.msg && e.frame_seq > latest_seq) {
                latest_seq = e.frame_seq;
                latest = e.msg;
            }
        }
        return latest;
    }

    // --- SEI mode: seq-keyed correlation ---

    // Purge entries older than (want_seq - kMaxSkewFrames) — they are too stale
    // to ever be matched and just occupy ring slots.
    if (want_seq > kMaxSkewFrames) {
        const uint64_t oldest_useful = want_seq - kMaxSkewFrames;
        for (auto& e : m_ring) {
            if (e.msg && e.frame_seq < oldest_useful) {
                e.msg.reset();
                e.frame_seq = 0;
            }
        }
    }

    // Exact match: consume entry so the same bbox is not drawn on future frames.
    for (auto& e : m_ring) {
        if (e.msg && e.frame_seq == want_seq) {
            auto result = std::move(e.msg); // consume
            e.frame_seq = 0;
            ++m_exact_count;
            DS_TRACE("[BBOX-SYNC] exact seq=%" PRIu64 "\n", want_seq);
            return result;
        }
    }

    // Fallback: closest entry within kMaxSkewFrames.
    // Prefer the closest (by absolute delta); among ties prefer the larger seq.
    std::shared_ptr<DsMsgBbox> best_msg;
    uint64_t best_seq   = 0;
    uint64_t best_delta = kMaxSkewFrames + 1; // exclusive upper bound
    for (auto& e : m_ring) {
        if (!e.msg) continue;
        const uint64_t delta = (e.frame_seq > want_seq)
                               ? e.frame_seq - want_seq
                               : want_seq - e.frame_seq;
        if (delta < best_delta || (delta == best_delta && e.frame_seq > best_seq)) {
            best_delta = delta;
            best_seq   = e.frame_seq;
            best_msg   = e.msg;
        }
    }

    if (best_msg) {
        ++m_fallback_count;
        m_skew_sum += best_delta;
        DS_TRACE("[BBOX-SYNC] fallback: want=%" PRIu64 " got=%" PRIu64
                 " delta=%" PRIu64 "\n", want_seq, best_seq, best_delta);
    } else {
        ++m_no_bbox_count;
        DS_TRACE("[BBOX-SYNC] no bbox for seq=%" PRIu64 "\n", want_seq);
    }

    return best_msg; // nullptr → caller skips bbox overlay for this frame
}

// ---------------------------------------------------------------------------
void BboxReceiver::emit_sync_stats_if_due(uint64_t now_mono_us) {
    if (m_cfg.debug.stats_interval_s == 0) return;
    const uint64_t interval_us =
        static_cast<uint64_t>(m_cfg.debug.stats_interval_s) * 1'000'000ULL;
    if (now_mono_us - m_last_print_us < interval_us) return;
    m_last_print_us = now_mono_us;

    const uint64_t total = m_exact_count + m_fallback_count + m_no_bbox_count;
    if (total > 0) {
        DS_INFO("[BBOX-SYNC]"
                "  exact=%" PRIu64 " (%.1f%%)"
                "  fallback=%" PRIu64 " (%.1f%%)"
                "  no_bbox=%" PRIu64 " (%.1f%%)"
                "  avg_skew=%.2f frames\n",
                m_exact_count,
                100.0 * m_exact_count    / static_cast<double>(total),
                m_fallback_count,
                100.0 * m_fallback_count / static_cast<double>(total),
                m_no_bbox_count,
                100.0 * m_no_bbox_count  / static_cast<double>(total),
                m_fallback_count > 0
                    ? static_cast<double>(m_skew_sum) /
                      static_cast<double>(m_fallback_count)
                    : 0.0);
        m_exact_count    = 0;
        m_fallback_count = 0;
        m_no_bbox_count  = 0;
        m_skew_sum       = 0;
    }
}
