/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 *
 * IDisplayOutput — abstract interface implemented by every render backend.
 * BboxReceiver   — shared ZMQ PULL thread that all backends use for bbox overlay.
 */
#pragma once

#include "../common/config.h"
#include "../common/protocol.h"
#include "../render/stream_decoder.h"

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <zmq.h>

// ---------------------------------------------------------------------------
// IDisplayOutput
// ---------------------------------------------------------------------------
class IDisplayOutput {
public:
    virtual ~IDisplayOutput() = default;

    /** Initialize the display backend (window / DRM device / Wayland connection). */
    virtual bool init() = 0;

    /**
     * Called from the StreamDecoder callback (any thread).
     * The backend stores the shared_ptr and uses its data on the next present().
     * No data is copied — the DecodedFrame is reference-counted until released.
     */
    virtual void update_frame(std::shared_ptr<DecodedFrame> frame) = 0;

    /**
     * Render one frame and present it to the display.
     * Called from the render loop on whatever thread owns the backend.
     * Returns false to signal quit (e.g. Escape key / window close / SIGINT).
     */
    virtual bool present() = 0;

    /** Cleanly tear down the backend and release all resources. */
    virtual void shutdown() = 0;
};

// ---------------------------------------------------------------------------
// BboxReceiver
//
// Shared ZMQ PULL recv thread.  All three backends include this as a member
// to receive DsMsgBbox messages and keep them in a ring buffer keyed by
// frame_seq for per-frame correlation.
// ---------------------------------------------------------------------------
class BboxReceiver {
public:
    static constexpr int      kRingSize      = 16; ///< bbox ring depth (increased)
    static constexpr uint64_t kMaxSkewFrames = 4;  ///< max seq delta for fallback match

    struct Entry {
        uint64_t                   frame_seq = 0;
        std::shared_ptr<DsMsgBbox> msg;
    };

    explicit BboxReceiver(const DsRenderConfig& cfg);
    ~BboxReceiver() { stop(); }

    /** Open ZMQ socket and start recv thread. */
    bool start();

    /** Signal stop and join recv thread. */
    void stop();

    /**
     * Retrieve the best-matching bbox message for a given video frame.
     * Uses sei_frame_seq for exact correlation when available; falls back to
     * the most-recently-received entry otherwise.
     * Also updates exact/fallback counters.
     */
    std::shared_ptr<DsMsgBbox> best_for_frame(const DecodedFrame* frame);

    /** Print bbox-frame sync quality stats if the interval has elapsed. */
    void emit_sync_stats_if_due(uint64_t now_mono_us);

private:
    void recv_loop();

    const DsRenderConfig& m_cfg;

    void*       m_zmq_ctx    = nullptr;
    void*       m_zmq_socket = nullptr;
    std::thread m_thread;
    std::atomic<bool> m_running{false};

    Entry     m_ring[kRingSize]{};
    size_t    m_ring_head = 0;
    std::mutex m_mtx;

    // Clock-skew one-time warning
    bool m_clock_skew_warned = false;

    // Sync quality counters (main thread only — no mutex needed)
    uint64_t m_exact_count    = 0; ///< frames matched by exact seq
    uint64_t m_fallback_count = 0; ///< frames matched within kMaxSkewFrames tolerance
    uint64_t m_no_bbox_count  = 0; ///< frames with SEI but no bbox within tolerance
    uint64_t m_skew_sum       = 0;
    uint64_t m_last_print_us  = 0;
};
