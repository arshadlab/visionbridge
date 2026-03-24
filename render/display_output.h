/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 *
 * DisplayOutput — SDL2 window + ZMQ PULL thread for bbox overlays.
 *
 *  ┌──────────────────────────────────────────────────┐
 *  │  StreamDecoder ──▶  update_frame()               │
 *  │  ZMQ PULL thread ──▶ m_bbox_ring (by frame_seq)  │
 *  │  caller calls present() from its render loop     │
 *  └──────────────────────────────────────────────────┘
 */
#pragma once

#include "../common/config.h"
#include "../common/protocol.h"
#include "../render/stream_decoder.h"

#include <SDL2/SDL.h>
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <zmq.h>

class DisplayOutput {
public:
    explicit DisplayOutput(const DsRenderConfig& cfg);
    ~DisplayOutput();

    /** Initialize SDL2 window + ZMQ PULL socket. */
    bool init();

    /** Called from StreamDecoder callback (any thread). */
    void update_frame(std::shared_ptr<DecodedFrame> frame);

    /**
     * Call from render loop on the main thread.
     * Returns false to signal quit (e.g. SDL_QUIT event).
     */
    bool present();

    /** Cleanly shut down ZMQ thread and SDL. */
    void shutdown();

private:
    // SDL2 rendering
    SDL_Window*   m_window   = nullptr;
    SDL_Renderer* m_renderer = nullptr;
    SDL_Texture*  m_texture  = nullptr;

    uint32_t m_tex_width  = 0;
    uint32_t m_tex_height = 0;
    bool     m_window_sized = false; ///< false until window resized to first frame

    // Latest decoded frame (written from decoder thread, read on main thread)
    std::shared_ptr<DecodedFrame> m_pending_frame;
    std::mutex m_frame_mtx;

    // Bbox ring buffer — keyed by frame_seq for per-frame correlation.
    // zmq_recv_loop() writes here; present() looks up by sei_frame_seq with
    // fallback to the most-recently-received entry when SEI is unavailable.
    static constexpr int kBboxRingSize = 8;
    struct BboxRingEntry {
        uint64_t                   frame_seq = 0;
        std::shared_ptr<DsMsgBbox> msg;
    };
    BboxRingEntry m_bbox_ring[kBboxRingSize]{};
    size_t        m_bbox_ring_head = 0;
    std::mutex    m_bbox_mtx;

    // ZMQ
    void*       m_zmq_ctx    = nullptr;
    void*       m_zmq_socket = nullptr;
    std::thread m_zmq_thread;
    std::atomic<bool> m_running{false};

    void zmq_recv_loop();

    void draw_bboxes(const DsMsgBbox& msg, float sx, float sy);

    // Frame rate throttle: cap present() to display_fps to prevent file over-speed.
    // Touched only on the main thread — no mutex needed.
    uint64_t m_last_present_us   = 0;

    // End-to-end latency stats — ZMQ path (capture_ts_us → SDL_RenderPresent).
    // All fields touched only on the main thread inside present() — no mutex needed.
    uint64_t m_lat_sum_us        = 0;
    uint64_t m_lat_count         = 0;
    uint64_t m_lat_min_us        = UINT64_MAX;
    uint64_t m_lat_max_us        = 0;
    uint64_t m_last_lat_print_us = 0;

    // End-to-end latency stats — SEI in-band path (sei_capture_ts_us → SDL_RenderPresent).
    uint64_t m_sei_lat_sum_us    = 0;
    uint64_t m_sei_lat_count     = 0;
    uint64_t m_sei_lat_min_us    = UINT64_MAX;
    uint64_t m_sei_lat_max_us    = 0;
    uint64_t m_sei_last_print_us = 0;

    const DsRenderConfig& m_cfg;
};
