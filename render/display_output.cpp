/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#include "display_output.h"
#include "../common/logger.h"
#include "../common/protocol.h"
#include "../common/time_utils.h"

#include <cstring>
#include <cinttypes>

// Bbox border colors per detector (RGBA components passed to SDL)
static constexpr SDL_Color kColorStub  = {180, 180, 180, 255}; // gray
static constexpr SDL_Color kColorMog2  = { 50, 220,  50, 255}; // green
static constexpr SDL_Color kColorYolo  = {220,  50,  50, 255}; // red
static constexpr SDL_Color kColorUnk   = {255, 255,   0, 255}; // yellow fallback

static const SDL_Color& color_for_id(uint8_t id) {
    switch (id) {
    case DS_DET_STUB: return kColorStub;
    case DS_DET_MOG2: return kColorMog2;
    case DS_DET_YOLO: return kColorYolo;
    default:          return kColorUnk;
    }
}

// ---------------------------------------------------------------------------
DisplayOutput::DisplayOutput(const DsRenderConfig& cfg) : m_cfg(cfg) {}

DisplayOutput::~DisplayOutput() { shutdown(); }

// ---------------------------------------------------------------------------
bool DisplayOutput::init() {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        DS_ERR("SDL_Init failed: %s\n", SDL_GetError());
        return false;
    }

    const int w = static_cast<int>(m_cfg.display_width);
    const int h = static_cast<int>(m_cfg.display_height);

    m_window = SDL_CreateWindow(m_cfg.display_title.c_str(),
                                SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                w, h,
                                SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
    if (!m_window) {
        DS_ERR("SDL_CreateWindow failed: %s\n", SDL_GetError());
        return false;
    }

    m_renderer = SDL_CreateRenderer(m_window, -1,
                                    SDL_RENDERER_ACCELERATED |
                                    SDL_RENDERER_PRESENTVSYNC);
    if (!m_renderer) {
        // fallback to software renderer
        m_renderer = SDL_CreateRenderer(m_window, -1, SDL_RENDERER_SOFTWARE);
    }
    if (!m_renderer) {
        DS_ERR("SDL_CreateRenderer failed: %s\n", SDL_GetError());
        return false;
    }
    SDL_RenderSetLogicalSize(m_renderer, w, h);

    // ZMQ PULL socket
    m_zmq_ctx = zmq_ctx_new();
    m_zmq_socket = zmq_socket(m_zmq_ctx, ZMQ_PULL);
    if (!m_zmq_socket) {
        DS_ERR("zmq_socket failed\n");
        return false;
    }

    int hwm = 1;
    zmq_setsockopt(m_zmq_socket, ZMQ_RCVHWM, &hwm, sizeof(hwm));
    int rcvtimeo = 100; // ms — allows graceful shutdown check
    zmq_setsockopt(m_zmq_socket, ZMQ_RCVTIMEO, &rcvtimeo, sizeof(rcvtimeo));

    const std::string ep = m_cfg.zmq.render_endpoint;
    if (zmq_connect(m_zmq_socket, ep.c_str()) != 0) {
        DS_ERR("ZMQ connect to %s failed: %s\n", ep.c_str(), zmq_strerror(zmq_errno()));
        return false;
    }
    DS_INFO("DisplayOutput: ZMQ PULL connected to %s\n", ep.c_str());

    m_running.store(true);
    m_zmq_thread = std::thread(&DisplayOutput::zmq_recv_loop, this);

    return true;
}

// ---------------------------------------------------------------------------
void DisplayOutput::update_frame(std::shared_ptr<DecodedFrame> frame) {
    std::lock_guard<std::mutex> lk(m_frame_mtx);
    m_pending_frame = std::move(frame);
}

// ---------------------------------------------------------------------------
void DisplayOutput::zmq_recv_loop() {
    DS_DBG("ZMQ recv thread started\n");
    while (m_running.load()) {
        DsMsgBbox msg{};
        const int nbytes = zmq_recv(m_zmq_socket, &msg, sizeof(msg), 0);
        if (nbytes < 0) {
            if (zmq_errno() == EAGAIN) continue; // timeout — loop to check m_running
            if (zmq_errno() == EINTR)  continue;
            DS_WARN("ZMQ recv error: %s\n", zmq_strerror(zmq_errno()));
            continue;
        }
        if (static_cast<size_t>(nbytes) < sizeof(DsMsgBbox)) {
            DS_WARN("ZMQ: short message %d bytes\n", nbytes);
            continue;
        }
        if (msg.type != DS_MSG_BBOX) continue;

        auto shared = std::make_shared<DsMsgBbox>(msg);
        std::lock_guard<std::mutex> lk(m_bbox_mtx);
        m_latest_bbox = std::move(shared);
    }
    DS_DBG("ZMQ recv thread exiting\n");
}

// ---------------------------------------------------------------------------
// draw_bboxes — sx/sy are scale factors (display / stream dimensions)
// ---------------------------------------------------------------------------
void DisplayOutput::draw_bboxes(const DsMsgBbox& msg, float sx, float sy) {
    const uint32_t count = msg.bbox_count < DS_MAX_BBOXES ? msg.bbox_count : DS_MAX_BBOXES;
    for (uint32_t i = 0; i < count; ++i) {
        const DsBbox& b = msg.bboxes[i];
        const SDL_Color& c = color_for_id(b.detector_id);
        SDL_SetRenderDrawColor(m_renderer, c.r, c.g, c.b, c.a);

        SDL_Rect rect;
        rect.x = static_cast<int>(b.x * sx);
        rect.y = static_cast<int>(b.y * sy);
        rect.w = static_cast<int>(b.w * sx);
        rect.h = static_cast<int>(b.h * sy);

        // Draw a 2-pixel border by nesting two rects
        SDL_RenderDrawRect(m_renderer, &rect);
        if (rect.w > 2 && rect.h > 2) {
            SDL_Rect inner = {rect.x + 1, rect.y + 1, rect.w - 2, rect.h - 2};
            SDL_RenderDrawRect(m_renderer, &inner);
        }
    }
}

// ---------------------------------------------------------------------------
// present — must be called from the main/SDL thread
// ---------------------------------------------------------------------------
bool DisplayOutput::present() {
    // Handle SDL events
    SDL_Event ev;
    while (SDL_PollEvent(&ev)) {
        if (ev.type == SDL_QUIT) return false;
        if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_ESCAPE) return false;
    }

    // Grab latest frame and bbox for this render cycle (under their respective locks)
    std::shared_ptr<DecodedFrame> frame;
    {
        std::lock_guard<std::mutex> lk(m_frame_mtx);
        frame = m_pending_frame;
    }

    std::shared_ptr<DsMsgBbox> bbox_msg;
    {
        std::lock_guard<std::mutex> lk(m_bbox_mtx);
        bbox_msg = m_latest_bbox;
    }

    if (frame) {
        // Re-create texture if dimensions changed
        if (!m_texture || m_tex_width != frame->width || m_tex_height != frame->height) {
            if (m_texture) SDL_DestroyTexture(m_texture);
            m_texture = SDL_CreateTexture(m_renderer,
                                          SDL_PIXELFORMAT_RGBA32,
                                          SDL_TEXTUREACCESS_STREAMING,
                                          static_cast<int>(frame->width),
                                          static_cast<int>(frame->height));
            m_tex_width  = frame->width;
            m_tex_height = frame->height;
        }
        if (m_texture)
            SDL_UpdateTexture(m_texture, nullptr, frame->data.data(),
                              static_cast<int>(frame->stride));
    }

    SDL_RenderClear(m_renderer);

    if (m_texture)
        SDL_RenderCopy(m_renderer, m_texture, nullptr, nullptr);

    // Overlay bboxes
    if (bbox_msg && frame) {
        const float sx = static_cast<float>(m_cfg.display_width)
                       / static_cast<float>(frame->width);
        const float sy = static_cast<float>(m_cfg.display_height)
                       / static_cast<float>(frame->height);
        draw_bboxes(*bbox_msg, sx, sy);
    }

    SDL_RenderPresent(m_renderer);

    // ---- End-to-end latency accounting (capture → display) ----
    // capture_ts_us was stamped at the moment the frame entered the source appsink
    // callback.  We measure it here, right after SDL_RenderPresent(), which is the
    // earliest point the frame is committed to the display.
    if (bbox_msg && bbox_msg->capture_ts_us != 0) {
        const uint64_t now = ds_realtime_us();
        const uint64_t lat = now - bbox_msg->capture_ts_us;
        // Guard against implausible values (clock step, unsynced clocks > 10 s, etc.)
        if (lat < 10'000'000ULL) {
            m_lat_sum_us += lat;
            ++m_lat_count;
            if (lat < m_lat_min_us) m_lat_min_us = lat;
            if (lat > m_lat_max_us) m_lat_max_us = lat;
        }
        const uint32_t interval_s = m_cfg.debug.stats_interval_s;
        if (interval_s > 0 && m_lat_count > 0 &&
            now - m_last_lat_print_us >= static_cast<uint64_t>(interval_s) * 1'000'000ULL) {
            DS_INFO("[E2E latency capture→display]  avg=%.1f ms  min=%.1f ms  max=%.1f ms"
                    "  samples=%" PRIu64 "\n",
                    m_lat_sum_us / 1000.0 / static_cast<double>(m_lat_count),
                    m_lat_min_us / 1000.0,
                    m_lat_max_us / 1000.0,
                    m_lat_count);
            // Reset accumulators for the next interval
            m_lat_sum_us        = 0;
            m_lat_count         = 0;
            m_lat_min_us        = UINT64_MAX;
            m_lat_max_us        = 0;
            m_last_lat_print_us = now;
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
void DisplayOutput::shutdown() {
    if (!m_running.load()) return;
    m_running.store(false);
    if (m_zmq_thread.joinable()) m_zmq_thread.join();

    if (m_zmq_socket) { zmq_close(m_zmq_socket); m_zmq_socket = nullptr; }
    if (m_zmq_ctx)    { zmq_ctx_destroy(m_zmq_ctx); m_zmq_ctx = nullptr; }

    if (m_texture)  { SDL_DestroyTexture(m_texture);   m_texture  = nullptr; }
    if (m_renderer) { SDL_DestroyRenderer(m_renderer); m_renderer = nullptr; }
    if (m_window)   { SDL_DestroyWindow(m_window);     m_window   = nullptr; }
    SDL_Quit();
}
