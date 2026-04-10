/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#include "display_output_sdl.h"
#include "../common/logger.h"
#include "../common/time_utils.h"

#include <cstring>
#include <cinttypes>
#include <cstdio>
#include <string>

// Bbox border colors per detector (RGBA)
static constexpr SDL_Color kColorStub = {180, 180, 180, 255};
static constexpr SDL_Color kColorMog2 = { 50, 220,  50, 255};
static constexpr SDL_Color kColorYolo = {220,  50,  50, 255};
static constexpr SDL_Color kColorUnk  = {255, 255,   0, 255};

static const SDL_Color& color_for_id(uint8_t id) {
    switch (id) {
    case DS_DET_STUB: return kColorStub;
    case DS_DET_MOG2: return kColorMog2;
    case DS_DET_YOLO: return kColorYolo;
    default:          return kColorUnk;
    }
}

// ---------------------------------------------------------------------------
DisplayOutputSdl::DisplayOutputSdl(const DsRenderConfig& cfg)
    : m_bbox(cfg), m_cfg(cfg) {}

// ---------------------------------------------------------------------------
bool DisplayOutputSdl::init() {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        DS_ERR("SDL_Init failed: %s\n", SDL_GetError());
        return false;
    }

    const int w = (m_cfg.display_width  > 0) ? static_cast<int>(m_cfg.display_width)  : 1;
    const int h = (m_cfg.display_height > 0) ? static_cast<int>(m_cfg.display_height) : 1;

    Uint32 flags = SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE;
    if (m_cfg.fullscreen) flags |= SDL_WINDOW_FULLSCREEN_DESKTOP;

    m_window = SDL_CreateWindow(m_cfg.display_title.c_str(),
                                SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                w, h, flags);
    if (!m_window) {
        DS_ERR("SDL_CreateWindow failed: %s\n", SDL_GetError());
        return false;
    }

    m_renderer = SDL_CreateRenderer(m_window, -1,
                                    SDL_RENDERER_ACCELERATED);
    if (!m_renderer)
        m_renderer = SDL_CreateRenderer(m_window, -1, SDL_RENDERER_SOFTWARE);
    if (!m_renderer) {
        DS_ERR("SDL_CreateRenderer failed: %s\n", SDL_GetError());
        return false;
    }

    if (m_cfg.display_width > 0 && m_cfg.display_height > 0) {
        SDL_RenderSetLogicalSize(m_renderer, w, h);
        m_window_sized = true;
    }

    if (!m_bbox.start()) return false;

    DS_INFO("DisplayOutputSdl: initialized (%dx%d)\n", w, h);
    return true;
}

// ---------------------------------------------------------------------------
void DisplayOutputSdl::update_frame(std::shared_ptr<DecodedFrame> frame) {
    // Latest-wins: always overwrite; render loop detects change via pointer compare.
    std::lock_guard<std::mutex> lk(m_frame_mtx);
    m_pending_frame = std::move(frame);
}

// ---------------------------------------------------------------------------
void DisplayOutputSdl::shutdown() {
    m_bbox.stop();
    if (m_texture)  { SDL_DestroyTexture(m_texture);   m_texture  = nullptr; }
    if (m_renderer) { SDL_DestroyRenderer(m_renderer); m_renderer = nullptr; }
    if (m_window)   { SDL_DestroyWindow(m_window);     m_window   = nullptr; }
    SDL_Quit();
}

// ---------------------------------------------------------------------------
void DisplayOutputSdl::draw_bboxes(const DsMsgBbox& msg, float sx, float sy) {
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
        SDL_RenderDrawRect(m_renderer, &rect);
        if (rect.w > 2 && rect.h > 2) {
            SDL_Rect inner = {rect.x + 1, rect.y + 1, rect.w - 2, rect.h - 2};
            SDL_RenderDrawRect(m_renderer, &inner);
        }
    }
}

// ---------------------------------------------------------------------------
void DisplayOutputSdl::emit_stats_if_due(
        uint64_t now_mono_us, uint64_t now_real_us,
        const std::shared_ptr<DsMsgBbox>& bbox_msg,
        const std::shared_ptr<DecodedFrame>& frame) {

    const uint64_t interval_us =
        static_cast<uint64_t>(m_cfg.debug.stats_interval_s) * 1'000'000ULL;
    if (m_cfg.debug.stats_interval_s == 0 || interval_us == 0) return;

    // Accumulate ZMQ-path E2E latency
    if (bbox_msg && bbox_msg->capture_ts_us != 0) {
        if (bbox_msg->capture_ts_us > now_real_us) {
            if (!m_clock_skew_warned) {
                DS_WARN("[E2E] Source clock ahead by ~%" PRIu64 " ms — sync NTP/PTP\n",
                        (bbox_msg->capture_ts_us - now_real_us) / 1000ULL);
                m_clock_skew_warned = true;
            }
        } else {
            const uint64_t lat = now_real_us - bbox_msg->capture_ts_us;
            if (lat < 10'000'000ULL) m_e2e_stats.record(lat);
        }
    }

    // Accumulate SEI in-band E2E latency
    if (frame && frame->sei_capture_ts_us != 0) {
        if (frame->sei_capture_ts_us > now_real_us) {
            if (!m_sei_clock_skew_warned) {
                DS_WARN("[E2E-SEI] Source clock ahead by ~%" PRIu64 " ms — sync NTP/PTP\n",
                        (frame->sei_capture_ts_us - now_real_us) / 1000ULL);
                m_sei_clock_skew_warned = true;
            }
        } else {
            const uint64_t lat = now_real_us - frame->sei_capture_ts_us;
            if (lat < 10'000'000ULL) m_sei_stats.record(lat);
        }
    }

    // Single unified print once per interval
    if (now_mono_us - m_last_stats_print_us < interval_us) return;
    m_last_stats_print_us = now_mono_us;

    bool have_any = false;
    char line[256];
    std::string msg = "[TIMING-RENDER SDL]\n";

    auto append_ms = [&](const char* tag, LatencyStats& s) {
        if (s.empty()) return;
        auto snap = s.reset_and_return();
        snprintf(line, sizeof(line),
                 "  %-22s  avg=%6.1f  std=%5.1f  min=%6" PRIu64 "  max=%6" PRIu64
                 "  ms  (n=%" PRIu64 ")\n",
                 tag, snap.avg_us() / 1000.0, snap.stddev_us() / 1000.0,
                 snap.min_us / 1000, snap.max_us / 1000, snap.count);
        msg += line;
        have_any = true;
    };
    auto append_us = [&](const char* tag, LatencyStats& s) {
        if (s.empty()) return;
        auto snap = s.reset_and_return();
        snprintf(line, sizeof(line),
                 "  %-22s  avg=%6.0f  std=%5.0f  min=%6" PRIu64 "  max=%6" PRIu64
                 "  us  (n=%" PRIu64 ")\n",
                 tag, snap.avg_us(), snap.stddev_us(),
                 snap.min_us, snap.max_us, snap.count);
        msg += line;
        have_any = true;
    };

    append_ms("e2e capture→display",  m_e2e_stats);
    append_ms("e2e-sei capture→disp", m_sei_stats);
    append_us("upload",               m_upload_stats);
    append_us("present",              m_present_stats);

    snprintf(line, sizeof(line), "  last_seq=%" PRIu64 "\n", m_last_displayed_seq);
    msg += line;

    DS_INFO("%s", msg.c_str());
}

// ---------------------------------------------------------------------------
bool DisplayOutputSdl::present() {
    // Poll SDL events (non-blocking).
    SDL_Event ev;
    while (SDL_PollEvent(&ev)) {
        if (ev.type == SDL_QUIT) return false;
        if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_ESCAPE) return false;
    }

    // Grab latest frame under lock (no copy — shared_ptr ref-count bump only).
    std::shared_ptr<DecodedFrame> frame;
    {
        std::lock_guard<std::mutex> lk(m_frame_mtx);
        frame = m_pending_frame;
    }

    // Only proceed if a genuinely new frame arrived since last render.
    if (!frame || frame.get() == m_last_frame_ptr)
        return true;

    std::shared_ptr<DsMsgBbox> bbox_msg = m_bbox.best_for_frame(frame.get());

    if (frame) {
        if (!m_window_sized) {
            SDL_SetWindowSize(m_window,
                              static_cast<int>(frame->width),
                              static_cast<int>(frame->height));
            SDL_RenderSetLogicalSize(m_renderer,
                                    static_cast<int>(frame->width),
                                    static_cast<int>(frame->height));
            SDL_SetWindowPosition(m_window, SDL_WINDOWPOS_CENTERED,
                                  SDL_WINDOWPOS_CENTERED);
            DS_INFO("DisplayOutputSdl: window sized to %ux%u\n",
                    frame->width, frame->height);
            m_window_sized = true;
        }

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

        if (m_texture) {
            const uint64_t t0 = ds_mono_us();
            // Single GPU upload path: DecodedFrame data → SDL streaming texture.
            // No intermediate CPU copy — frame->data was populated once in the
            // decoder callback (one memcpy from the GStreamer buffer).
            SDL_UpdateTexture(m_texture, nullptr,
                              frame->data.data(),
                              static_cast<int>(frame->stride));
            m_upload_stats.record(ds_mono_us() - t0);
        }
    }

    SDL_RenderClear(m_renderer);
    if (m_texture)
        SDL_RenderCopy(m_renderer, m_texture, nullptr, nullptr);

    if (bbox_msg && frame) {
        int lw = 0, lh = 0;
        SDL_RenderGetLogicalSize(m_renderer, &lw, &lh);
        // Scale from source appsink coords → render logical coords.
        // Use src_width/src_height from the message when available; fall back to
        // decoded frame size (same stream → same resolution in the normal case).
        const float src_w = (bbox_msg->src_width  > 0)
                            ? static_cast<float>(bbox_msg->src_width)
                            : static_cast<float>(frame->width);
        const float src_h = (bbox_msg->src_height > 0)
                            ? static_cast<float>(bbox_msg->src_height)
                            : static_cast<float>(frame->height);
        const float sx = static_cast<float>(lw) / src_w;
        const float sy = static_cast<float>(lh) / src_h;
        draw_bboxes(*bbox_msg, sx, sy);
    }

    const uint64_t present_t0 = ds_mono_us();
    SDL_RenderPresent(m_renderer);
    m_present_stats.record(ds_mono_us() - present_t0);

    DS_TRACE("DisplayOutputSdl: display seq=%" PRIu64 " bbox_seq=%" PRIu64 "\n",
             frame ? frame->sei_frame_seq : 0ULL,
             bbox_msg ? bbox_msg->frame_seq : 0ULL);
    m_last_frame_ptr = frame.get(); // mark as rendered
    if (frame && frame->sei_frame_seq != 0)
        m_last_displayed_seq = frame->sei_frame_seq;

    const uint64_t now_mono = ds_mono_us();
    const uint64_t now_real = ds_realtime_us();

    emit_stats_if_due(now_mono, now_real, bbox_msg, frame);
    m_bbox.emit_sync_stats_if_due(now_mono);

    return true;
}
