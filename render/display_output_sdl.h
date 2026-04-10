/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 *
 * DisplayOutputSdl — SDL2 window backend.
 */
#pragma once

#include "display_output_base.h"
#include "../common/pipeline_stats.h"

#include <SDL2/SDL.h>

class DisplayOutputSdl final : public IDisplayOutput {
public:
    explicit DisplayOutputSdl(const DsRenderConfig& cfg);
    ~DisplayOutputSdl() override { shutdown(); }

    bool init()     override;
    void update_frame(std::shared_ptr<DecodedFrame> frame) override;
    bool present()  override;
    void shutdown() override;

private:
    // SDL2 rendering
    SDL_Window*   m_window   = nullptr;
    SDL_Renderer* m_renderer = nullptr;
    SDL_Texture*  m_texture  = nullptr;

    uint32_t m_tex_width  = 0;
    uint32_t m_tex_height = 0;
    bool     m_window_sized = false;

    // Latest decoded frame — decoder thread writes, render thread reads.
    // Always overwritten with newest frame (latest-wins, no queuing).
    std::shared_ptr<DecodedFrame> m_pending_frame;
    std::mutex                    m_frame_mtx;
    // Raw pointer of last frame handed to the GPU — used to detect truly new frames.
    const DecodedFrame*           m_last_frame_ptr = nullptr;

    // Bbox support
    BboxReceiver m_bbox;

    // E2E latency stats — ZMQ path (capture_ts → SDL_RenderPresent)
    LatencyStats m_e2e_stats;
    bool         m_clock_skew_warned = false;

    // E2E latency stats — SEI in-band path
    LatencyStats m_sei_stats;
    bool         m_sei_clock_skew_warned = false;

    // Render-side timing stats (main thread only)
    LatencyStats m_upload_stats;     ///< SDL_UpdateTexture duration
    LatencyStats m_present_stats;    ///< SDL_RenderPresent duration (vsync wait)

    // Unified stats print timestamp
    uint64_t     m_last_stats_print_us = 0;
    uint64_t     m_last_displayed_seq  = 0; ///< sei_frame_seq of last presented frame

    const DsRenderConfig& m_cfg;

    void draw_bboxes(const DsMsgBbox& msg, float sx, float sy);
    void emit_stats_if_due(uint64_t now_mono_us, uint64_t now_real_us,
                           const std::shared_ptr<DsMsgBbox>& bbox_msg,
                           const std::shared_ptr<DecodedFrame>& frame);
};
