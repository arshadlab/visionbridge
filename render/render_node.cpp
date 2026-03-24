/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#include "render_node.h"
#include "../common/logger.h"
#include "../common/time_utils.h"

// ---------------------------------------------------------------------------
RenderNode::RenderNode(const DsRenderConfig& cfg)
    : m_cfg(cfg), m_decoder(cfg), m_display(cfg) {}

RenderNode::~RenderNode() { stop(); }

// ---------------------------------------------------------------------------
bool RenderNode::init() {
    if (!m_display.init()) {
        DS_ERR("RenderNode: DisplayOutput init failed\n");
        return false;
    }

    // Register frame callback: decoder thread → display
    m_decoder.set_frame_callback([this](std::shared_ptr<DecodedFrame> f) {
        m_display.update_frame(std::move(f));
    });

    if (!m_decoder.start()) {
        DS_ERR("RenderNode: StreamDecoder start failed\n");
        return false;
    }

    m_initialized = true;
    return true;
}

// ---------------------------------------------------------------------------
void RenderNode::run(std::atomic<bool>& stop_flag) {
    if (!m_initialized) {
        DS_ERR("RenderNode: not initialized\n");
        return;
    }

    DS_INFO("RenderNode: entering render loop\n");

    // Simple busy-ish loop: present at ~120Hz check rate (≈8ms per iter).
    // DisplayOutput::present() will block briefly anyway on SDL_RenderPresent
    // when vsync is enabled or on SDL_PollEvent.
    constexpr uint32_t kSleepMs = 4;

    while (!stop_flag.load()) {
        if (!m_display.present()) {
            DS_INFO("RenderNode: display quit requested\n");
            stop_flag.store(true);
            break;
        }
        ds_sleep_ms(kSleepMs);
    }

    DS_INFO("RenderNode: render loop exited\n");
}

// ---------------------------------------------------------------------------
void RenderNode::stop() {
    if (!m_initialized) return;
    m_decoder.stop();
    m_display.shutdown();
    m_initialized = false;
}
