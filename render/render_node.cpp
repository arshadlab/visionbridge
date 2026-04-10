/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#include "render_node.h"
#include "display_output_sdl.h"
#include "display_output_egl.h"
#include "display_output_wl_egl.h"
#include "../common/logger.h"
#include "../common/time_utils.h"

#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

// ---------------------------------------------------------------------------
std::unique_ptr<IDisplayOutput> RenderNode::make_backend(const DsRenderConfig& cfg) {
    if (cfg.backend == "egl") {
        DS_INFO("RenderNode: using EGL/KMS backend\n");
        return std::make_unique<DisplayOutputEgl>(cfg);
    }
    if (cfg.backend == "wayland") {
        DS_INFO("RenderNode: using Wayland EGL backend\n");
        return std::make_unique<DisplayOutputWlEgl>(cfg);
    }
    DS_INFO("RenderNode: using SDL backend\n");
    return std::make_unique<DisplayOutputSdl>(cfg);
}

// ---------------------------------------------------------------------------
RenderNode::RenderNode(const DsRenderConfig& cfg)
    : m_cfg(cfg), m_decoder(cfg), m_display(make_backend(cfg)) {}

RenderNode::~RenderNode() { stop(); }

// ---------------------------------------------------------------------------
bool RenderNode::init() {
    if (!m_display->init()) {
        DS_ERR("RenderNode: display backend init failed\n");
        return false;
    }

    // Register frame callback: decoder thread → display (no data copy)
    m_decoder.set_frame_callback([this](std::shared_ptr<DecodedFrame> f) {
        m_display->update_frame(std::move(f));
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

    DS_INFO("RenderNode: entering render loop  (press Esc or Ctrl+C to quit)\n");

    // Put stdin in raw non-canonical mode so ESC is delivered immediately
    // without waiting for Enter. Restore on exit.
    struct termios orig_term{};
    bool term_raw = false;
    if (isatty(STDIN_FILENO)) {
        if (tcgetattr(STDIN_FILENO, &orig_term) == 0) {
            struct termios raw = orig_term;
            raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
            raw.c_cc[VMIN]  = 0; // non-blocking read
            raw.c_cc[VTIME] = 0;
            tcsetattr(STDIN_FILENO, TCSANOW, &raw);
            term_raw = true;
        }
    }

    // Polling render loop: present() checks for a new frame (pointer compare) and
    // renders it if one has arrived since the last flip. If no new frame, it returns
    // immediately and we sleep 100 µs before polling again. This keeps latency low
    // while the CPU stays near-idle between decoded frames.
    while (!stop_flag.load()) {
        // Non-blocking stdin check for ESC key (works for all backends).
        {
            fd_set fds; FD_ZERO(&fds); FD_SET(STDIN_FILENO, &fds);
            struct timeval tv{0, 0};
            if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0) {
                char c = 0;
                if (read(STDIN_FILENO, &c, 1) == 1 && c == 0x1B) {
                    DS_INFO("RenderNode: ESC pressed — quitting\n");
                    stop_flag.store(true);
                    break;
                }
            }
        }

        if (!m_display->present()) {
            DS_INFO("RenderNode: display quit requested\n");
            stop_flag.store(true);
            break;
        }
        ds_usleep(100); // 100 µs poll interval
    }

    if (term_raw)
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_term);

    DS_INFO("RenderNode: render loop exited\n");
}

// ---------------------------------------------------------------------------
void RenderNode::stop() {
    if (!m_initialized) return;
    m_decoder.stop();
    m_display->shutdown();
    m_initialized = false;
}
