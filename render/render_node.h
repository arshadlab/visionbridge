/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 *
 * RenderNode — ties together StreamDecoder and DisplayOutput.
 */
#pragma once

#include "../common/config.h"
#include "stream_decoder.h"
#include "display_output_base.h"

#include <atomic>
#include <memory>

class RenderNode {
public:
    explicit RenderNode(const DsRenderConfig& cfg);
    ~RenderNode();

    /** Initialize decoder and display subsystems. */
    bool init();

    /**
     * Blocking render loop. Returns when user closes window
     * or a signal sets stop_flag.
     */
    void run(std::atomic<bool>& stop_flag);

    void stop();

private:
    const DsRenderConfig& m_cfg;

    StreamDecoder              m_decoder;
    std::unique_ptr<IDisplayOutput> m_display;

    bool m_initialized = false;

    static std::unique_ptr<IDisplayOutput> make_backend(const DsRenderConfig& cfg);
};
