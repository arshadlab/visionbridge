/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 *
 * visionbridge-render — Render node entry point.
 *
 * Transport modes (multicast is the default per render.json):
 *
 *   Multicast (default):
 *     visionbridge_render                                   # use render.json settings
 *     visionbridge_render --source 239.1.1.1                # auto-detected from IP range
 *     visionbridge_render --multicast 239.1.1.1             # explicit multicast group
 *
 *   Unicast:
 *     visionbridge_render --unicast 192.168.1.10            # explicit unicast source IP
 *     visionbridge_render --source  192.168.1.10            # auto-detected from IP range
 *
 * --source      : auto-detects mode (multicast if 224.x.x.x–239.x.x.x, else unicast).
 * --unicast     : explicit unicast — overrides host and disables multicast.
 * --multicast   : explicit multicast group — overrides host and enables multicast.
 * Both --unicast/--multicast also update the ZMQ PULL endpoint host.
 */

#include "../common/config.h"
#include "../common/config_loader.h"
#include "../common/logger.h"
#include "render_node.h"

#include <gst/gst.h>
#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sstream>

static std::atomic<bool> g_stop{false};

static void sig_handler(int) { g_stop.store(true); }

static void print_usage(const char* prog) {
    fprintf(stderr,
        "VisionBridge Render Node\n"
        "Usage: %s [OPTIONS]\n\n"
        "  --config <path>       Path to render JSON config [default: configs/render.json]\n"
        "  --source <host>       Override source host; auto-detects multicast vs unicast\n"
        "                        from IP range (224.x.x.x-239.x.x.x → multicast)\n"
        "  --unicast <host>      Set unicast source IP and disable multicast\n"
        "  --multicast <group>   Set multicast group and enable multicast\n"
        "  --log-level <0-4>     0=OFF 1=ERR 2=INFO 3=DBG 4=TRACE [default: 2]\n"
        "  --help                Print this message\n"
        "\n"
        "Transport mode (default: multicast 239.1.1.1:5004 per render.json):\n"
        "  Same machine:   %s                                # loopback, use render.json\n"
        "  Unicast LAN:    %s --unicast 192.168.1.10\n"
        "  Multicast LAN:  %s --multicast 239.1.1.1\n"
        "  Auto-detect:    %s --source 192.168.1.10         # unicast (non-multicast IP)\n"
        "  Auto-detect:    %s --source 239.1.1.1            # multicast\n"
        "\n"
        "Detector bbox overlay colours:\n"
        "  Stub → gray   MOG2 → green   YOLO → red\n",
        prog, prog, prog, prog, prog, prog);
}

// Returns true if 'host' is an IPv4 multicast address (224.0.0.0/4).
static bool is_multicast_addr(const std::string& host) {
    if (host.empty()) return false;
    // Fast check: multicast range is 224.x.x.x – 239.x.x.x
    unsigned first = 0;
    const char* p = host.c_str();
    while (*p >= '0' && *p <= '9') first = first * 10 + static_cast<unsigned>(*p++ - '0');
    return first >= 224 && first <= 239;
}

// Replace host in a ZMQ endpoint such as tcp://1.2.3.4:5560
static std::string replace_endpoint_host(const std::string& ep, const std::string& new_host) {
    // Find "://" then the next ':'
    const auto scheme_end = ep.find("://");
    if (scheme_end == std::string::npos) return ep;
    const auto host_start = scheme_end + 3;
    const auto colon_pos  = ep.find(':', host_start);
    if (colon_pos == std::string::npos) return ep;
    return ep.substr(0, host_start) + new_host + ep.substr(colon_pos);
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT,  sig_handler);
    std::signal(SIGTERM, sig_handler);

    std::string config_path  = "configs/render.json";
    std::string source_host;          // --source  (auto-detect mode)
    std::string unicast_host;         // --unicast (explicit unicast)
    std::string multicast_group;      // --multicast (explicit multicast)
    int log_level = -1;

    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--help"))      { print_usage(argv[0]); return 0; }
        else if (!strcmp(argv[i], "--config")    && i+1 < argc) config_path     = argv[++i];
        else if (!strcmp(argv[i], "--source")    && i+1 < argc) source_host     = argv[++i];
        else if (!strcmp(argv[i], "--unicast")   && i+1 < argc) unicast_host    = argv[++i];
        else if (!strcmp(argv[i], "--multicast") && i+1 < argc) multicast_group = argv[++i];
        else if (!strcmp(argv[i], "--log-level") && i+1 < argc) log_level       = atoi(argv[++i]);
        else { fprintf(stderr, "Unknown option: %s\n", argv[i]); print_usage(argv[0]); return 1; }
    }

    if (log_level >= 0) g_ds_log_level.store(log_level);

    // Load config
    DsRenderConfig cfg{};
    if (!ConfigLoader::load_render(config_path, cfg)) {
        DS_ERR("Failed to load render config: %s\n", config_path.c_str());
        return 1;
    }

    // Apply transport overrides (priority: --unicast > --multicast > --source auto-detect)
    if (!unicast_host.empty()) {
        cfg.transport.host      = unicast_host;
        cfg.transport.multicast = false;
        cfg.zmq.render_endpoint = replace_endpoint_host(cfg.zmq.render_endpoint, unicast_host);
        DS_INFO("Transport override: UNICAST host=%s, zmq=%s\n",
                cfg.transport.host.c_str(), cfg.zmq.render_endpoint.c_str());
    } else if (!multicast_group.empty()) {
        cfg.transport.host      = multicast_group;
        cfg.transport.multicast = true;
        cfg.zmq.render_endpoint = replace_endpoint_host(cfg.zmq.render_endpoint, multicast_group);
        DS_INFO("Transport override: MULTICAST group=%s, zmq=%s\n",
                cfg.transport.host.c_str(), cfg.zmq.render_endpoint.c_str());
    } else if (!source_host.empty()) {
        cfg.transport.host      = source_host;
        cfg.transport.multicast = is_multicast_addr(source_host);
        cfg.zmq.render_endpoint = replace_endpoint_host(cfg.zmq.render_endpoint, source_host);
        DS_INFO("Source override: %s host=%s, zmq=%s\n",
                cfg.transport.multicast ? "MULTICAST" : "UNICAST",
                cfg.transport.host.c_str(), cfg.zmq.render_endpoint.c_str());
    }

    // Override log level from config if CLI didn't set it
    if (log_level < 0) g_ds_log_level.store(cfg.debug.log_level);

    DS_INFO("visionbridge-render starting\n");
    DS_INFO("  transport: %s %s:%u\n",
            cfg.transport.multicast ? "[MULTICAST]" : "[UNICAST]",
            cfg.transport.host.c_str(), cfg.transport.rtp_port);
    DS_INFO("  zmq pull:  %s\n", cfg.zmq.render_endpoint.c_str());
    DS_INFO("  jitter:    %u ms\n", cfg.jitter_buffer_ms);
    DS_INFO("  hw_decode: %s\n", cfg.codec.hw_decode ? "yes" : "no");

    // Initialize GStreamer
    gst_init(nullptr, nullptr);

    {
        RenderNode node(cfg);
        if (!node.init()) {
            DS_ERR("RenderNode init failed\n");
            gst_deinit();
            return 1;
        }
        node.run(g_stop);
        node.stop();
    }

    gst_deinit();
    DS_INFO("visionbridge-render stopped\n");
    return 0;
}
