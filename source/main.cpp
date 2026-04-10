/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

// VisionBridge source node entry point
//
// Usage:
//   visionbridge_source --config configs/source.json [options]

#include "source_node.h"
#include "../common/config_loader.h"
#include "../common/logger.h"

#include <csignal>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <getopt.h>
#include <gst/gst.h>

static std::atomic<bool>  g_shutdown{false};
static SourceNode*        g_node = nullptr;

static void signal_handler(int sig) {
    DS_INFO("\nReceived signal %d, shutting down source node...\n", sig);
    g_shutdown.store(true);
    if (g_node) g_node->request_stop();
}

// Replace host in a ZMQ endpoint such as tcp://1.2.3.4:5560
static std::string replace_endpoint_host(const std::string& ep, const std::string& new_host) {
    const auto scheme_end = ep.find("://");
    if (scheme_end == std::string::npos) return ep;
    const auto host_start = scheme_end + 3;
    const auto colon_pos  = ep.find(':', host_start);
    if (colon_pos == std::string::npos) return ep;
    return ep.substr(0, host_start) + new_host + ep.substr(colon_pos);
}

static void print_usage(const char* prog) {
    printf("VisionBridge Source Node\n");
    printf("Usage: %s [options]\n\n", prog);
    printf("  -c, --config <file>       Source config JSON (default: configs/source.json)\n");
    printf("  -w, --webcam [device]     Override input to webcam (default: /dev/video0)\n");
    printf("  -f, --file <path>         Override input to video file\n");
    printf("  -d, --dest <host>         Send RTP unicast to specific host (disables multicast)\n");
    printf("  -m, --multicast [group]   Send RTP to multicast group (default: 239.1.1.1)\n");
    printf("  -l, --log-level <lvl>     Log level: error warn info debug trace (default: info)\n");
    printf("      --loop                Loop video file when EOS is reached (file input only)\n");
    printf("      --no-loop             Stop when video file ends\n");
    printf("  -h, --help                Show this help\n");
    printf("\nTransport mode (default: multicast 239.1.1.1:5004 per source.json):\n");
    printf("  Multicast (default): %s --config configs/source.json\n", prog);
    printf("  Multicast explicit:  %s --multicast 239.1.1.1\n", prog);
    printf("  Unicast:             %s --dest 192.168.1.20\n", prog);
    printf("\nOther examples:\n");
    printf("  %s --webcam /dev/video1 --log-level debug\n", prog);
    printf("  %s --file /path/to/video.mp4 --dest 192.168.1.20\n", prog);
}

int main(int argc, char* argv[]) {
    std::string config_path = "configs/source.json";
    std::string webcam_override;
    std::string file_override;
    std::string dest_host;      // --dest   unicast target
    std::string mcast_group;    // --multicast group override
    bool dest_set   = false;
    bool mcast_set  = false;
    int  loop_override = -1;    // -1 = not set, 0 = no-loop, 1 = loop
    int log_level = -1; // -1 = not set by CLI; will fall back to config value

    static const struct option long_opts[] = {
        {"config",    required_argument, nullptr, 'c'},
        {"webcam",    optional_argument, nullptr, 'w'},
        {"file",      required_argument, nullptr, 'f'},
        {"dest",      required_argument, nullptr, 'd'},
        {"multicast", optional_argument, nullptr, 'm'},
        {"loop",      no_argument,       nullptr, 'L'},
        {"no-loop",   no_argument,       nullptr, 'N'},
        {"log-level", required_argument, nullptr, 'l'},
        {"help",      no_argument,       nullptr, 'h'},
        {nullptr, 0, nullptr, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "c:w::f:d:m::l:hLN", long_opts, nullptr)) != -1) {
        switch (opt) {
        case 'c': config_path     = optarg; break;
        // optional_argument requires no space between flag and value (e.g. -w/dev/video1).
        // When invoked as "-w /dev/video1" optarg is NULL; treat the next bare argument
        // as the device path so the common "-w /dev/videoN" form works intuitively.
        case 'w':
            if (optarg) {
                webcam_override = optarg;
            } else if (optind < argc && argv[optind][0] != '-') {
                webcam_override = argv[optind++];
            } else {
                webcam_override = "/dev/video0";
            }
            break;
        case 'f': file_override   = optarg; break;
        case 'd': dest_host  = optarg; dest_set  = true; break;
        case 'm': mcast_group = (optarg ? optarg : "239.1.1.1"); mcast_set = true; break;
        case 'L': loop_override = 1; break;
        case 'N': loop_override = 0; break;
        case 'l':
            if      (strcmp(optarg, "error") == 0) log_level = 0;
            else if (strcmp(optarg, "warn")  == 0) log_level = 1;
            else if (strcmp(optarg, "info")  == 0) log_level = 2;
            else if (strcmp(optarg, "debug") == 0) log_level = 3;
            else if (strcmp(optarg, "trace") == 0) log_level = 4;
            break;
        case 'h': print_usage(argv[0]); return 0;
        default:  print_usage(argv[0]); return 1;
        }
    }

    // Apply CLI log level early so config-load messages are visible at the right level.
    // Default to INFO if not set by CLI yet (config may override below).
    if (log_level >= 0) ds_set_log_level(static_cast<LogLevel>(log_level));
    else                ds_set_log_level(LogLevel::INFO);

    DsSourceConfig cfg;
    if (!ConfigLoader::load_source(config_path, cfg)) {
        DS_ERR("Failed to load config: %s\n", config_path.c_str());
        return 1;
    }

    // Apply CLI overrides
    if (!webcam_override.empty()) {
        cfg.input.type   = "webcam";
        cfg.input.device = webcam_override;
        cfg.input.file.clear();
    }
    if (!file_override.empty()) {
        cfg.input.type = "file";
        cfg.input.file = file_override;
    }
    if (loop_override >= 0)
        cfg.input.loop = (loop_override == 1);
    // Transport mode: --dest takes priority over --multicast
    if (dest_set) {
        cfg.transport.host      = dest_host;
        cfg.transport.multicast = false;
        // Unicast: source PUSH will connect to render; update render_endpoint host
        cfg.zmq.render_endpoint = replace_endpoint_host(cfg.zmq.render_endpoint, dest_host);
        DS_INFO("Transport override: UNICAST dest=%s:%u, zmq connect=%s\n",
                cfg.transport.host.c_str(), cfg.transport.rtp_port,
                cfg.zmq.render_endpoint.c_str());
    } else if (mcast_set) {
        cfg.transport.host      = mcast_group;
        cfg.transport.multicast = true;
        DS_INFO("Transport override: MULTICAST group=%s:%u\n",
                cfg.transport.host.c_str(), cfg.transport.rtp_port);
    }

    // Apply final log level: CLI takes priority over config JSON value.
    if (log_level < 0) ds_set_log_level(static_cast<LogLevel>(cfg.debug.log_level));

    DS_INFO("visionbridge-source starting\n");
    DS_INFO("  input:     %s (%s)\n",
            cfg.input.type.c_str(),
            cfg.input.type == "file" ? cfg.input.file.c_str() : cfg.input.device.c_str());
    DS_INFO("  appsink:   sync=%s\n", cfg.appsink_sync ? "true (real-time throttle)" : "false");
    DS_INFO("  transport: %s %s:%u\n",
            cfg.transport.multicast ? "[MULTICAST]" : "[UNICAST]",
            cfg.transport.host.c_str(), cfg.transport.rtp_port);
    DS_INFO("  zmq push:  %s (%s)\n",
            cfg.transport.multicast ? cfg.zmq.source_endpoint.c_str() : cfg.zmq.render_endpoint.c_str(),
            cfg.transport.multicast ? "bind" : "connect");

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    gst_init(&argc, &argv);

    SourceNode node(cfg);
    g_node = &node;

    if (!node.init()) {
        DS_ERR("SourceNode init failed\n");
        gst_deinit();
        return 1;
    }

    int ret = node.run();
    gst_deinit();
    DS_INFO("SourceNode exited with code %d\n", ret);
    return ret;
}
