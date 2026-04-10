/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#include "config_loader.h"
#include "logger.h"

#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// ---------------------------------------------------------------------------
// Helper: safe extraction with a typed default fallback
// ---------------------------------------------------------------------------
template<typename T>
static T jget(const json& j, const std::string& key, T def) {
    return j.contains(key) ? j.at(key).get<T>() : def;
}

// ---------------------------------------------------------------------------
// Shared sub-parsers
// ---------------------------------------------------------------------------
static DsTransportConfig parse_transport(const json& t) {
    DsTransportConfig tc;
    tc.host          = jget<std::string>(t, "host",          "239.1.1.1");
    tc.rtp_port      = jget<uint16_t>  (t, "rtp_port",       5004);
    tc.iface         = jget<std::string>(t, "iface",         "");
    tc.multicast     = jget<bool>       (t, "multicast",      true);
    tc.multicast_ttl = static_cast<uint8_t>(jget<int>(t, "multicast_ttl", 4));
    return tc;
}

static DsCodecConfig parse_codec(const json& c) {
    DsCodecConfig cc;
    cc.bitrate_kbps = jget<uint32_t>(c, "bitrate_kbps", 4000);
    cc.keyint       = jget<uint32_t>(c, "keyint",       1);
    cc.hw_encode    = jget<bool>    (c, "hw_encode",    false);
    cc.hw_decode    = jget<bool>    (c, "hw_decode",    true);
    return cc;
}

static DsZmqConfig parse_zmq(const json& z) {
    DsZmqConfig zc;
    zc.source_endpoint = jget<std::string>(z, "source_endpoint", "tcp://*:5560");
    zc.render_endpoint = jget<std::string>(z, "render_endpoint", "tcp://127.0.0.1:5560");
    return zc;
}

static DsDebugConfig parse_debug(const json& d) {
    DsDebugConfig dc;
    dc.log_level        = jget<int>     (d, "log_level",        2);
    dc.stats_interval_s = jget<uint32_t>(d, "stats_interval_s", 1);
    return dc;
}

// ---------------------------------------------------------------------------
// ConfigLoader::load_source
// ---------------------------------------------------------------------------
bool ConfigLoader::load_source(const std::string& path, DsSourceConfig& out) {
    std::ifstream f(path);
    if (!f.is_open()) {
        DS_ERR("Cannot open source config: %s\n", path.c_str());
        return false;
    }

    try {
        json j = json::parse(f);

        // Input
        if (j.contains("input")) {
            const auto& i = j.at("input");
            out.input.type    = jget<std::string>(i, "type",   "webcam");
            out.input.device  = jget<std::string>(i, "device", "/dev/video0");
            out.input.file    = jget<std::string>(i, "file",   "");
            out.input.width   = jget<uint32_t>   (i, "width",   0);
            out.input.height  = jget<uint32_t>   (i, "height",  0);
            out.input.fps_num = jget<uint32_t>   (i, "fps_num", 0);
            out.input.fps_den = jget<uint32_t>   (i, "fps_den", 1);
            out.input.loop    = jget<bool>        (i, "loop",   true);
        }

        // Detectors
        if (j.contains("detectors")) {
            const auto& d = j.at("detectors");
            out.detectors.enable_stub = jget<bool>(d, "enable_stub", true);
            out.detectors.enable_mog2 = jget<bool>(d, "enable_mog2", true);
            out.detectors.enable_yolo = jget<bool>(d, "enable_yolo", true);

            out.detectors.mog2_history       = jget<int>   (d, "mog2_history",       500);
            out.detectors.mog2_var_threshold  = jget<double>(d, "mog2_var_threshold", 16.0);
            out.detectors.mog2_min_area       = jget<int>   (d, "mog2_min_area",      800);

            if (d.contains("yolo")) {
                const auto& y = d.at("yolo");
                out.detectors.yolo.model          = jget<std::string>(y, "model",          "");
                out.detectors.yolo.config         = jget<std::string>(y, "config",         "");
                out.detectors.yolo.names          = jget<std::string>(y, "names",          "");
                out.detectors.yolo.conf_threshold = jget<float>      (y, "conf_threshold", 0.4f);
                out.detectors.yolo.nms_threshold  = jget<float>      (y, "nms_threshold",  0.45f);
                out.detectors.yolo.input_width    = jget<int>        (y, "input_width",    416);
                out.detectors.yolo.input_height   = jget<int>        (y, "input_height",   416);
                out.detectors.yolo.backend        = jget<std::string>(y, "backend",        "cpu");
                // filter_classes: optional array of class name strings
                if (y.contains("filter_classes") && y.at("filter_classes").is_array()) {
                    for (const auto& item : y.at("filter_classes"))
                        if (item.is_string())
                            out.detectors.yolo.filter_classes.push_back(item.get<std::string>());
                }
            }
        }

        if (j.contains("transport")) out.transport = parse_transport(j.at("transport"));
        if (j.contains("codec"))     out.codec     = parse_codec    (j.at("codec"));
        if (j.contains("zmq"))       out.zmq       = parse_zmq      (j.at("zmq"));
        if (j.contains("debug"))     out.debug     = parse_debug    (j.at("debug"));

        out.appsink_sync = jget<bool>(j, "appsink_sync", false);

    } catch (const std::exception& e) {
        DS_ERR("Source config parse error: %s\n", e.what());
        return false;
    }

    DS_INFO("Loaded source config: %s (%s %ux%u@%u/%ufps)\n",
            path.c_str(),
            out.input.type.c_str(),
            out.input.width, out.input.height,
            out.input.fps_num, out.input.fps_den);
    return true;
}

// ---------------------------------------------------------------------------
// ConfigLoader::load_render
// ---------------------------------------------------------------------------
bool ConfigLoader::load_render(const std::string& path, DsRenderConfig& out) {
    std::ifstream f(path);
    if (!f.is_open()) {
        DS_ERR("Cannot open render config: %s\n", path.c_str());
        return false;
    }

    try {
        json j = json::parse(f);

        if (j.contains("transport")) out.transport = parse_transport(j.at("transport"));
        if (j.contains("codec"))     out.codec     = parse_codec    (j.at("codec"));
        if (j.contains("zmq"))       out.zmq       = parse_zmq      (j.at("zmq"));
        if (j.contains("debug"))     out.debug     = parse_debug    (j.at("debug"));

        out.jitter_buffer_ms = jget<uint32_t>(j, "jitter_buffer_ms", 20);
        if (j.contains("display")) {
            const auto& d = j.at("display");
            out.display_width      = jget<uint32_t>   (d, "width",             0);
            out.display_height     = jget<uint32_t>   (d, "height",            0);
            out.display_fps        = jget<uint32_t>   (d, "fps",               30);
            out.display_title      = jget<std::string>(d, "title",
                                                       "VisionBridge");
            out.fullscreen         = jget<bool>       (d, "fullscreen",        false);
            out.sdl_display_index  = jget<int>        (d, "sdl_display_index", -1);
            out.backend            = jget<std::string>(d, "backend",           "sdl");
            out.drm_device         = jget<std::string>(d, "drm_device",        "");
            out.drm_output_index   = jget<int>        (d, "drm_output_index",  0);
        }

    } catch (const std::exception& e) {
        DS_ERR("Render config parse error: %s\n", e.what());
        return false;
    }

    DS_INFO("Loaded render config: %s (display %ux%u)\n",
            path.c_str(), out.display_width, out.display_height);
    return true;
}
