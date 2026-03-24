/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#ifndef VISIONBRIDGE_CONFIG_H
#define VISIONBRIDGE_CONFIG_H

#include <string>
#include <vector>
#include <cstdint>

// ---------------------------------------------------------------------------
// InputConfig — video capture parameters (source side)
// ---------------------------------------------------------------------------
struct InputConfig {
    std::string type   = "webcam"; ///< "webcam" | "file"
    std::string device = "/dev/video0"; ///< V4L2 device path (webcam mode)
    std::string file;               ///< Video file path (file mode)
    uint32_t    width  = 0;   ///< 0 = use native input resolution
    uint32_t    height = 0;   ///< 0 = use native input resolution
    uint32_t    fps_num = 0;  ///< 0 = use native input frame rate
    uint32_t    fps_den = 1;
    bool        loop   = true;      ///< Loop file input when EOS is reached
};

// ---------------------------------------------------------------------------
// DetectorConfig — which detectors to enable and their parameters
// ---------------------------------------------------------------------------
struct YoloConfig {
    std::string model;  ///< Path to .weights or .onnx file
    std::string config; ///< Path to .cfg file (empty for ONNX models)
    std::string names;  ///< Path to coco.names or similar class list
    float conf_threshold  = 0.4f;
    float nms_threshold   = 0.45f;
    int   input_width     = 416;
    int   input_height    = 416;
    /// Inference backend: "cpu" | "opencl" | "opencl_fp16" | "cuda" | "cuda_fp16"
    /// Falls back to CPU automatically if the requested backend is unavailable.
    std::string backend = "cpu";
    /// If non-empty, only detections whose class name appears in this list are
    /// reported.  Names are matched case-insensitively against coco.names.
    /// Empty list = report all classes.
    std::vector<std::string> filter_classes;
};

struct DetectorConfig {
    bool enable_stub = true;
    bool enable_mog2 = true;
    bool enable_yolo = true;  ///< Silently disabled if OpenCV DNN not available

    // MOG2 parameters
    int   mog2_history       = 500;
    double mog2_var_threshold = 16.0;
    int   mog2_min_area      = 800;   ///< Minimum contour area in px² to report

    YoloConfig yolo;
};

// ---------------------------------------------------------------------------
// TransportConfig — RTP stream delivery
// ---------------------------------------------------------------------------
struct DsTransportConfig {
    std::string host           = "239.1.1.1"; ///< Multicast group or unicast host
    uint16_t    rtp_port       = 5004;
    std::string iface;                        ///< Source interface IP (empty = default)
    bool        multicast      = true;
    uint8_t     multicast_ttl  = 4;
};

// ---------------------------------------------------------------------------
// CodecConfig — H.264 encode/decode
// ---------------------------------------------------------------------------
struct DsCodecConfig {
    uint32_t bitrate_kbps = 4000;   ///< Target encode bitrate
    uint32_t keyint       = 1;      ///< GOP size (1 = all-intra, lowest latency)
    bool     hw_encode    = false;  ///< Try VAAPI hardware encode
    bool     hw_decode    = true;   ///< Try VAAPI hardware decode (render)
};

// ---------------------------------------------------------------------------
// ZmqConfig — ZeroMQ bbox channel
// ---------------------------------------------------------------------------
struct DsZmqConfig {
    std::string source_endpoint = "tcp://*:5560";  ///< PUSH bind endpoint
    std::string render_endpoint = "tcp://127.0.0.1:5560"; ///< PULL connect endpoint
};

// ---------------------------------------------------------------------------
// DebugConfig — logging and timing
// ---------------------------------------------------------------------------
struct DsDebugConfig {
    int      log_level       = 2;   ///< 0=err 1=warn 2=info 3=debug 4=trace
    uint32_t stats_interval_s = 1;  ///< 0 = disable all timing probes
};

// ---------------------------------------------------------------------------
// DsSourceConfig — full source node configuration
// ---------------------------------------------------------------------------
struct DsSourceConfig {
    InputConfig     input;
    DetectorConfig  detectors;
    DsTransportConfig transport;
    DsCodecConfig   codec;
    DsZmqConfig     zmq;
    DsDebugConfig   debug;

    uint32_t jitter_buffer_ms = 0; ///< unused on source, kept for completeness
};

// ---------------------------------------------------------------------------
// DsRenderConfig — full render node configuration
// ---------------------------------------------------------------------------
struct DsRenderConfig {
    DsTransportConfig transport;
    DsCodecConfig     codec;
    DsZmqConfig       zmq;
    DsDebugConfig     debug;

    uint32_t jitter_buffer_ms = 20;  ///< RTP jitter buffer (ms) — low for live
    bool     decoder_sync     = false; ///< appsink sync: false=free-run (bbox aligned), true=clock-paced

    // Display
    uint32_t    display_width  = 0;   ///< 0 = match decoded frame width
    uint32_t    display_height = 0;   ///< 0 = match decoded frame height
    uint32_t    display_fps    = 30;  ///< max render rate; caps file-playback speed
    std::string display_title  = "VisionBridge";
    bool        fullscreen     = false;
    int         sdl_display_index = -1; ///< -1 = primary display
};

#endif // VISIONBRIDGE_CONFIG_H
