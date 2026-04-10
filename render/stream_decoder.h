/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#ifndef VISIONBRIDGE_STREAM_DECODER_H
#define VISIONBRIDGE_STREAM_DECODER_H

#include "../common/config.h"
#include "../common/pipeline_stats.h"

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>

// ---------------------------------------------------------------------------
// DecodedFrame — one fully decoded RGBA frame in CPU memory
// ---------------------------------------------------------------------------
struct DecodedFrame {
    uint32_t             width  = 0;
    uint32_t             height = 0;
    uint32_t             stride = 0;   ///< bytes per row
    std::vector<uint8_t> data;         ///< RGBA packed, row-major

    // In-band E2E: extracted from the H.264 SEI NAL injected at source (0 = absent)
    uint64_t sei_capture_ts_us = 0; ///< CLOCK_REALTIME at source appsink entry (µs)
    uint64_t sei_frame_seq     = 0; ///< source frame counter

    size_t data_bytes() const { return static_cast<size_t>(stride) * height; }
};

// ---------------------------------------------------------------------------
// StreamDecoder
//
// Receives an H.264 RTP stream from the network and decodes it to RGBA frames.
// Each frame is delivered to the registered frame callback, which the
// DisplayOutput uses to update the SDL2 texture.
//
// GStreamer pipeline (SW decode):
//   udpsrc port=PORT [multicast-group=GRP auto-multicast=true]
//     caps="application/x-rtp,media=video,clock-rate=90000,
//           encoding-name=H264,payload=96"
//     buffer-size=2097152
//   ! rtpjitterbuffer latency=JB drop-on-latency=true
//   ! rtph264depay
//   ! h264parse name=vparser
//   ! [vaapih264dec | avdec_h264] name=vdec
//   ! videoconvert
//   ! video/x-raw,format=RGBA
//   ! appsink name=vsink sync=false max-buffers=1 drop=true emit-signals=true
//
// Probes (when stats_interval_s > 0):
//   h264parse src pad   → m_pre_dec_ts         (pre-decode entry timestamp)
//   vdec sink pad       → m_vdec_sink_ts        (decoder entry)
//   vdec src  pad       → pure_dec = now - m_vdec_sink_ts
//   appsink sink pad    → total_dec = now - m_pre_dec_ts
//
// Stats are printed as [TIMING-RECV] by a periodic GLib timer.
// ---------------------------------------------------------------------------
class StreamDecoder {
public:
    using FrameCallback = std::function<void(std::shared_ptr<DecodedFrame>)>;

    explicit StreamDecoder(const DsRenderConfig& cfg);
    ~StreamDecoder();

    void set_frame_callback(FrameCallback cb) { m_frame_cb = cb; }

    /// Build and start the decode pipeline.  Returns false on failure.
    bool start();

    /// Stop and tear down.
    void stop();

    bool is_running() const { return m_running.load(); }

    uint64_t frames_decoded() const { return m_frames_decoded.load(); }

private:
    std::string build_pipeline_str() const;

    static GstFlowReturn   on_new_sample       (GstAppSink*, gpointer);
    static gboolean        bus_callback        (GstBus*, GstMessage*, gpointer);

    // Timing probes (registered only when stats_interval_s > 0)
    static GstPadProbeReturn pre_dec_probe_cb    (GstPad*, GstPadProbeInfo*, gpointer);
    static GstPadProbeReturn vdec_sink_probe_cb  (GstPad*, GstPadProbeInfo*, gpointer);
    static GstPadProbeReturn vdec_src_probe_cb   (GstPad*, GstPadProbeInfo*, gpointer);
    static GstPadProbeReturn appsink_probe_cb    (GstPad*, GstPadProbeInfo*, gpointer);
    // SEI extract probe (always registered — not stats-gated)
    static GstPadProbeReturn sei_extract_probe_cb(GstPad*, GstPadProbeInfo*, gpointer);

    void gst_thread_func();
    void emit_periodic_stats();

    DsRenderConfig m_cfg;
    bool m_hw_decode = false; ///< resolved at start() — may differ from cfg if VAAPI absent
    FrameCallback  m_frame_cb;

    GstElement* m_pipeline  = nullptr;
    GstElement* m_appsink   = nullptr;
    GstElement* m_vdepay    = nullptr; ///< "vdepay" rtph264depay (SEI extract)
    GstElement* m_h264parse = nullptr; ///< "vparser"
    GstElement* m_vdec      = nullptr; ///< "vdec"
    GMainLoop*  m_loop      = nullptr;
    std::thread m_gst_thread;

    gulong m_pre_dec_probe_id     = 0;
    gulong m_vdec_sink_probe_id   = 0;
    gulong m_vdec_src_probe_id    = 0;
    gulong m_appsink_probe_id     = 0;
    gulong m_sei_extract_probe_id = 0; ///< SEI extract on vdepay src (always)
    guint  m_timing_timer_id      = 0;

    // SEI PTS correlation map (written by sei_extract_probe_cb, read by on_new_sample).
    // Keyed by GstBuffer PTS so that concurrent in-flight frames map correctly.
    struct SeiMapEntry {
        GstClockTime pts           = GST_CLOCK_TIME_NONE;
        uint64_t     capture_ts_us = 0; ///< 0 = empty slot
        uint64_t     frame_seq     = 0;
    };
    static constexpr int kSeiMapSize = 16;
    SeiMapEntry         m_sei_map[kSeiMapSize]{};
    size_t              m_sei_map_head = 0;
    mutable std::mutex  m_sei_mtx;

    std::atomic<bool>     m_running{false};
    std::atomic<uint64_t> m_frames_decoded{0};
    std::atomic<uint64_t> m_frames_dropped{0}; ///< SEI seq-gap drops detected in on_new_sample
    uint64_t              m_last_sei_seq = 0;   ///< last sei_frame_seq seen (main thread)

    // Timing state (protected by m_timing_mtx)
    mutable std::mutex m_timing_mtx;
    // FIFO entry timestamps (push at entry probe, pop at exit probe).
    // Using a small ring large enough to cover max decoder in-flight depth.
    static constexpr int kProbeFifoSize = 16;
    struct ProbeEntry { uint64_t ts = 0; bool used = false; };
    ProbeEntry m_pre_dec_fifo [kProbeFifoSize]{}; ///< h264parse src entry timestamps
    ProbeEntry m_vdec_sink_fifo[kProbeFifoSize]{}; ///< vdec sink entry timestamps
    size_t     m_pre_dec_head  = 0;
    size_t     m_vdec_head     = 0;
    LatencyStats m_pure_dec_stats;  ///< vdec sink→src (pure HW/SW decode)
    LatencyStats m_total_dec_stats; ///< h264parse src→appsink sink
};

#endif // VISIONBRIDGE_STREAM_DECODER_H
