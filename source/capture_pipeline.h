/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#ifndef VISIONBRIDGE_CAPTURE_PIPELINE_H
#define VISIONBRIDGE_CAPTURE_PIPELINE_H

#include "../common/config.h"
#include "../common/pipeline_stats.h"
#include "../common/protocol.h"
#include "tracker.h"

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>

// ---------------------------------------------------------------------------
// CapturePipeline
//
// Owns the GStreamer encode + capture pipeline for the source node.
//
// Pipeline topology
// -----------------
//
//  [webcam mode]
//   v4l2src device=DEV  !
//   videoconvert ! videoscale ! videorate !
//   video/x-raw,format=BGRx,width=W,height=H,framerate=F/D !
//   tee name=t
//     t.  ! queue leaky=downstream max-size-buffers=1 !
//         appsink name=vsink sync=false drop=true max-buffers=1
//              (→ tracker appsink callback: runs all 3 detectors,
//                 aggregates DsMsgBbox, sends via ZMQ PUSH)
//     t.  ! queue leaky=downstream max-size-buffers=2 !
//         x264enc name=venc tune=zerolatency speed-preset=ultrafast
//                 key-int-max=KI bitrate=BR !
//         h264parse !
//         rtph264pay name=vpay config-interval=-1 ssrc=987654321 pt=96 mtu=1400 !
//         udpsink host=HOST port=PORT auto-multicast=true sync=false
//
//  [file mode]
//   filesrc location=FILE ! decodebin !
//   videoconvert ! videoscale ! videorate !
//   video/x-raw,format=BGRx,width=W,height=H,framerate=F/D !
//   tee name=t
//   ... (same tee branches as above)
//
// GStreamer probes (when stats_interval_s > 0)
// --------------------------------------------
//   venc sink pad  → m_enc_in_ts       (capture encode entry timestamp)
//   venc src  pad  → compute enc_lat = now - m_enc_in_ts  (skip HEADER bufs)
//   vpay src  pad  → compute pay_lat = now - enc_out_ts   (RTP packetise time)
//
// Tracker latency is measured inside the appsink callback:
//   for each enabled tracker: t0 = ds_mono_us(); call process(); t1 = ds_mono_us();
//   tracker_lat[i] = t1 - t0
//
// Stats timer (g_timeout_add_seconds, interval = debug.stats_interval_s):
//   Prints [TIMING-SRC] with all accumulated LatencyStats, then resets.
//   When stats_interval_s == 0, no probes are attached and no timer is registered.
//
// Usage
// -----
//   1.  Construct with config and list of ITracker*
//   2.  set_bbox_callback()  — register the ZMQ-send function
//   3.  start()
//   4.  Poll is_running(); stop() to tear down
// ---------------------------------------------------------------------------
class CapturePipeline {
public:
    /// Called from the appsink thread with a ready-to-send DsMsgBbox.
    using BboxCallback = std::function<void(const DsMsgBbox&)>;

    CapturePipeline(const DsSourceConfig& cfg,
                    std::vector<std::unique_ptr<ITracker>> trackers);
    ~CapturePipeline();

    void set_bbox_callback(BboxCallback cb) { m_bbox_cb = cb; }

    /// Build and start the GStreamer pipeline.  Returns false on failure.
    bool start();

    /// Stop and tear down the pipeline.
    void stop();

    /// Toggle between PAUSED and PLAYING states.  Safe to call from any thread.
    void toggle_pause();

    bool is_running() const { return m_running.load(); }
    bool is_paused()  const { return m_paused.load(); }

private:
    // ---- Pipeline construction ----
    std::string build_pipeline_str() const;

    // ---- Static GStreamer callbacks ----
    static GstFlowReturn   on_new_sample(GstAppSink* sink, gpointer data);
    static gboolean        bus_callback (GstBus* bus, GstMessage* msg, gpointer data);

    // ---- Pad probe callbacks (all return GST_PAD_PROBE_OK) ----
    static GstPadProbeReturn enc_sink_probe_cb   (GstPad*, GstPadProbeInfo*, gpointer);
    static GstPadProbeReturn enc_src_probe_cb    (GstPad*, GstPadProbeInfo*, gpointer);
    static GstPadProbeReturn pay_src_probe_cb    (GstPad*, GstPadProbeInfo*, gpointer);
    static GstPadProbeReturn sei_inject_probe_cb (GstPad*, GstPadProbeInfo*, gpointer);

    // ---- GLib main loop thread ----
    void gst_thread_func();

    // ---- Timing stats printer (called by GLib timer) ----
    void print_timing_stats();

    // ---- Members ----
    DsSourceConfig m_cfg;
    std::vector<std::unique_ptr<ITracker>> m_trackers;
    BboxCallback m_bbox_cb;

    GstElement* m_pipeline   = nullptr;
    GstElement* m_appsink    = nullptr;
    GstElement* m_encoder    = nullptr; ///< "venc" (x264enc or vaapih264enc)
    GstElement* m_h264parse  = nullptr; ///< "vsparse" (source h264parse, SEI inject)
    GstElement* m_rtph264pay = nullptr; ///< "vpay"
    GMainLoop*  m_loop       = nullptr;
    std::thread m_gst_thread;

    // Probe IDs (removed before GST_STATE_NULL)
    gulong m_enc_sink_probe_id   = 0;
    gulong m_enc_src_probe_id    = 0;
    gulong m_pay_src_probe_id    = 0;
    gulong m_sei_inject_probe_id = 0; ///< SEI timestamp inject on h264parse src
    guint  m_timing_timer_id     = 0;

    std::atomic<bool>    m_running{false};
    std::atomic<bool>    m_paused{false};
    std::atomic<uint64_t> m_frame_seq{0};

    // ---- Timing state (protected by m_timing_mtx) ----
    mutable std::mutex  m_timing_mtx;
    uint64_t            m_enc_in_ts = 0;    ///< encoder entry timestamp (overwrite)
    uint64_t            m_enc_out_ts = 0;   ///< encoder exit timestamp (overwrite)
    LatencyStats        m_enc_stats;        ///< pure encode latency  (venc sink→src)
    LatencyStats        m_pay_stats;        ///< RTP pay latency      (venc src→vpay src)
    // Per-tracker latency stats; indexed by tracker position in m_trackers
    static constexpr int kMaxTrackers = 4;
    LatencyStats        m_tracker_stats[kMaxTrackers];

    // ---- SEI PTS correlation map (protected by m_sei_map_mtx) ----
    // Written from the appsink callback; consumed by sei_inject_probe_cb.
    // Ring buffer large enough for any encode in-flight depth.
    struct SeiMapEntry {
        GstClockTime pts           = GST_CLOCK_TIME_NONE;
        uint64_t     capture_ts_us = 0; ///< 0 = empty slot
        uint64_t     frame_seq     = 0;
    };
    static constexpr int kSeiMapSize = 16;
    SeiMapEntry         m_sei_map[kSeiMapSize]{};
    size_t              m_sei_map_head = 0;
    mutable std::mutex  m_sei_map_mtx;
};

#endif // VISIONBRIDGE_CAPTURE_PIPELINE_H
