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

#include <condition_variable>
#include <gst/gst.h>
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
//   videoconvert name=vconv ! videoscale ! videorate !
//   video/x-raw,format=BGRx,width=W,height=H,framerate=F/D !
//   queue name=venc_q ! videoconvert !
//   x264enc name=venc tune=zerolatency key-int-max=KI bitrate=BR !
//   h264parse name=vsparse !
//   rtph264pay name=vpay config-interval=-1 ssrc=987654321 pt=96 !
//   udpsink host=HOST port=PORT auto-multicast=true sync=true
//
//  [file mode]
//   filesrc location=FILE ! decodebin !
//   videoconvert name=vconv ! videoscale ! videorate !
//   video/x-raw,format=BGRx,width=W,height=H,framerate=F/D !
//   queue name=venc_q ! ...  (same as above)
//
// venc_q sink pad is probed by vraw_probe_cb on every BGRx frame:
//   memcpy to FrameSlot → notify executor thread (< 1 ms, non-blocking)
//   executor thread runs trackers / emits DsMsgBbox in parallel with encoder
//
// GStreamer probes (when stats_interval_s > 0)
// --------------------------------------------
//   vconv sink pad → m_pretee_in_ts  (encode chain entry timing)
//   venc_q sink pad → m_tee_ts_map   (venc_q queue-wait origin, via vraw_probe_cb)
//   venc sink pad  → m_enc_fifo      (encoder entry — measures queue wait)
//   venc src  pad  → enc_lat         (encoder duration)
//   vsparse src pad → SEI inject     (always, unconditional)
//
// Tracker latency is measured in inference_thread_func:
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
    /// Called from the executor thread with a ready-to-send DsMsgBbox.
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
    static gboolean        bus_callback (GstBus* bus, GstMessage* msg, gpointer data);

    // ---- Pad probe callbacks ----
    static GstPadProbeReturn vconv_sink_probe_cb (GstPad*, GstPadProbeInfo*, gpointer); ///< pre-encode chain start (timing)
    static GstPadProbeReturn vconv_src_probe_cb  (GstPad*, GstPadProbeInfo*, gpointer); ///< videoconvert output → next element (timing)
    static GstPadProbeReturn vraw_probe_cb       (GstPad*, GstPadProbeInfo*, gpointer); ///< BGRx frame copy + executor notify
    static GstPadProbeReturn enc_sink_probe_cb   (GstPad*, GstPadProbeInfo*, gpointer); ///< encoder entry (timing)
    static GstPadProbeReturn enc_src_probe_cb    (GstPad*, GstPadProbeInfo*, gpointer); ///< encoder output (timing)
    static GstPadProbeReturn sei_inject_probe_cb (GstPad*, GstPadProbeInfo*, gpointer); ///< SEI timestamp embed
    static GstPadProbeReturn vpay_src_probe_cb   (GstPad*, GstPadProbeInfo*, gpointer); ///< RTP packetiser output (trace + e2e timing)
    static GstPadProbeReturn filesrc_src_probe_cb(GstPad*, GstPadProbeInfo*, gpointer); ///< filesrc output → decodebin (trace + timing)

    // ---- GLib main loop thread ----
    void gst_thread_func();

    // ---- Executor thread: runs trackers (or dummy bbox) in parallel with encoder ----
    void inference_thread_func();

    // Inference synchronisation
    // --------------------------
    // vraw_probe_cb publishes each raw BGRx frame into m_frame_slot and
    // atomically bumps m_slot_seq, waking the inference thread immediately
    // in parallel with the encoder.  The inference thread (latest-wins) always
    // processes the most recent available frame.
    //
    // The ZMQ bbox message is sent as soon as inference completes — before the
    // encoded video for that frame has been transmitted.  The render side's
    // BboxReceiver ring absorbs the early arrival; wait_for_seq() blocks the
    // render loop until the matching bbox is in the ring or 1 s elapses.
    struct FrameSlot {
        std::vector<uint8_t> data;       ///< BGRx pixel data
        uint32_t width       = 0;
        uint32_t height      = 0;
        uint32_t stride      = 0;        ///< bytes per row
        uint64_t seq         = 0;        ///< frame_seq assigned in vraw_probe_cb
        uint64_t capture_ts_us = 0;      ///< realtime clock at probe entry
    };

    // Inference handoff: written by vraw_probe_cb, read by inference_thread_func
    FrameSlot                m_frame_slot;          ///< current frame for inference
    std::mutex               m_slot_mtx;            ///< guards m_frame_slot
    std::atomic<uint64_t>    m_slot_seq{UINT64_MAX};///< published seq; UINT64_MAX = none yet
    std::atomic<bool>        m_executor_stop{false};
    std::thread              m_executor_thread;

    // ---- Timing stats printer (called by GLib timer) ----
    void emit_periodic_stats();

    // ---- Members ----
    DsSourceConfig m_cfg;
    std::vector<std::unique_ptr<ITracker>> m_trackers;
    BboxCallback m_bbox_cb;

    GstElement* m_pipeline    = nullptr;
    GstElement* m_encoder     = nullptr; ///< "venc"   (x264enc or vaapih264enc)
    GstElement* m_h264parse   = nullptr; ///< "vsparse" (SEI inject)
    GstElement* m_vconv_first = nullptr; ///< "vconv"  (timing probe entry)
    GstElement* m_venc_q      = nullptr; ///< "venc_q" (vraw probe — BGRx frame copy)
    GstElement* m_vpay        = nullptr; ///< "vpay"   (rtph264pay — RTP send trace)
    GstElement* m_filesrc     = nullptr; ///< "vfsrc"  (filesrc — file mode only)
    GMainLoop*  m_loop        = nullptr;
    std::thread m_gst_thread;

    // Probe IDs (removed before GST_STATE_NULL)
    gulong m_vconv_sink_probe_id = 0; ///< vconv sink — encode chain entry timing
    gulong m_vconv_src_probe_id  = 0; ///< vconv src  — videoconvert output timing
    gulong m_vraw_probe_id       = 0; ///< venc_q sink — BGRx copy + executor notify
    gulong m_enc_sink_probe_id   = 0; ///< encoder sink — encode entry timing
    gulong m_enc_src_probe_id    = 0; ///< encoder src  — encode duration timing
    gulong m_sei_inject_probe_id = 0; ///< h264parse src — SEI embed
    gulong m_vpay_src_probe_id   = 0; ///< vpay src — RTP send trace (always on)
    gulong m_filesrc_src_probe_id = 0;///< filesrc src — compressed chunk → decodebin
    guint  m_timing_timer_id     = 0;

    std::atomic<bool>    m_running{false};
    std::atomic<bool>    m_paused{false};
    std::atomic<uint64_t> m_frame_seq{0};
    // Last frame_seq successfully matched and SEI-injected by sei_inject_probe_cb.
    // Written on the encoder streaming thread; read by vpay_src_probe_cb on the
    // same thread — no mutex needed, but atomic for correctness.
    std::atomic<uint64_t> m_last_encoded_seq{0};

    // ---- Timing state (protected by m_timing_mtx) ----
    mutable std::mutex  m_timing_mtx;
    uint64_t            m_pretee_in_ts = 0;  ///< vconv sink stamp (for pretee latency)
    LatencyStats        m_pretee_stats;      ///< vconv sink → venc_q sink (decode+colorconv+scale+rate)
    LatencyStats        m_vconv_stats;       ///< vconv sink → vconv src  (videoconvert processing only)
    LatencyStats        m_enc_branch_stats;  ///< venc_q sink → encoder sink (queue wait)
    LatencyStats        m_enc_stats;         ///< encoder sink → encoder src (encode duration)
    LatencyStats        m_executor_stats;    ///< executor thread: tracker inference duration
    // Per-tracker latency stats; indexed by tracker position in m_trackers
    static constexpr int kMaxTrackers = 4;
    LatencyStats        m_tracker_stats[kMaxTrackers];

    // FIFO ring for encoder entry timestamps (push at enc_sink, pop at enc_src).
    // Sized to cover the encoder's max in-flight depth.
    static constexpr int kEncFifoSize = 16;
    struct EncFifoEntry { uint64_t ts = 0; uint64_t frame_seq = 0; bool used = false; };
    EncFifoEntry m_enc_fifo[kEncFifoSize]{};
    size_t       m_enc_fifo_head = 0;

    // ---- venc_q-exit PTS timestamps for enc-branch queue latency (m_tee_ts_mtx) ----
    // Written by vraw_probe_cb; read and cleared by enc_sink_probe_cb.
    struct TeeTsEntry {
        GstClockTime pts      = GST_CLOCK_TIME_NONE;
        uint64_t     t_tee_us = 0; ///< 0 = empty slot
    };
    static constexpr int kTeeTsMapSize = 16;
    TeeTsEntry         m_tee_ts_map[kTeeTsMapSize]{};
    size_t             m_tee_ts_map_head = 0;
    mutable std::mutex m_tee_ts_mtx;

    // ---- SEI PTS correlation map (protected by m_sei_map_mtx) ----
    // Written by vraw_probe_cb; consumed by sei_inject_probe_cb.
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
