/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#include "capture_pipeline.h"
#include "../common/logger.h"
#include "../common/time_utils.h"

#include <sstream>
#include <cinttypes>
#include <cstring>
#include <opencv2/core.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------
CapturePipeline::CapturePipeline(const DsSourceConfig& cfg,
                                 std::vector<std::unique_ptr<ITracker>> trackers)
    : m_cfg(cfg)
    , m_trackers(std::move(trackers))
{}

CapturePipeline::~CapturePipeline() {
    stop();
}

// ---------------------------------------------------------------------------
// build_pipeline_str
// ---------------------------------------------------------------------------
std::string CapturePipeline::build_pipeline_str() const {
    std::ostringstream ss;

    const auto& in  = m_cfg.input;
    const auto& tr  = m_cfg.transport;
    const auto& co  = m_cfg.codec;

    // ----- Input source -----
    if (in.type == "file" && !in.file.empty()) {
        ss << "filesrc location=\"" << in.file << "\""
           << " ! decodebin";
    } else {
        // Live V4L2 webcam
        ss << "v4l2src device=\"" << in.device << "\"";
    }

    // ----- Common format conversion & scaling -----
    ss << " ! videoconvert"
       << " ! videoscale"
       << " ! videorate"
       // BGRx: 4-byte-aligned BGR (OpenCV can use directly without copy)
       << " ! video/x-raw,format=BGRx"
       << ",width="     << in.width
       << ",height="    << in.height
       << ",framerate=" << in.fps_num << "/" << in.fps_den
       << " ! tee name=t";

    // ----- Tracker/appsink branch -----
    // leaky=downstream: if appsink is busy (tracker running), drop the incoming
    // frame rather than blocking the encode branch.
    ss << "  t. ! queue leaky=downstream max-size-buffers=1"
          "         max-size-bytes=0 max-size-time=0"
       << " ! appsink name=vsink"
          "   sync=false emit-signals=true"
          "   max-buffers=1 drop=true";

    // ----- Encode branch -----
    // videoconvert is required here: the tee delivers BGRx (needed by the
    // appsink/OpenCV branch) but x264enc and vaapih264enc only accept YUV
    // formats (I420, NV12, ...).  The conversion is fast (~1 ms) and is
    // the only place we pay this cost.
    ss << "  t. ! queue leaky=downstream max-size-buffers=2"
          "         max-size-bytes=0 max-size-time=0"
       << " ! videoconvert";

    if (co.hw_encode) {
        ss << " ! vaapih264enc name=venc rate-control=cbr"
           << " bitrate="          << co.bitrate_kbps
           << " keyframe-period="  << co.keyint
           << " max-bframes=0 tune=low-power"
           << " ! video/x-h264,stream-format=byte-stream";
    } else {
        // tune=zerolatency: disables lookahead and B-frames; combined with
        // key-int-max=1 (all-intra) this gives the absolute minimum encode latency.
        ss << " ! x264enc name=venc tune=zerolatency"
           << " bitrate="     << co.bitrate_kbps
           << " key-int-max=" << co.keyint
           << " bframes=0 speed-preset=ultrafast"
           << " ! video/x-h264,stream-format=byte-stream";
    }

    // config-interval=-1 → inject SPS/PPS before every IDR
    // ssrc fixed → receiver treats all pipeline instances as the same sender
    ss << " ! h264parse"
       << " ! rtph264pay name=vpay config-interval=-1"
          " ssrc=987654321 pt=96 mtu=1400";

    // ----- Transport sink -----
    if (tr.multicast) {
        ss << " ! udpsink host=\"" << tr.host << "\""
           << " port=" << tr.rtp_port
           << " auto-multicast=true sync=false";
        if (!tr.iface.empty())
            ss << " multicast-iface=\"" << tr.iface << "\"";
    } else {
        ss << " ! udpsink host=\"" << tr.host << "\""
           << " port=" << tr.rtp_port
           << " sync=false";
    }

    return ss.str();
}

// ---------------------------------------------------------------------------
// on_new_sample — appsink callback (GStreamer streaming thread)
//
// 1. Extract BGRx pixel data
// 2. Run all enabled trackers
// 3. Aggregate results into DsMsgBbox
// 4. Fire bbox callback (ZMQ send)
// ---------------------------------------------------------------------------
GstFlowReturn CapturePipeline::on_new_sample(GstAppSink* sink, gpointer data) {
    // Record the wall-clock time as early as possible — this is the "frame enters
    // processing" timestamp used for end-to-end latency measurement on the render side.
    const uint64_t capture_ts = ds_realtime_us();

    auto* self = static_cast<CapturePipeline*>(data);

    GstSample* sample = gst_app_sink_pull_sample(sink);
    if (!sample) return GST_FLOW_ERROR;

    GstBuffer* buf  = gst_sample_get_buffer(sample);
    GstCaps*   caps = gst_sample_get_caps(sample);

    if (!buf || !caps) {
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    // Retrieve frame dimensions from caps
    GstVideoInfo vinfo;
    if (!gst_video_info_from_caps(&vinfo, caps)) {
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    GstMapInfo map;
    if (!gst_buffer_map(buf, &map, GST_MAP_READ)) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    const int W = vinfo.width;
    const int H = vinfo.height;
    const int stride = vinfo.stride[0]; // bytes per row (BGRx = 4 bytes/pixel)

    // Wrap pixel data in a cv::Mat (zero-copy — we hold the GstMapInfo)
    // BGRx has 4 channels; OpenCV detectors just read the first 3 (B, G, R).
    cv::Mat frame(H, W, CV_8UC4,
                  reinterpret_cast<void*>(map.data),
                  static_cast<size_t>(stride));

    // ---- Run all trackers and aggregate bboxes ----
    DsMsgBbox msg{};
    msg.type             = DS_MSG_BBOX;
    msg.protocol_version = DS_PROTOCOL_VERSION;
    msg.frame_seq        = self->m_frame_seq.fetch_add(1, std::memory_order_relaxed);
    msg.capture_ts_us    = capture_ts;  // set at callback entry — before tracker cost
    msg.send_ts_us       = 0;           // filled just before ZMQ send
    msg.bbox_count       = 0;

    const uint32_t stats_en = self->m_cfg.debug.stats_interval_s;

    for (size_t ti = 0; ti < self->m_trackers.size() && ti < kMaxTrackers; ++ti) {
        const uint64_t t0 = stats_en ? ds_mono_us() : 0;

        std::vector<DsBbox> boxes = self->m_trackers[ti]->process(frame);

        if (stats_en) {
            const uint64_t lat = ds_mono_us() - t0;
            std::lock_guard<std::mutex> lk(self->m_timing_mtx);
            self->m_tracker_stats[ti].record(lat);
        }

        // Append boxes to message (respect DS_MAX_BBOXES limit)
        for (const auto& b : boxes) {
            if (msg.bbox_count >= DS_MAX_BBOXES) break;
            msg.bboxes[msg.bbox_count++] = b;
        }
    }

    gst_buffer_unmap(buf, &map);
    gst_sample_unref(sample);

    // Fire callback (ZMQ send happens there)
    if (self->m_bbox_cb) {
        msg.send_ts_us = ds_realtime_us();
        self->m_bbox_cb(msg);
    }

    DS_TRACE("CapturePipeline: frame seq=%" PRIu64 " bboxes=%u\n",
             msg.frame_seq, msg.bbox_count);

    return GST_FLOW_OK;
}

// ---------------------------------------------------------------------------
// bus_callback
// ---------------------------------------------------------------------------
gboolean CapturePipeline::bus_callback(GstBus* /*bus*/, GstMessage* msg, gpointer data) {
    auto* self = static_cast<CapturePipeline*>(data);

    switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_EOS:
        // For file input with loop=true: seek back to start instead of stopping.
        if (self->m_cfg.input.type == "file" && self->m_cfg.input.loop) {
            DS_INFO("CapturePipeline: EOS — looping file from start\n");
            gst_element_seek(self->m_pipeline,
                             1.0,                  // rate
                             GST_FORMAT_TIME,
                             static_cast<GstSeekFlags>(GST_SEEK_FLAG_FLUSH |
                                                       GST_SEEK_FLAG_KEY_UNIT),
                             GST_SEEK_TYPE_SET, 0, // start: position 0
                             GST_SEEK_TYPE_NONE, GST_CLOCK_TIME_NONE); // end: none
        } else {
            DS_INFO("CapturePipeline: EOS\n");
            self->m_running.store(false);
            g_main_loop_quit(self->m_loop);
        }
        break;
    case GST_MESSAGE_ERROR: {
        GError* err = nullptr; gchar* dbg = nullptr;
        gst_message_parse_error(msg, &err, &dbg);
        DS_ERR("CapturePipeline: error: %s (%s)\n", err->message, dbg ? dbg : "");
        g_error_free(err); g_free(dbg);
        self->m_running.store(false);
        g_main_loop_quit(self->m_loop);
        break;
    }
    case GST_MESSAGE_WARNING: {
        GError* err = nullptr; gchar* dbg = nullptr;
        gst_message_parse_warning(msg, &err, &dbg);
        DS_WARN("CapturePipeline: warning: %s\n", err->message);
        g_error_free(err); g_free(dbg);
        break;
    }
    default:
        break;
    }
    return TRUE;
}

// ---------------------------------------------------------------------------
// Pad probe callbacks — encoder latency measurement
// ---------------------------------------------------------------------------

// venc sink pad → record entry timestamp (scalar overwrite — see fileSource for rationale)
GstPadProbeReturn CapturePipeline::enc_sink_probe_cb(GstPad*, GstPadProbeInfo*,
                                                     gpointer udata) {
    auto* self = static_cast<CapturePipeline*>(udata);
    const uint64_t ts = ds_mono_us();
    std::lock_guard<std::mutex> lk(self->m_timing_mtx);
    self->m_enc_in_ts = ts;
    return GST_PAD_PROBE_OK;
}

// venc src pad → compute encode latency (skip HEADER/SPS/PPS buffers)
GstPadProbeReturn CapturePipeline::enc_src_probe_cb(GstPad*, GstPadProbeInfo* info,
                                                    gpointer udata) {
    GstBuffer* buf = GST_PAD_PROBE_INFO_BUFFER(info);
    if (buf && GST_BUFFER_FLAG_IS_SET(buf, GST_BUFFER_FLAG_HEADER))
        return GST_PAD_PROBE_OK; // SPS/PPS — timing irrelevant

    auto* self = static_cast<CapturePipeline*>(udata);
    const uint64_t now = ds_mono_us();
    std::lock_guard<std::mutex> lk(self->m_timing_mtx);
    if (self->m_enc_in_ts != 0) {
        const uint64_t lat = now - self->m_enc_in_ts;
        if (lat < 2000000ULL) self->m_enc_stats.record(lat);
        self->m_enc_in_ts = 0;
    }
    self->m_enc_out_ts = now;
    return GST_PAD_PROBE_OK;
}

// vpay src pad → compute RTP packetise latency (throttled logging every 1 s)
GstPadProbeReturn CapturePipeline::pay_src_probe_cb(GstPad*, GstPadProbeInfo* info,
                                                    gpointer udata) {
    GstBuffer* buf = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!buf || !GST_BUFFER_PTS_IS_VALID(buf)) return GST_PAD_PROBE_OK;

    auto* self = static_cast<CapturePipeline*>(udata);
    const uint64_t now = ds_mono_us();
    std::lock_guard<std::mutex> lk(self->m_timing_mtx);
    if (self->m_enc_out_ts != 0) {
        const uint64_t lat = now - self->m_enc_out_ts;
        if (lat < 2000000ULL) self->m_pay_stats.record(lat);
        self->m_enc_out_ts = 0;
    }
    return GST_PAD_PROBE_OK;
}

// ---------------------------------------------------------------------------
// print_timing_stats — called by 1-second GLib timer
// ---------------------------------------------------------------------------
void CapturePipeline::print_timing_stats() {
    LatencyStats enc_snap, pay_snap;
    LatencyStats trk_snap[kMaxTrackers];
    bool have_enc = false, have_pay = false;
    bool have_trk[kMaxTrackers]{};
    {
        std::lock_guard<std::mutex> lk(m_timing_mtx);
        if (!m_enc_stats.empty()) { enc_snap = m_enc_stats.reset_and_return(); have_enc = true; }
        if (!m_pay_stats.empty()) { pay_snap = m_pay_stats.reset_and_return(); have_pay = true; }
        for (int i = 0; i < kMaxTrackers; ++i) {
            if (!m_tracker_stats[i].empty()) {
                trk_snap[i]  = m_tracker_stats[i].reset_and_return();
                have_trk[i]  = true;
            }
        }
    }

    if (have_enc)
        DS_INFO("[TIMING-SRC] encode:    cnt=%" PRIu64
                " avg=%.0f min=%" PRIu64 " max=%" PRIu64 " us\n",
                enc_snap.count, enc_snap.avg_us(),
                enc_snap.min_us, enc_snap.max_us);

    if (have_pay)
        DS_INFO("[TIMING-SRC] rtp_pay:   cnt=%" PRIu64
                " avg=%.0f min=%" PRIu64 " max=%" PRIu64 " us\n",
                pay_snap.count, pay_snap.avg_us(),
                pay_snap.min_us, pay_snap.max_us);

    for (size_t i = 0; i < m_trackers.size() && i < kMaxTrackers; ++i) {
        if (have_trk[i])
            DS_INFO("[TIMING-SRC] tracker[%s]: cnt=%" PRIu64
                    " avg=%.0f min=%" PRIu64 " max=%" PRIu64 " us\n",
                    m_trackers[i]->name(),
                    trk_snap[i].count, trk_snap[i].avg_us(),
                    trk_snap[i].min_us, trk_snap[i].max_us);
    }
}

// ---------------------------------------------------------------------------
// gst_thread_func — runs GLib main loop; registers stats timer
// ---------------------------------------------------------------------------
void CapturePipeline::gst_thread_func() {
    if (m_cfg.debug.stats_interval_s > 0) {
        m_timing_timer_id = g_timeout_add_seconds(m_cfg.debug.stats_interval_s,
            [](gpointer data) -> gboolean {
                static_cast<CapturePipeline*>(data)->print_timing_stats();
                return G_SOURCE_CONTINUE;
            }, this);
    }
    g_main_loop_run(m_loop);
}

// ---------------------------------------------------------------------------
// start
// ---------------------------------------------------------------------------
bool CapturePipeline::start() {
    if (m_running.load()) return true;

    const std::string desc = build_pipeline_str();
    DS_INFO("CapturePipeline:\n  %s\n", desc.c_str());

    GError* err = nullptr;
    m_pipeline = gst_parse_launch(desc.c_str(), &err);
    if (!m_pipeline || err) {
        DS_ERR("CapturePipeline: parse error: %s\n", err ? err->message : "unknown");
        if (err) g_error_free(err);
        return false;
    }

    // Grab appsink and connect callback
    m_appsink = gst_bin_get_by_name(GST_BIN(m_pipeline), "vsink");
    if (!m_appsink) {
        DS_ERR("CapturePipeline: appsink 'vsink' not found\n");
        gst_object_unref(m_pipeline); m_pipeline = nullptr;
        return false;
    }
    GstAppSinkCallbacks cbs{};
    cbs.new_sample = on_new_sample;
    gst_app_sink_set_callbacks(GST_APP_SINK(m_appsink), &cbs, this, nullptr);

    // Attach timing probes only when stats are enabled
    if (m_cfg.debug.stats_interval_s > 0) {
        m_encoder   = gst_bin_get_by_name(GST_BIN(m_pipeline), "venc");
        m_rtph264pay = gst_bin_get_by_name(GST_BIN(m_pipeline), "vpay");

        if (m_encoder) {
            GstPad* snk = gst_element_get_static_pad(m_encoder, "sink");
            GstPad* src = gst_element_get_static_pad(m_encoder, "src");
            if (snk) {
                m_enc_sink_probe_id = gst_pad_add_probe(snk,
                    GST_PAD_PROBE_TYPE_BUFFER, enc_sink_probe_cb, this, nullptr);
                gst_object_unref(snk);
            }
            if (src) {
                m_enc_src_probe_id = gst_pad_add_probe(src,
                    GST_PAD_PROBE_TYPE_BUFFER, enc_src_probe_cb, this, nullptr);
                gst_object_unref(src);
            }
        }

        if (m_rtph264pay) {
            GstPad* src = gst_element_get_static_pad(m_rtph264pay, "src");
            if (src) {
                m_pay_src_probe_id = gst_pad_add_probe(src,
                    GST_PAD_PROBE_TYPE_BUFFER, pay_src_probe_cb, this, nullptr);
                gst_object_unref(src);
            }
        }
    }

    // Bus watch
    GstBus* bus = gst_element_get_bus(m_pipeline);
    gst_bus_add_watch(bus, bus_callback, this);
    gst_object_unref(bus);

    m_loop = g_main_loop_new(nullptr, FALSE);

    if (gst_element_set_state(m_pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        DS_ERR("CapturePipeline: failed to set PLAYING\n");
        gst_object_unref(m_pipeline); m_pipeline = nullptr;
        g_main_loop_unref(m_loop);    m_loop = nullptr;
        return false;
    }

    m_running.store(true);
    m_gst_thread = std::thread(&CapturePipeline::gst_thread_func, this);

    DS_INFO("CapturePipeline: started (%s %ux%u@%u/%ufps → %s:%u)\n",
            m_cfg.input.type.c_str(),
            m_cfg.input.width, m_cfg.input.height,
            m_cfg.input.fps_num, m_cfg.input.fps_den,
            m_cfg.transport.host.c_str(), m_cfg.transport.rtp_port);
    return true;
}

// ---------------------------------------------------------------------------
// stop
// ---------------------------------------------------------------------------
void CapturePipeline::stop() {
    if (!m_pipeline) return;
    m_running.store(false);

    // Remove probes before transitioning to NULL to avoid use-after-free
    auto remove_probe = [](GstElement* elem, const char* pad_name, gulong& probe_id) {
        if (elem && probe_id) {
            GstPad* pad = gst_element_get_static_pad(elem, pad_name);
            if (pad) { gst_pad_remove_probe(pad, probe_id); gst_object_unref(pad); }
            probe_id = 0;
        }
    };
    remove_probe(m_encoder,    "sink", m_enc_sink_probe_id);
    remove_probe(m_encoder,    "src",  m_enc_src_probe_id);
    remove_probe(m_rtph264pay, "src",  m_pay_src_probe_id);

    if (m_timing_timer_id) {
        g_source_remove(m_timing_timer_id);
        m_timing_timer_id = 0;
    }

    gst_element_set_state(m_pipeline, GST_STATE_NULL);

    if (m_loop) g_main_loop_quit(m_loop);
    if (m_gst_thread.joinable()) m_gst_thread.join();

    if (m_encoder)    { gst_object_unref(m_encoder);    m_encoder    = nullptr; }
    if (m_rtph264pay) { gst_object_unref(m_rtph264pay); m_rtph264pay = nullptr; }
    if (m_appsink)    { gst_object_unref(m_appsink);    m_appsink    = nullptr; }
    gst_object_unref(m_pipeline); m_pipeline = nullptr;
    if (m_loop) { g_main_loop_unref(m_loop); m_loop = nullptr; }

    DS_INFO("CapturePipeline: stopped\n");
}
