/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#include "stream_decoder.h"
#include "../common/logger.h"
#include "../common/time_utils.h"

#include <sstream>
#include <cinttypes>
#include <cstring>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>

// ---------------------------------------------------------------------------
StreamDecoder::StreamDecoder(const DsRenderConfig& cfg) : m_cfg(cfg), m_hw_decode(cfg.codec.hw_decode) {}

StreamDecoder::~StreamDecoder() { stop(); }

// ---------------------------------------------------------------------------
// build_pipeline_str
// ---------------------------------------------------------------------------
std::string StreamDecoder::build_pipeline_str() const {
    std::ostringstream ss;

    const auto& tr = m_cfg.transport;

    ss << "udpsrc port=" << tr.rtp_port;

    if (tr.multicast && !tr.host.empty()) {
        ss << " multicast-group=" << tr.host
           << " auto-multicast=true";
        if (!tr.iface.empty())
            ss << " multicast-iface=" << tr.iface;
    }

    ss << " buffer-size=2097152"
       // Caps as an inline capsfilter element — more reliable than the caps=""
       // property in gst_parse_launch (avoids quoting/comma parsing ambiguity).
       << " ! application/x-rtp,media=video,clock-rate=90000,"
             "encoding-name=H264,payload=96";

    // rtpjitterbuffer: minimal latency (20ms default) since source is live
    ss << " ! rtpjitterbuffer latency=" << m_cfg.jitter_buffer_ms
       << " drop-on-latency=true"
       << " ! rtph264depay"
       << " ! h264parse name=vparser";

    if (m_hw_decode) {
        ss << " ! vaapih264dec name=vdec";
    } else {
        ss << " ! avdec_h264 name=vdec max-threads=4 output-corrupt=false";
    }

    ss << " ! videoconvert"
       << " ! video/x-raw,format=RGBA"
       // sync=false: don't throttle to GStreamer clock; render as fast as possible
       // drop=true + max-buffers=1: always show the newest decoded frame
       << " ! appsink name=vsink sync=false max-buffers=1 drop=true emit-signals=true";

    return ss.str();
}

// ---------------------------------------------------------------------------
// on_new_sample — appsink callback
// ---------------------------------------------------------------------------
GstFlowReturn StreamDecoder::on_new_sample(GstAppSink* sink, gpointer data) {
    auto* self = static_cast<StreamDecoder*>(data);

    GstSample* sample = gst_app_sink_pull_sample(sink);
    if (!sample) return GST_FLOW_ERROR;

    GstBuffer* buf  = gst_sample_get_buffer(sample);
    GstCaps*   caps = gst_sample_get_caps(sample);

    if (!buf || !caps) {
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

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

    auto frame = std::make_shared<DecodedFrame>();
    frame->width  = vinfo.width;
    frame->height = vinfo.height;
    frame->stride = static_cast<uint32_t>(vinfo.stride[0]);

    const size_t expected = static_cast<size_t>(frame->stride) * frame->height;
    frame->data.resize(expected);
    if (map.size >= expected)
        memcpy(frame->data.data(), map.data, expected);

    gst_buffer_unmap(buf, &map);
    gst_sample_unref(sample);

    ++self->m_frames_decoded;

    if (self->m_frame_cb)
        self->m_frame_cb(frame);

    return GST_FLOW_OK;
}

// ---------------------------------------------------------------------------
// bus_callback
// ---------------------------------------------------------------------------
gboolean StreamDecoder::bus_callback(GstBus* /*bus*/, GstMessage* msg, gpointer data) {
    auto* self = static_cast<StreamDecoder*>(data);
    switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_EOS:
        DS_INFO("StreamDecoder: pipeline EOS\n");
        break;
    case GST_MESSAGE_ERROR: {
        GError* err = nullptr; gchar* dbg = nullptr;
        gst_message_parse_error(msg, &err, &dbg);
        DS_ERR("StreamDecoder: error: %s (%s)\n", err->message, dbg ? dbg : "");
        g_error_free(err); g_free(dbg);
        self->m_running.store(false);
        g_main_loop_quit(self->m_loop);
        break;
    }
    case GST_MESSAGE_WARNING: {
        GError* err = nullptr; gchar* dbg = nullptr;
        gst_message_parse_warning(msg, &err, &dbg);
        DS_WARN("StreamDecoder: warning: %s\n", err->message);
        g_error_free(err); g_free(dbg);
        break;
    }
    default: break;
    }
    return TRUE;
}

// ---------------------------------------------------------------------------
// Timing probes
// ---------------------------------------------------------------------------

// h264parse src pad — record pre-decode entry time (overwrite scalar)
GstPadProbeReturn StreamDecoder::pre_dec_probe_cb(GstPad*, GstPadProbeInfo*,
                                                  gpointer udata) {
    auto* self = static_cast<StreamDecoder*>(udata);
    const uint64_t ts = ds_mono_us();
    std::lock_guard<std::mutex> lk(self->m_timing_mtx);
    self->m_pre_dec_ts = ts;
    return GST_PAD_PROBE_OK;
}

// vdec sink pad — record decoder entry time
GstPadProbeReturn StreamDecoder::vdec_sink_probe_cb(GstPad*, GstPadProbeInfo*,
                                                    gpointer udata) {
    auto* self = static_cast<StreamDecoder*>(udata);
    const uint64_t ts = ds_mono_us();
    std::lock_guard<std::mutex> lk(self->m_timing_mtx);
    self->m_vdec_sink_ts = ts;
    return GST_PAD_PROBE_OK;
}

// vdec src pad — compute pure decode latency
GstPadProbeReturn StreamDecoder::vdec_src_probe_cb(GstPad*, GstPadProbeInfo*,
                                                   gpointer udata) {
    auto* self = static_cast<StreamDecoder*>(udata);
    const uint64_t now = ds_mono_us();
    std::lock_guard<std::mutex> lk(self->m_timing_mtx);
    if (self->m_vdec_sink_ts != 0) {
        const uint64_t lat = now - self->m_vdec_sink_ts;
        if (lat < 2000000ULL) self->m_pure_dec_stats.record(lat);
        self->m_vdec_sink_ts = 0;
    }
    return GST_PAD_PROBE_OK;
}

// appsink sink pad — compute total decode latency (pre_dec → appsink entry)
GstPadProbeReturn StreamDecoder::appsink_probe_cb(GstPad*, GstPadProbeInfo*,
                                                  gpointer udata) {
    auto* self = static_cast<StreamDecoder*>(udata);
    const uint64_t now = ds_mono_us();
    std::lock_guard<std::mutex> lk(self->m_timing_mtx);
    if (self->m_pre_dec_ts != 0) {
        const uint64_t lat = now - self->m_pre_dec_ts;
        if (lat < 2000000ULL) self->m_total_dec_stats.record(lat);
        self->m_pre_dec_ts = 0;
    }
    return GST_PAD_PROBE_OK;
}

// ---------------------------------------------------------------------------
// print_timing_stats (called by GLib timer)
// ---------------------------------------------------------------------------
void StreamDecoder::print_timing_stats() {
    LatencyStats pure_snap, total_snap;
    bool have_pure = false, have_total = false;
    {
        std::lock_guard<std::mutex> lk(m_timing_mtx);
        if (!m_pure_dec_stats.empty())  { pure_snap  = m_pure_dec_stats.reset_and_return();  have_pure  = true; }
        if (!m_total_dec_stats.empty()) { total_snap = m_total_dec_stats.reset_and_return(); have_total = true; }
    }
    if (have_pure)
        DS_INFO("[TIMING-RECV] decode_pure:  cnt=%" PRIu64
                " avg=%.0f min=%" PRIu64 " max=%" PRIu64 " us\n",
                pure_snap.count, pure_snap.avg_us(),
                pure_snap.min_us, pure_snap.max_us);
    if (have_total)
        DS_INFO("[TIMING-RECV] decode_total: cnt=%" PRIu64
                " avg=%.0f min=%" PRIu64 " max=%" PRIu64 " us\n",
                total_snap.count, total_snap.avg_us(),
                total_snap.min_us, total_snap.max_us);
}

// ---------------------------------------------------------------------------
// gst_thread_func
// ---------------------------------------------------------------------------
void StreamDecoder::gst_thread_func() {
    if (m_cfg.debug.stats_interval_s > 0) {
        m_timing_timer_id = g_timeout_add_seconds(m_cfg.debug.stats_interval_s,
            [](gpointer data) -> gboolean {
                static_cast<StreamDecoder*>(data)->print_timing_stats();
                return G_SOURCE_CONTINUE;
            }, this);
    }
    g_main_loop_run(m_loop);
}

// ---------------------------------------------------------------------------
// start
// ---------------------------------------------------------------------------
bool StreamDecoder::start() {
    if (m_running.load()) return true;

    // Runtime VAAPI probe: instantiate vaapih264dec and try to reach READY state.
    // Simply finding the factory does not guarantee the GPU/driver is functional;
    // only a READY transition confirms actual hardware availability.
    m_hw_decode = m_cfg.codec.hw_decode;
    if (m_hw_decode) {
        GstElement* probe = gst_element_factory_make("vaapih264dec", "vaapi_probe");
        if (!probe) {
            DS_WARN("StreamDecoder: vaapih264dec factory missing — falling back to avdec_h264\n");
            m_hw_decode = false;
        } else {
            GstStateChangeReturn ret = gst_element_set_state(probe, GST_STATE_READY);
            gst_element_set_state(probe, GST_STATE_NULL);
            gst_object_unref(probe);
            if (ret == GST_STATE_CHANGE_FAILURE) {
                DS_WARN("StreamDecoder: vaapih264dec failed to initialise (no GPU/driver?) — falling back to avdec_h264\n");
                m_hw_decode = false;
            } else {
                DS_INFO("StreamDecoder: vaapih264dec runtime probe OK — using hardware decode\n");
            }
        }
    }

    const std::string desc = build_pipeline_str();
    DS_INFO("StreamDecoder pipeline:\n  %s\n", desc.c_str());

    GError* err = nullptr;
    m_pipeline = gst_parse_launch(desc.c_str(), &err);
    if (!m_pipeline || err) {
        DS_ERR("StreamDecoder: parse error: %s\n", err ? err->message : "unknown");
        if (err) g_error_free(err);
        return false;
    }

    // Appsink callback
    m_appsink = gst_bin_get_by_name(GST_BIN(m_pipeline), "vsink");
    if (!m_appsink) {
        DS_ERR("StreamDecoder: appsink 'vsink' not found\n");
        gst_object_unref(m_pipeline); m_pipeline = nullptr;
        return false;
    }
    GstAppSinkCallbacks cbs{};
    cbs.new_sample = on_new_sample;
    gst_app_sink_set_callbacks(GST_APP_SINK(m_appsink), &cbs, this, nullptr);

    // Attach timing probes
    if (m_cfg.debug.stats_interval_s > 0) {
        m_h264parse = gst_bin_get_by_name(GST_BIN(m_pipeline), "vparser");
        m_vdec      = gst_bin_get_by_name(GST_BIN(m_pipeline), "vdec");

        auto attach_probe = [](GstElement* elem, const char* pad_name,
                               GstPadProbeType type, GstPadProbeCallback cb,
                               gpointer data) -> gulong {
            if (!elem) return 0;
            GstPad* pad = gst_element_get_static_pad(elem, pad_name);
            if (!pad) return 0;
            gulong id = gst_pad_add_probe(pad, type, cb, data, nullptr);
            gst_object_unref(pad);
            return id;
        };

        m_pre_dec_probe_id   = attach_probe(m_h264parse, "src",
                               GST_PAD_PROBE_TYPE_BUFFER, pre_dec_probe_cb,  this);
        m_vdec_sink_probe_id = attach_probe(m_vdec,      "sink",
                               GST_PAD_PROBE_TYPE_BUFFER, vdec_sink_probe_cb, this);
        m_vdec_src_probe_id  = attach_probe(m_vdec,      "src",
                               GST_PAD_PROBE_TYPE_BUFFER, vdec_src_probe_cb,  this);
        m_appsink_probe_id   = attach_probe(m_appsink,   "sink",
                               GST_PAD_PROBE_TYPE_BUFFER, appsink_probe_cb,   this);
    }

    // Bus watch
    GstBus* bus = gst_element_get_bus(m_pipeline);
    gst_bus_add_watch(bus, bus_callback, this);
    gst_object_unref(bus);

    m_loop = g_main_loop_new(nullptr, FALSE);

    if (gst_element_set_state(m_pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        DS_ERR("StreamDecoder: failed to set PLAYING\n");
        gst_object_unref(m_pipeline); m_pipeline = nullptr;
        g_main_loop_unref(m_loop);    m_loop = nullptr;
        return false;
    }

    m_running.store(true);
    m_gst_thread = std::thread(&StreamDecoder::gst_thread_func, this);

    DS_INFO("StreamDecoder: started, listening on port %u\n", m_cfg.transport.rtp_port);
    return true;
}

// ---------------------------------------------------------------------------
// stop
// ---------------------------------------------------------------------------
void StreamDecoder::stop() {
    if (!m_pipeline) return;
    m_running.store(false);

    auto remove_probe = [](GstElement* elem, const char* pad_name, gulong& probe_id) {
        if (elem && probe_id) {
            GstPad* pad = gst_element_get_static_pad(elem, pad_name);
            if (pad) { gst_pad_remove_probe(pad, probe_id); gst_object_unref(pad); }
            probe_id = 0;
        }
    };
    remove_probe(m_h264parse, "src",  m_pre_dec_probe_id);
    remove_probe(m_vdec,      "sink", m_vdec_sink_probe_id);
    remove_probe(m_vdec,      "src",  m_vdec_src_probe_id);
    remove_probe(m_appsink,   "sink", m_appsink_probe_id);

    if (m_timing_timer_id) { g_source_remove(m_timing_timer_id); m_timing_timer_id = 0; }

    gst_element_set_state(m_pipeline, GST_STATE_NULL);
    if (m_loop) g_main_loop_quit(m_loop);
    if (m_gst_thread.joinable()) m_gst_thread.join();

    if (m_h264parse) { gst_object_unref(m_h264parse); m_h264parse = nullptr; }
    if (m_vdec)      { gst_object_unref(m_vdec);      m_vdec      = nullptr; }
    if (m_appsink)   { gst_object_unref(m_appsink);   m_appsink   = nullptr; }
    gst_object_unref(m_pipeline); m_pipeline = nullptr;
    if (m_loop) { g_main_loop_unref(m_loop); m_loop = nullptr; }

    DS_INFO("StreamDecoder: stopped (frames decoded: %" PRIu64 ")\n",
            m_frames_decoded.load());
}
