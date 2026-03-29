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
#include <vector>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>

// ---------------------------------------------------------------------------
// SEI extraction helpers
// ---------------------------------------------------------------------------

// Must match kVbSeiUUID in source/capture_pipeline.cpp
static const uint8_t kVbSeiUUID[16] = {
    0xB5, 0x58, 0x0A, 0xB2,  0x1C, 0x7D, 0x4E, 0xF6,
    0x81, 0x93, 0xA4, 0xC3,  0xD7, 0xE5, 0xF2, 0x9B
};

// EBSP → RBSP: remove emulation-prevention bytes (00 00 03 → 00 00)
static void ebsp_decode(const uint8_t* src, size_t len,
                        std::vector<uint8_t>& out) {
    out.clear();
    out.reserve(len);
    for (size_t i = 0; i < len; ) {
        if (i + 2 < len &&
            src[i] == 0x00 && src[i+1] == 0x00 && src[i+2] == 0x03) {
            out.push_back(0x00);
            out.push_back(0x00);
            i += 3; // skip the 0x03 emulation-prevention byte
        } else {
            out.push_back(src[i++]);
        }
    }
}

// Scan an Annex-B byte-stream buffer for a VisionBridge SEI NAL.
// Returns true and fills ts_out / seq_out when found.
static bool sei_parse_nal(const uint8_t* data, size_t size,
                           uint64_t& ts_out, uint64_t& seq_out) {
    if (size < 39) return false;
    for (size_t i = 0; i + 4 <= size; ) {
        // Scan for Annex-B start code (4-byte preferred, 3-byte fallback)
        if (data[i] == 0x00 && data[i+1] == 0x00) {
            if (i + 3 < size && data[i+2] == 0x00 && data[i+3] == 0x01) {
                i += 4;
            } else if (data[i+2] == 0x01) {
                i += 3;
            } else { ++i; continue; }
        } else { ++i; continue; }

        if (i >= size) break;
        const uint8_t nal_type = data[i] & 0x1F;
        ++i;
        if (nal_type != 6) continue; // not SEI

        // Locate end of this NAL (next start code or end of buffer)
        size_t nal_end = size;
        for (size_t j = i; j + 2 < size; ++j) {
            if (data[j] == 0x00 && data[j+1] == 0x00 &&
                (data[j+2] == 0x01 ||
                 (j + 3 < size && data[j+2] == 0x00 && data[j+3] == 0x01)))
            { nal_end = j; break; }
        }

        // EBSP → RBSP
        std::vector<uint8_t> rbsp;
        ebsp_decode(data + i, nal_end - i, rbsp);

        // Parse SEI messages in RBSP
        size_t pos = 0;
        while (pos < rbsp.size()) {
            if (rbsp[pos] != 0x05) break; // only user_data_unregistered
            ++pos;
            size_t payload_size = 0;
            while (pos < rbsp.size() && rbsp[pos] == 0xFF)
                { payload_size += 255; ++pos; }
            if (pos >= rbsp.size()) break;
            payload_size += rbsp[pos++];

            if (payload_size >= 32 && pos + payload_size <= rbsp.size() &&
                std::memcmp(rbsp.data() + pos, kVbSeiUUID, 16) == 0) {
                std::memcpy(&ts_out,  rbsp.data() + pos + 16, 8);
                std::memcpy(&seq_out, rbsp.data() + pos + 24, 8);
                return true;
            }
            pos += payload_size;
        }
    }
    return false;
}

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
       << " ! rtph264depay name=vdepay"
       << " ! h264parse name=vparser";

    if (m_hw_decode) {
        ss << " ! vaapih264dec name=vdec";
    } else {
        ss << " ! avdec_h264 name=vdec max-threads=4 output-corrupt=false";
    }

    ss << " ! videoconvert"
       << " ! video/x-raw,format=RGBA"
       // sync=false: deliver frames free-run as fast as decoded; frame rate is capped
       // in the render loop by display_fps.  sync=true would pace to the GStreamer clock
       // and risk misaligning decoded frames with ZMQ bbox messages.
       << " ! appsink name=vsink sync=false"
       << " max-buffers=1 drop=true emit-signals=true";

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

    // Stamp frame with SEI data matched by PTS (precise per-frame correlation)
    if (GST_BUFFER_PTS_IS_VALID(buf)) {
        const GstClockTime pts = GST_BUFFER_PTS(buf);
        std::lock_guard<std::mutex> lk(self->m_sei_mtx);
        for (auto& e : self->m_sei_map) {
            if (e.pts == pts && e.capture_ts_us != 0) {
                frame->sei_capture_ts_us = e.capture_ts_us;
                frame->sei_frame_seq     = e.frame_seq;
                e.capture_ts_us = 0; // consume
                break;
            }
        }
    }

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

// rtph264depay src pad — scan Annex-B output for a VisionBridge SEI NAL and cache result.
// Fires before the buffer enters h264parse / vdec; the raw depacketized byte-stream
// still contains the injected SEI.
GstPadProbeReturn StreamDecoder::sei_extract_probe_cb(GstPad*, GstPadProbeInfo* info,
                                                      gpointer udata) {
    auto* self = static_cast<StreamDecoder*>(udata);
    GstBuffer* buf = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!buf) return GST_PAD_PROBE_OK;

    GstMapInfo map;
    if (!gst_buffer_map(buf, &map, GST_MAP_READ))
        return GST_PAD_PROBE_OK;

    uint64_t ts = 0, seq = 0;
    const bool found = sei_parse_nal(map.data, map.size, ts, seq);
    gst_buffer_unmap(buf, &map);

    if (found) {
        if (!GST_BUFFER_PTS_IS_VALID(buf)) {
            // No PTS — fall back to overwriting latest (best effort)
            std::lock_guard<std::mutex> lk(self->m_sei_mtx);
            self->m_sei_map[self->m_sei_map_head % kSeiMapSize] = {
                GST_CLOCK_TIME_NONE, ts, seq };
            ++self->m_sei_map_head;
        } else {
            const GstClockTime pts = GST_BUFFER_PTS(buf);
            std::lock_guard<std::mutex> lk(self->m_sei_mtx);
            auto& slot = self->m_sei_map[self->m_sei_map_head % kSeiMapSize];
            slot.pts           = pts;
            slot.capture_ts_us = ts;
            slot.frame_seq     = seq;
            ++self->m_sei_map_head;
        }
        DS_TRACE("StreamDecoder: SEI capture_ts=%" PRIu64 " seq=%" PRIu64 "\n",
                 ts, seq);
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

    // Attach SEI extract probe on h264parse src — at this point h264parse has
    // assigned the final PTS that the decoder preserves through to the appsink,
    // so PTS-keyed lookup in on_new_sample gives exact per-frame correlation.
    // We always need h264parse, so grab it here regardless of stats.
    if (!m_h264parse)
        m_h264parse = gst_bin_get_by_name(GST_BIN(m_pipeline), "vparser");
    if (m_h264parse) {
        GstPad* src = gst_element_get_static_pad(m_h264parse, "src");
        if (src) {
            m_sei_extract_probe_id = gst_pad_add_probe(src,
                GST_PAD_PROBE_TYPE_BUFFER, sei_extract_probe_cb, this, nullptr);
            gst_object_unref(src);
        }
    } else {
        DS_WARN("StreamDecoder: h264parse 'vparser' not found — SEI extraction disabled\n");
    }

    // Attach timing probes
    if (m_cfg.debug.stats_interval_s > 0) {
        if (!m_h264parse)
            m_h264parse = gst_bin_get_by_name(GST_BIN(m_pipeline), "vparser");
        m_vdec = gst_bin_get_by_name(GST_BIN(m_pipeline), "vdec");

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
    remove_probe(m_h264parse, "src",  m_sei_extract_probe_id);
    remove_probe(m_h264parse, "src",  m_pre_dec_probe_id);
    remove_probe(m_vdec,      "sink", m_vdec_sink_probe_id);
    remove_probe(m_vdec,      "src",  m_vdec_src_probe_id);
    remove_probe(m_appsink,   "sink", m_appsink_probe_id);

    if (m_timing_timer_id) { g_source_remove(m_timing_timer_id); m_timing_timer_id = 0; }

    gst_element_set_state(m_pipeline, GST_STATE_NULL);
    if (m_loop) g_main_loop_quit(m_loop);
    if (m_gst_thread.joinable()) m_gst_thread.join();

    if (m_vdepay)    { gst_object_unref(m_vdepay);    m_vdepay    = nullptr; }
    if (m_h264parse) { gst_object_unref(m_h264parse); m_h264parse = nullptr; }
    if (m_vdec)      { gst_object_unref(m_vdec);      m_vdec      = nullptr; }
    if (m_appsink)   { gst_object_unref(m_appsink);   m_appsink   = nullptr; }
    gst_object_unref(m_pipeline); m_pipeline = nullptr;
    if (m_loop) { g_main_loop_unref(m_loop); m_loop = nullptr; }

    DS_INFO("StreamDecoder: stopped (frames decoded: %" PRIu64 ")\n",
            m_frames_decoded.load());
}
