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
#include <vector>
#include <unistd.h>
#include <opencv2/core.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>

// ---------------------------------------------------------------------------
// SEI injection helpers
// ---------------------------------------------------------------------------

// VisionBridge SEI UUID (H.264 user_data_unregistered, payloadType=5).
// All bytes non-zero to minimise EBSP emulation-prevention insertions.
// Must match kVbSeiUUID in render/stream_decoder.cpp.
static const uint8_t kVbSeiUUID[16] = {
    0xB5, 0x58, 0x0A, 0xB2,  0x1C, 0x7D, 0x4E, 0xF6,
    0x81, 0x93, 0xA4, 0xC3,  0xD7, 0xE5, 0xF2, 0x9B
};

// Build an Annex-B SEI NAL carrying {capture_ts_us, frame_seq}.
// Wire layout: [00 00 00 01][06] EBSP{ 05 20 <UUID 16B> <ts 8B LE> <seq 8B LE> 80 }
static std::vector<uint8_t> sei_build_nal(uint64_t capture_ts_us,
                                          uint64_t frame_seq) {
    // Raw RBSP (before EBSP encoding):
    //   payloadType(1) + payloadSize(1) + UUID(16) + ts(8) + seq(8) + stop_bit(1) = 35
    uint8_t rbsp[35];
    rbsp[0] = 0x05;                          // user_data_unregistered
    rbsp[1] = 0x20;                          // payload size = 32 bytes
    std::memcpy(rbsp +  2, kVbSeiUUID,     16);
    std::memcpy(rbsp + 18, &capture_ts_us,  8);
    std::memcpy(rbsp + 26, &frame_seq,      8);
    rbsp[34] = 0x80;                         // RBSP trailing stop bit

    // EBSP encode: insert 0x03 after any "00 00" preceding 0x00–0x03
    std::vector<uint8_t> ebsp;
    ebsp.reserve(40);
    int zeros = 0;
    for (int i = 0; i < 35; ++i) {
        if (zeros == 2 && rbsp[i] <= 0x03)
            { ebsp.push_back(0x03); zeros = 0; }
        ebsp.push_back(rbsp[i]);
        zeros = (rbsp[i] == 0x00) ? zeros + 1 : 0;
    }

    // Prepend Annex-B start code + SEI NAL header (nal_unit_type=6)
    std::vector<uint8_t> nal;
    nal.reserve(5 + ebsp.size());
    nal.push_back(0x00); nal.push_back(0x00);
    nal.push_back(0x00); nal.push_back(0x01);
    nal.push_back(0x06);
    nal.insert(nal.end(), ebsp.begin(), ebsp.end());
    return nal;
}

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
        if (co.hw_decode) {
            ss << "filesrc name=vfsrc location=\"" << in.file << "\""
               << " ! qtdemux name=demux"
               << "  demux.video_0 ! h264parse ! vah264dec";
        } else {
            ss << "filesrc name=vfsrc location=\"" << in.file << "\""
               << " ! decodebin";
        }
    } else {
        ss << "v4l2src device=\"" << in.device << "\"";
    }

    // ----- Common format conversion & scaling -----
    ss << " ! videoconvert name=vconv";
    if (in.width > 0 || in.height > 0)
        ss << " ! videoscale";
    if (in.fps_num > 0)
        ss << " ! videorate";
    // BGRx: 4-byte-aligned BGR (OpenCV reads first 3 channels directly)
    ss << " ! video/x-raw,format=BGRx";
    if (in.width  > 0) ss << ",width="  << in.width;
    if (in.height > 0) ss << ",height=" << in.height;
    if (in.fps_num > 0) ss << ",framerate=" << in.fps_num << "/" << in.fps_den;

    // ----- Single linear encode path (no tee, no appsink) -----
    // venc_q sink pad is probed by vraw_probe_cb: memcpy BGRx frame → shared
    // FrameSlot, assign seq#, notify executor thread — all in <1 ms so the
    // streaming thread (and encoder) are not blocked by inference.
    if (co.hw_encode) {
        ss << " ! queue name=venc_q leaky=downstream max-size-buffers=2"
              "         max-size-bytes=0 max-size-time=0"
           << " ! videoconvert"
           << " ! video/x-raw,format=NV12"
           << " ! vaapih264enc name=venc rate-control=cbr"
           << " bitrate="         << co.bitrate_kbps
           << " keyframe-period=" << co.keyint
           << " max-bframes=0 tune=low-power"
           << " ! video/x-h264,stream-format=byte-stream";
    } else {
        ss << " ! queue name=venc_q max-size-buffers=10"
              "         max-size-bytes=0 max-size-time=0"
           << " ! videoconvert"
           << " ! video/x-raw,format=I420"
           << " ! x264enc name=venc tune=zerolatency"
           << " bitrate="     << co.bitrate_kbps
           << " key-int-max=" << co.keyint
           << " bframes=0 speed-preset=ultrafast"
           << " ! video/x-h264,stream-format=byte-stream";
    }

    ss << " ! h264parse name=vsparse"
       << " ! rtph264pay name=vpay config-interval=-1"
          " ssrc=987654321 pt=96 mtu=1400";

    if (tr.multicast) {
        ss << " ! udpsink host=\"" << tr.host << "\""
           << " port=" << tr.rtp_port
           << " auto-multicast=true sync=true";
        if (!tr.iface.empty())
            ss << " multicast-iface=\"" << tr.iface << "\"";
    } else {
        ss << " ! udpsink host=\"" << tr.host << "\""
           << " port=" << tr.rtp_port
           << " sync=true";
    }

    return ss.str();
}

// ---------------------------------------------------------------------------
// vraw_probe_cb — venc_q sink pad probe (GStreamer streaming thread)
//
// Fires for every BGRx frame entering the encoder queue.  Does three things:
//   1. Assigns a seq# and records capture_ts (real-time clock).
//   2. Populates the SEI map (PTS → {capture_ts, seq}) for sei_inject_probe_cb.
//   3. memcpy's the BGRx pixels into m_frame_slot (latest-wins) and signals
//      the executor thread to run trackers in parallel with the encoder.
//
// Returns immediately — the encoder is never blocked by tracker inference.
// ---------------------------------------------------------------------------
GstPadProbeReturn CapturePipeline::vraw_probe_cb(GstPad* pad,
                                                  GstPadProbeInfo* info,
                                                  gpointer udata) {
    auto* self = static_cast<CapturePipeline*>(udata);

    GstBuffer* buf = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!buf) return GST_PAD_PROBE_OK;

    const uint64_t capture_ts = ds_realtime_us();
    const uint64_t now_mono   = ds_mono_us();
    const uint64_t seq = self->m_frame_seq.fetch_add(1, std::memory_order_relaxed);

    // Determine frame geometry from the pad's negotiated caps
    GstCaps* caps = gst_pad_get_current_caps(pad);
    if (!caps) return GST_PAD_PROBE_OK;
    GstVideoInfo vinfo;
    const bool caps_ok = gst_video_info_from_caps(&vinfo, caps);
    gst_caps_unref(caps);
    if (!caps_ok) return GST_PAD_PROBE_OK;

    // Populate SEI map keyed by PTS so sei_inject_probe_cb can embed timestamps
    if (GST_BUFFER_PTS_IS_VALID(buf)) {
        std::lock_guard<std::mutex> lk(self->m_sei_map_mtx);
        auto& slot = self->m_sei_map[self->m_sei_map_head % kSeiMapSize];
        slot.pts           = GST_BUFFER_PTS(buf);
        slot.capture_ts_us = capture_ts;
        slot.frame_seq     = seq;
        ++self->m_sei_map_head;
    }

    // Seed tee_ts_map for enc_sink_probe_cb to measure venc_q wait latency
    if (self->m_cfg.debug.stats_interval_s > 0 && GST_BUFFER_PTS_IS_VALID(buf)) {
        std::lock_guard<std::mutex> lk(self->m_tee_ts_mtx);
        auto& e = self->m_tee_ts_map[self->m_tee_ts_map_head % kTeeTsMapSize];
        e.pts      = GST_BUFFER_PTS(buf);
        e.t_tee_us = now_mono;
        ++self->m_tee_ts_map_head;
    }

    // Pre-encode-chain latency: vconv sink → venc_q sink
    {
        std::lock_guard<std::mutex> lk(self->m_timing_mtx);
        if (self->m_pretee_in_ts != 0) {
            const uint64_t lat = now_mono - self->m_pretee_in_ts;
            if (lat < 2000000ULL) self->m_pretee_stats.record(lat);
            self->m_pretee_in_ts = 0;
        }
    }

    // memcpy BGRx → inference slot; publish seq atomically so inference thread
    // starts immediately in parallel with the encoder.
    GstMapInfo map;
    if (gst_buffer_map(buf, &map, GST_MAP_READ)) {
        {
            std::lock_guard<std::mutex> lk(self->m_slot_mtx);
            auto& fs        = self->m_frame_slot;
            fs.data.resize(map.size);
            std::memcpy(fs.data.data(), map.data, map.size);
            fs.width        = static_cast<uint32_t>(vinfo.width);
            fs.height       = static_cast<uint32_t>(vinfo.height);
            fs.stride       = static_cast<uint32_t>(vinfo.stride[0]);
            fs.seq          = seq;
            fs.capture_ts_us = capture_ts;
        }
        gst_buffer_unmap(buf, &map);
        self->m_slot_seq.store(seq, std::memory_order_release);
    }

    return GST_PAD_PROBE_OK;
}

// ---------------------------------------------------------------------------
// executor_thread_func — tracker/bbox thread (parallel with GStreamer encoder)
//
// Takes the latest available FrameSlot (latest-wins: if inference is slower
// than source fps it always works on the newest frame), runs enabled trackers,
// and fires the bbox callback (ZMQ send).  dummy_bbox mode fires immediately
// with bbox_count=0 — no tracker execution.
// ---------------------------------------------------------------------------
void CapturePipeline::inference_thread_func() {
    DS_INFO("CapturePipeline: executor thread started\n");

    uint64_t last_seen_seq = UINT64_MAX; // sentinel: nothing processed yet

    while (!m_executor_stop.load(std::memory_order_acquire)) {
        // Poll for a new seq at 100 µs intervals — no condvar, no queue.
        const uint64_t published = m_slot_seq.load(std::memory_order_acquire);
        if (published == last_seen_seq || published == UINT64_MAX) {
            usleep(100);
            continue;
        }

        // A new frame is available — copy it out under the slot lock.
        // We always take the newest frame in the slot (latest-wins), but we
        // tag the ZMQ message with slot.seq (not 'published') so the seq
        // matches the actual pixels used.  last_seen_seq tracks slot.seq so
        // we never re-process the same slot twice.
        FrameSlot slot;
        {
            std::lock_guard<std::mutex> lk(m_slot_mtx);
            slot = m_frame_slot;
        }
        if (slot.seq == last_seen_seq) {
            // Slot didn't advance past what we last processed — nothing new.
            usleep(100);
            continue;
        }
        last_seen_seq = slot.seq;

        DsMsgBbox msg{};
        msg.type             = DS_MSG_BBOX;
        msg.protocol_version = DS_PROTOCOL_VERSION;
        msg.frame_seq        = slot.seq;
        msg.capture_ts_us    = slot.capture_ts_us;
        msg.src_width        = slot.width;
        msg.src_height       = slot.height;
        msg.bbox_count       = 0;

        const uint32_t stats_en = m_cfg.debug.stats_interval_s;
        const uint64_t exec_t0  = ds_mono_us(); // always measured for the TRACE

        if (!m_cfg.detectors.dummy_bbox && !m_trackers.empty()) {
            cv::Mat frame(static_cast<int>(slot.height),
                          static_cast<int>(slot.width),
                          CV_8UC4,
                          slot.data.data(),
                          static_cast<size_t>(slot.stride));

            for (size_t ti = 0; ti < m_trackers.size() && ti < kMaxTrackers; ++ti) {
                const uint64_t t0 = stats_en ? ds_mono_us() : 0;

                std::vector<DsBbox> boxes = m_trackers[ti]->process(frame, slot.seq);

                if (stats_en) {
                    std::lock_guard<std::mutex> lk(m_timing_mtx);
                    m_tracker_stats[ti].record(ds_mono_us() - t0);
                }
                for (const auto& b : boxes) {
                    if (msg.bbox_count >= DS_MAX_BBOXES) break;
                    msg.bboxes[msg.bbox_count++] = b;
                }
            }
        }

        const uint64_t exec_dur = ds_mono_us() - exec_t0;

        // Record total executor duration (all trackers combined, or 0 for dummy).
        if (stats_en) {
            std::lock_guard<std::mutex> lk(m_timing_mtx);
            m_executor_stats.record(exec_dur);
        }

        if (m_bbox_cb) {
            msg.send_ts_us = ds_realtime_us();
            m_bbox_cb(msg);
        }

        // One TRACE per frame: detector names, inference duration, bbox count.
        char det_names[64] = "dummy";
        if (!m_cfg.detectors.dummy_bbox) {
            det_names[0] = '\0';
            for (size_t ti = 0; ti < m_trackers.size() && ti < kMaxTrackers; ++ti) {
                if (det_names[0]) std::strncat(det_names, "+", sizeof(det_names) - 1 - std::strlen(det_names));
                std::strncat(det_names, m_trackers[ti]->name(), sizeof(det_names) - 1 - std::strlen(det_names));
            }
            if (det_names[0] == '\0') std::strncpy(det_names, "none", sizeof(det_names) - 1);
        }
        DS_TRACE("CapturePipeline: executor  seq=%-6" PRIu64
                 "  det=%-8s  dur=%5" PRIu64 " us  bboxes=%u\n",
                 msg.frame_seq, det_names, exec_dur, msg.bbox_count);
    }

    DS_INFO("CapturePipeline: executor thread exited\n");
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
// Pad probe callbacks — pipeline latency measurement
// ---------------------------------------------------------------------------

// "vconv" (first videoconvert) sink pad — records the moment a decoded/captured
// frame enters the pre-encode processing chain (decode+colorconv+scale+rate).
GstPadProbeReturn CapturePipeline::vconv_sink_probe_cb(GstPad*, GstPadProbeInfo*,
                                                        gpointer udata) {
    auto* self = static_cast<CapturePipeline*>(udata);
    const uint64_t ts = ds_mono_us();
    std::lock_guard<std::mutex> lk(self->m_timing_mtx);
    self->m_pretee_in_ts = ts;
    return GST_PAD_PROBE_OK;
}

// vconv src pad — fires once per BGRx frame exiting videoconvert, before videoscale/
// videorate.  When stats are enabled measures the videoconvert element's own processing
// latency (vconv_sink_ts → vconv_src_ts) and records it in m_vconv_stats.
// m_pretee_in_ts is left non-zero so that vraw_probe_cb can still measure the full
// pre-encode chain latency (vconv_sink → venc_q_sink).
GstPadProbeReturn CapturePipeline::vconv_src_probe_cb(GstPad*, GstPadProbeInfo*,
                                                       gpointer udata) {
    auto* self = static_cast<CapturePipeline*>(udata);
    const uint64_t ts = ds_mono_us();

    {
        std::lock_guard<std::mutex> lk(self->m_timing_mtx);
        if (self->m_pretee_in_ts != 0) {
            const uint64_t lat = ts - self->m_pretee_in_ts;
            if (lat < 2000000ULL) self->m_vconv_stats.record(lat);
        }
    }

    return GST_PAD_PROBE_OK;
}

// filesrc src pad — fires for every compressed bitstream chunk pushed from filesrc
// into decodebin (file mode only).  One chunk ≠ one frame (decodebin buffers internally),
// so this probe provides chunk-level throughput visibility rather than per-frame timing.
GstPadProbeReturn CapturePipeline::filesrc_src_probe_cb(GstPad*, GstPadProbeInfo* info,
                                                         gpointer /*udata*/) {
    GstBuffer* buf = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!buf) return GST_PAD_PROBE_OK;

    DS_TRACE("filesrc → decodebin  offset=%" G_GUINT64_FORMAT " bytes=%" G_GSIZE_FORMAT "\n",
             GST_BUFFER_OFFSET_IS_VALID(buf) ? GST_BUFFER_OFFSET(buf) : 0ULL,
             gst_buffer_get_size(buf));
    return GST_PAD_PROBE_OK;
}

// venc sink pad — push encoder entry timestamp into FIFO; also compute encode-branch
// queue latency (venc_q → encoder entry) and clear the tee-ts map entry.
GstPadProbeReturn CapturePipeline::enc_sink_probe_cb(GstPad*, GstPadProbeInfo* info,
                                                     gpointer udata) {
    auto* self = static_cast<CapturePipeline*>(udata);
    const uint64_t ts = ds_mono_us();

    uint64_t enc_branch_lat = 0;
    uint64_t frame_seq      = 0;
    GstBuffer* buf = GST_PAD_PROBE_INFO_BUFFER(info);
    const GstClockTime pts = (buf && GST_BUFFER_PTS_IS_VALID(buf))
                             ? GST_BUFFER_PTS(buf) : GST_CLOCK_TIME_NONE;

    if (pts != GST_CLOCK_TIME_NONE) {
        // venc_q preserves PTS unchanged — lookup SEI map (read-only, don't consume)
        // to get frame_seq for the trace.
        {
            std::lock_guard<std::mutex> lk(self->m_sei_map_mtx);
            for (const auto& e : self->m_sei_map) {
                if (e.pts == pts && e.capture_ts_us != 0) {
                    frame_seq = e.frame_seq;
                    break;
                }
            }
        }
        // venc_q queue-wait latency
        {
            std::lock_guard<std::mutex> lk(self->m_tee_ts_mtx);
            for (auto& e : self->m_tee_ts_map) {
                if (e.pts == pts && e.t_tee_us != 0) {
                    const uint64_t lat = ts - e.t_tee_us;
                    if (lat < 2000000ULL) enc_branch_lat = lat;
                    e.t_tee_us = 0;
                    break;
                }
            }
        }
    }

    std::lock_guard<std::mutex> lk(self->m_timing_mtx);
    if (enc_branch_lat) self->m_enc_branch_stats.record(enc_branch_lat);
    // Push entry timestamp + seq into FIFO for enc_src_probe_cb to consume
    auto& slot    = self->m_enc_fifo[self->m_enc_fifo_head % kEncFifoSize];
    slot.ts        = ts;
    slot.frame_seq = frame_seq;
    slot.used      = true;
    ++self->m_enc_fifo_head;
    return GST_PAD_PROBE_OK;
}

// venc src pad — pop oldest enc_fifo entry, record encode duration.
// Skip HEADER (SPS/PPS) buffers — they have no corresponding enc_sink entry.
GstPadProbeReturn CapturePipeline::enc_src_probe_cb(GstPad*, GstPadProbeInfo* info,
                                                    gpointer udata) {
    GstBuffer* buf = GST_PAD_PROBE_INFO_BUFFER(info);
    if (buf && GST_BUFFER_FLAG_IS_SET(buf, GST_BUFFER_FLAG_HEADER))
        return GST_PAD_PROBE_OK;

    auto* self = static_cast<CapturePipeline*>(udata);
    const uint64_t now = ds_mono_us();

    uint64_t frame_seq = 0;
    uint64_t enc_dur   = 0;
    {
        std::lock_guard<std::mutex> lk(self->m_timing_mtx);
        for (auto& e : self->m_enc_fifo) {
            if (e.used) {
                enc_dur   = now - e.ts;
                if (enc_dur < 2000000ULL) self->m_enc_stats.record(enc_dur);
                else enc_dur = 0;
                frame_seq = e.frame_seq;
                e.used    = false;
                break;
            }
        }
    }

    DS_TRACE("CapturePipeline: encode    seq=%-6" PRIu64 "  dur=%5" PRIu64 " us\n",
             frame_seq, enc_dur);
    return GST_PAD_PROBE_OK;
}

// h264parse src pad — prepend a VisionBridge SEI NAL
// carrying {capture_ts_us, frame_seq}.  The SEI rides through RTP/UDP unchanged
// and is extracted by sei_extract_probe_cb on the render side.
GstPadProbeReturn CapturePipeline::sei_inject_probe_cb(GstPad*, GstPadProbeInfo* info,
                                                       gpointer udata) {
    GstBuffer* orig = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!orig || GST_BUFFER_FLAG_IS_SET(orig, GST_BUFFER_FLAG_HEADER))
        return GST_PAD_PROBE_OK; // skip codec header (SPS/PPS) buffers
    if (!GST_BUFFER_PTS_IS_VALID(orig))
        return GST_PAD_PROBE_OK;

    auto* self = static_cast<CapturePipeline*>(udata);
    const GstClockTime pts = GST_BUFFER_PTS(orig);

    // Look up the capture timestamp for this PTS.
    // Primary: exact PTS match — works when venc_q-sink PTS == h264parse-src PTS.
    // Fallback: if the encoder transforms PTSes internally (x264enc converts
    //   stream_time ↔ running_time, so a file with a large PTS base causes a
    //   systematic mismatch), pop the oldest unconsumed entry by frame_seq.
    //   This is safe for bframes=0 because frames are serialised in encode order;
    //   the occasional wrong assignment during a leaky-queue drop is acceptable.
    uint64_t capture_ts = 0, frame_seq = 0;
    {
        std::lock_guard<std::mutex> lk(self->m_sei_map_mtx);

        // 1. Exact PTS match (ideal path)
        for (auto& e : self->m_sei_map) {
            if (e.pts == pts && e.capture_ts_us != 0) {
                capture_ts = e.capture_ts_us;
                frame_seq  = e.frame_seq;
                e.capture_ts_us = 0; // consumed
                break;
            }
        }

        // 2. FIFO fallback: PTS domain mismatch (e.g. x264enc running-time
        //    vs stream-time transformation).  Find the oldest unconsumed entry.
        if (capture_ts == 0) {
            SeiMapEntry* oldest = nullptr;
            for (auto& e : self->m_sei_map) {
                if (e.capture_ts_us != 0) {
                    if (!oldest || e.frame_seq < oldest->frame_seq)
                        oldest = &e;
                }
            }
            if (oldest) {
                capture_ts = oldest->capture_ts_us;
                frame_seq  = oldest->frame_seq;
                oldest->capture_ts_us = 0;
                DS_DBG("CapturePipeline: SEI inject FIFO fallback (PTS %" PRIu64
                       " not in map) → seq=%" PRIu64 "\n", pts, frame_seq);
            }
        }
    }
    if (capture_ts == 0)
        return GST_PAD_PROBE_OK; // no data at all (still in startup)

    const auto sei_bytes = sei_build_nal(capture_ts, frame_seq);

    // Build combined buffer: [SEI NAL bytes] followed by [original H.264 NAL(s)]
    GstMapInfo omap;
    if (!gst_buffer_map(orig, &omap, GST_MAP_READ))
        return GST_PAD_PROBE_OK;

    const gsize total = sei_bytes.size() + omap.size;
    GstBuffer* new_buf = gst_buffer_new_and_alloc(total);
    GstMapInfo nmap;
    gst_buffer_map(new_buf, &nmap, GST_MAP_WRITE);
    std::memcpy(nmap.data,                    sei_bytes.data(), sei_bytes.size());
    std::memcpy(nmap.data + sei_bytes.size(), omap.data,        omap.size);
    gst_buffer_unmap(new_buf, &nmap);
    gst_buffer_unmap(orig,    &omap);

    // Copy timestamps and flags from original buffer
    gst_buffer_copy_into(new_buf, orig,
        static_cast<GstBufferCopyFlags>(GST_BUFFER_COPY_FLAGS |
                                        GST_BUFFER_COPY_TIMESTAMPS), 0, -1);

    // Replace buffer in probe info (release original's ref, hand new_buf ref to probe)
    gst_buffer_unref(orig);
    GST_PAD_PROBE_INFO_DATA(info) = new_buf;

    return GST_PAD_PROBE_OK;
}

// ---------------------------------------------------------------------------
// vpay src pad — fires once per RTP packet that leaves the payloader (→ udpsink).
// Emits a TRACE line per packet with seq# looked up via the SEI map.
// NOTE: a single encoded frame typically maps to one RTP packet for sub-MTU
// frames; larger frames are fragmented into multiple RTP packets by rtph264pay,
// so this probe fires once per *RTP packet*, not once per frame.
// ---------------------------------------------------------------------------
GstPadProbeReturn CapturePipeline::vpay_src_probe_cb(GstPad*, GstPadProbeInfo* info,
                                                      gpointer /*udata*/) {
    GstBuffer* buf = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!buf) return GST_PAD_PROBE_OK;

    // Skip SPS/PPS codec-data packets — they carry no frame payload.
    if (GST_BUFFER_FLAG_IS_SET(buf, GST_BUFFER_FLAG_HEADER))
        return GST_PAD_PROBE_OK;

    return GST_PAD_PROBE_OK;
}

// ---------------------------------------------------------------------------
// emit_periodic_stats — called by 1-second GLib timer
// ---------------------------------------------------------------------------
void CapturePipeline::emit_periodic_stats() {
    LatencyStats pretee_snap, enc_br_snap, enc_snap, executor_snap, vconv_snap;
    LatencyStats trk_snap[kMaxTrackers];
    bool have_pretee = false, have_enc_br = false, have_enc = false, have_executor = false;
    bool have_vconv  = false;
    bool have_trk[kMaxTrackers]{};
    {
        std::lock_guard<std::mutex> lk(m_timing_mtx);
        if (!m_pretee_stats.empty())     { pretee_snap    = m_pretee_stats.reset_and_return();     have_pretee    = true; }
        if (!m_vconv_stats.empty())      { vconv_snap     = m_vconv_stats.reset_and_return();      have_vconv     = true; }
        if (!m_enc_branch_stats.empty()) { enc_br_snap    = m_enc_branch_stats.reset_and_return(); have_enc_br    = true; }
        if (!m_enc_stats.empty())        { enc_snap       = m_enc_stats.reset_and_return();        have_enc       = true; }
        if (!m_executor_stats.empty())   { executor_snap  = m_executor_stats.reset_and_return();   have_executor  = true; }
        for (int i = 0; i < kMaxTrackers; ++i) {
            if (!m_tracker_stats[i].empty()) {
                trk_snap[i] = m_tracker_stats[i].reset_and_return();
                have_trk[i] = true;
            }
        }
    }

    // Build single message string so everything emits under one log timestamp.
    char line[256];
    std::string msg = "[TIMING-SRC]\n";

    auto append = [&](const char* tag, const LatencyStats& s) {
        snprintf(line, sizeof(line),
                 "  %-18s  avg=%6.0f  std=%5.0f  min=%6" PRIu64 "  max=%6" PRIu64
                 "  us  (n=%" PRIu64 ")\n",
                 tag, s.avg_us(), s.stddev_us(), s.min_us, s.max_us, s.count);
        msg += line;
    };

    if (have_pretee)    append("pretee",      pretee_snap);
    if (have_vconv)     append("vconv",       vconv_snap);
    if (have_enc_br)    append("enc_queue",   enc_br_snap);
    if (have_enc)       append("encode",      enc_snap);
    if (have_executor)  append("executor",    executor_snap);

    for (size_t i = 0; i < m_trackers.size() && i < kMaxTrackers; ++i) {
        if (have_trk[i]) {
            char tag[32];
            snprintf(tag, sizeof(tag), "tracker[%s]", m_trackers[i]->name());
            append(tag, trk_snap[i]);
        }
    }

    if (msg.size() > sizeof("[TIMING-SRC]\n") - 1)
        DS_INFO("%s", msg.c_str());
}

// ---------------------------------------------------------------------------
// gst_thread_func — runs GLib main loop; registers stats timer
// ---------------------------------------------------------------------------
void CapturePipeline::gst_thread_func() {
    if (m_cfg.debug.stats_interval_s > 0) {
        m_timing_timer_id = g_timeout_add_seconds(m_cfg.debug.stats_interval_s,
            [](gpointer data) -> gboolean {
                static_cast<CapturePipeline*>(data)->emit_periodic_stats();
                return G_SOURCE_CONTINUE;
            }, this);
    }
    g_main_loop_run(m_loop);
}

// ---------------------------------------------------------------------------
// toggle_pause
// ---------------------------------------------------------------------------
void CapturePipeline::toggle_pause() {
    if (!m_pipeline || !m_running.load()) return;
    if (m_paused.load()) {
        m_paused.store(false);
        gst_element_set_state(m_pipeline, GST_STATE_PLAYING);
        DS_INFO("CapturePipeline: resumed\n");
    } else {
        m_paused.store(true);
        gst_element_set_state(m_pipeline, GST_STATE_PAUSED);
        DS_INFO("CapturePipeline: paused\n");
    }
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

    // Get venc_q element — its sink pad is probed by vraw_probe_cb for frame
    // copy + executor notify (works for both normal and dummy_bbox modes).
    m_venc_q = gst_bin_get_by_name(GST_BIN(m_pipeline), "venc_q");
    if (!m_venc_q) {
        DS_ERR("CapturePipeline: queue 'venc_q' not found\n");
        gst_object_unref(m_pipeline); m_pipeline = nullptr;
        return false;
    }
    {
        GstPad* snk = gst_element_get_static_pad(m_venc_q, "sink");
        if (snk) {
            m_vraw_probe_id = gst_pad_add_probe(snk,
                GST_PAD_PROBE_TYPE_BUFFER, vraw_probe_cb, this, nullptr);
            gst_object_unref(snk);
        }
    }

    // Attach SEI inject probe unconditionally — in-band E2E timestamp embed
    m_h264parse = gst_bin_get_by_name(GST_BIN(m_pipeline), "vsparse");
    if (m_h264parse) {
        GstPad* src = gst_element_get_static_pad(m_h264parse, "src");
        if (src) {
            m_sei_inject_probe_id = gst_pad_add_probe(src,
                GST_PAD_PROBE_TYPE_BUFFER, sei_inject_probe_cb, this, nullptr);
            gst_object_unref(src);
        }
    } else {
        DS_WARN("CapturePipeline: h264parse 'vsparse' not found — SEI injection disabled\n");
    }

    // Always attach encoder probes — enc_src triggers inference at encoder-output time
    // ensuring video seq and bbox seq are aligned.
    m_encoder = gst_bin_get_by_name(GST_BIN(m_pipeline), "venc");
    if (m_encoder) {
        GstPad* snk = gst_element_get_static_pad(m_encoder, "sink");
        if (snk) {
            m_enc_sink_probe_id = gst_pad_add_probe(snk,
                GST_PAD_PROBE_TYPE_BUFFER, enc_sink_probe_cb, this, nullptr);
            gst_object_unref(snk);
        }
        GstPad* src = gst_element_get_static_pad(m_encoder, "src");
        if (src) {
            m_enc_src_probe_id = gst_pad_add_probe(src,
                GST_PAD_PROBE_TYPE_BUFFER, enc_src_probe_cb, this, nullptr);
            gst_object_unref(src);
        }
    } else {
        DS_WARN("CapturePipeline: encoder 'venc' not found — inference will not run\n");
    }

    // Attach vconv timing probes only when stats are enabled
    if (m_cfg.debug.stats_interval_s > 0) {
        m_vconv_first = gst_bin_get_by_name(GST_BIN(m_pipeline), "vconv");

        if (m_vconv_first) {
            GstPad* snk = gst_element_get_static_pad(m_vconv_first, "sink");
            if (snk) {
                m_vconv_sink_probe_id = gst_pad_add_probe(snk,
                    GST_PAD_PROBE_TYPE_BUFFER, vconv_sink_probe_cb, this, nullptr);
                gst_object_unref(snk);
            }
            GstPad* src = gst_element_get_static_pad(m_vconv_first, "src");
            if (src) {
                m_vconv_src_probe_id = gst_pad_add_probe(src,
                    GST_PAD_PROBE_TYPE_BUFFER, vconv_src_probe_cb, this, nullptr);
                gst_object_unref(src);
            }
        } else {
            DS_WARN("CapturePipeline: 'vconv' not found — pre-encode chain timing disabled\n");
        }

        // filesrc output probe disabled — uncomment to re-enable chunk-level TRACE
        // if (m_cfg.input.type == "file") {
        //     m_filesrc = gst_bin_get_by_name(GST_BIN(m_pipeline), "vfsrc");
        //     if (m_filesrc) {
        //         GstPad* src = gst_element_get_static_pad(m_filesrc, "src");
        //         if (src) {
        //             m_filesrc_src_probe_id = gst_pad_add_probe(src,
        //                 GST_PAD_PROBE_TYPE_BUFFER, filesrc_src_probe_cb, this, nullptr);
        //             gst_object_unref(src);
        //         }
        //     } else {
        //         DS_WARN("CapturePipeline: 'vfsrc' not found — filesrc output probe disabled\n");
        //     }
        // }
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
    // Start executor thread — runs trackers (or dummy bbox) in parallel with encoder
    m_executor_stop.store(false);
    m_executor_thread = std::thread(&CapturePipeline::inference_thread_func, this);
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

    // Signal executor thread to exit and wait for it before tearing down GStreamer
    m_executor_stop.store(true);
    if (m_executor_thread.joinable()) m_executor_thread.join();

    // Remove probes before transitioning to NULL to avoid use-after-free
    auto remove_probe = [](GstElement* elem, const char* pad_name, gulong& probe_id) {
        if (elem && probe_id) {
            GstPad* pad = gst_element_get_static_pad(elem, pad_name);
            if (pad) { gst_pad_remove_probe(pad, probe_id); gst_object_unref(pad); }
            probe_id = 0;
        }
    };
    remove_probe(m_vconv_first, "sink", m_vconv_sink_probe_id);
    remove_probe(m_vconv_first, "src",  m_vconv_src_probe_id);
    remove_probe(m_venc_q,      "sink", m_vraw_probe_id);
    remove_probe(m_h264parse,   "src",  m_sei_inject_probe_id);
    remove_probe(m_encoder,     "sink", m_enc_sink_probe_id);
    remove_probe(m_encoder,     "src",  m_enc_src_probe_id);
    // remove_probe(m_filesrc,     "src",  m_filesrc_src_probe_id); // filesrc probe disabled

    if (m_timing_timer_id) {
        g_source_remove(m_timing_timer_id);
        m_timing_timer_id = 0;
    }

    gst_element_set_state(m_pipeline, GST_STATE_NULL);

    if (m_loop) g_main_loop_quit(m_loop);
    if (m_gst_thread.joinable()) m_gst_thread.join();

    if (m_vconv_first){ gst_object_unref(m_vconv_first); m_vconv_first = nullptr; }
    if (m_venc_q)     { gst_object_unref(m_venc_q);      m_venc_q      = nullptr; }
    if (m_h264parse)  { gst_object_unref(m_h264parse);   m_h264parse   = nullptr; }
    if (m_vpay)       { gst_object_unref(m_vpay);        m_vpay        = nullptr; }
    if (m_encoder)    { gst_object_unref(m_encoder);     m_encoder     = nullptr; }
    if (m_filesrc)    { gst_object_unref(m_filesrc);     m_filesrc     = nullptr; }
    gst_object_unref(m_pipeline); m_pipeline = nullptr;
    if (m_loop) { g_main_loop_unref(m_loop); m_loop = nullptr; }

    DS_INFO("CapturePipeline: stopped\n");
}
