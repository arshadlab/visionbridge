/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#ifndef VISIONBRIDGE_PROTOCOL_H
#define VISIONBRIDGE_PROTOCOL_H

#include <cstdint>

// ---------------------------------------------------------------------------
// Wire protocol version — bump when message layout changes.
// ---------------------------------------------------------------------------
#define DS_PROTOCOL_VERSION 3U

// ---------------------------------------------------------------------------
// Message types
// ---------------------------------------------------------------------------
enum DS_MSG_TYPE : uint8_t {
    DS_MSG_BBOX = 0x01, ///< Bounding-box update from source to render
};

// ---------------------------------------------------------------------------
// Detector IDs — used to colour-code boxes on the render side.
// ---------------------------------------------------------------------------
enum DS_DETECTOR_ID : uint8_t {
    DS_DET_STUB = 0, ///< Stub / placeholder (gray)
    DS_DET_MOG2 = 1, ///< OpenCV BackgroundSubtractorMOG2 + contours (green)
    DS_DET_YOLO = 2, ///< OpenCV DNN — YOLOv4-tiny (red)
};

// ---------------------------------------------------------------------------
// DsBbox — one detected bounding box
// ---------------------------------------------------------------------------
struct DsBbox {
    int32_t  x;           ///< Top-left X in pixels (source frame coords)
    int32_t  y;           ///< Top-left Y in pixels
    int32_t  w;           ///< Box width
    int32_t  h;           ///< Box height
    float    confidence;  ///< Detection confidence in [0..1]
    uint8_t  detector_id; ///< DS_DETECTOR_ID
    uint8_t  _pad[3];
} __attribute__((packed));

// ---------------------------------------------------------------------------
// DsMsgBbox — one ZMQ message carrying all detections for one frame.
//
// Sent on every frame where at least one detector is enabled.
// The render side receives this via ZMQ PULL and uses it to draw overlays.
//
// Transport: ZMQ PUSH (source) → ZMQ PULL (render), TCP or IPC.
// HWM = 1 on both ends: if render is slow, the latest message replaces
// the previous one — render always sees the most recent detections.
//
// Maximum 96 bboxes total (32 per detector × 3 detectors).  This is more
// than sufficient for surveillance scenes at 1280×720.
// ---------------------------------------------------------------------------
#define DS_MAX_BBOXES 96

struct DsMsgBbox {
    uint8_t  type;              ///< DS_MSG_BBOX
    uint8_t  protocol_version;  ///< DS_PROTOCOL_VERSION
    uint8_t  _pad[2];
    uint64_t frame_seq;         ///< Monotonically increasing frame counter
    uint64_t capture_ts_us;     ///< CLOCK_REALTIME when frame entered appsink (µs)
    uint64_t send_ts_us;        ///< CLOCK_REALTIME just before ZMQ send (µs)
    uint32_t src_width;         ///< Source appsink frame width (bbox coordinate space)
    uint32_t src_height;        ///< Source appsink frame height (bbox coordinate space)
    uint32_t bbox_count;        ///< Number of valid entries in bboxes[]
    DsBbox   bboxes[DS_MAX_BBOXES];
} __attribute__((packed));

#endif // VISIONBRIDGE_PROTOCOL_H
