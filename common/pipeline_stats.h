/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#ifndef VISIONBRIDGE_PIPELINE_STATS_H
#define VISIONBRIDGE_PIPELINE_STATS_H

// ---------------------------------------------------------------------------
// LatencyStats — accumulates per-frame latency samples for one pipeline stage.
//
// Usage:
//   - GStreamer pad probe records wall-clock entry/exit times.
//   - delta = exit_ts - entry_ts; call record(delta).
//   - A g_timeout_add_seconds() timer calls reset_and_return() once per
//     stats_interval_s and prints the snapshot.
//
// Thread safety: caller must hold a mutex when probes run on different threads.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <limits>

struct LatencyStats {
    uint64_t count  = 0;
    uint64_t sum_us = 0;
    uint64_t min_us = UINT64_MAX;
    uint64_t max_us = 0;

    void record(uint64_t us) {
        ++count;
        sum_us += us;
        if (us < min_us) min_us = us;
        if (us > max_us) max_us = us;
    }

    double avg_us() const {
        return count ? static_cast<double>(sum_us) / static_cast<double>(count) : 0.0;
    }

    bool empty() const { return count == 0; }

    /// Snapshot this window and reset for the next period.
    LatencyStats reset_and_return() {
        LatencyStats snap = *this;
        *this = {};
        return snap;
    }
};

#endif // VISIONBRIDGE_PIPELINE_STATS_H
