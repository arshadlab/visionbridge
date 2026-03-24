/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#ifndef VISIONBRIDGE_TIME_UTILS_H
#define VISIONBRIDGE_TIME_UTILS_H

#include <cstdint>
#include <ctime>

// ---------------------------------------------------------------------------
// Clock accessors
// ---------------------------------------------------------------------------

/// Monotonic clock in microseconds (for intra-node durations / profiling).
inline uint64_t ds_mono_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL
         + static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
}

/// Real-time clock in microseconds (for cross-node bbox latency measurement).
inline uint64_t ds_realtime_us() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL
         + static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
}

/// Nanosecond-resolution monotonic clock.
inline uint64_t ds_mono_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL
         + static_cast<uint64_t>(ts.tv_nsec);
}

// ---------------------------------------------------------------------------
// Sleep helpers
// ---------------------------------------------------------------------------

/// Busy-sleep for the given number of milliseconds.
inline void ds_sleep_ms(uint32_t ms) {
    struct timespec ts{ static_cast<time_t>(ms / 1000),
                        static_cast<long>((ms % 1000) * 1000000L) };
    nanosleep(&ts, nullptr);
}

#endif // VISIONBRIDGE_TIME_UTILS_H
