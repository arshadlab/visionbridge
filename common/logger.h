/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#ifndef VISIONBRIDGE_LOGGER_H
#define VISIONBRIDGE_LOGGER_H

#include <cstdio>
#include <ctime>
#include <atomic>

// ---------------------------------------------------------------------------
// Log levels (ascending verbosity)
// ---------------------------------------------------------------------------
enum class LogLevel {
    ERR   = 0,
    WARN  = 1,
    INFO  = 2,
    DEBUG = 3,
    TRACE = 4,
};

/// Global log level — set once at startup via ds_set_log_level().
extern std::atomic<int> g_ds_log_level;

inline void ds_set_log_level(LogLevel lvl) {
    g_ds_log_level.store(static_cast<int>(lvl));
}

inline LogLevel ds_get_log_level() {
    return static_cast<LogLevel>(g_ds_log_level.load());
}

// ---------------------------------------------------------------------------
// Monotonic timestamp in seconds (for log prefix)
// ---------------------------------------------------------------------------
inline double ds_log_ts_sec() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

// ---------------------------------------------------------------------------
// Log macros
// ---------------------------------------------------------------------------
#define DS_LOG_IMPL(lvl_val, prefix, fmt, ...)                              \
    do {                                                                    \
        if (g_ds_log_level.load() >= (lvl_val)) {                           \
            _Pragma("GCC diagnostic push")                                  \
            _Pragma("GCC diagnostic ignored \"-Wformat\"")                  \
            fprintf(stderr, "[%8.3f] " prefix " %s:%d: " fmt,              \
                    ds_log_ts_sec(), __func__, __LINE__, ##__VA_ARGS__);    \
            _Pragma("GCC diagnostic pop")                                   \
        }                                                                   \
    } while (0)

#define DS_ERR(fmt, ...)   DS_LOG_IMPL(0, "\033[31m[ERR]\033[0m",  fmt, ##__VA_ARGS__)
#define DS_WARN(fmt, ...)  DS_LOG_IMPL(1, "\033[33m[WRN]\033[0m",  fmt, ##__VA_ARGS__)
#define DS_INFO(fmt, ...)  DS_LOG_IMPL(2, "\033[32m[INF]\033[0m",  fmt, ##__VA_ARGS__)
#define DS_DBG(fmt, ...)   DS_LOG_IMPL(3, "\033[36m[DBG]\033[0m",  fmt, ##__VA_ARGS__)
#define DS_TRACE(fmt, ...) DS_LOG_IMPL(4,         "[TRC]",          fmt, ##__VA_ARGS__)

#endif // VISIONBRIDGE_LOGGER_H
