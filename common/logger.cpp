/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#include "logger.h"

// g_ds_log_level defined once here; linked into both source and render executables.
std::atomic<int> g_ds_log_level{2}; // default: INFO
