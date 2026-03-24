/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#ifndef VISIONBRIDGE_CONFIG_LOADER_H
#define VISIONBRIDGE_CONFIG_LOADER_H

#include "config.h"
#include <string>

// ---------------------------------------------------------------------------
// ConfigLoader — reads JSON configuration files with nlohmann/json.
//
// Returns true on success, false if the file cannot be opened or the JSON
// contains a parse error (details are written to stderr).
// ---------------------------------------------------------------------------
class ConfigLoader {
public:
    /// Load source-node configuration from a JSON file.
    static bool load_source(const std::string& path, DsSourceConfig& out);

    /// Load render-node configuration from a JSON file.
    static bool load_render(const std::string& path, DsRenderConfig& out);
};

#endif // VISIONBRIDGE_CONFIG_LOADER_H
