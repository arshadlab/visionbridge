#!/usr/bin/env bash
# Copyright (c) 2025-2026 Arshad Mehmood
# SPDX-License-Identifier: MIT
#
# Build script for VisionBridge using Meson + Ninja.
#
# Usage:
#   ./build_meson.sh [build_type] [clean]
#
#   build_type: release (default) | debug | debugoptimized | minsize
#   clean:      pass "clean" as second arg to wipe and recreate the build dir
#
# Examples:
#   ./build_meson.sh                        # release build
#   ./build_meson.sh debug                  # debug build
#   ./build_meson.sh debugoptimized clean   # fresh debugoptimized build
#
# Note: This script must be run from the visionbridge/ subdirectory.
#       Meson is invoked from the repo root so that the project() declaration
#       in the root meson.build is satisfied.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/builddir"
BUILD_TYPE="${1:-release}"

echo "=================================================="
echo "Building VisionBridge with Meson (Linux)"
echo "=================================================="
echo "Build type: ${BUILD_TYPE}"
echo ""

# Optionally wipe build directory
if [[ "${2:-}" == "clean" && -d "${BUILD_DIR}" ]]; then
    echo "Cleaning existing build directory..."
    rm -rf "${BUILD_DIR}"
fi

# Configure (or reconfigure) the build directory
if [[ ! -f "${BUILD_DIR}/build.ninja" ]]; then
    echo "Configuring..."
    meson setup "${BUILD_DIR}" "${SCRIPT_DIR}" \
        --buildtype="${BUILD_TYPE}" \
        --prefix=/usr/local
    echo ""
else
    echo "Build directory exists, reconfiguring if needed..."
    meson configure "${BUILD_DIR}" \
        --buildtype="${BUILD_TYPE}"
    echo ""
fi

# Build
echo "Building..."
meson compile -C "${BUILD_DIR}" \
    visionbridge_source visionbridge_render

echo ""
echo "=================================================="
echo "Build complete!"
echo "=================================================="
echo "Binaries:"
echo "  visionbridge_source : ${BUILD_DIR}/visionbridge_source"
echo "  visionbridge_render : ${BUILD_DIR}/visionbridge_render"
echo ""
echo "To install: sudo meson install -C ${BUILD_DIR}"
echo "=================================================="
