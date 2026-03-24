#!/usr/bin/env bash
# Copyright (c) 2025-2026 Arshad Mehmood
# SPDX-License-Identifier: MIT
#
# Build script for VisionBridge using CMake.
#
# Usage:
#   ./build_cmake.sh [build_type] [clean]
#
#   build_type: Release (default) | Debug | RelWithDebInfo | MinSizeRel
#   clean:      pass "clean" as second arg to wipe and recreate the build dir
#
# Examples:
#   ./build_cmake.sh                        # Release build
#   ./build_cmake.sh Debug                  # Debug build
#   ./build_cmake.sh RelWithDebInfo clean   # fresh RelWithDebInfo build

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
BUILD_TYPE="${1:-Release}"

echo "=================================================="
echo "Building VisionBridge with CMake (Linux)"
echo "=================================================="
echo "Build type: ${BUILD_TYPE}"
echo ""

# Optionally wipe build directory
if [[ "${2:-}" == "clean" && -d "${BUILD_DIR}" ]]; then
    echo "Cleaning existing build directory..."
    rm -rf "${BUILD_DIR}"
fi

mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

# Configure
echo "Configuring..."
cmake .. \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"

# Build
echo ""
echo "Building..."
cmake --build . -- -j"$(nproc)"

echo ""
echo "=================================================="
echo "Build complete!"
echo "=================================================="
echo "Binaries:"
echo "  visionbridge_source : ${BUILD_DIR}/visionbridge_source"
echo "  visionbridge_render : ${BUILD_DIR}/visionbridge_render"
echo ""
echo "To install: cd ${BUILD_DIR} && sudo cmake --install ."
echo "=================================================="
