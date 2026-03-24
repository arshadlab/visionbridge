# Copyright (c) 2025-2026 Arshad Mehmood
# SPDX-License-Identifier: MIT
#
# VisionBridge — Multi-detector surveillance application
#
# Standalone build:
#   mkdir build && cd build
#   cmake ../visionbridge -DCMAKE_BUILD_TYPE=RelWithDebInfo
#   make -j$(nproc)
#
# As part of vsyncalter:
#   cmake .. -DBUILD_VISIONBRIDGE=ON
#

cmake_minimum_required(VERSION 3.16)

if(NOT CMAKE_PROJECT_NAME STREQUAL "vsyncalter")
    project(visionbridge VERSION 1.0.0 LANGUAGES CXX)
    set(STANDALONE_BUILD TRUE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Find dependencies
# ---------------------------------------------------------------------------
find_package(PkgConfig REQUIRED)

pkg_check_modules(GSTREAMER       REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_APP   REQUIRED gstreamer-app-1.0)
pkg_check_modules(GSTREAMER_VIDEO REQUIRED gstreamer-video-1.0)
pkg_check_modules(GSTREAMER_RTP   REQUIRED gstreamer-rtp-1.0)

find_package(SDL2 REQUIRED)

pkg_check_modules(ZMQ REQUIRED libzmq)

# OpenCV
find_package(OpenCV REQUIRED COMPONENTS core imgproc video)
find_package(OpenCV QUIET  COMPONENTS dnn)
if(OpenCV_dnn_FOUND OR TARGET opencv_dnn)
    set(DS_HAVE_OPENCV_DNN TRUE)
    message(STATUS "visionbridge: OpenCV DNN found — YOLO tracker enabled")
else()
    message(STATUS "visionbridge: OpenCV DNN not found — YOLO tracker disabled")
endif()

# nlohmann/json (header-only)
find_path(NLOHMANN_JSON_INCLUDE_DIR nlohmann/json.hpp
    PATHS /usr/include /usr/local/include
    DOC "nlohmann JSON header")
if(NOT NLOHMANN_JSON_INCLUDE_DIR)
    message(FATAL_ERROR "nlohmann/json not found. Install: sudo apt install nlohmann-json3-dev")
endif()

# ---------------------------------------------------------------------------
# ds_common — static library
# ---------------------------------------------------------------------------
add_library(ds_common STATIC
    common/logger.cpp
    common/config_loader.cpp
)

target_include_directories(ds_common PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${NLOHMANN_JSON_INCLUDE_DIR}
)

target_compile_options(ds_common PRIVATE -Wall -Wextra -Wpedantic)

# ---------------------------------------------------------------------------
# visionbridge_source
# ---------------------------------------------------------------------------
add_executable(visionbridge_source
    source/main.cpp
    source/stub_tracker.cpp
    source/mog2_tracker.cpp
    source/yolo_tracker.cpp
    source/capture_pipeline.cpp
    source/source_node.cpp
)

target_include_directories(visionbridge_source PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/source
    ${GSTREAMER_INCLUDE_DIRS}
    ${GSTREAMER_APP_INCLUDE_DIRS}
    ${GSTREAMER_VIDEO_INCLUDE_DIRS}
    ${GSTREAMER_RTP_INCLUDE_DIRS}
    ${ZMQ_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${NLOHMANN_JSON_INCLUDE_DIR}
)

target_link_libraries(visionbridge_source PRIVATE
    ds_common
    ${GSTREAMER_LIBRARIES}
    ${GSTREAMER_APP_LIBRARIES}
    ${GSTREAMER_VIDEO_LIBRARIES}
    ${GSTREAMER_RTP_LIBRARIES}
    ${ZMQ_LIBRARIES}
    ${OpenCV_LIBS}
    pthread
    rt
)

if(DS_HAVE_OPENCV_DNN)
    target_compile_definitions(visionbridge_source PRIVATE DS_HAVE_OPENCV_DNN=1)
endif()

target_compile_options(visionbridge_source PRIVATE -Wall -Wextra)

# ---------------------------------------------------------------------------
# visionbridge_render
# ---------------------------------------------------------------------------
add_executable(visionbridge_render
    render/main.cpp
    render/render_node.cpp
    render/stream_decoder.cpp
    render/display_output.cpp
)

target_include_directories(visionbridge_render PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    ${CMAKE_CURRENT_SOURCE_DIR}/render
    ${GSTREAMER_INCLUDE_DIRS}
    ${GSTREAMER_APP_INCLUDE_DIRS}
    ${GSTREAMER_VIDEO_INCLUDE_DIRS}
    ${GSTREAMER_RTP_INCLUDE_DIRS}
    ${ZMQ_INCLUDE_DIRS}
    ${SDL2_INCLUDE_DIRS}
    ${NLOHMANN_JSON_INCLUDE_DIR}
)

target_link_libraries(visionbridge_render PRIVATE
    ds_common
    ${GSTREAMER_LIBRARIES}
    ${GSTREAMER_APP_LIBRARIES}
    ${GSTREAMER_VIDEO_LIBRARIES}
    ${GSTREAMER_RTP_LIBRARIES}
    ${ZMQ_LIBRARIES}
    ${SDL2_LIBRARIES}
    pthread
    rt
)

target_compile_options(visionbridge_render PRIVATE -Wall -Wextra)

# ---------------------------------------------------------------------------
# Install targets
# ---------------------------------------------------------------------------
install(TARGETS visionbridge_source visionbridge_render
    RUNTIME DESTINATION bin
)
install(DIRECTORY configs/
    DESTINATION share/visionbridge/configs
)

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
message(STATUS "")
message(STATUS "=== VisionBridge Build Configuration ===")
message(STATUS "  GStreamer:       ${GSTREAMER_VERSION}")
message(STATUS "  SDL2:            found")
message(STATUS "  ZeroMQ:          ${ZMQ_VERSION}")
message(STATUS "  OpenCV:          ${OpenCV_VERSION}")
message(STATUS "  YOLO (DNN):      ${DS_HAVE_OPENCV_DNN}")
message(STATUS "  Install prefix:  ${CMAKE_INSTALL_PREFIX}/bin")
message(STATUS "========================================")
message(STATUS "")
