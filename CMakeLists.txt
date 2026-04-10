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

# EGL / GBM / DRM (for --backend egl)
pkg_check_modules(EGL    QUIET egl)
pkg_check_modules(GLESV2 QUIET glesv2)
pkg_check_modules(GBM    QUIET gbm)
pkg_check_modules(LIBDRM QUIET libdrm)
if(EGL_FOUND AND GLESV2_FOUND AND GBM_FOUND AND LIBDRM_FOUND)
    set(HAVE_EGL TRUE)
    message(STATUS "visionbridge: EGL/KMS backend enabled")
else()
    message(STATUS "visionbridge: EGL/KMS backend disabled (missing egl/glesv2/gbm/libdrm)")
endif()

# Wayland EGL (for --backend wayland)
pkg_check_modules(WAYLAND_CLIENT   QUIET wayland-client)
pkg_check_modules(WAYLAND_EGL      QUIET wayland-egl)
pkg_check_modules(WAYLAND_PROTOS   QUIET wayland-protocols)
find_program(WAYLAND_SCANNER wayland-scanner)
if(HAVE_EGL AND WAYLAND_CLIENT_FOUND AND WAYLAND_EGL_FOUND AND WAYLAND_PROTOS_FOUND AND WAYLAND_SCANNER)
    set(HAVE_WAYLAND_EGL TRUE)
    # Generate xdg-shell protocol sources
    execute_process(COMMAND pkg-config --variable=pkgdatadir wayland-protocols
        OUTPUT_VARIABLE WP_DATADIR OUTPUT_STRIP_TRAILING_WHITESPACE)
    set(XDG_SHELL_XML "${WP_DATADIR}/stable/xdg-shell/xdg-shell.xml")
    add_custom_command(OUTPUT
        "${CMAKE_CURRENT_BINARY_DIR}/xdg-shell-client-protocol.h"
        COMMAND ${WAYLAND_SCANNER} client-header ${XDG_SHELL_XML}
                "${CMAKE_CURRENT_BINARY_DIR}/xdg-shell-client-protocol.h")
    add_custom_command(OUTPUT
        "${CMAKE_CURRENT_BINARY_DIR}/xdg-shell-protocol.c"
        COMMAND ${WAYLAND_SCANNER} private-code ${XDG_SHELL_XML}
                "${CMAKE_CURRENT_BINARY_DIR}/xdg-shell-protocol.c")
    message(STATUS "visionbridge: Wayland EGL backend enabled")
else()
    message(STATUS "visionbridge: Wayland EGL backend disabled")
endif()

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
set(RENDER_SOURCES
    render/main.cpp
    render/render_node.cpp
    render/stream_decoder.cpp
    render/display_output_base.cpp
    render/display_output_sdl.cpp
)
set(RENDER_DEFINES)
set(RENDER_INCLUDES
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
set(RENDER_LIBS
    ds_common
    ${GSTREAMER_LIBRARIES} ${GSTREAMER_APP_LIBRARIES}
    ${GSTREAMER_VIDEO_LIBRARIES} ${GSTREAMER_RTP_LIBRARIES}
    ${ZMQ_LIBRARIES} ${SDL2_LIBRARIES}
    pthread rt
)

if(HAVE_EGL)
    list(APPEND RENDER_SOURCES render/display_output_egl.cpp)
    list(APPEND RENDER_DEFINES HAVE_EGL=1)
    list(APPEND RENDER_INCLUDES ${EGL_INCLUDE_DIRS} ${GLESV2_INCLUDE_DIRS}
                                ${GBM_INCLUDE_DIRS} ${LIBDRM_INCLUDE_DIRS})
    list(APPEND RENDER_LIBS ${EGL_LIBRARIES} ${GLESV2_LIBRARIES}
                            ${GBM_LIBRARIES} ${LIBDRM_LIBRARIES})
endif()

if(HAVE_WAYLAND_EGL)
    list(APPEND RENDER_SOURCES
        render/display_output_wl_egl.cpp
        "${CMAKE_CURRENT_BINARY_DIR}/xdg-shell-protocol.c"
        "${CMAKE_CURRENT_BINARY_DIR}/xdg-shell-client-protocol.h"
    )
    list(APPEND RENDER_DEFINES HAVE_WAYLAND_EGL=1)
    list(APPEND RENDER_INCLUDES
        ${WAYLAND_CLIENT_INCLUDE_DIRS} ${WAYLAND_EGL_INCLUDE_DIRS}
        ${CMAKE_CURRENT_BINARY_DIR})
    list(APPEND RENDER_LIBS
        ${WAYLAND_CLIENT_LIBRARIES} ${WAYLAND_EGL_LIBRARIES})
endif()

add_executable(visionbridge_render ${RENDER_SOURCES})
target_include_directories(visionbridge_render PRIVATE ${RENDER_INCLUDES})
target_link_libraries(visionbridge_render PRIVATE ${RENDER_LIBS})
target_compile_definitions(visionbridge_render PRIVATE ${RENDER_DEFINES})
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
