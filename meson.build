# Copyright (c) 2025-2026 Arshad Mehmood
# SPDX-License-Identifier: MIT
#
# VisionBridge — Multi-detector surveillance application
#
# Standalone build:
#   meson setup builddir
#   ninja -C builddir

project('visionbridge', 'cpp',
  version         : '1.0.0',
  default_options : ['cpp_std=c++17', 'warning_level=2'],
)

cpp_compiler = meson.get_compiler('cpp')

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
gst_dep       = dependency('gstreamer-1.0',       required: true)
gst_app_dep   = dependency('gstreamer-app-1.0',   required: true)
gst_video_dep = dependency('gstreamer-video-1.0', required: true)
gst_rtp_dep   = dependency('gstreamer-rtp-1.0',   required: true)

zmq_dep  = dependency('libzmq', required: true)
sdl2_dep = dependency('sdl2',   required: true)

# nlohmann/json — header-only
nlohmann_dep = dependency('nlohmann_json', required: false)
if not nlohmann_dep.found()
  nlohmann_inc = include_directories('/usr/include', '/usr/local/include',
                                     is_system: true)
  nlohmann_dep = declare_dependency(include_directories: nlohmann_inc)
  message('visionbridge: nlohmann/json using system include fallback')
endif

# OpenCV — required for MOG2; DNN module enables YOLO
opencv_dep     = dependency('opencv4', required: false)
if not opencv_dep.found()
  opencv_dep   = dependency('opencv', required: true)
endif
opencv_dnn_dep = dependency('opencv4', required: false, modules: ['opencv_dnn'])

opencv_dnn_args = []
if opencv_dnn_dep.found()
  opencv_dnn_args = ['-DDS_HAVE_OPENCV_DNN=1']
  message('visionbridge: OpenCV DNN found — YOLO tracker enabled')
else
  message('visionbridge: OpenCV DNN not found — YOLO tracker disabled (stub fallback)')
endif

# POSIX threads + rt
threads_dep = dependency('threads', required: true)
rt_dep      = cpp_compiler.find_library('rt', required: true)

# ---------------------------------------------------------------------------
# Common include directories
# ---------------------------------------------------------------------------
ds_common_inc = include_directories('common')
ds_source_inc = include_directories('source')
ds_render_inc = include_directories('render')

# ---------------------------------------------------------------------------
# ds_common — static library shared by source and render
# ---------------------------------------------------------------------------
ds_common_lib = static_library('ds_common',
  files(
    'common/logger.cpp',
    'common/config_loader.cpp',
  ),
  include_directories : [ds_common_inc],
  dependencies        : [nlohmann_dep],
  cpp_args            : ['-Wall', '-Wextra', '-Wpedantic'],
  override_options    : ['cpp_std=c++17'],
  pic                 : true,
)

ds_common_dep = declare_dependency(
  link_with           : ds_common_lib,
  include_directories : [ds_common_inc],
  dependencies        : [nlohmann_dep],
)

# ---------------------------------------------------------------------------
# visionbridge_source
# ---------------------------------------------------------------------------
ds_source_exe = executable('visionbridge_source',
  files(
    'source/main.cpp',
    'source/stub_tracker.cpp',
    'source/mog2_tracker.cpp',
    'source/yolo_tracker.cpp',
    'source/capture_pipeline.cpp',
    'source/source_node.cpp',
  ),
  include_directories : [ds_common_inc, ds_source_inc],
  dependencies        : [
    ds_common_dep,
    gst_dep, gst_app_dep, gst_video_dep, gst_rtp_dep,
    opencv_dep,
    zmq_dep,
    threads_dep,
    rt_dep,
  ],
  cpp_args         : ['-Wall', '-Wextra'] + opencv_dnn_args,
  override_options : ['cpp_std=c++17'],
  install          : true,
)

# ---------------------------------------------------------------------------
# visionbridge_render
# ---------------------------------------------------------------------------
ds_render_exe = executable('visionbridge_render',
  files(
    'render/main.cpp',
    'render/render_node.cpp',
    'render/stream_decoder.cpp',
    'render/display_output.cpp',
  ),
  include_directories : [ds_common_inc, ds_render_inc],
  dependencies        : [
    ds_common_dep,
    gst_dep, gst_app_dep, gst_video_dep, gst_rtp_dep,
    zmq_dep,
    sdl2_dep,
    threads_dep,
    rt_dep,
  ],
  cpp_args         : ['-Wall', '-Wextra'],
  override_options : ['cpp_std=c++17'],
  install          : true,
)

# ---------------------------------------------------------------------------
# Install configs
# ---------------------------------------------------------------------------
install_data(
  files('configs/source.json', 'configs/render.json'),
  install_dir: get_option('datadir') / 'visionbridge' / 'configs',
)
