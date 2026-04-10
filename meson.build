# Copyright (c) 2025-2026 Arshad Mehmood
# SPDX-License-Identifier: MIT
#
# VisionBridge — Multi-detector surveillance application
#
# Standalone build:
#   meson setup builddir
#   ninja -C builddir

project('visionbridge', ['cpp', 'c'],
  version         : '1.0.0',
  default_options : ['cpp_std=c++17', 'c_std=c11', 'warning_level=2'],
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

# EGL / GBM / DRM (for --backend egl)
egl_dep     = dependency('egl',       required: false)
glesv2_dep  = dependency('glesv2',    required: false)
gbm_dep     = dependency('gbm',       required: false)
libdrm_dep  = dependency('libdrm',    required: false)
have_egl = egl_dep.found() and glesv2_dep.found() and gbm_dep.found() and libdrm_dep.found()

# Wayland EGL (for --backend wayland)
wayland_client_dep = dependency('wayland-client',   required: false)
wayland_egl_dep    = dependency('wayland-egl',      required: false)
wayland_protos_dep = dependency('wayland-protocols', required: false)
have_wayland = have_egl and wayland_client_dep.found() and wayland_egl_dep.found() and wayland_protos_dep.found()

# xdg-shell protocol generated C sources (needed only for Wayland EGL)
xdg_shell_src = []
xdg_shell_args = []
if have_wayland
  wayland_scanner = find_program('wayland-scanner', required: false)
  if wayland_scanner.found()
    wp_stable = wayland_protos_dep.get_variable(pkgconfig: 'pkgdatadir') / 'stable/xdg-shell/xdg-shell.xml'
    xdg_shell_hdr = custom_target('xdg-shell-client-protocol.h',
      output  : 'xdg-shell-client-protocol.h',
      input   : wp_stable,
      command : [wayland_scanner, 'client-header', '@INPUT@', '@OUTPUT@'],
      install : false,
    )
    xdg_shell_c = custom_target('xdg-shell-protocol.c',
      output  : 'xdg-shell-protocol.c',
      input   : wp_stable,
      command : [wayland_scanner, 'private-code', '@INPUT@', '@OUTPUT@'],
      install : false,
    )
    xdg_shell_src = [xdg_shell_hdr, xdg_shell_c]
    xdg_shell_args = ['-DHAVE_WAYLAND_EGL=1']
    message('visionbridge: Wayland EGL backend enabled')
  else
    have_wayland = false
    message('visionbridge: wayland-scanner not found — Wayland EGL backend disabled')
  endif
endif

if have_egl
  message('visionbridge: EGL/KMS backend enabled')
else
  message('visionbridge: EGL/KMS backend disabled (missing egl/glesv2/gbm/libdrm)')
endif

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
render_sources = files(
  'render/main.cpp',
  'render/render_node.cpp',
  'render/stream_decoder.cpp',
  'render/display_output_base.cpp',
  'render/display_output_sdl.cpp',
)
render_deps = [
  ds_common_dep,
  gst_dep, gst_app_dep, gst_video_dep, gst_rtp_dep,
  zmq_dep,
  sdl2_dep,
  threads_dep,
  rt_dep,
]
render_args = ['-Wall', '-Wextra']

if have_egl
  render_sources += files(
    'render/display_output_egl.cpp',
  )
  render_deps += [egl_dep, glesv2_dep, gbm_dep, libdrm_dep]
  render_args += ['-DHAVE_EGL=1']
endif

if have_wayland
  render_sources += files(
    'render/display_output_wl_egl.cpp',
  )
  render_sources += xdg_shell_src
  render_deps += [wayland_client_dep, wayland_egl_dep]
  render_args += xdg_shell_args
endif

ds_render_exe = executable('visionbridge_render',
  render_sources,
  include_directories : [ds_common_inc, ds_render_inc],
  dependencies        : render_deps,
  cpp_args            : render_args,
  override_options    : ['cpp_std=c++17'],
  install             : true,
)

# ---------------------------------------------------------------------------
# Install configs
# ---------------------------------------------------------------------------
install_data(
  files('configs/source.json', 'configs/render.json'),
  install_dir: get_option('datadir') / 'visionbridge' / 'configs',
)
