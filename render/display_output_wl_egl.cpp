/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#include "display_output_wl_egl.h"
#include "../common/logger.h"
#include "../common/time_utils.h"

#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <string>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <wayland-client.h>
#include <wayland-egl.h>
#include "xdg-shell-client-protocol.h"

// ---------------------------------------------------------------------------
// GLSL shaders (identical to EGL backend)
// ---------------------------------------------------------------------------
static const char* kWlVertVideo = R"(
    #version 100
    precision mediump float;
    attribute vec2 position;
    attribute vec2 texcoord;
    varying   vec2 v_uv;
    void main() {
        gl_Position = vec4(position, 0.0, 1.0);
        v_uv = texcoord;
    }
)";
static const char* kWlFragVideo = R"(
    #version 100
    precision mediump float;
    uniform sampler2D texture0;
    varying vec2 v_uv;
    void main() { gl_FragColor = texture2D(texture0, v_uv); }
)";
static const char* kWlVertLine = R"(
    #version 100
    precision mediump float;
    attribute vec2 position;
    void main() { gl_Position = vec4(position, 0.0, 1.0); }
)";
static const char* kWlFragLine = R"(
    #version 100
    precision mediump float;
    uniform vec4 u_color;
    void main() { gl_FragColor = u_color; }
)";

static const float kQuadVerts[] = {
    -1.0f, -1.0f,  0.0f, 1.0f,
     1.0f, -1.0f,  1.0f, 1.0f,
    -1.0f,  1.0f,  0.0f, 0.0f,
     1.0f,  1.0f,  1.0f, 0.0f,
};

// ---------------------------------------------------------------------------
// Shader helpers
// ---------------------------------------------------------------------------
GLuint DisplayOutputWlEgl::compile_shader(GLenum type, const char* src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok = 0; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512]; glGetShaderInfoLog(s, sizeof(log), nullptr, log);
        DS_ERR("DisplayOutputWlEgl: shader error: %s\n", log);
        glDeleteShader(s); return 0;
    }
    return s;
}
GLuint DisplayOutputWlEgl::link_program(GLuint vs, GLuint fs) {
    GLuint p = glCreateProgram();
    glAttachShader(p, vs); glAttachShader(p, fs); glLinkProgram(p);
    GLint ok = 0; glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[512]; glGetProgramInfoLog(p, sizeof(log), nullptr, log);
        DS_ERR("DisplayOutputWlEgl: link error: %s\n", log);
        glDeleteProgram(p); return 0;
    }
    glDeleteShader(vs); glDeleteShader(fs);
    return p;
}

// ---------------------------------------------------------------------------
DisplayOutputWlEgl::DisplayOutputWlEgl(const DsRenderConfig& cfg)
    : m_bbox(cfg), m_cfg(cfg) {}

// ---------------------------------------------------------------------------
// Wayland registry listeners
// ---------------------------------------------------------------------------
void DisplayOutputWlEgl::registry_global(
        void* data, wl_registry* reg, uint32_t name,
        const char* iface, uint32_t version) {
    auto* self = static_cast<DisplayOutputWlEgl*>(data);
    if (std::strcmp(iface, wl_compositor_interface.name) == 0) {
        self->m_wl_compositor = static_cast<wl_compositor*>(
            wl_registry_bind(reg, name, &wl_compositor_interface,
                             std::min(version, 4u)));
    } else if (std::strcmp(iface, xdg_wm_base_interface.name) == 0) {
        self->m_xdg_wm_base = static_cast<xdg_wm_base*>(
            wl_registry_bind(reg, name, &xdg_wm_base_interface,
                             std::min(version, 2u)));
        static const xdg_wm_base_listener kWmListener = {
            DisplayOutputWlEgl::xdg_wm_base_ping };
        xdg_wm_base_add_listener(self->m_xdg_wm_base, &kWmListener, self);
    }
}
void DisplayOutputWlEgl::registry_global_remove(void*, wl_registry*, uint32_t) {}

void DisplayOutputWlEgl::xdg_wm_base_ping(void* /*data*/,
                                           xdg_wm_base* base, uint32_t serial) {
    xdg_wm_base_pong(base, serial);
}

void DisplayOutputWlEgl::xdg_surface_configure(
        void* data, xdg_surface* surf, uint32_t serial) {
    auto* self = static_cast<DisplayOutputWlEgl*>(data);
    xdg_surface_ack_configure(surf, serial);
    self->m_configured = true;
    // Resize wl_egl_window if dimensions were provided in xdg_toplevel_configure
    if (self->m_egl_window && self->m_surface_w > 0 && self->m_surface_h > 0)
        wl_egl_window_resize(self->m_egl_window,
                             self->m_surface_w, self->m_surface_h, 0, 0);
}
void DisplayOutputWlEgl::xdg_toplevel_configure(
        void* data, xdg_toplevel*, int32_t w, int32_t h, wl_array*) {
    auto* self = static_cast<DisplayOutputWlEgl*>(data);
    if (w > 0 && h > 0) { self->m_surface_w = w; self->m_surface_h = h; }
}
void DisplayOutputWlEgl::xdg_toplevel_close(void* data, xdg_toplevel*) {
    static_cast<DisplayOutputWlEgl*>(data)->m_should_close = true;
}

// ---------------------------------------------------------------------------
// connect_wayland
// ---------------------------------------------------------------------------
bool DisplayOutputWlEgl::connect_wayland() {
    m_wl_display = wl_display_connect(nullptr);
    if (!m_wl_display) {
        DS_ERR("DisplayOutputWlEgl: wl_display_connect failed"
               " — is a Wayland compositor running? (WAYLAND_DISPLAY=%s)\n",
               getenv("WAYLAND_DISPLAY") ? getenv("WAYLAND_DISPLAY") : "(unset)");
        return false;
    }

    m_wl_registry = wl_display_get_registry(m_wl_display);
    static const wl_registry_listener kRegListener = {
        DisplayOutputWlEgl::registry_global,
        DisplayOutputWlEgl::registry_global_remove
    };
    wl_registry_add_listener(m_wl_registry, &kRegListener, this);
    wl_display_roundtrip(m_wl_display); // collect globals

    if (!m_wl_compositor) {
        DS_ERR("DisplayOutputWlEgl: wl_compositor not advertised\n");
        return false;
    }
    if (!m_xdg_wm_base) {
        DS_ERR("DisplayOutputWlEgl: xdg_wm_base not advertised"
               " — compositor must support xdg-shell\n");
        return false;
    }

    // Create surface
    m_wl_surface = wl_compositor_create_surface(m_wl_compositor);

    // Attach xdg role
    m_xdg_surface  = xdg_wm_base_get_xdg_surface(m_xdg_wm_base, m_wl_surface);
    m_xdg_toplevel = xdg_surface_get_toplevel(m_xdg_surface);

    static const xdg_surface_listener kSurfListener = {
        DisplayOutputWlEgl::xdg_surface_configure };
    xdg_surface_add_listener(m_xdg_surface, &kSurfListener, this);

    static const xdg_toplevel_listener kTopListener = {
        DisplayOutputWlEgl::xdg_toplevel_configure,
        DisplayOutputWlEgl::xdg_toplevel_close,
        nullptr, // configure_bounds (xdg-shell >= 4)
        nullptr  // wm_capabilities (xdg-shell >= 5)
    };
    xdg_toplevel_add_listener(m_xdg_toplevel, &kTopListener, this);

    xdg_toplevel_set_title(m_xdg_toplevel, m_cfg.display_title.c_str());
    xdg_toplevel_set_app_id(m_xdg_toplevel, "visionbridge");
    xdg_toplevel_set_fullscreen(m_xdg_toplevel, nullptr); // first available output

    // Initial surface dimensions (will be overridden by configure events)
    const int w = m_cfg.display_width  > 0 ? static_cast<int>(m_cfg.display_width)  : 1920;
    const int h = m_cfg.display_height > 0 ? static_cast<int>(m_cfg.display_height) : 1080;
    m_surface_w = w;
    m_surface_h = h;

    wl_surface_commit(m_wl_surface);
    wl_display_roundtrip(m_wl_display); // trigger xdg configure round-trip

    DS_INFO("DisplayOutputWlEgl: Wayland connection established  surface=%dx%d\n",
            m_surface_w, m_surface_h);
    return true;
}

// ---------------------------------------------------------------------------
// init_egl
// ---------------------------------------------------------------------------
bool DisplayOutputWlEgl::init_egl() {
    PFNEGLGETPLATFORMDISPLAYEXTPROC get_plat =
        reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(
            eglGetProcAddress("eglGetPlatformDisplayEXT"));

    if (get_plat)
        m_egl_dpy = get_plat(EGL_PLATFORM_WAYLAND_KHR, m_wl_display, nullptr);
    else
        m_egl_dpy = eglGetDisplay(reinterpret_cast<EGLNativeDisplayType>(m_wl_display));

    if (m_egl_dpy == EGL_NO_DISPLAY) {
        DS_ERR("DisplayOutputWlEgl: eglGetDisplay failed\n");
        return false;
    }

    EGLint major = 0, minor = 0;
    if (!eglInitialize(m_egl_dpy, &major, &minor)) {
        DS_ERR("DisplayOutputWlEgl: eglInitialize failed\n");
        return false;
    }
    DS_INFO("DisplayOutputWlEgl: EGL %d.%d\n", major, minor);

    eglBindAPI(EGL_OPENGL_ES_API);

    static const EGLint kCfgAttribs[] = {
        EGL_SURFACE_TYPE,    EGL_WINDOW_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
        EGL_RED_SIZE,   8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8,
        EGL_NONE
    };
    EGLConfig cfg; EGLint n = 0;
    if (!eglChooseConfig(m_egl_dpy, kCfgAttribs, &cfg, 1, &n) || n == 0) {
        DS_ERR("DisplayOutputWlEgl: eglChooseConfig failed\n");
        return false;
    }

    static const EGLint kCtx[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE };
    m_egl_ctx = eglCreateContext(m_egl_dpy, cfg, EGL_NO_CONTEXT, kCtx);

    m_egl_window = wl_egl_window_create(m_wl_surface, m_surface_w, m_surface_h);
    if (!m_egl_window) {
        DS_ERR("DisplayOutputWlEgl: wl_egl_window_create failed\n");
        return false;
    }

    m_egl_surf = eglCreateWindowSurface(m_egl_dpy, cfg,
                     reinterpret_cast<EGLNativeWindowType>(m_egl_window), nullptr);
    if (m_egl_surf == EGL_NO_SURFACE) {
        DS_ERR("DisplayOutputWlEgl: eglCreateWindowSurface failed\n");
        return false;
    }

    eglMakeCurrent(m_egl_dpy, m_egl_surf, m_egl_surf, m_egl_ctx);
    eglSwapInterval(m_egl_dpy, 0); // non-blocking: don't wait for compositor vblank
    return true;
}

// ---------------------------------------------------------------------------
// init_gl (same structure as EGL backend)
// ---------------------------------------------------------------------------
bool DisplayOutputWlEgl::init_gl() {
    GLuint vs = compile_shader(GL_VERTEX_SHADER,   kWlVertVideo);
    GLuint fs = compile_shader(GL_FRAGMENT_SHADER, kWlFragVideo);
    if (!vs || !fs) return false;
    m_prog_video = link_program(vs, fs);
    if (!m_prog_video) return false;

    m_pos_loc = glGetAttribLocation (m_prog_video, "position");
    m_uv_loc  = glGetAttribLocation (m_prog_video, "texcoord");
    m_tex_loc = glGetUniformLocation(m_prog_video, "texture0");

    GLuint lv = compile_shader(GL_VERTEX_SHADER,   kWlVertLine);
    GLuint lf = compile_shader(GL_FRAGMENT_SHADER, kWlFragLine);
    if (!lv || !lf) return false;
    m_prog_line = link_program(lv, lf);
    if (!m_prog_line) return false;

    m_lpos_loc  = glGetAttribLocation (m_prog_line, "position");
    m_color_loc = glGetUniformLocation(m_prog_line, "u_color");

    glGenBuffers(1, &m_vbo_quad);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo_quad);
    glBufferData(GL_ARRAY_BUFFER, sizeof(kQuadVerts), kQuadVerts, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glGenBuffers(1, &m_vbo_line);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo_line);
    glBufferData(GL_ARRAY_BUFFER,
                 DS_MAX_BBOXES * 4 * 4 * 2 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glGenTextures(1, &m_tex);
    glBindTexture(GL_TEXTURE_2D, m_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);
    return true;
}

// ---------------------------------------------------------------------------
// draw_frame
// ---------------------------------------------------------------------------
void DisplayOutputWlEgl::draw_frame(const DecodedFrame& frame) {
    glBindTexture(GL_TEXTURE_2D, m_tex);

    const uint64_t t0 = ds_mono_us();
    if (m_tex_w != frame.width || m_tex_h != frame.height) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
                     static_cast<GLsizei>(frame.width),
                     static_cast<GLsizei>(frame.height),
                     0, GL_RGBA, GL_UNSIGNED_BYTE, frame.data.data());
        m_tex_w = frame.width; m_tex_h = frame.height;
    } else {
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
                        static_cast<GLsizei>(frame.width),
                        static_cast<GLsizei>(frame.height),
                        GL_RGBA, GL_UNSIGNED_BYTE, frame.data.data());
    }
    m_upload_stats.record(ds_mono_us() - t0);

    glUseProgram(m_prog_video);
    glActiveTexture(GL_TEXTURE0);
    glUniform1i(m_tex_loc, 0);

    glBindBuffer(GL_ARRAY_BUFFER, m_vbo_quad);
    const GLsizei stride = 4 * sizeof(float);
    glEnableVertexAttribArray(m_pos_loc);
    glVertexAttribPointer(m_pos_loc, 2, GL_FLOAT, GL_FALSE, stride,
                          reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(m_uv_loc);
    glVertexAttribPointer(m_uv_loc,  2, GL_FLOAT, GL_FALSE, stride,
                          reinterpret_cast<void*>(2 * sizeof(float)));
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glDisableVertexAttribArray(m_pos_loc);
    glDisableVertexAttribArray(m_uv_loc);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

// ---------------------------------------------------------------------------
// draw_bboxes
// ---------------------------------------------------------------------------
void DisplayOutputWlEgl::draw_bboxes(const DsMsgBbox& msg, int rw, int rh) {
    const uint32_t count = msg.bbox_count < DS_MAX_BBOXES ? msg.bbox_count : DS_MAX_BBOXES;
    if (count == 0) return;
    static const float kColors[4][4] = {
        {0.71f, 0.71f, 0.71f, 1.0f},
        {0.20f, 0.86f, 0.20f, 1.0f},
        {0.86f, 0.20f, 0.20f, 1.0f},
        {1.00f, 1.00f, 0.00f, 1.0f},
    };
    glUseProgram(m_prog_line);
    glEnableVertexAttribArray(m_lpos_loc);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo_line);
    // Map through source frame space → NDC; independent of display resolution.
    const float frw = (msg.src_width  > 0) ? static_cast<float>(msg.src_width)
                                            : static_cast<float>(rw);
    const float frh = (msg.src_height > 0) ? static_cast<float>(msg.src_height)
                                            : static_cast<float>(rh);
    for (uint32_t i = 0; i < count; ++i) {
        const DsBbox& b = msg.bboxes[i];
        const uint8_t cid = b.detector_id < 3 ? b.detector_id : 3;
        glUniform4fv(m_color_loc, 1, kColors[cid]);
        const float x0 = (b.x         / frw) * 2.0f - 1.0f;
        const float x1 = ((b.x + b.w) / frw) * 2.0f - 1.0f;
        const float y0 = 1.0f - (b.y         / frh) * 2.0f;
        const float y1 = 1.0f - ((b.y + b.h) / frh) * 2.0f;
        const float verts[16] = {
            x0, y0,  x1, y0,  x1, y0,  x1, y1,
            x1, y1,  x0, y1,  x0, y1,  x0, y0,
        };
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(verts), verts);
        glVertexAttribPointer(m_lpos_loc, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
        glDrawArrays(GL_LINES, 0, 8);
    }
    glDisableVertexAttribArray(m_lpos_loc);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

// ---------------------------------------------------------------------------
// IDisplayOutput::init
// ---------------------------------------------------------------------------
bool DisplayOutputWlEgl::init() {
    if (!connect_wayland()) return false;
    if (!init_egl())        return false;
    if (!init_gl())         return false;
    if (!m_bbox.start())    return false;

    // Pump Wayland events until xdg_surface configure arrives
    while (!m_configured && wl_display_dispatch(m_wl_display) != -1)
        ;

    DS_INFO("DisplayOutputWlEgl: ready  %dx%d\n", m_surface_w, m_surface_h);
    return true;
}

// ---------------------------------------------------------------------------
// IDisplayOutput::update_frame
// ---------------------------------------------------------------------------
void DisplayOutputWlEgl::update_frame(std::shared_ptr<DecodedFrame> frame) {
    // Latest-wins: always overwrite; render loop detects change via pointer compare.
    std::lock_guard<std::mutex> lk(m_frame_mtx);
    m_pending_frame = std::move(frame);
}

// ---------------------------------------------------------------------------
// IDisplayOutput::present
// ---------------------------------------------------------------------------
bool DisplayOutputWlEgl::present() {
    // Pump pending Wayland events (configure, close, ping/pong) — non-blocking.
    wl_display_dispatch_pending(m_wl_display);
    if (m_should_close) return false;

    // Grab latest frame (no copy — shared_ptr ref-count bump only).
    std::shared_ptr<DecodedFrame> frame;
    {
        std::lock_guard<std::mutex> lk(m_frame_mtx);
        frame = m_pending_frame;
    }

    // Only render if a genuinely new frame arrived since last render.
    if (!frame || frame.get() == m_last_frame_ptr)
        return true;

    // Wait for the matching bbox before rendering.
    std::shared_ptr<DsMsgBbox> bbox;
    if (frame->sei_frame_seq != 0) {
        bbox = m_bbox.wait_for_seq(frame->sei_frame_seq);
        if (!bbox)
            DS_ERR("[BBOX-SYNC] timeout waiting for bbox seq=%" PRIu64
                   " — rendering without overlay\n", frame->sei_frame_seq);
    } else {
        bbox = m_bbox.best_for_frame(frame.get());
    }

    glViewport(0, 0, m_surface_w, m_surface_h);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    if (frame) draw_frame(*frame);
    if (bbox && frame) draw_bboxes(*bbox, m_surface_w, m_surface_h);

    const uint64_t t0 = ds_mono_us();
    eglSwapBuffers(m_egl_dpy, m_egl_surf);
    m_present_stats.record(ds_mono_us() - t0);

    const uint64_t now_mono = ds_mono_us();
    const uint64_t now_real = ds_realtime_us();
    m_bbox.emit_sync_stats_if_due(now_mono);

    // Accumulate SEI E2E latency
    if (frame && frame->sei_capture_ts_us != 0) {
        if (frame->sei_capture_ts_us <= now_real) {
            const uint64_t lat = now_real - frame->sei_capture_ts_us;
            if (lat < 10'000'000ULL) m_sei_stats.record(lat);
        } else if (!m_sei_skew_warn) {
            DS_WARN("[E2E-SEI] Source clock ahead — enable NTP/PTP sync\n");
            m_sei_skew_warn = true;
        }
    }

    DS_TRACE("DisplayOutputWlEgl: display seq=%" PRIu64 " bbox_seq=%" PRIu64 "\n",
             frame ? frame->sei_frame_seq : 0ULL,
             bbox ? bbox->frame_seq : 0ULL);
    m_last_frame_ptr = frame.get(); // mark as rendered
    if (frame && frame->sei_frame_seq != 0)
        m_last_displayed_seq = frame->sei_frame_seq;

    emit_render_stats_if_due(now_mono);

    return true;
}

// ---------------------------------------------------------------------------
void DisplayOutputWlEgl::emit_render_stats_if_due(uint64_t now_us) {
    if (m_cfg.debug.stats_interval_s == 0) return;
    const uint64_t interval_us =
        static_cast<uint64_t>(m_cfg.debug.stats_interval_s) * 1'000'000ULL;
    if (now_us - m_last_stats_us < interval_us) return;
    m_last_stats_us = now_us;

    char line[256];
    std::string msg = "[TIMING-RENDER WL]\n";
    bool have_any = false;

    auto append_ms = [&](const char* tag, LatencyStats& s) {
        if (s.empty()) return;
        auto snap = s.reset_and_return();
        snprintf(line, sizeof(line),
                 "  %-22s  avg=%6.1f  std=%5.1f  min=%6" PRIu64 "  max=%6" PRIu64
                 "  ms  (n=%" PRIu64 ")\n",
                 tag, snap.avg_us() / 1000.0, snap.stddev_us() / 1000.0,
                 snap.min_us / 1000, snap.max_us / 1000, snap.count);
        msg += line;
        have_any = true;
    };
    auto append_us = [&](const char* tag, LatencyStats& s) {
        if (s.empty()) return;
        auto snap = s.reset_and_return();
        snprintf(line, sizeof(line),
                 "  %-22s  avg=%6.0f  std=%5.0f  min=%6" PRIu64 "  max=%6" PRIu64
                 "  us  (n=%" PRIu64 ")\n",
                 tag, snap.avg_us(), snap.stddev_us(),
                 snap.min_us, snap.max_us, snap.count);
        msg += line;
        have_any = true;
    };

    append_ms("e2e-sei capture→disp", m_sei_stats);
    append_us("gpu_upload",           m_upload_stats);
    append_us("egl_present",          m_present_stats);

    snprintf(line, sizeof(line), "  last_seq=%" PRIu64 "\n", m_last_displayed_seq);
    msg += line;

    DS_INFO("%s", msg.c_str());
}

// ---------------------------------------------------------------------------
// cleanup_gl / shutdown
// ---------------------------------------------------------------------------
void DisplayOutputWlEgl::cleanup_gl() {
    if (m_tex)        { glDeleteTextures(1, &m_tex);     m_tex        = 0; }
    if (m_vbo_quad)   { glDeleteBuffers(1, &m_vbo_quad); m_vbo_quad   = 0; }
    if (m_vbo_line)   { glDeleteBuffers(1, &m_vbo_line); m_vbo_line   = 0; }
    if (m_prog_video) { glDeleteProgram(m_prog_video);   m_prog_video = 0; }
    if (m_prog_line)  { glDeleteProgram(m_prog_line);    m_prog_line  = 0; }
}

void DisplayOutputWlEgl::shutdown() {
    m_bbox.stop();
    cleanup_gl();

    if (m_egl_dpy != EGL_NO_DISPLAY) {
        eglMakeCurrent(m_egl_dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if (m_egl_surf   != EGL_NO_SURFACE) eglDestroySurface(m_egl_dpy, m_egl_surf);
        if (m_egl_ctx    != EGL_NO_CONTEXT) eglDestroyContext(m_egl_dpy, m_egl_ctx);
        eglTerminate(m_egl_dpy);
    }
    m_egl_dpy  = EGL_NO_DISPLAY;
    m_egl_ctx  = EGL_NO_CONTEXT;
    m_egl_surf = EGL_NO_SURFACE;

    if (m_egl_window)  { wl_egl_window_destroy(m_egl_window);         m_egl_window  = nullptr; }
    if (m_xdg_toplevel){ xdg_toplevel_destroy(m_xdg_toplevel);        m_xdg_toplevel= nullptr; }
    if (m_xdg_surface) { xdg_surface_destroy(m_xdg_surface);          m_xdg_surface = nullptr; }
    if (m_xdg_wm_base) { xdg_wm_base_destroy(m_xdg_wm_base);         m_xdg_wm_base = nullptr; }
    if (m_wl_surface)  { wl_surface_destroy(m_wl_surface);            m_wl_surface  = nullptr; }
    if (m_wl_compositor){ wl_compositor_destroy(m_wl_compositor);     m_wl_compositor=nullptr; }
    if (m_wl_registry) { wl_registry_destroy(m_wl_registry);          m_wl_registry = nullptr; }
    if (m_wl_display)  { wl_display_disconnect(m_wl_display);         m_wl_display  = nullptr; }
}
