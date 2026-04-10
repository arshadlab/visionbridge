/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 *
 * DisplayOutputWlEgl — Wayland EGL fullscreen backend.
 *
 * Creates a fullscreen xdg_toplevel Wayland surface and renders decoded RGBA
 * frames using OpenGL ES 2 via EGL.  The compositor retains DRM master so no
 * compositor teardown is required.
 *
 * Frame path (single memcopy):
 *   GStreamer buffer  →  DecodedFrame::data (memcpy in on_new_sample)
 *                     →  GL texture (glTexImage2D GPU upload)
 *                     →  eglSwapBuffers
 * There is no intermediate CPU copy between the decoded frame and the display.
 */
#pragma once

#include "display_output_base.h"
#include "../common/pipeline_stats.h"

#include <EGL/egl.h>
#include <GLES2/gl2.h>
#include <wayland-client.h>
#include <wayland-egl.h>

// xdg-shell protocol types (forward declarations to avoid pulling all headers)
struct xdg_wm_base;
struct xdg_surface;
struct xdg_toplevel;

class DisplayOutputWlEgl final : public IDisplayOutput {
public:
    explicit DisplayOutputWlEgl(const DsRenderConfig& cfg);
    ~DisplayOutputWlEgl() override { shutdown(); }

    bool init()     override;
    void update_frame(std::shared_ptr<DecodedFrame> frame) override;
    bool present()  override;
    void shutdown() override;

private:
    bool connect_wayland();
    bool init_egl();
    bool init_gl();
    void cleanup_gl();
    static GLuint compile_shader(GLenum type, const char* src);
    static GLuint link_program(GLuint vs, GLuint fs);

    void draw_frame(const DecodedFrame& frame);
    void draw_bboxes(const DsMsgBbox& msg, int render_w, int render_h);
    void emit_render_stats_if_due(uint64_t now_us);

    // Wayland registry listener helpers
    static void registry_global    (void*, wl_registry*, uint32_t, const char*, uint32_t);
    static void registry_global_remove(void*, wl_registry*, uint32_t);

    // xdg_wm_base ping/pong
    static void xdg_wm_base_ping(void*, xdg_wm_base*, uint32_t);

    // xdg_surface configure acknowledgement
    static void xdg_surface_configure(void*, xdg_surface*, uint32_t);

    // xdg_toplevel configure (resize / fullscreen state)
    static void xdg_toplevel_configure(void*, xdg_toplevel*, int32_t, int32_t, wl_array*);
    static void xdg_toplevel_close    (void*, xdg_toplevel*);

    // Wayland objects
    wl_display*    m_wl_display    = nullptr;
    wl_registry*   m_wl_registry   = nullptr;
    wl_compositor* m_wl_compositor = nullptr;
    wl_surface*    m_wl_surface    = nullptr;
    xdg_wm_base*   m_xdg_wm_base  = nullptr;
    xdg_surface*   m_xdg_surface  = nullptr;
    xdg_toplevel*  m_xdg_toplevel = nullptr;
    wl_egl_window* m_egl_window   = nullptr;

    int32_t m_surface_w = 0; ///< current configured surface width
    int32_t m_surface_h = 0;
    bool    m_should_close = false;
    bool    m_configured   = false;

    // EGL
    EGLDisplay m_egl_dpy  = EGL_NO_DISPLAY;
    EGLContext m_egl_ctx  = EGL_NO_CONTEXT;
    EGLSurface m_egl_surf = EGL_NO_SURFACE;

    // GL resources (same shaders as EGL backend)
    GLuint m_prog_video = 0;
    GLuint m_prog_line  = 0;
    GLuint m_tex        = 0;
    GLuint m_vbo_quad   = 0;
    GLuint m_vbo_line   = 0;

    GLint  m_pos_loc   = -1;
    GLint  m_uv_loc    = -1;
    GLint  m_tex_loc   = -1;
    GLint  m_lpos_loc  = -1;
    GLint  m_color_loc = -1;

    uint32_t m_tex_w = 0;
    uint32_t m_tex_h = 0;

    // Latest decoded frame — decoder thread writes, render thread reads.
    std::shared_ptr<DecodedFrame> m_pending_frame;
    std::mutex                    m_frame_mtx;
    const DecodedFrame*           m_last_frame_ptr = nullptr;

    BboxReceiver m_bbox;

    // Telemetry (render thread only)
    LatencyStats m_upload_stats;
    LatencyStats m_present_stats;
    uint64_t     m_last_stats_us = 0;
    uint64_t     m_last_displayed_seq = 0;

    LatencyStats m_sei_stats;
    bool         m_sei_skew_warn = false;

    const DsRenderConfig& m_cfg;
};
