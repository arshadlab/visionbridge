/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 *
 * DisplayOutputEgl — EGL/GBM/KMS direct-modesetting backend.
 *
 * Renders decoded RGBA frames directly to hardware via:
 *   GBM buffer allocation → EGL surface → OpenGL ES 2 rendering
 *   → eglSwapBuffers → drmModePageFlip at hardware vblank
 *
 * The compositor must NOT hold DRM master when this backend is used.
 * Switch to a console TTY (Ctrl+F3) before starting visionbridge_render
 * with this backend.  Press Ctrl+F1 to return to the GUI afterwards.
 *
 * Frame path (single memcopy):
 *   GStreamer buffer  →  DecodedFrame::data (memcpy in on_new_sample)
 *                     →  GL texture (glTexImage2D GPU upload)
 *                     →  eglSwapBuffers / drmModePageFlip
 * There is no intermediate CPU copy between the decoded frame and the display.
 */
#pragma once

#include "display_output_base.h"
#include "../common/pipeline_stats.h"

#include <EGL/egl.h>
#include <GLES2/gl2.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

struct gbm_device;
struct gbm_surface;
struct gbm_bo;

class DisplayOutputEgl final : public IDisplayOutput {
public:
    explicit DisplayOutputEgl(const DsRenderConfig& cfg);
    ~DisplayOutputEgl() override { shutdown(); }

    bool init()     override;
    void update_frame(std::shared_ptr<DecodedFrame> frame) override;
    bool present()  override; ///< render + eglSwapBuffers + drmModePageFlip
    void shutdown() override;

private:
    bool open_drm();
    bool init_gbm();
    bool init_egl();
    bool init_gl();
    void cleanup_gl();
    static GLuint compile_shader(GLenum type, const char* src);
    static GLuint link_program(GLuint vs, GLuint fs);

    bool page_flip_nowait();
    static void flip_complete_cb(int fd, unsigned int seq,
                                 unsigned int tv_sec, unsigned int tv_usec,
                                 void* data);

    void draw_frame(const DecodedFrame& frame);
    void draw_bboxes(const DsMsgBbox& msg, int render_w, int render_h);

    void emit_render_stats_if_due(uint64_t now_us);

    // DRM
    int  m_drm_fd    = -1;
    bool m_own_drm   = false; ///< true = we opened the fd and must close it

    uint32_t   m_crtc_id      = 0;
    uint32_t   m_connector_id = 0;
    drmModeModeInfo m_mode{};

    // GBM
    gbm_device*  m_gbm_dev  = nullptr;
    gbm_surface* m_gbm_surf = nullptr;
    gbm_bo*      m_front_bo = nullptr; ///< currently displayed BO
    uint32_t     m_front_fb = 0;       ///< FB ID for m_front_bo

    // BO → FB ID cache (avoids re-registering on every frame)
    struct FbCacheEntry { gbm_bo* bo = nullptr; uint32_t fb_id = 0; };
    static constexpr int kFbCacheSize = 4;
    FbCacheEntry m_fb_cache[kFbCacheSize]{};

    uint32_t bo_to_fb(gbm_bo* bo);
    void     release_old_front();

    // EGL
    EGLDisplay m_egl_dpy  = EGL_NO_DISPLAY;
    EGLContext m_egl_ctx  = EGL_NO_CONTEXT;
    EGLSurface m_egl_surf = EGL_NO_SURFACE;

    // GL resources
    GLuint m_prog_video = 0; ///< video frame shader program
    GLuint m_prog_line  = 0; ///< flat-color shader for bbox overlay
    GLuint m_tex        = 0; ///< RGBA frame texture
    GLuint m_vbo_quad   = 0; ///< fullscreen quad vertices + UVs
    GLuint m_vbo_line   = 0; ///< dynamic line/rect vertices for bbox

    GLint  m_pos_loc   = -1; ///< "position" attrib (video shader)
    GLint  m_uv_loc    = -1; ///< "texcoord" attrib (video shader)
    GLint  m_tex_loc   = -1; ///< "texture0" uniform (video shader)
    GLint  m_lpos_loc  = -1; ///< "position" attrib (line shader)
    GLint  m_color_loc = -1; ///< "u_color" uniform  (line shader)

    // Page-flip synchronisation (non-blocking)
    bool            m_flip_pending = false;
    bool            m_mode_set     = false;
    gbm_bo*         m_pending_bo   = nullptr; ///< BO queued in pending flip
    uint32_t        m_pending_fb   = 0;       ///< FB for pending flip
    drmEventContext m_evctx{};                ///< reused for non-blocking drain
    uint64_t        m_last_flip_complete_us = 0; ///< debug: mono-us of last flip done

    // Latest decoded frame — decoder thread writes, render thread reads.
    std::shared_ptr<DecodedFrame> m_pending_frame;
    std::mutex                    m_frame_mtx;
    const DecodedFrame*           m_last_frame_ptr = nullptr;

    // Dimensions of the last uploaded texture
    uint32_t m_tex_w = 0;
    uint32_t m_tex_h = 0;

    // Bbox support
    BboxReceiver m_bbox;

    // Render telemetry (render thread only — no mutex needed)
    LatencyStats m_upload_stats; ///< glTexImage2D duration
    LatencyStats m_flip_stats;   ///< drmModePageFlip + wait duration
    uint64_t     m_last_stats_us = 0;
    uint64_t     m_last_displayed_seq = 0;

    // E2E latency stats (render thread only)
    LatencyStats m_sei_stats;
    bool         m_sei_skew_warn = false;

    const DsRenderConfig& m_cfg;
};
