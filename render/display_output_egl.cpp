/*
 * Copyright (c) 2025-2026 Arshad Mehmood
 * SPDX-License-Identifier: MIT
 */

#include "display_output_egl.h"
#include "../common/logger.h"
#include "../common/time_utils.h"

#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <glob.h>
#include <string>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <gbm.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

// ---------------------------------------------------------------------------
// GLSL shaders
// ---------------------------------------------------------------------------

// Video frame: fullscreen textured quad
static const char* kVertVideo = R"(
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

static const char* kFragVideo = R"(
    #version 100
    precision mediump float;
    uniform sampler2D texture0;
    varying vec2 v_uv;
    void main() {
        gl_FragColor = texture2D(texture0, v_uv);
    }
)";

// Flat color: bbox overlay rectangles
static const char* kVertLine = R"(
    #version 100
    precision mediump float;
    attribute vec2 position;
    void main() {
        gl_Position = vec4(position, 0.0, 1.0);
    }
)";

static const char* kFragLine = R"(
    #version 100
    precision mediump float;
    uniform vec4 u_color;
    void main() {
        gl_FragColor = u_color;
    }
)";

// Fullscreen quad: NDC [-1,1] with UV [0,1]
// Layout: x, y, u, v (interleaved 4 floats per vertex)
static const float kQuadVerts[] = {
    -1.0f, -1.0f,  0.0f, 1.0f,
     1.0f, -1.0f,  1.0f, 1.0f,
    -1.0f,  1.0f,  0.0f, 0.0f,
     1.0f,  1.0f,  1.0f, 0.0f,
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
GLuint DisplayOutputEgl::compile_shader(GLenum type, const char* src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512]; glGetShaderInfoLog(s, sizeof(log), nullptr, log);
        DS_ERR("DisplayOutputEgl: shader compile error: %s\n", log);
        glDeleteShader(s);
        return 0;
    }
    return s;
}

GLuint DisplayOutputEgl::link_program(GLuint vs, GLuint fs) {
    GLuint p = glCreateProgram();
    glAttachShader(p, vs); glAttachShader(p, fs);
    glLinkProgram(p);
    GLint ok = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[512]; glGetProgramInfoLog(p, sizeof(log), nullptr, log);
        DS_ERR("DisplayOutputEgl: program link error: %s\n", log);
        glDeleteProgram(p);
        return 0;
    }
    glDeleteShader(vs); glDeleteShader(fs);
    return p;
}

// ---------------------------------------------------------------------------
DisplayOutputEgl::DisplayOutputEgl(const DsRenderConfig& cfg)
    : m_bbox(cfg), m_cfg(cfg) {}

// ---------------------------------------------------------------------------
// open_drm — find and open the DRM device; select connector and mode
// ---------------------------------------------------------------------------
bool DisplayOutputEgl::open_drm() {
    // Prefer explicit drm_device config; otherwise scan /dev/dri/card*
    std::string dev_path = m_cfg.drm_device;
    if (dev_path.empty()) {
        glob_t gl{};
        if (glob("/dev/dri/card*", 0, nullptr, &gl) == 0 && gl.gl_pathc > 0)
            dev_path = gl.gl_pathv[0];
        globfree(&gl);
    }
    if (dev_path.empty()) {
        DS_ERR("DisplayOutputEgl: no DRM device found\n");
        return false;
    }

    m_drm_fd = open(dev_path.c_str(), O_RDWR | O_CLOEXEC);
    if (m_drm_fd < 0) {
        DS_ERR("DisplayOutputEgl: cannot open %s: %s\n",
               dev_path.c_str(), strerror(errno));
        return false;
    }
    m_own_drm = true;

    if (drmSetMaster(m_drm_fd) != 0) {
        DS_ERR("DisplayOutputEgl: drmSetMaster failed — is a compositor still running?\n"
               "  Switch to a console TTY (Ctrl+F3) before starting with --backend egl\n");
        ::close(m_drm_fd); m_drm_fd = -1;
        return false;
    }

    drmModeRes* res = drmModeGetResources(m_drm_fd);
    if (!res) {
        DS_ERR("DisplayOutputEgl: drmModeGetResources failed\n");
        return false;
    }

    // Find the Nth connected connector (drm_output_index)
    int found = 0;
    for (int i = 0; i < res->count_connectors && m_connector_id == 0; ++i) {
        drmModeConnector* conn = drmModeGetConnector(m_drm_fd, res->connectors[i]);
        if (!conn) continue;
        if (conn->connection == DRM_MODE_CONNECTED && conn->count_modes > 0) {
            if (found == m_cfg.drm_output_index) {
                m_connector_id = conn->connector_id;
                m_mode         = conn->modes[0]; // first (preferred) mode

                // Find associated CRTC via encoder
                if (conn->encoder_id) {
                    drmModeEncoder* enc = drmModeGetEncoder(m_drm_fd, conn->encoder_id);
                    if (enc) { m_crtc_id = enc->crtc_id; drmModeFreeEncoder(enc); }
                }
                // Fallback: pick first available CRTC
                if (!m_crtc_id && res->count_crtcs > 0)
                    m_crtc_id = res->crtcs[0];
            }
            ++found;
        }
        drmModeFreeConnector(conn);
    }
    drmModeFreeResources(res);

    if (!m_connector_id || !m_crtc_id) {
        DS_ERR("DisplayOutputEgl: no connected display at output_index=%d\n",
               m_cfg.drm_output_index);
        return false;
    }

    DS_INFO("DisplayOutputEgl: %s  connector=%u crtc=%u  mode=%ux%u@%uHz\n",
            dev_path.c_str(), m_connector_id, m_crtc_id,
            m_mode.hdisplay, m_mode.vdisplay, m_mode.vrefresh);
    return true;
}

// ---------------------------------------------------------------------------
// init_gbm
// ---------------------------------------------------------------------------
bool DisplayOutputEgl::init_gbm() {
    m_gbm_dev = gbm_create_device(m_drm_fd);
    if (!m_gbm_dev) {
        DS_ERR("DisplayOutputEgl: gbm_create_device failed\n");
        return false;
    }

    m_gbm_surf = gbm_surface_create(m_gbm_dev,
                                    m_mode.hdisplay, m_mode.vdisplay,
                                    GBM_FORMAT_XRGB8888,
                                    GBM_BO_USE_SCANOUT | GBM_BO_USE_RENDERING);
    if (!m_gbm_surf) {
        DS_ERR("DisplayOutputEgl: gbm_surface_create failed\n");
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// init_egl
// ---------------------------------------------------------------------------
bool DisplayOutputEgl::init_egl() {
    // Use GBM platform extension
    PFNEGLGETPLATFORMDISPLAYEXTPROC get_plat =
        reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(
            eglGetProcAddress("eglGetPlatformDisplayEXT"));

    if (get_plat)
        m_egl_dpy = get_plat(EGL_PLATFORM_GBM_KHR, m_gbm_dev, nullptr);
    else
        m_egl_dpy = eglGetDisplay(reinterpret_cast<EGLNativeDisplayType>(m_gbm_dev));

    if (m_egl_dpy == EGL_NO_DISPLAY) {
        DS_ERR("DisplayOutputEgl: eglGetDisplay failed\n");
        return false;
    }

    EGLint major = 0, minor = 0;
    if (!eglInitialize(m_egl_dpy, &major, &minor)) {
        DS_ERR("DisplayOutputEgl: eglInitialize failed\n");
        return false;
    }
    DS_INFO("DisplayOutputEgl: EGL %d.%d\n", major, minor);

    if (!eglBindAPI(EGL_OPENGL_ES_API)) {
        DS_ERR("DisplayOutputEgl: eglBindAPI failed\n");
        return false;
    }

    static const EGLint kCfgAttribs[] = {
        EGL_SURFACE_TYPE,    EGL_WINDOW_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
        EGL_RED_SIZE,   8, EGL_GREEN_SIZE, 8,
        EGL_BLUE_SIZE,  8, EGL_ALPHA_SIZE, 0,
        EGL_NONE
    };
    EGLConfig cfg; EGLint n = 0;
    if (!eglChooseConfig(m_egl_dpy, kCfgAttribs, &cfg, 1, &n) || n == 0) {
        DS_ERR("DisplayOutputEgl: eglChooseConfig failed\n");
        return false;
    }

    static const EGLint kCtxAttribs[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE };
    m_egl_ctx = eglCreateContext(m_egl_dpy, cfg, EGL_NO_CONTEXT, kCtxAttribs);
    if (m_egl_ctx == EGL_NO_CONTEXT) {
        DS_ERR("DisplayOutputEgl: eglCreateContext failed\n");
        return false;
    }

    m_egl_surf = eglCreateWindowSurface(m_egl_dpy, cfg,
                     reinterpret_cast<EGLNativeWindowType>(m_gbm_surf), nullptr);
    if (m_egl_surf == EGL_NO_SURFACE) {
        DS_ERR("DisplayOutputEgl: eglCreateWindowSurface failed\n");
        return false;
    }

    if (!eglMakeCurrent(m_egl_dpy, m_egl_surf, m_egl_surf, m_egl_ctx)) {
        DS_ERR("DisplayOutputEgl: eglMakeCurrent failed\n");
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// init_gl
// ---------------------------------------------------------------------------
bool DisplayOutputEgl::init_gl() {
    // --- Video shader ---
    GLuint vs = compile_shader(GL_VERTEX_SHADER,   kVertVideo);
    GLuint fs = compile_shader(GL_FRAGMENT_SHADER, kFragVideo);
    if (!vs || !fs) return false;
    m_prog_video = link_program(vs, fs);
    if (!m_prog_video) return false;

    m_pos_loc = glGetAttribLocation (m_prog_video, "position");
    m_uv_loc  = glGetAttribLocation (m_prog_video, "texcoord");
    m_tex_loc = glGetUniformLocation(m_prog_video, "texture0");

    // --- Line/bbox shader ---
    GLuint lv = compile_shader(GL_VERTEX_SHADER,   kVertLine);
    GLuint lf = compile_shader(GL_FRAGMENT_SHADER, kFragLine);
    if (!lv || !lf) return false;
    m_prog_line = link_program(lv, lf);
    if (!m_prog_line) return false;

    m_lpos_loc  = glGetAttribLocation (m_prog_line, "position");
    m_color_loc = glGetUniformLocation(m_prog_line, "u_color");

    // --- Fullscreen quad VBO ---
    glGenBuffers(1, &m_vbo_quad);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo_quad);
    glBufferData(GL_ARRAY_BUFFER, sizeof(kQuadVerts), kQuadVerts, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // --- Dynamic VBO for bbox lines (pre-allocate room for DS_MAX_BBOXES * 4 rects
    //     × 4 line segments × 2 vertices × 2 floats) ---
    glGenBuffers(1, &m_vbo_line);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo_line);
    glBufferData(GL_ARRAY_BUFFER,
                 DS_MAX_BBOXES * 4 * 4 * 2 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // --- Frame texture ---
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
// bo_to_fb — look up or register a framebuffer ID for a GBM BO
// ---------------------------------------------------------------------------
uint32_t DisplayOutputEgl::bo_to_fb(gbm_bo* bo) {
    for (auto& e : m_fb_cache)
        if (e.bo == bo) return e.fb_id;

    // Not cached — register with KMS
    const uint32_t w   = gbm_bo_get_width(bo);
    const uint32_t h   = gbm_bo_get_height(bo);
    const uint32_t fmt = gbm_bo_get_format(bo);
    const uint32_t str = gbm_bo_get_stride(bo);
    const uint32_t hdl = gbm_bo_get_handle(bo).u32;

    uint32_t fb_id = 0;
    if (drmModeAddFB(m_drm_fd, w, h, 24, 32, str, hdl, &fb_id) != 0) {
        DS_ERR("DisplayOutputEgl: drmModeAddFB failed: %s (fmt=%u)\n",
               strerror(errno), fmt);
        return 0;
    }

    // Store in first empty slot (evict oldest if full)
    for (auto& e : m_fb_cache) {
        if (!e.bo) { e.bo = bo; e.fb_id = fb_id; return fb_id; }
    }
    // All slots full — overwrite first (oldest)
    drmModeRmFB(m_drm_fd, m_fb_cache[0].fb_id);
    m_fb_cache[0] = {bo, fb_id};
    return fb_id;
}

// ---------------------------------------------------------------------------
// release_old_front — release previous front BO back to GBM
// ---------------------------------------------------------------------------
void DisplayOutputEgl::release_old_front() {
    if (m_front_bo) {
        gbm_surface_release_buffer(m_gbm_surf, m_front_bo);
        m_front_bo = nullptr;
        m_front_fb = 0;
    }
}

// ---------------------------------------------------------------------------
// flip_complete_cb — called by drmHandleEvent when a page flip finishes.
// Records completion time (debug) and releases the old front BO.
// ---------------------------------------------------------------------------
void DisplayOutputEgl::flip_complete_cb(int /*fd*/, unsigned int /*seq*/,
                                        unsigned int /*tv_sec*/,
                                        unsigned int /*tv_usec*/,
                                        void* data) {
    DisplayOutputEgl* self = static_cast<DisplayOutputEgl*>(data);
    self->m_last_flip_complete_us = ds_mono_us();
    DS_TRACE("DisplayOutputEgl: flip complete  pending_bo=%p → front\n",
             static_cast<void*>(self->m_pending_bo));

    // Release the BO that was on-screen before this flip was issued.
    if (self->m_front_bo)
        gbm_surface_release_buffer(self->m_gbm_surf, self->m_front_bo);

    // The pending BO is now the front (currently displayed).
    self->m_front_bo  = self->m_pending_bo;
    self->m_front_fb  = self->m_pending_fb;
    self->m_pending_bo = nullptr;
    self->m_pending_fb = 0;
    self->m_flip_pending = false;
}

// ---------------------------------------------------------------------------
// page_flip_nowait — non-blocking flip.
//
// 1. Drain any already-completed flip events (timeout=0 → instantaneous).
// 2. If a flip is still in-flight, skip this frame (returns true — not fatal).
//    The render loop will try again on the next 100 µs poll; rate is naturally
//    capped to the display refresh when the source is faster than the display.
// 3. Lock the next GBM front buffer (always a different BO from the pending one),
//    issue drmModePageFlip without waiting, and return immediately.
// ---------------------------------------------------------------------------
bool DisplayOutputEgl::page_flip_nowait() {
    // Initialise evctx once.
    if (m_evctx.version == 0) {
        m_evctx.version           = DRM_EVENT_CONTEXT_VERSION;
        m_evctx.page_flip_handler = flip_complete_cb;
    }

    // Non-blocking drain: process any flip completion events already in the fd.
    if (m_flip_pending) {
        fd_set fds; FD_ZERO(&fds); FD_SET(m_drm_fd, &fds);
        struct timeval tv { 0, 0 };
        if (select(m_drm_fd + 1, &fds, nullptr, nullptr, &tv) > 0)
            drmHandleEvent(m_drm_fd, &m_evctx); // may clear m_flip_pending
    }

    if (m_flip_pending) {
        // Previous flip not yet complete — skip this frame.
        DS_TRACE("DisplayOutputEgl: flip in-flight, skipping frame\n");
        return true;
    }

    gbm_bo* bo = gbm_surface_lock_front_buffer(m_gbm_surf);
    if (!bo) {
        DS_ERR("DisplayOutputEgl: gbm_surface_lock_front_buffer failed\n");
        return false;
    }
    const uint32_t fb_id = bo_to_fb(bo);
    if (!fb_id) { gbm_surface_release_buffer(m_gbm_surf, bo); return false; }

    // First frame: set CRTC mode (blocking once, unavoidable).
    if (!m_mode_set) {
        if (drmModeSetCrtc(m_drm_fd, m_crtc_id, fb_id, 0, 0,
                           &m_connector_id, 1, &m_mode) != 0) {
            DS_ERR("DisplayOutputEgl: drmModeSetCrtc failed: %s\n", strerror(errno));
            gbm_surface_release_buffer(m_gbm_surf, bo);
            return false;
        }
        m_mode_set  = true;
        m_front_bo  = bo;
        m_front_fb  = fb_id;
        return true;
    }

    // Issue page flip — returns immediately; completion fires flip_complete_cb.
    const int ret = drmModePageFlip(m_drm_fd, m_crtc_id, fb_id,
                                    DRM_MODE_PAGE_FLIP_EVENT, this);
    if (ret != 0) {
        DS_ERR("DisplayOutputEgl: drmModePageFlip failed: %s\n", strerror(errno));
        gbm_surface_release_buffer(m_gbm_surf, bo);
        return false;
    }

    // Track: m_front_bo is still displayed; m_pending_bo will become front after flip.
    m_pending_bo   = bo;
    m_pending_fb   = fb_id;
    m_flip_pending = true;
    return true;
}

// ---------------------------------------------------------------------------
// draw_frame — upload RGBA data and draw fullscreen quad
// ---------------------------------------------------------------------------
void DisplayOutputEgl::draw_frame(const DecodedFrame& frame) {
    glBindTexture(GL_TEXTURE_2D, m_tex);

    const uint64_t t0 = ds_mono_us();
    if (m_tex_w != frame.width || m_tex_h != frame.height) {
        // First frame or resolution change: allocate texture storage
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
                     static_cast<GLsizei>(frame.width),
                     static_cast<GLsizei>(frame.height),
                     0, GL_RGBA, GL_UNSIGNED_BYTE, frame.data.data());
        m_tex_w = frame.width;
        m_tex_h = frame.height;
    } else {
        // Same size: sub-image update (avoids texture re-allocation overhead)
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
                        static_cast<GLsizei>(frame.width),
                        static_cast<GLsizei>(frame.height),
                        GL_RGBA, GL_UNSIGNED_BYTE, frame.data.data());
    }
    m_upload_stats.record(ds_mono_us() - t0);

    // Draw fullscreen quad
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
// draw_bboxes — draw colored rectangle outlines via GL_LINES
// ---------------------------------------------------------------------------
void DisplayOutputEgl::draw_bboxes(const DsMsgBbox& msg, int render_w, int render_h) {
    const uint32_t count = msg.bbox_count < DS_MAX_BBOXES ? msg.bbox_count : DS_MAX_BBOXES;
    if (count == 0) return;

    static const float kColors[4][4] = {
        {0.71f, 0.71f, 0.71f, 1.0f}, // stub  — gray
        {0.20f, 0.86f, 0.20f, 1.0f}, // mog2  — green
        {0.86f, 0.20f, 0.20f, 1.0f}, // yolo  — red
        {1.00f, 1.00f, 0.00f, 1.0f}, // other — yellow
    };

    glUseProgram(m_prog_line);
    glEnableVertexAttribArray(m_lpos_loc);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo_line);

    // Convert pixel coords (source frame space) directly to NDC via source dims.
    // This makes bbox overlay correct regardless of display resolution.
    const float rw = (msg.src_width  > 0) ? static_cast<float>(msg.src_width)
                                           : static_cast<float>(render_w);
    const float rh = (msg.src_height > 0) ? static_cast<float>(msg.src_height)
                                           : static_cast<float>(render_h);

    for (uint32_t i = 0; i < count; ++i) {
        const DsBbox& b = msg.bboxes[i];
        const uint8_t cid = b.detector_id < 3 ? b.detector_id : 3;
        glUniform4fv(m_color_loc, 1, kColors[cid]);

        // Convert pixel coords to NDC [-1, 1]
        const float x0 = (b.x         / rw) * 2.0f - 1.0f;
        const float x1 = ((b.x + b.w) / rw) * 2.0f - 1.0f;
        const float y0 = 1.0f - (b.y         / rh) * 2.0f;
        const float y1 = 1.0f - ((b.y + b.h) / rh) * 2.0f;

        // 8 vertices for 4 line pairs forming the rectangle outline
        const float verts[16] = {
            x0, y0,  x1, y0,   // top
            x1, y0,  x1, y1,   // right
            x1, y1,  x0, y1,   // bottom
            x0, y1,  x0, y0,   // left
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
bool DisplayOutputEgl::init() {
    if (!open_drm())  return false;
    if (!init_gbm())  return false;
    if (!init_egl())  return false;
    if (!init_gl())   return false;
    if (!m_bbox.start()) return false;

    DS_INFO("DisplayOutputEgl: ready  %ux%u  crtc=%u\n",
            m_mode.hdisplay, m_mode.vdisplay, m_crtc_id);
    return true;
}

// ---------------------------------------------------------------------------
// IDisplayOutput::update_frame
// ---------------------------------------------------------------------------
void DisplayOutputEgl::update_frame(std::shared_ptr<DecodedFrame> frame) {
    // Latest-wins: always overwrite; render loop detects change via pointer compare.
    std::lock_guard<std::mutex> lk(m_frame_mtx);
    m_pending_frame = std::move(frame);
}

// ---------------------------------------------------------------------------
// IDisplayOutput::present
// ---------------------------------------------------------------------------
bool DisplayOutputEgl::present() {
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

    glViewport(0, 0,
               static_cast<GLsizei>(m_mode.hdisplay),
               static_cast<GLsizei>(m_mode.vdisplay));
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    if (frame)
        draw_frame(*frame);

    if (bbox && frame)
        draw_bboxes(*bbox,
                    static_cast<int>(m_mode.hdisplay),
                    static_cast<int>(m_mode.vdisplay));

    // Non-blocking flip: issue immediately, don't wait for vblank.
    // page_flip_nowait() drains any already-complete events first (debug tracking),
    // then queues the next flip using a different GBM BO.
    eglSwapBuffers(m_egl_dpy, m_egl_surf);
    const bool ok = page_flip_nowait();

    m_last_frame_ptr = frame.get(); // mark as rendered regardless of flip result

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

    DS_TRACE("DisplayOutputEgl: display seq=%" PRIu64 " bbox_seq=%" PRIu64 "\n",
             frame ? frame->sei_frame_seq : 0ULL,
             bbox ? bbox->frame_seq : 0ULL);
    if (frame && frame->sei_frame_seq != 0)
        m_last_displayed_seq = frame->sei_frame_seq;

    emit_render_stats_if_due(now_mono);

    return ok;
}

// ---------------------------------------------------------------------------
void DisplayOutputEgl::emit_render_stats_if_due(uint64_t now_us) {
    if (m_cfg.debug.stats_interval_s == 0) return;
    const uint64_t interval_us =
        static_cast<uint64_t>(m_cfg.debug.stats_interval_s) * 1'000'000ULL;
    if (now_us - m_last_stats_us < interval_us) return;
    m_last_stats_us = now_us;

    char line[256];
    std::string msg = "[TIMING-RENDER EGL]\n";
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
    // flip_stats no longer used (flip is non-blocking); last_flip_complete shown in TRACE

    snprintf(line, sizeof(line), "  last_seq=%" PRIu64 "\n", m_last_displayed_seq);
    msg += line;

    DS_INFO("%s", msg.c_str());
}

// ---------------------------------------------------------------------------
// cleanup_gl
// ---------------------------------------------------------------------------
void DisplayOutputEgl::cleanup_gl() {
    if (m_tex)        { glDeleteTextures(1, &m_tex);        m_tex        = 0; }
    if (m_vbo_quad)   { glDeleteBuffers(1, &m_vbo_quad);    m_vbo_quad   = 0; }
    if (m_vbo_line)   { glDeleteBuffers(1, &m_vbo_line);    m_vbo_line   = 0; }
    if (m_prog_video) { glDeleteProgram(m_prog_video);      m_prog_video = 0; }
    if (m_prog_line)  { glDeleteProgram(m_prog_line);       m_prog_line  = 0; }
}

// ---------------------------------------------------------------------------
// IDisplayOutput::shutdown
// ---------------------------------------------------------------------------
void DisplayOutputEgl::shutdown() {
    m_bbox.stop();
    cleanup_gl();

    // Restore saved CRTC state (optional — leave display in known state)
    if (m_egl_dpy != EGL_NO_DISPLAY) {
        eglMakeCurrent(m_egl_dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if (m_egl_surf != EGL_NO_SURFACE)
            eglDestroySurface(m_egl_dpy, m_egl_surf);
        if (m_egl_ctx  != EGL_NO_CONTEXT)
            eglDestroyContext(m_egl_dpy, m_egl_ctx);
        eglTerminate(m_egl_dpy);
    }
    m_egl_dpy  = EGL_NO_DISPLAY;
    m_egl_ctx  = EGL_NO_CONTEXT;
    m_egl_surf = EGL_NO_SURFACE;

    if (m_gbm_surf) { gbm_surface_destroy(m_gbm_surf); m_gbm_surf = nullptr; }
    if (m_gbm_dev)  { gbm_device_destroy(m_gbm_dev);   m_gbm_dev  = nullptr; }

    // Release BO → FB cache
    for (auto& e : m_fb_cache) {
        if (e.fb_id) drmModeRmFB(m_drm_fd, e.fb_id);
        e = {};
    }

    if (m_own_drm && m_drm_fd >= 0) {
        drmDropMaster(m_drm_fd);
        ::close(m_drm_fd);
        m_drm_fd  = -1;
        m_own_drm = false;
    }
}
