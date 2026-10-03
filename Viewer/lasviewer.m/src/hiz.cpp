// hiz.cpp — Hi-Z (max-depth mip pyramid) occlusion culling.
#include "hiz.h"
#include "shaders.h"

#include <algorithm>
#include <iostream>

bool HiZ::init() {
    hizCopyProgram_ = shaders::linkProgram(shaders::kHiZVert, shaders::kHiZCopyFrag);
    hizDownsampleProgram_ = shaders::linkProgram(shaders::kHiZVert, shaders::kHiZDownsampleFrag);
    if (!hizCopyProgram_ || !hizDownsampleProgram_) {
        std::cerr << "[occlusion] Hi-Z shader setup failed — occlusion culling disabled"
                  << std::endl;
        if (hizCopyProgram_) { glDeleteProgram(hizCopyProgram_); hizCopyProgram_ = 0; }
        if (hizDownsampleProgram_) { glDeleteProgram(hizDownsampleProgram_); hizDownsampleProgram_ = 0; }
        return false;
    }
    glGenVertexArrays(1, &hizFullscreenVAO_);
    glGenFramebuffers(1, &hizFBO_);
    return true;
}

// Approximate size of the one mip level read back to the CPU per frame.
static const int kHiZTargetReadSize = 64;

void HiZ::resizeHiZIfNeeded(int viewportW, int viewportH) {
    if (!hizCopyProgram_ || !hizDownsampleProgram_) return; // init failed — disabled
    if (viewportW <= 0 || viewportH <= 0) return;
    if (viewportW == hizCaptureW_ && viewportH == hizCaptureH_ && hizPyramidTex_ != 0) {
        return; // already correctly sized
    }
    hizCaptureW_ = viewportW;
    hizCaptureH_ = viewportH;

    if (hizDepthCaptureTex_) { glDeleteTextures(1, &hizDepthCaptureTex_); hizDepthCaptureTex_ = 0; }
    glGenTextures(1, &hizDepthCaptureTex_);
    glBindTexture(GL_TEXTURE_2D, hizDepthCaptureTex_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, viewportW, viewportH, 0,
                 GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    if (hizPyramidTex_) { glDeleteTextures(1, &hizPyramidTex_); hizPyramidTex_ = 0; }
    glGenTextures(1, &hizPyramidTex_);
    glBindTexture(GL_TEXTURE_2D, hizPyramidTex_);

    int levels = 1;
    int w = viewportW, h = viewportH;
    while ((w > kHiZTargetReadSize || h > kHiZTargetReadSize) && levels < 16) {
        w = std::max(1, w / 2);
        h = std::max(1, h / 2);
        ++levels;
    }
    hizLevels_ = levels;
    hizReadLevel_ = levels - 1;
    hizReadW_ = std::max(1, viewportW >> hizReadLevel_);
    hizReadH_ = std::max(1, viewportH >> hizReadLevel_);

    for (int lvl = 0; lvl < levels; ++lvl) {
        int lw = std::max(1, viewportW >> lvl);
        int lh = std::max(1, viewportH >> lvl);
        glTexImage2D(GL_TEXTURE_2D, lvl, GL_R32F, lw, lh, 0, GL_RED, GL_FLOAT, nullptr);
    }
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, levels - 1);

    // 1.0 = far/empty — a conservative "nothing occluded yet" default
    // until the first real build completes (hizReady_ stays false until
    // then regardless, but this avoids the vector holding uninitialized
    // memory in the meantime).
    hizReadback_.assign(static_cast<size_t>(hizReadW_) * hizReadH_, 1.0f);
    hizReady_ = false;
    std::cerr << "[occlusion] Hi-Z pyramid (re)sized: " << viewportW << "x" << viewportH
              << ", " << levels << " levels, CPU readback at "
              << hizReadW_ << "x" << hizReadH_ << std::endl;
}

void HiZ::build(int viewportW, int viewportH) {
    if (!hizCopyProgram_ || !hizDownsampleProgram_) return; // init failed — disabled
    resizeHiZIfNeeded(viewportW, viewportH);
    if (!hizPyramidTex_ || !hizDepthCaptureTex_ || !hizFBO_) return;

    // Runs after all scene drawing; save and restore the GL state it touches
    // so later passes (overlays, UI, picking) are unaffected.
    GLint prevViewport[4];
    glGetIntegerv(GL_VIEWPORT, prevViewport);
    GLint prevDrawFBO = 0, prevReadFBO = 0;
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &prevDrawFBO);
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &prevReadFBO);
    GLboolean depthTestWasEnabled = glIsEnabled(GL_DEPTH_TEST);
    GLboolean depthMaskWas = GL_TRUE;
    glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMaskWas);
    GLint prevProgram = 0;
    glGetIntegerv(GL_CURRENT_PROGRAM, &prevProgram);

    // Step 1: blit the real depth buffer — the default framebuffer (this
    // app renders directly to it, never through an intermediate FBO) —
    // into hizDepthCaptureTex_.
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, hizFBO_);
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                           GL_TEXTURE_2D, hizDepthCaptureTex_, 0);
    GLenum drawBufNone = GL_NONE;
    glDrawBuffers(1, &drawBufNone); // depth-only target this pass, no color buffer
    glBlitFramebuffer(0, 0, viewportW, viewportH, 0, 0, viewportW, viewportH,
                      GL_DEPTH_BUFFER_BIT, GL_NEAREST);

    // Step 2: copy pass — depth texture -> mip 0 of the Hi-Z pyramid.
    glBindFramebuffer(GL_FRAMEBUFFER, hizFBO_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, 0, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D, hizPyramidTex_, 0);
    GLenum drawBufColor0 = GL_COLOR_ATTACHMENT0;
    glDrawBuffers(1, &drawBufColor0);
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glViewport(0, 0, viewportW, viewportH);
    glUseProgram(hizCopyProgram_);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, hizDepthCaptureTex_);
    glUniform1i(glGetUniformLocation(hizCopyProgram_, "uSrcDepth"), 0);
    glBindVertexArray(hizFullscreenVAO_);
    glDrawArrays(GL_TRIANGLES, 0, 3);

    // Step 3: successive 2x2 max-reduction passes, each sampling the
    // PREVIOUS Hi-Z mip (never the raw depth texture directly) and
    // rendering into the next.
    glUseProgram(hizDownsampleProgram_);
    glBindTexture(GL_TEXTURE_2D, hizPyramidTex_);
    glUniform1i(glGetUniformLocation(hizDownsampleProgram_, "uSrcMip"), 0);
    GLint texelSizeLoc = glGetUniformLocation(hizDownsampleProgram_, "uSrcTexelSize");
    for (int lvl = 1; lvl <= hizReadLevel_; ++lvl) {
        int srcW = std::max(1, viewportW >> (lvl - 1));
        int srcH = std::max(1, viewportH >> (lvl - 1));
        int dstW = std::max(1, viewportW >> lvl);
        int dstH = std::max(1, viewportH >> lvl);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, hizPyramidTex_, lvl);
        glViewport(0, 0, dstW, dstH);
        glUniform2f(texelSizeLoc, 1.0f / srcW, 1.0f / srcH);
        // Restrict the sampler to read ONLY the source mip for this pass
        // — we're simultaneously rendering into a DIFFERENT mip of the
        // same texture, and sampling the full mip range while doing so
        // (which would include the mip currently being written) is
        // undefined behavior.
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, lvl - 1);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, lvl - 1);
        glDrawArrays(GL_TRIANGLES, 0, 3);
    }
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, hizLevels_ - 1);

    // Step 4: read back JUST the small coarse target level — once per
    // frame, not per tile (see the class comment for why).
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D, hizPyramidTex_, hizReadLevel_);
    hizReadback_.resize(static_cast<size_t>(hizReadW_) * hizReadH_);
    glReadPixels(0, 0, hizReadW_, hizReadH_, GL_RED, GL_FLOAT, hizReadback_.data());
    hizReady_ = true;

    // Restore everything.
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, static_cast<GLuint>(prevDrawFBO));
    glBindFramebuffer(GL_READ_FRAMEBUFFER, static_cast<GLuint>(prevReadFBO));
    glViewport(prevViewport[0], prevViewport[1], prevViewport[2], prevViewport[3]);
    if (depthTestWasEnabled) glEnable(GL_DEPTH_TEST); else glDisable(GL_DEPTH_TEST);
    glDepthMask(depthMaskWas);
    glUseProgram(static_cast<GLuint>(prevProgram));
    glBindVertexArray(0);
}

// True only when the box's whole screen footprint lies behind the farthest
// depth recorded there last frame.
bool HiZ::isOccluded(const glm::vec3& lo, const glm::vec3& hi, const glm::mat4& VP) const {
    if (!hizReady_ || hizReadback_.empty()) return false; // nothing to test against yet

    const glm::vec3 corners[8] = {
        {lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z}, {lo.x, hi.y, lo.z}, {hi.x, hi.y, lo.z},
        {lo.x, lo.y, hi.z}, {hi.x, lo.y, hi.z}, {lo.x, hi.y, hi.z}, {hi.x, hi.y, hi.z},
    };

    float minNdcX = 1e30f, maxNdcX = -1e30f, minNdcY = 1e30f, maxNdcY = -1e30f;
    float nearestNdcZ = 1e30f;
    for (const auto& c : corners) {
        glm::vec4 clip = VP * glm::vec4(c, 1.0f);
        if (clip.w <= 1e-5f) {
            // Behind the camera or at/near the eye — projecting this
            // corner to NDC would be meaningless (or blow up). Bail out
            // of the test entirely rather than risk a wrong cull; just
            // render the tile normally.
            return false;
        }
        float ndcX = clip.x / clip.w;
        float ndcY = clip.y / clip.w;
        float ndcZ = clip.z / clip.w;
        minNdcX = std::min(minNdcX, ndcX); maxNdcX = std::max(maxNdcX, ndcX);
        minNdcY = std::min(minNdcY, ndcY); maxNdcY = std::max(maxNdcY, ndcY);
        nearestNdcZ = std::min(nearestNdcZ, ndcZ); // smaller = nearer (standard depth range)
    }

    // Off screen: a frustum question, not an occlusion one.
    if (maxNdcX < -1.0f || minNdcX > 1.0f || maxNdcY < -1.0f || minNdcY > 1.0f) {
        return false;
    }

    // NDC [-1,1] -> depth-buffer [0,1] (standard depth range) -> readback
    // pixel indices.
    float depthNear = nearestNdcZ * 0.5f + 0.5f;
    float u0 = std::clamp(minNdcX * 0.5f + 0.5f, 0.0f, 1.0f);
    float u1 = std::clamp(maxNdcX * 0.5f + 0.5f, 0.0f, 1.0f);
    float v0 = std::clamp(minNdcY * 0.5f + 0.5f, 0.0f, 1.0f);
    float v1 = std::clamp(maxNdcY * 0.5f + 0.5f, 0.0f, 1.0f);
    int px0 = std::clamp(static_cast<int>(u0 * hizReadW_), 0, hizReadW_ - 1);
    int px1 = std::clamp(static_cast<int>(u1 * hizReadW_), 0, hizReadW_ - 1);
    int py0 = std::clamp(static_cast<int>(v0 * hizReadH_), 0, hizReadH_ - 1);
    int py1 = std::clamp(static_cast<int>(v1 * hizReadH_), 0, hizReadH_ - 1);

    // Farthest known depth across every readback texel the tile's screen
    // footprint touches — the conservative (MAX) value the whole region
    // must be farther than for a safe cull.
    float farthestKnown = 0.0f;
    for (int py = py0; py <= py1; ++py) {
        for (int px = px0; px <= px1; ++px) {
            farthestKnown = std::max(farthestKnown, hizReadback_[static_cast<size_t>(py) * hizReadW_ + px]);
        }
    }

    return depthNear > farthestKnown;
}

bool aabbOutsideFrustum(const glm::vec3& lo, const glm::vec3& hi, const glm::mat4& VP) {
    // Outside if all 8 corners are beyond the same clip plane.
    const glm::vec3 corners[8] = {
        {lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z}, {lo.x, hi.y, lo.z}, {hi.x, hi.y, lo.z},
        {lo.x, lo.y, hi.z}, {hi.x, lo.y, hi.z}, {lo.x, hi.y, hi.z}, {hi.x, hi.y, hi.z},
    };
    int outside[6] = {0, 0, 0, 0, 0, 0};
    for (const auto& c : corners) {
        glm::vec4 p = VP * glm::vec4(c, 1.0f);
        if (p.x < -p.w) ++outside[0];
        if (p.x >  p.w) ++outside[1];
        if (p.y < -p.w) ++outside[2];
        if (p.y >  p.w) ++outside[3];
        if (p.z < -p.w) ++outside[4];
        if (p.z >  p.w) ++outside[5];
    }
    for (int n : outside) if (n == 8) return true;
    return false;
}

void HiZ::destroy() {
    if (hizCopyProgram_) { glDeleteProgram(hizCopyProgram_); hizCopyProgram_ = 0; }
    if (hizDownsampleProgram_) { glDeleteProgram(hizDownsampleProgram_); hizDownsampleProgram_ = 0; }
    if (hizFullscreenVAO_) { glDeleteVertexArrays(1, &hizFullscreenVAO_); hizFullscreenVAO_ = 0; }
    if (hizFBO_) { glDeleteFramebuffers(1, &hizFBO_); hizFBO_ = 0; }
    if (hizDepthCaptureTex_) { glDeleteTextures(1, &hizDepthCaptureTex_); hizDepthCaptureTex_ = 0; }
    if (hizPyramidTex_) { glDeleteTextures(1, &hizPyramidTex_); hizPyramidTex_ = 0; }
    hizReady_ = false;
}

