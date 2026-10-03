// hiz.h — Hi-Z occlusion culling shared by the whole scene.
//
// After all layers have drawn, build() max-reduces the frame's depth buffer
// into a mip pyramid and reads one small level (~64x64) back to the CPU.
// Next frame, isOccluded() tests a GL-space box against it, so any layer's
// geometry (a DEM, dense tiles) can hide streamed tiles. One frame stale by
// design: the only failure mode is a newly revealed box missing for a frame.
#pragma once
#include "gl_platform.h"
#include <glm/glm.hpp>
#include <vector>

class HiZ {
public:
    bool init();
    void destroy();
    // Default framebuffer depth → pyramid → CPU readback.
    void build(int viewportW, int viewportH);
    bool ready() const { return hizReady_; }
    // Forget the pyramid (when a frame was drawn without building one).
    void invalidate() { hizReady_ = false; }
    // lo/hi: GL-space box with the Z exaggeration already applied.
    bool isOccluded(const glm::vec3& lo, const glm::vec3& hi, const glm::mat4& VP) const;

private:
    void resizeHiZIfNeeded(int viewportW, int viewportH);

    GLuint hizCopyProgram_ = 0, hizDownsampleProgram_ = 0;
    GLuint hizFullscreenVAO_ = 0;
    GLuint hizFBO_ = 0;
    GLuint hizDepthCaptureTex_ = 0; // GL_DEPTH_COMPONENT32F, blit target
    GLuint hizPyramidTex_ = 0;      // GL_R32F, mipmapped
    int hizCaptureW_ = 0, hizCaptureH_ = 0;
    int hizLevels_ = 0;
    int hizReadLevel_ = 0;
    int hizReadW_ = 0, hizReadH_ = 0;
    std::vector<float> hizReadback_;
    bool hizReady_ = false;
};

// True when the GL-space box is entirely outside one clip plane.
bool aabbOutsideFrustum(const glm::vec3& lo, const glm::vec3& hi, const glm::mat4& VP);
