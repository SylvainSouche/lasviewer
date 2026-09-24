// dem_tess_mesh.h — GPU-tessellated DEM/DSM mesh with height displacement.
//
// Requires an OpenGL 4.0+ context (tessellation shaders); macOS provides 4.1
// core. Design and rationale: docs/design-tessellation-displacement.md.
//
// Summary:
//   - The CPU builds an adaptive quadtree over the DEM (subdivide by angular
//     geometric error + texture span, balance pass so adjacent leaves differ
//     by at most one level) and emits one GL_PATCHES quad per leaf.
//   - Cells entirely nodata produce no patch; cells straddling a nodata edge
//     are subdivided further to localize it.
//   - Crack avoidance: edges shared with a same-level neighbor use the
//     continuous screen-space tessellation formula (both sides compute
//     identical values); edges at a ±1 level transition use a fixed,
//     level-derived tessellation level.
//   - The full-resolution heightmap is uploaded as an R32F texture holding
//     GL-space Y; the evaluation shader replaces each generated vertex's
//     interpolated height with the sampled one (patch corners are exact).
//   - Two UV sets: patchUVs (orthophoto-relative, for color) and
//     patchHeightUVs (DEM-raster-relative, for the heightmap). The ortho and
//     DEM generally cover different extents, so they must not be conflated.
#pragma once
#include "gl_platform.h"
#include "scene_frame.h"
#include <glm/glm.hpp>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct Orthophoto; // geotiff.h

// True if the current GL context supports tessellation shaders (GL >= 4.0).
// Call after glfwMakeContextCurrent().
bool demTessSupported();

struct DEMTessMesh {
    // --- CPU patch data (loadFromDEM; no GL context needed). Flat,
    //     non-indexed: 4 corner vertices per patch. ---
    std::vector<float> patchPositions;      // 3 floats/vertex
    std::vector<float> patchUVs;            // 2 floats/vertex, orthophoto-relative
    std::vector<float> patchHeightUVs;      // 2 floats/vertex, DEM-raster-relative
    // Per-patch edge constraint replicated to all 4 corners (x=bottom,
    // y=right, z=top, w=left): 0 = free tessellation, 1 = LOD transition.
    std::vector<float> patchEdgeConstraint; // 4 floats/vertex
    int patchCount = 0;

    // Heightmap in GL-space Y, kept only until uploadGPU().
    std::vector<float> heightmapGLSpace;
    int heightmapSrcW = 0, heightmapSrcH = 0;

    SceneFrame frame;
    glm::dvec3 bboxMin{0.0}, bboxMax{0.0};
    glm::vec2 glBBoxMin{0.0f}, glBBoxMax{0.0f};
    float glBBoxMinY = 0.0f, glBBoxMaxY = 0.0f;
    bool loaded = false;   // CPU data ready
    bool valid = false;    // GPU resources ready
    // False when the orthophoto has geo tags that do not overlap the DEM (even
    // after CRS reprojection): the mesh then uses the elevation ramp rather
    // than stretching unrelated imagery over it. An ortho with no geo tags at
    // all is deliberately stretched over the DEM extent instead.
    bool orthoUsable = true;

    // Build parameters of the current mesh. Both are load-time quadtree
    // parameters, so changing either needs a rebuild (requestBackgroundBuild).
    double collapseAngleDeg = 1.0; // angular geometric-error threshold
    int maxLevel = 5;              // quadtree depth ceiling (COARSE * 2^maxLevel per axis)

    GLuint vao = 0, posVBO = 0, uvVBO = 0, heightUVVBO = 0, edgeConstraintVBO = 0;
    GLuint heightmapTex = 0;
    GLuint colorTex = 0;   // orthophoto; 0 = elevation ramp
    bool showTexture = true; // false = elevation ramp even with a colorTex

    // Step 1 (no GL context needed). ortho may be null. cancelFlag, if set
    // during the build, makes it return false early.
    bool loadFromDEM(const std::string& path, const Orthophoto* ortho, const SceneFrame& frame,
                     double angleThresholdDeg = 1.0, int maxLevelParam = 5,
                     const std::atomic<bool>* cancelFlag = nullptr);

    // Step 2 (GL context current): upload patch buffers, heightmap, and (once)
    // the orthophoto color texture. `ortho` must be the one given to
    // loadFromDEM.
    bool uploadGPU(const Orthophoto* ortho);

    // Box-filter downsample to capTexels if needed, then (re)upload.
    bool uploadHeightmapTexture(const std::vector<float>& glSpaceData,
                                int srcW, int srcH, int capTexels);

    // tessProgram: shaders::linkTessProgram(kMeshTessVert, kMeshTessControl,
    // kMeshTessEval, kMeshFrag). fov in degrees. targetPixelsPerSegment sets
    // triangle density (smaller = denser).
    void render(GLuint tessProgram, const glm::mat4& V, const glm::mat4& P,
                const glm::vec3& camPosGL, float fov, float viewportH,
                float zScale, float targetPixelsPerSegment,
                bool useDisplacement, bool showMasterEdges) const;

    void releaseGeometryGL(); // everything except the color texture
    void destroy();

    // --- Background rebuild ---
    // requestBackgroundBuild() is non-blocking: it builds a separate
    // DEMTessMesh on a worker thread while this one keeps rendering.
    // pollBackgroundBuild() (call every frame, main thread) uploads and swaps
    // in a finished build. At most one worker runs; a newer request cancels
    // the running build and is started once it has exited (intermediate
    // requests coalesce into the latest).
    void requestBackgroundBuild(const std::string& path, const Orthophoto* ortho,
                                const SceneFrame& frame, double angleThresholdDeg,
                                int maxLevelParam);
    bool pollBackgroundBuild(const Orthophoto* ortho);
    bool backgroundBuildInProgress() const { return bgInProgress.load(); }

    ~DEMTessMesh();

private:
    void startBackgroundBuildNow(const std::string& path, const Orthophoto* ortho,
                                 const SceneFrame& frame, double angleThresholdDeg,
                                 int maxLevelParam);

    std::thread bgThread;
    std::mutex bgMutex;
    std::atomic<bool> bgInProgress{false};
    std::atomic<bool> bgHasResult{false};
    std::unique_ptr<DEMTessMesh> bgPending; // guarded by bgMutex
    // One flag per build (shared with its thread), so replacing it for a new
    // build doesn't un-cancel the old one.
    std::shared_ptr<std::atomic<bool>> bgCancelFlag;

    bool bgHasPendingRequest = false;
    std::string bgPendingPath;
    const Orthophoto* bgPendingOrtho = nullptr;
    SceneFrame bgPendingFrame;
    double bgPendingAngle = 1.0;
    int bgPendingMaxLevel = 5;
};
