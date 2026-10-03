// dem_tess_mesh.h — GPU-tessellated DEM/DSM mesh with height displacement.
//
// Requires an OpenGL 4.0+ context (tessellation shaders); macOS provides 4.1
// core. Design and rationale: docs/design-tessellation-displacement.md.
//
// Summary (design doc §6v):
//   - The CPU builds an adaptive quadtree over the DEM (dem_quadtree.h): a
//     cell stays whole when the flat patch through its corners is within the
//     collapse angle of every DEM pixel inside it and spans at most 64
//     pixels; leaves are balanced to one level apart along whole edges and
//     emitted as one GL_PATCHES quad each, with per-edge codes.
//   - Cells entirely nodata produce no patch; cells straddling a nodata edge
//     are subdivided down to the maximum level.
//   - GPU tessellation splits edges by on-screen length, capped at the DEM
//     pixels they span; at a level transition the coarse side uses twice the
//     segments of each fine half edge, so the two sides share every vertex.
//   - The heightmap is an RG32F texture: GL-space heights (nodata filled
//     with the nearest valid height) and validity. Every generated vertex
//     takes its height from it, so it sits on the DEM; the fragment shader
//     discards nodata and can hill-shade from its gradient.
//   - Two UV sets: patchUVs (orthophoto-relative, for color) and
//     patchHeightUVs (DEM-raster-relative, for the heightmap). The ortho and
//     DEM generally cover different extents, so they must not be conflated.
//   - An optional auxiliary raster on the DEM's grid (see DemSourceData)
//     lets the fragment shader relate the surface to another one: cut an
//     above-ground surface below a height, or tell a ground surface where
//     something stands on it.
#pragma once
#include "gl_platform.h"
#include "raster.h"
#include "scene_frame.h"

#include <glm/glm.hpp>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// What a mesh is built from: the elevation raster, plus an optional
// auxiliary raster on exactly the same grid, in meters (empty = none):
//   DemAux::Ground        aux = the ground elevation under this surface
//   DemAux::HeightAbove   aux = the height of what stands on this surface
struct DemSourceData {
    DemRaster dem;
    std::vector<float> aux;
};
// Produces the data; called again for every rebuild, possibly on a worker
// thread, so it must not touch GL or shared mutable state.
using DemSource = std::function<bool(DemSourceData&)>;

enum class DemAux {
    None,
    Ground,      // above-ground surface: fragments less than `threshold` above aux are cut
    HeightAbove, // ground surface: the orthophoto is not drawn where aux > `threshold`
};

// Per-draw appearance.
struct DemStyle {
    float opacity = 1.0f; // < 1: blended (the caller sets the blend state)
    DemAux auxMode = DemAux::None;
    float threshold = 0.0f; // meters, see DemAux
    bool shade = true;      // hill-shading from the heightmap gradient
};

// True if the current GL context supports tessellation shaders (GL >= 4.0).
// Call after glfwMakeContextCurrent().
bool demTessSupported();

struct DEMTessMesh {
    // --- CPU patch data (loadFromDEM; no GL context needed). Flat,
    //     non-indexed: 4 corner vertices per patch. ---
    std::vector<float> patchPositions; // 3 floats/vertex
    std::vector<float> patchUVs;       // 2 floats/vertex, orthophoto-relative
    std::vector<float> patchHeightUVs; // 2 floats/vertex, DEM-raster-relative
    // Per-patch edge constraint replicated to all 4 corners (x=bottom,
    // y=right, z=top, w=left): 0 = free tessellation, 1 = LOD transition.
    std::vector<float> patchEdgeConstraint; // 4 floats/vertex
    int patchCount = 0;

    // Heightmap in GL-space Y, and the auxiliary raster (meters, same grid),
    // kept only until uploadGPU().
    std::vector<float> heightmapGLSpace;
    std::vector<float> heightmapValid; // 1 = data, 0 = nodata (filled)
    std::vector<float> auxData;
    int heightmapSrcW = 0, heightmapSrcH = 0;
    int heightmapTexW = 0, heightmapTexH = 0; // uploaded size (after any downsample)
    double demPixelW = 1.0, demPixelH = 1.0;  // world size of one DEM pixel

    SceneFrame frame;
    glm::dvec3 bboxMin{0.0}, bboxMax{0.0};
    glm::vec2 glBBoxMin{0.0f}, glBBoxMax{0.0f};
    float glBBoxMinY = 0.0f, glBBoxMaxY = 0.0f;
    bool loaded = false; // CPU data ready
    bool valid = false;  // GPU resources ready
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
    GLuint auxTex = 0;       // 0 = no auxiliary raster
    GLuint colorTex = 0;     // orthophoto; 0 = elevation ramp
    bool showTexture = true; // false = elevation ramp even with a colorTex

    // Step 1 (no GL context needed). ortho may be null. cancelFlag, if set
    // during the build, makes it return false early.
    bool loadFromDEM(const DemSource& source, const Orthophoto* ortho, const SceneFrame& frame,
                     double angleThresholdDeg = 1.0, int maxLevelParam = 5,
                     const std::atomic<bool>* cancelFlag = nullptr);

    // Step 2 (GL context current): upload patch buffers, heightmap, and (once)
    // the orthophoto color texture. `ortho` must be the one given to
    // loadFromDEM.
    bool uploadGPU(const Orthophoto* ortho);

    // Box-filter downsample to capTexels if needed, then (re)upload.
    bool uploadHeightmapTexture(const std::vector<float>& glSpaceData,
                                const std::vector<float>& validData, int srcW, int srcH,
                                int capTexels);
    bool hasAux() const { return auxTex != 0; }

    // tessProgram: shaders::linkTessProgram(kMeshTessVert, kMeshTessControl,
    // kMeshTessEval, kMeshFrag). fov in degrees. targetPixelsPerSegment sets
    // triangle density (smaller = denser).
    void render(GLuint tessProgram, const glm::mat4& V, const glm::mat4& P,
                const glm::vec3& camPosGL, float fov, float viewportH, float zScale,
                float targetPixelsPerSegment, bool useDisplacement, bool showMasterEdges,
                const DemStyle& style = {}) const;

    void releaseGeometryGL(); // everything except the color texture
    void destroy();

    // --- Background rebuild ---
    // requestBackgroundBuild() is non-blocking: it builds a separate
    // DEMTessMesh on a worker thread while this one keeps rendering.
    // pollBackgroundBuild() (call every frame, main thread) uploads and swaps
    // in a finished build. At most one worker runs; a newer request cancels
    // the running build and is started once it has exited (intermediate
    // requests coalesce into the latest).
    void requestBackgroundBuild(const DemSource& source, const Orthophoto* ortho,
                                const SceneFrame& frame, double angleThresholdDeg,
                                int maxLevelParam);
    bool pollBackgroundBuild(const Orthophoto* ortho);
    bool backgroundBuildInProgress() const { return bgInProgress_.load(); }

    ~DEMTessMesh();

  private:
    void startBackgroundBuildNow(const DemSource& source, const Orthophoto* ortho,
                                 const SceneFrame& frame, double angleThresholdDeg,
                                 int maxLevelParam);

    std::thread bgThread_;
    std::mutex bgMutex_;
    std::atomic<bool> bgInProgress_{false};
    std::atomic<bool> bgHasResult_{false};
    std::unique_ptr<DEMTessMesh> bgPending_; // guarded by bgMutex_
    // One flag per build (shared with its thread), so replacing it for a new
    // build doesn't un-cancel the old one.
    std::shared_ptr<std::atomic<bool>> bgCancelFlag_;

    bool bgHasPendingRequest_ = false;
    DemSource bgPendingSource_;
    const Orthophoto* bgPendingOrtho_ = nullptr;
    SceneFrame bgPendingFrame_;
    double bgPendingAngle_ = 1.0;
    int bgPendingMaxLevel_ = 5;
};
