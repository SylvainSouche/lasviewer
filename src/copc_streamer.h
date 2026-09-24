// copc_streamer.h — Async COPC tile streaming with distance-based LOD
#pragma once
#include "gl_platform.h"
#include <glm/glm.hpp>
#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <thread>

enum class TileState { UNLOADED, REQUESTED, LOADING, LOADED, FAILED, EVICTING };

struct Tile {
    double minX, minY, minZ, maxX, maxY, maxZ;
    int gx, gy;
    glm::vec3 glMin, glMax, glCenter;
    float glRadius = 0.0f;
    TileState state = TileState::UNLOADED;
    float loadedResolution = 0.0f;
    double lastUsedTime = 0.0;
    GLuint vao = 0, vboPos = 0, vboCol = 0;
    GLsizei pointCount = 0;
    glm::vec3 principalAxis1{0,1,0}, principalAxis2{1,0,0}, principalAxis3{0,0,1};
    float obbExtent1 = 0, obbExtent2 = 0, obbExtent3 = 0;
    float pointSpacing = 0.01f;
    std::vector<float> positions, colors;
};

struct Orthophoto; // forward decl from geotiff.h

struct TileGrid {
    std::vector<Tile> tiles;
    int gridX = 8, gridY = 8;
    double tileW, tileH;
    double worldCenterX, worldCenterY, worldCenterZ, worldScale;
    int maxConcurrentLoads = 16;
    int pendingLoads = 0;
    bool occlusionInit = false;
    GLuint occVAO = 0, occVBO = 0;
    std::vector<GLuint> occlusionQueries;

    // --- Hi-Z occlusion culling for streamed tiles ---
    // Replaces the disabled query-based scaffolding above (occlusionInit/
    // occlusionQueries — allocated but never issued; caused tile
    // flickering when tried, per specs.md §5.6, due to occlusion-query
    // result latency: the GPU answer for "was this tile visible" often
    // isn't available until a LATER frame, so you end up deciding THIS
    // frame's visibility from a stale answer to a question about a
    // different camera position).
    //
    // Mechanism instead: build a conservative MAX-reduced depth mip
    // pyramid (Hi-Z) from THIS frame's own rendered depth, then use it on
    // the NEXT frame to skip tiles whose entire screen footprint is
    // already known to be behind closer geometry. This is deliberately
    // one-frame-stale too — but a FIXED, deterministic one-frame offset
    // (not an unbounded, driver-dependent query latency), and safe in a
    // specific sense: the only failure mode is a tile that SHOULD be
    // visible this frame not being drawn for one frame during fast camera
    // motion (self-corrects immediately next frame, since that frame's
    // own depth updates regardless of what got culled) — never a tile
    // that's actually occluded being wrongly drawn (that would just be
    // wasted work, not a visible bug, same as no culling at all).
    //
    // Since this app's DEM and point-cloud-streaming render paths are
    // mutually exclusive (see main.cpp — only one is ever active), the
    // only thing tiles can occlude here is OTHER TILES, not DEM geometry.
    //
    // Deliberately reads back only a small, FIXED-SIZE coarse mip level
    // (~64x64, see kHizTargetReadSize in copc_streamer.cpp) ONCE per
    // frame, rather than doing a proper GPU-side per-tile test — this
    // architecture decides per-tile draw/skip on the CPU (a simple loop
    // issuing glDrawArrays calls), and a GPU->CPU readback for every one
    // of gridX*gridY tiles individually would stall far more than it
    // saves. One small, bounded transfer per frame trades some spatial
    // precision (each readback texel can cover a fair few real tiles'
    // worth of screen space) for keeping the CPU-driven draw loop cheap.
    GLuint hizCopyProgram = 0, hizDownsampleProgram = 0;
    GLuint hizFullscreenVAO = 0;
    GLuint hizFBO = 0;
    GLuint hizDepthCaptureTex = 0; // GL_DEPTH_COMPONENT32F, blit target from the default framebuffer
    GLuint hizPyramidTex = 0;      // GL_R32F, mipmapped — the actual Hi-Z pyramid
    int hizCaptureW = 0, hizCaptureH = 0; // mip 0 size, tracks viewport; resized on change
    int hizLevels = 0;
    int hizReadLevel = 0;
    int hizReadW = 0, hizReadH = 0;  // the coarse mip actually read back
    std::vector<float> hizReadback;  // CPU copy of hizReadLevel, from the LAST successful build
    bool hizInitAttempted = false;
    bool hizReady = false; // false until the first successful build — never test against garbage

    void initHiZ();
    void resizeHiZIfNeeded(int viewportW, int viewportH);
    void captureAndBuildHiZ(int viewportW, int viewportH);
    bool isTileOccludedByHiZ(const Tile& t, const glm::mat4& VP) const;
    void destroyHiZ();

    // Tile loader thread.
    struct Request { Tile* tile; double resolution; double minX,minY,minZ,maxX,maxY,maxZ; };
    std::queue<Request> requests;
    std::queue<Tile*> results;
    std::mutex mtx;
    std::condition_variable cv;
    bool shutdown = false;
    std::thread worker;
    std::string copcPath;
    const Orthophoto* orthoPtr = nullptr;

    void init(const std::string& copcPath,
              double minX, double minY, double minZ,
              double maxX, double maxY, double maxZ,
              const Orthophoto* ortho);
    ~TileGrid();
    void uploadTile(Tile& t);
    double desiredResolution(const Tile& t, const glm::vec3& camPos, float fov, float viewportH);
    void update(const glm::vec3& camPos, float fov, float viewportH, double currentTime);
    void render(GLuint pointProgram, const glm::mat4& V, const glm::mat4& P,
                const glm::vec3& camPos, float fov, float viewportW, float viewportH,
                float zScale, float pointSizeMul, double currentTime,
                bool useOcclusion);
    void stop();
    void loaderRun();
};
