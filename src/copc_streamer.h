// copc_streamer.h — async COPC tile streaming with apparent-size-driven LOD.
//
// Threading model: the main thread owns every Tile field. The loader thread
// only ever sees an immutable LoadRequest (a copy of the tile's bounds) and
// hands back a LoadResult by value; the main thread applies results in
// update(). No Tile is touched from two threads.
#pragma once
#include "gl_platform.h"
#include <glm/glm.hpp>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct Orthophoto; // geotiff.h

struct Tile {
    double minX, minY, minZ, maxX, maxY, maxZ;
    int gx, gy;
    glm::vec3 glMin, glMax, glCenter;
    float glRadius = 0.0f;

    // Main-thread-only state.
    bool inFlight = false;          // a load request is queued or running
    int failures = 0;               // consecutive failed loads (backs off resolution)
    float loadedResolution = 0.0f;  // 0 = nothing loaded
    GLuint vao = 0, vboPos = 0, vboCol = 0;
    GLsizei pointCount = 0;

    // Per-tile PCA (oriented bounding box), drives desiredResolution().
    glm::vec3 principalAxis1{0, 1, 0}, principalAxis2{1, 0, 0}, principalAxis3{0, 0, 1};
    float obbExtent1 = 0, obbExtent2 = 0, obbExtent3 = 0;
    float pointSpacing = 0.01f;

    bool hasGeometry() const { return vao != 0 && pointCount > 0; }
};

struct TileGrid {
    std::vector<Tile> tiles;
    int gridX = 8, gridY = 8;
    double tileW = 0, tileH = 0;
    double worldCenterX = 0, worldCenterY = 0, worldCenterZ = 0, worldScale = 1;
    double colorZMin = 0, colorZMax = 1; // elevation-ramp range (world units)
    int maxConcurrentLoads = 16;
    int pendingLoads = 0;

    // --- Hi-Z occlusion culling (see captureAndBuildHiZ) ---
    // Each frame's depth is max-reduced into a mip pyramid; a small (~64x64)
    // level is read back once and used on the NEXT frame to skip tiles whose
    // whole screen footprint lies behind known geometry. One frame stale by
    // design: the only failure mode is a newly-disoccluded tile missing for
    // one frame.
    GLuint hizCopyProgram = 0, hizDownsampleProgram = 0;
    GLuint hizFullscreenVAO = 0;
    GLuint hizFBO = 0;
    GLuint hizDepthCaptureTex = 0; // GL_DEPTH_COMPONENT32F, blit target
    GLuint hizPyramidTex = 0;      // GL_R32F, mipmapped
    int hizCaptureW = 0, hizCaptureH = 0;
    int hizLevels = 0;
    int hizReadLevel = 0;
    int hizReadW = 0, hizReadH = 0;
    std::vector<float> hizReadback;
    bool hizReady = false;

    void initHiZ();
    void resizeHiZIfNeeded(int viewportW, int viewportH);
    void captureAndBuildHiZ(int viewportW, int viewportH);
    bool isTileOccludedByHiZ(const Tile& t, const glm::mat4& VP) const;
    void destroyHiZ();

    // --- Loader thread ---
    struct LoadRequest {
        int tileIndex;
        double resolution;
        double minX, minY, minZ, maxX, maxY, maxZ;
    };
    struct LoadResult {
        int tileIndex;
        double resolution;
        bool ok = false;
        std::vector<float> positions, colors;
    };
    std::deque<LoadRequest> requests;
    std::deque<LoadResult> results;
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
    void requestLoad(int tileIndex, double resolution);
    void uploadTile(Tile& t, LoadResult& r);
    void releaseTileGL(Tile& t);
    double desiredResolution(const Tile& t, const glm::vec3& camPos, float fov, float viewportH);
    void update(const glm::vec3& camPos, float fov, float viewportH);
    void render(GLuint pointProgram, const glm::mat4& V, const glm::mat4& P,
                const glm::vec3& camPos, float fov, float viewportW, float viewportH,
                float zScale, float pointSizeMul, bool useOcclusion);
    void stop();
    void loaderRun();
    LoadResult loadTile(const LoadRequest& req) const;
};
