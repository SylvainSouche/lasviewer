// copc_streamer.h — async COPC tile streaming with apparent-size-driven LOD.
//
// The loader thread opens the file once with copc-lib, keeps its octree node
// list, and answers each tile request from the nodes that intersect the tile
// at the depth matching the requested resolution.
//
// Threading model: the main thread owns every Tile field. The loader thread
// only ever sees an immutable LoadRequest (a copy of the tile's bounds) and
// hands back a LoadResult by value; the main thread applies results in
// update(). No Tile is touched from two threads.
#pragma once
#include "gl_platform.h"
#include "scene_frame.h"

#include <glm/glm.hpp>

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct Orthophoto;    // raster.h
struct RenderContext; // layer.h

struct Tile {
    double minX, minY, minZ, maxX, maxY, maxZ;
    int gx, gy;
    glm::vec3 glMin, glMax, glCenter;
    float glRadius = 0.0f;

    // Main-thread-only state.
    bool inFlight = false;         // a load request is queued or running
    int failures = 0;              // consecutive failed loads (backs off resolution)
    float loadedResolution = 0.0f; // 0 = nothing loaded
    GLuint vao = 0, vboPos = 0, vboCol = 0, vboOrthoCol = 0; // vboOrthoCol: 0 without ortho
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
    SceneFrame frame;
    bool useOrthoColors = false;
    int maxConcurrentLoads = 16;
    int pendingLoads = 0;

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
        std::vector<float> positions, colors, orthoColors;
    };
    std::deque<LoadRequest> requests;
    std::deque<LoadResult> results;
    std::mutex mtx;
    std::condition_variable cv;
    bool shutdown = false;
    std::thread worker;
    std::string copcPath;
    double fileMaxX = 0, fileMaxY = 0; // header extent: the last tiles include their max edge
    struct CopcSource;                 // copc-lib reader + node list (loader thread only)
    std::unique_ptr<CopcSource> source;
    const Orthophoto* orthoPtr = nullptr;

    // bounds: the file's header extent. ortho may be null (elevation colors
    // only); it must outlive the grid.
    void init(const std::string& copcPath, const WorldBounds& bounds, const Orthophoto* ortho,
              const SceneFrame& frame);
    TileGrid(); // out of line: CopcSource is only complete in the .cpp
    ~TileGrid();
    void requestLoad(int tileIndex, double resolution);
    void uploadTile(Tile& t, LoadResult& r);
    void releaseTileGL(Tile& t);
    double desiredResolution(const Tile& t, const glm::vec3& camPos, float fov, float viewportH);
    void update(const glm::vec3& camPos, float fov, float viewportH);
    void render(const RenderContext& ctx);
    void setUseOrthoColors(bool useOrtho);

    // Last render()'s counts.
    size_t drawnTiles = 0, drawnPoints = 0, culledTiles = 0;
    void stop();
    void loaderRun();
    LoadResult loadTile(const LoadRequest& req); // loader thread
};
