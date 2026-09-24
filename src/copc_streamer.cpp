// copc_streamer.cpp — async COPC tile streaming.
//
// A background loader thread turns LoadRequests into LoadResults (PDAL
// bounds+resolution query, GL-space transform, coloring). The main thread
// drains results in update(), uploads them, and requests refinements based
// on each tile's projected on-screen size.
#include "copc_streamer.h"
#include "geotiff.h"
#include "shaders.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Options.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/Stage.hpp>
#include <pdal/Dimension.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

static glm::vec3 elevationColorRamp(float t) {
    t = glm::clamp(t, 0.0f, 1.0f);
    static const glm::vec3 stops[5] = {
        {0.1f, 0.2f, 0.8f}, {0.2f, 0.7f, 0.9f}, {0.3f, 0.8f, 0.3f},
        {0.95f, 0.85f, 0.2f}, {0.85f, 0.25f, 0.2f}
    };
    float s = t * 4.0f;
    int si = static_cast<int>(s);
    if (si >= 4) return stops[4];
    return glm::mix(stops[si], stops[si + 1], s - si);
}

// ===========================================================================
// Loader thread
// ===========================================================================

TileGrid::LoadResult TileGrid::loadTile(const LoadRequest& req) const {
    LoadResult res;
    res.tileIndex = req.tileIndex;
    res.resolution = req.resolution;

    pdal::StageFactory factory;
    std::string driver = factory.inferReaderDriver(copcPath);
    pdal::Stage* reader = factory.createStage(driver);
    if (!reader) return res;

    pdal::Options options;
    options.add("filename", copcPath);
    std::ostringstream bs;
    bs.precision(17);
    bs << "([" << req.minX << "," << req.maxX << "],"
       << "[" << req.minY << "," << req.maxY << "],"
       << "[" << req.minZ << "," << req.maxZ << "])";
    options.add("bounds", bs.str());
    if (req.resolution > 0) options.add("resolution", req.resolution);
    reader->setOptions(options);

    pdal::PointTable table;
    reader->prepare(table);
    pdal::PointViewSet views = reader->execute(table);
    bool hasZ = table.layout()->hasDim(pdal::Dimension::Id::Z);

    double zRange = std::max(colorZMax - colorZMin, 1e-6);
    double invScale = 1.0 / worldScale;
    for (const auto& view : views) {
        res.positions.reserve(res.positions.size() + view->size() * 3);
        res.colors.reserve(res.colors.size() + view->size() * 3);
        for (pdal::PointId i = 0; i < view->size(); ++i) {
            double wx = view->getFieldAs<double>(pdal::Dimension::Id::X, i);
            double wy = view->getFieldAs<double>(pdal::Dimension::Id::Y, i);
            double wz = hasZ ? view->getFieldAs<double>(pdal::Dimension::Id::Z, i) : 0.0;
            res.positions.push_back(static_cast<float>((wx - worldCenterX) * invScale));
            res.positions.push_back(static_cast<float>((wz - worldCenterZ) * invScale));
            res.positions.push_back(static_cast<float>(-(wy - worldCenterY) * invScale));
            glm::vec3 c;
            if (orthoPtr && orthoPtr->hasGeo) {
                double ocol = (orthoPtr->A != 0) ? (wx - orthoPtr->C) / orthoPtr->A : 0;
                double orow = (orthoPtr->E != 0) ? (wy - orthoPtr->F) / orthoPtr->E : 0;
                c = sampleOrthoBilinear(*orthoPtr, ocol, orow);
            } else {
                c = elevationColorRamp(static_cast<float>((wz - colorZMin) / zRange));
            }
            res.colors.push_back(c.r);
            res.colors.push_back(c.g);
            res.colors.push_back(c.b);
        }
    }
    res.ok = true;
    return res;
}

void TileGrid::loaderRun() {
    while (true) {
        LoadRequest req;
        {
            std::unique_lock<std::mutex> lk(mtx);
            cv.wait(lk, [&] { return shutdown || !requests.empty(); });
            if (shutdown) return;
            req = requests.front();
            requests.pop_front();
        }
        LoadResult res;
        try {
            res = loadTile(req);
        } catch (const std::exception& e) {
            std::cerr << "[tile] load failed (tile " << req.tileIndex << "): " << e.what()
                      << std::endl;
            res = LoadResult{};
            res.tileIndex = req.tileIndex;
            res.resolution = req.resolution;
        }
        std::lock_guard<std::mutex> lk(mtx);
        results.push_back(std::move(res));
    }
}

void TileGrid::requestLoad(int tileIndex, double resolution) {
    Tile& t = tiles[tileIndex];
    t.inFlight = true;
    ++pendingLoads;
    {
        std::lock_guard<std::mutex> lk(mtx);
        requests.push_back({tileIndex, resolution,
                            t.minX, t.minY, t.minZ, t.maxX, t.maxY, t.maxZ});
    }
    cv.notify_one();
}

// ===========================================================================
// init / stop
// ===========================================================================

void TileGrid::init(const std::string& copcPath_,
                    double minX, double minY, double minZ,
                    double maxX, double maxY, double maxZ,
                    const Orthophoto* ortho) {
    orthoPtr = ortho;
    copcPath = copcPath_;
    worldCenterX = (minX + maxX) * 0.5;
    worldCenterY = (minY + maxY) * 0.5;
    worldCenterZ = (minZ + maxZ) * 0.5;
    worldScale = glm::length(glm::dvec3(maxX - minX, maxY - minY, maxZ - minZ));
    if (worldScale < 1e-9) worldScale = 1.0;
    colorZMin = minZ;
    colorZMax = maxZ;

    tileW = (maxX - minX) / gridX;
    tileH = (maxY - minY) / gridY;

    tiles.resize(static_cast<size_t>(gridX) * gridY);
    double invScale = 1.0 / worldScale;
    for (int gy = 0; gy < gridY; ++gy) {
        for (int gx = 0; gx < gridX; ++gx) {
            Tile& t = tiles[gy * gridX + gx];
            t.gx = gx; t.gy = gy;
            t.minX = minX + gx * tileW;
            t.maxX = t.minX + tileW;
            t.minY = minY + gy * tileH;
            t.maxY = t.minY + tileH;
            t.minZ = minZ; t.maxZ = maxZ;
            t.glMin = glm::vec3(
                static_cast<float>((t.minX - worldCenterX) * invScale),
                static_cast<float>((t.minZ - worldCenterZ) * invScale),
                static_cast<float>(-(t.maxY - worldCenterY) * invScale));
            t.glMax = glm::vec3(
                static_cast<float>((t.maxX - worldCenterX) * invScale),
                static_cast<float>((t.maxZ - worldCenterZ) * invScale),
                static_cast<float>(-(t.minY - worldCenterY) * invScale));
            t.glCenter = (t.glMin + t.glMax) * 0.5f;
            t.glRadius = glm::length(t.glMax - t.glMin) * 0.5f;
        }
    }

    worker = std::thread(&TileGrid::loaderRun, this);
    initHiZ();

    // Coarse first pass: every tile at ~500 points, for a fast first frame.
    double coarseRes = std::clamp(std::sqrt(tileW * tileH / 500.0), 1.0, 50.0);
    std::cerr << "[stream] coarse pass (resolution=" << coarseRes << "m) for "
              << tiles.size() << " tiles" << std::endl;
    for (size_t i = 0; i < tiles.size(); ++i) requestLoad(static_cast<int>(i), coarseRes);
}

void TileGrid::stop() {
    {
        std::lock_guard<std::mutex> lk(mtx);
        shutdown = true;
        requests.clear();
    }
    cv.notify_all();
    if (worker.joinable()) worker.join();
    results.clear();
}

TileGrid::~TileGrid() {
    stop();
    for (auto& t : tiles) releaseTileGL(t);
    destroyHiZ();
}

void TileGrid::releaseTileGL(Tile& t) {
    if (t.vao) glDeleteVertexArrays(1, &t.vao);
    if (t.vboPos) glDeleteBuffers(1, &t.vboPos);
    if (t.vboCol) glDeleteBuffers(1, &t.vboCol);
    t.vao = t.vboPos = t.vboCol = 0;
    t.pointCount = 0;
}

// ===========================================================================
// Hi-Z occlusion culling — see the mechanism comment on the member
// declarations in copc_streamer.h for the full explanation.
// ===========================================================================

void TileGrid::initHiZ() {
    hizCopyProgram = shaders::linkProgram(shaders::kHiZVert, shaders::kHiZCopyFrag);
    hizDownsampleProgram = shaders::linkProgram(shaders::kHiZVert, shaders::kHiZDownsampleFrag);
    if (!hizCopyProgram || !hizDownsampleProgram) {
        std::cerr << "[occlusion] Hi-Z shader setup failed — occlusion culling disabled "
                     "for this session (tiles will render exactly as if useOcclusion "
                     "were false)" << std::endl;
        if (hizCopyProgram) { glDeleteProgram(hizCopyProgram); hizCopyProgram = 0; }
        if (hizDownsampleProgram) { glDeleteProgram(hizDownsampleProgram); hizDownsampleProgram = 0; }
        return;
    }
    glGenVertexArrays(1, &hizFullscreenVAO);
    glGenFramebuffers(1, &hizFBO);
    std::cerr << "[occlusion] Hi-Z occlusion culling initialized" << std::endl;
}

// Target size (in texels, roughly square) for the ONE mip level read back
// to the CPU each frame — see the "deliberately reads back only a small,
// fixed-size" comment in copc_streamer.h for why this is a single small
// transfer rather than a per-tile GPU query.
static const int kHiZTargetReadSize = 64;

void TileGrid::resizeHiZIfNeeded(int viewportW, int viewportH) {
    if (!hizCopyProgram || !hizDownsampleProgram) return; // init failed — disabled
    if (viewportW <= 0 || viewportH <= 0) return;
    if (viewportW == hizCaptureW && viewportH == hizCaptureH && hizPyramidTex != 0) {
        return; // already correctly sized
    }
    hizCaptureW = viewportW;
    hizCaptureH = viewportH;

    if (hizDepthCaptureTex) { glDeleteTextures(1, &hizDepthCaptureTex); hizDepthCaptureTex = 0; }
    glGenTextures(1, &hizDepthCaptureTex);
    glBindTexture(GL_TEXTURE_2D, hizDepthCaptureTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, viewportW, viewportH, 0,
                 GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    if (hizPyramidTex) { glDeleteTextures(1, &hizPyramidTex); hizPyramidTex = 0; }
    glGenTextures(1, &hizPyramidTex);
    glBindTexture(GL_TEXTURE_2D, hizPyramidTex);

    int levels = 1;
    int w = viewportW, h = viewportH;
    while ((w > kHiZTargetReadSize || h > kHiZTargetReadSize) && levels < 16) {
        w = std::max(1, w / 2);
        h = std::max(1, h / 2);
        ++levels;
    }
    hizLevels = levels;
    hizReadLevel = levels - 1;
    hizReadW = std::max(1, viewportW >> hizReadLevel);
    hizReadH = std::max(1, viewportH >> hizReadLevel);

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
    // until the first real build completes (hizReady stays false until
    // then regardless, but this avoids the vector holding uninitialized
    // memory in the meantime).
    hizReadback.assign(static_cast<size_t>(hizReadW) * hizReadH, 1.0f);
    hizReady = false;
    std::cerr << "[occlusion] Hi-Z pyramid (re)sized: " << viewportW << "x" << viewportH
              << ", " << levels << " levels, CPU readback at "
              << hizReadW << "x" << hizReadH << std::endl;
}

void TileGrid::captureAndBuildHiZ(int viewportW, int viewportH) {
    if (!hizCopyProgram || !hizDownsampleProgram) return; // init failed — disabled
    resizeHiZIfNeeded(viewportW, viewportH);
    if (!hizPyramidTex || !hizDepthCaptureTex || !hizFBO) return;

    // This runs at the very end of render(), after all of this frame's
    // real tile drawing — save every piece of state it touches and
    // restore it afterward, so nothing later in the frame (text overlay,
    // etc.) is affected.
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
    // into hizDepthCaptureTex.
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, hizFBO);
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                           GL_TEXTURE_2D, hizDepthCaptureTex, 0);
    GLenum drawBufNone = GL_NONE;
    glDrawBuffers(1, &drawBufNone); // depth-only target this pass, no color buffer
    glBlitFramebuffer(0, 0, viewportW, viewportH, 0, 0, viewportW, viewportH,
                      GL_DEPTH_BUFFER_BIT, GL_NEAREST);

    // Step 2: copy pass — depth texture -> mip 0 of the Hi-Z pyramid.
    glBindFramebuffer(GL_FRAMEBUFFER, hizFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, 0, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D, hizPyramidTex, 0);
    GLenum drawBufColor0 = GL_COLOR_ATTACHMENT0;
    glDrawBuffers(1, &drawBufColor0);
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glViewport(0, 0, viewportW, viewportH);
    glUseProgram(hizCopyProgram);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, hizDepthCaptureTex);
    glUniform1i(glGetUniformLocation(hizCopyProgram, "uSrcDepth"), 0);
    glBindVertexArray(hizFullscreenVAO);
    glDrawArrays(GL_TRIANGLES, 0, 3);

    // Step 3: successive 2x2 max-reduction passes, each sampling the
    // PREVIOUS Hi-Z mip (never the raw depth texture directly) and
    // rendering into the next.
    glUseProgram(hizDownsampleProgram);
    glBindTexture(GL_TEXTURE_2D, hizPyramidTex);
    glUniform1i(glGetUniformLocation(hizDownsampleProgram, "uSrcMip"), 0);
    GLint texelSizeLoc = glGetUniformLocation(hizDownsampleProgram, "uSrcTexelSize");
    for (int lvl = 1; lvl <= hizReadLevel; ++lvl) {
        int srcW = std::max(1, viewportW >> (lvl - 1));
        int srcH = std::max(1, viewportH >> (lvl - 1));
        int dstW = std::max(1, viewportW >> lvl);
        int dstH = std::max(1, viewportH >> lvl);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, hizPyramidTex, lvl);
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
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, hizLevels - 1);

    // Step 4: read back JUST the small coarse target level — once per
    // frame, not per tile (see the class comment for why).
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D, hizPyramidTex, hizReadLevel);
    hizReadback.resize(static_cast<size_t>(hizReadW) * hizReadH);
    glReadPixels(0, 0, hizReadW, hizReadH, GL_RED, GL_FLOAT, hizReadback.data());
    hizReady = true;

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

// Tests a tile's world-space AABB against the Hi-Z pyramid BUILT LAST
// FRAME (see the class comment for why one-frame-stale is the deliberate,
// safe design here). Returns true only when EVERY corner of the tile's
// screen-space footprint is provably behind the farthest known depth in
// its region — i.e. only when culling is certain to be safe, never a
// guess.
bool TileGrid::isTileOccludedByHiZ(const Tile& t, const glm::mat4& VP) const {
    if (!hizReady || hizReadback.empty()) return false; // nothing to test against yet

    glm::vec3 lo = t.glMin, hi = t.glMax;
    glm::vec3 corners[8] = {
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

    // Fully outside the view frustum in X/Y — not an occlusion question
    // at all (that's frustum culling, a separate, simpler concern this
    // function doesn't attempt); don't claim occlusion here, just let it
    // fall through to the existing frustum/distance-based logic elsewhere.
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
    int px0 = std::clamp(static_cast<int>(u0 * hizReadW), 0, hizReadW - 1);
    int px1 = std::clamp(static_cast<int>(u1 * hizReadW), 0, hizReadW - 1);
    int py0 = std::clamp(static_cast<int>(v0 * hizReadH), 0, hizReadH - 1);
    int py1 = std::clamp(static_cast<int>(v1 * hizReadH), 0, hizReadH - 1);

    // Farthest known depth across every readback texel the tile's screen
    // footprint touches — the conservative (MAX) value the whole region
    // must be farther than for a safe cull.
    float farthestKnown = 0.0f;
    for (int py = py0; py <= py1; ++py) {
        for (int px = px0; px <= px1; ++px) {
            farthestKnown = std::max(farthestKnown, hizReadback[static_cast<size_t>(py) * hizReadW + px]);
        }
    }

    return depthNear > farthestKnown;
}

void TileGrid::destroyHiZ() {
    if (hizCopyProgram) { glDeleteProgram(hizCopyProgram); hizCopyProgram = 0; }
    if (hizDownsampleProgram) { glDeleteProgram(hizDownsampleProgram); hizDownsampleProgram = 0; }
    if (hizFullscreenVAO) { glDeleteVertexArrays(1, &hizFullscreenVAO); hizFullscreenVAO = 0; }
    if (hizFBO) { glDeleteFramebuffers(1, &hizFBO); hizFBO = 0; }
    if (hizDepthCaptureTex) { glDeleteTextures(1, &hizDepthCaptureTex); hizDepthCaptureTex = 0; }
    if (hizPyramidTex) { glDeleteTextures(1, &hizPyramidTex); hizPyramidTex = 0; }
    hizReady = false;
}

// ===========================================================================
// uploadTile: PCA for LOD + GPU upload. Main thread.
// ===========================================================================

static void computeTilePCA(Tile& t, const std::vector<float>& pos) {
    // PCA: compute the inertia axes (principal components) of the tile.
    glm::vec3 centroid(0.0f);
    for (size_t i = 0; i < static_cast<size_t>(t.pointCount); ++i) {
        centroid += glm::vec3(pos[i*3], pos[i*3+1], pos[i*3+2]);
    }
    centroid /= static_cast<float>(t.pointCount);

    float cov[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    for (size_t i = 0; i < static_cast<size_t>(t.pointCount); ++i) {
        glm::vec3 p(pos[i*3], pos[i*3+1], pos[i*3+2]);
        p -= centroid;
        cov[0][0] += p.x * p.x;
        cov[0][1] += p.x * p.y;
        cov[0][2] += p.x * p.z;
        cov[1][1] += p.y * p.y;
        cov[1][2] += p.y * p.z;
        cov[2][2] += p.z * p.z;
    }
    float invN = 1.0f / static_cast<float>(t.pointCount);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j <= i; ++j) {
            cov[i][j] *= invN;
            cov[j][i] = cov[i][j];
        }

    // Jacobi eigendecomposition for a 3×3 symmetric matrix.
    float eigenvectors[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    for (int iter = 0; iter < 50; ++iter) {
        int p = 0, q = 1;
        float maxOff = std::abs(cov[0][1]);
        if (std::abs(cov[0][2]) > maxOff) { p = 0; q = 2; maxOff = std::abs(cov[0][2]); }
        if (std::abs(cov[1][2]) > maxOff) { p = 1; q = 2; maxOff = std::abs(cov[1][2]); }
        if (maxOff < 1e-10f) break;

        float theta = (cov[q][q] - cov[p][p]) / (2.0f * cov[p][q]);
        float t_ = (theta >= 0 ? 1.0f : -1.0f) /
                   (std::abs(theta) + std::sqrt(theta * theta + 1.0f));
        float c = 1.0f / std::sqrt(t_ * t_ + 1.0f);
        float s = t_ * c;

        float newPP = cov[p][p] - t_ * cov[p][q];
        float newQQ = cov[q][q] + t_ * cov[p][q];
        cov[p][p] = newPP;
        cov[q][q] = newQQ;
        cov[p][q] = cov[q][p] = 0.0f;
        for (int r = 0; r < 3; ++r) {
            if (r != p && r != q) {
                float newRP = c * cov[r][p] - s * cov[r][q];
                float newRQ = s * cov[r][p] + c * cov[r][q];
                cov[r][p] = cov[p][r] = newRP;
                cov[r][q] = cov[q][r] = newRQ;
            }
        }
        for (int r = 0; r < 3; ++r) {
            float newRP = c * eigenvectors[r][p] - s * eigenvectors[r][q];
            float newRQ = s * eigenvectors[r][p] + c * eigenvectors[r][q];
            eigenvectors[r][p] = newRP;
            eigenvectors[r][q] = newRQ;
        }
    }

    int order[3] = {0, 1, 2};
    for (int i = 0; i < 2; ++i) {
        for (int j = i + 1; j < 3; ++j) {
            if (cov[order[j]][order[j]] > cov[order[i]][order[i]]) {
                std::swap(order[i], order[j]);
            }
        }
    }
    t.principalAxis1 = glm::vec3(eigenvectors[0][order[0]], eigenvectors[1][order[0]], eigenvectors[2][order[0]]);
    t.principalAxis2 = glm::vec3(eigenvectors[0][order[1]], eigenvectors[1][order[1]], eigenvectors[2][order[1]]);
    t.principalAxis3 = glm::vec3(eigenvectors[0][order[2]], eigenvectors[1][order[2]], eigenvectors[2][order[2]]);
    t.obbExtent1 = std::sqrt(std::max(0.0f, cov[order[0]][order[0]]));
    t.obbExtent2 = std::sqrt(std::max(0.0f, cov[order[1]][order[1]]));
    t.obbExtent3 = std::sqrt(std::max(0.0f, cov[order[2]][order[2]]));

    float obbVolume = (2.0f * t.obbExtent1) * (2.0f * t.obbExtent2) * (2.0f * t.obbExtent3);
    if (obbVolume < 1e-15f) obbVolume = 1e-15f;
    t.pointSpacing = std::cbrt(obbVolume / static_cast<float>(t.pointCount));
}

void TileGrid::uploadTile(Tile& t, LoadResult& r) {
    releaseTileGL(t);
    t.loadedResolution = static_cast<float>(r.resolution);
    t.pointCount = static_cast<GLsizei>(r.positions.size() / 3);
    if (t.pointCount == 0) return;

    // Tighten the vertical bounds to the actual loaded points.
    float minY = 1e30f, maxY = -1e30f;
    for (size_t i = 0; i < static_cast<size_t>(t.pointCount); ++i) {
        minY = std::min(minY, r.positions[i * 3 + 1]);
        maxY = std::max(maxY, r.positions[i * 3 + 1]);
    }
    t.glMin.y = minY;
    t.glMax.y = maxY;
    t.glCenter = (t.glMin + t.glMax) * 0.5f;
    t.glRadius = glm::length(t.glMax - t.glMin) * 0.5f;

    computeTilePCA(t, r.positions);

    glGenVertexArrays(1, &t.vao);
    glBindVertexArray(t.vao);
    glGenBuffers(1, &t.vboPos);
    glBindBuffer(GL_ARRAY_BUFFER, t.vboPos);
    glBufferData(GL_ARRAY_BUFFER, r.positions.size() * sizeof(float),
                 r.positions.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glGenBuffers(1, &t.vboCol);
    glBindBuffer(GL_ARRAY_BUFFER, t.vboCol);
    glBufferData(GL_ARRAY_BUFFER, r.colors.size() * sizeof(float),
                 r.colors.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glBindVertexArray(0);
}

// ===========================================================================
// desiredResolution: PDAL resolution giving ~1 point per 25 px² of the
// tile's projected OBB area. Returns 0 to leave the tile as is.
// ===========================================================================

double TileGrid::desiredResolution(const Tile& t, const glm::vec3& camPos,
                                   float fov, float viewportH) {
    if (t.pointCount == 0) return 0.0;
    float dist = std::max(glm::length(t.glCenter - camPos), 1e-6f);

    float pxPerUnit = viewportH / (2.0f * dist * std::tan(glm::radians(fov) * 0.5f));
    glm::vec3 sight = glm::normalize(t.glCenter - camPos);

    struct Face { glm::vec3 normal; float dim1, dim2; };
    Face faces[3] = {
        { t.principalAxis1, 2.0f * t.obbExtent2, 2.0f * t.obbExtent3 },
        { t.principalAxis2, 2.0f * t.obbExtent1, 2.0f * t.obbExtent3 },
        { t.principalAxis3, 2.0f * t.obbExtent1, 2.0f * t.obbExtent2 },
    };
    float projectedAreaGL = 0.0f;
    for (const auto& f : faces) {
        projectedAreaGL += f.dim1 * f.dim2 * std::abs(glm::dot(f.normal, sight));
    }
    float projectedAreaPx = projectedAreaGL * pxPerUnit * pxPerUnit;
    if (projectedAreaPx < 25.0f) return 0.0;

    float targetPoints = std::clamp(projectedAreaPx / 25.0f, 100.0f, 500000.0f);
    double res = std::sqrt(tileW * tileH / targetPoints);
    if (res > 50.0) return 0.0;
    return std::max(res, 0.1);
}

// ===========================================================================
// update: apply finished loads and request refinements. Nothing is evicted:
// every tile stays resident at its last loaded resolution.
// ===========================================================================

void TileGrid::update(const glm::vec3& camPos, float fov, float viewportH) {
    std::deque<LoadResult> done;
    {
        std::lock_guard<std::mutex> lk(mtx);
        done.swap(results);
    }
    for (LoadResult& r : done) {
        Tile& t = tiles[r.tileIndex];
        t.inFlight = false;
        --pendingLoads;
        if (r.ok) {
            t.failures = 0;
            uploadTile(t, r);
        } else if (++t.failures < 4) {
            // Retry coarser; a very dense query is the usual failure cause.
            requestLoad(r.tileIndex, std::min(std::max(r.resolution, 1.0) * 2.0, 50.0));
        }
    }

    // Refine tiles whose loaded resolution is too coarse for the current view.
    for (size_t i = 0; i < tiles.size() && pendingLoads < maxConcurrentLoads; ++i) {
        Tile& t = tiles[i];
        if (t.inFlight || t.pointCount == 0 || t.failures > 0) continue;
        double desired = desiredResolution(t, camPos, fov, viewportH);
        if (desired > 0.0 && t.loadedResolution > desired * 1.5) {
            requestLoad(static_cast<int>(i), desired);
        }
    }
}

// ===========================================================================
// render: draw all loaded tiles with per-tile density uniforms.
// ===========================================================================

void TileGrid::render(GLuint pointProgram, const glm::mat4& V, const glm::mat4& P,
                      const glm::vec3& camPos, float fov, float viewportW, float viewportH,
                      float zScale, float pointSizeMul, bool useOcclusion) {
    (void)camPos;
    glUseProgram(pointProgram);
    glUniformMatrix4fv(glGetUniformLocation(pointProgram, "uView"), 1, GL_FALSE, glm::value_ptr(V));
    glUniformMatrix4fv(glGetUniformLocation(pointProgram, "uProj"), 1, GL_FALSE, glm::value_ptr(P));
    glUniform1f(glGetUniformLocation(pointProgram, "uViewportH"), viewportH);
    glUniform1f(glGetUniformLocation(pointProgram, "uZScale"), zScale);
    glUniform1f(glGetUniformLocation(pointProgram, "uTanHalfFov"),
                std::tan(glm::radians(fov) * 0.5f));
    glUniform1f(glGetUniformLocation(pointProgram, "uDisableSubsampling"), 0.0f);
    glUniform1f(glGetUniformLocation(pointProgram, "uOrtho"), 0.0f);
    glUniform1f(glGetUniformLocation(pointProgram, "uPointSize"), 0.0015f * pointSizeMul);
    glUniform1f(glGetUniformLocation(pointProgram, "uTargetPixelSpacing"), 2.83f);
    glUniform1f(glGetUniformLocation(pointProgram, "uDensityMul"), 1.0f);
    GLint densityLoc = glGetUniformLocation(pointProgram, "uDensity");

    glm::mat4 VP = P * V;
    for (Tile& t : tiles) {
        if (!t.hasGeometry()) continue;
        if (useOcclusion && isTileOccludedByHiZ(t, VP)) continue;
        float tileDensity = (t.pointSpacing > 1e-10f)
            ? 1.0f / (t.pointSpacing * t.pointSpacing) : 1e15f;
        glUniform1f(densityLoc, tileDensity);
        glBindVertexArray(t.vao);
        glDrawArrays(GL_POINTS, 0, t.pointCount);
    }
    glBindVertexArray(0);

    // Build next frame's Hi-Z from this frame's depth.
    if (useOcclusion) {
        captureAndBuildHiZ(static_cast<int>(viewportW), static_cast<int>(viewportH));
    }
}
