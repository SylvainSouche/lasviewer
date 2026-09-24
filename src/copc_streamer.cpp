// copc_streamer.cpp — Async COPC tile streaming with distance-based LOD.
//
// Implements the entire TileGrid class: init, destructor, uploadTile,
// desiredResolution, update, render, stop, and the background loader thread
// (loaderRun). The loader thread uses PDAL to fetch tile sub-bounds at a
// requested resolution; the main thread drains completed tiles and uploads
// them to the GPU.
#include "copc_streamer.h"
#include "geotiff.h"
#include "shaders.h"
// GLFW/OpenGL headers now come from copc_streamer.h -> gl_platform.h (see
// that file for why this used to be a fragile per-file ad-hoc block).

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Options.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/Stage.hpp>
#include <pdal/Dimension.hpp>

#include <iostream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>
#include <thread>
#include <atomic>

#ifndef LASVIEWER_NO_OPENMP
#  ifdef _OPENMP
#    define LASVIEWER_HAS_OPENMP 1
#  else
#    define LASVIEWER_HAS_OPENMP 0
#  endif
#else
#  define LASVIEWER_HAS_OPENMP 0
#endif

// Elevation gradient (shared with point_cloud.cpp / dem_mesh.cpp).
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
// Background loader thread: pulls requests from the queue, loads via PDAL,
// and pushes finished tiles to the results queue.
// ===========================================================================

void TileGrid::loaderRun() {
    while (true) {
        Request req;
        {
            std::unique_lock<std::mutex> lk(mtx);
            cv.wait(lk, [&]{ return shutdown || !requests.empty(); });
            if (shutdown && requests.empty()) return;
            req = requests.front();
            requests.pop();
            req.tile->state = TileState::LOADING;
        }
        // Load via PDAL with bounds + resolution.
        try {
            pdal::StageFactory factory;
            std::string driver = factory.inferReaderDriver(copcPath);
            pdal::Stage* reader = factory.createStage(driver);
            if (!reader) { req.tile->state = TileState::FAILED; continue; }

            pdal::Options options;
            options.add("filename", copcPath);
            std::ostringstream bs;
            bs << "([" << req.minX << "," << req.maxX << "],"
               << "[" << req.minY << "," << req.maxY << "],"
               << "[" << req.minZ << "," << req.maxZ << "])";
            options.add("bounds", bs.str());
            if (req.resolution > 0) {
                options.add("resolution", req.resolution);
            }
            reader->setOptions(options);

            pdal::PointTable table;
            reader->prepare(table);
            pdal::PointViewSet views = reader->execute(table);
            pdal::PointLayoutPtr layout = table.layout();
            bool hasZ = layout->hasDim(pdal::Dimension::Id::Z);

            std::vector<float> pos, col;
            float zMin = 1e30f, zMax = -1e30f;
            for (const auto& view : views) {
                for (pdal::PointId i = 0; i < view->size(); ++i) {
                    float z = hasZ ? view->getFieldAs<float>(pdal::Dimension::Id::Z, i) : 0.0f;
                    if (z < zMin) zMin = z;
                    if (z > zMax) zMax = z;
                }
            }
            if (zMax - zMin < 1e-6f) zMax = zMin + 1.0f;

            double invScale = 1.0 / worldScale;
            for (const auto& view : views) {
                for (pdal::PointId i = 0; i < view->size(); ++i) {
                    double wx = view->getFieldAs<double>(pdal::Dimension::Id::X, i);
                    double wy = view->getFieldAs<double>(pdal::Dimension::Id::Y, i);
                    double wz = hasZ ? view->getFieldAs<double>(pdal::Dimension::Id::Z, i) : 0.0;
                    pos.push_back(static_cast<float>((wx - worldCenterX) * invScale));
                    pos.push_back(static_cast<float>((wz - worldCenterZ) * invScale));
                    pos.push_back(static_cast<float>(-(wy - worldCenterY) * invScale));
                    glm::vec3 c;
                    if (orthoPtr && orthoPtr->hasGeo) {
                        double dx = wx - orthoPtr->C;
                        double dy = wy - orthoPtr->F;
                        double ocol = (orthoPtr->A != 0) ? dx / orthoPtr->A : 0;
                        double orow = (orthoPtr->E != 0) ? dy / orthoPtr->E : 0;
                        c = sampleOrthoBilinear(*orthoPtr, ocol, orow);
                    } else {
                        float t = (static_cast<float>(wz) - zMin) / (zMax - zMin);
                        c = elevationColorRamp(t);
                    }
                    col.push_back(c.r); col.push_back(c.g); col.push_back(c.b);
                }
            }
            req.tile->positions = std::move(pos);
            req.tile->colors = std::move(col);
            req.tile->loadedResolution = req.resolution;
            req.tile->state = TileState::LOADED;
        } catch (const std::exception& e) {
            std::cerr << "[tile] load failed (" << req.tile->gx << "," << req.tile->gy
                      << "): " << e.what() << " — retrying at coarser resolution" << std::endl;
            double retryRes = req.resolution * 2.0;
            if (retryRes < 50.0) {
                req.tile->state = TileState::UNLOADED;
                req.tile->loadedResolution = 0.0f;
                std::lock_guard<std::mutex> lk(mtx);
                requests.push({req.tile, retryRes,
                               req.tile->minX, req.tile->minY, req.tile->minZ,
                               req.tile->maxX, req.tile->maxY, req.tile->maxZ});
                req.tile->state = TileState::REQUESTED;
                cv.notify_one();
            } else {
                req.tile->state = TileState::FAILED;
            }
        }
        {
            std::lock_guard<std::mutex> lk(mtx);
            results.push(req.tile);
        }
    }
}

// ===========================================================================
// init: build the tile grid, start the worker thread, and request an initial
// coarse pass for instant first render.
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
    glm::dvec3 diag(maxX - minX, maxY - minY, maxZ - minZ);
    worldScale = glm::length(diag);
    if (worldScale < 1e-9) worldScale = 1.0;

    tileW = (maxX - minX) / gridX;
    tileH = (maxY - minY) / gridY;

    tiles.resize(gridX * gridY);
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

    // Allocate occlusion query objects + reusable proxy VAO. Kept
    // allocated (unused — see specs.md §5.6) rather than removed, as
    // documented scaffolding for a future retry of the query-based
    // approach; Hi-Z (initHiZ(), below) is the actual occlusion
    // mechanism now in use.
    occlusionQueries.resize(gridX * gridY);
    glGenQueries(gridX * gridY, occlusionQueries.data());
    glGenVertexArrays(1, &occVAO);
    glGenBuffers(1, &occVBO);
    glBindVertexArray(occVAO);
    glBindBuffer(GL_ARRAY_BUFFER, occVBO);
    glBufferData(GL_ARRAY_BUFFER, 24 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glBindVertexArray(0);
    occlusionInit = true;

    initHiZ();

    // Phase 1: request ALL tiles at a coarse resolution (~500 pts/tile).
    double tileArea = tileW * tileH;
    double coarseRes = std::sqrt(tileArea / 500.0);
    if (coarseRes < 1.0) coarseRes = 1.0;
    if (coarseRes > 50.0) coarseRes = 50.0;
    std::cerr << "[stream] Phase 1: coarsest pass (resolution=" << coarseRes
              << "m, ~500 pts/tile) for " << gridX * gridY << " tiles" << std::endl;
    for (auto& t : tiles) {
        std::lock_guard<std::mutex> lk(mtx);
        requests.push({&t, coarseRes, t.minX, t.minY, t.minZ, t.maxX, t.maxY, t.maxZ});
        t.state = TileState::REQUESTED;
        ++pendingLoads;
        cv.notify_one();
    }
}

// ===========================================================================
// stop: signal shutdown, drain the request queue, join the worker thread.
// ===========================================================================

void TileGrid::stop() {
    {
        std::lock_guard<std::mutex> lk(mtx);
        shutdown = true;
        std::queue<Request> empty;
        std::swap(requests, empty);
    }
    cv.notify_one();
    if (worker.joinable()) worker.join();
    std::lock_guard<std::mutex> lk(mtx);
    std::queue<Tile*> emptyResults;
    std::swap(results, emptyResults);
}

TileGrid::~TileGrid() {
    stop();
    for (auto& t : tiles) {
        if (t.vao) glDeleteVertexArrays(1, &t.vao);
        if (t.vboPos) glDeleteBuffers(1, &t.vboPos);
        if (t.vboCol) glDeleteBuffers(1, &t.vboCol);
    }
    if (occlusionInit) glDeleteQueries(gridX * gridY, occlusionQueries.data());
    destroyHiZ();
}

// ===========================================================================
// Hi-Z occlusion culling — see the mechanism comment on the member
// declarations in copc_streamer.h for the full explanation.
// ===========================================================================

void TileGrid::initHiZ() {
    hizInitAttempted = true;
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
// uploadTile: move CPU buffers to the GPU and run a per-tile PCA for
// density-aware shader subsampling. Called on the main thread.
// ===========================================================================

void TileGrid::uploadTile(Tile& t) {
    if (t.positions.empty()) {
        t.pointCount = 0;
        t.state = TileState::LOADED;
        std::cerr << "[tile] empty tile (" << t.gx << "," << t.gy << ") — 0 points" << std::endl;
        return;
    }
    t.pointCount = static_cast<GLsizei>(t.positions.size() / 3);

    // Recompute Y (elevation) bbox from actual loaded points.
    float minY = 1e30f, maxY = -1e30f;
    for (size_t i = 0; i < static_cast<size_t>(t.pointCount); ++i) {
        float y = t.positions[i * 3 + 1];
        if (y < minY) minY = y;
        if (y > maxY) maxY = y;
    }
    t.glMin.y = minY;
    t.glMax.y = maxY;
    t.glCenter = (t.glMin + t.glMax) * 0.5f;
    t.glRadius = glm::length(t.glMax - t.glMin) * 0.5f;

    // PCA: compute the inertia axes (principal components) of the tile.
    glm::vec3 centroid(0.0f);
    for (size_t i = 0; i < static_cast<size_t>(t.pointCount); ++i) {
        centroid += glm::vec3(t.positions[i*3], t.positions[i*3+1], t.positions[i*3+2]);
    }
    centroid /= static_cast<float>(t.pointCount);

    float cov[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    for (size_t i = 0; i < static_cast<size_t>(t.pointCount); ++i) {
        glm::vec3 p(t.positions[i*3], t.positions[i*3+1], t.positions[i*3+2]);
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

    // Upload to GPU.
    glGenVertexArrays(1, &t.vao);
    glBindVertexArray(t.vao);
    glGenBuffers(1, &t.vboPos);
    glBindBuffer(GL_ARRAY_BUFFER, t.vboPos);
    glBufferData(GL_ARRAY_BUFFER, t.positions.size() * sizeof(float),
                 t.positions.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glGenBuffers(1, &t.vboCol);
    glBindBuffer(GL_ARRAY_BUFFER, t.vboCol);
    glBufferData(GL_ARRAY_BUFFER, t.colors.size() * sizeof(float),
                 t.colors.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glBindVertexArray(0);
    t.positions.clear(); t.positions.shrink_to_fit();
    t.colors.clear(); t.colors.shrink_to_fit();
}

// ===========================================================================
// desiredResolution: compute the PDAL resolution that would give ~1 pt per
// 5px² at the current camera distance. Returns 0 to skip the tile.
// ===========================================================================

double TileGrid::desiredResolution(const Tile& t, const glm::vec3& camPos,
                                   float fov, float viewportH) {
    if (t.pointCount == 0) return 0.0f;
    float dist = glm::length(t.glCenter - camPos);
    if (dist < 1e-6f) dist = 1e-6f;

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
        float cosAngle = std::abs(glm::dot(f.normal, sight));
        projectedAreaGL += f.dim1 * f.dim2 * cosAngle;
    }
    float projectedAreaPx = projectedAreaGL * pxPerUnit * pxPerUnit;
    if (projectedAreaPx < 25.0f) return 0.0f;

    float targetPoints = projectedAreaPx / 25.0f;
    if (targetPoints < 100.0f) targetPoints = 100.0f;
    if (targetPoints > 500000.0f) targetPoints = 500000.0f;

    double tileAreaM = tileW * tileH;
    double res = std::sqrt(tileAreaM / targetPoints);
    if (res < 0.1) res = 0.1;
    if (res > 50.0) return 0.0f;
    return res;
}

// ===========================================================================
// update: drain completed loads, request refinements, evict stale tiles.
// ===========================================================================

void TileGrid::update(const glm::vec3& camPos, float fov, float viewportH,
                      double currentTime) {
    // 1. Drain completed loads from the worker thread.
    std::vector<Tile*> completed;
    {
        std::lock_guard<std::mutex> lk(mtx);
        while (!results.empty()) {
            completed.push_back(results.front());
            results.pop();
        }
    }
    for (Tile* t : completed) {
        if (t->state == TileState::LOADED) {
            if (t->vao) glDeleteVertexArrays(1, &t->vao);
            if (t->vboPos) glDeleteBuffers(1, &t->vboPos);
            if (t->vboCol) glDeleteBuffers(1, &t->vboCol);
            uploadTile(*t);
        } else if (t->state == TileState::FAILED) {
            t->state = TileState::UNLOADED;
            if (pendingLoads < maxConcurrentLoads) {
                std::lock_guard<std::mutex> lk(mtx);
                requests.push({t, 10.0, t->minX, t->minY, t->minZ, t->maxX, t->maxY, t->maxZ});
                t->state = TileState::REQUESTED;
                ++pendingLoads;
                cv.notify_one();
            }
        }
        --pendingLoads;
    }

    // 2. Re-evaluate ALL tiles against the current camera position.
    for (auto& t : tiles) {
        if (pendingLoads >= maxConcurrentLoads) break;
        if (t.state != TileState::LOADED && t.state != TileState::REQUESTED) continue;
        if (t.pointCount == 0) continue;

        double desired = desiredResolution(t, camPos, fov, viewportH);
        if (desired <= 0.0) continue;

        if (t.loadedResolution > desired * 1.5) {
            if (t.state == TileState::LOADED) {
                t.state = TileState::REQUESTED;
                std::lock_guard<std::mutex> lk(mtx);
                requests.push({&t, desired, t.minX, t.minY, t.minZ, t.maxX, t.maxY, t.maxZ});
                ++pendingLoads;
                cv.notify_one();
            }
        }
    }

    // 3. LRU eviction: tiles not used in the last 10s are unloaded.
    for (auto& t : tiles) {
        if (t.state != TileState::LOADED) continue;
        if (currentTime - t.lastUsedTime > 10.0) {
            t.state = TileState::EVICTING;
        }
    }
    for (auto& t : tiles) {
        if (t.state != TileState::EVICTING) continue;
        if (t.vao) glDeleteVertexArrays(1, &t.vao);
        if (t.vboPos) glDeleteBuffers(1, &t.vboPos);
        if (t.vboCol) glDeleteBuffers(1, &t.vboCol);
        t.vao = t.vboPos = t.vboCol = 0;
        t.pointCount = 0;
        t.state = TileState::UNLOADED;
        t.loadedResolution = 0.0f;
    }
}

// ===========================================================================
// render: draw all loaded tiles with per-tile density uniforms.
// ===========================================================================

void TileGrid::render(GLuint pointProgram, const glm::mat4& V, const glm::mat4& P,
                      const glm::vec3& camPos, float fov, float viewportW, float viewportH,
                      float zScale, float pointSizeMul, double currentTime,
                      bool useOcclusion) {
    glUseProgram(pointProgram);
    glUniformMatrix4fv(glGetUniformLocation(pointProgram, "uView"), 1, GL_FALSE, glm::value_ptr(V));
    glUniformMatrix4fv(glGetUniformLocation(pointProgram, "uProj"), 1, GL_FALSE, glm::value_ptr(P));
    glUniform1f(glGetUniformLocation(pointProgram, "uViewportH"), viewportH);
    glUniform1f(glGetUniformLocation(pointProgram, "uZScale"), zScale);
    glUniform1f(glGetUniformLocation(pointProgram, "uTanHalfFov"),
                std::tan(glm::radians(fov) * 0.5f));
    glUniform1f(glGetUniformLocation(pointProgram, "uDisableSubsampling"), 0.0f);
    glUniform1f(glGetUniformLocation(pointProgram, "uOrtho"), 0.0f);

    glm::mat4 VP = P * V;
    size_t drawnTiles = 0, drawnPoints = 0, culledTiles = 0;
    for (int i = 0; i < gridX * gridY; ++i) {
        Tile& t = tiles[i];
        if (t.pointCount == 0) continue;
        if (t.state != TileState::LOADED && t.state != TileState::REQUESTED) continue;

        t.lastUsedTime = currentTime;

        if (useOcclusion && isTileOccludedByHiZ(t, VP)) {
            ++culledTiles;
            continue;
        }

        float dist = glm::length(t.glCenter - camPos);
        if (dist < 1e-6f) dist = 1e-6f;
        glUniform1f(glGetUniformLocation(pointProgram, "uPointSize"),
                    0.0015f * pointSizeMul);
        glUniform1f(glGetUniformLocation(pointProgram, "uTargetPixelSpacing"), 2.83f);
        float tileDensity = (t.pointSpacing > 1e-10f)
            ? 1.0f / (t.pointSpacing * t.pointSpacing) : 1e15f;
        glUniform1f(glGetUniformLocation(pointProgram, "uDensity"), tileDensity);
        glUniform1f(glGetUniformLocation(pointProgram, "uDensityMul"), 1.0f);

        glBindVertexArray(t.vao);
        glDrawArrays(GL_POINTS, 0, t.pointCount);
        glBindVertexArray(0);
        ++drawnTiles;
        drawnPoints += t.pointCount;
    }

    // Build the Hi-Z pyramid from THIS frame's now-rendered depth, for
    // NEXT frame's isTileOccludedByHiZ() calls above — see the class
    // comment in copc_streamer.h for why this is deliberately
    // one-frame-stale rather than tested against itself same-frame
    // (chicken-and-egg: the tiles just drawn above are the only depth
    // there is to build from in this mutually-exclusive-with-DEM render
    // path — see main.cpp).
    if (useOcclusion) {
        captureAndBuildHiZ(static_cast<int>(viewportW), static_cast<int>(viewportH));
    }

    static double lastReport = 0.0;
    if (currentTime - lastReport >= 1.0) {
        int loaded = 0;
        for (auto& t : tiles) if (t.state == TileState::LOADED) ++loaded;
        std::cerr << "[stream] drew " << drawnPoints << " pts from " << drawnTiles
                  << " tiles (" << loaded << "/" << gridX * gridY << " loaded, "
                  << pendingLoads << " pending, " << culledTiles << " occlusion-culled)"
                  << std::endl;
        lastReport = currentTime;
    }
}
