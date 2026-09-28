// copc_streamer.cpp — async COPC tile streaming.
//
// A background loader thread turns LoadRequests into LoadResults (the
// copc-lib octree nodes intersecting the tile at the depth matching the
// requested resolution, GL-space transform, coloring). The main thread
// drains results in update(), uploads them, and requests refinements based
// on each tile's projected on-screen size.
#include "copc_streamer.h"
#include "raster.h"
#include "hiz.h"
#include "layer.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "las_format.h"

#include <copc-lib/geometry/box.hpp>
#include <copc-lib/hierarchy/node.hpp>
#include <copc-lib/io/copc_reader.hpp>

#include <algorithm>
#include <cmath>
#include <list>
#include <map>
#include <tuple>
#include <iostream>
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

struct TileGrid::CopcSource {
    explicit CopcSource(const std::string& path) : reader(path) {
        header = reader.CopcConfig().LasHeader();
        spacing = reader.CopcConfig().CopcInfo().spacing;
        nodes = reader.GetAllNodes();
        for (const copc::Node& n : nodes) maxDepth = std::max(maxDepth, n.key.d);
        if (!lasRecordLayout(header.PointFormatId(), header.PointRecordLength(),
                             glm::dvec3(header.Scale().x, header.Scale().y, header.Scale().z),
                             glm::dvec3(header.Offset().x, header.Offset().y, header.Offset().z),
                             layout)) {
            throw std::runtime_error("unsupported COPC point format");
        }
    }

    // Shallowest depth whose point spacing is at most `resolution` (copc-lib's
    // own rule); <= 0 means the full depth.
    int depthAtResolution(double resolution) const {
        if (resolution <= 0.0) return maxDepth;
        double r = spacing;
        for (int d = 0; d <= maxDepth; ++d) {
            if (r <= resolution) return d;
            r /= 2.0;
        }
        return maxDepth;
    }

    // Decompressed records of a node. Shallow nodes cover many tiles, so they
    // are kept (least recently used evicted beyond kCacheBytes) instead of
    // being decompressed again for every tile they overlap.
    std::shared_ptr<const std::vector<char>> pointData(const copc::Node& node) {
        Key k{node.key.d, node.key.x, node.key.y, node.key.z};
        auto it = cache.find(k);
        if (it != cache.end()) {
            lru.splice(lru.begin(), lru, it->second.second);
            return it->second.first;
        }
        auto data = std::make_shared<const std::vector<char>>(reader.GetPointData(node));
        cacheBytes += data->size();
        lru.push_front(k);
        cache.emplace(k, std::make_pair(data, lru.begin()));
        while (cacheBytes > kCacheBytes && lru.size() > 1) {
            auto victim = cache.find(lru.back());
            cacheBytes -= victim->second.first->size();
            cache.erase(victim);
            lru.pop_back();
        }
        return data;
    }

    copc::FileReader reader;
    copc::las::LasHeader header;
    std::vector<copc::Node> nodes;
    double spacing = 0.0;
    int maxDepth = 0;
    LasRecordLayout layout;

    using Key = std::tuple<int, int, int, int>; // octree d, x, y, z
    static constexpr size_t kCacheBytes = 512u << 20;
    std::list<Key> lru;
    std::map<Key, std::pair<std::shared_ptr<const std::vector<char>>, std::list<Key>::iterator>> cache;
    size_t cacheBytes = 0;
};

TileGrid::LoadResult TileGrid::loadTile(const LoadRequest& req) {
    LoadResult res;
    res.tileIndex = req.tileIndex;
    res.resolution = req.resolution;
    if (!source) source = std::make_unique<CopcSource>(copcPath);
    CopcSource& src = *source;

    const int depth = src.depthAtResolution(req.resolution);
    const copc::Box box(req.minX, req.minY, req.maxX, req.maxY);
    // Half-open tile extents so that edge points belong to one tile only; the
    // last row/column also takes the header's max edge.
    const bool lastX = req.maxX >= fileMaxX, lastY = req.maxY >= fileMaxY;
    auto inside = [&](double x, double y) {
        return x >= req.minX && (x < req.maxX || (lastX && x <= req.maxX)) &&
               y >= req.minY && (y < req.maxY || (lastY && y <= req.maxY));
    };

    const double zRange = std::max(frame.zMax - frame.zMin, 1e-6);
    const bool withOrtho = orthoPtr && orthoPtr->hasGeo;
    const LasRecordLayout& layout = src.layout;
    for (const copc::Node& node : src.nodes) {
        if (node.key.d > depth || node.point_count <= 0 || !node.key.Intersects(src.header, box))
            continue;
        std::shared_ptr<const std::vector<char>> records = src.pointData(node);
        const std::vector<char>& data = *records; // decompressed LAS records
        size_t n = data.size() / static_cast<size_t>(layout.recordLength);
        res.positions.reserve(res.positions.size() + n * 3);
        res.colors.reserve(res.colors.size() + n * 3);
        if (withOrtho) res.orthoColors.reserve(res.orthoColors.size() + n * 3);
        for (size_t i = 0; i < n; ++i) {
            double wx, wy, wz;
            layout.xyz(data.data() + i * layout.recordLength, wx, wy, wz);
            if (!inside(wx, wy)) continue;
            glm::vec3 p = frame.toGL(wx, wy, wz);
            res.positions.insert(res.positions.end(), {p.x, p.y, p.z});
            glm::vec3 c = elevationColorRamp(static_cast<float>((wz - frame.zMin) / zRange));
            res.colors.insert(res.colors.end(), {c.r, c.g, c.b});
            if (withOrtho) {
                double ocol = (orthoPtr->A != 0) ? (wx - orthoPtr->C) / orthoPtr->A : 0;
                double orow = (orthoPtr->E != 0) ? (wy - orthoPtr->F) / orthoPtr->E : 0;
                glm::vec3 o = sampleOrthoBilinear(*orthoPtr, ocol, orow);
                res.orthoColors.insert(res.orthoColors.end(), {o.r, o.g, o.b});
            }
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

void TileGrid::init(const std::string& copcPath_, const WorldBounds& bounds,
                    const Orthophoto* ortho, const SceneFrame& frame_) {
    orthoPtr = ortho;
    copcPath = copcPath_;
    frame = frame_;
    fileMaxX = bounds.max.x;
    fileMaxY = bounds.max.y;
    useOrthoColors = ortho && ortho->hasGeo;
    double minX = bounds.min.x, minY = bounds.min.y, minZ = bounds.min.z;
    double maxX = bounds.max.x, maxY = bounds.max.y, maxZ = bounds.max.z;

    tileW = (maxX - minX) / gridX;
    tileH = (maxY - minY) / gridY;

    tiles.resize(static_cast<size_t>(gridX) * gridY);
    for (int gy = 0; gy < gridY; ++gy) {
        for (int gx = 0; gx < gridX; ++gx) {
            Tile& t = tiles[gy * gridX + gx];
            t.gx = gx; t.gy = gy;
            t.minX = minX + gx * tileW;
            t.maxX = t.minX + tileW;
            t.minY = minY + gy * tileH;
            t.maxY = t.minY + tileH;
            t.minZ = minZ; t.maxZ = maxZ;
            t.glMin = frame.toGL(t.minX, t.maxY, t.minZ);
            t.glMax = frame.toGL(t.maxX, t.minY, t.maxZ);
            t.glCenter = (t.glMin + t.glMax) * 0.5f;
            t.glRadius = glm::length(t.glMax - t.glMin) * 0.5f;
        }
    }

    worker = std::thread(&TileGrid::loaderRun, this);

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

TileGrid::TileGrid() = default;

TileGrid::~TileGrid() {
    stop();
    for (auto& t : tiles) releaseTileGL(t);
}

void TileGrid::releaseTileGL(Tile& t) {
    if (t.vao) glDeleteVertexArrays(1, &t.vao);
    if (t.vboPos) glDeleteBuffers(1, &t.vboPos);
    if (t.vboCol) glDeleteBuffers(1, &t.vboCol);
    if (t.vboOrthoCol) glDeleteBuffers(1, &t.vboOrthoCol);
    t.vao = t.vboPos = t.vboCol = t.vboOrthoCol = 0;
    t.pointCount = 0;
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
    if (!r.orthoColors.empty()) {
        glGenBuffers(1, &t.vboOrthoCol);
        glBindBuffer(GL_ARRAY_BUFFER, t.vboOrthoCol);
        glBufferData(GL_ARRAY_BUFFER, r.orthoColors.size() * sizeof(float),
                     r.orthoColors.data(), GL_STATIC_DRAW);
    }
    glBindBuffer(GL_ARRAY_BUFFER, (useOrthoColors && t.vboOrthoCol) ? t.vboOrthoCol : t.vboCol);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glBindVertexArray(0);
}

void TileGrid::setUseOrthoColors(bool useOrtho) {
    useOrthoColors = useOrtho;
    for (Tile& t : tiles) {
        if (!t.vao) continue;
        glBindVertexArray(t.vao);
        glBindBuffer(GL_ARRAY_BUFFER, (useOrtho && t.vboOrthoCol) ? t.vboOrthoCol : t.vboCol);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    }
    glBindVertexArray(0);
}

// ===========================================================================
// desiredResolution: point spacing (m) giving ~1 point per 25 px² of the
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
// render: draw loaded tiles, skipping those outside the frustum or hidden
// behind last frame's depth.
// ===========================================================================

void TileGrid::render(const RenderContext& ctx) {
    const ViewSettings& vs = *ctx.settings;
    GLuint prog = ctx.programs->point;
    glUseProgram(prog);
    glUniformMatrix4fv(glGetUniformLocation(prog, "uView"), 1, GL_FALSE, glm::value_ptr(ctx.view));
    glUniformMatrix4fv(glGetUniformLocation(prog, "uProj"), 1, GL_FALSE, glm::value_ptr(ctx.proj));
    glUniform1f(glGetUniformLocation(prog, "uViewportH"), ctx.viewportH);
    glUniform1f(glGetUniformLocation(prog, "uZScale"), vs.zScale);
    glUniform1f(glGetUniformLocation(prog, "uTanHalfFov"), std::tan(glm::radians(ctx.fovDeg) * 0.5f));
    glUniform1f(glGetUniformLocation(prog, "uDisableSubsampling"), 0.0f);
    glUniform1f(glGetUniformLocation(prog, "uOrtho"), ctx.ortho ? 1.0f : 0.0f);
    glUniform1f(glGetUniformLocation(prog, "uOrthoHeight"), ctx.orthoHeight);
    glUniform1f(glGetUniformLocation(prog, "uPointSize"), 0.0015f * vs.pointSizeMul);
    glUniform1f(glGetUniformLocation(prog, "uTargetPixelSpacing"), 2.83f);
    glUniform1f(glGetUniformLocation(prog, "uDensityMul"), vs.pointDensityMul);
    GLint densityLoc = glGetUniformLocation(prog, "uDensity");

    glm::mat4 VP = ctx.proj * ctx.view;
    bool occlusion = vs.useOcclusion && ctx.hiz && ctx.hiz->ready();
    drawnTiles = drawnPoints = culledTiles = 0;
    for (Tile& t : tiles) {
        if (!t.hasGeometry()) continue;
        glm::vec3 lo = t.glMin, hi = t.glMax;
        lo.y *= vs.zScale;
        hi.y *= vs.zScale;
        if (aabbOutsideFrustum(lo, hi, VP) || (occlusion && ctx.hiz->isOccluded(lo, hi, VP))) {
            ++culledTiles;
            continue;
        }
        float tileDensity = (t.pointSpacing > 1e-10f)
            ? 1.0f / (t.pointSpacing * t.pointSpacing) : 1e15f;
        glUniform1f(densityLoc, tileDensity);
        glBindVertexArray(t.vao);
        glDrawArrays(GL_POINTS, 0, t.pointCount);
        ++drawnTiles;
        drawnPoints += static_cast<size_t>(t.pointCount);
    }
    glBindVertexArray(0);
}
