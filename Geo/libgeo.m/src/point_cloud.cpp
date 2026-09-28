// point_cloud.cpp — LAZ/LAS point cloud loading (laz-perf).
//
//   - loadPointCloud()  : full load into the scene frame, RGB or elevation
//                         colors, XY-grid thinning for very large clouds.
//   - readCloudHeader() : header-only bounds, point count, CRS.
#include "point_cloud.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "las_format.h"
#include "raster.h"

#include <lazperf/readers.hpp>

#include <iostream>
#include <cmath>
#include <algorithm>
#include <thread>
#include <atomic>
#include <vector>
#include <string>

#ifndef LASVIEWER_NO_OPENMP
#  ifdef _OPENMP
#    include <omp.h>
#    define LASVIEWER_HAS_OPENMP 1
#  else
#    define LASVIEWER_HAS_OPENMP 0
#  endif
#else
#  define LASVIEWER_HAS_OPENMP 0
#endif

// ---------------------------------------------------------------------------
// Elevation color ramp (blue → cyan → green → yellow → red).
// Used when a cloud has no RGB channel.
// ---------------------------------------------------------------------------
static glm::vec3 elevationColor(double z, double zMin, double zRange) {
    double t = (z - zMin) / zRange;
    t = glm::clamp(t, 0.0, 1.0);
    static const glm::vec3 stops[5] = {
        {0.1f, 0.2f, 0.8f},  // blue
        {0.2f, 0.7f, 0.9f},  // cyan
        {0.3f, 0.8f, 0.3f},  // green
        {0.95f, 0.85f, 0.2f},// yellow
        {0.85f, 0.25f, 0.2f} // red
    };
    float s = static_cast<float>(t) * 4.0f;
    int i = static_cast<int>(s);
    if (i >= 4) return stops[4];
    float f = s - i;
    return glm::mix(stops[i], stops[i + 1], f);
}

// ---------------------------------------------------------------------------
// LAZ/LAS loader
// ---------------------------------------------------------------------------

namespace {

// Record layout of an open laz-perf reader's file.
bool layoutOf(const lazperf::header14& h, LasRecordLayout& out) {
    return lasRecordLayout(h.point_format_id, h.point_record_length,
                           glm::dvec3(h.scale.x, h.scale.y, h.scale.z),
                           glm::dvec3(h.offset.x, h.offset.y, h.offset.z), out);
}

} // namespace

bool loadPointCloud(const std::string& path, const SceneFrame& frame, PointCloud& cloud) {
    lazperf::reader::named_file reader(path); // throws lazperf::error on bad input
    LasRecordLayout layout;
    if (!layoutOf(reader.header(), layout)) {
        std::cerr << "ERROR: unsupported LAS point format "
                  << int(reader.header().point_format_id & 0x3F) << " in " << path << std::endl;
        return false;
    }
    const bool hasRGB = layout.hasRGB();
    const uint64_t count = reader.pointCount();

    // First pass: decode every record, and the bbox for recentering.
    std::vector<double> rawPos; // x,y,z
    std::vector<uint16_t> rawRGB;
    rawPos.reserve(count * 3);
    if (hasRGB) rawRGB.reserve(count * 3);
    std::vector<char> rec(static_cast<size_t>(layout.recordLength));
    bool first = true;
    glm::dvec3 bmin{0}, bmax{0};
    for (uint64_t i = 0; i < count; ++i) {
        reader.readPoint(rec.data());
        double x, y, z;
        layout.xyz(rec.data(), x, y, z);
        rawPos.insert(rawPos.end(), {x, y, z});
        if (hasRGB) {
            uint16_t c[3];
            layout.rgb(rec.data(), c);
            rawRGB.insert(rawRGB.end(), {c[0], c[1], c[2]});
        }
        glm::dvec3 p(x, y, z);
        if (first) { bmin = bmax = p; first = false; }
        else { bmin = glm::min(bmin, p); bmax = glm::max(bmax, p); }
    }

    if (rawPos.empty()) {
        std::cerr << "ERROR: no points read from: " << path << std::endl;
        return false;
    }

    cloud.bboxMin = bmin;
    cloud.bboxMax = bmax;
    cloud.worldCenter = frame.center;
    cloud.worldScale = frame.scale;
    cloud.hasRGB = hasRGB;

    // Second pass: recenter + rescale, build colors (RGB or elevation gradient).
    cloud.pointCount = rawPos.size() / 3;
    cloud.positions.resize(rawPos.size());
    cloud.colors.resize(rawPos.size());

    double invScale = 1.0 / cloud.worldScale;
    double zMin = frame.zMin;
    double zRange = std::max(frame.zMax - frame.zMin, 1e-9);

    for (size_t i = 0; i < cloud.pointCount; ++i) {
        double wx = rawPos[i * 3 + 0];
        double wy = rawPos[i * 3 + 1];
        double wz = rawPos[i * 3 + 2];
        // World X -> GL X (east), world Z (elevation) -> GL Y (up),
        // world Y (northing) -> GL Z, NEGATED so that world north maps to
        // GL -Z (away from the default camera).
        cloud.positions[i * 3 + 0] = static_cast<float>((wx - cloud.worldCenter.x) * invScale);
        cloud.positions[i * 3 + 1] = static_cast<float>((wz - cloud.worldCenter.z) * invScale);
        cloud.positions[i * 3 + 2] = static_cast<float>(-(wy - cloud.worldCenter.y) * invScale);

        glm::vec3 col;
        if (hasRGB) {
            col.r = rawRGB[i * 3 + 0] / 65535.0f;
            col.g = rawRGB[i * 3 + 1] / 65535.0f;
            col.b = rawRGB[i * 3 + 2] / 65535.0f;
        } else {
            col = elevationColor(wz, zMin, zRange);
        }
        cloud.colors[i * 3 + 0] = col.r;
        cloud.colors[i * 3 + 1] = col.g;
        cloud.colors[i * 3 + 2] = col.b;
    }

    rawPos.clear();
    rawPos.shrink_to_fit();
    rawRGB.clear();
    rawRGB.shrink_to_fit();

    // --- Spatial grid subsampling ---
    // LiDAR points are stored in scan order, so "every Nth point" leaves
    // spatial gaps. We divide the XY plane into a uniform grid and keep ONE
    // point per occupied cell (like PDAL's filters.sample).
    const size_t MAX_POINTS = 2'000'000;
    if (cloud.pointCount > MAX_POINTS) {
        double minX = cloud.bboxMin.x, maxX = cloud.bboxMax.x;
        double minY = cloud.bboxMin.y, maxY = cloud.bboxMax.y;
        double area = (maxX - minX) * (maxY - minY);
        double cellSize = std::sqrt(area / static_cast<double>(MAX_POINTS));
        if (cellSize < 1e-9) cellSize = 1e-9;

        int gridX = std::max(1, static_cast<int>((maxX - minX) / cellSize) + 1);
        int gridY = std::max(1, static_cast<int>((maxY - minY) / cellSize) + 1);
        const int MAX_GRID_DIM = 2048;
        if (gridX > MAX_GRID_DIM) gridX = MAX_GRID_DIM;
        if (gridY > MAX_GRID_DIM) gridY = MAX_GRID_DIM;
        cellSize = std::max((maxX - minX) / gridX, (maxY - minY) / gridY);

        // Each cell keeps its lowest-index point (atomic min), so the result
        // doesn't depend on thread scheduling.
        const size_t kNone = SIZE_MAX;
        std::vector<std::atomic<size_t>> cellFirst(static_cast<size_t>(gridX) * gridY);
        for (auto& c : cellFirst) c.store(kNone, std::memory_order_relaxed);
        std::vector<uint32_t> cellOf(cloud.pointCount);

#if LASVIEWER_HAS_OPENMP
        int nThreads = std::max(1, std::min((int)cloud.pointCount / 100000,
                                           (int)std::thread::hardware_concurrency()));
        #pragma omp parallel for num_threads(nThreads) schedule(static)
#endif
        for (size_t i = 0; i < cloud.pointCount; ++i) {
            double wx = cloud.positions[i * 3 + 0] * cloud.worldScale + cloud.worldCenter.x;
            double wy = -cloud.positions[i * 3 + 2] * cloud.worldScale + cloud.worldCenter.y;
            int cx = static_cast<int>((wx - minX) / cellSize);
            int cy = static_cast<int>((wy - minY) / cellSize);
            if (cx < 0) cx = 0; else if (cx >= gridX) cx = gridX - 1;
            if (cy < 0) cy = 0; else if (cy >= gridY) cy = gridY - 1;
            size_t cellIdx = static_cast<size_t>(cy) * gridX + cx;
            cellOf[i] = static_cast<uint32_t>(cellIdx);
            size_t cur = cellFirst[cellIdx].load(std::memory_order_relaxed);
            while (i < cur && !cellFirst[cellIdx].compare_exchange_weak(
                                  cur, i, std::memory_order_relaxed)) {
            }
        }

        std::vector<float> tmpPos, tmpCol;
        tmpPos.reserve(std::min(cloud.pointCount, MAX_POINTS) * 3);
        tmpCol.reserve(std::min(cloud.pointCount, MAX_POINTS) * 3);
        for (size_t i = 0; i < cloud.pointCount && tmpPos.size() < MAX_POINTS * 3; ++i) {
            if (cellFirst[cellOf[i]].load(std::memory_order_relaxed) != i) continue;
            tmpPos.insert(tmpPos.end(), cloud.positions.begin() + i * 3,
                          cloud.positions.begin() + i * 3 + 3);
            tmpCol.insert(tmpCol.end(), cloud.colors.begin() + i * 3,
                          cloud.colors.begin() + i * 3 + 3);
        }
        size_t outIdx = tmpPos.size() / 3;
        cloud.positions = std::move(tmpPos);
        cloud.colors = std::move(tmpCol);
        std::cerr << "  spatial subsample: " << cloud.pointCount << " -> " << outIdx
                  << " (grid " << gridX << "x" << gridY
                  << ", cellSize=" << cellSize << "m)" << std::endl;
        cloud.pointCount = outIdx;
    }

    // Compute the GL-space bbox (after Z-up swap) for orthophoto clipping and
    // dynamic near/far plane computation.
    {
        float minX = 1e30f, maxX = -1e30f, minZ = 1e30f, maxZ = -1e30f;
        float minY = 1e30f, maxY = -1e30f;
        for (size_t i = 0; i < cloud.pointCount; ++i) {
            float x = cloud.positions[i * 3 + 0];
            float y = cloud.positions[i * 3 + 1];
            float z = cloud.positions[i * 3 + 2];
            if (x < minX) minX = x;
            if (x > maxX) maxX = x;
            if (y < minY) minY = y;
            if (y > maxY) maxY = y;
            if (z < minZ) minZ = z;
            if (z > maxZ) maxZ = z;
        }
        cloud.glBBoxMin = glm::vec2(minX, minZ);
        cloud.glBBoxMax = glm::vec2(maxX, maxZ);
        cloud.glBBoxMinY = minY;
        cloud.glBBoxMaxY = maxY;
        float glArea = (maxX - minX) * (maxZ - minZ);
        cloud.glDensity = (glArea > 1e-12f)
            ? static_cast<float>(cloud.pointCount) / glArea
            : 1.0f;
    }

    std::cerr << "Loaded " << cloud.pointCount << " points from " << path << "\n"
              << "  bbox: (" << bmin.x << ", " << bmin.y << ", " << bmin.z << ")"
              << " -> (" << bmax.x << ", " << bmax.y << ", " << bmax.z << ")\n"
              << "  RGB: " << (hasRGB ? "yes" : "no (using elevation gradient)") << std::endl;
    return true;
}

// ---------------------------------------------------------------------------
// Header-only read: bounds, point count, CRS.
// ---------------------------------------------------------------------------

bool readCloudHeader(const std::string& path, CloudHeader& out) {
    lazperf::reader::named_file reader(path); // throws lazperf::error on bad input
    const lazperf::header14& h = reader.header();
    out.pointCount = reader.pointCount();
    out.bounds.extendXY(h.minx, h.miny, h.maxx, h.maxy);
    out.bounds.extendZ(h.minz, h.maxz);
    out.wkt = lasCrsWkt(path);
    out.epsg = horizontalEPSG(out.wkt);
    return out.pointCount > 0 && out.bounds.valid();
}
