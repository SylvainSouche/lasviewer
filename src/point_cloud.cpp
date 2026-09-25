// point_cloud.cpp — LAZ/LAS point cloud loading (PDAL).
//
//   - loadPointCloud()  : full load into the scene frame, RGB or elevation
//                         colors, XY-grid thinning for very large clouds.
//   - readCloudHeader() : header-only bounds, point count, CRS.
#include "point_cloud.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Options.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/Stage.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/Metadata.hpp>
#include <pdal/SpatialReference.hpp>

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
// LAZ/LAS loader (PDAL)
// ---------------------------------------------------------------------------

bool loadPointCloud(const std::string& path, const SceneFrame& frame, PointCloud& cloud) {
    pdal::StageFactory factory;
    std::string driver = factory.inferReaderDriver(path);
    if (driver.empty()) {
        std::cerr << "ERROR: could not infer PDAL driver for: " << path << std::endl;
        return false;
    }

    pdal::Stage* reader = factory.createStage(driver);
    if (!reader) {
        std::cerr << "ERROR: could not create PDAL stage: " << driver << std::endl;
        return false;
    }

    pdal::Options options;
    options.add("filename", path);
    reader->setOptions(options);

    pdal::PointTable table;
    reader->prepare(table);
    pdal::PointViewSet views = reader->execute(table);

    pdal::PointLayoutPtr layout = table.layout();
    bool hasRGB = layout->hasDim(pdal::Dimension::Id::Red) &&
                  layout->hasDim(pdal::Dimension::Id::Green) &&
                  layout->hasDim(pdal::Dimension::Id::Blue);
    bool hasZ  = layout->hasDim(pdal::Dimension::Id::Z);

    // First pass: compute bbox in world coords so we can recenter.
    std::vector<double> rawPos; // x,y,z
    std::vector<uint16_t> rawRGB;
    bool first = true;
    glm::dvec3 bmin{0}, bmax{0};

    for (const auto& view : views) {
        for (pdal::PointId i = 0; i < view->size(); ++i) {
            double x = view->getFieldAs<double>(pdal::Dimension::Id::X, i);
            double y = view->getFieldAs<double>(pdal::Dimension::Id::Y, i);
            double z = hasZ ? view->getFieldAs<double>(pdal::Dimension::Id::Z, i) : 0.0;
            rawPos.push_back(x);
            rawPos.push_back(y);
            rawPos.push_back(z);
            if (hasRGB) {
                rawRGB.push_back(view->getFieldAs<uint16_t>(pdal::Dimension::Id::Red,   i));
                rawRGB.push_back(view->getFieldAs<uint16_t>(pdal::Dimension::Id::Green, i));
                rawRGB.push_back(view->getFieldAs<uint16_t>(pdal::Dimension::Id::Blue,  i));
            }
            glm::dvec3 p(x, y, z);
            if (first) { bmin = bmax = p; first = false; }
            else { bmin = glm::min(bmin, p); bmax = glm::max(bmax, p); }
        }
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
    // point per occupied cell — same approach as PDAL's filters.sample.
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

        std::vector<std::atomic<uint8_t>> occupied(static_cast<size_t>(gridX) * gridY);
        for (auto& o : occupied) o.store(0, std::memory_order_relaxed);

        std::vector<size_t> keepIdx(cloud.pointCount, SIZE_MAX);
        std::atomic<size_t> outCount{0};

#if LASVIEWER_HAS_OPENMP
        int nThreads = std::max(1, std::min((int)cloud.pointCount / 100000,
                                           (int)std::thread::hardware_concurrency()));
        #pragma omp parallel for num_threads(nThreads) schedule(static)
#endif
        for (size_t i = 0; i < cloud.pointCount; ++i) {
            double wx = cloud.positions[i * 3 + 0] * cloud.worldScale + cloud.worldCenter.x;
            double wy = cloud.positions[i * 3 + 2] * cloud.worldScale + cloud.worldCenter.y;
            int cx = static_cast<int>((wx - minX) / cellSize);
            int cy = static_cast<int>((wy - minY) / cellSize);
            if (cx < 0) cx = 0; else if (cx >= gridX) cx = gridX - 1;
            if (cy < 0) cy = 0; else if (cy >= gridY) cy = gridY - 1;
            size_t cellIdx = static_cast<size_t>(cy) * gridX + cx;
            uint8_t expected = 0;
            if (occupied[cellIdx].compare_exchange_strong(expected, 1,
                    std::memory_order_relaxed)) {
                size_t outIdx = outCount.fetch_add(1, std::memory_order_relaxed);
                if (outIdx < MAX_POINTS) {
                    keepIdx[i] = outIdx;
                }
            }
        }

        size_t outIdx = outCount.load(std::memory_order_relaxed);
        if (outIdx > MAX_POINTS) outIdx = MAX_POINTS;
        std::vector<float> tmpPos(outIdx * 3), tmpCol(outIdx * 3);
        for (size_t i = 0; i < cloud.pointCount; ++i) {
            if (keepIdx[i] < outIdx) {
                size_t o = keepIdx[i];
                tmpPos[o * 3 + 0] = cloud.positions[i * 3 + 0];
                tmpPos[o * 3 + 1] = cloud.positions[i * 3 + 1];
                tmpPos[o * 3 + 2] = cloud.positions[i * 3 + 2];
                tmpCol[o * 3 + 0] = cloud.colors[i * 3 + 0];
                tmpCol[o * 3 + 1] = cloud.colors[i * 3 + 1];
                tmpCol[o * 3 + 2] = cloud.colors[i * 3 + 2];
            }
        }
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
    pdal::StageFactory factory;
    std::string driver = factory.inferReaderDriver(path);
    if (driver.empty()) return false;
    pdal::Stage* reader = factory.createStage(driver);
    if (!reader) return false;
    pdal::Options options;
    options.add("filename", path);
    reader->setOptions(options);
    pdal::PointTable table;
    reader->prepare(table);

    try {
        pdal::SpatialReference srs = reader->getSpatialReference();
        if (srs.empty()) srs = table.anySpatialReference();
        if (!srs.empty()) {
            out.wkt = srs.getWKT();
            std::string code = srs.identifyHorizontalEPSG();
            if (!code.empty()) out.epsg = std::stoi(code);
        }
    } catch (const std::exception&) {
        out.epsg = 0;
    }

    pdal::MetadataNode meta = table.metadata();
    pdal::MetadataNode reader_meta = meta.findChild(driver);
    pdal::MetadataNode countNode = meta.findChild("count");
    if (countNode.empty() && !reader_meta.empty()) countNode = reader_meta.findChild("count");
    if (!countNode.empty()) {
        try { out.pointCount = std::stoull(countNode.value()); } catch (const std::exception&) {}
    }

    pdal::MetadataNode boundsNode = meta.findChild("bounds");
    if (!boundsNode.empty()) {
        try {
            out.bounds.extendXY(std::stod(boundsNode.findChild("minx").value()),
                                std::stod(boundsNode.findChild("miny").value()),
                                std::stod(boundsNode.findChild("maxx").value()),
                                std::stod(boundsNode.findChild("maxy").value()));
            out.bounds.extendZ(std::stod(boundsNode.findChild("minz").value()),
                               std::stod(boundsNode.findChild("maxz").value()));
            return true;
        } catch (const std::exception& e) {
            std::cerr << "WARNING: could not parse PDAL bounds metadata (" << e.what()
                      << ") — scanning points instead" << std::endl;
        }
    }
    // Fallback: read every point.
    pdal::PointViewSet views = reader->execute(table);
    uint64_t n = 0;
    for (const auto& view : views) {
        for (pdal::PointId i = 0; i < view->size(); ++i, ++n) {
            double x = view->getFieldAs<double>(pdal::Dimension::Id::X, i);
            double y = view->getFieldAs<double>(pdal::Dimension::Id::Y, i);
            double z = view->getFieldAs<double>(pdal::Dimension::Id::Z, i);
            out.bounds.extendXY(x, y, x, y);
            out.bounds.extendZ(z, z);
        }
    }
    out.pointCount = n;
    return n > 0;
}
