// point_cloud.cpp — LAZ/LAS point cloud loading (laz-perf).
//
//   - loadPointCloud()  : full load into the scene frame, RGB or elevation
//                         colors, XY-grid thinning for very large clouds,
//                         in one streaming pass.
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
#include <limits>
#include <vector>
#include <string>

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

// Clouds over this many points are thinned to about this many.
constexpr size_t kMaxPoints = 2'000'000;
// Thinning grid size limit per axis (for very elongated clouds).
constexpr int kMaxGridDim = 2048;

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
    const lazperf::header14& hdr = reader.header();
    LasRecordLayout layout;
    if (!layoutOf(hdr, layout)) {
        std::cerr << "ERROR: unsupported LAS point format "
                  << int(hdr.point_format_id & 0x3F) << " in " << path << std::endl;
        return false;
    }
    const bool hasRGB = layout.hasRGB();
    const uint64_t count = reader.pointCount();

    // Thinning grid. Clouds over kMaxPoints keep one point per cell of an XY
    // grid of about kMaxPoints cells: the first point of each cell in file
    // order (LiDAR is stored in scan order, so "every Nth point" would leave
    // gaps). The grid comes from the header bounds, so the file is read
    // once and only the kept points are stored: memory is the grid (4 bytes
    // a cell) plus the kept points, not the whole cloud. Points outside the
    // header bounds (a sloppy header) fall in the border cells.
    double minX = hdr.minx, maxX = hdr.maxx, minY = hdr.miny, maxY = hdr.maxy;
    const bool thin = count > kMaxPoints;
    if (thin && !(minX <= maxX && minY <= maxY)) {
        // No usable header bounds: one extra pass for the extent.
        lazperf::reader::named_file scan(path);
        std::vector<char> rec(static_cast<size_t>(layout.recordLength));
        minX = minY = std::numeric_limits<double>::max();
        maxX = maxY = std::numeric_limits<double>::lowest();
        for (uint64_t i = 0; i < count; ++i) {
            scan.readPoint(rec.data());
            double x, y, z;
            layout.xyz(rec.data(), x, y, z);
            minX = std::min(minX, x); maxX = std::max(maxX, x);
            minY = std::min(minY, y); maxY = std::max(maxY, y);
        }
    }
    int gridX = 1, gridY = 1;
    double cellSize = 1.0;
    if (thin) {
        double area = std::max((maxX - minX) * (maxY - minY), 1e-12);
        cellSize = std::max(std::sqrt(area / static_cast<double>(kMaxPoints)), 1e-9);
        gridX = std::clamp(static_cast<int>((maxX - minX) / cellSize) + 1, 1, kMaxGridDim);
        gridY = std::clamp(static_cast<int>((maxY - minY) / cellSize) + 1, 1, kMaxGridDim);
        cellSize = std::max({(maxX - minX) / gridX, (maxY - minY) / gridY, 1e-9});
    }
    std::vector<uint8_t> cellTaken(thin ? static_cast<size_t>(gridX) * gridY : 0, 0);

    cloud.worldCenter = frame.center;
    cloud.worldScale = frame.scale;
    cloud.hasRGB = hasRGB;
    cloud.positions.clear();
    cloud.colors.clear();
    const size_t expected = thin ? cellTaken.size() : static_cast<size_t>(count);
    cloud.positions.reserve(expected * 3);
    cloud.colors.reserve(expected * 3);

    const double invScale = 1.0 / cloud.worldScale;
    const double zMin = frame.zMin;
    const double zRange = std::max(frame.zMax - frame.zMin, 1e-9);
    std::vector<char> rec(static_cast<size_t>(layout.recordLength));
    bool first = true;
    glm::dvec3 bmin{0}, bmax{0};
    for (uint64_t i = 0; i < count; ++i) {
        reader.readPoint(rec.data());
        double x, y, z;
        layout.xyz(rec.data(), x, y, z);
        glm::dvec3 p(x, y, z);
        if (first) { bmin = bmax = p; first = false; }
        else { bmin = glm::min(bmin, p); bmax = glm::max(bmax, p); }
        if (thin) {
            int cx = std::clamp(static_cast<int>((x - minX) / cellSize), 0, gridX - 1);
            int cy = std::clamp(static_cast<int>((y - minY) / cellSize), 0, gridY - 1);
            uint8_t& taken = cellTaken[static_cast<size_t>(cy) * gridX + cx];
            if (taken) continue;
            taken = 1;
        }
        // World X -> GL X (east), world Z (elevation) -> GL Y (up), world Y
        // (northing) -> GL Z, negated so that north is GL -Z.
        cloud.positions.insert(cloud.positions.end(),
                               {static_cast<float>((x - cloud.worldCenter.x) * invScale),
                                static_cast<float>((z - cloud.worldCenter.z) * invScale),
                                static_cast<float>(-(y - cloud.worldCenter.y) * invScale)});
        glm::vec3 col;
        if (hasRGB) {
            uint16_t c[3];
            layout.rgb(rec.data(), c);
            col = glm::vec3(c[0], c[1], c[2]) / 65535.0f;
        } else {
            col = elevationColor(z, zMin, zRange);
        }
        cloud.colors.insert(cloud.colors.end(), {col.r, col.g, col.b});
    }

    if (cloud.positions.empty()) {
        std::cerr << "ERROR: no points read from: " << path << std::endl;
        return false;
    }
    cloud.bboxMin = bmin;
    cloud.bboxMax = bmax;
    cloud.pointCount = cloud.positions.size() / 3;
    cloud.positions.shrink_to_fit();
    cloud.colors.shrink_to_fit();
    if (thin) {
        std::cerr << "  spatial subsample: " << count << " -> " << cloud.pointCount << " (grid "
                  << gridX << "x" << gridY << ", cellSize=" << cellSize << "m)" << std::endl;
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
