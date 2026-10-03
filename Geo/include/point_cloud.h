// point_cloud.h — LAZ/LAS point cloud loading (laz-perf)
#pragma once
#include "scene_frame.h"

#include <glm/glm.hpp>

#include <cstdint>
#include <string>
#include <vector>

struct PointCloud {
    std::vector<float> positions; // GL space, 3 floats/point
    std::vector<float> colors;    // file RGB, or elevation ramp
    bool hasRGB = false;
    std::vector<float> orthoColors;
    bool hasOrthoColors = false;
    size_t pointCount = 0;
    glm::dvec3 worldCenter{0.0};
    double worldScale = 1.0;
    glm::dvec3 bboxMin{0.0}, bboxMax{0.0};
    glm::vec2 glBBoxMin{0.0f}, glBBoxMax{0.0f};
    float glBBoxMinY = 0.0f, glBBoxMaxY = 0.0f;
    float glDensity = 1.0f;
};

// Load a LAZ/LAS/COPC point cloud in full, into the given scene frame, in
// one pass. Clouds over 2M points are thinned to about 2M on an XY grid (the
// first point of each cell, in file order); memory holds only the kept
// points. Throws lazperf::error (a std::runtime_error) on an unreadable
// file; returns false for an unsupported format or an empty cloud.
bool loadPointCloud(const std::string& path, const SceneFrame& frame, PointCloud& cloud);

struct CloudHeader {
    WorldBounds bounds; // hasZ = true
    uint64_t pointCount = 0;
    int epsg = 0;    // horizontal CRS, 0 = unknown
    std::string wkt; // full CRS, empty = unknown
};
// Header-only read: bounds, point count, CRS. Throws lazperf::error (a
// std::runtime_error) on an unreadable file.
bool readCloudHeader(const std::string& path, CloudHeader& out);
