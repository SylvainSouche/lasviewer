// point_cloud.h — LAZ/LAS point cloud loading via PDAL
#pragma once
#include "gl_platform.h"
#include "scene_frame.h"
#include <glm/glm.hpp>
#include <string>
#include <vector>
#include <cstdint>

struct PointCloud {
    std::vector<float> positions;   // GL space, 3 floats/point
    std::vector<float> colors;      // file RGB, or elevation ramp
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

// Load a LAZ/LAS/COPC point cloud in full, into the given scene frame.
// Clouds over 2M points are thinned on an XY grid.
bool loadPointCloud(const std::string& path, const SceneFrame& frame, PointCloud& cloud);

struct CloudHeader {
    WorldBounds bounds;   // hasZ = true
    uint64_t pointCount = 0;
    int epsg = 0;         // horizontal CRS, 0 = unknown
};
// Header-only read (no points loaded, except as a fallback for bounds).
bool readCloudHeader(const std::string& path, CloudHeader& out);
