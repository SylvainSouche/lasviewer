// point_cloud.h — LAZ/LAS point cloud loading via PDAL
#pragma once
#include "gl_platform.h"
#include <glm/glm.hpp>
#include <string>
#include <vector>
#include <cstdint>

struct PointCloud {
    std::vector<float> positions;
    std::vector<float> colors;
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

// Load a LAZ/LAS/COPC point cloud (full load, not streaming).
bool loadPointCloud(const std::string& path, PointCloud& cloud);

// Get bbox from header only (no point data loaded).
bool getCloudBounds(const std::string& path,
                    double& minX, double& minY, double& minZ,
                    double& maxX, double& maxY, double& maxZ);

// Upload point cloud to GPU. Returns VAO, writes color VBO to outColorVBO.
GLuint uploadPointCloudGL(const PointCloud& cloud, bool useOrthoColors,
                          GLuint* outColorVBO = nullptr);
