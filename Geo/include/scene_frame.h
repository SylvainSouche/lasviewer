// scene_frame.h — the world→GL transform shared by every layer of a scene.
//
//   GL_X =  (worldX - center.x) / scale    (easting)
//   GL_Y =  (worldZ - center.z) / scale    (elevation, up)
//   GL_Z = -(worldY - center.y) / scale    (northing, negated for north-up)
//
// All layers must use the same frame so that adjacent tiles and different
// data types (points, DEMs) line up. The frame is fixed before any layer is
// loaded, from the inputs' header/tag extents.
#pragma once
#include <glm/glm.hpp>
#include <algorithm>
#include <limits>
#include <string>

struct WorldBounds {
    glm::dvec3 min{std::numeric_limits<double>::max()};
    glm::dvec3 max{std::numeric_limits<double>::lowest()};
    bool hasZ = false; // min.z/max.z are meaningful

    bool valid() const { return min.x <= max.x && min.y <= max.y; }
    void extendXY(double x0, double y0, double x1, double y1) {
        min.x = std::min(min.x, x0); max.x = std::max(max.x, x1);
        min.y = std::min(min.y, y0); max.y = std::max(max.y, y1);
    }
    void extendZ(double z0, double z1) {
        min.z = std::min(min.z, z0); max.z = std::max(max.z, z1);
        hasZ = true;
    }
};

struct SceneFrame {
    glm::dvec3 center{0.0};
    double scale = 1.0;
    // Elevation range used by point-cloud elevation ramps, so that every
    // layer/tile maps the same elevation to the same color.
    double zMin = 0.0, zMax = 1.0;
    // The scene's CRS (WKT; empty = unknown). Rasters in another horizontal
    // CRS are warped into it on load.
    std::string crsWkt;

    glm::vec3 toGL(double x, double y, double z) const {
        double inv = 1.0 / scale;
        return glm::vec3(static_cast<float>((x - center.x) * inv),
                         static_cast<float>((z - center.z) * inv),
                         static_cast<float>(-(y - center.y) * inv));
    }

    // Inverse of toGL (GL Y must not include the Z exaggeration).
    glm::dvec3 toWorld(const glm::vec3& gl) const {
        return glm::dvec3(center.x + gl.x * scale, center.y - gl.z * scale,
                          center.z + gl.y * scale);
    }

    static SceneFrame fromBounds(const WorldBounds& b) {
        SceneFrame f;
        if (!b.valid()) return f;
        double zLo = b.hasZ ? b.min.z : 0.0, zHi = b.hasZ ? b.max.z : 0.0;
        f.center = glm::dvec3((b.min.x + b.max.x) * 0.5, (b.min.y + b.max.y) * 0.5,
                              (zLo + zHi) * 0.5);
        f.scale = glm::length(glm::dvec3(b.max.x - b.min.x, b.max.y - b.min.y, zHi - zLo));
        if (f.scale < 1e-9) f.scale = 1.0;
        f.zMin = zLo;
        f.zMax = (zHi - zLo > 1e-9) ? zHi : zLo + 1.0;
        return f;
    }
};

// Axis-aligned box in GL space (Y not yet multiplied by the Z exaggeration).
struct GLBounds {
    glm::vec3 min{std::numeric_limits<float>::max()};
    glm::vec3 max{std::numeric_limits<float>::lowest()};

    bool valid() const { return min.x <= max.x && min.y <= max.y && min.z <= max.z; }
    void extend(const GLBounds& o) {
        if (!o.valid()) return;
        min = glm::min(min, o.min);
        max = glm::max(max, o.max);
    }
    glm::vec3 center() const { return (min + max) * 0.5f; }
};
