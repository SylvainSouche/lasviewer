// dem_mesh.h — Adaptive triangulated DEM mesh with texture
#pragma once
#include "gl_platform.h"
#include <glm/glm.hpp>
#include <string>
#include <vector>
#include <cstdint>
#include <tiffio.h>

struct Orthophoto; // forward decl

// Read the first band of a (possibly tiled) TIFF as Float32 elevation
// values, including Terrain RGB decoding. Shared with dem_tess_mesh.cpp so
// the GPU-tessellated DEM path doesn't duplicate format-handling logic.
// Defined in dem_mesh.cpp.
bool readDEMElevations(TIFF* tif, uint32_t w, uint32_t h,
                       std::vector<float>& out, uint16_t& outSpp);

// Reads the GeoTIFF's own declared NODATA value (the GDAL_NODATA tag,
// 42113 — a de facto standard, not part of the baseline TIFF spec, hence
// not a libtiff-predefined constant) if present. Returns true and sets
// outNodata if found; false if the file has no declared nodata value, in
// which case callers should fall back to a heuristic threshold rather
// than assume there's no nodata at all. This matters: without it, nodata
// detection previously only caught the common "< -9000" convention
// (e.g. -9999) — a DEM using a different sentinel (0, a large negative
// float-min value, etc.) would have those pixels silently treated as
// valid elevation, generating real geometry over what should have been
// excluded (reported as "displays geometry where there's only
// orthophoto"). Shared with dem_tess_mesh.cpp, defined in dem_mesh.cpp.
bool readDEMNodataValue(TIFF* tif, float& outNodata);

struct DEMMesh {
    std::vector<float> vertices;
    std::vector<float> uvs;
    std::vector<uint32_t> indices;
    GLuint texture = 0;
    int texW = 0, texH = 0;
    glm::dvec3 worldCenter{0.0};
    double worldScale = 1.0;
    glm::dvec3 bboxMin{0.0}, bboxMax{0.0};
    glm::vec2 glBBoxMin{0.0f}, glBBoxMax{0.0f};
    float glBBoxMinY = 0.0f, glBBoxMaxY = 0.0f;
    bool valid = false;
    // False specifically when the orthophoto HAS geo tags but genuinely
    // does not geographically overlap the DEM — see DEMTessMesh's
    // identical field (dem_tess_mesh.h) for the full reasoning. main.cpp
    // checks this before uploading `texture`, skipping it entirely
    // (falls back to the elevation color ramp) rather than stretching an
    // unrelated image onto the mesh.
    bool orthoUsable = true;

    // For reload() (I/O keys, point-collapsing angle — main.cpp): the DEM
    // path (re-read from disk, same reasoning as DEMTessMesh's equivalent
    // — no second full-resolution CPU copy kept resident) and the angle
    // threshold used by the last load.
    std::string sourcePath;
    double collapseAngleDeg = 1.0;
    // Quadtree depth ceiling (S/F keys) — see DEMTessMesh::maxLevel's
    // comment in dem_tess_mesh.h for the full reasoning. Default 6, not 5
    // — this is the CPU-only fallback path with no GPU tessellation to
    // compensate for a shallower quadtree, so it never got the same
    // walked-back coarsening DEMTessMesh did; kept at its original depth.
    int maxLevel = 6;

    bool loadFromDEM(const std::string& path, const Orthophoto* ortho,
                     double angleThresholdDeg = 1.0, int maxLevelParam = 6);

    // Full rebuild with a new point-collapsing angle and/or quadtree depth
    // ceiling — changes the quadtree subdivision decision itself (§4b of
    // docs/design-tessellation-displacement.md), so vertices/uvs/indices
    // must be regenerated. Does NOT touch `texture` or require the caller
    // to re-upload it: the orthophoto's content is unaffected by either
    // parameter, loadFromDEM() never sets `texture` either way,
    // and its CPU-side pixel buffer is freed right after the initial
    // upload in main.cpp — there'd be nothing to re-upload from even if
    // this tried to. Only invalidates the mesh VAO/VBO/IBO, which
    // render()'s existing "create on first render" check then recreates
    // from the rebuilt geometry. Synchronous — will hitch the frame.
    bool reload(const Orthophoto* ortho, double newAngleThresholdDeg, int newMaxLevel);

    void render(GLuint meshProgram, const glm::mat4& V, const glm::mat4& P,
                float zScale, bool showMasterEdges);
    void destroy();
};
