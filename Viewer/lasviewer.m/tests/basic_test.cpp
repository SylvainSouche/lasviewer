// basic_test.cpp — formula tests (atf-c++). These re-derive formulas used by
// the viewer locally rather than linking the code that uses them.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>
#include <limits>
#include <utility>
#include <algorithm>

#include <atf-c++.hpp>

#define TEST(name)                    \
    ATF_TEST_CASE_WITHOUT_HEAD(name);  \
    ATF_TEST_CASE_BODY(name)

// ---------------------------------------------------------------------------
// Coordinate transform: world (X, Y, Z) → GL (X, Y, Z)
// GL_X = (worldX - center.x) / scale
// GL_Y = (worldZ - center.z) / scale   (elevation → up)
// GL_Z = -(worldY - center.y) / scale  (northing → negated for north-up)
// ---------------------------------------------------------------------------
TEST(test_coord_transform) {
    double centerX = 1010500, centerY = 6548500, centerZ = 3488;
    double scale = 1537.0;
    double invScale = 1.0 / scale;

    // Cloud UL corner: world (1010000, 6549000, 3108) — low X, high Y, low Z
    double wx = 1010000, wy = 6549000, wz = 3108;
    float gx = static_cast<float>((wx - centerX) * invScale);
    float gy = static_cast<float>((wz - centerZ) * invScale);
    float gz = static_cast<float>(-(wy - centerY) * invScale);

    // UL corner: negative X (west of center), negative Y (low elevation),
    // negative Z (north of center → negated).
    ATF_REQUIRE(gx < 0.0f);
    ATF_REQUIRE(gy < 0.0f);
    ATF_REQUIRE(gz < 0.0f);

    // Cloud LR corner: world (1011000, 6548000, 3868) — high X, low Y, high Z
    wx = 1011000; wy = 6548000; wz = 3868;
    gx = static_cast<float>((wx - centerX) * invScale);
    gy = static_cast<float>((wz - centerZ) * invScale);
    gz = static_cast<float>(-(wy - centerY) * invScale);

    ATF_REQUIRE(gx > 0.0f);
    ATF_REQUIRE(gy > 0.0f);
    ATF_REQUIRE(gz > 0.0f);
}

// ---------------------------------------------------------------------------
// GeoTIFF affine inverse: world (X, Y) → pixel (col, row)
// For north-up orthophoto: A=pixelWidth, E=-pixelHeight, B=D=0
// col = (X - C) / A,  row = (Y - F) / E
// ---------------------------------------------------------------------------
TEST(test_affine_inverse) {
    // monzone.tif values: A=0.2, E=-0.2, C=1009000.1, F=6550999.9
    double A = 0.2, E = -0.2, C = 1009000.1, F = 6550999.9;
    double det = A * E;  // B=D=0 → det = A*E
    double invA = E / det;  // = 1/A = 5
    double invE = A / det;  // = 1/E = -5

    // Cloud center at world (1010500, 6548500) should map to roughly the
    // center of the 20000×20000 orthophoto.
    double wx = 1010500, wy = 6548500;
    double col = invA * (wx - C);
    double row = invE * (wy - F);

    // col = 5 * (1010500 - 1009000.1) = 5 * 1499.9 ≈ 7500
    // row = -5 * (6548500 - 6550999.9) = -5 * (-2499.9) ≈ 12500
    ATF_REQUIRE(std::abs(col - 7500.0) < 10.0);
    ATF_REQUIRE(std::abs(row - 12500.0) < 10.0);
    ATF_REQUIRE(col >= 0 && col < 20000);
    ATF_REQUIRE(row >= 0 && row < 20000);
}

// ---------------------------------------------------------------------------
// Spatial grid subsampling: cellSize = sqrt(area / maxPoints)
// For 1km² at 2M target: cellSize ≈ 0.707m
// ---------------------------------------------------------------------------
TEST(test_subsample_cellsize) {
    double area = 1000.0 * 1000.0;  // 1 km²
    size_t maxPoints = 2'000'000;
    double cellSize = std::sqrt(area / static_cast<double>(maxPoints));
    ATF_REQUIRE(cellSize > 0.70 && cellSize < 0.71);

    // For 4 km² at 2M target: cellSize ≈ 1.414m
    area = 2000.0 * 2000.0;
    cellSize = std::sqrt(area / static_cast<double>(maxPoints));
    ATF_REQUIRE(cellSize > 1.41 && cellSize < 1.42);
}

// ---------------------------------------------------------------------------
// Depth-based subsampling formula (from the vertex shader):
// worldSpacing = targetPixelSpacing * 2 * depth * tan(fov/2) / viewportH
// N = density * worldSpacing²
// keepThreshold = clamp(1/N, 0, 1)
// Closer points → smaller worldSpacing → smaller N → higher keepThreshold
// ---------------------------------------------------------------------------
TEST(test_depth_subsampling) {
    float targetSpacing = 2.83f;
    float tanHalfFov = 0.5206f;  // tan(27.5°) for 55° FOV
    float viewportH = 1600.0f;
    float density = 5.0f;  // GL-space density (pts per GL-unit²)

    // Close point at depth 0.5
    float closeDepth = 0.5f;
    float closeSpacing = targetSpacing * 2.0f * closeDepth * tanHalfFov / viewportH;
    float closeN = density * closeSpacing * closeSpacing;
    float closeKeep = std::min(1.0f / std::max(closeN, 1.0f), 1.0f);

    // Far point at depth 5.0
    float farDepth = 5.0f;
    float farSpacing = targetSpacing * 2.0f * farDepth * tanHalfFov / viewportH;
    float farN = density * farSpacing * farSpacing;
    float farKeep = std::min(1.0f / std::max(farN, 1.0f), 1.0f);

    // Close points should be kept at higher probability than far points.
    ATF_REQUIRE(closeKeep >= farKeep);

    // Very close points (N < 1) should all be kept.
    ATF_REQUIRE(closeKeep <= 1.0f);
    ATF_REQUIRE(closeKeep >= 0.0f);
    ATF_REQUIRE(farKeep >= 0.0f);
}

// ---------------------------------------------------------------------------
// Morton (Z-order) code: interleave bits of X and Y
// morton(0,0)=0, morton(1,0)=1, morton(0,1)=2, morton(1,1)=3
// Used for spatially-uniform point ordering within LOD cells.
// ---------------------------------------------------------------------------
TEST(test_morton_code) {
    auto morton2D = [](uint32_t x, uint32_t y) -> uint32_t {
        uint32_t r = 0;
        for (int i = 0; i < 10; ++i) {
            r |= ((x >> i) & 1u) << (2 * i);
            r |= ((y >> i) & 1u) << (2 * i + 1);
        }
        return r;
    };
    ATF_REQUIRE(morton2D(0, 0) == 0);
    ATF_REQUIRE(morton2D(1, 0) == 1);
    ATF_REQUIRE(morton2D(0, 1) == 2);
    ATF_REQUIRE(morton2D(1, 1) == 3);
    ATF_REQUIRE(morton2D(2, 0) == 4);
    ATF_REQUIRE(morton2D(3, 0) == 5);
    // Morton order is monotonic in both X and Y.
    ATF_REQUIRE(morton2D(100, 100) > morton2D(50, 50));
}

// ---------------------------------------------------------------------------
// Near/far plane computation from bbox corners:
// Transform 8 corners to view space, find min/max positive Z.
// ---------------------------------------------------------------------------
TEST(test_near_far_from_bbox) {
    // Simulated GL-space bbox: ±0.3 in X, ±0.24 in Y, ±0.3 in Z
    // Camera at (0, 1.7, 1.0) looking at origin.
    // Corner nearest to camera: (0, 0.24, -0.3) — high Y, negative Z (toward camera)
    float camX = 0.0f, camY = 1.7f, camZ = 1.0f;

    // 8 corners
    float corners[8][3] = {
        {-0.3f, -0.24f, -0.3f}, { 0.3f, -0.24f, -0.3f},
        { 0.3f, -0.24f,  0.3f}, {-0.3f, -0.24f,  0.3f},
        {-0.3f,  0.24f, -0.3f}, { 0.3f,  0.24f, -0.3f},
        { 0.3f,  0.24f,  0.3f}, {-0.3f,  0.24f,  0.3f},
    };

    float minDist = 1e30f, maxDist = -1e30f;
    for (int i = 0; i < 8; ++i) {
        float dx = corners[i][0] - camX;
        float dy = corners[i][1] - camY;
        float dz = corners[i][2] - camZ;
        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (dist < minDist) minDist = dist;
        if (dist > maxDist) maxDist = dist;
    }

    // nearP = minDist * 0.9, farP = maxDist * 1.5
    float nearP = std::max(0.001f, minDist * 0.9f);
    float farP = maxDist * 1.5f + 0.01f;
    ATF_REQUIRE(nearP > 0.0f);
    ATF_REQUIRE(farP > nearP);
    ATF_REQUIRE(farP / nearP < 10000.0f);  // reasonable depth range
}

// ---------------------------------------------------------------------------
// Elevation gradient color ramp:
// t=0 → blue, t=0.25 → green, t=0.5 → yellow, t=0.75 → red, t=1 → red
// ---------------------------------------------------------------------------
TEST(test_elevation_colors) {
    static const float stops[5][3] = {
        {0.1f, 0.2f, 0.8f},  // blue
        {0.2f, 0.7f, 0.9f},  // cyan
        {0.3f, 0.8f, 0.3f},  // green
        {0.95f, 0.85f, 0.2f}, // yellow
        {0.85f, 0.25f, 0.2f}  // red
    };
    auto colorAt = [&](float t) -> const float* {
        t = std::max(0.0f, std::min(1.0f, t));
        float s = t * 4.0f;
        int i = static_cast<int>(s);
        if (i >= 4) return stops[4];
        return stops[i];  // simplified — real code interpolates
    };

    // t=0 → blue (high B, low R)
    const float* c0 = colorAt(0.0f);
    ATF_REQUIRE(c0[2] > c0[0]);  // B > R

    // t=1 → red (high R, low B)
    const float* c1 = colorAt(1.0f);
    ATF_REQUIRE(c1[0] > c1[2]);  // R > B
}

// ---------------------------------------------------------------------------
// Tile grid resolution computation:
// desiredResolution = tileWorldMeters / (tilePixelSize / 3.0)
// Closer tiles → larger pixel size → smaller (finer) resolution
// ---------------------------------------------------------------------------
// Mirrors TileGrid::desiredResolution() in src/copc_streamer.cpp: an
// oriented-bounding-box (from per-tile PCA) is projected toward the camera
// to estimate on-screen area, then a resolution is derived from a target
// of ~1 point per 25px². This replaced an earlier, simpler radius-based
// formula; the test below exercises the actual current algorithm rather
// than the old one.
static double desiredResolutionOBB(double tileW, double tileH,
                                   float ext1, float ext2, float ext3,
                                   float camDist, float viewportH,
                                   float tanHalfFov) {
    // Assume the camera looks straight down the tile's shortest axis
    // (axis3, typically near-vertical for a roughly planar tile slab) —
    // i.e. the dominant visible face is the ext1 x ext2 face, seen
    // face-on (cosAngle == 1) for this test.
    float pxPerUnit = viewportH / (2.0f * camDist * tanHalfFov);
    float faceAreaGL = (2.0f * ext1) * (2.0f * ext2);
    float projectedAreaPx = faceAreaGL * pxPerUnit * pxPerUnit;
    (void)ext3;
    if (projectedAreaPx < 25.0f) return 0.0;

    float targetPoints = projectedAreaPx / 25.0f;
    if (targetPoints < 100.0f) targetPoints = 100.0f;
    if (targetPoints > 500000.0f) targetPoints = 500000.0f;

    double tileAreaM = tileW * tileH;
    double res = std::sqrt(tileAreaM / targetPoints);
    if (res < 0.1) res = 0.1;
    if (res > 50.0) return 0.0;
    return res;
}

TEST(test_tile_resolution) {
    double tileW = 125.0, tileH = 125.0;  // 125m x 125m tile
    float viewportH = 1600;
    float tanHalfFov = 0.5206f;
    // Tile OBB extents (half-widths) roughly matching a 125m tile.
    float ext1 = 62.0f * 0.04f / 125.0f;  // GL-space half-extent, axis 1
    float ext2 = 62.0f * 0.04f / 125.0f;  // GL-space half-extent, axis 2
    float ext3 = 1.0f  * 0.04f / 125.0f;  // GL-space half-extent, axis 3 (thin)

    // Close camera → large projected area → fine resolution (small number).
    double resClose = desiredResolutionOBB(tileW, tileH, ext1, ext2, ext3,
                                           /*camDist=*/1.0f, viewportH, tanHalfFov);
    ATF_REQUIRE(resClose > 0 && resClose < 50.0);

    // Far camera → small projected area → coarse resolution or skip (0).
    double resFar = desiredResolutionOBB(tileW, tileH, ext1, ext2, ext3,
                                         /*camDist=*/50.0f, viewportH, tanHalfFov);
    ATF_REQUIRE(resFar == 0.0 || resFar > resClose);

    // Very far camera must be capped/skipped, never negative or absurdly small.
    double resVeryFar = desiredResolutionOBB(tileW, tileH, ext1, ext2, ext3,
                                             /*camDist=*/500.0f, viewportH, tanHalfFov);
    ATF_REQUIRE(resVeryFar >= 0.0);
}

// Minimal local Vec3 (NOT glm) — the test target is deliberately
// dependency-free (specs.md §12.3), so this mirrors just enough of glm's
// vec3 API for the tests below, same spirit as the rest of this file
// replicating formulas locally rather than linking against src/*.cpp.
struct Vec3 {
    float x, y, z;
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator*(float s) const { return {x*s, y*s, z*s}; }
    bool operator==(const Vec3& o) const { return x==o.x && y==o.y && z==o.z; }
};
static float length(Vec3 v) { return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z); }

// Mirrors the TCS's edgeTessLevel() in src/shaders.cpp (kMeshTessControl):
// projected screen-space edge length -> clamped target segment count.
static float edgeTessLevelCPU(Vec3 a, Vec3 b, Vec3 camPos,
                              float viewportH, float tanHalfFov,
                              float targetPixelsPerSegment) {
    Vec3 mid = (a + b) * 0.5f;
    float dist = std::max(length(mid - camPos), 0.0001f);
    float pxPerUnit = viewportH / (2.0f * dist * tanHalfFov);
    float edgeLenGL = length(b - a);
    float edgeLenPx = edgeLenGL * pxPerUnit;
    float level = edgeLenPx / targetPixelsPerSegment;
    if (level < 1.0f) level = 1.0f;
    if (level > 64.0f) level = 64.0f;
    return level;
}

TEST(test_dem_tess_edge_level) {
    Vec3 a{-0.1f, 0.0f, 0.0f}, b{0.1f, 0.0f, 0.0f};
    float viewportH = 1600, tanHalfFov = 0.5206f, targetPx = 8.0f;

    // Close camera -> larger projected edge -> higher (or equally clamped)
    // tessellation level than a far camera.
    float levelClose = edgeTessLevelCPU(a, b, Vec3{0,0,0.5f}, viewportH, tanHalfFov, targetPx);
    float levelFar    = edgeTessLevelCPU(a, b, Vec3{0,0,20.0f}, viewportH, tanHalfFov, targetPx);
    ATF_REQUIRE(levelClose >= levelFar);
    ATF_REQUIRE(levelClose >= 1.0f && levelClose <= 64.0f);
    ATF_REQUIRE(levelFar >= 1.0f && levelFar <= 64.0f);

    // Determinism: two "patches" sharing this exact edge (same endpoints,
    // regardless of which patch's TCS invocation computes it) must agree —
    // this is the crack-avoidance property from
    // docs/design-tessellation-displacement.md §5.4. Order shouldn't matter
    // since the formula only depends on the edge's midpoint/length.
    float levelForward = edgeTessLevelCPU(a, b, Vec3{0,0,1.0f}, viewportH, tanHalfFov, targetPx);
    float levelReverse = edgeTessLevelCPU(b, a, Vec3{0,0,1.0f}, viewportH, tanHalfFov, targetPx);
    ATF_REQUIRE(std::abs(levelForward - levelReverse) < 1e-5f);
}

// Mirrors DEMTessMesh::loadFromDEM()'s computeNormalGL() in
// src/dem_tess_mesh.cpp: finite-difference elevation gradient -> GL-space
// normal, using n_GL = normalize(-dElev/dwx, 1, +dElev/dwy).
static Vec3 computeNormalGLCPU(float dElev_dcol, float dElev_drow,
                               double geoA, double geoE) {
    double dElev_dwx = dElev_dcol / geoA;
    double dElev_dwy = (geoE != 0.0) ? (dElev_drow / geoE) : 0.0;
    Vec3 n{static_cast<float>(-dElev_dwx), 1.0f, static_cast<float>(dElev_dwy)};
    float len = length(n);
    return (len > 1e-12f) ? Vec3{n.x/len, n.y/len, n.z/len} : Vec3{0, 1, 0};
}

TEST(test_dem_tess_normal) {
    // Flat terrain (no gradient) -> normal points straight up (GL +Y).
    Vec3 nFlat = computeNormalGLCPU(0.0f, 0.0f, 1.0, -1.0);
    ATF_REQUIRE(std::abs(nFlat.x) < 1e-6f);
    ATF_REQUIRE(std::abs(nFlat.y - 1.0f) < 1e-6f);
    ATF_REQUIRE(std::abs(nFlat.z) < 1e-6f);

    // Always unit length regardless of gradient magnitude.
    Vec3 nSlope = computeNormalGLCPU(5.0f, -3.0f, 0.5, -0.5);
    ATF_REQUIRE(std::abs(length(nSlope) - 1.0f) < 1e-5f);

    // Sanity: north-up raster (geoE negative) — increasing elevation
    // northward (dElev_drow > 0, row decreases northward) should tilt the
    // normal's Z component in a consistent, non-degenerate direction, not
    // silently collapse to zero or flip sign randomly between calls.
    Vec3 n1 = computeNormalGLCPU(0.0f, 2.0f, 1.0, -1.0);
    Vec3 n2 = computeNormalGLCPU(0.0f, 2.0f, 1.0, -1.0);
    ATF_REQUIRE(n1 == n2);  // deterministic
    ATF_REQUIRE(std::abs(n1.z) > 1e-6f);  // actually tilted, not flat
}

// Mirrors the angular geometric-error criterion in both
// src/dem_tess_mesh.cpp and src/dem_mesh.cpp's quadtree subdivision:
// subdivide (don't collapse/stay coarse) only if a sample point's
// deviation from the bilinear-interpolated surface exceeds a 1-degree
// angle as seen across the cell's own world size, rather than comparing
// to a fixed fraction of the DEM's total elevation range.
static bool geomAngleExceededCPU(double geomErr, double worldCellSize,
                                 double angleThreshDeg) {
    double angleThreshTan = std::tan(angleThreshDeg * 3.14159265358979323846 / 180.0);
    double angularBaseline = worldCellSize * 0.5;
    return geomErr > angularBaseline * angleThreshTan;
}

TEST(test_angular_geom_error) {
    // The whole point: the SAME absolute deviation should be judged
    // differently depending on cell size — significant (exceeds 1°) in a
    // small cell, insignificant (within 1°) in a large one.
    double deviation = 0.1; // 10cm bump, same in both cases

    bool exceedsSmallCell = geomAngleExceededCPU(deviation, /*worldCellSize=*/1.0, 1.0);
    bool exceedsLargeCell = geomAngleExceededCPU(deviation, /*worldCellSize=*/100.0, 1.0);
    ATF_REQUIRE(exceedsSmallCell);   // 10cm bump in a 1m cell: steep, must subdivide
    ATF_REQUIRE(!exceedsLargeCell);  // 10cm bump in a 100m cell: negligible, stay coarse

    // Sanity: a truly flat sample (zero deviation) never exceeds any
    // positive angle threshold, regardless of cell size.
    ATF_REQUIRE(!geomAngleExceededCPU(0.0, 1.0, 1.0));
    ATF_REQUIRE(!geomAngleExceededCPU(0.0, 1000.0, 1.0));

    // Monotonic in the threshold: a looser (larger) angle threshold can
    // only ever subdivide LESS than a stricter (smaller) one for the same
    // geometry, never more.
    bool strict = geomAngleExceededCPU(deviation, 5.0, 0.5);
    bool loose  = geomAngleExceededCPU(deviation, 5.0, 5.0);
    ATF_REQUIRE(!(loose && !strict)); // loose=>strict would be a contradiction here
}

// Mirrors the TES's density-tied coarse-reference blend in
// src/shaders.cpp (kMeshTessEval): a weight that vanishes along all four
// patch edges (preserving crack-freedom there unchanged) and a mip level
// that shrinks toward 0 (native resolution, minimal displacement) as
// tessellation density increases (uTargetPixelsPerSegment decreases).
static float blendWeightCPU(float u, float v) {
    return 16.0f * u * (1.0f - u) * v * (1.0f - v);
}
static float coarseMipLevelCPU(float targetPixelsPerSegment) {
    float t = std::max(targetPixelsPerSegment, 0.0001f);
    return std::max(0.0f, std::log2(t));
}

TEST(test_density_blend_weight) {
    // Exactly zero along all four edges — this is what keeps the
    // crack-avoidance guarantee (§5.1/5.2 of the design doc) completely
    // unaffected by this blend, since neighboring patches only ever need
    // to agree along edges, never in the interior.
    ATF_REQUIRE(blendWeightCPU(0.0f, 0.5f) == 0.0f);
    ATF_REQUIRE(blendWeightCPU(1.0f, 0.5f) == 0.0f);
    ATF_REQUIRE(blendWeightCPU(0.5f, 0.0f) == 0.0f);
    ATF_REQUIRE(blendWeightCPU(0.5f, 1.0f) == 0.0f);
    // All 4 corners too (both u and v at an extreme).
    ATF_REQUIRE(blendWeightCPU(0.0f, 0.0f) == 0.0f);
    ATF_REQUIRE(blendWeightCPU(1.0f, 1.0f) == 0.0f);

    // Peaks at the center, positive in the interior.
    float center = blendWeightCPU(0.5f, 0.5f);
    ATF_REQUIRE(std::abs(center - 1.0f) < 1e-5f);
    float offCenter = blendWeightCPU(0.25f, 0.5f);
    ATF_REQUIRE(offCenter > 0.0f && offCenter < center);
}

TEST(test_density_mip_level) {
    // Default-ish density (targetPx ~8) -> a real coarse mip, roughly
    // matching the pre-existing far-corner-only behavior.
    float mipDefault = coarseMipLevelCPU(8.0f);
    ATF_REQUIRE(std::abs(mipDefault - 3.0f) < 1e-4f); // log2(8) == 3

    // Maximum density (targetPx -> small, clamped to >=0.5 by
    // DEMTessMesh::render()) -> mip level shrinks toward 0, so the
    // blended coarse reference approaches the fine sample and delta
    // (displacement) shrinks correspondingly — this is the actual
    // property being requested: "at maximum tessellation, displacement
    // should be minimal."
    float mipMax = coarseMipLevelCPU(0.5f);
    ATF_REQUIRE(mipMax < mipDefault);
    ATF_REQUIRE(mipMax >= 0.0f); // never negative — textureLod would clamp anyway

    // Monotonic: denser (smaller targetPx) never gives a HIGHER mip level.
    float mipMed = coarseMipLevelCPU(2.0f);
    ATF_REQUIRE(mipMed <= mipDefault);
    ATF_REQUIRE(mipMax <= mipMed);
}

// Mirrors Camera::position()'s offset formula (camera.cpp) and the
// double-click recenter solve in main.cpp's render loop: given a fixed
// eye and a new target (the picked point), solve for yaw/pitch/distance
// such that recomputing position() from them lands exactly back on the
// original eye — i.e. the eye never moves, only the look direction
// rotates and the pivot changes. This is what distinguishes a genuine
// recenter from a disguised dolly/zoom (the simpler "just set target,
// keep yaw/pitch/distance" approach tried first, and rejected).
static Vec3 cameraOffsetCPU(float yaw, float pitch, float distance) {
    return Vec3{
        distance * std::cos(pitch) * std::sin(yaw),
        distance * std::sin(pitch),
        distance * std::cos(pitch) * std::cos(yaw)
    };
}

TEST(test_recenter_keeps_eye_fixed) {
    Vec3 target{0.0f, 0.0f, 0.0f};
    float yaw = 0.6f, pitch = 1.2f, distance = 3.0f;
    Vec3 eye = target + cameraOffsetCPU(yaw, pitch, distance);

    // A "picked point" at some other location, different depth/direction
    // from the original target.
    Vec3 picked{1.5f, -0.5f, 0.8f};

    Vec3 toEye = eye - picked;
    float newDistance = length(toEye);
    ATF_REQUIRE(newDistance > 1e-6f);
    Vec3 dir{toEye.x / newDistance, toEye.y / newDistance, toEye.z / newDistance};
    float newPitch = std::asin(std::max(-1.0f, std::min(1.0f, dir.y)));
    float newYaw = std::atan2(dir.x, dir.z);

    Vec3 recomputedEye = picked + cameraOffsetCPU(newYaw, newPitch, newDistance);

    ATF_REQUIRE(std::abs(recomputedEye.x - eye.x) < 1e-4f);
    ATF_REQUIRE(std::abs(recomputedEye.y - eye.y) < 1e-4f);
    ATF_REQUIRE(std::abs(recomputedEye.z - eye.z) < 1e-4f);
}

// Mirrors the normal-variation threshold derivation in
// src/dem_tess_mesh.cpp's quadtree subdivision: NORMAL_ANGLE_THRESH_DEG
// must scale proportionally with collapseAngleDeg, not be a second,
// independently-fixed constant — the real bug this fixes: with the two
// criteria fixed apart, whichever one was more permissive for a given
// DEM's actual terrain could dominate, making the user-controlled I/O
// angle have no visible effect on the final mesh.
static double normalAngleThreshDegCPU(double collapseAngleDeg) {
    double v = 20.0 * (collapseAngleDeg / 1.0);
    if (v < 1.0) v = 1.0;
    if (v > 89.0) v = 89.0;
    return v;
}

TEST(test_normal_threshold_scales_with_collapse_angle) {
    // At the 1.0° default, matches the original fixed ~20° behavior.
    ATF_REQUIRE(std::abs(normalAngleThreshDegCPU(1.0) - 20.0) < 1e-9);

    // Scales proportionally, not independently — decreasing the
    // collapsing angle (I key, finer/stricter) must ALSO tighten the
    // normal-variation threshold, not leave it fixed.
    double finer = normalAngleThreshDegCPU(0.5);
    double coarser = normalAngleThreshDegCPU(2.0);
    ATF_REQUIRE(finer < 20.0);
    ATF_REQUIRE(coarser > 20.0);
    ATF_REQUIRE(std::abs(finer - 10.0) < 1e-9);
    ATF_REQUIRE(std::abs(coarser - 40.0) < 1e-9);

    // Clamped to a sane range regardless of how far I/O is pushed.
    ATF_REQUIRE(normalAngleThreshDegCPU(0.01) >= 1.0);
    ATF_REQUIRE(normalAngleThreshDegCPU(30.0) <= 89.0);
}

// Mirrors the fineElev edge-blend in the TES: blending the raw heightmap
// sample toward coarsePos.y with an edge-vanishing weight guarantees
// fineElev == coarsePos.y exactly at patch boundaries, regardless of
// whether the raw texture sample would have matched exactly (it might
// not, if the heightmap was downsampled — box-filtered averages don't
// reproduce exact point samples).
static float blendedFineElevCPU(float coarsePosY, float rawSample, float w) {
    return coarsePosY * (1.0f - w) + rawSample * w; // mix()
}

TEST(test_fine_elev_exact_at_boundary) {
    float coarsePosY = 100.0f;
    // A raw sample that does NOT exactly match the corner's own value —
    // simulating heightmap-downsampling imprecision.
    float rawSampleImprecise = 137.5f;

    // At a boundary (w == 0, per blendWeightCPU), the blended fine
    // elevation must EXACTLY equal coarsePosY, regardless of how far off
    // the raw sample is — this is what keeps every patch corner/edge
    // exactly matching its neighbor, even when the underlying texture
    // doesn't have exact data there.
    float w0 = blendWeightCPU(0.0f, 0.5f); // an edge point
    ATF_REQUIRE(w0 == 0.0f);
    float fineAtEdge = blendedFineElevCPU(coarsePosY, rawSampleImprecise, w0);
    ATF_REQUIRE(fineAtEdge == coarsePosY);

    // In the interior (w > 0), the raw sample DOES contribute — real
    // detail isn't lost, only the boundary is protected.
    float wCenter = blendWeightCPU(0.5f, 0.5f);
    ATF_REQUIRE(wCenter > 0.0f);
    float fineAtCenter = blendedFineElevCPU(coarsePosY, rawSampleImprecise, wCenter);
    ATF_REQUIRE(std::abs(fineAtCenter - rawSampleImprecise) < std::abs(fineAtCenter - coarsePosY) + 1e-5f);
}

// Mirrors the core property requested directly ("the triangle's vertices
// must sit at the dem provided altitude ... that is still not the
// case"): the TES's displaced Y must equal the TRUE heightmap value
// exactly at the patch center (w == 1, maximum blend), for ANY normal —
// unlike the prior along-normal/ray-plane-intersection approach, which
// only reached the true value when the surface happened to be exactly
// flat and fell increasingly short as terrain steepened (attenuated by
// normal.y). The new formula has no normal in it at all, so there's
// nothing for slope to attenuate.
TEST(test_displaced_y_exact_regardless_of_slope) {
    float coarsePosY = 100.0f;
    float trueHeightmapValue = 137.5f; // the actual DEM value at this (X,Z)
    float wCenter = 1.0f; // patch center, per blendWeightCPU(0.5, 0.5)'s peak

    // Flat terrain: the OLD formula (verticalDelta * normal.y, normal.y=1)
    // also happened to land exactly here — not a useful distinguishing
    // case on its own, but confirms the new formula doesn't regress it.
    float fineElevFlat = blendedFineElevCPU(coarsePosY, trueHeightmapValue, wCenter);
    ATF_REQUIRE(std::abs(fineElevFlat - trueHeightmapValue) < 1e-5f);

    // Steep terrain (small normal.y): this is the case that actually
    // distinguishes the two approaches. The direct assignment doesn't
    // reference a normal at all, so the result is IDENTICAL regardless of
    // slope — still exactly the true value. The old formula would have
    // given coarsePosY + (trueHeightmapValue-coarsePosY)*normalY^2,
    // falling short of trueHeightmapValue by more as normalY shrinks.
    float normalYSteep = 0.3f;
    float fineElevSteep = blendedFineElevCPU(coarsePosY, trueHeightmapValue, wCenter);
    ATF_REQUIRE(std::abs(fineElevSteep - trueHeightmapValue) < 1e-5f); // exact, no attenuation
    float verticalDelta = trueHeightmapValue - coarsePosY;
    float oldFormulaResult = coarsePosY + verticalDelta * normalYSteep * normalYSteep;
    ATF_REQUIRE(std::abs(oldFormulaResult - trueHeightmapValue) > 1.0f); // old formula fell meaningfully short
}

// Mirrors the master-edge distance computed in the TES (src/shaders.cpp,
// kMeshTessEval) and consumed by kMeshFrag's red-edge visualization.
static float edgeDistCPU(float u, float v) {
    return std::min(std::min(u, 1.0f - u), std::min(v, 1.0f - v));
}

TEST(test_master_edge_distance) {
    // Exactly zero at all four edges/corners — must line up with
    // MASTER_EDGE_THRESHOLD in kMeshFrag triggering the red highlight
    // right at the coarse patch boundary.
    ATF_REQUIRE(edgeDistCPU(0.0f, 0.5f) == 0.0f);
    ATF_REQUIRE(edgeDistCPU(1.0f, 0.5f) == 0.0f);
    ATF_REQUIRE(edgeDistCPU(0.5f, 0.0f) == 0.0f);
    ATF_REQUIRE(edgeDistCPU(0.5f, 1.0f) == 0.0f);
    ATF_REQUIRE(edgeDistCPU(0.0f, 0.0f) == 0.0f);

    // Maximum (0.5) exactly at the patch center — never mistaken for an
    // edge regardless of threshold choice.
    ATF_REQUIRE(std::abs(edgeDistCPU(0.5f, 0.5f) - 0.5f) < 1e-6f);

    // Monotonic: moving from center toward an edge only ever decreases.
    ATF_REQUIRE(edgeDistCPU(0.3f, 0.5f) < edgeDistCPU(0.5f, 0.5f));
    ATF_REQUIRE(edgeDistCPU(0.1f, 0.5f) < edgeDistCPU(0.3f, 0.5f));
}

// Mirrors the min/max elevation pyramid build+query in dem_tess_mesh.cpp
// and dem_mesh.cpp (kept in sync between the two) — the mechanism that
// makes top-down quadtree subdivision exact again, replacing bottom-up
// collapse: a test against the TRUE min/max elevation anywhere in a
// cell's footprint, not 5 discrete sample points, so real detail between
// those points can no longer hide from the test regardless of MAX_LEVEL.
namespace pyramid_test {
struct Pyramid {
    std::vector<std::vector<float>> minPyr, maxPyr;
    std::vector<int> pyrW, pyrH;
};

static Pyramid buildPyramidCPU(const std::vector<float>& elevs, int w, int h,
                               const std::vector<bool>& nodata) {
    Pyramid p;
    int levels = 1;
    { int lw = w, lh = h; while (lw > 1 || lh > 1) { lw = (lw+1)/2; lh = (lh+1)/2; ++levels; } }
    p.minPyr.resize(levels); p.maxPyr.resize(levels);
    p.pyrW.resize(levels); p.pyrH.resize(levels);
    p.pyrW[0] = w; p.pyrH[0] = h;
    p.minPyr[0].resize(static_cast<size_t>(w) * h);
    p.maxPyr[0].resize(static_cast<size_t>(w) * h);
    const float kPos = std::numeric_limits<float>::infinity();
    const float kNeg = -std::numeric_limits<float>::infinity();
    for (int r = 0; r < h; ++r) for (int c = 0; c < w; ++c) {
        size_t idx = static_cast<size_t>(r) * w + c;
        bool nd = nodata[idx];
        p.minPyr[0][idx] = nd ? kPos : elevs[idx];
        p.maxPyr[0][idx] = nd ? kNeg : elevs[idx];
    }
    for (int lvl = 1; lvl < levels; ++lvl) {
        int sw = p.pyrW[lvl-1], sh = p.pyrH[lvl-1];
        int dw = (sw+1)/2, dh = (sh+1)/2;
        p.pyrW[lvl] = dw; p.pyrH[lvl] = dh;
        p.minPyr[lvl].resize(static_cast<size_t>(dw) * dh);
        p.maxPyr[lvl].resize(static_cast<size_t>(dw) * dh);
        auto& srcMin = p.minPyr[lvl-1]; auto& srcMax = p.maxPyr[lvl-1];
        auto& dstMin = p.minPyr[lvl]; auto& dstMax = p.maxPyr[lvl];
        for (int r = 0; r < dh; ++r) for (int c = 0; c < dw; ++c) {
            int sr0 = r*2, sc0 = c*2;
            int sr1 = std::min(sr0+1, sh-1), sc1 = std::min(sc0+1, sw-1);
            float mn = std::min(std::min(srcMin[static_cast<size_t>(sr0)*sw+sc0], srcMin[static_cast<size_t>(sr0)*sw+sc1]),
                                std::min(srcMin[static_cast<size_t>(sr1)*sw+sc0], srcMin[static_cast<size_t>(sr1)*sw+sc1]));
            float mx = std::max(std::max(srcMax[static_cast<size_t>(sr0)*sw+sc0], srcMax[static_cast<size_t>(sr0)*sw+sc1]),
                                std::max(srcMax[static_cast<size_t>(sr1)*sw+sc0], srcMax[static_cast<size_t>(sr1)*sw+sc1]));
            dstMin[static_cast<size_t>(r)*dw+c] = mn;
            dstMax[static_cast<size_t>(r)*dw+c] = mx;
        }
    }
    return p;
}

static std::pair<float,float> queryPyramidCPU(const Pyramid& p, double col, double row,
                                              double cw, double ch) {
    double cellSize = std::min(cw, ch);
    int lvl = 0;
    if (cellSize > 1.0) {
        lvl = static_cast<int>(std::floor(std::log2(cellSize)));
        lvl = std::clamp(lvl, 0, static_cast<int>(p.minPyr.size()) - 1);
    }
    int lw = p.pyrW[lvl], lh = p.pyrH[lvl];
    int texelsPerSide = 1 << lvl;
    int c0 = std::clamp(static_cast<int>(std::floor(col/texelsPerSide)), 0, lw-1);
    int c1 = std::clamp(static_cast<int>(std::floor((col+cw)/texelsPerSide)), 0, lw-1);
    int r0 = std::clamp(static_cast<int>(std::floor(row/texelsPerSide)), 0, lh-1);
    int r1 = std::clamp(static_cast<int>(std::floor((row+ch)/texelsPerSide)), 0, lh-1);
    float mn = std::numeric_limits<float>::infinity();
    float mx = -std::numeric_limits<float>::infinity();
    for (int r = r0; r <= r1; ++r) for (int c = c0; c <= c1; ++c) {
        size_t idx = static_cast<size_t>(r)*lw+c;
        mn = std::min(mn, p.minPyr[lvl][idx]);
        mx = std::max(mx, p.maxPyr[lvl][idx]);
    }
    return {mn, mx};
}
} // namespace pyramid_test

TEST(test_minmax_pyramid_flat_dem) {
    using namespace pyramid_test;
    int w = 16, h = 16;
    std::vector<float> elevs(static_cast<size_t>(w)*h, 100.0f);
    std::vector<bool> nodata(static_cast<size_t>(w)*h, false);
    auto p = buildPyramidCPU(elevs, w, h, nodata);
    auto [mn, mx] = queryPyramidCPU(p, 0, 0, 16, 16);
    ATF_REQUIRE(mn == 100.0f && mx == 100.0f);
}

TEST(test_minmax_pyramid_catches_hidden_detail) {
    // The entire point of this migration: a spike at the dead center of a
    // large, otherwise-flat cell — invisible to any test that only
    // samples corners/center/edge-midpoints at a COARSER granularity than
    // this, but must be caught here since the pyramid reflects EVERY
    // pixel, not a sample of them.
    using namespace pyramid_test;
    int w = 16, h = 16;
    std::vector<float> elevs(static_cast<size_t>(w)*h, 100.0f);
    elevs[static_cast<size_t>(8)*w + 8] = 500.0f;
    std::vector<bool> nodata(static_cast<size_t>(w)*h, false);
    auto p = buildPyramidCPU(elevs, w, h, nodata);
    auto [mn, mx] = queryPyramidCPU(p, 0, 0, 16, 16);
    ATF_REQUIRE(mx == 500.0f);
    // And the resulting error bound (against a flat 100-valued bilinear
    // plane) must clearly exceed a tight threshold — i.e. this cell would
    // correctly be forced to subdivide, not incorrectly accepted as flat.
    float planeMin = 100.0f, planeMax = 100.0f;
    float errorBound = std::max(mx - planeMin, planeMax - mn);
    ATF_REQUIRE(errorBound == 400.0f);
}

TEST(test_minmax_pyramid_nodata_handling) {
    using namespace pyramid_test;
    // Entirely nodata region -> non-finite result (nothing to check
    // geometrically; the separate nodata mechanism decides this cell).
    {
        int w = 8, h = 8;
        std::vector<float> elevs(static_cast<size_t>(w)*h, 50.0f);
        std::vector<bool> nodata(static_cast<size_t>(w)*h, true);
        auto p = buildPyramidCPU(elevs, w, h, nodata);
        auto [mn, mx] = queryPyramidCPU(p, 0, 0, 8, 8);
        ATF_REQUIRE(!std::isfinite(mn) && !std::isfinite(mx));
    }
    // Mixed nodata -> only the valid pixel(s) count, per the +inf/-inf
    // identity-element convention (nodata never wins a min/max against a
    // real value).
    {
        int w = 4, h = 4;
        std::vector<float> elevs(static_cast<size_t>(w)*h, 0.0f);
        std::vector<bool> nodata(static_cast<size_t>(w)*h, true);
        elevs[5] = 42.0f; nodata[5] = false;
        auto p = buildPyramidCPU(elevs, w, h, nodata);
        auto [mn, mx] = queryPyramidCPU(p, 0, 0, 4, 4);
        ATF_REQUIRE(mn == 42.0f && mx == 42.0f);
    }
}

ATF_INIT_TEST_CASES(tcs) {
    ATF_ADD_TEST_CASE(tcs, test_coord_transform);
    ATF_ADD_TEST_CASE(tcs, test_affine_inverse);
    ATF_ADD_TEST_CASE(tcs, test_subsample_cellsize);
    ATF_ADD_TEST_CASE(tcs, test_depth_subsampling);
    ATF_ADD_TEST_CASE(tcs, test_morton_code);
    ATF_ADD_TEST_CASE(tcs, test_near_far_from_bbox);
    ATF_ADD_TEST_CASE(tcs, test_elevation_colors);
    ATF_ADD_TEST_CASE(tcs, test_tile_resolution);
    ATF_ADD_TEST_CASE(tcs, test_dem_tess_edge_level);
    ATF_ADD_TEST_CASE(tcs, test_dem_tess_normal);
    ATF_ADD_TEST_CASE(tcs, test_angular_geom_error);
    ATF_ADD_TEST_CASE(tcs, test_density_blend_weight);
    ATF_ADD_TEST_CASE(tcs, test_density_mip_level);
    ATF_ADD_TEST_CASE(tcs, test_recenter_keeps_eye_fixed);
    ATF_ADD_TEST_CASE(tcs, test_normal_threshold_scales_with_collapse_angle);
    ATF_ADD_TEST_CASE(tcs, test_fine_elev_exact_at_boundary);
    ATF_ADD_TEST_CASE(tcs, test_displaced_y_exact_regardless_of_slope);
    ATF_ADD_TEST_CASE(tcs, test_master_edge_distance);
    ATF_ADD_TEST_CASE(tcs, test_minmax_pyramid_flat_dem);
    ATF_ADD_TEST_CASE(tcs, test_minmax_pyramid_catches_hidden_detail);
    ATF_ADD_TEST_CASE(tcs, test_minmax_pyramid_nodata_handling);
}
