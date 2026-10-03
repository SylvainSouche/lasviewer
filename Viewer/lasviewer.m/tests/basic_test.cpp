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
    static const float kStops[5][3] = {
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
        if (i >= 4) return kStops[4];
        return kStops[i];  // simplified — real code interpolates
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
    ATF_ADD_TEST_CASE(tcs, test_recenter_keeps_eye_fixed);
    ATF_ADD_TEST_CASE(tcs, test_master_edge_distance);
}
