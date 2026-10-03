// copc_test.cpp — tests of COPC streaming (copc_streamer.cpp) on a small COPC
// file written here with copc-lib: tile layout, what a tile load returns at
// each resolution (octree depth), tile borders, and the level of detail.
#include "copc_streamer.h"

#include <atf-c++.hpp>
#include <copc-lib/hierarchy/key.hpp>
#include <copc-lib/io/copc_writer.hpp>
#include <copc-lib/las/header.hpp>
#include <copc-lib/las/points.hpp>

#include <cmath>
#include <set>
#include <tuple>
#include <vector>

#define CHECK(cond) ATF_REQUIRE(cond)

namespace {

// An 80 m × 80 m COPC file (z 0..80), root spacing 10 m:
//   depth 0: 81 points on a 10 m lattice, z = 1, including every tile
//            border of an 8×8 grid (multiples of 10 m) and the max edges;
//   depth 1: 256 points on a 5 m lattice offset by 2.5 m, z = 2;
//   depth 2: 1024 points on a 2.5 m lattice offset by 1.25 m, z = 3.
// Each point is stored in the node of its depth containing it.
const double kSide = 80.0;
const int kDepth0 = 81, kDepth1 = 256, kDepth2 = 1024;

std::string writeCopc(const std::string& path) {
    copc::CopcConfigWriter cfg(6, {0.01, 0.01, 0.01}, {0.0, 0.0, 0.0}, "", {}, false);
    cfg.LasHeader()->min = copc::Vector3(0.0, 0.0, 0.0);
    cfg.LasHeader()->max = copc::Vector3(kSide, kSide, kSide);
    cfg.CopcInfo()->spacing = 10.0;
    copc::FileWriter writer(path, cfg);
    auto header = *writer.CopcConfig()->LasHeader();

    auto addDepth = [&](int depth, double step, double offset, double z) {
        int n = static_cast<int>(std::round((kSide - 2 * offset) / step)) + 1;
        int keysPerAxis = 1 << depth;
        double voxel = kSide / keysPerAxis;
        std::vector<copc::las::Points> nodes(static_cast<size_t>(keysPerAxis) * keysPerAxis,
                                             copc::las::Points(header));
        for (int j = 0; j < n; ++j)
            for (int i = 0; i < n; ++i) {
                double x = offset + i * step, y = offset + j * step;
                int kx = std::min(static_cast<int>(x / voxel), keysPerAxis - 1);
                int ky = std::min(static_cast<int>(y / voxel), keysPerAxis - 1);
                auto p = nodes[static_cast<size_t>(ky) * keysPerAxis + kx].CreatePoint();
                p->X(x);
                p->Y(y);
                p->Z(z);
                nodes[static_cast<size_t>(ky) * keysPerAxis + kx].AddPoint(p);
            }
        for (int ky = 0; ky < keysPerAxis; ++ky)
            for (int kx = 0; kx < keysPerAxis; ++kx) {
                auto& pts = nodes[static_cast<size_t>(ky) * keysPerAxis + kx];
                if (pts.Size() > 0) writer.AddNode(copc::VoxelKey(depth, kx, ky, 0), pts);
            }
    };
    addDepth(0, 10.0, 0.0, 1.0);
    addDepth(1, 5.0, 2.5, 2.0);
    addDepth(2, 2.5, 1.25, 3.0);
    writer.Close();
    return path;
}

TileGrid* makeGrid(const std::string& path) {
    WorldBounds b;
    b.extendXY(0.0, 0.0, kSide, kSide);
    b.extendZ(0.0, kSide);
    auto* grid = new TileGrid();
    grid->layoutTiles(path, b, nullptr, SceneFrame::fromBounds(b));
    return grid;
}

TileGrid::LoadRequest requestFor(const TileGrid& g, int index, double resolution) {
    const Tile& t = g.tiles[index];
    return {index, resolution, t.minX, t.minY, t.minZ, t.maxX, t.maxY, t.maxZ};
}

// Points of all tiles at a resolution, as rounded world (x, y, z).
std::multiset<std::tuple<long, long, long>> loadAll(TileGrid& g, double resolution) {
    std::multiset<std::tuple<long, long, long>> pts;
    for (size_t i = 0; i < g.tiles.size(); ++i) {
        TileGrid::LoadResult r = g.loadTile(requestFor(g, static_cast<int>(i), resolution));
        CHECK(r.ok && r.positions.size() == r.colors.size() && r.orthoColors.empty());
        for (size_t k = 0; k < r.positions.size(); k += 3) {
            glm::dvec3 w =
                g.frame.toWorld(glm::vec3(r.positions[k], r.positions[k + 1], r.positions[k + 2]));
            pts.insert({std::lround(w.x * 100), std::lround(w.y * 100), std::lround(w.z * 100)});
        }
    }
    return pts;
}

} // namespace

ATF_TEST_CASE_WITHOUT_HEAD(test_tile_layout);
ATF_TEST_CASE_BODY(test_tile_layout) {
    std::unique_ptr<TileGrid> g(makeGrid(writeCopc("layout.copc.laz")));
    CHECK(g->tiles.size() == 64);
    CHECK(g->tileW == 10.0 && g->tileH == 10.0);
    const Tile& last = g->tiles.back();
    CHECK(last.gx == 7 && last.gy == 7 && last.maxX == kSide && last.maxY == kSide);
    for (size_t i = 1; i < g->tiles.size(); ++i)
        if (g->tiles[i].gx > 0) CHECK(g->tiles[i].minX == g->tiles[i - 1].maxX); // contiguous
}

// Each resolution loads the depths whose spacing is at most that resolution
// (root spacing 10 m, halved per depth), and every point exactly once across
// the tiles: borders are half-open, the last row and column take the max
// edge.
ATF_TEST_CASE_WITHOUT_HEAD(test_load_by_resolution_each_point_once);
ATF_TEST_CASE_BODY(test_load_by_resolution_each_point_once) {
    std::unique_ptr<TileGrid> g(makeGrid(writeCopc("res.copc.laz")));
    struct Case {
        double resolution;
        size_t expected;
    };
    for (Case c :
         {Case{10.0, kDepth0}, Case{20.0, kDepth0}, Case{5.0, kDepth0 + kDepth1},
          Case{2.5, kDepth0 + kDepth1 + kDepth2}, Case{0.0, kDepth0 + kDepth1 + kDepth2}}) {
        auto pts = loadAll(*g, c.resolution);
        CHECK(pts.size() == c.expected);
        std::set<std::tuple<long, long, long>> unique(pts.begin(), pts.end());
        CHECK(unique.size() == pts.size()); // no point in two tiles
    }
    // The border points: (10, 10) belongs to tile (1,1) only; (80, 80) to the
    // last tile.
    auto inTile = [&](int index, double x, double y) {
        TileGrid::LoadResult r = g->loadTile(requestFor(*g, index, 10.0));
        for (size_t k = 0; k < r.positions.size(); k += 3) {
            glm::dvec3 w =
                g->frame.toWorld(glm::vec3(r.positions[k], r.positions[k + 1], r.positions[k + 2]));
            if (std::abs(w.x - x) < 1e-6 && std::abs(w.y - y) < 1e-6) return true;
        }
        return false;
    };
    CHECK(inTile(1 * 8 + 1, 10.0, 10.0));
    CHECK(!inTile(0, 10.0, 10.0));
    CHECK(inTile(63, 80.0, 80.0));
}

// The oriented box of a flat, regularly sampled tile, and its point spacing,
// which must not depend on the terrain's vertical spread (a flat field and a
// sloped one sampled the same way have the same spacing).
ATF_TEST_CASE_WITHOUT_HEAD(test_tile_obb_and_spacing);
ATF_TEST_CASE_BODY(test_tile_obb_and_spacing) {
    auto grid = [](double slope) {
        std::vector<float> pos;
        for (int j = 0; j < 100; ++j)
            for (int i = 0; i < 100; ++i) // 1 unit spacing over 100 × 100
                pos.insert(pos.end(), {static_cast<float>(i), static_cast<float>(slope * i),
                                       static_cast<float>(j)});
        return pos;
    };
    Tile flat{};
    flat.pointCount = 10000;
    computeTileObb(flat, grid(0.0));
    CHECK(std::abs(std::abs(flat.principalAxis3.y) - 1.0f) < 1e-3f); // normal is up
    CHECK(flat.obbExtent3 < 1e-3f);
    CHECK(std::abs(flat.pointSpacing - 1.0f) < 0.02f);

    Tile sloped{};
    sloped.pointCount = 10000;
    computeTileObb(sloped, grid(0.3));                // the same points on a 0.3 slope
    float alongSlope = std::sqrt(1.0f + 0.3f * 0.3f); // true spacing along the surface in x
    // The box follows the slope: the smallest extent is across it (~0), and
    // its axis is the slope's normal.
    CHECK(sloped.obbExtent3 < 1e-2f);
    glm::vec3 normal = glm::normalize(glm::vec3(-0.3f, 1.0f, 0.0f));
    CHECK(std::abs(std::abs(glm::dot(sloped.principalAxis3, normal)) - 1.0f) < 1e-3f);
    CHECK(std::abs(sloped.pointSpacing - std::sqrt(alongSlope)) < 0.02f);
}

// Level of detail: nearer means a finer resolution; a tile smaller than
// ~25 px on screen is left as is (0); the result is clamped to >= 0.1.
ATF_TEST_CASE_WITHOUT_HEAD(test_desired_resolution);
ATF_TEST_CASE_BODY(test_desired_resolution) {
    std::unique_ptr<TileGrid> g(makeGrid(writeCopc("lod.copc.laz")));
    Tile& t = g->tiles[27];
    std::vector<float> pos;
    for (int j = 0; j < 50; ++j)
        for (int i = 0; i < 50; ++i) {
            glm::vec3 p = g->frame.toGL(t.minX + i * 0.2, t.minY + j * 0.2, 1.0);
            pos.insert(pos.end(), {p.x, p.y, p.z});
        }
    t.pointCount = 2500;
    computeTileObb(t, pos);
    auto at = [&](float dist) {
        glm::vec3 cam = t.glCenter + glm::vec3(0.0f, dist, 0.0f); // looking straight down
        return g->desiredResolution(t, cam, 55.0f, 900.0f);
    };
    double near = at(0.05f), mid = at(0.5f), far = at(500.0f);
    CHECK(near > 0.0 && mid > 0.0);
    CHECK(near < mid);  // nearer: finer
    CHECK(near >= 0.1); // clamped
    CHECK(far == 0.0);  // a few pixels: leave it
}

ATF_INIT_TEST_CASES(tcs) {
    ATF_ADD_TEST_CASE(tcs, test_tile_layout);
    ATF_ADD_TEST_CASE(tcs, test_load_by_resolution_each_point_once);
    ATF_ADD_TEST_CASE(tcs, test_tile_obb_and_spacing);
    ATF_ADD_TEST_CASE(tcs, test_desired_resolution);
}
