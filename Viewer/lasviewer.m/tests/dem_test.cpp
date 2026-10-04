// dem_test.cpp — tests of the DEM mesh's level of detail on the real code
// (dem_quadtree.cpp, DEMTessMesh::loadFromDEM), on synthetic rasters.
#include "dem_quadtree.h"
#include "dem_tess_mesh.h"

#include <atf-c++.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <random>
#include <set>
#include <vector>

#define CHECK(cond) ATF_REQUIRE(cond)

namespace {

// A w×h raster with elevation f(col, row) and an optional nodata mask.
struct Raster {
    int w, h;
    std::vector<float> elev;
    std::vector<uint8_t> nodata;
    template <typename F>
    Raster(int width, int height, F f)
        : w(width), h(height), elev(static_cast<size_t>(width) * height), nodata(elev.size(), 0) {
        for (int r = 0; r < h; ++r)
            for (int c = 0; c < w; ++c)
                elev[static_cast<size_t>(r) * w + c] = static_cast<float>(f(c, r));
    }
    DemGrid grid() const { return {elev.data(), nodata.data(), w, h}; }
};

std::map<int, int> histogram(const std::vector<QuadCell>& leaves) {
    std::map<int, int> h;
    for (const QuadCell& c : leaves) h[c.level]++;
    return h;
}

// Every finest-level cell of the grid → the leaf level covering it (-1 none).
std::vector<int> levelMap(const std::vector<QuadCell>& leaves, int coarse, int maxLevel) {
    LeafIndex index(leaves, coarse, maxLevel);
    int n = index.finestCount();
    std::vector<int> m(static_cast<size_t>(n) * n);
    for (int y = 0; y < n; ++y)
        for (int x = 0; x < n; ++x) m[static_cast<size_t>(y) * n + x] = index.levelAt(x, y);
    return m;
}

} // namespace

// A tilted plane is exactly a bilinear patch: zero deviation. The old test
// (elevation range of the cell vs the tolerance) failed every sloped cell.
ATF_TEST_CASE_WITHOUT_HEAD(test_deviation_plane_is_zero);
ATF_TEST_CASE_BODY(test_deviation_plane_is_zero) {
    Raster r(65, 65, [](int c, int row) { return 100.0 + 0.3 * c - 0.7 * row; });
    QuadCell cell;
    cell.cw = cell.ch = 64;
    CHECK(cellDeviation(r.grid(), cell) < 1e-4f);
    QuadCell half;
    half.col = 13.5;
    half.row = 7.25;
    half.cw = 20.5;
    half.ch = 30.0; // fractional footprint
    CHECK(cellDeviation(r.grid(), half) < 1e-4f);
}

// A single raised pixel anywhere inside the cell is found exactly; nodata
// pixels are ignored.
ATF_TEST_CASE_WITHOUT_HEAD(test_deviation_finds_hidden_pixel);
ATF_TEST_CASE_BODY(test_deviation_finds_hidden_pixel) {
    Raster r(65, 65, [](int, int) { return 50.0; });
    r.elev[static_cast<size_t>(37) * 65 + 21] = 52.5f; // off-centre, between any samples
    QuadCell cell;
    cell.cw = cell.ch = 64;
    CHECK(std::abs(cellDeviation(r.grid(), cell) - 2.5f) < 1e-4f);
    r.nodata[static_cast<size_t>(37) * 65 + 21] = 1;
    CHECK(cellDeviation(r.grid(), cell) < 1e-4f);
}

// On a plane, the tree stays at the coarsest level the span cap allows,
// whatever the slope: the regression the old criterion had (all leaves at
// maxLevel on any slope).
ATF_TEST_CASE_WITHOUT_HEAD(test_plane_stays_coarse);
ATF_TEST_CASE_BODY(test_plane_stays_coarse) {
    Raster small(257, 257, [](int c, int row) { return 10.0 + 0.5 * c + 0.2 * row; }); // steep
    auto leaves = buildLeaves(small.grid(), 8, 5, 1.0, 1.0, 1.0, 64.0);
    CHECK(leaves.size() == 64);
    CHECK(histogram(leaves)[0] == 64);

    // Level-0 cells of a 1025-pixel raster span 128 px > 64: one level down.
    Raster big(1025, 1025, [](int c, int) { return 0.1 * c; });
    leaves = buildLeaves(big.grid(), 8, 5, 1.0, 1.0, 1.0, 64.0);
    CHECK(leaves.size() == 256);
    CHECK(histogram(leaves)[1] == 256);
}

// A spike subdivides only the cells around it, down to maxLevel.
ATF_TEST_CASE_WITHOUT_HEAD(test_spike_subdivides_locally);
ATF_TEST_CASE_BODY(test_spike_subdivides_locally) {
    Raster r(257, 257, [](int c, int) { return 0.05 * c; });
    r.elev[static_cast<size_t>(100) * 257 + 150] += 5.0f;
    auto leaves = buildLeaves(r.grid(), 8, 4, 1.0, 1.0, 1.0, 64.0);
    auto h = histogram(leaves);
    CHECK(h[4] >= 1);                      // the spike's cell went all the way down
    CHECK(h[0] >= 60);                     // almost everything else stayed coarse
    CHECK(leaves.size() < 64 + 4 * 4 * 3); // a few levels of 3 extra cells each
}

// A larger collapse angle never produces more patches, and on rough
// terrain it produces clearly fewer: the angle now controls the mesh.
ATF_TEST_CASE_WITHOUT_HEAD(test_collapse_angle_controls_patch_count);
ATF_TEST_CASE_BODY(test_collapse_angle_controls_patch_count) {
    std::mt19937 rng(7);
    std::normal_distribution<double> noise(0.0, 0.05);
    Raster r(513, 513, [&](int c, int row) {
        return 20.0 * std::sin(c / 60.0) * std::cos(row / 45.0) + noise(rng);
    });
    size_t prev = SIZE_MAX;
    std::vector<size_t> counts;
    for (double angle : {0.5, 1.0, 2.0, 5.0, 10.0, 20.0}) {
        size_t n = buildLeaves(r.grid(), 8, 5, angle, 0.5, 0.5, 64.0).size();
        CHECK(n <= prev);
        counts.push_back(n);
        prev = n;
    }
    CHECK(counts.back() * 4 < counts.front());
}

// After balancing, leaves touching along an edge differ by at most one
// level, checked over the whole finest grid (not just edge midpoints).
ATF_TEST_CASE_WITHOUT_HEAD(test_balance_one_level_everywhere);
ATF_TEST_CASE_BODY(test_balance_one_level_everywhere) {
    Raster r(513, 513, [](int c, int) { return 0.01 * c; });
    for (int k = 0; k < 6; ++k)
        r.elev[static_cast<size_t>(50 + 60 * k) * 513 + 70 + 61 * k] += 8.0f;
    const int coarse = 8, maxLevel = 6;
    auto leaves = buildLeaves(r.grid(), coarse, maxLevel, 1.0, 1.0, 1.0, 64.0);
    double cw0 = 512.0 / coarse;
    balanceLeaves(leaves, coarse, maxLevel, cw0, cw0);
    auto m = levelMap(leaves, coarse, maxLevel);
    int n = coarse << maxLevel;
    for (int y = 0; y < n; ++y)
        for (int x = 0; x + 1 < n; ++x) {
            CHECK(std::abs(m[y * n + x] - m[y * n + x + 1]) <= 1);
            CHECK(std::abs(m[x * n + y] - m[(x + 1) * n + y]) <= 1);
        }
    // And the leaves still tile the grid exactly (no overlap, no gap).
    size_t area = 0;
    for (const QuadCell& c : leaves) area += static_cast<size_t>(1) << (2 * (maxLevel - c.level));
    CHECK(area == static_cast<size_t>(n) * n);
    for (int v : m) CHECK(v >= 0);
}

// Edge codes agree across every edge: same level → 0 on both sides; one
// level apart → 2 on the coarse side, 1 or 3 on the fine side, whose far
// point and shared corner are exactly the coarse edge's two ends. With the
// TCS rule (coarse 2K segments, fine K per half edge, K from the coarse
// edge on both sides) the vertices then coincide.
ATF_TEST_CASE_WITHOUT_HEAD(test_edge_codes_agree);
ATF_TEST_CASE_BODY(test_edge_codes_agree) {
    Raster r(513, 513, [](int c, int) { return 0.01 * c; });
    for (int k = 0; k < 5; ++k)
        r.elev[static_cast<size_t>(80 + 70 * k) * 513 + 400 - 65 * k] += 6.0f;
    const int coarse = 8, maxLevel = 5;
    auto leaves = buildLeaves(r.grid(), coarse, maxLevel, 1.0, 1.0, 1.0, 64.0);
    balanceLeaves(leaves, coarse, maxLevel, 64.0, 64.0);
    LeafIndex index(leaves, coarse, maxLevel);
    // Map each finest cell to its leaf's (level, codes, far points).
    int n = coarse << maxLevel;
    std::vector<const QuadCell*> owner(static_cast<size_t>(n) * n, nullptr);
    std::map<const QuadCell*, std::vector<float>> codes;
    std::map<const QuadCell*, std::vector<std::pair<int, int>>> fars;
    for (const QuadCell& c : leaves) {
        float e[4];
        int far[4][2] = {};
        edgeCodes(c, index, maxLevel, e, far);
        codes[&c] = {e[0], e[1], e[2], e[3]};
        for (const auto& f : far) fars[&c].emplace_back(f[0], f[1]);
        int s = 1 << (maxLevel - c.level);
        for (int y = c.iy * s; y < (c.iy + 1) * s; ++y)
            for (int x = c.ix * s; x < (c.ix + 1) * s; ++x)
                owner[static_cast<size_t>(y) * n + x] = &c;
    }
    auto corner = [&](const QuadCell* c, int k) {
        int fx, fy;
        cellCorner(*c, k, maxLevel, fx, fy);
        return std::make_pair(fx, fy);
    };
    // The fine cell's edge on `side` (corners side → side+1) against the
    // coarse cell's edge on `coarseSide`: shared corner + far = its ends.
    auto checkFine = [&](const QuadCell* fine, int side, const QuadCell* coarseCell,
                         int coarseSide) {
        float code = codes[fine][side];
        CHECK(code == 1.0f || code == 3.0f);
        auto shared = code == 1.0f ? corner(fine, side) : corner(fine, (side + 1) % 4);
        std::set<std::pair<int, int>> got{shared, fars[fine][side]};
        std::set<std::pair<int, int>> want{corner(coarseCell, coarseSide),
                                           corner(coarseCell, (coarseSide + 1) % 4)};
        CHECK(got == want);
    };
    int transitions = 0;
    for (int y = 0; y < n; ++y)
        for (int x = 0; x + 1 < n; ++x) {
            const QuadCell* a = owner[static_cast<size_t>(y) * n + x];     // left
            const QuadCell* b = owner[static_cast<size_t>(y) * n + x + 1]; // right
            if (a == b) continue;
            float ca = codes[a][1], cb = codes[b][3]; // a's col+ edge, b's col− edge
            if (a->level == b->level)
                CHECK(ca == 0.0f && cb == 0.0f);
            else if (a->level < b->level) {
                CHECK(ca == 2.0f);
                checkFine(b, 3, a, 1);
                ++transitions;
            } else {
                CHECK(cb == 2.0f);
                checkFine(a, 1, b, 3);
                ++transitions;
            }
        }
    CHECK(transitions > 0);
    // The TCS rule makes the vertex sets identical: coarse edge at k/(2K),
    // fine half edges at j/(2K) and 1/2 + j/(2K).
    for (int K = 1; K <= 32; ++K) {
        std::set<int> coarseSide, fineSide;
        for (int k = 0; k <= 2 * K; ++k) coarseSide.insert(k);
        for (int j = 0; j <= K; ++j) {
            fineSide.insert(j);
            fineSide.insert(K + j);
        }
        CHECK(coarseSide == fineSide);
    }
}

// LeafIndex answers which leaf covers a finest cell, including dropped
// (nodata) regions and positions outside the grid.
ATF_TEST_CASE_WITHOUT_HEAD(test_leaf_index);
ATF_TEST_CASE_BODY(test_leaf_index) {
    std::vector<QuadCell> leaves;
    QuadCell a;
    a.level = 0;
    a.ix = 0;
    a.iy = 0; // covers finest x,y ∈ [0,4)
    QuadCell b;
    b.level = 2;
    b.ix = 5;
    b.iy = 1; // finest (5,1)
    leaves.push_back(a);
    leaves.push_back(b);
    LeafIndex idx(leaves, 2, 2); // finest grid 8×8
    CHECK(idx.finestCount() == 8);
    CHECK(idx.levelAt(0, 0) == 0);
    CHECK(idx.levelAt(3, 3) == 0);
    CHECK(idx.levelAt(5, 1) == 2);
    CHECK(idx.levelAt(6, 1) == -1);
    CHECK(idx.levelAt(-1, 0) == -1);
    CHECK(idx.levelAt(8, 0) == -1);
}

// The real mesh build on a 2000×2000 raster: fast (the old neighbour scan
// took seconds), heightmap UVs on texel centres, edge codes uploaded.
ATF_TEST_CASE_WITHOUT_HEAD(test_mesh_build);
ATF_TEST_CASE_BODY(test_mesh_build) {
    // The raster is made once, outside the timed build (the source only
    // copies it), so the timing measures the mesh build alone.
    const int W = 2000, H = 2000;
    std::mt19937 rng(3);
    std::normal_distribution<double> noise(0.0, 0.03);
    DemRaster raster;
    raster.A = 0.5;
    raster.E = -0.5;
    raster.C = 1000.25;
    raster.F = 2999.75;
    raster.hasGeo = true;
    raster.width = W;
    raster.height = H;
    raster.hasNodata = true;
    raster.nodata = -9999.0f;
    raster.elevations.resize(static_cast<size_t>(W) * H);
    for (int r = 0; r < H; ++r)
        for (int c = 0; c < W; ++c)
            raster.elevations[static_cast<size_t>(r) * W + c] =
                static_cast<float>(300.0 + 30.0 * std::sin(c / 300.0) + noise(rng));
    DemSource source = [&](DemSourceData& out) {
        out.dem = raster;
        return true;
    };
    SceneFrame frame;
    frame.center = glm::dvec3(1500.0, 2500.0, 300.0);
    frame.scale = 500.0;
    DEMTessMesh mesh;
    auto t0 = std::chrono::steady_clock::now();
    CHECK(mesh.loadFromDEM(source, nullptr, frame, 1.0, 5));
    double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    // About 0.1 s on a 2026 laptop. The old quadratic neighbour search took
    // 6.7 s for the same build there; the bound leaves room for slow CI
    // machines and still catches a quadratic algorithm.
    CHECK(secs < 2.5);
    CHECK(mesh.patchCount > 64);
    CHECK(mesh.patchHeightUVs.size() == static_cast<size_t>(mesh.patchCount) * 8);
    CHECK(mesh.patchEdgeConstraint.size() == static_cast<size_t>(mesh.patchCount) * 16);
    // First corner of the first patch sits on a pixel centre: its UV is
    // (col + 0.5) / W for an integer-or-fractional col, i.e. u·W − 0.5 ∈ [0, W−1].
    float minU = 1, maxU = 0;
    for (size_t i = 0; i < mesh.patchHeightUVs.size(); i += 2) {
        minU = std::min(minU, mesh.patchHeightUVs[i]);
        maxU = std::max(maxU, mesh.patchHeightUVs[i]);
    }
    CHECK(std::abs(minU - 0.5f / W) < 1e-6f);
    CHECK(std::abs(maxU - (W - 0.5f) / W) < 1e-6f);
    for (float c : mesh.patchEdgeConstraint)
        CHECK(c == 0.0f || c == 1.0f || c == 2.0f || c == 3.0f);
    // A fine side's far point is, bit for bit, a corner of some patch (the
    // coarse neighbour's): the TCS computes the same K on both sides.
    CHECK(mesh.patchEdgeFar.size() == static_cast<size_t>(mesh.patchCount) * 20);
    std::set<std::array<float, 5>> corners;
    for (size_t v = 0; v < mesh.patchPositions.size() / 3; ++v)
        corners.insert({mesh.patchPositions[3 * v], mesh.patchPositions[3 * v + 1],
                        mesh.patchPositions[3 * v + 2], mesh.patchHeightUVs[2 * v],
                        mesh.patchHeightUVs[2 * v + 1]});
    int fineSides = 0;
    for (size_t v = 0; v < mesh.patchPositions.size() / 3; ++v) {
        float code = mesh.patchEdgeConstraint[4 * v + v % 4]; // edge starting at corner v%4
        if (code != 1.0f && code != 3.0f) continue;
        const float* f = &mesh.patchEdgeFar[5 * v];
        CHECK(corners.count({f[0], f[1], f[2], f[3], f[4]}) == 1);
        ++fineSides;
    }
    CHECK(fineSides > 0);
}

// Nodata takes the nearest valid value; valid pixels are untouched.
ATF_TEST_CASE_WITHOUT_HEAD(test_fill_nodata_nearest);
ATF_TEST_CASE_BODY(test_fill_nodata_nearest) {
    const int w = 6, h = 3;
    std::vector<float> e = {1, 1, 0, 0, 0, 9, 1, 1, 0, 0, 0, 9, 1, 1, 0, 0, 0, 9};
    std::vector<uint8_t> nd = {0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0};
    fillNodataNearest(e, nd, w, h);
    for (int r = 0; r < h; ++r) {
        CHECK(e[r * w + 2] == 1.0f); // next to the 1s
        CHECK(e[r * w + 4] == 9.0f); // next to the 9s
        CHECK(e[r * w + 0] == 1.0f && e[r * w + 5] == 9.0f);
    }
    std::vector<float> none(4, -9999.0f);
    fillNodataNearest(none, std::vector<uint8_t>(4, 1), 2, 2); // nothing valid: unchanged
    CHECK(none[0] == -9999.0f);
}

// The depth at which cells fit kMaxTessSegments pixels: a 20000-pixel DEM
// (8 level-0 cells of ~2500 px) needs level 6, a 2000-pixel one level 2.
ATF_TEST_CASE_WITHOUT_HEAD(test_level_for_span);
ATF_TEST_CASE_BODY(test_level_for_span) {
    CHECK(levelForSpan(19999.0 / 8, 19999.0 / 8, 64.0) == 6);
    CHECK(levelForSpan(1999.0 / 8, 1999.0 / 8, 64.0) == 2);
    CHECK(levelForSpan(64.0, 10.0, 64.0) == 0);
    CHECK(levelForSpan(10.0, 64.5, 64.0) == 1);
}

ATF_INIT_TEST_CASES(tcs) {
    ATF_ADD_TEST_CASE(tcs, test_level_for_span);
    ATF_ADD_TEST_CASE(tcs, test_fill_nodata_nearest);
    ATF_ADD_TEST_CASE(tcs, test_deviation_plane_is_zero);
    ATF_ADD_TEST_CASE(tcs, test_deviation_finds_hidden_pixel);
    ATF_ADD_TEST_CASE(tcs, test_plane_stays_coarse);
    ATF_ADD_TEST_CASE(tcs, test_spike_subdivides_locally);
    ATF_ADD_TEST_CASE(tcs, test_collapse_angle_controls_patch_count);
    ATF_ADD_TEST_CASE(tcs, test_balance_one_level_everywhere);
    ATF_ADD_TEST_CASE(tcs, test_edge_codes_agree);
    ATF_ADD_TEST_CASE(tcs, test_leaf_index);
    ATF_ADD_TEST_CASE(tcs, test_mesh_build);
}
