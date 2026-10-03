// height_model_test.cpp — tests of height_model.cpp on in-memory rasters
// (atf-c++; no files).
#include "height_model.h"

#include <atf-c++.hpp>

#include <cmath>
#include <vector>

#define CHECK(cond) ATF_REQUIRE(cond)

static bool near(double a, double b, double tol) {
    return std::abs(a - b) <= tol;
}

// A north-up raster with pixel centers at (x0 + c*res, y0 - r*res).
static DemRaster makeRaster(int w, int h, double x0, double y0, double res,
                            const std::vector<float>& values) {
    DemRaster d;
    d.A = res;
    d.E = -res;
    d.C = x0;
    d.F = y0;
    d.hasGeo = true;
    d.width = w;
    d.height = h;
    d.hasNodata = true;
    d.nodata = -9999.0f;
    d.elevations = values;
    return d;
}

// A plane z = 100 + x/10 sampled on a coarser grid is exact under bilinear.
ATF_TEST_CASE_WITHOUT_HEAD(test_resample_plane);
ATF_TEST_CASE_BODY(test_resample_plane) {
    std::vector<float> v;
    for (int r = 0; r < 5; ++r)
        for (int c = 0; c < 5; ++c) v.push_back(100.0f + c * 1.0f / 10.0f);
    DemRaster src = makeRaster(5, 5, 0.0, 4.0, 1.0, v);
    RasterGeo grid;
    grid.A = 0.5;
    grid.E = -0.5;
    grid.C = 0.25;
    grid.F = 3.75;
    std::vector<float> out = resampleOnGrid(src, grid, 8, 8);
    for (int r = 0; r < 8; ++r)
        for (int c = 0; c < 8; ++c) {
            double x = 0.25 + 0.5 * c;
            CHECK(near(out[r * 8 + c], 100.0 + x / 10.0, 1e-4));
        }
}

// Nodata neighbors are left out; far outside is NaN.
ATF_TEST_CASE_WITHOUT_HEAD(test_resample_nodata_and_outside);
ATF_TEST_CASE_BODY(test_resample_nodata_and_outside) {
    DemRaster src = makeRaster(2, 1, 0.0, 0.0, 1.0, {10.0f, -9999.0f});
    RasterGeo grid;
    grid.A = 1.0;
    grid.E = -1.0;
    grid.C = 0.5; // halfway between the valid and the nodata pixel
    grid.F = 0.0;
    std::vector<float> out = resampleOnGrid(src, grid, 3, 1);
    CHECK(near(out[0], 10.0, 1e-6)); // only the valid neighbor counts
    CHECK(std::isnan(out[1]));       // x = 1.5: nodata pixel and beyond the edge
    CHECK(std::isnan(out[2]));       // x = 2.5: outside
}

// DHM and DSM inputs give the same surface and ground; heights <= 0 give
// the ground itself.
ATF_TEST_CASE_WITHOUT_HEAD(test_compose_height_and_surface);
ATF_TEST_CASE_BODY(test_compose_height_and_surface) {
    DemRaster dtm = makeRaster(3, 1, 0.0, 0.0, 1.0, {100.0f, 101.0f, 102.0f});
    DemRaster dhm = makeRaster(3, 1, 0.0, 0.0, 1.0, {0.0f, 8.0f, -0.2f});
    DemRaster dsm = makeRaster(3, 1, 0.0, 0.0, 1.0, {100.0f, 109.0f, 101.9f});

    DemRaster a, b;
    std::vector<float> ga, gb;
    CHECK(composeAboveGround(dtm, dhm, AboveGroundKind::Height, a, ga));
    CHECK(composeAboveGround(dtm, dsm, AboveGroundKind::Surface, b, gb));
    const float expected[3] = {100.0f, 109.0f, 102.0f};
    for (int i = 0; i < 3; ++i) {
        CHECK(near(a.elevations[i], expected[i], 1e-4));
        CHECK(near(b.elevations[i], expected[i], 1e-4));
        CHECK(near(ga[i], dtm.elevations[i], 1e-4));
        CHECK(near(gb[i], dtm.elevations[i], 1e-4));
    }
}

// Unknown ground or height is nodata; no overlap at all fails.
ATF_TEST_CASE_WITHOUT_HEAD(test_compose_nodata_and_no_overlap);
ATF_TEST_CASE_BODY(test_compose_nodata_and_no_overlap) {
    DemRaster dtm = makeRaster(2, 1, 0.0, 0.0, 1.0, {100.0f, -9999.0f});
    DemRaster dhm = makeRaster(3, 1, 0.0, 0.0, 1.0, {5.0f, 5.0f, -9999.0f});
    DemRaster out;
    std::vector<float> ground;
    CHECK(composeAboveGround(dtm, dhm, AboveGroundKind::Height, out, ground));
    CHECK(near(out.elevations[0], 105.0, 1e-4));
    CHECK(out.isNodata(out.elevations[1])); // no ground
    CHECK(out.isNodata(out.elevations[2])); // no height, outside the ground

    DemRaster far = makeRaster(2, 1, 1000.0, 0.0, 1.0, {5.0f, 5.0f});
    CHECK(!composeAboveGround(dtm, far, AboveGroundKind::Height, out, ground));
}

// Height above ground on the DTM grid, from a finer DHM and from a DSM.
ATF_TEST_CASE_WITHOUT_HEAD(test_height_above_ground);
ATF_TEST_CASE_BODY(test_height_above_ground) {
    DemRaster dtm = makeRaster(2, 1, 0.0, 0.0, 2.0, {100.0f, 100.0f});
    // 1 m DHM over the same 4 m extent: object 6 m tall around x = 2.
    DemRaster dhm = makeRaster(4, 1, -0.5, 0.0, 1.0, {0.0f, 0.0f, 6.0f, 6.0f});
    std::vector<float> h = heightAboveGround(dtm, dhm, AboveGroundKind::Height);
    CHECK(near(h[0], 0.0, 1e-4));
    CHECK(near(h[1], 6.0, 1e-4));

    DemRaster dsm = makeRaster(2, 1, 0.0, 0.0, 2.0, {100.5f, 112.0f});
    h = heightAboveGround(dtm, dsm, AboveGroundKind::Surface);
    CHECK(near(h[0], 0.5, 1e-4));
    CHECK(near(h[1], 12.0, 1e-4));
}

ATF_INIT_TEST_CASES(tcs) {
    ATF_ADD_TEST_CASE(tcs, test_resample_plane);
    ATF_ADD_TEST_CASE(tcs, test_resample_nodata_and_outside);
    ATF_ADD_TEST_CASE(tcs, test_compose_height_and_surface);
    ATF_ADD_TEST_CASE(tcs, test_compose_nodata_and_no_overlap);
    ATF_ADD_TEST_CASE(tcs, test_height_above_ground);
}
