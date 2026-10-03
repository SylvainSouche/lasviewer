// raster_test.cpp — tests of raster.cpp against small GeoTIFFs written here
// with GDAL (atf-c++; no external data needed).
#include "raster.h"
#include "scene_frame.h"

#include <atf-c++.hpp>
#include <cpl_conv.h>
#include <gdal_priv.h>
#include <ogr_spatialref.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#define CHECK(cond) ATF_REQUIRE(cond)

static bool near(double a, double b, double tol) {
    return std::abs(a - b) <= tol;
}

static std::string wktOf(const char* userInput) {
    OGRSpatialReference s;
    s.SetFromUserInput(userInput);
    char* w = nullptr;
    s.exportToWkt(&w);
    std::string out = w ? w : "";
    CPLFree(w);
    return out;
}

// Writes a GeoTIFF. gt = GDAL geotransform (corner origin); srs may be null.
static std::string writeTiff(const std::string& name, int w, int h, int bands, GDALDataType type,
                             const double* gt, const char* srs,
                             const std::vector<std::vector<double>>& bandValues,
                             const double* nodata = nullptr) {
    rasterInit();
    std::string path = name; // kyua runs each test case in its own scratch dir
    GDALDriver* drv = GetGDALDriverManager()->GetDriverByName("GTiff");
    GDALDataset* ds = drv->Create(path.c_str(), w, h, bands, type, nullptr);
    if (gt) ds->SetGeoTransform(const_cast<double*>(gt));
    if (srs) {
        OGRSpatialReference s;
        s.SetFromUserInput(srs);
        ds->SetSpatialRef(&s);
    }
    for (int b = 0; b < bands; ++b) {
        std::vector<double> v = bandValues[b];
        GDALRasterBand* band = ds->GetRasterBand(b + 1);
        if (nodata) band->SetNoDataValue(*nodata);
        CPLErr e = band->RasterIO(GF_Write, 0, 0, w, h, v.data(), w, h, GDT_Float64, 0, 0, nullptr);
        (void)e;
    }
    GDALClose(ds);
    return path;
}

// Georeferencing: GDAL's corner-origin geotransform becomes the pixel-center
// affine; EPSG and full-resolution pixels are read correctly.
ATF_TEST_CASE_WITHOUT_HEAD(test_ortho_georef_pixel_center);
ATF_TEST_CASE_BODY(test_ortho_georef_pixel_center) {
    const double gt[6] = {1000.0, 2.0, 0.0, 5000.0, 0.0, -2.0};
    std::vector<double> r(100), g(100), b(100);
    for (int i = 0; i < 100; ++i) {
        r[i] = i;
        g[i] = 2 * i;
        b[i] = 255 - i;
    }
    std::string p = writeTiff("ortho.tif", 10, 10, 3, GDT_Byte, gt, "EPSG:2154", {r, g, b});
    Orthophoto o;
    CHECK(loadOrthophoto(p, o, 1'000'000, ""));
    CHECK(o.width == 10 && o.height == 10 && o.hasGeo);
    CHECK(near(o.C, 1001.0, 1e-9) && near(o.F, 4999.0, 1e-9));
    CHECK(near(o.A, 2.0, 1e-12) && near(o.E, -2.0, 1e-12));
    CHECK(o.epsg == 2154 && !o.reprojected);
    size_t px = (3 * 10 + 7) * 4; // row 3, col 7 → index 37
    CHECK(o.pixels[px] == 37 && o.pixels[px + 1] == 74 && o.pixels[px + 2] == 218);
    CHECK(o.pixels[px + 3] == 255);
}

// Downsampling: pixel count capped, 2x2 blocks averaged, affine rescaled
// with the pixel-center origin moved to the center of the new pixel.
ATF_TEST_CASE_WITHOUT_HEAD(test_ortho_downsample);
ATF_TEST_CASE_BODY(test_ortho_downsample) {
    const double gt[6] = {1000.0, 1.0, 0.0, 5000.0, 0.0, -1.0};
    std::vector<double> v(64 * 64);
    for (int y = 0; y < 64; ++y)
        for (int x = 0; x < 64; ++x) v[y * 64 + x] = ((x / 2 + y / 2) % 2) ? 200 : 100;
    std::string p = writeTiff("big.tif", 64, 64, 3, GDT_Byte, gt, "EPSG:2154", {v, v, v});
    Orthophoto o;
    CHECK(loadOrthophoto(p, o, 32 * 32, ""));
    CHECK(o.width == 32 && o.height == 32);
    CHECK(near(o.A, 2.0, 1e-12) && near(o.E, -2.0, 1e-12));
    CHECK(near(o.C, 1001.0, 1e-9) && near(o.F, 4999.0, 1e-9));
    // Each output pixel is exactly one uniform 2x2 block.
    CHECK(o.pixels[0] == 100 && o.pixels[4] == 200);
}

// A .tfw world file (pixel-center convention) georeferences an untagged TIFF.
ATF_TEST_CASE_WITHOUT_HEAD(test_ortho_world_file);
ATF_TEST_CASE_BODY(test_ortho_world_file) {
    std::vector<double> v(16, 50);
    std::string p = writeTiff("noref.tif", 4, 4, 3, GDT_Byte, nullptr, nullptr, {v, v, v});
    std::ofstream("noref.tfw") << "0.5\n0\n0\n-0.5\n200000.25\n6000000.75\n";
    Orthophoto o;
    CHECK(loadOrthophoto(p, o, 1'000'000, ""));
    CHECK(o.hasGeo);
    CHECK(near(o.C, 200000.25, 1e-9) && near(o.F, 6000000.75, 1e-9));
    CHECK(near(o.A, 0.5, 1e-12) && near(o.E, -0.5, 1e-12));
}

// Declared nodata is honoured (even 0); undeclared falls back to < -9000.
ATF_TEST_CASE_WITHOUT_HEAD(test_dem_nodata);
ATF_TEST_CASE_BODY(test_dem_nodata) {
    const double gt[6] = {0.0, 1.0, 0.0, 10.0, 0.0, -1.0};
    std::vector<double> v = {0, 5, 10, 15};
    double nd = 0.0;
    std::string p = writeTiff("dem_nd.tif", 2, 2, 1, GDT_Float32, gt, "EPSG:2154", {v}, &nd);
    DemRaster d;
    CHECK(loadDEM(p, d, ""));
    CHECK(d.hasNodata && d.nodata == 0.0f);
    CHECK(d.isNodata(0.0f) && !d.isNodata(5.0f) && !d.isNodata(-9999.0f));
    CHECK(d.elevations.size() == 4 && d.elevations[3] == 15.0f);

    std::string q =
        writeTiff("dem_plain.tif", 2, 2, 1, GDT_Float32, gt, "EPSG:2154", {{-9999, 1, 2, 3}});
    DemRaster e;
    CHECK(loadDEM(q, e, ""));
    CHECK(!e.hasNodata && e.isNodata(-9999.0f) && !e.isNodata(0.0f));
}

// Terrain RGB: h = (R*65536 + G*256 + B) * 0.1 - 10000.
ATF_TEST_CASE_WITHOUT_HEAD(test_dem_terrain_rgb);
ATF_TEST_CASE_BODY(test_dem_terrain_rgb) {
    const double gt[6] = {0.0, 1.0, 0.0, 10.0, 0.0, -1.0};
    // 1234.5 m → (1234.5 + 10000) / 0.1 = 112345 = 1*65536 + 182*256 + 217
    std::string p = writeTiff("trgb.tif", 1, 1, 3, GDT_Byte, gt, "EPSG:2154", {{1}, {182}, {217}});
    DemRaster d;
    CHECK(loadDEM(p, d, ""));
    CHECK(d.terrainRGB);
    CHECK(near(d.elevations[0], 1234.5, 1e-2));
}

// CRS helpers: a compound CRS matches its horizontal part.
ATF_TEST_CASE_WITHOUT_HEAD(test_crs_helpers);
ATF_TEST_CASE_BODY(test_crs_helpers) {
    std::string l93 = wktOf("EPSG:2154"), l93ngf = wktOf("EPSG:2154+5720"),
                wgs = wktOf("EPSG:4326");
    CHECK(sameHorizontalCRS(l93, l93ngf));
    CHECK(!sameHorizontalCRS(l93, wgs));
    CHECK(!sameHorizontalCRS(l93, ""));
    CHECK(horizontalEPSG(l93ngf) == 2154);
    CHECK(horizontalEPSG(wgs) == 4326);
    CHECK(horizontalEPSG("") == 0);

    WorldBounds b;
    b.extendXY(6.75, 46.03, 6.80, 46.07); // lon/lat
    CHECK(transformExtent(b, wgs, l93));
    CHECK(b.min.x > 980000 && b.max.x < 1000000 && b.min.y > 6550000 && b.max.y < 6565000);
}

// A DEM in another CRS is warped into the scene CRS: a linear ramp in
// longitude read back at a Lambert-93 point matches the value expected at
// that point's longitude.
ATF_TEST_CASE_WITHOUT_HEAD(test_dem_warped_into_scene_crs);
ATF_TEST_CASE_BODY(test_dem_warped_into_scene_crs) {
    const double lon0 = 6.75, lat0 = 46.07, step = 0.0005;
    const int n = 100;
    const double gt[6] = {lon0, step, 0.0, lat0, 0.0, -step};
    std::vector<double> v(n * n);
    for (int y = 0; y < n; ++y)
        for (int x = 0; x < n; ++x)
            v[y * n + x] = 1000.0 + 10000.0 * (lon0 + (x + 0.5) * step - lon0);
    std::string p = writeTiff("dem_wgs.tif", n, n, 1, GDT_Float32, gt, "EPSG:4326", {v});

    std::string l93 = wktOf("EPSG:2154");
    DemRaster d;
    CHECK(loadDEM(p, d, l93));
    CHECK(d.reprojected && d.epsg == 2154 && d.hasNodata);

    // Pick a point in the middle, in lon/lat, and find its Lambert-93 position.
    double lon = lon0 + 0.0237, lat = lat0 - 0.0211;
    OGRSpatialReference wgs, lam;
    wgs.SetFromUserInput("EPSG:4326");
    wgs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
    lam.SetFromUserInput("EPSG:2154");
    lam.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
    OGRCoordinateTransformation* ct = OGRCreateCoordinateTransformation(&wgs, &lam);
    double x = lon, y = lat;
    CHECK(ct && ct->Transform(1, &x, &y));
    OGRCoordinateTransformation::DestroyCT(ct);

    // Invert the (north-up) pixel-center affine of the warped grid.
    double col = (x - d.C) / d.A, row = (y - d.F) / d.E;
    int c = static_cast<int>(std::lround(col)), r = static_cast<int>(std::lround(row));
    CHECK(c > 0 && r > 0 && c < d.width && r < d.height);
    float got = d.elevations[static_cast<size_t>(r) * d.width + c];
    double expected = 1000.0 + 10000.0 * (lon - lon0);
    // Nearest warped pixel ≈ 30 m; the ramp is ~0.13 m of value per metre east.
    CHECK(!d.isNodata(got));
    CHECK(near(got, expected, 5.0));
}

// A rotated grid (in the scene CRS already) is resampled to north-up.
ATF_TEST_CASE_WITHOUT_HEAD(test_dem_rotated_grid_made_north_up);
ATF_TEST_CASE_BODY(test_dem_rotated_grid_made_north_up) {
    const double a = 0.3; // radians
    const double gt[6] = {1000.0, std::cos(a), std::sin(a), 5000.0, std::sin(a), -std::cos(a)};
    std::vector<double> v(20 * 20, 42.0);
    std::string p = writeTiff("dem_rot.tif", 20, 20, 1, GDT_Float32, gt, "EPSG:2154", {v});
    DemRaster d;
    CHECK(loadDEM(p, d, wktOf("EPSG:2154")));
    CHECK(d.reprojected && d.B == 0.0 && d.D == 0.0 && d.epsg == 2154);
    // The rotated square covers roughly half of its north-up bounding box.
    size_t valid = 0;
    for (float e : d.elevations)
        if (!d.isNodata(e)) {
            ++valid;
            CHECK(near(e, 42.0, 1e-3));
        }
    CHECK(valid > d.elevations.size() / 3 && valid < d.elevations.size());
}

ATF_INIT_TEST_CASES(tcs) {
    ATF_ADD_TEST_CASE(tcs, test_ortho_georef_pixel_center);
    ATF_ADD_TEST_CASE(tcs, test_ortho_downsample);
    ATF_ADD_TEST_CASE(tcs, test_ortho_world_file);
    ATF_ADD_TEST_CASE(tcs, test_dem_nodata);
    ATF_ADD_TEST_CASE(tcs, test_dem_terrain_rgb);
    ATF_ADD_TEST_CASE(tcs, test_crs_helpers);
    ATF_ADD_TEST_CASE(tcs, test_dem_warped_into_scene_crs);
    ATF_ADD_TEST_CASE(tcs, test_dem_rotated_grid_made_north_up);
}
