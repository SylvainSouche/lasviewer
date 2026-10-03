// cli_test.cpp — tests of the command line (cli.cpp), including the
// classification of .tif files by content (small GeoTIFFs written here).
#include "cli.h"
#include "raster.h"

#include <atf-c++.hpp>
#include <gdal_priv.h>

#include <initializer_list>
#include <sstream>
#include <string>
#include <vector>

#define CHECK(cond) ATF_REQUIRE(cond)

namespace {

// A 4×4 GeoTIFF: `bands` bands of `type`.
std::string writeTiff(const std::string& path, int bands, GDALDataType type) {
    rasterInit();
    GDALDriver* drv = GetGDALDriverManager()->GetDriverByName("GTiff");
    GDALDataset* ds = drv->Create(path.c_str(), 4, 4, bands, type, nullptr);
    double gt[6] = {991000.0, 1.0, 0.0, 6558000.0, 0.0, -1.0};
    ds->SetGeoTransform(gt);
    std::vector<double> zeros(16, 0.0);
    for (int b = 1; b <= bands; ++b)
        CHECK(ds->GetRasterBand(b)->RasterIO(GF_Write, 0, 0, 4, 4, zeros.data(), 4, 4, GDT_Float64,
                                             0, 0, nullptr) == CE_None);
    GDALClose(GDALDataset::ToHandle(ds));
    return path;
}

struct Result {
    CliStatus status;
    CommandLine cmd;
    std::string out, err;
};

Result parse(std::initializer_list<const char*> args) {
    std::vector<const char*> argv{"lasviewer"};
    argv.insert(argv.end(), args.begin(), args.end());
    Result r;
    std::ostringstream out, err;
    r.status = parseCommandLine(static_cast<int>(argv.size()), argv.data(), r.cmd, out, err);
    r.out = out.str();
    r.err = err.str();
    return r;
}

bool mentions(const std::string& s, const char* what) {
    return s.find(what) != std::string::npos;
}

} // namespace

ATF_TEST_CASE_WITHOUT_HEAD(test_help_and_errors);
ATF_TEST_CASE_BODY(test_help_and_errors) {
    Result r = parse({"-h"});
    CHECK(r.status == CliStatus::Help && mentions(r.out, "Usage:") &&
          mentions(r.out, kLasviewerVersion));
    CHECK(parse({"--help"}).status == CliStatus::Help);

    r = parse({});
    CHECK(r.status == CliStatus::Error && mentions(r.err, "no point cloud or DEM"));
    r = parse({"--frobnicate", "a.laz"});
    CHECK(r.status == CliStatus::Error && mentions(r.err, "unknown option --frobnicate"));
    for (const char* opt : {"-o", "-d", "-dtm", "-mnt", "-dhm", "-mnh", "-dsm", "-mns", "-cop",
                            "--dem-lod", "--view", "--snapshot"}) {
        r = parse({"a.laz", opt});
        CHECK(r.status == CliStatus::Error && mentions(r.err, "needs a value"));
    }
}

ATF_TEST_CASE_WITHOUT_HEAD(test_point_clouds);
ATF_TEST_CASE_BODY(test_point_clouds) {
    Result r = parse({"a.laz", "b.copc.laz", "-cop", "c.LAS", "--snapshot", "s.ppm"});
    CHECK(r.status == CliStatus::Run);
    CHECK(r.cmd.plan.clouds == (std::vector<std::string>{"a.laz", "b.copc.laz", "c.LAS"}));
    CHECK(r.cmd.plan.dems.empty() && r.cmd.plan.ortho.empty());
    CHECK(r.cmd.snapshot == "s.ppm" && !r.cmd.viewSet);
}

// Positional .tif: 8-bit with >= 3 bands is the orthophoto, anything else a
// DEM; the extension is matched case-insensitively.
ATF_TEST_CASE_WITHOUT_HEAD(test_tiff_classified_by_content);
ATF_TEST_CASE_BODY(test_tiff_classified_by_content) {
    std::string rgb = writeTiff("ortho.TIF", 3, GDT_Byte);
    std::string dem = writeTiff("dem.tiff", 1, GDT_Float32);
    std::string grey = writeTiff("grey.tif", 1, GDT_Byte); // 8-bit, 1 band: a DEM
    Result r = parse({dem.c_str(), rgb.c_str(), grey.c_str(), "c.laz"});
    CHECK(r.status == CliStatus::Run);
    CHECK(r.cmd.plan.ortho == rgb);
    CHECK(r.cmd.plan.dems == (std::vector<std::string>{dem, grey}));

    // Two orthophotos, positional or one given with -o: an error.
    std::string rgb2 = writeTiff("ortho2.tif", 4, GDT_Byte);
    r = parse({rgb.c_str(), rgb2.c_str(), dem.c_str()});
    CHECK(r.status == CliStatus::Error && mentions(r.err, "several orthophotos"));
    r = parse({"-o", rgb2.c_str(), rgb.c_str(), dem.c_str()});
    CHECK(r.status == CliStatus::Error && mentions(r.err, "several orthophotos"));
    // -d forces a DEM even for imagery (8-bit Terrain-RGB DEMs).
    r = parse({"-d", rgb.c_str()});
    CHECK(r.status == CliStatus::Run && r.cmd.plan.dems.size() == 1 && r.cmd.plan.ortho.empty());
}

ATF_TEST_CASE_WITHOUT_HEAD(test_height_models);
ATF_TEST_CASE_BODY(test_height_models) {
    Result r = parse(
        {"-mnt", "t.tif", "-mnh", "h.tif", "-dsm", "s.tif", "-dtm", "t2.tif", "-d", "t3.tif"});
    CHECK(r.status == CliStatus::Run);
    CHECK(r.cmd.plan.dems == (std::vector<std::string>{"t.tif", "t2.tif", "t3.tif"}));
    CHECK(r.cmd.plan.aboveGround.size() == 2);
    CHECK(r.cmd.plan.aboveGround[0].path == "h.tif" &&
          r.cmd.plan.aboveGround[0].kind == AboveGroundKind::Height);
    CHECK(r.cmd.plan.aboveGround[1].path == "s.tif" &&
          r.cmd.plan.aboveGround[1].kind == AboveGroundKind::Surface);
    r = parse({"-mnh", "h.tif", "c.laz"}); // no terrain model to stand on
    CHECK(r.status == CliStatus::Error && mentions(r.err, "need a terrain model"));
}

ATF_TEST_CASE_WITHOUT_HEAD(test_dem_lod_and_view);
ATF_TEST_CASE_BODY(test_dem_lod_and_view) {
    Result r = parse({"--dem-lod", "2.5,7", "--view", "991000,6557000,800,150,-20,35.5", "a.laz"});
    CHECK(r.status == CliStatus::Run);
    CHECK(r.cmd.plan.demAngle == 2.5 && r.cmd.plan.demMaxLevel == 7);
    CHECK(r.cmd.viewSet && r.cmd.view[0] == 991000 && r.cmd.view[3] == 150 &&
          r.cmd.view[5] == 35.5);
    for (const char* bad : {"0,5", "-1,5", "1,11", "1,-1", "1,2.5", "1,5x", "1", "a,b", "1,5,6"}) {
        r = parse({"--dem-lod", bad, "a.laz"});
        CHECK(r.status == CliStatus::Error && mentions(r.err, "--dem-lod needs"));
    }
    for (const char* bad : {"1,2,3,4,5", "1,2,3,0,5,6", "1,2,3,4,5,6,7", "1,2,3,4,5,6x"}) {
        r = parse({"--view", bad, "a.laz"});
        CHECK(r.status == CliStatus::Error && mentions(r.err, "--view needs"));
    }
}

ATF_INIT_TEST_CASES(tcs) {
    ATF_ADD_TEST_CASE(tcs, test_help_and_errors);
    ATF_ADD_TEST_CASE(tcs, test_point_clouds);
    ATF_ADD_TEST_CASE(tcs, test_tiff_classified_by_content);
    ATF_ADD_TEST_CASE(tcs, test_height_models);
    ATF_ADD_TEST_CASE(tcs, test_dem_lod_and_view);
}
