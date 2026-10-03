// las_test.cpp — tests of las_format.cpp and point_cloud.cpp (laz-perf) on
// small LAS files written here byte by byte from the LAS specification
// (atf-c++; no external data needed).
#include "las_format.h"
#include "point_cloud.h"
#include "raster.h"
#include "scene_frame.h"

#include <atf-c++.hpp>
#include <cpl_conv.h>
#include <ogr_spatialref.h>

#include <cmath>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

namespace {

bool near(double a, double b, double tol) {
    return std::abs(a - b) <= tol;
}

// Little-endian byte writer.
struct Bytes {
    std::vector<char> b;
    template <typename T> void put(T v) {
        const char* p = reinterpret_cast<const char*>(&v);
        b.insert(b.end(), p, p + sizeof(T));
    }
    void text(const char* s, size_t width) {
        std::string t(s);
        t.resize(width, '\0');
        b.insert(b.end(), t.begin(), t.end());
    }
    void zeros(size_t n) { b.insert(b.end(), n, '\0'); }
    template <typename T> void patch(size_t at, T v) { std::memcpy(b.data() + at, &v, sizeof(T)); }
};

struct Pt {
    double x, y, z;
    uint16_t r, g, b;
};

const double kScale = 0.01, kOffX = 991000.0, kOffY = 6557000.0, kOffZ = 0.0;

// Common header part (bytes 0-226); returns the builder.
Bytes header(uint8_t minor, uint16_t headerSize, uint32_t vlrCount, uint8_t format,
             uint16_t recordLength, uint32_t legacyCount, const std::vector<Pt>& pts) {
    Bytes h;
    h.text("LASF", 4);
    h.put<uint16_t>(0); // file source id
    h.put<uint16_t>(0); // global encoding
    h.zeros(16);        // GUID
    h.put<uint8_t>(1);
    h.put<uint8_t>(minor);
    h.text("lasviewer test", 32);
    h.text("las_test.cpp", 32);
    h.put<uint16_t>(1);
    h.put<uint16_t>(2026);
    h.put<uint16_t>(headerSize);
    h.put<uint32_t>(0); // offset to point data, patched later (byte 96)
    h.put<uint32_t>(vlrCount);
    h.put<uint8_t>(format);
    h.put<uint16_t>(recordLength);
    h.put<uint32_t>(legacyCount);
    for (int i = 0; i < 5; ++i) h.put<uint32_t>(i == 0 ? legacyCount : 0);
    h.put<double>(kScale);
    h.put<double>(kScale);
    h.put<double>(kScale);
    h.put<double>(kOffX);
    h.put<double>(kOffY);
    h.put<double>(kOffZ);
    double minx = 1e30, maxx = -1e30, miny = 1e30, maxy = -1e30, minz = 1e30, maxz = -1e30;
    for (const Pt& p : pts) {
        minx = std::min(minx, p.x);
        maxx = std::max(maxx, p.x);
        miny = std::min(miny, p.y);
        maxy = std::max(maxy, p.y);
        minz = std::min(minz, p.z);
        maxz = std::max(maxz, p.z);
    }
    h.put<double>(maxx);
    h.put<double>(minx);
    h.put<double>(maxy);
    h.put<double>(miny);
    h.put<double>(maxz);
    h.put<double>(minz);
    return h;
}

void vlrHeader(Bytes& f, const char* user, uint16_t recordId, uint16_t length) {
    f.put<uint16_t>(0);
    f.text(user, 16);
    f.put<uint16_t>(recordId);
    f.put<uint16_t>(length);
    f.text("", 32);
}

void pointRecord(Bytes& f, const Pt& p, int format) {
    f.put<int32_t>(static_cast<int32_t>(std::lround((p.x - kOffX) / kScale)));
    f.put<int32_t>(static_cast<int32_t>(std::lround((p.y - kOffY) / kScale)));
    f.put<int32_t>(static_cast<int32_t>(std::lround((p.z - kOffZ) / kScale)));
    if (format == 2) {
        f.zeros(8);         // intensity .. point source id: 20 bytes so far
    } else {                // format 7
        f.zeros(10);        // intensity .. point source id
        f.put<double>(0.0); // GPS time: 30 bytes so far
    }
    f.put<uint16_t>(p.r);
    f.put<uint16_t>(p.g);
    f.put<uint16_t>(p.b);
}

const std::vector<Pt> kPts = {
    {991010.25, 6557020.50, 800.75, 65535, 0, 0},
    {991500.00, 6557500.00, 900.00, 0, 65535, 0},
    {991990.50, 6557980.25, 1000.50, 0, 0, 65535},
};

// LAS 1.2, format 2 (RGB at byte 20), CRS as a GeoTIFF key directory VLR.
std::string writeLas12(const std::string& path) {
    const uint16_t keys[] = {1, 1, 0, 1, 3072, 0, 1, 2154}; // ProjectedCSTypeGeoKey = 2154
    Bytes f = header(2, 227, 1, 2, 26, static_cast<uint32_t>(kPts.size()), kPts);
    vlrHeader(f, "LASF_Projection", 34735, sizeof(keys));
    f.b.insert(f.b.end(), reinterpret_cast<const char*>(keys),
               reinterpret_cast<const char*>(keys) + sizeof(keys));
    f.patch<uint32_t>(96, static_cast<uint32_t>(f.b.size()));
    for (const Pt& p : kPts) pointRecord(f, p, 2);
    std::ofstream(path, std::ios::binary)
        .write(f.b.data(), static_cast<std::streamsize>(f.b.size()));
    return path;
}

// LAS 1.4, format 7 (RGB at byte 30), CRS as WKT in an extended VLR after
// the point data.
std::string writeLas14(const std::string& path, const std::string& wkt) {
    Bytes f = header(4, 375, 0, 7, 36, 0, kPts);
    f.put<uint64_t>(0); // start of waveform data
    size_t evlrOffsetAt = f.b.size();
    f.put<uint64_t>(0);           // start of first EVLR, patched below
    f.put<uint32_t>(1);           // number of EVLRs
    f.put<uint64_t>(kPts.size()); // point count (64-bit)
    for (int i = 0; i < 15; ++i) f.put<uint64_t>(i == 0 ? kPts.size() : 0);
    f.patch<uint32_t>(96, static_cast<uint32_t>(f.b.size()));
    for (const Pt& p : kPts) pointRecord(f, p, 7);
    f.patch<uint64_t>(evlrOffsetAt, f.b.size());
    f.put<uint16_t>(0);
    f.text("LASF_Projection", 16);
    f.put<uint16_t>(2112);
    f.put<uint64_t>(wkt.size() + 1);
    f.text("", 32);
    f.text(wkt.c_str(), wkt.size() + 1);
    std::ofstream(path, std::ios::binary)
        .write(f.b.data(), static_cast<std::streamsize>(f.b.size()));
    return path;
}

std::string wktOf(const char* input) {
    rasterInit();
    OGRSpatialReference s;
    s.SetFromUserInput(input);
    char* w = nullptr;
    s.exportToWkt(&w);
    std::string out = w ? w : "";
    CPLFree(w);
    return out;
}

void checkPoints(const PointCloud& cloud, const SceneFrame& frame) {
    ATF_REQUIRE_EQ(cloud.pointCount, kPts.size());
    ATF_REQUIRE(cloud.hasRGB);
    for (size_t i = 0; i < kPts.size(); ++i) {
        glm::dvec3 w = frame.toWorld(glm::vec3(cloud.positions[i * 3], cloud.positions[i * 3 + 1],
                                               cloud.positions[i * 3 + 2]));
        ATF_REQUIRE(near(w.x, kPts[i].x, 0.02) && near(w.y, kPts[i].y, 0.02) &&
                    near(w.z, kPts[i].z, 0.02));
        ATF_REQUIRE(near(cloud.colors[i * 3 + 0], kPts[i].r / 65535.0, 1e-6));
        ATF_REQUIRE(near(cloud.colors[i * 3 + 1], kPts[i].g / 65535.0, 1e-6));
        ATF_REQUIRE(near(cloud.colors[i * 3 + 2], kPts[i].b / 65535.0, 1e-6));
    }
}

} // namespace

// RGB offsets per point format, compression bits masked, short records refused.
ATF_TEST_CASE_WITHOUT_HEAD(test_record_layout);
ATF_TEST_CASE_BODY(test_record_layout) {
    LasRecordLayout l;
    ATF_REQUIRE(lasRecordLayout(2, 26, glm::dvec3(1), glm::dvec3(0), l) && l.rgbOffset == 20);
    ATF_REQUIRE(lasRecordLayout(3, 34, glm::dvec3(1), glm::dvec3(0), l) && l.rgbOffset == 28);
    ATF_REQUIRE(lasRecordLayout(6, 30, glm::dvec3(1), glm::dvec3(0), l) && !l.hasRGB());
    ATF_REQUIRE(lasRecordLayout(0x80 | 7, 36, glm::dvec3(1), glm::dvec3(0), l) && l.format == 7 &&
                l.rgbOffset == 30);
    ATF_REQUIRE(!lasRecordLayout(2, 20, glm::dvec3(1), glm::dvec3(0), l));  // too short
    ATF_REQUIRE(!lasRecordLayout(11, 80, glm::dvec3(1), glm::dvec3(0), l)); // unknown
}

// LAS 1.2: header, GeoTIFF-key CRS, full load with RGB.
ATF_TEST_CASE_WITHOUT_HEAD(test_las12_geokeys_rgb);
ATF_TEST_CASE_BODY(test_las12_geokeys_rgb) {
    std::string path = writeLas12("t12.las");
    CloudHeader h;
    ATF_REQUIRE(readCloudHeader(path, h));
    ATF_REQUIRE_EQ(h.pointCount, 3u);
    ATF_REQUIRE(near(h.bounds.min.x, 991010.25, 1e-6) && near(h.bounds.max.y, 6557980.25, 1e-6));
    ATF_REQUIRE(near(h.bounds.min.z, 800.75, 1e-6) && near(h.bounds.max.z, 1000.5, 1e-6));
    ATF_REQUIRE_EQ(h.epsg, 2154);

    SceneFrame frame = SceneFrame::fromBounds(h.bounds);
    PointCloud cloud;
    ATF_REQUIRE(loadPointCloud(path, frame, cloud));
    checkPoints(cloud, frame);
}

// LAS 1.4, point format 7: 64-bit count, WKT in an extended VLR (compound
// CRS, whose horizontal part is identified), full load with RGB.
ATF_TEST_CASE_WITHOUT_HEAD(test_las14_evlr_wkt_rgb);
ATF_TEST_CASE_BODY(test_las14_evlr_wkt_rgb) {
    std::string wkt = wktOf("EPSG:2154+5720");
    std::string path = writeLas14("t14.las", wkt);
    ATF_REQUIRE(sameHorizontalCRS(lasCrsWkt(path), wkt));
    CloudHeader h;
    ATF_REQUIRE(readCloudHeader(path, h));
    ATF_REQUIRE_EQ(h.pointCount, 3u);
    ATF_REQUIRE_EQ(h.epsg, 2154);

    SceneFrame frame = SceneFrame::fromBounds(h.bounds);
    PointCloud cloud;
    ATF_REQUIRE(loadPointCloud(path, frame, cloud));
    checkPoints(cloud, frame);
}

// A file with no CRS record yields an empty WKT, not an error.
ATF_TEST_CASE_WITHOUT_HEAD(test_no_crs);
ATF_TEST_CASE_BODY(test_no_crs) {
    Bytes f = header(2, 227, 0, 2, 26, 3, kPts);
    f.patch<uint32_t>(96, static_cast<uint32_t>(f.b.size()));
    for (const Pt& p : kPts) pointRecord(f, p, 2);
    std::ofstream("nocrs.las", std::ios::binary)
        .write(f.b.data(), static_cast<std::streamsize>(f.b.size()));
    ATF_REQUIRE(lasCrsWkt("nocrs.las").empty());
    CloudHeader h;
    ATF_REQUIRE(readCloudHeader("nocrs.las", h));
    ATF_REQUIRE_EQ(h.epsg, 0);
}

// A cloud over the 2M-point limit is thinned in one pass: about 2M points
// kept (no cut-off at exactly 2M, which used to drop the cells reached last
// in file order), spread evenly, deterministic, and each kept point the
// first of its cell in file order. Point index i is stored in the RGB
// (r = low 16 bits, g = high 16 bits) so kept points can be identified.
ATF_TEST_CASE_WITHOUT_HEAD(test_thinning_streamed);
ATF_TEST_CASE_BODY(test_thinning_streamed) {
    const int side = 1600;    // 2.56M points, scan order
    const double step = 0.05; // 80 m x 80 m
    std::vector<Pt> corners = {
        {991000.0, 6557000.0, 10.0, 0, 0, 0},
        {991000.0 + step * (side - 1), 6557000.0 + step * (side - 1), 20.0, 0, 0, 0}};
    const uint32_t n = static_cast<uint32_t>(side) * side;
    Bytes f = header(2, 227, 0, 2, 26, n, corners);
    f.patch<uint32_t>(96, static_cast<uint32_t>(f.b.size()));
    f.b.reserve(f.b.size() + static_cast<size_t>(n) * 26);
    for (uint32_t i = 0; i < n; ++i) {
        Pt p{991000.0 + step * (i % side),      6557000.0 + step * (i / side),  10.0 + (i % 7),
             static_cast<uint16_t>(i & 0xFFFF), static_cast<uint16_t>(i >> 16), 0};
        pointRecord(f, p, 2);
    }
    std::ofstream("big.las", std::ios::binary)
        .write(f.b.data(), static_cast<std::streamsize>(f.b.size()));

    CloudHeader h;
    ATF_REQUIRE(readCloudHeader("big.las", h));
    SceneFrame frame = SceneFrame::fromBounds(h.bounds);
    PointCloud a, b;
    ATF_REQUIRE(loadPointCloud("big.las", frame, a));
    ATF_REQUIRE(loadPointCloud("big.las", frame, b));
    ATF_REQUIRE(a.positions == b.positions && a.colors == b.colors); // deterministic
    // The points are denser than the ~1415 x 1415 grid, so every cell is
    // occupied and every cell must be kept: more than 2M exactly (the old
    // loader stopped at 2,000,000, dropping the cells reached last).
    ATF_REQUIRE(a.pointCount > 2'000'000 && a.pointCount < 2'050'000);

    // Recover each kept point's file index; indices strictly increase (file
    // order kept), and the grid's last row (the cells reached last in file
    // order, which the old cut-off at exactly 2M dropped) is represented.
    uint32_t prev = 0;
    int lastRows = 0;
    std::vector<int> blocks(16, 0); // 4 x 4 blocks of the area
    for (size_t k = 0; k < a.pointCount; ++k) {
        uint32_t r = static_cast<uint32_t>(std::lround(a.colors[k * 3] * 65535.0));
        uint32_t g = static_cast<uint32_t>(std::lround(a.colors[k * 3 + 1] * 65535.0));
        uint32_t idx = r | (g << 16);
        ATF_REQUIRE(k == 0 || idx > prev);
        prev = idx;
        if (idx / side >= side - 3) ++lastRows;
        blocks[(idx / side) * 4 / side * 4 + (idx % side) * 4 / side]++;
    }
    ATF_REQUIRE(lastRows > side / 2); // about one grid row (~1400 cells)
    for (int c : blocks)
        ATF_REQUIRE(std::abs(c - static_cast<int>(a.pointCount / 16)) <
                    static_cast<int>(a.pointCount / 160));
}

ATF_INIT_TEST_CASES(tcs) {
    ATF_ADD_TEST_CASE(tcs, test_thinning_streamed);
    ATF_ADD_TEST_CASE(tcs, test_record_layout);
    ATF_ADD_TEST_CASE(tcs, test_las12_geokeys_rgb);
    ATF_ADD_TEST_CASE(tcs, test_las14_evlr_wkt_rgb);
    ATF_ADD_TEST_CASE(tcs, test_no_crs);
}
