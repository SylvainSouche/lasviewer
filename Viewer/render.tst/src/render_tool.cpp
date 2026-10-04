// render_tool — helper of the render tests (testcases/*.sh): writes
// synthetic DEMs and measures lasviewer --snapshot images.
//
//   render_tool dem <out.tif> terrain|slope|height   synthetic 1025×1025 DEM, 0.5 m
//   render_tool holes <snapshot.ppm>           background pixels enclosed by surface
//   render_tool coverage <snapshot.ppm>        fraction of non-background pixels
//   render_tool diff <a.ppm> <b.ppm>           mean absolute difference (0..255)
//   (dem: an optional west|east picks one half, as two adjacent tiles)
//
// The DEMs cover x 1000000..1000512, y 5999488..6000000 (EPSG:2154):
//   terrain  relief at every scale (octaves 200 m .. 3 m) and a 6 m cliff:
//            at a large collapse angle, a mesh with many level transitions;
//   slope    gentle 200 m hills on a 20 % regional rise eastward: its west
//            and east halves have clearly different elevation ranges (like a
//            valley tile next to a hillside tile), and no fine relief;
//   height   0 m except 8 m blocks (buildings) on a 64 m grid: an MNH.
#include "raster.h"

#include <gdal_priv.h>
#include <ogr_spatialref.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

namespace {

constexpr int kSize = 1025;
constexpr double kPixel = 0.5;
constexpr double kOriginX = 1000000.0, kOriginY = 6000000.0;

// part: "full", or the "west" / "east" half (columns 0..512 / 512..1024: the
// two tiles share the middle column, like adjacent DEM tiles).
int writeDem(const std::string& path, const std::string& kind, const std::string& part) {
    int c0 = 0, c1 = kSize - 1;
    if (part == "west")
        c1 = kSize / 2;
    else if (part == "east")
        c0 = kSize / 2;
    else if (part != "full") {
        std::cerr << "render_tool dem: unknown part " << part << "\n";
        return 2;
    }
    const int width = c1 - c0 + 1;
    rasterInit();
    std::vector<float> z(static_cast<size_t>(width) * kSize);
    for (int r = 0; r < kSize; ++r) {
        for (int c = c0; c <= c1; ++c) {
            double x = c * kPixel, y = r * kPixel;
            float v;
            if (kind == "slope") {
                // Smooth (no fine relief, whose hill-shading would alias
                // differently in one tile and in two): gentle 200 m hills on
                // a 20 % regional rise eastward.
                v = static_cast<float>(100.0 + 0.2 * x +
                                       10.0 * std::sin(x * 6.2832 / 200.0) *
                                           std::cos(y * 6.2832 / 260.0));
            } else if (kind == "terrain") {
                // Relief at every scale, like real terrain: octaves from
                // 200 m down to 3 m, amplitude proportional to wavelength.
                double h = 100.0;
                for (double wl = 200.0; wl >= 3.0; wl /= 2.0)
                    h += 0.06 * wl * std::sin(x * 6.2832 / wl + wl) *
                         std::cos(y * 6.2832 / (wl * 1.3) + 2 * wl);
                v = static_cast<float>(h + (x > 300.0 ? 6.0 : 0.0));
            } else if (kind == "height") {
                bool block = std::fmod(x, 64.0) > 20.0 && std::fmod(x, 64.0) < 40.0 &&
                             std::fmod(y, 64.0) > 20.0 && std::fmod(y, 64.0) < 40.0;
                v = block ? 8.0f : 0.0f;
            } else {
                std::cerr << "render_tool dem: unknown kind " << kind << "\n";
                return 2;
            }
            z[static_cast<size_t>(r) * width + (c - c0)] = v;
        }
    }
    GDALDriver* drv = GetGDALDriverManager()->GetDriverByName("GTiff");
    GDALDataset* ds = drv->Create(path.c_str(), width, kSize, 1, GDT_Float32, nullptr);
    if (!ds) return 1;
    double gt[6] = {kOriginX + c0 * kPixel, kPixel, 0.0, kOriginY, 0.0, -kPixel};
    ds->SetGeoTransform(gt);
    OGRSpatialReference srs;
    srs.importFromEPSG(2154);
    ds->SetSpatialRef(&srs);
    ds->GetRasterBand(1)->SetNoDataValue(-9999.0);
    CPLErr err = ds->GetRasterBand(1)->RasterIO(GF_Write, 0, 0, width, kSize, z.data(), width,
                                                kSize, GDT_Float32, 0, 0, nullptr);
    GDALClose(GDALDataset::ToHandle(ds));
    return err == CE_None ? 0 : 1;
}

struct Image {
    int w = 0, h = 0;
    std::vector<unsigned char> rgb;
    // The viewer's clear colour (0.10, 0.11, 0.13).
    bool background(int x, int y) const {
        const unsigned char* p = &rgb[(static_cast<size_t>(y) * w + x) * 3];
        return std::abs(p[0] - 26) <= 2 && std::abs(p[1] - 28) <= 2 && std::abs(p[2] - 33) <= 2;
    }
};

bool readPpm(const std::string& path, Image& img) {
    std::ifstream f(path, std::ios::binary);
    std::string magic;
    int maxval = 0;
    if (!(f >> magic >> img.w >> img.h >> maxval) || magic != "P6" || maxval != 255) return false;
    f.get();
    img.rgb.resize(static_cast<size_t>(img.w) * img.h * 3);
    return static_cast<bool>(f.read(reinterpret_cast<char*>(img.rgb.data()),
                                    static_cast<std::streamsize>(img.rgb.size())));
}

// A background pixel with surface on both sides, horizontally and
// vertically, within kRadius: seen from below the terrain, a hole through
// the mesh. The UI panel (left) is skipped.
int countHoles(const Image& img) {
    const int kRadius = 6, kLeft = img.w * 380 / 1400;
    int holes = 0;
    auto surfaceAlong = [&](int x, int y, int dx, int dy) {
        for (int k = 1; k <= kRadius; ++k)
            if (!img.background(x + dx * k, y + dy * k)) return true;
        return false;
    };
    for (int y = kRadius; y < img.h - kRadius; ++y)
        for (int x = std::max(kLeft, kRadius); x < img.w - kRadius; ++x)
            if (img.background(x, y) && surfaceAlong(x, y, -1, 0) && surfaceAlong(x, y, 1, 0) &&
                surfaceAlong(x, y, 0, -1) && surfaceAlong(x, y, 0, 1))
                ++holes;
    return holes;
}

// Mean absolute difference (0..255 per channel) between two snapshots,
// compared on a coarse grid of block averages in normalised coordinates
// (64 × 40 blocks over the 3D view; the UI panel on the left is skipped),
// using only blocks that are entirely surface in both images. Snapshot sizes
// vary by a few pixels between runs (window placement), so pixel-by-pixel
// comparison is meaningless, and blocks on the outline of the surface would
// count that misalignment rather than colours.
double meanDifference(const Image& a, const Image& b) {
    constexpr int kBx = 64, kBy = 40;
    struct Blocks {
        std::vector<double> rgb;
        std::vector<long> count, background;
    };
    auto blocks = [&](const Image& img) {
        Blocks out{std::vector<double>(static_cast<size_t>(kBx) * kBy * 3, 0.0),
                   std::vector<long>(static_cast<size_t>(kBx) * kBy, 0),
                   std::vector<long>(static_cast<size_t>(kBx) * kBy, 0)};
        const int left = img.w * 380 / 1400;
        for (int y = 0; y < img.h; ++y)
            for (int x = left; x < img.w; ++x) {
                int bx = (x - left) * kBx / (img.w - left), by = y * kBy / img.h;
                size_t k = static_cast<size_t>(by) * kBx + bx;
                const unsigned char* p = &img.rgb[(static_cast<size_t>(y) * img.w + x) * 3];
                for (int c = 0; c < 3; ++c) out.rgb[k * 3 + c] += p[c];
                ++out.count[k];
                if (img.background(x, y)) ++out.background[k];
            }
        for (size_t k = 0; k < out.count.size(); ++k)
            for (int c = 0; c < 3; ++c) out.rgb[k * 3 + c] /= std::max(out.count[k], 1L);
        return out;
    };
    Blocks ba = blocks(a), bb = blocks(b);
    double sum = 0.0;
    long n = 0;
    for (size_t k = 0; k < ba.count.size(); ++k) {
        if (ba.background[k] || bb.background[k]) continue;
        for (int c = 0; c < 3; ++c, ++n) sum += std::abs(ba.rgb[k * 3 + c] - bb.rgb[k * 3 + c]);
    }
    return n ? sum / static_cast<double>(n) : -1.0;
}

double coverage(const Image& img) {
    long surface = 0;
    for (int y = 0; y < img.h; ++y)
        for (int x = 0; x < img.w; ++x) surface += img.background(x, y) ? 0 : 1;
    return static_cast<double>(surface) / (static_cast<double>(img.w) * img.h);
}

int usage() {
    std::cerr << "usage: render_tool dem <out.tif> terrain|slope|height [full|west|east]\n"
                 "       render_tool holes <snapshot.ppm>\n"
                 "       render_tool coverage <snapshot.ppm>\n"
                 "       render_tool diff <a.ppm> <b.ppm>\n";
    return 2;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3) return usage();
    std::string cmd = argv[1];
    if (cmd == "dem")
        return argc == 4 || argc == 5 ? writeDem(argv[2], argv[3], argc == 5 ? argv[4] : "full")
                                      : usage();
    if (cmd == "diff") {
        Image a, b;
        if (argc != 4 || !readPpm(argv[2], a) || !readPpm(argv[3], b)) return usage();
        std::printf("%.2f\n", meanDifference(a, b));
        return 0;
    }
    Image img;
    if (!readPpm(argv[2], img)) {
        std::cerr << "render_tool: cannot read " << argv[2] << "\n";
        return 1;
    }
    if (cmd == "holes")
        std::printf("%d\n", countHoles(img));
    else if (cmd == "coverage")
        std::printf("%.3f\n", coverage(img));
    else
        return usage();
    return 0;
}
