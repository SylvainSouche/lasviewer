// render_tool — helper of the render tests (testcases/*.sh): writes
// synthetic DEMs and measures lasviewer --snapshot images.
//
//   render_tool dem <out.tif> terrain|height   synthetic 1025×1025 DEM, 0.5 m
//   render_tool holes <snapshot.ppm>           background pixels enclosed by surface
//   render_tool coverage <snapshot.ppm>        fraction of non-background pixels
//
// The DEMs cover x 1000000..1000512, y 5999488..6000000 (EPSG:2154):
//   terrain  relief at every scale (octaves 200 m .. 3 m) and a 6 m cliff:
//            at a large collapse angle, a mesh with many level transitions;
//   height   0 m except 8 m blocks (buildings) on a 64 m grid: an MNH.
#include "raster.h"

#include <gdal_priv.h>
#include <ogr_spatialref.h>

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

int writeDem(const std::string& path, const std::string& kind) {
    rasterInit();
    std::vector<float> z(static_cast<size_t>(kSize) * kSize);
    for (int r = 0; r < kSize; ++r) {
        for (int c = 0; c < kSize; ++c) {
            double x = c * kPixel, y = r * kPixel;
            float v;
            if (kind == "terrain") {
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
            z[static_cast<size_t>(r) * kSize + c] = v;
        }
    }
    GDALDriver* drv = GetGDALDriverManager()->GetDriverByName("GTiff");
    GDALDataset* ds = drv->Create(path.c_str(), kSize, kSize, 1, GDT_Float32, nullptr);
    if (!ds) return 1;
    double gt[6] = {kOriginX, kPixel, 0.0, kOriginY, 0.0, -kPixel};
    ds->SetGeoTransform(gt);
    OGRSpatialReference srs;
    srs.importFromEPSG(2154);
    ds->SetSpatialRef(&srs);
    ds->GetRasterBand(1)->SetNoDataValue(-9999.0);
    CPLErr err = ds->GetRasterBand(1)->RasterIO(GF_Write, 0, 0, kSize, kSize, z.data(), kSize,
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

double coverage(const Image& img) {
    long surface = 0;
    for (int y = 0; y < img.h; ++y)
        for (int x = 0; x < img.w; ++x) surface += img.background(x, y) ? 0 : 1;
    return static_cast<double>(surface) / (static_cast<double>(img.w) * img.h);
}

int usage() {
    std::cerr << "usage: render_tool dem <out.tif> terrain|height\n"
                 "       render_tool holes <snapshot.ppm>\n"
                 "       render_tool coverage <snapshot.ppm>\n";
    return 2;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3) return usage();
    std::string cmd = argv[1];
    if (cmd == "dem") return argc == 4 ? writeDem(argv[2], argv[3]) : usage();
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
