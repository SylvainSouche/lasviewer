// lasviewer — 3D viewer for LiDAR point clouds (LAS/LAZ/COPC) and GeoTIFF
// DEMs, with orthophoto draping. This file only parses the command line; see
// src/viewer_app.h for the application and src/scene.h for loading.
#include "log_capture.h"
#include "raster.h"
#include "viewer_app.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

void printUsage(const char* prog) {
    std::cerr << "Usage: " << prog << " [options] <file>...\n\n"
              << "Files (any number, shown together in one scene):\n"
              << "  *.las, *.laz       point cloud, loaded in full (thinned to 2M points)\n"
              << "  *.copc.laz         point cloud, streamed tile by tile\n"
              << "  *.tif, *.tiff      DEM/DSM elevation raster (Float/Int), or the orthophoto\n"
              << "                     if it is 8-bit RGB(A) imagery\n\n"
              << "Options:\n"
              << "  -o <ortho.tif>     orthophoto used to color points and texture DEMs\n"
              << "  -d <dem.tif>       add a DEM explicitly (needed for 8-bit Terrain-RGB DEMs)\n"
              << "  -dtm, -mnt <f.tif> same as -d: a terrain model, ground of -dhm / -dsm\n"
              << "  -dhm, -mnh <f.tif> height model (height above ground): what stands on the\n"
              << "                     DTM it overlaps, drawn semi-transparent above a height\n"
              << "                     threshold; the DTM is left without orthophoto under it\n"
              << "  -dsm, -mns <f.tif> surface model, used like -dhm (height = DSM - DTM)\n"
              << "  -cop <cloud.laz>   add a point cloud explicitly\n"
              << "  --snapshot <f.ppm> save a frame once loading has settled, then quit\n"
              << "  --dem-lod angle,level  DEM collapse angle (degrees, default 1) and maximum\n"
              << "                     quadtree level (default 5) at load\n"
              << "  --view x,y,z,d,yaw,pitch\n"
              << "                     start looking at world point x,y,z (scene CRS) from d\n"
              << "                     meters away; yaw and pitch in degrees\n"
              << "  -h, --help         this help\n\n"
              << "Examples:\n"
              << "  " << prog << " cloud.copc.laz ortho.tif\n"
              << "  " << prog << " tile1.copc.laz tile2.copc.laz -o ortho.tif\n"
              << "  " << prog << " dem.tif cloud.laz -o ortho.tif\n"
              << "  " << prog << " -mnt mnt.tif -mnh mnh.tif -o ortho.tif\n\n"
              << "Press H in the viewer for the controls.\n";
}

std::string lowerExt(const std::string& path) {
    size_t dot = path.find_last_of('.');
    if (dot == std::string::npos) return {};
    std::string ext = path.substr(dot + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    return ext;
}

// Returns false (after printing why) on invalid arguments.
struct InitialView {
    bool set = false;
    double v[6] = {};
};

bool parseArgs(int argc, char** argv, LoadPlan& plan, std::string& snapshot, InitialView& view) {
    std::vector<std::string> images; // positional TIFFs that look like imagery
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto needValue = [&]() -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "ERROR: " << arg << " needs a file argument\n";
                return nullptr;
            }
            return argv[++i];
        };
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            std::exit(0);
        } else if (arg == "-o") {
            const char* v = needValue();
            if (!v) return false;
            plan.ortho = v;
        } else if (arg == "-d" || arg == "-dtm" || arg == "-mnt") {
            const char* v = needValue();
            if (!v) return false;
            plan.dems.push_back(v);
        } else if (arg == "-dhm" || arg == "-mnh" || arg == "-dsm" || arg == "-mns") {
            const char* v = needValue();
            if (!v) return false;
            bool height = (arg == "-dhm" || arg == "-mnh");
            plan.aboveGround.push_back(
                {v, height ? AboveGroundKind::Height : AboveGroundKind::Surface});
        } else if (arg == "-cop") {
            const char* v = needValue();
            if (!v) return false;
            plan.clouds.push_back(v);
        } else if (arg == "--dem-lod") {
            const char* v = needValue();
            if (!v) return false;
            if (std::sscanf(v, "%lf,%d", &plan.demAngle, &plan.demMaxLevel) != 2 ||
                plan.demAngle <= 0.0 || plan.demMaxLevel < 0 || plan.demMaxLevel > 10) {
                std::cerr << "ERROR: --dem-lod needs angle,level (angle > 0, level 0..10)\n";
                return false;
            }
        } else if (arg == "--view") {
            const char* v = needValue();
            if (!v) return false;
            double* o = view.v;
            if (std::sscanf(v, "%lf,%lf,%lf,%lf,%lf,%lf", &o[0], &o[1], &o[2], &o[3], &o[4],
                            &o[5]) != 6) {
                std::cerr << "ERROR: --view needs x,y,z,distance,yaw,pitch\n";
                return false;
            }
            view.set = true;
        } else if (arg == "--snapshot") {
            const char* v = needValue();
            if (!v) return false;
            snapshot = v;
        } else if (!arg.empty() && arg[0] == '-') {
            std::cerr << "ERROR: unknown option " << arg << "\n";
            return false;
        } else {
            std::string ext = lowerExt(arg);
            if (ext == "tif" || ext == "tiff") {
                RasterInfo info;
                if (readRasterInfo(arg, info) && info.byteImage)
                    images.push_back(arg);
                else
                    plan.dems.push_back(arg);
            } else {
                plan.clouds.push_back(arg);
            }
        }
    }
    if (!images.empty()) {
        if (!plan.ortho.empty() || images.size() > 1) {
            std::cerr << "ERROR: several orthophotos given (";
            for (const auto& im : images) std::cerr << " " << im;
            if (!plan.ortho.empty()) std::cerr << " " << plan.ortho;
            std::cerr << " ) — only one is supported. Use -d for 8-bit Terrain-RGB DEMs.\n";
            return false;
        }
        plan.ortho = images.front();
    }
    if (plan.clouds.empty() && plan.dems.empty()) {
        std::cerr << "ERROR: no point cloud or DEM given\n";
        return false;
    }
    if (!plan.aboveGround.empty() && plan.dems.empty()) {
        std::cerr << "ERROR: -dhm / -dsm need a terrain model (-dtm) to stand on\n";
        return false;
    }
    return true;
}

} // namespace

int main(int argc, char** argv) {
    LoadPlan plan;
    std::string snapshot;
    InitialView view;
    if (!parseArgs(argc, argv, plan, snapshot, view)) {
        std::cerr << "Run " << argv[0] << " -h for help.\n";
        return 1;
    }
    std::cerr << "lasviewer 0.3.0 — LiDAR + DEM + orthophoto 3D viewer" << std::endl;

    LogCapture logCapture; // mirrors std::cerr into the in-app log window
    ViewerApp app;
    app.setSnapshotPath(snapshot);
    if (view.set)
        app.setInitialView(glm::dvec3(view.v[0], view.v[1], view.v[2]), view.v[3], view.v[4],
                           view.v[5]);
    if (!app.init(plan)) return 1;
    app.run();
    return 0;
}
