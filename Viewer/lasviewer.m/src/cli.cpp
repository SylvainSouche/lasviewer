// cli.cpp — see cli.h.
#include "cli.h"

#include "raster.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <vector>

namespace {

std::string lowerExt(const std::string& path) {
    size_t dot = path.find_last_of('.');
    if (dot == std::string::npos) return {};
    std::string ext = path.substr(dot + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return ext;
}

// n comma-separated numbers, nothing else ("1,5x" is rejected).
bool parseNumbers(const char* s, double* out, int n) {
    int pos = 0;
    for (int k = 0; k < n; ++k) {
        int used = 0;
        if (std::sscanf(s + pos, k == 0 ? "%lf%n" : ",%lf%n", &out[k], &used) != 1) return false;
        pos += used;
    }
    return s[pos] == '\0';
}

} // namespace

void printUsage(const char* prog, std::ostream& out) {
    out << "lasviewer " << kLasviewerVersion << " — LiDAR + DEM + orthophoto 3D viewer\n\n"
        << "Usage: " << prog << " [options] <file>...\n\n"
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

CliStatus parseCommandLine(int argc, const char* const* argv, CommandLine& cmd,
                           std::ostream& outStream, std::ostream& err) {
    LoadPlan& plan = cmd.plan;
    std::vector<std::string> images; // positional TIFFs that look like imagery
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto needValue = [&]() -> const char* {
            if (i + 1 >= argc) {
                err << "ERROR: " << arg << " needs a value\n";
                return nullptr;
            }
            return argv[++i];
        };
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0], outStream);
            return CliStatus::Help;
        } else if (arg == "-o") {
            const char* v = needValue();
            if (!v) return CliStatus::Error;
            plan.ortho = v;
        } else if (arg == "-d" || arg == "-dtm" || arg == "-mnt") {
            const char* v = needValue();
            if (!v) return CliStatus::Error;
            plan.dems.push_back(v);
        } else if (arg == "-dhm" || arg == "-mnh" || arg == "-dsm" || arg == "-mns") {
            const char* v = needValue();
            if (!v) return CliStatus::Error;
            bool height = (arg == "-dhm" || arg == "-mnh");
            plan.aboveGround.push_back(
                {v, height ? AboveGroundKind::Height : AboveGroundKind::Surface});
        } else if (arg == "-cop") {
            const char* v = needValue();
            if (!v) return CliStatus::Error;
            plan.clouds.push_back(v);
        } else if (arg == "--dem-lod") {
            const char* v = needValue();
            if (!v) return CliStatus::Error;
            double lod[2];
            if (!parseNumbers(v, lod, 2) || lod[0] <= 0.0 || lod[1] < 0 || lod[1] > 10 ||
                lod[1] != static_cast<int>(lod[1])) {
                err << "ERROR: --dem-lod needs angle,level (angle > 0, level 0..10)\n";
                return CliStatus::Error;
            }
            plan.demAngle = lod[0];
            plan.demMaxLevel = static_cast<int>(lod[1]);
        } else if (arg == "--view") {
            const char* v = needValue();
            if (!v) return CliStatus::Error;
            if (!parseNumbers(v, cmd.view, 6) || cmd.view[3] <= 0.0) {
                err << "ERROR: --view needs x,y,z,distance,yaw,pitch (distance > 0)\n";
                return CliStatus::Error;
            }
            cmd.viewSet = true;
        } else if (arg == "--snapshot") {
            const char* v = needValue();
            if (!v) return CliStatus::Error;
            cmd.snapshot = v;
        } else if (!arg.empty() && arg[0] == '-') {
            err << "ERROR: unknown option " << arg << "\n";
            return CliStatus::Error;
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
            err << "ERROR: several orthophotos given (";
            for (const auto& im : images) err << " " << im;
            if (!plan.ortho.empty()) err << " " << plan.ortho;
            err << " ) — only one is supported. Use -d for 8-bit Terrain-RGB DEMs.\n";
            return CliStatus::Error;
        }
        plan.ortho = images.front();
    }
    if (plan.clouds.empty() && plan.dems.empty()) {
        err << "ERROR: no point cloud or DEM given\n";
        return CliStatus::Error;
    }
    if (!plan.aboveGround.empty() && plan.dems.empty()) {
        err << "ERROR: -dhm / -dsm need a terrain model (-dtm) to stand on\n";
        return CliStatus::Error;
    }
    return CliStatus::Run;
}
