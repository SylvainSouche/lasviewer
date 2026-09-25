// scene.cpp — scene loading: read every input's header extent first, fix the
// shared frame from their union, then create one layer per input.
#include "scene.h"
#include "copc_layer.h"
#include "dem_layer.h"
#include "raster.h"
#include "point_cloud.h"
#include "point_cloud_layer.h"

#include <algorithm>
#include <iostream>

namespace {

// Orthophotos larger than this are box-filtered down on load.
constexpr int kMaxOrthoPixels = 64'000'000;
// Minimum fraction of a cloud's XY extent the ortho must cover to color it.
constexpr double kMinOrthoCoverage = 0.25;

bool isCopcPath(const std::string& p) { return p.find(".copc.") != std::string::npos; }

// PDAL reports unreadable files by throwing; turn that into a logged failure.
template <typename F>
bool guarded(const std::string& path, F&& f) {
    try {
        return f();
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << path << ": " << e.what() << std::endl;
        return false;
    }
}

double orthoCoverage(const Orthophoto& o, const WorldBounds& b) {
    double ox0, oy0, ox1, oy1;
    orthoExtent(o, ox0, oy0, ox1, oy1);
    double w = std::max(0.0, std::min(ox1, b.max.x) - std::max(ox0, b.min.x));
    double h = std::max(0.0, std::min(oy1, b.max.y) - std::max(oy0, b.min.y));
    double area = (b.max.x - b.min.x) * (b.max.y - b.min.y);
    return area > 0 ? (w * h) / area : 0.0;
}

} // namespace

Scene::Scene() = default;

Scene::~Scene() {
    layers.clear();
    ortho.reset();
}

GLBounds Scene::bounds(bool visibleOnly) const {
    GLBounds b;
    for (const auto& l : layers)
        if (!visibleOnly || l->visible) b.extend(l->bounds());
    return b;
}

bool Scene::load(const LoadPlan& plan, const Programs& programs,
                 const std::function<void(const std::string&)>& progress) {
    // --- 1. Headers. ---
    struct CloudInput { std::string path; CloudHeader header; };
    struct DemInput { std::string path; RasterInfo info; WorldBounds bounds; };
    std::vector<CloudInput> clouds;
    std::vector<DemInput> dems;

    progress("Reading headers");
    for (const std::string& p : plan.clouds) {
        CloudInput in{p, {}};
        if (!guarded(p, [&] { return readCloudHeader(p, in.header); })) {
            std::cerr << "ERROR: cannot read point cloud header: " << p << std::endl;
            continue;
        }
        const WorldBounds& b = in.header.bounds;
        std::cerr << "[scene] " << p << ": " << in.header.pointCount << " pts, X[" << b.min.x
                  << ", " << b.max.x << "] Y[" << b.min.y << ", " << b.max.y << "] Z["
                  << b.min.z << ", " << b.max.z << "]"
                  << (in.header.epsg ? ", EPSG:" + std::to_string(in.header.epsg) : "")
                  << std::endl;
        clouds.push_back(std::move(in));
    }
    for (const std::string& p : plan.dems) {
        DemInput in{p, {}, {}};
        if (!readRasterInfo(p, in.info) || !in.info.hasGeo) {
            std::cerr << "ERROR: DEM is unreadable or not georeferenced: " << p << std::endl;
            continue;
        }
        double x0, y0, x1, y1;
        rasterExtent(in.info, in.info.width, in.info.height, x0, y0, x1, y1);
        in.bounds.extendXY(x0, y0, x1, y1);
        dems.push_back(std::move(in));
    }
    if (clouds.empty() && dems.empty()) return false;

    // --- 2. Scene CRS: the first input that declares one, point clouds first
    // (they are never reprojected; rasters are warped into this CRS). ---
    std::string sceneWkt;
    for (const auto& c : clouds)
        if (sceneWkt.empty() && !c.header.wkt.empty()) sceneWkt = c.header.wkt;
    for (const auto& d : dems)
        if (sceneWkt.empty() && !d.info.wkt.empty()) sceneWkt = d.info.wkt;
    if (sceneWkt.empty() && !plan.ortho.empty()) {
        RasterInfo oi;
        if (readRasterInfo(plan.ortho, oi)) sceneWkt = oi.wkt;
    }
    int sceneEpsg = horizontalEPSG(sceneWkt);
    std::cerr << "[scene] CRS: "
              << (sceneEpsg ? "EPSG:" + std::to_string(sceneEpsg)
                            : (sceneWkt.empty() ? std::string("unknown") : std::string("custom")))
              << std::endl;

    // --- 3. Extents in the scene CRS → shared frame. ---
    WorldBounds all;
    for (const CloudInput& c : clouds) {
        if (!c.header.wkt.empty() && !sameHorizontalCRS(c.header.wkt, sceneWkt)) {
            std::cerr << "WARNING: " << c.path << " is in another CRS"
                      << (c.header.epsg ? " (EPSG:" + std::to_string(c.header.epsg) + ")" : "")
                      << "; point clouds are not reprojected, it will be misplaced" << std::endl;
        }
        const WorldBounds& b = c.header.bounds;
        all.extendXY(b.min.x, b.min.y, b.max.x, b.max.y);
        all.extendZ(b.min.z, b.max.z);
    }
    for (DemInput& d : dems) {
        if (!d.info.wkt.empty() && !sceneWkt.empty() && !sameHorizontalCRS(d.info.wkt, sceneWkt)) {
            if (!transformExtent(d.bounds, d.info.wkt, sceneWkt)) {
                std::cerr << "WARNING: cannot transform the extent of " << d.path
                          << " into the scene CRS" << std::endl;
            }
        }
        all.extendXY(d.bounds.min.x, d.bounds.min.y, d.bounds.max.x, d.bounds.max.y);
    }
    frame = SceneFrame::fromBounds(all);
    frame.crsWkt = sceneWkt;
    std::cerr << "[scene] frame center (" << frame.center.x << ", " << frame.center.y << ", "
              << frame.center.z << "), scale " << frame.scale << " m" << std::endl;

    // --- 4. Orthophoto (shared), warped into the scene CRS if needed. ---
    if (!plan.ortho.empty()) {
        progress("Loading orthophoto");
        ortho = std::make_unique<Orthophoto>();
        if (loadOrthophoto(plan.ortho, *ortho, kMaxOrthoPixels, sceneWkt)) {
            orthoPath = plan.ortho;
        } else {
            std::cerr << "ERROR: could not load orthophoto: " << plan.ortho << std::endl;
            ortho.reset();
        }
    }

    // The ortho colors a point cloud only if both are placed in the scene
    // CRS and the ortho covers enough of the cloud.
    auto orthoForCloud = [&](const CloudInput& c) -> const Orthophoto* {
        if (!ortho) return nullptr;
        if (!ortho->hasGeo) {
            std::cerr << "[ortho] not georeferenced — cannot color " << c.path << std::endl;
            return nullptr;
        }
        if (!c.header.wkt.empty() && !sameHorizontalCRS(c.header.wkt, sceneWkt)) return nullptr;
        double cov = orthoCoverage(*ortho, c.header.bounds);
        std::cerr << "[ortho] covers " << cov * 100.0 << "% of " << c.path << std::endl;
        if (cov < kMinOrthoCoverage) {
            std::cerr << "[ortho] below " << kMinOrthoCoverage * 100.0
                      << "% — using elevation colors for " << c.path << std::endl;
            return nullptr;
        }
        return ortho.get();
    };

    // --- 3. Layers. ---
    for (const CloudInput& c : clouds) {
        progress("Loading " + c.path);
        const Orthophoto* o = orthoForCloud(c);
        if (isCopcPath(c.path)) {
            auto layer = std::make_unique<CopcLayer>(c.path, c.header.bounds, c.header.pointCount, o);
            layer->epsg = c.header.epsg;
            layer->start(frame);
            layers.push_back(std::move(layer));
        } else {
            auto layer = std::make_unique<PointCloudLayer>(c.path, o);
            layer->epsg = c.header.epsg;
            if (guarded(c.path, [&] { return layer->load(frame); })) layers.push_back(std::move(layer));
            else std::cerr << "ERROR: could not load point cloud: " << c.path << std::endl;
        }
    }
    for (const DemInput& d : dems) {
        if (!programs.tess) {
            std::cerr << "ERROR: DEM display needs OpenGL 4.0+ tessellation — skipping "
                      << d.path << std::endl;
            continue;
        }
        progress("Loading " + d.path);
        auto layer = std::make_unique<DemLayer>(d.path, ortho.get());
        layer->epsg = d.info.epsg;
        if (layer->load(frame)) layers.push_back(std::move(layer));
        else std::cerr << "ERROR: could not load DEM: " << d.path << std::endl;
    }
    return !layers.empty();
}
