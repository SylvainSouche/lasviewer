// scene.cpp — scene loading: read every input's header extent first, fix the
// shared frame from their union, then create one layer per input.
#include "scene.h"
#include "copc_layer.h"
#include "dem_layer.h"
#include "geotiff.h"
#include "point_cloud.h"
#include "point_cloud_layer.h"

#include <algorithm>
#include <iostream>
#include <set>

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
    // --- 1. Header extents → shared frame. ---
    struct CloudInput { std::string path; CloudHeader header; };
    struct DemInput { std::string path; WorldBounds bounds; int epsg = 0; };
    std::vector<CloudInput> clouds;
    std::vector<DemInput> dems;
    WorldBounds all;

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
        all.extendXY(b.min.x, b.min.y, b.max.x, b.max.y);
        all.extendZ(b.min.z, b.max.z);
        clouds.push_back(std::move(in));
    }
    for (const std::string& p : plan.dems) {
        DemInput in{p, {}, 0};
        if (!readRasterExtent(p, in.bounds, in.epsg)) {
            std::cerr << "ERROR: DEM is unreadable or not georeferenced: " << p << std::endl;
            continue;
        }
        all.extendXY(in.bounds.min.x, in.bounds.min.y, in.bounds.max.x, in.bounds.max.y);
        dems.push_back(std::move(in));
    }
    if (clouds.empty() && dems.empty()) return false;

    frame = SceneFrame::fromBounds(all);
    std::cerr << "[scene] frame center (" << frame.center.x << ", " << frame.center.y << ", "
              << frame.center.z << "), scale " << frame.scale << " m" << std::endl;

    std::set<int> crs;
    for (const auto& c : clouds) if (c.header.epsg) crs.insert(c.header.epsg);
    for (const auto& d : dems) if (d.epsg) crs.insert(d.epsg);
    if (crs.size() > 1) {
        std::cerr << "WARNING: inputs use different CRSs (";
        for (int e : crs) std::cerr << " EPSG:" << e;
        std::cerr << " ) — layers will not line up; reproject them first" << std::endl;
    }

    // --- 2. Orthophoto (shared). ---
    if (!plan.ortho.empty()) {
        progress("Loading orthophoto");
        ortho = std::make_unique<Orthophoto>();
        if (loadOrthophoto(plan.ortho, *ortho, kMaxOrthoPixels)) {
            orthoPath = plan.ortho;
        } else {
            std::cerr << "ERROR: could not load orthophoto: " << plan.ortho << std::endl;
            ortho.reset();
        }
    }

    // The ortho colors a point cloud only if georeferenced, in the same CRS,
    // and covering enough of it.
    auto orthoForCloud = [&](const CloudInput& c) -> const Orthophoto* {
        if (!ortho) return nullptr;
        if (!ortho->hasGeo) {
            std::cerr << "[ortho] no georeferencing (tags or .tfw) — cannot color "
                      << c.path << std::endl;
            return nullptr;
        }
        if (ortho->epsg && c.header.epsg && ortho->epsg != c.header.epsg) {
            std::cerr << "[ortho] EPSG:" << ortho->epsg << " differs from " << c.path
                      << " (EPSG:" << c.header.epsg << ") — not used for its colors" << std::endl;
            return nullptr;
        }
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
        layer->epsg = d.epsg;
        if (layer->load(frame)) layers.push_back(std::move(layer));
        else std::cerr << "ERROR: could not load DEM: " << d.path << std::endl;
    }
    return !layers.empty();
}
