// scene.cpp — scene loading: read every input's header extent first, fix the
// shared frame from their union, then create one layer per input.
#include "scene.h"
#include "copc_layer.h"
#include "dem_layer.h"
#include "raster.h"
#include "point_cloud.h"
#include "point_cloud_layer.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace {

// Orthophotos larger than this are box-filtered down on load.
constexpr int kMaxOrthoPixels = 64'000'000;
// Minimum fraction of a cloud's XY extent the ortho must cover to color it.
constexpr double kMinOrthoCoverage = 0.25;

bool isCopcPath(const std::string& p) { return p.find(".copc.") != std::string::npos; }

// The LAS/LAZ/COPC readers report unreadable files by throwing; turn that into
// a logged failure.
template <typename F>
bool guarded(const std::string& path, F&& f) {
    try {
        return f();
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << path << ": " << e.what() << std::endl;
        return false;
    }
}

// Area of the intersection of two XY extents.
double overlapArea(const WorldBounds& a, const WorldBounds& b) {
    double w = std::min(a.max.x, b.max.x) - std::max(a.min.x, b.min.x);
    double h = std::min(a.max.y, b.max.y) - std::max(a.min.y, b.min.y);
    return (w > 0 && h > 0) ? w * h : 0.0;
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
    struct DemInput {
        std::string path;
        RasterInfo info;
        WorldBounds bounds;
        AboveGroundKind kind = AboveGroundKind::Height; // above-ground inputs
        int ground = -1; // above-ground inputs: index in `dems`
    };
    std::vector<CloudInput> clouds;
    std::vector<DemInput> dems, above;

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
    for (const LoadPlan::AboveGround& a : plan.aboveGround) {
        DemInput in{a.path, {}, {}, a.kind};
        if (!readRasterInfo(a.path, in.info) || !in.info.hasGeo) {
            std::cerr << "ERROR: raster is unreadable or not georeferenced: " << a.path << std::endl;
            continue;
        }
        double x0, y0, x1, y1;
        rasterExtent(in.info, in.info.width, in.info.height, x0, y0, x1, y1);
        in.bounds.extendXY(x0, y0, x1, y1);
        above.push_back(std::move(in));
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
    // Above-ground rasters: extent in the scene CRS, then the ground they
    // overlap most. Not added to the frame: only their overlap with the
    // ground is drawn.
    for (DemInput& a : above) {
        if (!a.info.wkt.empty() && !sceneWkt.empty() && !sameHorizontalCRS(a.info.wkt, sceneWkt))
            transformExtent(a.bounds, a.info.wkt, sceneWkt);
        double best = 0.0;
        for (size_t i = 0; i < dems.size(); ++i) {
            double area = overlapArea(a.bounds, dems[i].bounds);
            if (area > best) {
                best = area;
                a.ground = static_cast<int>(i);
            }
        }
        if (a.ground < 0) {
            std::cerr << "ERROR: " << a.path << " overlaps no ground model (DTM); skipped"
                      << std::endl;
        } else {
            std::cerr << "[scene] " << a.path << " stands on " << dems[a.ground].path << std::endl;
        }
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
    if (!programs.tess) {
        for (const DemInput& d : dems)
            std::cerr << "ERROR: DEM display needs OpenGL 4.0+ tessellation — skipping "
                      << d.path << std::endl;
        return !layers.empty();
    }

    // Rasters are re-read for every mesh rebuild (see DemSource), so the
    // sources capture paths, not data.
    const std::string wkt = sceneWkt;
    auto kindName = [](AboveGroundKind k) { return k == AboveGroundKind::Height ? "DHM" : "DSM"; };
    std::vector<DemLayer*> grounds(dems.size(), nullptr);
    for (size_t i = 0; i < dems.size(); ++i) {
        const DemInput& d = dems[i];
        struct Cover { std::string path; AboveGroundKind kind; };
        std::vector<Cover> covers;
        for (const DemInput& a : above)
            if (a.ground == static_cast<int>(i)) covers.push_back({a.path, a.kind});
        DemSource source = [path = d.path, wkt, covers](DemSourceData& out) {
            if (!loadDEM(path, out.dem, wkt)) return false;
            if (covers.empty()) return true;
            // Height of the tallest thing standing on each ground pixel.
            out.aux.assign(out.dem.elevations.size(), 0.0f);
            for (const Cover& c : covers) {
                DemRaster r;
                if (!loadDEM(c.path, r, wkt)) continue;
                std::vector<float> h = heightAboveGround(out.dem, r, c.kind);
                for (size_t k = 0; k < h.size(); ++k)
                    if (h[k] > out.aux[k]) out.aux[k] = h[k]; // NaN compares false
            }
            return true;
        };
        progress("Loading " + d.path);
        auto layer = std::make_unique<DemLayer>(d.path, ortho.get(), source,
                                                covers.empty() ? DemRole::Plain : DemRole::Ground,
                                                covers.empty() ? "DEM" : "DTM");
        layer->epsg = d.info.epsg;
        if (plan.demAngle > 0.0 || plan.demMaxLevel >= 0)
            layer->setLod(plan.demAngle > 0.0 ? plan.demAngle : 1.0,
                          plan.demMaxLevel >= 0 ? plan.demMaxLevel : 5);
        if (layer->load(frame)) {
            grounds[i] = layer.get();
            layers.push_back(std::move(layer));
        } else {
            std::cerr << "ERROR: could not load DEM: " << d.path << std::endl;
        }
    }
    for (const DemInput& a : above) {
        if (a.ground < 0 || !grounds[a.ground]) continue;
        DemSource source = [path = a.path, groundPath = dems[a.ground].path, kind = a.kind,
                            wkt](DemSourceData& out) {
            DemRaster ground, top;
            if (!loadDEM(groundPath, ground, wkt) || !loadDEM(path, top, wkt)) return false;
            if (!composeAboveGround(ground, top, kind, out.dem, out.aux)) {
                std::cerr << "ERROR: " << path << " does not overlap its ground " << groundPath
                          << std::endl;
                return false;
            }
            // Unknown ground never occurs where the surface exists; any value
            // keeps the texture free of NaNs.
            for (float& g : out.aux)
                if (std::isnan(g)) g = -10000.0f;
            return true;
        };
        progress("Loading " + a.path);
        auto layer = std::make_unique<DemLayer>(a.path, ortho.get(), source,
                                                DemRole::AboveGround, kindName(a.kind));
        layer->epsg = a.info.epsg;
        if (plan.demAngle > 0.0 || plan.demMaxLevel >= 0)
            layer->setLod(plan.demAngle > 0.0 ? plan.demAngle : 1.0,
                          plan.demMaxLevel >= 0 ? plan.demMaxLevel : 5);
        if (layer->load(frame)) {
            grounds[a.ground]->addAboveLayer(layer.get());
            layers.push_back(std::move(layer));
        } else {
            std::cerr << "ERROR: could not load " << a.path << std::endl;
        }
    }
    return !layers.empty();
}
