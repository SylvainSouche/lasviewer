// raster.cpp — raster I/O through GDAL. See raster.h.
#include "raster.h"
#include "point_cloud.h"
#include "scene_frame.h"

#include <cpl_conv.h>
#include <cpl_string.h>
#include <gdal_priv.h>
#include <gdal_utils.h>
#include <ogr_spatialref.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>

#ifndef LASVIEWER_NO_OPENMP
#  ifdef _OPENMP
#    include <omp.h>
#    define LASVIEWER_HAS_OPENMP 1
#  else
#    define LASVIEWER_HAS_OPENMP 0
#  endif
#else
#  define LASVIEWER_HAS_OPENMP 0
#endif

namespace {

struct DatasetCloser {
    void operator()(GDALDataset* d) const { if (d) GDALClose(GDALDataset::ToHandle(d)); }
};
using DatasetPtr = std::unique_ptr<GDALDataset, DatasetCloser>;

struct TransformDeleter {
    void operator()(OGRCoordinateTransformation* t) const {
        OGRCoordinateTransformation::DestroyCT(t);
    }
};

DatasetPtr openRaster(const std::string& path) {
    rasterInit();
    return DatasetPtr(GDALDataset::Open(path.c_str(), GDAL_OF_RASTER | GDAL_OF_READONLY));
}

// Imports a WKT CRS, keeping only its horizontal part, with x=easting /
// longitude axis order.
bool importHorizontal(const std::string& wkt, OGRSpatialReference& srs) {
    if (wkt.empty()) return false;
    srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
    if (srs.importFromWkt(wkt.c_str()) != OGRERR_NONE) return false;
    if (srs.IsCompound()) srs.StripVertical();
    return true;
}

std::string horizontalWkt(const std::string& wkt) {
    OGRSpatialReference srs;
    if (!importHorizontal(wkt, srs)) return {};
    char* out = nullptr;
    const char* opts[] = {"FORMAT=WKT2_2019", nullptr};
    srs.exportToWkt(&out, opts);
    std::string s = out ? out : "";
    CPLFree(out);
    return s;
}

// GDAL geotransform (corner origin) → pixel-center affine.
void setGeoFromTransform(const double gt[6], RasterGeo& g) {
    g.A = gt[1];
    g.B = gt[2];
    g.D = gt[4];
    g.E = gt[5];
    g.C = gt[0] + 0.5 * gt[1] + 0.5 * gt[2];
    g.F = gt[3] + 0.5 * gt[4] + 0.5 * gt[5];
    g.hasGeo = true;
}

void readGeo(GDALDataset& ds, RasterGeo& g) {
    double gt[6];
    bool identity = true;
    if (ds.GetGeoTransform(gt) == CE_None) {
        identity = gt[0] == 0 && gt[1] == 1 && gt[2] == 0 && gt[3] == 0 && gt[4] == 0 &&
                   gt[5] == 1;
        if (!identity) setGeoFromTransform(gt, g);
    }
    if (const OGRSpatialReference* srs = ds.GetSpatialRef()) {
        char* out = nullptr;
        const char* opts[] = {"FORMAT=WKT2_2019", nullptr};
        srs->exportToWkt(&out, opts);
        g.wkt = out ? out : "";
        CPLFree(out);
        g.epsg = horizontalEPSG(g.wkt);
    }
}

bool crsDiffers(const RasterGeo& g, const std::string& sceneWkt) {
    return g.hasGeo && !g.wkt.empty() && !sceneWkt.empty() && !sameHorizontalCRS(g.wkt, sceneWkt);
}

// Rotated/sheared grids are warped to north-up too: the rest of the viewer
// assumes B = D = 0.
bool isRotated(const RasterGeo& g) { return g.hasGeo && (g.B != 0.0 || g.D != 0.0); }

bool needsWarp(const RasterGeo& g, const std::string& sceneWkt) {
    return crsDiffers(g, sceneWkt) || isRotated(g);
}



std::string describeCRS(const RasterGeo& g);

std::string warpReason(const RasterGeo& g, const std::string& sceneWkt) {
    if (crsDiffers(g, sceneWkt)) return "reprojecting from " + describeCRS(g) + " into the scene CRS";
    return "resampling a rotated grid to north-up";
}

std::string describeCRS(const RasterGeo& g) {
    return g.epsg ? "EPSG:" + std::to_string(g.epsg) : (g.wkt.empty() ? "unknown CRS" : "custom CRS");
}

// gdalwarp, bilinear, to a north-up grid in the scene's horizontal CRS
// (or the raster's own CRS when the scene has none / they already match).
DatasetPtr warpInto(GDALDataset* src, const RasterGeo& g, const std::string& sceneWkt,
                    const std::vector<std::string>& extraArgs) {
    CPLStringList argv;
    if (crsDiffers(g, sceneWkt)) {
        argv.AddString("-t_srs");
        argv.AddString(horizontalWkt(sceneWkt).c_str());
    }
    for (const char* a : {"-r", "bilinear", "-multi", "-wo", "NUM_THREADS=ALL_CPUS"})
        argv.AddString(a);
    for (const std::string& a : extraArgs) argv.AddString(a.c_str());
    GDALWarpAppOptions* opts = GDALWarpAppOptionsNew(argv.List(), nullptr);
    if (!opts) return nullptr;
    GDALDatasetH srcH = GDALDataset::ToHandle(src);
    int usageError = 0;
    GDALDatasetH out = GDALWarp("", nullptr, 1, &srcH, opts, &usageError);
    GDALWarpAppOptionsFree(opts);
    return DatasetPtr(GDALDataset::FromHandle(out));
}

} // namespace

// ---------------------------------------------------------------------------
// Setup, CRS helpers
// ---------------------------------------------------------------------------

void rasterInit() {
    static std::once_flag once;
    std::call_once(once, [] { GDALAllRegister(); });
}

int horizontalEPSG(const std::string& wkt) {
    OGRSpatialReference srs;
    if (!importHorizontal(wkt, srs)) return 0;
    const char* name = srs.GetAuthorityName(nullptr);
    const char* code = srs.GetAuthorityCode(nullptr);
    if (!(name && code)) {
        srs.AutoIdentifyEPSG();
        name = srs.GetAuthorityName(nullptr);
        code = srs.GetAuthorityCode(nullptr);
    }
    if (name && code && EQUAL(name, "EPSG")) return std::atoi(code);
    return 0;
}

bool sameHorizontalCRS(const std::string& wktA, const std::string& wktB) {
    OGRSpatialReference a, b;
    if (!importHorizontal(wktA, a) || !importHorizontal(wktB, b)) return false;
    const char* opts[] = {"IGNORE_DATA_AXIS_TO_SRS_AXIS_MAPPING=YES",
                          "CRITERION=EQUIVALENT_EXCEPT_AXIS_ORDER_GEOGCRS", nullptr};
    return a.IsSame(&b, opts);
}

bool transformExtent(WorldBounds& b, const std::string& fromWkt, const std::string& toWkt) {
    OGRSpatialReference from, to;
    if (!importHorizontal(fromWkt, from) || !importHorizontal(toWkt, to)) return false;
    std::unique_ptr<OGRCoordinateTransformation, TransformDeleter> ct(
        OGRCreateCoordinateTransformation(&from, &to));
    if (!ct) return false;
    // Sample every edge densely: a reprojected rectangle's bounding box is
    // not the box of its four transformed corners.
    const int n = 21;
    std::vector<double> xs, ys;
    for (int i = 0; i < n; ++i) {
        double t = static_cast<double>(i) / (n - 1);
        double x = b.min.x + t * (b.max.x - b.min.x);
        double y = b.min.y + t * (b.max.y - b.min.y);
        xs.insert(xs.end(), {x, x, b.min.x, b.max.x});
        ys.insert(ys.end(), {b.min.y, b.max.y, y, y});
    }
    std::vector<int> ok(xs.size());
    if (!ct->Transform(static_cast<int>(xs.size()), xs.data(), ys.data(), nullptr, ok.data()))
        return false;
    WorldBounds out;
    out.min.z = b.min.z;
    out.max.z = b.max.z;
    out.hasZ = b.hasZ;
    for (size_t i = 0; i < xs.size(); ++i)
        if (ok[i]) out.extendXY(xs[i], ys[i], xs[i], ys[i]);
    if (!out.valid()) return false;
    b = out;
    return true;
}

// ---------------------------------------------------------------------------
// Info, extents
// ---------------------------------------------------------------------------

bool readRasterInfo(const std::string& path, RasterInfo& out) {
    DatasetPtr ds = openRaster(path);
    if (!ds) return false;
    out.width = ds->GetRasterXSize();
    out.height = ds->GetRasterYSize();
    out.bands = ds->GetRasterCount();
    if (out.bands == 0) return false;
    readGeo(*ds, out);
    out.byteImage = out.bands >= 3 && ds->GetRasterBand(1)->GetRasterDataType() == GDT_Byte;
    return true;
}

void rasterExtent(const RasterGeo& g, int width, int height,
                  double& minX, double& minY, double& maxX, double& maxY) {
    minX = minY = std::numeric_limits<double>::max();
    maxX = maxY = std::numeric_limits<double>::lowest();
    const double cols[2] = {0.0, static_cast<double>(width - 1)};
    const double rows[2] = {0.0, static_cast<double>(height - 1)};
    for (double c : cols) {
        for (double r : rows) {
            double x = g.C + g.A * c + g.B * r;
            double y = g.F + g.D * c + g.E * r;
            minX = std::min(minX, x); maxX = std::max(maxX, x);
            minY = std::min(minY, y); maxY = std::max(maxY, y);
        }
    }
}

void orthoExtent(const Orthophoto& o, double& minX, double& minY, double& maxX, double& maxY) {
    rasterExtent(o, o.width, o.height, minX, minY, maxX, maxY);
}

// ---------------------------------------------------------------------------
// Orthophoto
// ---------------------------------------------------------------------------

bool loadOrthophoto(const std::string& path, Orthophoto& ortho, int maxPixels,
                    const std::string& sceneWkt) {
    std::cerr << "[raster] opening orthophoto: " << path << std::endl;
    DatasetPtr ds = openRaster(path);
    if (!ds || ds->GetRasterCount() == 0) {
        std::cerr << "ERROR: GDAL cannot read " << path << std::endl;
        return false;
    }
    readGeo(*ds, ortho);
    GDALDataset* src = ds.get();
    DatasetPtr warped;
    if (needsWarp(ortho, sceneWkt)) {
        // Lazy warped VRT: pixels are warped as they are read below (from the
        // source's overviews when it has them).
        std::string reason = warpReason(ortho, sceneWkt);
        warped = warpInto(src, ortho, sceneWkt, {"-of", "VRT", "-dstalpha"});
        if (!warped) {
            std::cerr << "ERROR: " << path << ": failed " << reason << std::endl;
            return false;
        }
        std::cerr << "[raster] orthophoto: " << reason << std::endl;
        src = warped.get();
        ortho = Orthophoto{};
        readGeo(*src, ortho);
        ortho.reprojected = true;
    }

    const int srcW = src->GetRasterXSize(), srcH = src->GetRasterYSize();
    const int bands = src->GetRasterCount();
    double scale = std::min(1.0, std::sqrt(static_cast<double>(maxPixels) /
                                           (static_cast<double>(srcW) * srcH)));
    int dstW = std::max(1, static_cast<int>(srcW * scale));
    int dstH = std::max(1, static_cast<int>(srcH * scale));

    int colorBands[3] = {1, 1, 1};
    if (bands >= 3) { colorBands[1] = 2; colorBands[2] = 3; }
    int alphaBand = 0;
    for (int b = 1; b <= bands; ++b)
        if (src->GetRasterBand(b)->GetColorInterpretation() == GCI_AlphaBand) alphaBand = b;
    if (src->GetRasterBand(1)->GetRasterDataType() != GDT_Byte) {
        std::cerr << "[raster] WARNING: " << path << " is not 8-bit; values are clamped to 0-255"
                  << std::endl;
    }

    ortho.width = dstW;
    ortho.height = dstH;
    ortho.pixels.assign(static_cast<size_t>(dstW) * dstH * 4, 255);
    GDALRasterIOExtraArg extra;
    INIT_RASTERIO_EXTRA_ARG(extra);
    extra.eResampleAlg = (dstW < srcW) ? GRIORA_Average : GRIORA_NearestNeighbour;
    CPLErr err = src->RasterIO(GF_Read, 0, 0, srcW, srcH, ortho.pixels.data(), dstW, dstH,
                               GDT_Byte, 3, colorBands, 4, static_cast<GSpacing>(dstW) * 4, 1,
                               &extra);
    if (err == CE_None && alphaBand) {
        err = src->GetRasterBand(alphaBand)->RasterIO(GF_Read, 0, 0, srcW, srcH,
                                                     ortho.pixels.data() + 3, dstW, dstH,
                                                     GDT_Byte, 4, static_cast<GSpacing>(dstW) * 4,
                                                     &extra);
    }
    if (err != CE_None) {
        std::cerr << "ERROR: reading pixels of " << path << " failed" << std::endl;
        return false;
    }

    // The affine of the resampled grid (pixel-center convention).
    double gt[6];
    if (ortho.hasGeo && src->GetGeoTransform(gt) == CE_None) {
        double sx = static_cast<double>(srcW) / dstW, sy = static_cast<double>(srcH) / dstH;
        gt[1] *= sx; gt[4] *= sx;
        gt[2] *= sy; gt[5] *= sy;
        setGeoFromTransform(gt, ortho);
    }
    std::cerr << "[raster] orthophoto " << srcW << "x" << srcH;
    if (dstW != srcW) std::cerr << " -> " << dstW << "x" << dstH;
    std::cerr << ", " << (ortho.hasGeo ? describeCRS(ortho) : "not georeferenced") << std::endl;
    return true;
}

// ---------------------------------------------------------------------------
// DEM
// ---------------------------------------------------------------------------

bool DemRaster::isNodata(float v) const {
    if (std::isnan(v)) return true;
    if (hasNodata) {
        if (std::isnan(nodata)) return false;
        float tol = 1e-3f * std::max(1.0f, std::abs(nodata));
        return std::abs(v - nodata) < tol;
    }
    return v < -9000.0f;
}

bool loadDEM(const std::string& path, DemRaster& dem, const std::string& sceneWkt) {
    DatasetPtr ds = openRaster(path);
    if (!ds || ds->GetRasterCount() == 0) {
        std::cerr << "ERROR: GDAL cannot read DEM " << path << std::endl;
        return false;
    }
    readGeo(*ds, dem);
    if (!dem.hasGeo) {
        std::cerr << "ERROR: DEM " << path << " is not georeferenced" << std::endl;
        return false;
    }
    int w = ds->GetRasterXSize(), h = ds->GetRasterYSize();
    GDALRasterBand* band1 = ds->GetRasterBand(1);
    dem.terrainRGB = ds->GetRasterCount() >= 3 && band1->GetRasterDataType() == GDT_Byte;

    GDALDataset* src = ds.get();
    DatasetPtr decoded; // Terrain RGB decoded to float, when it must be warped
    if (dem.terrainRGB) {
        // IGN MNS LiDAR HD encoding: h = (R*65536 + G*256 + B) * 0.1 - 10000.
        std::cerr << "[raster] Terrain RGB DEM, decoding" << std::endl;
        std::vector<uint8_t> rgb(static_cast<size_t>(w) * h * 3);
        int bandMap[3] = {1, 2, 3};
        if (ds->RasterIO(GF_Read, 0, 0, w, h, rgb.data(), w, h, GDT_Byte, 3, bandMap, 3,
                         static_cast<GSpacing>(w) * 3, 1, nullptr) != CE_None) {
            std::cerr << "ERROR: reading " << path << " failed" << std::endl;
            return false;
        }
        dem.elevations.resize(static_cast<size_t>(w) * h);
        for (size_t i = 0; i < dem.elevations.size(); ++i) {
            float v = (rgb[i * 3] * 65536.0f + rgb[i * 3 + 1] * 256.0f + rgb[i * 3 + 2]) * 0.1f -
                      10000.0f;
            dem.elevations[i] = (v < -9000.0f) ? -9999.0f : v;
        }
        dem.hasNodata = true;
        dem.nodata = -9999.0f;
        if (needsWarp(dem, sceneWkt)) {
            GDALDriver* mem = GetGDALDriverManager()->GetDriverByName("MEM");
            decoded.reset(mem->Create("", w, h, 1, GDT_Float32, nullptr));
            double gt[6];
            ds->GetGeoTransform(gt);
            decoded->SetGeoTransform(gt);
            decoded->SetSpatialRef(ds->GetSpatialRef());
            GDALRasterBand* b = decoded->GetRasterBand(1);
            b->SetNoDataValue(-9999.0);
            if (b->RasterIO(GF_Write, 0, 0, w, h, dem.elevations.data(), w, h, GDT_Float32, 0,
                            0, nullptr) != CE_None) {
                std::cerr << "ERROR: preparing " << path << " for reprojection failed"
                          << std::endl;
                return false;
            }
            src = decoded.get();
        }
    } else {
        int has = 0;
        double nd = band1->GetNoDataValue(&has);
        dem.hasNodata = has != 0;
        dem.nodata = static_cast<float>(nd);
    }

    DatasetPtr warped;
    if (needsWarp(dem, sceneWkt)) {
        std::string nd = dem.hasNodata ? CPLSPrintf("%.9g", dem.nodata) : "-9999";
        std::string reason = warpReason(dem, sceneWkt);
        warped = warpInto(src, dem, sceneWkt, {"-of", "MEM", "-ot", "Float32", "-dstnodata", nd});
        if (!warped) {
            std::cerr << "ERROR: DEM " << path << ": failed " << reason << std::endl;
            return false;
        }
        std::cerr << "[raster] DEM: " << reason << std::endl;
        src = warped.get();
        RasterGeo g;
        readGeo(*src, g);
        static_cast<RasterGeo&>(dem) = g;
        dem.reprojected = true;
        dem.hasNodata = true;
        dem.nodata = static_cast<float>(std::atof(nd.c_str()));
        w = src->GetRasterXSize();
        h = src->GetRasterYSize();
        dem.elevations.clear();
    }

    if (dem.elevations.empty()) {
        dem.elevations.resize(static_cast<size_t>(w) * h);
        if (src->GetRasterBand(1)->RasterIO(GF_Read, 0, 0, w, h, dem.elevations.data(), w, h,
                                            GDT_Float32, 0, 0, nullptr) != CE_None) {
            std::cerr << "ERROR: reading DEM " << path << " failed" << std::endl;
            return false;
        }
    }
    dem.width = w;
    dem.height = h;
    std::cerr << "[raster] DEM " << w << "x" << h << ", " << describeCRS(dem) << ", nodata "
              << (dem.hasNodata ? CPLSPrintf("%g", dem.nodata) : "not declared (< -9000 assumed)")
              << std::endl;
    return true;
}

// ---------------------------------------------------------------------------
// Sampling and point colorization
// ---------------------------------------------------------------------------

glm::vec3 sampleOrthoBilinear(const Orthophoto& ortho, double col, double row) {
    if (col < 0 || col >= ortho.width || row < 0 || row >= ortho.height) {
        return {0.35f, 0.35f, 0.35f}; // mid-dark grey outside the image
    }
    int x0 = static_cast<int>(col);
    int y0 = static_cast<int>(row);
    int x1 = std::min(x0 + 1, ortho.width - 1);
    int y1 = std::min(y0 + 1, ortho.height - 1);
    float fx = static_cast<float>(col - x0);
    float fy = static_cast<float>(row - y0);
    auto s = [&](int x, int y) -> glm::vec3 {
        size_t i = (static_cast<size_t>(y) * ortho.width + x) * 4;
        return {ortho.pixels[i + 0] / 255.0f, ortho.pixels[i + 1] / 255.0f,
                ortho.pixels[i + 2] / 255.0f};
    };
    glm::vec3 c00 = s(x0, y0), c10 = s(x1, y0), c01 = s(x0, y1), c11 = s(x1, y1);
    return c00 * (1 - fx) * (1 - fy) + c10 * fx * (1 - fy) + c01 * (1 - fx) * fy +
           c11 * fx * fy;
}

void colorizeFromOrthophoto(PointCloud& cloud, const Orthophoto& ortho) {
    if (ortho.pixels.empty() || !ortho.hasGeo || cloud.pointCount == 0) return;
    std::cerr << "[colorize] sampling orthophoto for " << cloud.pointCount << " points..."
              << std::endl;
    cloud.orthoColors.resize(cloud.pointCount * 3);

    // Inverse affine, world (X, Y) → pixel (col, row), folded together with
    // the GL transform (GL_Z = -(worldY - center.y) / scale).
    double det = ortho.A * ortho.E - ortho.B * ortho.D;
    if (std::abs(det) < 1e-15) return;
    double invA = ortho.E / det, invB = -ortho.B / det;
    double invD = -ortho.D / det, invE = ortho.A / det;
    double offsetX = cloud.worldCenter.x - ortho.C;
    double offsetY = cloud.worldCenter.y - ortho.F;
    double scale = cloud.worldScale;
    double kColX = invA * scale, kColZ = -invB * scale;
    double kCol0 = invA * offsetX + invB * offsetY;
    double kRowX = invD * scale, kRowZ = -invE * scale;
    double kRow0 = invD * offsetX + invE * offsetY;

    std::atomic<size_t> outOfBounds{0};
#if LASVIEWER_HAS_OPENMP
    int nThreads = std::max(1, std::min(static_cast<int>(cloud.pointCount / 100000),
                                        static_cast<int>(std::thread::hardware_concurrency())));
    #pragma omp parallel for num_threads(nThreads) schedule(static)
#endif
    for (size_t i = 0; i < cloud.pointCount; ++i) {
        double gx = cloud.positions[i * 3 + 0];
        double gz = cloud.positions[i * 3 + 2];
        double col = kColX * gx + kColZ * gz + kCol0;
        double row = kRowX * gx + kRowZ * gz + kRow0;
        glm::vec3 c = sampleOrthoBilinear(ortho, col, row);
        cloud.orthoColors[i * 3 + 0] = c.r;
        cloud.orthoColors[i * 3 + 1] = c.g;
        cloud.orthoColors[i * 3 + 2] = c.b;
        if (col < 0 || col >= ortho.width || row < 0 || row >= ortho.height)
            outOfBounds.fetch_add(1, std::memory_order_relaxed);
    }
    cloud.hasOrthoColors = true;
    std::cerr << "[colorize] done (" << outOfBounds << " / " << cloud.pointCount
              << " outside orthophoto extent)" << std::endl;
}
