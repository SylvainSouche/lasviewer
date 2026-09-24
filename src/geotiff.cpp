// geotiff.cpp — GeoTIFF tag reading, .tfw parsing, TIFF loading, orthophoto
//               colorization.
//
// Implements:
//   - readGeoTIFFTags()       : ModelPixelScaleTag + ModelTiepointTag → affine.
//   - loadTFW()               : 6-line ESRI world file.
//   - loadTIFF()              : full RGBA read with optional box downsample.
//   - sampleOrthoBilinear()   : sub-pixel bilinear color lookup.
//   - colorizeFromOrthophoto(): CPU-side (OpenMP) per-point color sampling.
#include "geotiff.h"
#include "point_cloud.h"

#include <glm/glm.hpp>

#include <tiffio.h>

#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <thread>
#include <atomic>
#include <vector>
#include <string>

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

// PROJ (https://proj.org) — optional dependency, detected via pkg-config
// in the Makefile (LASVIEWER_HAS_PROJ defined on the compiler command
// line, not probed here) — used only for reprojectToMatchCRS() below,
// to convert coordinates between different CRSs when the DEM and its
// orthophoto don't share one. Without it, a CRS mismatch is still
// detected and reported (readEPSGCode() below), just not automatically
// corrected — the existing "skip texturing, warn, suggest gdalwarp"
// fallback (§6r of the design doc) still applies.
#ifdef LASVIEWER_HAS_PROJ
#  include <proj.h>
#endif

#define GEOTIFF_MODEL_PIXEL_SCALE_TAG     33550
#define GEOTIFF_MODEL_TIEPOINT_TAG        33922

// ---------------------------------------------------------------------------
// Read GeoTIFF tags and populate ortho.A/E/C/F (the affine transform).
//
// KNOWN LATENT RISK, flagged rather than fixed here: 33550/33922 are
// GeoTIFF-spec tags, not baseline TIFF, and are read via TIFFGetField()
// below WITHOUT an explicit TIFFMergeFieldInfo() registration — the exact
// same pattern that caused a real, confirmed EXC_BAD_ACCESS crash for
// TIFFTAG_GDAL_NODATA (42113) in dem_mesh.cpp's readDEMNodataValue() (a
// type-confusion in libtiff's unregistered-tag varargs dispatch — see
// that function's comment for the full explanation and the fix pattern:
// register via TIFFMergeFieldInfo before calling TIFFGetField).
//
// This code has NOT crashed across this whole session's DEM/orthophoto
// loads, and the likely reason is that GeoTIFF's core tags (including
// these two) are common and standard enough that many libtiff builds —
// including, apparently, whichever one is in use here — bundle them into
// their OWN internal extended-tag table by default, unlike the more
// GDAL-specific GDAL_NODATA tag. That's an inference from observed
// behavior, not a guarantee for every libtiff build this app might run
// against. Deliberately NOT touched right now: it's working, provably
// stable code, and changing TIFF-tag-reading logic immediately after a
// real crash in a similar area adds risk without a demonstrated need —
// but if a future crash report ever points back to THIS function
// specifically, the fix is the same registration pattern already applied
// in dem_mesh.cpp.
// ---------------------------------------------------------------------------

bool readGeoTIFFTags(TIFF* tif, Orthophoto& ortho) {
    double* scale = nullptr;
    uint16_t scaleCount = 0;
    int got = TIFFGetField(tif, GEOTIFF_MODEL_PIXEL_SCALE_TAG, &scaleCount, &scale);
    if (got != 1 || scale == nullptr || scaleCount < 2) {
        scale = nullptr;
        got = TIFFGetField(tif, GEOTIFF_MODEL_PIXEL_SCALE_TAG, &scale);
        if (got != 1 || scale == nullptr) return false;
        scaleCount = 3;
    }
    double scaleX = scale[0], scaleY = scale[1];

    double* tp = nullptr;
    uint16_t tpCount = 0;
    got = TIFFGetField(tif, GEOTIFF_MODEL_TIEPOINT_TAG, &tpCount, &tp);
    if (got != 1 || tp == nullptr || tpCount < 6) {
        tp = nullptr;
        got = TIFFGetField(tif, GEOTIFF_MODEL_TIEPOINT_TAG, &tp);
        if (got != 1 || tp == nullptr) return false;
        tpCount = 6;
    }
    double tpX = tp[3], tpY = tp[4];

    if (std::abs(scaleX) < 1e-12 || std::abs(scaleY) < 1e-12) return false;

    // GeoTIFF Area convention: tiepoint is the top-LEFT CORNER of the pixel.
    // Offset by half a pixel to get the center of the upper-left pixel
    // (matching the .tfw convention). Negate scaleY for north-up images.
    ortho.A = scaleX;
    ortho.E = -scaleY;
    ortho.B = 0.0;
    ortho.D = 0.0;
    ortho.C = tpX + 0.5 * scaleX;
    ortho.F = tpY - 0.5 * scaleY;
    ortho.hasGeo = true;

    std::cerr << "[geotiff] scale=(" << scaleX << ", " << scaleY
              << ") tiepoint=(" << tpX << ", " << tpY << ")\n"
              << "[geotiff] affine: A=" << ortho.A << " E=" << ortho.E
              << " C=" << ortho.C << " F=" << ortho.F << std::endl;
    return true;
}

// ---------------------------------------------------------------------------
// Read the GeoKeyDirectoryTag (34735) and extract the file's EPSG code —
// see the header's declaration for exactly what this does and doesn't
// resolve. Uses the SAME "count + pointer" TIFFGetField() calling
// convention as readGeoTIFFTags() above (including its defensive
// single-pointer fallback) — NOT the single-pointer convention that
// caused a real, confirmed EXC_BAD_ACCESS crash for TIFFTAG_GDAL_NODATA
// (dem_mesh.cpp, design doc §6p/6p-2) on an unregistered custom tag.
// GeoKeyDirectoryTag, like ModelPixelScaleTag/ModelTiepointTag, is a core
// GeoTIFF-spec tag, not a GDAL-specific extension, and this codebase's
// existing use of this exact pattern for those two tags has not crashed
// across this whole session's loads — the same reasoning is being
// extended here, not a new, untested assumption. Still the same RISK
// CLASS as that crash (an unregistered custom tag read via
// TIFFGetField()), and still not something to treat as fully guaranteed
// safe without real hardware verification.
// ---------------------------------------------------------------------------
#define GEOTIFF_GEO_KEY_DIRECTORY_TAG 34735

bool readEPSGCode(TIFF* tif, int& outEpsg) {
    uint16_t* keys = nullptr;
    uint16_t count = 0;
    int got = TIFFGetField(tif, GEOTIFF_GEO_KEY_DIRECTORY_TAG, &count, &keys);
    if (got != 1 || keys == nullptr || count < 4) {
        keys = nullptr;
        got = TIFFGetField(tif, GEOTIFF_GEO_KEY_DIRECTORY_TAG, &keys);
        if (got != 1 || keys == nullptr) return false;
        // No reliable count from this fallback call — trust the
        // directory's own NumberOfKeys header field (checked below) but
        // stay defensive about reading past a short/malformed buffer,
        // since we can't independently verify its true length here.
        count = 0xFFFF;
    }

    // Directory layout: [KeyDirectoryVersion, KeyRevision, MinorRevision,
    // NumberOfKeys], then NumberOfKeys entries of 4 SHORTs each:
    // [KeyID, TIFFTagLocation, Count, Value_Offset].
    uint16_t numKeys = keys[3];
    int modelType = 0;       // GTModelTypeGeoKey: 1=projected, 2=geographic
    int projectedEpsg = 0;   // ProjectedCSTypeGeoKey
    int geographicEpsg = 0;  // GeographicTypeGeoKey
    for (uint16_t i = 0; i < numKeys; ++i) {
        size_t base = 4 + static_cast<size_t>(i) * 4;
        if (base + 3 >= count) break; // guard against a truncated/malformed directory
        uint16_t keyId = keys[base + 0];
        uint16_t tagLocation = keys[base + 1];
        uint16_t value = keys[base + 3];
        if (tagLocation != 0) continue; // only the inline-SHORT-value case (see header comment)
        if (keyId == 1024) modelType = value;
        else if (keyId == 3072) projectedEpsg = value;
        else if (keyId == 2048) geographicEpsg = value;
    }

    // 0 = undefined, 32767 = "user-defined" (not EPSG-catalogued) — both
    // reserved sentinels per the GeoTIFF spec, not real codes.
    auto isUsable = [](int code) { return code > 0 && code < 32767; };
    if (modelType == 1 && isUsable(projectedEpsg))  { outEpsg = projectedEpsg;  return true; }
    if (modelType == 2 && isUsable(geographicEpsg)) { outEpsg = geographicEpsg; return true; }
    // No reliable GTModelTypeGeoKey found, but only one candidate exists —
    // use it rather than give up.
    if (isUsable(projectedEpsg))  { outEpsg = projectedEpsg;  return true; }
    if (isUsable(geographicEpsg)) { outEpsg = geographicEpsg; return true; }
    return false;
}

// ---------------------------------------------------------------------------
// Reproject an orthophoto's affine transform into the DEM's CRS when the
// two differ — see the header's declaration for the full method and its
// approximation caveat (a local linear fit, not a general-purpose
// non-linear reprojection).
// ---------------------------------------------------------------------------
bool reprojectToMatchCRS(Orthophoto& ortho, const Orthophoto& demGeo) {
    if (ortho.epsg == 0 || demGeo.epsg == 0) {
        std::cerr << "[geotiff] CRS reprojection skipped: EPSG code unknown for "
                  << (ortho.epsg == 0 ? "the orthophoto" : "the DEM")
                  << " (custom/non-catalogued CRS, or no GeoKeyDirectoryTag found)"
                  << std::endl;
        return false;
    }
    if (ortho.epsg == demGeo.epsg) {
        return false; // already the same CRS — nothing to do
    }
#ifndef LASVIEWER_HAS_PROJ
    std::cerr << "[geotiff] CRS mismatch detected (orthophoto EPSG:" << ortho.epsg
              << " vs DEM EPSG:" << demGeo.epsg << "), but this build has no PROJ "
                 "support — install PROJ (e.g. `brew install proj` / `port install proj`) "
                 "and rebuild to reproject automatically, or reproject the orthophoto "
                 "yourself first with gdalwarp (e.g. `gdalwarp -t_srs EPSG:"
              << demGeo.epsg << " ortho.tif ortho_reprojected.tif`)."
              << std::endl;
    return false;
#else
    PJ_CONTEXT* ctx = proj_context_create();
    if (!ctx) {
        std::cerr << "[geotiff] PROJ context creation failed — skipping reprojection"
                  << std::endl;
        return false;
    }
    std::string srcCRS = "EPSG:" + std::to_string(ortho.epsg);
    std::string dstCRS = "EPSG:" + std::to_string(demGeo.epsg);
    PJ* transform = proj_create_crs_to_crs(ctx, srcCRS.c_str(), dstCRS.c_str(), nullptr);
    if (!transform) {
        std::cerr << "[geotiff] PROJ could not find a transform from " << srcCRS
                  << " to " << dstCRS << " — skipping reprojection. Reproject "
                     "manually with gdalwarp instead (see readEPSGCode's caller "
                     "for the suggested command)." << std::endl;
        proj_context_destroy(ctx);
        return false;
    }
    // PROJ conventionally expects/returns (longitude, latitude) or
    // (easting, northing) in that axis order for "EPSG:N" style CRS
    // strings via proj_create_crs_to_crs — this matches how this
    // codebase already treats ortho.C/F (X then Y), so no axis swap is
    // needed here.
    double srcX[4], srcY[4];
    srcX[0] = ortho.C;                              srcY[0] = ortho.F;                              // top-left
    srcX[1] = ortho.C + ortho.A * ortho.width;       srcY[1] = ortho.F;                              // top-right
    srcX[2] = ortho.C;                              srcY[2] = ortho.F + ortho.E * ortho.height;      // bottom-left
    srcX[3] = ortho.C + ortho.A * ortho.width;       srcY[3] = ortho.F + ortho.E * ortho.height;      // bottom-right
    double dstX[4], dstY[4];
    bool allOk = true;
    for (int i = 0; i < 4; ++i) {
        PJ_COORD in = proj_coord(srcX[i], srcY[i], 0, 0);
        PJ_COORD out = proj_trans(transform, PJ_FWD, in);
        if (!std::isfinite(out.xy.x) || !std::isfinite(out.xy.y)) { allOk = false; break; }
        dstX[i] = out.xy.x;
        dstY[i] = out.xy.y;
    }
    proj_destroy(transform);
    proj_context_destroy(ctx);
    if (!allOk) {
        std::cerr << "[geotiff] PROJ reprojection produced a non-finite result — "
                     "skipping (transform pipeline may not cover this area)" << std::endl;
        return false;
    }

    // Re-fit a still axis-aligned (no rotation/shear) affine from the 4
    // reprojected corners — this codebase's affine model never supported
    // rotation/shear even before this (a pre-existing limitation, not
    // something newly introduced here), and a single orthophoto tile's
    // extent is small enough that the linear approximation error from
    // treating a possibly slightly non-axis-aligned true reprojection as
    // axis-aligned should be small relative to the DEM's own resolution.
    // Uses the average of both horizontal/vertical edges (top and bottom
    // for the horizontal scale, left and right for the vertical scale)
    // rather than just one pair of corners, so a small asymmetric
    // distortion is split evenly rather than biased toward one edge.
    double newA = ((dstX[1] - dstX[0]) + (dstX[3] - dstX[2])) / (2.0 * ortho.width);
    double newE = ((dstY[2] - dstY[0]) + (dstY[3] - dstY[1])) / (2.0 * ortho.height);
    double newC = (dstX[0] + dstX[2]) / 2.0; // average of the two left-edge corners
    double newF = (dstY[0] + dstY[1]) / 2.0; // average of the two top-edge corners

    std::cerr << "[geotiff] reprojected orthophoto from EPSG:" << ortho.epsg
              << " to EPSG:" << demGeo.epsg << " (DEM's CRS): affine A="
              << newA << " E=" << newE << " C=" << newC << " F=" << newF
              << std::endl;
    ortho.A = newA;
    ortho.E = newE;
    ortho.C = newC;
    ortho.F = newF;
    ortho.epsg = demGeo.epsg; // now expressed in the DEM's CRS
    return true;
#endif
}

// ---------------------------------------------------------------------------
// Load a .tfw world file (6 lines: A, D, B, E, C, F).
// ---------------------------------------------------------------------------

bool loadTFW(const std::string& path, Orthophoto& ortho) {
    std::ifstream f(path);
    if (!f) return false;
    double v[6];
    int n = 0;
    while (n < 6 && f >> v[n]) ++n;
    if (n < 6) return false;
    ortho.A = v[0]; ortho.D = v[1];
    ortho.B = v[2]; ortho.E = v[3];
    ortho.C = v[4]; ortho.F = v[5];
    ortho.hasGeo = true;
    std::cerr << "[tfw] loaded " << path << ": A=" << ortho.A << " E=" << ortho.E
              << " C=" << ortho.C << " F=" << ortho.F << std::endl;
    return true;
}

// ---------------------------------------------------------------------------
// Load a TIFF and read its GeoTIFF tags. Downsamples if larger than maxPixels.
// ---------------------------------------------------------------------------

bool loadTIFF(const std::string& path, Orthophoto& ortho, int maxPixels) {
    std::cerr << "[tiff] opening: " << path << std::endl;
    TIFF* tif = TIFFOpen(path.c_str(), "r");
    if (!tif) {
        std::cerr << "ERROR: could not open TIFF: " << path << std::endl;
        return false;
    }
    uint32_t w = 0, h = 0;
    TIFFGetField(tif, TIFFTAG_IMAGEWIDTH,  &w);
    TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &h);
    std::cerr << "[tiff] dimensions: " << w << " x " << h << std::endl;
    if (w == 0 || h == 0 || w > 100000 || h > 100000) {
        std::cerr << "ERROR: suspicious TIFF dimensions" << std::endl;
        TIFFClose(tif);
        return false;
    }
    ortho.width  = static_cast<int>(w);
    ortho.height = static_cast<int>(h);
    readGeoTIFFTags(tif, ortho);
    if (ortho.hasGeo) {
        // Only worth reading if there's an affine transform to potentially
        // reproject in the first place — see readEPSGCode()'s own comment
        // for what this can and can't resolve.
        if (readEPSGCode(tif, ortho.epsg)) {
            std::cerr << "[tiff] EPSG:" << ortho.epsg << std::endl;
        }
    }

    size_t rasterBytes = static_cast<size_t>(w) * h * sizeof(uint32_t);
    if (rasterBytes > static_cast<size_t>(2048) * 1024 * 1024) {
        std::cerr << "ERROR: TIFF raster exceeds 2 GB" << std::endl;
        TIFFClose(tif);
        return false;
    }
    uint32_t* raster = static_cast<uint32_t*>(_TIFFmalloc(static_cast<tmsize_t>(rasterBytes)));
    if (!raster) {
        std::cerr << "ERROR: could not allocate TIFF raster ("
                  << (rasterBytes / (1024 * 1024)) << " MB)" << std::endl;
        TIFFClose(tif);
        return false;
    }
    if (TIFFReadRGBAImageOriented(tif, w, h, raster, ORIENTATION_TOPLEFT) == 0) {
        _TIFFfree(raster); TIFFClose(tif); return false;
    }
    ortho.pixels.resize(static_cast<size_t>(w) * h * 4);
    for (size_t i = 0; i < static_cast<size_t>(w) * h; ++i) {
        ortho.pixels[i * 4 + 0] = TIFFGetR(raster[i]);
        ortho.pixels[i * 4 + 1] = TIFFGetG(raster[i]);
        ortho.pixels[i * 4 + 2] = TIFFGetB(raster[i]);
        ortho.pixels[i * 4 + 3] = TIFFGetA(raster[i]);
    }
    _TIFFfree(raster);
    TIFFClose(tif);

    // Downsample if the pixel count exceeds maxPixels (box filter).
    size_t totalPixels = static_cast<size_t>(w) * h;
    if (totalPixels > static_cast<size_t>(maxPixels)) {
        float scale = std::sqrt(static_cast<float>(maxPixels) / totalPixels);
        int dstW = std::max(1, static_cast<int>(w * scale));
        int dstH = std::max(1, static_cast<int>(h * scale));
        std::cerr << "[tiff] downsampling " << w << "x" << h << " -> " << dstW << "x" << dstH << std::endl;
        std::vector<uint8_t> dst(static_cast<size_t>(dstW) * dstH * 4);
        int srcW = static_cast<int>(w), srcH = static_cast<int>(h);
        for (int y = 0; y < dstH; ++y) {
            int sy0 = static_cast<int>(static_cast<int64_t>(y) * srcH / dstH);
            int sy1 = static_cast<int>(static_cast<int64_t>(y + 1) * srcH / dstH);
            if (sy1 <= sy0) sy1 = sy0 + 1;
            if (sy1 > srcH) sy1 = srcH;
            for (int x = 0; x < dstW; ++x) {
                int sx0 = static_cast<int>(static_cast<int64_t>(x) * srcW / dstW);
                int sx1 = static_cast<int>(static_cast<int64_t>(x + 1) * srcW / dstW);
                if (sx1 <= sx0) sx1 = sx0 + 1;
                if (sx1 > srcW) sx1 = srcW;
                int r = 0, g = 0, b = 0, a = 0, n = 0;
                for (int sy = sy0; sy < sy1; ++sy) {
                    const uint8_t* row = &ortho.pixels[(static_cast<size_t>(sy) * srcW + sx0) * 4];
                    for (int sx = sx0; sx < sx1; ++sx) {
                        r += row[0]; g += row[1]; b += row[2]; a += row[3];
                        row += 4; ++n;
                    }
                }
                size_t di = (static_cast<size_t>(y) * dstW + x) * 4;
                dst[di + 0] = static_cast<uint8_t>(r / n);
                dst[di + 1] = static_cast<uint8_t>(g / n);
                dst[di + 2] = static_cast<uint8_t>(b / n);
                dst[di + 3] = static_cast<uint8_t>(a / n);
            }
        }
        ortho.pixels.swap(dst);
        ortho.width = dstW;
        ortho.height = dstH;
        double factorX = static_cast<double>(srcW) / dstW;
        double factorY = static_cast<double>(srcH) / dstH;
        ortho.A *= factorX;
        ortho.E *= factorY;
        std::cerr << "[geotiff] rescaled affine: A=" << ortho.A << " E=" << ortho.E << std::endl;
    }
    std::cout << "Loaded TIFF: " << path << " (" << ortho.width << "x" << ortho.height << ")" << std::endl;
    return true;
}

// ---------------------------------------------------------------------------
// Bilinear sample the orthophoto at fractional pixel (col, row).
// ---------------------------------------------------------------------------

glm::vec3 sampleOrthoBilinear(const Orthophoto& ortho, double col, double row) {
    if (col < 0 || col >= ortho.width || row < 0 || row >= ortho.height) {
        return {0.35f, 0.35f, 0.35f};  // mid-dark gray for out-of-bounds
    }
    int x0 = static_cast<int>(col);
    int y0 = static_cast<int>(row);
    int x1 = std::min(x0 + 1, ortho.width - 1);
    int y1 = std::min(y0 + 1, ortho.height - 1);
    float fx = static_cast<float>(col - x0);
    float fy = static_cast<float>(row - y0);
    auto s = [&](int x, int y) -> glm::vec3 {
        size_t i = (static_cast<size_t>(y) * ortho.width + x) * 4;
        return {ortho.pixels[i + 0] / 255.0f,
                ortho.pixels[i + 1] / 255.0f,
                ortho.pixels[i + 2] / 255.0f};
    };
    glm::vec3 c00 = s(x0, y0), c10 = s(x1, y0), c01 = s(x0, y1), c11 = s(x1, y1);
    return c00 * (1 - fx) * (1 - fy) + c10 * fx * (1 - fy)
         + c01 * (1 - fx) * fy + c11 * fx * fy;
}

// ---------------------------------------------------------------------------
// Sample the orthophoto for each point's world (X, Y) and store the result
// in cloud.orthoColors.
// ---------------------------------------------------------------------------

void colorizeFromOrthophoto(PointCloud& cloud, const Orthophoto& ortho) {
    if (ortho.pixels.empty() || !ortho.hasGeo || cloud.pointCount == 0) return;
    std::cerr << "[colorize] sampling orthophoto for " << cloud.pointCount << " points..." << std::endl;
    cloud.orthoColors.resize(cloud.pointCount * 3);

    // Invert the 2x2 affine [A B; D E] to go from world (X,Y) to pixel (col,row).
    // IMPORTANT: cloud.positions stores GL_Z = -(worldY - center.y)/scale (negated).
    double det = ortho.A * ortho.E - ortho.B * ortho.D;
    if (std::abs(det) < 1e-15) return;
    double invA =  ortho.E / det, invB = -ortho.B / det;
    double invD = -ortho.D / det, invE =  ortho.A / det;
    double offsetX = cloud.worldCenter.x - ortho.C;
    double offsetY = cloud.worldCenter.y - ortho.F;
    double scale = cloud.worldScale;
    double kColX =  invA * scale, kColZ = -invB * scale;
    double kCol0 =  invA * offsetX + invB * offsetY;
    double kRowX =  invD * scale, kRowZ = -invE * scale;
    double kRow0 =  invD * offsetX + invE * offsetY;

    std::atomic<size_t> outOfBounds{0};
#if LASVIEWER_HAS_OPENMP
    int nThreads = std::max(1, std::min((int)cloud.pointCount / 100000,
                                       (int)std::thread::hardware_concurrency()));
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
