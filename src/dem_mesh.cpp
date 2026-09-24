// dem_mesh.cpp — Adaptive triangulated DEM mesh with texture.
//
// Implements DEMMesh::loadFromDEM(), render(), destroy().
//
// Triangulation strategy (T-junction free):
// ------------------------------------------
//   1. Adaptive quadtree subdivision based on (a) geometry error vs bilinear
//      interpolation of corners and (b) texture span.
//   2. Balance pass: any leaf whose neighbor is 2+ levels finer is subdivided.
//      This guarantees no two adjacent leaves differ by more than 1 level.
//   3. Edge midpoint welding: for each leaf, insert a midpoint vertex on each
//      edge whose neighbor is at the same level OR finer (level >= my level).
//      Same-level neighbors both insert the midpoint → they weld.
//      Finer (level+1) neighbors have their corner at this cell's edge midpoint
//      → they weld with this cell's midpoint.
//      Coarser (level-1) neighbors have a straight edge — no midpoint needed.
//   4. Triangle fan: each leaf is triangulated as a fan from its center vertex
//      out to its boundary vertices (4 corners + 0..4 midpoints).
//
// Texture strategy:
// -----------------
//   - If an orthophoto is provided with GeoTIFF tags: use proper world→UV
//     transform.
//   - If an orthophoto is provided WITHOUT geo: stretch the texture over the
//     DEM extent (UV = col/(w-1), row/(h-1)).
//   - If no orthophoto: the mesh shader falls back to an elevation-based
//     color ramp (green→brown→white).
//
// FIX: The previous implementation used TIFFReadRGBAImageOriented to read
// elevation data. That function converts YCbCr JPEG-compressed TIFFs into
// RGBA, which corrupts the elevation values. We now read the raw first
// band via TIFFReadScanline (for stripped TIFFs) or TIFFReadTile (for tiled
// TIFFs), preserving the actual numeric elevation values.
#include "dem_mesh.h"
#include "geotiff.h"
// GLFW/OpenGL headers now come from dem_mesh.h -> gl_platform.h (see that
// file for why this used to be a fragile per-file ad-hoc block).

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <tiffio.h>

#include <iostream>
#include <cmath>
#include <algorithm>
#include <functional>
#include <unordered_map>
#include <vector>
#include <string>
#include <utility>
#include <limits>

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

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Reads the GeoTIFF's own declared NODATA value (GDAL_NODATA tag, 42113 —
// not part of the baseline TIFF spec, so no libtiff-predefined constant;
// stored as an ASCII string, e.g. "-9999" or "-3.4028234663852886e+38").
// See dem_mesh.h's declaration for why this matters — without it, nodata
// detection previously only caught the "< -9000" convention, silently
// missing any DEM using a different sentinel.
// ---------------------------------------------------------------------------
#ifndef TIFFTAG_GDAL_NODATA
#define TIFFTAG_GDAL_NODATA 42113
#endif

bool readDEMNodataValue(TIFF* /*tif*/, float& /*outNodata*/) {
    // DISABLED after a SECOND identical crash, despite a targeted fix in
    // between. History, in full, because getting this wrong twice matters
    // for whoever revisits it:
    //
    //   1. ORIGINAL bug: called TIFFGetField() directly on
    //      TIFFTAG_GDAL_NODATA (42113), a GDAL-private tag not part of the
    //      baseline TIFF spec. libtiff doesn't know an unregistered
    //      custom tag's type, so its varargs-based field dispatch
    //      (_TIFFVGetField) has no way to know what to write through the
    //      char** pointer passed to it — EXC_BAD_ACCESS, crash address
    //      0x3ff0000000000000 (the IEEE-754 bit pattern of the double
    //      1.0, misinterpreted as a pointer — textbook type confusion).
    //
    //   2. FIRST FIX (this function, until now): register the tag first
    //      via TIFFMergeFieldInfo() with a TIFFFieldInfo declaring it
    //      TIFF_ASCII, FIELD_CUSTOM — the same registration pattern
    //      GDAL's own libtiff-based readers are known to use for this
    //      exact tag. Only proceed to TIFFGetField() if the registration
    //      call returned success.
    //
    //   3. SAME CRASH RECURRED, in exactly the same place, on real
    //      hardware — meaning TIFFMergeFieldInfo() returned success (the
    //      registration wasn't rejected), yet the tag still wasn't
    //      safely readable via TIFFGetField() afterward. That means the
    //      registration itself was subtly wrong in some way libtiff's own
    //      validation didn't catch — a different, more specific bug than
    //      "forgot to register," and one I cannot diagnose further
    //      without a real libtiff instance and debugger to iterate
    //      against, which isn't available in this environment. A third
    //      attempt at the exact same mechanism, from the same position of
    //      not being able to verify it, isn't a responsible thing to ship
    //      a second time.
    //
    // So: this function is now a permanent no-op, always returning false.
    // Every caller already treats false as "no declared value found, use
    // the legacy < -9000 heuristic instead" (see e.g. DEMMesh::loadFromDEM
    // and DEMTessMesh::loadFromDEM) — that heuristic is the ORIGINAL,
    // long-proven-safe behavior from before this feature existed at all,
    // and is what every DEM load now unconditionally falls back to. This
    // does reintroduce the original, narrower gap this function was meant
    // to close (a DEM declaring a nodata sentinel other than the common
    // -9999-style convention, e.g. 0, won't be detected) — a real, known
    // limitation, but a correctness gap is a strictly better failure mode
    // than a crash. Re-enabling this would need a fundamentally different,
    // verifiable approach — e.g. parsing the TIFF IFD directly at the byte
    // level (bypassing libtiff's per-tag type dispatch entirely, so there
    // is no "wrong type" for it to guess) — not another variant of the
    // same TIFFGetField()-based mechanism that has now failed twice.
    return false;
}

// ---------------------------------------------------------------------------
// Read the first band of a (possibly tiled) TIFF as Float32 elevation values.
// Supports the common DEM sample formats:
//   - UInt8, UInt16, Int16, UInt32, Int32  → integer rasters
//   - Float32                                → most modern DEMs (SRTM-3, RGE Alti)
// Falls back to TIFFReadScanline for stripped TIFFs and TIFFReadTile for
// tiled TIFFs. Returns true on success; fills `out` with w*h floats.
// ---------------------------------------------------------------------------
bool readDEMElevations(TIFF* tif, uint32_t w, uint32_t h,
                       std::vector<float>& out,
                       uint16_t& outSpp) {
    uint16_t bps = 8, sf = SAMPLEFORMAT_UINT, spp = 1;
    TIFFGetFieldDefaulted(tif, TIFFTAG_BITSPERSAMPLE,  &bps);
    TIFFGetFieldDefaulted(tif, TIFFTAG_SAMPLEFORMAT,   &sf);
    TIFFGetFieldDefaulted(tif, TIFFTAG_SAMPLESPERPIXEL, &spp);

    std::cerr << "[dem] sample format: bps=" << bps << " sf=" << sf
              << " spp=" << spp << std::endl;

    out.resize(static_cast<size_t>(w) * h);

    // Determine the byte size of one sample.
    size_t bytesPerSample = bps / 8;
    if (bytesPerSample == 0) bytesPerSample = 1;

    // Detect tiled vs stripped.
    uint32_t tileW = 0, tileH = 0;
    bool isTiled = TIFFIsTiled(tif) != 0;
    if (isTiled) {
        TIFFGetField(tif, TIFFTAG_TILEWIDTH,  &tileW);
        TIFFGetField(tif, TIFFTAG_TILELENGTH, &tileH);
    }

    // Helper: interpret one raw sample as a float.
    auto interpretSample = [&](const uint8_t* raw) -> float {
        if (sf == SAMPLEFORMAT_IEEEFP) {
            if (bytesPerSample == 4) {
                float v;
                std::memcpy(&v, raw, 4);
                return v;
            } else if (bytesPerSample == 8) {
                double v;
                std::memcpy(&v, raw, 8);
                return static_cast<float>(v);
            }
        }
        // Integer sample formats.
        int64_t iv = 0;
        bool isSigned = (sf == SAMPLEFORMAT_INT);
        // Read little-endian (libtiff's native byte order is preserved in
        // the scanline buffer; for cross-endian files libtiff handles it).
        for (size_t b = 0; b < bytesPerSample; ++b) {
            iv |= static_cast<int64_t>(raw[b]) << (8 * b);
        }
        if (isSigned && bytesPerSample < 8) {
            // Sign-extend.
            int64_t signBit = static_cast<int64_t>(1) << (8 * bytesPerSample - 1);
            if (iv & signBit) iv |= ~((static_cast<int64_t>(1) << (8 * bytesPerSample)) - 1);
        }
        return static_cast<float>(iv);
    };

    // For multi-band 8-bit TIFFs (spp >= 3, bps=8), the elevation may be
    // encoded as Terrain RGB: height = (R*65536 + G*256 + B) * 0.1 - 10000.
    // This is the standard IGN MNS LiDAR HD encoding.
    bool isTerrainRGB = (spp >= 3 && bps == 8 && sf == SAMPLEFORMAT_UINT);
    if (isTerrainRGB) {
        std::cerr << "[dem] Terrain RGB encoding detected (spp=" << spp
                  << ", bps=8). Decoding: h = (R*65536+G*256+B)*0.1 - 10000" << std::endl;
    }

    auto decodePixel = [&](const uint8_t* p) -> float {
        if (isTerrainRGB) {
            // Read R, G, B bands (bands are interleaved: RGBRGB...)
            int r = p[0];
            int g = p[1];
            int b = p[2];
            // Terrain RGB: height = (R*65536 + G*256 + B) * 0.1 - 10000
            float h = (static_cast<float>(r) * 65536.0f +
                       static_cast<float>(g) * 256.0f +
                       static_cast<float>(b)) * 0.1f - 10000.0f;
            if (h < -9000.0f) h = 0.0f;  // nodata
            return h;
        }
        return interpretSample(p);
    };

    if (!isTiled) {
        // Stripped TIFF — read scanline by scanline.
        // TIFFScanlineSize returns bytes per scanline (across all samples).
        tmsize_t scanlineSize = TIFFScanlineSize(tif);
        if (scanlineSize <= 0) {
            std::cerr << "ERROR: TIFFScanlineSize returned " << scanlineSize << std::endl;
            return false;
        }
        std::vector<uint8_t> buf(static_cast<size_t>(scanlineSize));
        for (uint32_t row = 0; row < h; ++row) {
            if (TIFFReadScanline(tif, buf.data(), row, 0) < 0) {
                std::cerr << "ERROR: TIFFReadScanline failed at row " << row << std::endl;
                return false;
            }
            const uint8_t* p = buf.data();
            for (uint32_t col = 0; col < w; ++col) {
                out[static_cast<size_t>(row) * w + col] = decodePixel(p);
                p += bytesPerSample * spp;
            }
        }
        outSpp = spp;
        return true;
    }

    // Tiled TIFF — read tile by tile.
    if (tileW == 0 || tileH == 0) {
        std::cerr << "ERROR: invalid tile dimensions (" << tileW << "x" << tileH << ")" << std::endl;
        return false;
    }
    tmsize_t tileSize = TIFFTileSize(tif);
    if (tileSize <= 0) {
        std::cerr << "ERROR: TIFFTileSize returned " << tileSize << std::endl;
        return false;
    }
    std::vector<uint8_t> buf(static_cast<size_t>(tileSize));
    for (uint32_t ty = 0; ty < h; ty += tileH) {
        for (uint32_t tx = 0; tx < w; tx += tileW) {
            if (TIFFReadTile(tif, buf.data(), tx, ty, 0, 0) < 0) {
                std::cerr << "ERROR: TIFFReadTile failed at (" << tx << "," << ty << ")" << std::endl;
                return false;
            }
            uint32_t copyW = std::min(tileW, w - tx);
            uint32_t copyH = std::min(tileH, h - ty);
            for (uint32_t r = 0; r < copyH; ++r) {
                const uint8_t* rowPtr = buf.data() + (r * tileW * bytesPerSample * spp);
                for (uint32_t c = 0; c < copyW; ++c) {
                    out[static_cast<size_t>(ty + r) * w + (tx + c)] = decodePixel(rowPtr);
                    rowPtr += bytesPerSample * spp;
                }
            }
        }
    }
    outSpp = spp;
    return true;
}

// ---------------------------------------------------------------------------
// DEMMesh::loadFromDEM — build an adaptive, T-junction-free triangulation.
//
// Pipeline:
//   1. Read DEM elevations (raw scanline/tile, preserves Float32 values).
//   2. Compute world bbox and GL-space transform.
//   3. Adaptive quadtree subdivision based on geometry error & texture span.
//   4. Balance pass: subdivide any leaf whose neighbor is 2+ levels finer.
//   5. For each leaf, insert edge midpoints on edges with same-level or
//      finer neighbors (welds with neighbor's midpoint or corner).
//   6. Fan-triangulate each leaf from its center vertex.
// ---------------------------------------------------------------------------

bool DEMMesh::loadFromDEM(const std::string& path, const Orthophoto* ortho,
                          double angleThresholdDeg, int maxLevelParam) {
    sourcePath = path;
    collapseAngleDeg = angleThresholdDeg;
    maxLevel = maxLevelParam;
    orthoUsable = true; // reset each load — this object can be reloaded with
                        // a different DEM/ortho pairing; a stale false from
                        // a previous load must not persist
    std::cerr << "[dem-mesh] opening: " << path << std::endl;
    if (ortho) {
        std::cerr << "[dem-mesh] orthophoto: " << ortho->width << "x" << ortho->height
                  << " hasGeo=" << ortho->hasGeo
                  << " pixels=" << (ortho->pixels.empty() ? 0 : ortho->pixels.size() / 4)
                  << std::endl;
    } else {
        std::cerr << "[dem-mesh] no orthophoto provided — will use elevation color ramp"
                  << std::endl;
    }
    TIFF* tif = TIFFOpen(path.c_str(), "r");
    if (!tif) { std::cerr << "ERROR: could not open DEM" << std::endl; return false; }

    uint32_t w = 0, h = 0;
    TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &w);
    TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &h);
    if (w == 0 || h == 0) { TIFFClose(tif); return false; }

    // Read GeoTIFF tags.
    Orthophoto geo;
    geo.width = static_cast<int>(w);
    geo.height = static_cast<int>(h);
    readGeoTIFFTags(tif, geo);
    if (geo.hasGeo && readEPSGCode(tif, geo.epsg)) {
        std::cerr << "[dem-mesh] DEM EPSG:" << geo.epsg << std::endl;
    }
    if (!geo.hasGeo) {
        std::cerr << "ERROR: DEM has no GeoTIFF tags" << std::endl;
        TIFFClose(tif);
        return false;
    }

    // Read elevations using the proper scanline/tile path (NOT
    // TIFFReadRGBAImageOriented — that corrupts Float32 DEMs by going through
    // a YCbCr/RGBA decode step).
    std::vector<float> elevs;
    uint16_t demSpp = 1;
    if (!readDEMElevations(tif, w, h, elevs, demSpp)) {
        TIFFClose(tif);
        return false;
    }

    // Declared NODATA value (GDAL_NODATA tag) — read while the TIFF is
    // still open. See dem_mesh.h's declaration for why this matters:
    // without it, nodata detection only ever caught the legacy "< -9000"
    // heuristic, silently missing DEMs using a different sentinel value.
    float declaredNodata = 0.0f;
    bool hasDeclaredNodata = readDEMNodataValue(tif, declaredNodata);
    if (!hasDeclaredNodata) {
        std::cerr << "[dem-mesh] no declared NODATA tag — using legacy < -9000 heuristic"
                  << std::endl;
    }
    TIFFClose(tif);

    // Unified nodata test: exact(-ish) match against the DEM's own
    // declared value if it has one, otherwise the legacy heuristic. Used
    // everywhere in this function instead of a hardcoded "< -9000.0f" —
    // see dem_mesh.h's readDEMNodataValue() declaration for why that
    // heuristic alone silently missed DEMs using a different sentinel.
    auto isNodataValue = [&](float v) -> bool {
        if (hasDeclaredNodata) {
            float tol = 1e-3f * std::max(1.0f, std::abs(declaredNodata));
            return std::abs(v - declaredNodata) < tol;
        }
        return v < -9000.0f;
    };

    // Find Z range.
    float zMin = 1e30f, zMax = -1e30f;
    for (float v : elevs) {
        if (isNodataValue(v)) continue;
        if (v < zMin) zMin = v;
        if (v > zMax) zMax = v;
    }
    if (zMin > zMax) { zMin = 0; zMax = 1; }
    std::cerr << "[dem-mesh] elevation: " << zMin << " .. " << zMax << std::endl;

    // World bbox.
    double minX = geo.C, minY = geo.F;
    double maxX = geo.C + geo.A * (w - 1);
    double maxY = geo.F + geo.E * (h - 1);
    if (minX > maxX) std::swap(minX, maxX);
    if (minY > maxY) std::swap(minY, maxY);
    bboxMin = glm::dvec3(minX, minY, zMin);
    bboxMax = glm::dvec3(maxX, maxY, zMax);
    worldCenter = (bboxMin + bboxMax) * 0.5;
    worldScale = glm::length(bboxMax - bboxMin);
    if (worldScale < 1e-9) worldScale = 1.0;
    double invScale = 1.0 / worldScale;

    // Bilinear elevation sampler (clamps to edge).
    auto sampleElev = [&](double col, double row) -> float {
        if (col < 0) col = 0;
        if (col > w - 1) col = w - 1;
        if (row < 0) row = 0;
        if (row > h - 1) row = h - 1;
        int c0 = static_cast<int>(col);
        int r0 = static_cast<int>(row);
        int c1 = std::min(c0 + 1, static_cast<int>(w) - 1);
        int r1 = std::min(r0 + 1, static_cast<int>(h) - 1);
        double fx = col - c0, fy = row - r0;
        float e00 = elevs[static_cast<size_t>(r0) * w + c0];
        float e10 = elevs[static_cast<size_t>(r0) * w + c1];
        float e01 = elevs[static_cast<size_t>(r1) * w + c0];
        float e11 = elevs[static_cast<size_t>(r1) * w + c1];
        if (isNodataValue(e00)) e00 = 0;
        if (isNodataValue(e10)) e10 = 0;
        if (isNodataValue(e01)) e01 = 0;
        if (isNodataValue(e11)) e11 = 0;
        return e00 * (1-fx)*(1-fy) + e10 * fx*(1-fy) + e01 * (1-fx)*fy + e11 * fx*fy;
    };

    // Raw (nearest-neighbor, no clamping) nodata test — deliberately
    // distinct from sampleElev() above, which smooths nodata to 0 for
    // general elevation queries where some numeric value is always
    // needed. Geometry must only exist where there is real DEM/point
    // data, never fabricated over gaps just because an orthophoto happens
    // to cover the area — same policy and same implementation as
    // dem_tess_mesh.cpp's isNodataAt(), kept in sync there.
    auto isNodataAt = [&](double col, double row) -> bool {
        int c = static_cast<int>(std::lround(col));
        int r = static_cast<int>(std::lround(row));
        c = std::clamp(c, 0, static_cast<int>(w) - 1);
        r = std::clamp(r, 0, static_cast<int>(h) - 1);
        return isNodataValue(elevs[static_cast<size_t>(r) * w + c]);
    };

    // UV computation:
    //   - If orthophoto has geo AND its extent overlaps the DEM: use
    //     world→UV transform.
    //   - If orthophoto has geo but does NOT overlap the DEM (different CRS):
    //     warn and disable texturing for this DEM (elevation color ramp) —
    //     NOT stretched. There IS geo metadata here, and it says the two
    //     rasters don't correspond; stretching an unrelated image onto the
    //     DEM would be actively misleading, not merely imprecise (reported
    //     directly: "the full orthophoto is displayed" when it shouldn't
    //     be — see orthoUsable in dem_mesh.h).
    //   - If orthophoto has no geo: stretch over DEM extent (deliberately
    //     unchanged — a convenience for manually pairing an untagged image
    //     with a DEM, where there's no correspondence info to contradict).
    //   - If no orthophoto: return (0,0); shader will use elevation ramp.
    // `ortho` itself is a shared, externally-owned, const object, so it
    // can't be mutated in place for reprojection — see dem_tess_mesh.cpp's
    // identical introduction of this pattern for the full reasoning.
    // `orthoEff` is what the rest of this function uses for geo-
    // referencing math from here on.
    Orthophoto orthoReprojected;
    const Orthophoto* orthoEff = ortho;
    if (ortho && ortho->hasGeo && geo.hasGeo && ortho->epsg != 0 && geo.epsg != 0
        && ortho->epsg != geo.epsg) {
        orthoReprojected = *ortho;
        if (reprojectToMatchCRS(orthoReprojected, geo)) {
            orthoEff = &orthoReprojected;
        }
    }

    bool orthoStretch = false;  // true = stretch ortho over DEM (no geo match)
    if (orthoEff && !orthoEff->pixels.empty()) {
        if (!orthoEff->hasGeo) {
            orthoStretch = true;
        } else {
            // Check geographic overlap between DEM and orthophoto.
            double demMinX = minX, demMaxX = maxX;
            double demMinY = minY, demMaxY = maxY;
            double orthoMinX = orthoEff->C;
            double orthoMaxX = orthoEff->C + orthoEff->A * (orthoEff->width - 1);
            double orthoMinY = orthoEff->F + orthoEff->E * (orthoEff->height - 1);
            double orthoMaxY = orthoEff->F;
            if (orthoMinX > orthoMaxX) std::swap(orthoMinX, orthoMaxX);
            if (orthoMinY > orthoMaxY) std::swap(orthoMinY, orthoMaxY);
            double overlapMinX = std::max(demMinX, orthoMinX);
            double overlapMaxX = std::min(demMaxX, orthoMaxX);
            double overlapMinY = std::max(demMinY, orthoMinY);
            double overlapMaxY = std::min(demMaxY, orthoMaxY);
            bool hasOverlap = (overlapMinX < overlapMaxX && overlapMinY < overlapMaxY);
            if (!hasOverlap) {
                // Deliberately NOT orthoStretch = true — see
                // DEMTessMesh::loadFromDEM's identical comment
                // (dem_tess_mesh.cpp) for why: there IS geo metadata, and
                // it says these two rasters don't correspond, so
                // stretching the orthophoto onto the DEM anyway would
                // show imagery from a completely unrelated location.
                // orthoStretch (the col/(w-1),row/(h-1) fallback) stays
                // reserved for the genuinely different case of NO geo
                // metadata at all. Can still happen even after a
                // successful reprojection above — same CRS now, but the
                // two rasters genuinely don't cover the same ground.
                std::cerr << "[dem-mesh] WARNING: orthophoto extent does not overlap DEM extent.\n"
                          << "[dem-mesh]   DEM:    X[" << demMinX << ", " << demMaxX
                          << "] Y[" << demMinY << ", " << demMaxY << "]\n"
                          << "[dem-mesh]   Ortho:  X[" << orthoMinX << ", " << orthoMaxX
                          << "] Y[" << orthoMinY << ", " << orthoMaxY << "]\n";
                if (ortho->epsg != 0 && geo.epsg != 0 && ortho->epsg != geo.epsg) {
                    std::cerr << "[dem-mesh]   Different CRS confirmed (orthophoto EPSG:"
                              << ortho->epsg << " vs DEM EPSG:" << geo.epsg << ")";
#ifndef LASVIEWER_HAS_PROJ
                    std::cerr << " — this build has no PROJ support to reproject "
                                 "automatically; install PROJ and rebuild, or reproject "
                                 "manually: gdalwarp -t_srs EPSG:" << geo.epsg
                              << " ortho.tif ortho_reprojected.tif";
#else
                    std::cerr << " — automatic reprojection was attempted and did not "
                                 "resolve the mismatch (see the reprojectToMatchCRS log "
                                 "line above for why)";
#endif
                    std::cerr << "\n";
                } else if (geo.epsg != 0) {
                    std::cerr << "[dem-mesh]   Different CRS? Reproject with: gdalwarp -t_srs EPSG:"
                              << geo.epsg << " ortho.tif ortho_reprojected.tif\n";
                } else {
                    std::cerr << "[dem-mesh]   Different CRS? Reproject the orthophoto to "
                                 "match the DEM's CRS with gdalwarp (DEM's own EPSG code "
                                 "could not be determined here — check both files with "
                                 "`gdalinfo` to confirm their CRS)\n";
                }
                std::cerr << "[dem-mesh]   Texturing disabled for this DEM (elevation color ramp), not stretched." << std::endl;
                orthoUsable = false;
            }
        }
    }

    auto uvFor = [&](double col, double row) -> std::pair<float,float> {
        if (!orthoEff || orthoEff->pixels.empty()) return {0,0};
        if (orthoEff->hasGeo && !orthoStretch) {
            double dx = (geo.C + geo.A * col) - orthoEff->C;
            double dy = (geo.F + geo.E * row) - orthoEff->F;
            double u = (orthoEff->A != 0) ? dx / (orthoEff->A * orthoEff->width) : 0;
            double v = (orthoEff->E != 0) ? dy / (orthoEff->E * orthoEff->height) : 0;
            return {static_cast<float>(u), static_cast<float>(v)};
        }
        // Stretch orthophoto over the DEM extent.
        // col in [0, w-1] → u in [0, 1]; row in [0, h-1] → v in [0, 1].
        double u = (w > 1) ? col / (w - 1) : 0.5;
        double v = (h > 1) ? row / (h - 1) : 0.5;
        return {static_cast<float>(u), static_cast<float>(v)};
    };

    // -------------------------------------------------------------------
    // 1. Top-down subdivision, tested against an exact min/max elevation
    //    pyramid — same algorithm and reasoning as dem_tess_mesh.cpp's
    //    quadtree, kept in sync there; see its header comment for the
    //    full history (top-down -> bottom-up -> top-down again, and why
    //    each move happened — in short: the ORIGINAL top-down's problem
    //    was testing only 5 discrete sample points per cell, not the
    //    traversal direction itself; a test against the TRUE min/max over
    //    a cell's whole footprint fixes that directly, restoring
    //    top-down's early-termination advantage safely). Parallelized
    //    with OpenMP over the independent COARSE-grid cells, same
    //    convention already used elsewhere in this codebase (geotiff.cpp,
    //    point_cloud.cpp). This file's acceptability test stays without
    //    the normal-variation criterion (this CPU-only fallback never had
    //    it) — only the positional test's source of truth changed, from
    //    a single center sample to the exact pyramid-derived bound.
    // -------------------------------------------------------------------
    struct QuadCell {
        double col, row, cw, ch;
        int level;
    };

    const int MAX_LEVEL = maxLevel; // S/F keys — see the member's comment in dem_mesh.h
    // ANGULAR criterion (replaces an earlier magnitude-based GEOM_THRESH =
    // (zMax-zMin)*0.01, an arbitrary percentage of the DEM's total
    // elevation range): comparing the deviation's ANGLE — how far the
    // true sample point tips away from the flat/bilinear surface the
    // surrounding corners imply, as seen across the cell's own size — is
    // scale-invariant, unlike comparing to a fixed fraction of the whole
    // DEM's elevation range (the same absolute bump means something
    // different in a 1m cell vs a 100m one). Same criterion and reasoning
    // as dem_tess_mesh.cpp's quadtree, kept in sync there. Not tuned
    // against real hardware.
    const double ANGLE_THRESH_DEG = collapseAngleDeg;
    const double ANGLE_THRESH_TAN = std::tan(ANGLE_THRESH_DEG * 3.14159265358979323846 / 180.0);
    const double MAX_TEX_SPAN = 64.0;

    const int COARSE = 8;

    // Min/max elevation pyramid + query function — identical to
    // dem_tess_mesh.cpp's, kept in sync there; see its comment for the
    // full derivation (why this makes top-down subdivision both exact —
    // no aliasing blind spot, unlike testing 5 discrete points — and fast
    // — early termination for flat regions, not a MAX_LEVEL-deep
    // exploration every time).
    std::vector<std::vector<float>> minPyr, maxPyr;
    std::vector<int> pyrW, pyrH;
    {
        int levels = 1;
        {
            int lw = static_cast<int>(w), lh = static_cast<int>(h);
            while (lw > 1 || lh > 1) { lw = (lw + 1) / 2; lh = (lh + 1) / 2; ++levels; }
        }
        minPyr.resize(levels);
        maxPyr.resize(levels);
        pyrW.resize(levels);
        pyrH.resize(levels);
        pyrW[0] = static_cast<int>(w);
        pyrH[0] = static_cast<int>(h);
        minPyr[0].resize(static_cast<size_t>(w) * h);
        maxPyr[0].resize(static_cast<size_t>(w) * h);
        const float kPosInf = std::numeric_limits<float>::infinity();
        const float kNegInf = -std::numeric_limits<float>::infinity();
#if LASVIEWER_HAS_OPENMP
        #pragma omp parallel for schedule(static)
#endif
        for (int r = 0; r < static_cast<int>(h); ++r) {
            for (int c = 0; c < static_cast<int>(w); ++c) {
                size_t idx = static_cast<size_t>(r) * w + c;
                float v = elevs[idx];
                bool nd = isNodataValue(v);
                minPyr[0][idx] = nd ? kPosInf : v;
                maxPyr[0][idx] = nd ? kNegInf : v;
            }
        }
        for (int lvl = 1; lvl < levels; ++lvl) {
            int sw = pyrW[lvl - 1], sh = pyrH[lvl - 1];
            int dw = (sw + 1) / 2, dh = (sh + 1) / 2;
            pyrW[lvl] = dw;
            pyrH[lvl] = dh;
            minPyr[lvl].resize(static_cast<size_t>(dw) * dh);
            maxPyr[lvl].resize(static_cast<size_t>(dw) * dh);
            const std::vector<float>& srcMin = minPyr[lvl - 1];
            const std::vector<float>& srcMax = maxPyr[lvl - 1];
            std::vector<float>& dstMin = minPyr[lvl];
            std::vector<float>& dstMax = maxPyr[lvl];
#if LASVIEWER_HAS_OPENMP
            #pragma omp parallel for schedule(static)
#endif
            for (int r = 0; r < dh; ++r) {
                for (int c = 0; c < dw; ++c) {
                    int sr0 = r * 2, sc0 = c * 2;
                    int sr1 = std::min(sr0 + 1, sh - 1);
                    int sc1 = std::min(sc0 + 1, sw - 1);
                    float mn = std::min(
                        std::min(srcMin[static_cast<size_t>(sr0) * sw + sc0],
                                 srcMin[static_cast<size_t>(sr0) * sw + sc1]),
                        std::min(srcMin[static_cast<size_t>(sr1) * sw + sc0],
                                 srcMin[static_cast<size_t>(sr1) * sw + sc1]));
                    float mx = std::max(
                        std::max(srcMax[static_cast<size_t>(sr0) * sw + sc0],
                                 srcMax[static_cast<size_t>(sr0) * sw + sc1]),
                        std::max(srcMax[static_cast<size_t>(sr1) * sw + sc0],
                                 srcMax[static_cast<size_t>(sr1) * sw + sc1]));
                    dstMin[static_cast<size_t>(r) * dw + c] = mn;
                    dstMax[static_cast<size_t>(r) * dw + c] = mx;
                }
            }
        }
    }

    auto queryTrueMinMax = [&](const QuadCell& cell) -> std::pair<float,float> {
        double cellSize = std::min(cell.cw, cell.ch);
        int lvl = 0;
        if (cellSize > 1.0) {
            lvl = static_cast<int>(std::floor(std::log2(cellSize)));
            lvl = std::clamp(lvl, 0, static_cast<int>(minPyr.size()) - 1);
        }
        int lw = pyrW[lvl], lh = pyrH[lvl];
        int texelsPerSide = 1 << lvl;
        int c0 = std::clamp(static_cast<int>(std::floor(cell.col / texelsPerSide)), 0, lw - 1);
        int c1 = std::clamp(static_cast<int>(std::floor((cell.col + cell.cw) / texelsPerSide)), 0, lw - 1);
        int r0 = std::clamp(static_cast<int>(std::floor(cell.row / texelsPerSide)), 0, lh - 1);
        int r1 = std::clamp(static_cast<int>(std::floor((cell.row + cell.ch) / texelsPerSide)), 0, lh - 1);
        float mn = std::numeric_limits<float>::infinity();
        float mx = -std::numeric_limits<float>::infinity();
        const std::vector<float>& lvlMin = minPyr[lvl];
        const std::vector<float>& lvlMax = maxPyr[lvl];
        for (int r = r0; r <= r1; ++r) {
            for (int c = c0; c <= c1; ++c) {
                size_t idx = static_cast<size_t>(r) * lw + c;
                mn = std::min(mn, lvlMin[idx]);
                mx = std::max(mx, lvlMax[idx]);
            }
        }
        return {mn, mx};
    };

    // True if `cell`, taken as a single unsplit cell, satisfies every
    // adaptive criterion — i.e. does NOT need to subdivide further.
    // Nodata is handled separately by the caller.
    auto cellIsAcceptable = [&](const QuadCell& cell) -> bool {
        float e00 = sampleElev(cell.col, cell.row);
        float e10 = sampleElev(cell.col + cell.cw, cell.row);
        float e01 = sampleElev(cell.col, cell.row + cell.ch);
        float e11 = sampleElev(cell.col + cell.cw, cell.row + cell.ch);

        // Exact geometric test against the true DEM (min/max pyramid),
        // replacing the single center-sample-vs-bilinear-average test —
        // see dem_tess_mesh.cpp's identical change and queryTrueMinMax()'s
        // comment for the derivation. A bilinear surface is convex, so
        // its own min/max over the footprint is just the min/max of its
        // 4 corners.
        float planeMin = std::min({e00, e10, e01, e11});
        float planeMax = std::max({e00, e10, e01, e11});
        auto [trueMin, trueMax] = queryTrueMinMax(cell);
        float geomErr = 0.0f;
        if (std::isfinite(trueMin) && std::isfinite(trueMax)) {
            geomErr = std::max(trueMax - planeMin, planeMax - trueMin);
            geomErr = std::max(geomErr, 0.0f);
        }

        double cw_world = geo.A * cell.cw;
        double ch_world = std::abs(geo.E) * cell.ch;
        double worldCellSize = std::min(std::abs(cw_world), std::abs(ch_world));
        double angularBaseline = worldCellSize * 0.5;
        bool geomAngleExceeded =
            geomErr > static_cast<float>(angularBaseline * ANGLE_THRESH_TAN);

        double texSpan = 0;
        if (orthoEff && orthoEff->hasGeo) {
            texSpan = std::max(cw_world / (orthoEff->A * orthoEff->width) * orthoEff->width,
                               ch_world / (std::abs(orthoEff->E) * orthoEff->height) * orthoEff->height);
        }

        return !geomAngleExceeded && !(texSpan > MAX_TEX_SPAN);
    };

    auto nodataCount = [&](const QuadCell& cell) -> int {
        bool nd00 = isNodataAt(cell.col,             cell.row);
        bool nd10 = isNodataAt(cell.col + cell.cw,   cell.row);
        bool nd01 = isNodataAt(cell.col,             cell.row + cell.ch);
        bool nd11 = isNodataAt(cell.col + cell.cw,   cell.row + cell.ch);
        bool ndC  = isNodataAt(cell.col + cell.cw*0.5, cell.row + cell.ch*0.5);
        return (nd00?1:0) + (nd10?1:0) + (nd01?1:0) + (nd11?1:0) + (ndC?1:0);
    };

    // Recursive top-down build — see dem_tess_mesh.cpp's identical
    // rewrite (kept in sync) for the full derivation of why testing
    // first against the exact pyramid bound, and only recursing on
    // failure, is now safe and early-terminating, unlike the original
    // (pre-bottom-up) top-down attempt's 5-point sampling.
    std::function<void(const QuadCell&, std::vector<QuadCell>&)> buildTopDown =
        [&](const QuadCell& cell, std::vector<QuadCell>& out) {
        int ndCount = nodataCount(cell);
        if (ndCount == 5) return; // entirely nodata: no geometry here at all

        if (ndCount == 0 && cellIsAcceptable(cell)) {
            out.push_back(cell); // passes the exact test — stop here, no exploration needed
            return;
        }

        if (cell.level >= MAX_LEVEL) {
            if (ndCount > 0 && isNodataAt(cell.col + cell.cw*0.5, cell.row + cell.ch*0.5)) return;
            out.push_back(cell);
            return;
        }

        double hw = cell.cw * 0.5, hh = cell.ch * 0.5;
        QuadCell children[4] = {
            {cell.col,      cell.row,      hw, hh, cell.level + 1},
            {cell.col + hw, cell.row,      hw, hh, cell.level + 1},
            {cell.col,      cell.row + hh, hw, hh, cell.level + 1},
            {cell.col + hw, cell.row + hh, hw, hh, cell.level + 1},
        };
        for (const auto& c : children) buildTopDown(c, out);
    };

    std::vector<QuadCell> leaves;
    leaves.reserve(1024);
    {
        double cw = static_cast<double>(w-1)/COARSE;
        double ch = static_cast<double>(h-1)/COARSE;
        // See dem_tess_mesh.cpp's identical loop for the parallelization
        // rationale (embarrassingly parallel, highly uneven per-cell
        // workload with early termination restored, dynamic scheduling).
        std::vector<std::vector<QuadCell>> perCellLeaves(
            static_cast<size_t>(COARSE) * COARSE);
#if LASVIEWER_HAS_OPENMP
        #pragma omp parallel for schedule(dynamic)
#endif
        for (int i = 0; i < COARSE * COARSE; ++i) {
            int gx = i % COARSE, gy = i / COARSE;
            buildTopDown({gx*cw, gy*ch, cw, ch, 0}, perCellLeaves[i]);
        }
        for (auto& cellLeaves : perCellLeaves) {
            leaves.insert(leaves.end(), cellLeaves.begin(), cellLeaves.end());
        }
    }
    std::cerr << "[dem-mesh] initial leaves (after top-down subdivision"
#if LASVIEWER_HAS_OPENMP
                 ", OpenMP"
#endif
                 "): " << leaves.size() << std::endl;

    // Level-distribution histogram — see dem_tess_mesh.cpp's identical
    // block for the full reasoning (a direct, checkable answer to
    // "is the collapsing angle actually doing anything," vs. MAX_LEVEL
    // being the dominant factor for genuinely noisy real terrain).
    {
        std::vector<int> levelCounts(MAX_LEVEL + 1, 0);
        for (const auto& c : leaves) {
            if (c.level >= 0 && c.level <= MAX_LEVEL) levelCounts[c.level]++;
        }
        std::cerr << "[dem-mesh] leaf level histogram (angle=" << ANGLE_THRESH_DEG
                  << "\u00b0, maxLevel=" << MAX_LEVEL << "):";
        for (int lvl = 0; lvl <= MAX_LEVEL; ++lvl) {
            std::cerr << "  L" << lvl << "=" << levelCounts[lvl];
        }
        std::cerr << std::endl;
    }

    // -------------------------------------------------------------------
    // 2. Balance pass: subdivide any leaf whose neighbor is 2+ levels
    //    finer. Repeat until stable.
    // -------------------------------------------------------------------
    // Helper: find the leaf containing the query point (col, row).
    // Returns nullptr if no leaf contains the point (outside domain).
    auto findLeafAt = [&](const std::vector<QuadCell>& lst,
                          double col, double row) -> const QuadCell* {
        for (const auto& c : lst) {
            if (col >= c.col && col <= c.col + c.cw &&
                row >= c.row && row <= c.row + c.ch) {
                return &c;
            }
        }
        return nullptr;
    };

    int balanceIters = 0;
    bool changed = true;
    while (changed && balanceIters < 40) {
        changed = false;
        std::vector<QuadCell> newLeaves;
        newLeaves.reserve(leaves.size() * 2);
        for (const auto& c : leaves) {
            bool needSub = false;
            // Check 4 neighbors. Query point just outside each edge, in the
            // middle of that edge, offset by a small epsilon to avoid landing
            // on the boundary.
            double eps = std::min(c.cw, c.ch) * 0.01;
            double midC = c.col + c.cw * 0.5;
            double midR = c.row + c.ch * 0.5;
            // Right (col + cw + eps, midR)
            // Left  (col - eps, midR)
            // Top   (midC, row + ch + eps)
            // Bottom(midC, row - eps)
            const QuadCell* nbr = nullptr;
            nbr = findLeafAt(leaves, c.col + c.cw + eps, midR);
            if (nbr && nbr->level > c.level + 1) needSub = true;
            if (!needSub) {
                nbr = findLeafAt(leaves, c.col - eps, midR);
                if (nbr && nbr->level > c.level + 1) needSub = true;
            }
            if (!needSub) {
                nbr = findLeafAt(leaves, midC, c.row + c.ch + eps);
                if (nbr && nbr->level > c.level + 1) needSub = true;
            }
            if (!needSub) {
                nbr = findLeafAt(leaves, midC, c.row - eps);
                if (nbr && nbr->level > c.level + 1) needSub = true;
            }

            if (needSub) {
                changed = true;
                double hw = c.cw * 0.5, hh = c.ch * 0.5;
                newLeaves.push_back({c.col,      c.row,      hw, hh, c.level+1});
                newLeaves.push_back({c.col + hw, c.row,      hw, hh, c.level+1});
                newLeaves.push_back({c.col,      c.row + hh, hw, hh, c.level+1});
                newLeaves.push_back({c.col + hw, c.row + hh, hw, hh, c.level+1});
            } else {
                newLeaves.push_back(c);
            }
        }
        leaves = std::move(newLeaves);
        ++balanceIters;
    }
    std::cerr << "[dem-mesh] after balance (" << balanceIters
              << " iters): " << leaves.size() << " leaves" << std::endl;

    // -------------------------------------------------------------------
    // 3. Build vertices: for each leaf, 4 corners + 1 center + (midpoint on
    //    edges with same-level or finer neighbor). Fan-triangulate from
    //    center.
    // -------------------------------------------------------------------
    // Vertex key: (col, row) snapped to 1/1024 of a pixel. This handles the
    // case where two cells compute the "same" vertex via slightly different
    // floating-point paths.
    struct VKey {
        int32_t c, r;
        bool operator==(const VKey& o) const { return c == o.c && r == o.r; }
    };
    struct VHash {
        size_t operator()(const VKey& k) const {
            return (static_cast<uint64_t>(k.c) * 1000003u) ^
                   static_cast<uint32_t>(k.r);
        }
    };
    auto makeKey = [](double col, double row) -> VKey {
        // Snap to 1/1024 of a pixel.
        return VKey{ static_cast<int32_t>(std::round(col * 1024.0)),
                     static_cast<int32_t>(std::round(row * 1024.0)) };
    };

    std::unordered_map<VKey, uint32_t, VHash> vertMap;
    vertMap.reserve(leaves.size() * 9 / 2);

    std::vector<float> verts, uvs;
    verts.reserve(leaves.size() * 9);
    uvs.reserve(leaves.size() * 6);

    auto getVert = [&](double col, double row) -> uint32_t {
        VKey key = makeKey(col, row);
        auto it = vertMap.find(key);
        if (it != vertMap.end()) return it->second;
        float elev = sampleElev(col, row);
        double wx = geo.C + geo.A * col;
        double wy = geo.F + geo.E * row;
        verts.push_back(static_cast<float>((wx - worldCenter.x) * invScale));
        verts.push_back(static_cast<float>((elev - worldCenter.z) * invScale));
        verts.push_back(static_cast<float>(-(wy - worldCenter.y) * invScale));
        auto [u, v] = uvFor(col, row);
        uvs.push_back(u); uvs.push_back(v);
        uint32_t idx = static_cast<uint32_t>(verts.size() / 3 - 1);
        vertMap[key] = idx;
        return idx;
    };

    // For each leaf, determine the neighbor level on each of its 4 edges.
    // 0=right, 1=top, 2=left, 3=bottom. -1 = no neighbor (domain boundary).
    auto neighborLevel = [&](const QuadCell& c, int side) -> int {
        double eps = std::min(c.cw, c.ch) * 0.01;
        double midC = c.col + c.cw * 0.5;
        double midR = c.row + c.ch * 0.5;
        double nc, nr;
        switch (side) {
            case 0: nc = c.col + c.cw + eps; nr = midR; break; // right
            case 1: nc = midC; nr = c.row + c.ch + eps; break; // top
            case 2: nc = c.col - eps; nr = midR; break;        // left
            case 3: nc = midC; nr = c.row - eps; break;        // bottom
            default: return -1;
        }
        const QuadCell* nbr = findLeafAt(leaves, nc, nr);
        return nbr ? nbr->level : -1;
    };

    // Build triangle indices.
    std::vector<uint32_t> tris;
    tris.reserve(leaves.size() * 4 * 3);

    for (const auto& c : leaves) {
        // 4 corners (CCW from bottom-left): c0=(col,row), c1=(col+cw,row),
        // c2=(col+cw,row+ch), c3=(col,row+ch).
        uint32_t c0 = getVert(c.col,           c.row);
        uint32_t c1 = getVert(c.col + c.cw,    c.row);
        uint32_t c2 = getVert(c.col + c.cw,    c.row + c.ch);
        uint32_t c3 = getVert(c.col,           c.row + c.ch);

        // Optional edge midpoints (only on edges with same-level or finer
        // neighbor).
        // m0 = right edge midpoint  (col+cw, row+ch/2)
        // m1 = top edge midpoint    (col+cw/2, row+ch)
        // m2 = left edge midpoint   (col, row+ch/2)
        // m3 = bottom edge midpoint (col+cw/2, row)
        bool hasM0 = false, hasM1 = false, hasM2 = false, hasM3 = false;
        uint32_t m0 = 0, m1 = 0, m2 = 0, m3 = 0;
        int nl;
        nl = neighborLevel(c, 0);
        if (nl >= 0 && nl >= c.level) {
            m0 = getVert(c.col + c.cw,       c.row + c.ch * 0.5);
            hasM0 = true;
        }
        nl = neighborLevel(c, 1);
        if (nl >= 0 && nl >= c.level) {
            m1 = getVert(c.col + c.cw * 0.5, c.row + c.ch);
            hasM1 = true;
        }
        nl = neighborLevel(c, 2);
        if (nl >= 0 && nl >= c.level) {
            m2 = getVert(c.col,              c.row + c.ch * 0.5);
            hasM2 = true;
        }
        nl = neighborLevel(c, 3);
        if (nl >= 0 && nl >= c.level) {
            m3 = getVert(c.col + c.cw * 0.5, c.row);
            hasM3 = true;
        }

        // Center vertex.
        uint32_t ctr = getVert(c.col + c.cw * 0.5, c.row + c.ch * 0.5);

        // Fan triangulation from center, walking the boundary CCW.
        // Corners in CCW order from BL: c0(BL) → c1(BR) → c2(TR) → c3(TL).
        // Each edge midpoint goes BETWEEN its two corners:
        //   c0(BL) → c1(BR) : passes through m3 (BOTTOM edge)
        //   c1(BR) → c2(TR) : passes through m0 (RIGHT edge)
        //   c2(TR) → c3(TL) : passes through m1 (TOP edge)
        //   c3(TL) → c0(BL) : passes through m2 (LEFT edge)
        // Each boundary segment becomes one triangle (ctr, b_i, b_{i+1}).
        uint32_t boundary[8];
        int nb = 0;
        boundary[nb++] = c0;
        if (hasM3) boundary[nb++] = m3;  // BOTTOM midpoint (between c0 and c1)
        boundary[nb++] = c1;
        if (hasM0) boundary[nb++] = m0;  // RIGHT midpoint (between c1 and c2)
        boundary[nb++] = c2;
        if (hasM1) boundary[nb++] = m1;  // TOP midpoint (between c2 and c3)
        boundary[nb++] = c3;
        if (hasM2) boundary[nb++] = m2;  // LEFT midpoint (between c3 and c0)

        for (int i = 0; i < nb; ++i) {
            tris.push_back(ctr);
            tris.push_back(boundary[i]);
            tris.push_back(boundary[(i + 1) % nb]);
        }
    }

    std::cerr << "[dem-mesh] adaptive: " << leaves.size() << " leaves, "
              << verts.size() / 3 << " vertices, "
              << tris.size() / 3 << " triangles" << std::endl;

    vertices = std::move(verts);
    this->uvs = std::move(uvs);
    indices = std::move(tris);

    elevs.clear();
    elevs.shrink_to_fit();

    // Compute GL-space bbox.
    float mnx = 1e30f, mxx = -1e30f, mny = 1e30f, mxy = -1e30f, mnz = 1e30f, mxz = -1e30f;
    for (size_t i = 0; i < vertices.size(); i += 3) {
        float x = vertices[i], y = vertices[i+1], z = vertices[i+2];
        if (x < mnx) mnx = x; if (x > mxx) mxx = x;
        if (y < mny) mny = y; if (y > mxy) mxy = y;
        if (z < mnz) mnz = z; if (z > mxz) mxz = z;
    }
    glBBoxMin = glm::vec2(mnx, mnz);
    glBBoxMax = glm::vec2(mxx, mxz);
    glBBoxMinY = mny;
    glBBoxMaxY = mxy;

    valid = true;
    std::cerr << "[dem-mesh] built: " << vertices.size() / 3 << " vertices, "
              << indices.size() / 3 << " triangles" << std::endl;
    return true;
}

// ---------------------------------------------------------------------------
// DEMMesh::render — draw the mesh as textured triangles.
//
// VAO/VBO/IBO are created lazily on first render (the GL context may not be
// current when loadFromDEM is called). They are file-statics so a DEMMesh
// instance has a single set of GPU buffers.
// ---------------------------------------------------------------------------

static GLuint g_meshVAO = 0, g_meshVBO = 0, g_meshUVBO = 0, g_meshIBO = 0;

void DEMMesh::render(GLuint meshProgram, const glm::mat4& V, const glm::mat4& P,
                     float zScale, bool showMasterEdges) {
    if (!valid || indices.empty()) return;

    // Create VAO on first render.
    if (g_meshVAO == 0) {
        glGenVertexArrays(1, &g_meshVAO);
        glGenBuffers(1, &g_meshVBO);
        glGenBuffers(1, &g_meshUVBO);
        glGenBuffers(1, &g_meshIBO);

        glBindVertexArray(g_meshVAO);
        glBindBuffer(GL_ARRAY_BUFFER, g_meshVBO);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float),
                     vertices.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

        glBindBuffer(GL_ARRAY_BUFFER, g_meshUVBO);
        glBufferData(GL_ARRAY_BUFFER, uvs.size() * sizeof(float),
                     uvs.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, g_meshIBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     indices.size() * sizeof(uint32_t),
                     indices.data(), GL_STATIC_DRAW);
        glBindVertexArray(0);
    }

    glUseProgram(meshProgram);
    glUniformMatrix4fv(glGetUniformLocation(meshProgram, "uView"), 1, GL_FALSE, glm::value_ptr(V));
    glUniformMatrix4fv(glGetUniformLocation(meshProgram, "uProj"), 1, GL_FALSE, glm::value_ptr(P));
    glUniform1f(glGetUniformLocation(meshProgram, "uZScale"), zScale);
    glUniform1i(glGetUniformLocation(meshProgram, "uShowMasterEdges"), showMasterEdges ? 1 : 0);

    // Pass GL-space Y range for the elevation color ramp fallback.
    glUniform1f(glGetUniformLocation(meshProgram, "uMinElev"), glBBoxMinY);
    glUniform1f(glGetUniformLocation(meshProgram, "uMaxElev"), glBBoxMaxY);

    if (texture) {
        glUniform1i(glGetUniformLocation(meshProgram, "uHasTexture"), 1);
        glUniform1i(glGetUniformLocation(meshProgram, "uTexture"), 0);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture);
    } else {
        glUniform1i(glGetUniformLocation(meshProgram, "uHasTexture"), 0);
    }

    glBindVertexArray(g_meshVAO);
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices.size()),
                   GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

void DEMMesh::destroy() {
    if (texture) glDeleteTextures(1, &texture);
    if (g_meshVAO) glDeleteVertexArrays(1, &g_meshVAO);
    if (g_meshVBO) glDeleteBuffers(1, &g_meshVBO);
    if (g_meshUVBO) glDeleteBuffers(1, &g_meshUVBO);
    if (g_meshIBO) glDeleteBuffers(1, &g_meshIBO);
    g_meshVAO = g_meshVBO = g_meshUVBO = g_meshIBO = 0;
    texture = 0;
    valid = false;
}

// ---------------------------------------------------------------------------
// DEMMesh::reload — rebuilds vertices/uvs/indices with a new point-
// collapsing angle. Deliberately does NOT call destroy() / touch
// `texture`: the orthophoto's content is unaffected by the collapsing
// angle, loadFromDEM() never touches `texture` either way, and its CPU-
// side pixel buffer is freed right after the initial upload (see
// main.cpp) — there's no data to re-upload from even if this tried to.
// Reusing the existing GPU texture unchanged is both simpler and correct.
// Only the mesh VAO/VBO/IBO are invalidated, forcing render() to recreate
// them (its existing "create on first render" check) from the rebuilt
// geometry.
// ---------------------------------------------------------------------------
bool DEMMesh::reload(const Orthophoto* ortho, double newAngleThresholdDeg, int newMaxLevel) {
    if (sourcePath.empty()) {
        std::cerr << "ERROR: DEMMesh::reload called with no prior sourcePath" << std::endl;
        return false;
    }
    std::string path = sourcePath;
    if (g_meshVAO)  { glDeleteVertexArrays(1, &g_meshVAO);  g_meshVAO = 0; }
    if (g_meshVBO)  { glDeleteBuffers(1, &g_meshVBO);       g_meshVBO = 0; }
    if (g_meshUVBO) { glDeleteBuffers(1, &g_meshUVBO);      g_meshUVBO = 0; }
    if (g_meshIBO)  { glDeleteBuffers(1, &g_meshIBO);       g_meshIBO = 0; }
    valid = false;

    if (!loadFromDEM(path, ortho, newAngleThresholdDeg, newMaxLevel)) {
        std::cerr << "ERROR: DEMMesh::reload failed to rebuild mesh" << std::endl;
        return false;
    }
    std::cerr << "[dem-mesh] reloaded, collapsing angle = " << newAngleThresholdDeg
              << "\u00b0, max level = " << newMaxLevel
              << " (" << indices.size() / 3 << " triangles)" << std::endl;
    return true;
}
