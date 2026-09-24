// dem_tess_mesh.cpp — GPU hardware-tessellated DEM/DSM mesh with
// normal-directed displacement mapping. See dem_tess_mesh.h and
// docs/design-tessellation-displacement.md for the full design.
//
// SECOND REVISION: patches now come from the same adaptive quadtree
// DEMMesh uses (see loadFromDEM() below), not a uniform grid — see the
// header banner in dem_tess_mesh.h for why the first revision's uniform
// grid was wrong (it discarded exactly the adaptivity that concentrates
// small patches around sharp features, causing visible over-smoothing on
// e.g. buildings sitting in an otherwise flat field).
//
// UNVERIFIED ON REAL GPU HARDWARE. This was written and reviewed without
// access to a GL 4.x context or a compiler with GLFW/PDAL/libtiff
// available (see specs.md §12.3 / README for the sandbox this was
// developed in). Known specific risks to check first — see
// docs/design-tessellation-displacement.md §9:
//   - GLSL quad-domain gl_TessLevelOuter[] <-> physical-edge correspondence
//     (documented per the GLSL spec below, flagged for visual verification)
//   - Real seam/crack behavior under camera motion, ESPECIALLY at
//     different-level LOD transitions on steep terrain (the tilted-normal
//     boundary residual documented in dem_tess_mesh.h's header banner)
//   - Heightmap texture bandwidth at high tessellation factors
#include "dem_tess_mesh.h"
#include "dem_io.h"
#include "geotiff.h"
// GLFW/OpenGL headers now come from dem_tess_mesh.h -> gl_platform.h (see
// that file for why this used to be a fragile per-file ad-hoc block, and
// specifically why relying on it silently broke for shaders.cpp).

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <tiffio.h>

#include <iostream>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <utility>
#include <functional>
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

// Adaptive quadtree parameters — same spirit and same starting values as
// DEMMesh's (dem_mesh.cpp), since this module's whole point is to restore
// that same adaptive concentration of detail around sharp features while
// adding GPU-side fine relief on top. Patch count = COARSE * 2^maxLevel
// per axis; a level-L leaf's world size is (initial COARSE cell) / 2^L.
//
// maxLevel is now a runtime, per-instance member (S/F keys, main.cpp) —
// see DEMTessMesh::maxLevel in dem_tess_mesh.h — rather than a fixed
// constant. History: originally 6, reduced to 3 in direct response to a
// "coarser base tessellation, ~10x fewer per axis" request, on the theory
// that GPU tessellation + displacement would gracefully fill in whatever
// detail the coarser CPU mesh no longer captured. Real-hardware testing
// showed that theory was wrong at that level of coarsening: a
// MAX_LEVEL=3 patch could span enough real-world area to contain
// substantial elevation variation entirely inside itself, contributing to
// reported displacement overhangs (since fixed more directly — see
// shaders.cpp's TES history). Walked back to 5 as a middle ground before
// being made directly user-adjustable here. COARSE (the initial grid
// before any adaptive subdivision) stays a fixed constant — only the
// depth ceiling was requested as a live control.
static const int COARSE = 8;
static const double MAX_TEX_SPAN = 64.0;

// NOTE: the actual fixed tessellation level used for constrained (LOD-
// transition) edges lives in the shader as CONSTRAINED_EDGE_TESS_LEVEL
// (src/shaders.cpp, kMeshTessControl) — the CPU side here only classifies
// each patch edge as constrained (1.0) or unconstrained (0.0) via
// patchEdgeConstraint; the shader decides what value a constrained edge
// actually gets. No CPU-side constant needed (would be unused/dead code).

// Heightmap texture is capped at this many texels (downsampled via box
// filter if the source DEM is larger), matching the spirit of the existing
// orthophoto downsample cap in geotiff.cpp::loadTIFF.
//
// This cap directly limits how much real DEM detail displacement mapping
// can ever reveal, no matter how much GPU tessellation density is dialed
// up (S/F keys) — beyond this resolution there is no more real information
// left in the texture to sample, so extra triangles just smoothly
// interpolate an already-downsampled surface instead of following the
// actual DEM. Raised from an initial 4096x4096 — kept at R32F (see
// uploadGPU()'s format comment for why R16F wasn't used despite allowing a
// larger cap for the same VRAM: it trades resolution for per-texel
// precision, which risks visible quantization in subtle-relief scenes).
// 6144x6144 costs roughly 151MB at R32F, a real but generally affordable
// VRAM cost on a discrete GPU — not verified against actual hardware
// limits. Raise further (with or without switching to R16F) if you have
// VRAM headroom and are still hitting this limit; lower it if 151MB is too
// much for your GPU.
static const int MAX_HEIGHTMAP_TEXELS = 6144 * 6144;

// ---------------------------------------------------------------------------
// demTessSupported — query the current GL context version. Must be called
// after glfwMakeContextCurrent(). GL_MAJOR_VERSION/GL_MINOR_VERSION are core
// since GL 3.0, so this query itself works even on a GL 3.3 context (it'll
// just report 3.3 and we return false).
// ---------------------------------------------------------------------------
bool demTessSupported() {
    GLint major = 0, minor = 0;
    glGetIntegerv(GL_MAJOR_VERSION, &major);
    glGetIntegerv(GL_MINOR_VERSION, &minor);
    if (glGetError() != GL_NO_ERROR) {
        // GL_MAJOR_VERSION query itself is only reliable on 3.0+; if it
        // errored we're definitely not on a tessellation-capable context.
        return false;
    }
    bool ok = (major > 4) || (major == 4 && minor >= 0);
    std::cerr << "[dem-tess] GL context version " << major << "." << minor
              << " — tessellation " << (ok ? "supported" : "NOT supported")
              << std::endl;
    return ok;
}

// ---------------------------------------------------------------------------
// DEMTessMesh::loadFromDEM — CPU-only. Reads the DEM, builds a uniform
// coarse patch grid with per-vertex normals, and keeps the full-resolution
// heightmap (pre-converted to GL-space Y) for uploadGPU() to texture.
// No GL calls here — mirrors DEMMesh::loadFromDEM's calling convention,
// which main.cpp invokes before a GL context exists.
// ---------------------------------------------------------------------------
bool DEMTessMesh::loadFromDEM(const std::string& path, const Orthophoto* ortho,
                              double angleThresholdDeg, int maxLevelParam,
                              const std::atomic<bool>* cancelFlag) {
    collapseAngleDeg = angleThresholdDeg;
    maxLevel = maxLevelParam;
    orthoUsable = true; // reset each load — this object can be reloaded with
                        // a different DEM/ortho pairing; a stale false from
                        // a previous load must not persist
    std::cerr << "[dem-tess] opening: " << path << std::endl;
    TIFF* tif = TIFFOpen(path.c_str(), "r");
    if (!tif) { std::cerr << "ERROR: could not open DEM" << std::endl; return false; }

    uint32_t w = 0, h = 0;
    TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &w);
    TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &h);
    if (w == 0 || h == 0) { TIFFClose(tif); return false; }

    Orthophoto geo;
    geo.width = static_cast<int>(w);
    geo.height = static_cast<int>(h);
    readGeoTIFFTags(tif, geo);
    if (geo.hasGeo && readEPSGCode(tif, geo.epsg)) {
        std::cerr << "[dem-tess] DEM EPSG:" << geo.epsg << std::endl;
    }
    if (!geo.hasGeo) {
        std::cerr << "ERROR: DEM has no GeoTIFF tags" << std::endl;
        TIFFClose(tif);
        return false;
    }

    std::vector<float> elevs;
    uint16_t demSpp = 1;
    if (!readDEMElevations(tif, w, h, elevs, demSpp)) {
        TIFFClose(tif);
        return false;
    }
    // Declared NODATA value (GDAL_NODATA tag) — read while the TIFF is
    // still open. See dem_mesh.h's readDEMNodataValue() declaration and
    // dem_mesh.cpp's use of it (kept in sync here) for why this matters.
    float declaredNodata = 0.0f;
    bool hasDeclaredNodata = readDEMNodataValue(tif, declaredNodata);
    if (!hasDeclaredNodata) {
        std::cerr << "[dem-tess] no declared NODATA tag — using legacy < -9000 heuristic"
                  << std::endl;
    }
    TIFFClose(tif);

    // Unified nodata test — see dem_mesh.cpp's identical helper for the
    // full reasoning. Used everywhere in this function instead of a
    // hardcoded "< -9000.0f".
    auto isNodataValue = [&](float v) -> bool {
        if (hasDeclaredNodata) {
            float tol = 1e-3f * std::max(1.0f, std::abs(declaredNodata));
            return std::abs(v - declaredNodata) < tol;
        }
        return v < -9000.0f;
    };

    float zMin = 1e30f, zMax = -1e30f;
    for (float v : elevs) {
        if (isNodataValue(v)) continue;
        if (v < zMin) zMin = v;
        if (v > zMax) zMax = v;
    }
    if (zMin > zMax) { zMin = 0; zMax = 1; }
    std::cerr << "[dem-tess] elevation: " << zMin << " .. " << zMax << std::endl;

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

    // Bilinear elevation sampler (clamps to edge) — same as DEMMesh's.
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
    // general elevation queries (heightmap texture, normal finite
    // differences) where SOME numeric value is always needed. This helper
    // is only used to decide whether geometry should exist at a location
    // at all — per the requirement that geometry (patches/points) must
    // only be drawn where there is real DEM/point data, never fabricated
    // over nodata gaps just because an orthophoto happens to cover them.
    auto isNodataAt = [&](double col, double row) -> bool {
        int c = static_cast<int>(std::lround(col));
        int r = static_cast<int>(std::lround(row));
        c = std::clamp(c, 0, static_cast<int>(w) - 1);
        r = std::clamp(r, 0, static_cast<int>(h) - 1);
        return isNodataValue(elevs[static_cast<size_t>(r) * w + c]);
    };

    // Same UV-mode determination as DEMMesh::loadFromDEM (geo-matched vs.
    // stretch-fit fallback) — kept as a small, deliberate duplication here
    // rather than factoring a shared helper, to keep this module's only
    // cross-file dependency on dem_mesh.cpp limited to readDEMElevations().
    // `ortho` itself is a shared, externally-owned, const object — also
    // read concurrently by other background builds (§6l) — so it can't be
    // mutated in place for reprojection. `orthoEff` is what the rest of
    // this function actually uses for geo-referencing math from here on;
    // it points at `ortho` unchanged unless a CRS mismatch was found and
    // corrected, in which case it points at `orthoReprojected` (a private
    // copy, reprojected into the DEM's own CRS) instead.
    Orthophoto orthoReprojected;
    const Orthophoto* orthoEff = ortho;
    if (ortho && ortho->hasGeo && geo.hasGeo && ortho->epsg != 0 && geo.epsg != 0
        && ortho->epsg != geo.epsg) {
        orthoReprojected = *ortho;
        if (reprojectToMatchCRS(orthoReprojected, geo)) {
            orthoEff = &orthoReprojected;
        }
        // If reprojection wasn't possible (no PROJ, or PROJ found no valid
        // pipeline), orthoEff stays pointed at the original `ortho` —
        // reprojectToMatchCRS() already logged why, and the existing
        // overlap check below still correctly falls back to "texturing
        // disabled" rather than silently misusing mismatched coordinates.
    }

    bool orthoStretch = false;
    if (orthoEff && !orthoEff->pixels.empty()) {
        if (!orthoEff->hasGeo) {
            orthoStretch = true;
        } else {
            double orthoMinX = orthoEff->C;
            double orthoMaxX = orthoEff->C + orthoEff->A * (orthoEff->width - 1);
            double orthoMinY = orthoEff->F + orthoEff->E * (orthoEff->height - 1);
            double orthoMaxY = orthoEff->F;
            if (orthoMinX > orthoMaxX) std::swap(orthoMinX, orthoMaxX);
            if (orthoMinY > orthoMaxY) std::swap(orthoMinY, orthoMaxY);
            double overlapMinX = std::max(minX, orthoMinX);
            double overlapMaxX = std::min(maxX, orthoMaxX);
            double overlapMinY = std::max(minY, orthoMinY);
            double overlapMaxY = std::min(maxY, orthoMaxY);
            bool hasOverlap = (overlapMinX < overlapMaxX && overlapMinY < overlapMaxY);
            if (!hasOverlap) {
                // Deliberately NOT orthoStretch = true here. There IS geo
                // metadata, and it says these two rasters don't
                // correspond — stretching the orthophoto onto the DEM
                // anyway would show imagery from a completely unrelated
                // location, actively misleading rather than merely
                // imprecise. orthoStretch (the col/(w-1),row/(h-1)
                // fallback) stays reserved for the genuinely different
                // case of NO geo metadata at all (!orthoEff->hasGeo,
                // above), where there's no correspondence information to
                // contradict in the first place. Here, orthoUsable=false
                // tells uploadGPU() to skip texturing entirely — falls
                // back to the elevation color ramp, not a fake fit. Note
                // this can still happen even after a successful
                // reprojection above — same CRS now, but the two rasters
                // genuinely don't cover the same ground.
                std::cerr << "[dem-tess] WARNING: orthophoto extent does not overlap "
                             "DEM extent — texturing disabled for this DEM "
                             "(falls back to elevation color ramp), not stretched."
                          << std::endl;
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
        double u = (w > 1) ? col / (w - 1) : 0.5;
        double v = (h > 1) ? row / (h - 1) : 0.5;
        return {static_cast<float>(u), static_cast<float>(v)};
    };

    // CRITICAL: separate from uvFor() above. uvFor() computes UV relative
    // to the ORTHOPHOTO's own geo extent (for color sampling) — the
    // heightmap texture, by contrast, always spans this DEM's OWN raster
    // extent 1:1 (see uploadGPU(), which uploads heightmapGLSpace exactly
    // as read, only ever box-filter-downsampled, never re-windowed to
    // another extent). Reusing uvFor()'s output to sample uHeightmap was a
    // real bug in the first implementation: whenever the orthophoto's geo
    // extent differs from the DEM's (the normal case, not an edge case —
    // it's exactly what the coverage/overlap checks elsewhere in this
    // codebase exist to handle), the heightmap got sampled at essentially
    // uncorrelated locations, producing large, wrong displacement deltas
    // ("dimension of displacement seems far too important") and visibly
    // broken geometry. demUVFor() is always DEM-space, independent of the
    // orthophoto entirely.
    auto demUVFor = [&](double col, double row) -> std::pair<float,float> {
        double u = (w > 1) ? col / (w - 1) : 0.5;
        double v = (h > 1) ? row / (h - 1) : 0.5;
        return {static_cast<float>(u), static_cast<float>(v)};
    };

    // Per-vertex normal from the FULL-resolution heightmap (not the coarse
    // grid), so patch-corner normals reflect true local relief direction.
    // Moved before the quadtree build (was previously defined after it)
    // because the revised subdivision criteria below need it during the
    // subdivide decision itself, not just when building final patch
    // corners.
    //
    // Derivation: the world->GL map is (wx,wy,elev) -> GL(x,y,z) where
    //   GLx = (wx - cx) * invScale         (no flip)
    //   GLy = (elev - cz) * invScale       (no flip, elev is "up")
    //   GLz = -(wy - cy) * invScale        (Y-negation, per specs.md §3.8)
    // This is a uniform scale + axis permutation/reflection (no shear), so
    // direction vectors — including normals — transform the same way as
    // position deltas; the uniform invScale factor cancels out under
    // normalize(), leaving only the axis correspondence/sign to handle:
    //   n_world (wx,wy,elev basis) = normalize(-dElev/dwx, -dElev/dwy, 1)
    //   n_GL = normalize(-dElev/dwx, 1, +dElev/dwy)
    // dElev/dwx = (dElev/dcol) / geo.A ; dElev/dwy = (dElev/drow) / geo.E
    // (dividing by geo.E, which is typically negative for north-up
    // rasters, correctly folds in the row-vs-Y-axis sign flip — no extra
    // manual negation needed).
    auto computeNormalGL = [&](double col, double row) -> glm::vec3 {
        double dcol = std::max(1.0, (w - 1) / 512.0); // finite-diff step in pixels
        double drow = std::max(1.0, (h - 1) / 512.0);
        float eL = sampleElev(col - dcol, row);
        float eR = sampleElev(col + dcol, row);
        float eD = sampleElev(col, row - drow);
        float eU = sampleElev(col, row + drow);
        double dElev_dcol = (eR - eL) / (2.0 * dcol);
        double dElev_drow = (eU - eD) / (2.0 * drow);
        double dElev_dwx = dElev_dcol / geo.A;
        double dElev_dwy = (geo.E != 0.0) ? (dElev_drow / geo.E) : 0.0;
        glm::vec3 n(static_cast<float>(-dElev_dwx), 1.0f, static_cast<float>(dElev_dwy));
        float len = glm::length(n);
        return (len > 1e-12f) ? (n / len) : glm::vec3(0, 1, 0);
    };

    // Moved here (was previously declared just before cellIsAcceptable,
    // much further down) — queryTrueMinMax(), part of the pyramid
    // machinery immediately below, needs this type, and needs to appear
    // before its first use.
    struct QuadCell { double col, row, cw, ch; int level; };

    // -------------------------------------------------------------------
    // Min/max elevation pyramid — built once, here, before the quadtree
    // traversal. This is what makes the top-down rewrite below both
    // CORRECT (no aliasing blind spot — see the traversal's own comment)
    // and FAST (early termination for flat regions, O(1) per test instead
    // of always recursing to MAX_LEVEL). Same structure as the Hi-Z depth
    // pyramid already built for occlusion culling (copc_streamer.cpp) —
    // level 0 = native DEM resolution, level L+1 = a 2x2 min/max
    // reduction of level L.
    //
    // Nodata handling: a nodata pixel contributes +inf to the min
    // pyramid and -inf to the max pyramid — the identity elements for
    // min/max respectively — so a texel that's entirely nodata correctly
    // ends up with min=+inf, max=-inf ("no valid data here") with no
    // separate validity mask needed, and a texel with ANY valid data
    // correctly reflects only the valid pixels' range, since +inf/-inf
    // never win a min/max comparison against a real value. The query
    // function below checks for this (non-finite result) and treats it
    // as "nothing to check here" — nodata cells are handled by the
    // existing, separate nodataCount()/isNodataAt() mechanism regardless,
    // which still runs first in the traversal.
    //
    // Parallelized per level (OpenMP) — each output texel depends only on
    // up to 4 texels of the previous, already-complete level, a regular,
    // branch-free, embarrassingly parallel reduction; a good SIMD/OpenMP
    // fit in a way the (irregular, branchy) quadtree traversal itself is
    // not (see the "explore parallelism and SIMD" discussion this session
    // preceding this rewrite).
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

    // Query the TRUE min/max elevation anywhere within a cell's pixel-
    // space footprint [col, col+cw) x [row, row+ch) — an O(1) lookup
    // (touches a small, bounded number of texels, typically 4-9,
    // regardless of the pyramid's total size), by picking the coarsest
    // pyramid level whose texel size doesn't exceed the cell's own size,
    // then combining min/max over every texel at that level whose region
    // overlaps the footprint. This can be SLIGHTLY conservative (a texel
    // may extend a little beyond the cell's exact edge) — deliberately:
    // that only ever means checking a marginally larger area than
    // strictly necessary, never a smaller one, so it can never miss real
    // detail. Returns (+inf, -inf) if the queried region is entirely
    // nodata (see the pyramid's own comment for why) — callers must check
    // for this and treat it as "nothing to verify here."
    auto queryTrueMinMax = [&](const QuadCell& cell) -> std::pair<float,float> {
        double cellSize = std::min(cell.cw, cell.ch);
        int lvl = 0;
        if (cellSize > 1.0) {
            lvl = static_cast<int>(std::floor(std::log2(cellSize)));
            lvl = std::clamp(lvl, 0, static_cast<int>(minPyr.size()) - 1);
        }
        int lw = pyrW[lvl], lh = pyrH[lvl];
        double texelPixels = static_cast<double>(w - 1 > 0 ? w : 1) / std::max(1, pyrW[0]);
        (void)texelPixels; // pyrW[0] == w exactly, so level-0 texel size is 1 pixel by construction
        int texelsPerSide = 1 << lvl; // level `lvl` texel spans this many level-0 pixels per side
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

    // -------------------------------------------------------------------
    // Adaptive quadtree subdivision — REVISED from the first pass, which
    // reused DEMMesh's positional geometric-error test unchanged (single
    // sample at the cell center vs. bilinear interpolation of the 4
    // corners). That test measures *deviation from planarity*, not
    // *amount of relief* — a perfectly (or near-)planar but STEEP slope
    // (a cliff face, a roof) scores near-zero error and doesn't subdivide,
    // while a genuinely FLAT area with small-amplitude noise (ground
    // texture, scan noise) scores nonzero error at whatever single pixel
    // the center sample happens to land on and over-subdivides. This is
    // exactly backwards from what's wanted, and exactly the pattern
    // reported: too many triangles on flat noisy ground, too few on steep
    // slopes. Two changes:
    //   1. Positional error now takes the MAX over 5 samples (center + 4
    //      edge midpoints vs. what bilinear interpolation predicts at each)
    //      instead of 1, reducing sensitivity to a single noisy pixel.
    //   2. NEW: a normal-variation criterion — compute the surface normal
    //      at all 4 corners and subdivide if any pair differs by more than
    //      NORMAL_ANGLE_THRESH_COS (as a cosine, so smaller = stricter).
    //      This threshold SCALES with the user-controlled collapsing angle
    //      (I/O keys, collapseAngleDeg) rather than being a second,
    //      independently-fixed constant — an earlier version fixed it at
    //      ~20° regardless of collapseAngleDeg, which meant that on real
    //      terrain with genuine local slope variation, THIS criterion
    //      alone could dominate and the user-controlled angle would have
    //      no visible effect on the final mesh (reported directly: "doesn't
    //      change displayed geometry at all") since those cells subdivided
    //      anyway via this criterion no matter what the other one was set
    //      to. This directly targets curving/steep terrain: a cell can be
    //      "flat" by the positional test (a planar cliff) while still
    //      having normals that vary a lot from corner to corner if the
    //      cell straddles a ridge, corner, or curving slope — and matters
    //      specifically because normal direction drives displacement
    //      direction (§5.3 of the design doc); a bilinear-interpolated
    //      normal across a cell where the true normal varies a lot is a
    //      poor approximation regardless of how well POSITION interpolates.
    // ANGULAR criterion (replaces an earlier magnitude-based GEOM_THRESH =
    // (zMax-zMin)*0.01, an arbitrary percentage of the DEM's total
    // elevation range): a positional error of a given ABSOLUTE size means
    // something different depending on cell size — the same 10cm bump is
    // a big deal in a 1m cell and irrelevant in a 100m one. Comparing the
    // deviation's ANGLE (how far the true sample point tips away from the
    // flat/bilinear surface the surrounding corners imply, as seen across
    // the cell's own size) is scale-invariant and is a direct, literal
    // implementation of "collapse this vertex if it's within N° of where
    // the surface would lie without it" — subdivide (i.e. DON'T collapse/
    // stay coarse) only when that angle exceeds the threshold.
    // Both thresholds are starting points, not tuned values — flagged for
    // real-hardware adjustment same as PATCH_GRID/CONSTRAINED_EDGE_TESS_LEVEL
    // were.
    //
    // TOP-DOWN SUBDIVISION AGAIN — but not a reversion to the first pass.
    // This project went top-down -> bottom-up -> top-down again, and it
    // matters why each move happened, not just which direction won.
    //
    // The ORIGINAL top-down approach (test each cell's own 5 sample
    // points, subdivide only if THAT cell's samples show excess error)
    // had a real aliasing risk: a coarse cell's error can look acceptable
    // at its own sample points while hiding genuine fine detail BETWEEN
    // them. Bottom-up collapse (previously here) avoided that by
    // construction — explore every branch all the way to MAX_LEVEL first,
    // only merge a parent back up if every one of its children
    // independently agreed it was safe — but at real, stated cost:
    // O(4^MAX_LEVEL) per initial COARSE cell, unconditionally, even over
    // enormous flat regions that a single O(1) test could have resolved
    // immediately.
    //
    // Raised directly, and correctly: bottom-up doesn't actually test
    // against the real DEM either — it uses the exact same 5-point sample
    // at every level, just at whatever depth it happens to stop
    // recursing. Its correctness is only as good as MAX_LEVEL's
    // granularity relative to the DEM's true resolution; it doesn't
    // eliminate the aliasing blind spot, it just pushes it down to a
    // finer scale and pays a lot of redundant exploration to get there.
    // The actual fix isn't which direction the tree gets built, it's
    // WHAT THE TEST MEASURES: a test against the TRUE min/max elevation
    // over a cell's entire footprint (the min/max pyramid built just
    // above, not 5 discrete points) is exact regardless of MAX_LEVEL, and
    // makes top-down's early-termination advantage safe to take: a cell
    // that provably fits (checked against every pixel in its footprint,
    // not a sample of it) can stop immediately, honestly, without
    // exploring a single child.
    //
    // The test itself (see cellFitsWithinTolerance below): a bilinear
    // surface through 4 corners is convex, so its own min/max over the
    // footprint is just min/max of the 4 corner values — call these
    // planeMin/planeMax. Given the TRUE min/max anywhere in the footprint
    // (trueMin/trueMax, from the pyramid), the worst-case deviation
    // anywhere in the footprint is bounded by
    // max(trueMax - planeMin, planeMax - trueMin) — this can be a LOOSE
    // bound (the true extremes and the plane's extremes need not occur at
    // the same point), but it is a SAFE one: if this bound is within
    // tolerance, the actual surface is GUARANTEED to fit within tolerance
    // everywhere in the footprint, not just at 5 sample points. Looseness
    // only ever means subdividing somewhat more than the tightest-possible
    // correct answer would — never less, never missing real detail.
    //
    // PARALLELISM: with early termination restored, the COARSE-grid outer
    // loop (below) is embarrassingly parallel and highly UNEVEN (a flat
    // COARSE cell now terminates in a handful of O(1) pyramid lookups; a
    // detailed one still recurses deeply) — OpenMP with dynamic
    // scheduling, matching the pattern already used elsewhere in this
    // codebase (geotiff.cpp, point_cloud.cpp) rather than introducing a
    // new threading convention.
    //
    // The angular (not magnitude-based) threshold and the normal-variation
    // criterion below are UNCHANGED from the bottom-up version — both
    // still evaluated from the cell's own 4 corners, same as before; only
    // the positional/geometric test's SOURCE OF TRUTH changed, from 5
    // discrete samples to the exact pyramid-derived bound.
    // -------------------------------------------------------------------
    const double ANGLE_THRESH_DEG = collapseAngleDeg;
    const int MAX_LEVEL = maxLevel; // S/F keys — see the member's comment in dem_tess_mesh.h
    const double ANGLE_THRESH_TAN = std::tan(ANGLE_THRESH_DEG * 3.14159265358979323846 / 180.0);
    // Scaled proportionally to collapseAngleDeg (relative to its 1.0°
    // default), NOT a second independently-fixed constant. Real bug this
    // fixes: with the two criteria previously fixed apart (ANGLE_THRESH_DEG
    // user-controlled at ~1°, this one hardcoded at ~20° regardless), on
    // real terrain with genuine local slope variation this criterion alone
    // was very plausibly the one actually triggering subdivision for most
    // cells — meaning the I/O keys could change ANGLE_THRESH_DEG all they
    // wanted and the final patch set barely changed, since those cells got
    // subdivided anyway via THIS criterion regardless of what the user set.
    // Scaling both together means the collapsing angle actually controls
    // the full subdivision decision, not just one of three OR'd criteria.
    const double NORMAL_ANGLE_THRESH_DEG =
        std::min(89.0, std::max(1.0, 20.0 * (collapseAngleDeg / 1.0)));
    const float NORMAL_ANGLE_THRESH_COS = static_cast<float>(
        std::cos(NORMAL_ANGLE_THRESH_DEG * 3.14159265358979323846 / 180.0));

    // True if `cell`, taken as a single unsplit cell, satisfies every
    // adaptive criterion (geometric angle, texture span, normal
    // variation) — i.e. does NOT need to subdivide further. Nodata is
    // handled separately by the caller (a cell straddling a nodata
    // boundary must subdivide regardless of what this returns).
    auto cellIsAcceptable = [&](const QuadCell& cell) -> bool {
        float e00 = sampleElev(cell.col, cell.row);
        float e10 = sampleElev(cell.col + cell.cw, cell.row);
        float e01 = sampleElev(cell.col, cell.row + cell.ch);
        float e11 = sampleElev(cell.col + cell.cw, cell.row + cell.ch);

        // Exact geometric test against the true DEM, not a 5-point
        // sample — see queryTrueMinMax()'s comment and the big comment
        // block above for the derivation. A bilinear surface is convex,
        // so its own min/max over the footprint is just the min/max of
        // its 4 corners.
        float planeMin = std::min({e00, e10, e01, e11});
        float planeMax = std::max({e00, e10, e01, e11});
        auto [trueMin, trueMax] = queryTrueMinMax(cell);
        float geomErr = 0.0f;
        if (std::isfinite(trueMin) && std::isfinite(trueMax)) {
            geomErr = std::max(trueMax - planeMin, planeMax - trueMin);
            geomErr = std::max(geomErr, 0.0f); // the bound is never meant to go negative; guards float noise
        }
        // else: footprint is entirely nodata (per the pyramid's +inf/-inf
        // identity-element convention) — nothing to check geometrically;
        // nodataCount()/isNodataAt() (below, and at the top of the
        // traversal) are what actually decide this cell's fate.

        // World-space cell size — see ANGLE_THRESH_TAN's comment for the
        // units assumption (horizontal CRS units == elevation units).
        double cw_world = geo.A * cell.cw;
        double ch_world = std::abs(geo.E) * cell.ch;
        double worldCellSize = std::min(std::abs(cw_world), std::abs(ch_world));
        double angularBaseline = worldCellSize * 0.5;
        bool geomAngleExceeded =
            geomErr > static_cast<float>(angularBaseline * ANGLE_THRESH_TAN);

        glm::vec3 n00 = computeNormalGL(cell.col,            cell.row);
        glm::vec3 n10 = computeNormalGL(cell.col + cell.cw,  cell.row);
        glm::vec3 n01 = computeNormalGL(cell.col,            cell.row + cell.ch);
        glm::vec3 n11 = computeNormalGL(cell.col + cell.cw,  cell.row + cell.ch);
        float minDot = std::min({
            glm::dot(n00, n10), glm::dot(n00, n01), glm::dot(n00, n11),
            glm::dot(n10, n01), glm::dot(n10, n11), glm::dot(n01, n11),
        });
        bool normalVaries = minDot < NORMAL_ANGLE_THRESH_COS;

        double texSpan = 0;
        if (orthoEff && orthoEff->hasGeo) {
            texSpan = std::max(cw_world / (orthoEff->A * orthoEff->width) * orthoEff->width,
                               ch_world / (std::abs(orthoEff->E) * orthoEff->height) * orthoEff->height);
        }

        return !geomAngleExceeded && !(texSpan > MAX_TEX_SPAN) && !normalVaries;
    };

    // Nodata classification for one cell — see isNodataAt()'s comment
    // above for the policy (geometry must only exist where there's real
    // DEM data). Returns the count of nodata samples among the 4 corners
    // + center (5 total).
    auto nodataCount = [&](const QuadCell& cell) -> int {
        bool nd00 = isNodataAt(cell.col,             cell.row);
        bool nd10 = isNodataAt(cell.col + cell.cw,   cell.row);
        bool nd01 = isNodataAt(cell.col,             cell.row + cell.ch);
        bool nd11 = isNodataAt(cell.col + cell.cw,   cell.row + cell.ch);
        bool ndC  = isNodataAt(cell.col + cell.cw*0.5, cell.row + cell.ch*0.5);
        return (nd00?1:0) + (nd10?1:0) + (nd01?1:0) + (nd11?1:0) + (ndC?1:0);
    };

    // Recursive top-down build for one subtree rooted at `cell`. Returns
    // the resulting leaves for this subtree (empty if the whole subtree
    // is nodata and should be dropped entirely). See the big comment
    // block above for why testing FIRST (against the exact pyramid
    // bound) and only recursing on failure is now safe — the earlier
    // top-down version's problem was WHAT was tested (5 discrete points),
    // not the traversal direction itself.
    std::function<void(const QuadCell&, std::vector<QuadCell>&)> buildTopDown =
        [&](const QuadCell& cell, std::vector<QuadCell>& out) {
        // Checked on every invocation — see the equivalent comment this
        // replaces for why (cheap, and unwinds a cancelled build quickly).
        if (cancelFlag && cancelFlag->load(std::memory_order_relaxed)) return;
        int ndCount = nodataCount(cell);
        if (ndCount == 5) return; // entirely nodata: no geometry here at all

        if (ndCount == 0 && cellIsAcceptable(cell)) {
            // Passes the exact test — stop here. This is the early
            // termination bottom-up collapse could never offer: no
            // exploration of this cell's children at all, and no
            // MAX_LEVEL-depth dependency for the guarantee to hold (the
            // pyramid is checked against the DEM's true resolution,
            // whatever that is, not against however deep MAX_LEVEL
            // happens to allow).
            out.push_back(cell);
            return;
        }

        if (cell.level >= MAX_LEVEL) {
            // Can't go finer regardless of what the test says. Mixed-
            // nodata at max resolution resolves by the center sample
            // alone (same policy as the bottom-up version had).
            if (ndCount > 0 && isNodataAt(cell.col + cell.cw*0.5, cell.row + cell.ch*0.5)) return;
            out.push_back(cell);
            return;
        }

        // Failed the test (or mixed nodata) and can still go finer —
        // subdivide and recurse. No merge step: unlike bottom-up, a
        // top-down traversal never needs to reassemble a coarser cell
        // from children, since it never descended past the point where
        // staying coarse stopped being justified.
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
        // Each of the COARSE*COARSE top-level cells is fully independent
        // (distinct, non-overlapping regions; all read-only against the
        // shared elevs/pyramid data) — an embarrassingly parallel outer
        // loop, same OpenMP convention already used elsewhere in this
        // codebase (geotiff.cpp, point_cloud.cpp) rather than a new
        // threading approach. dynamic scheduling, not static: with early
        // termination restored, workload per cell is now highly uneven —
        // a flat cell resolves in a handful of O(1) checks, a detailed one
        // still recurses deeply — so handing out cells one at a time as
        // threads free up matters here in a way it didn't for the flat,
        // uniform per-point loops OpenMP is already used for.
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
    std::cerr << "[dem-tess] initial leaves (after top-down subdivision"
#if LASVIEWER_HAS_OPENMP
                 ", OpenMP"
#endif
                 "): " << leaves.size() << std::endl;

    // Level-distribution histogram — a direct, checkable answer to
    // "is the collapsing angle (I/O) actually doing anything," rather
    // than trusting the mechanism blindly. If most leaves sit at exactly
    // MAX_LEVEL regardless of how far I/O is pushed, that's a real,
    // meaningful signal: it means MAX_LEVEL (S/F) — a hard ceiling the
    // angle test can never override — is the dominant factor for this
    // particular DEM, not the angle. This is expected, not necessarily a
    // bug, for genuinely noisy real terrain: natural ground has SOME
    // roughness at nearly every scale, so a bottom-up merge test can
    // keep failing at the finest levels regardless of a "reasonable"
    // angle value (0.5°-5°, say) — only a substantially looser angle
    // would let noisy terrain merge at all. See the histogram below to
    // check this empirically for your actual data instead of guessing.
    {
        std::vector<int> levelCounts(MAX_LEVEL + 1, 0);
        for (const auto& c : leaves) {
            if (c.level >= 0 && c.level <= MAX_LEVEL) levelCounts[c.level]++;
        }
        std::cerr << "[dem-tess] leaf level histogram (angle=" << ANGLE_THRESH_DEG
                  << "\u00b0, maxLevel=" << MAX_LEVEL << "):";
        for (int lvl = 0; lvl <= MAX_LEVEL; ++lvl) {
            std::cerr << "  L" << lvl << "=" << levelCounts[lvl];
        }
        std::cerr << std::endl;
    }

    if (cancelFlag && cancelFlag->load(std::memory_order_relaxed)) {
        return false; // superseded — no point doing the balance pass or
                       // patch-building work below on a result about to
                       // be discarded anyway
    }

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
        if (cancelFlag && cancelFlag->load(std::memory_order_relaxed)) return false;
        changed = false;
        std::vector<QuadCell> newLeaves;
        newLeaves.reserve(leaves.size() * 2);
        for (const auto& c : leaves) {
            bool needSub = false;
            double eps = std::min(c.cw, c.ch) * 0.01;
            double midC = c.col + c.cw * 0.5;
            double midR = c.row + c.ch * 0.5;
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
    std::cerr << "[dem-tess] after balance (" << balanceIters
              << " iters): " << leaves.size() << " leaves" << std::endl;

    // computeNormalGL is still used above (cellIsAcceptable's normalVaries
    // criterion during the subdivide decision) but its result is no longer
    // uploaded per-corner to the GPU — see below and shaders.cpp's TES:
    // displacement is now a direct vertical (Y-only) correction against
    // the true heightmap value, not a projection along an interpolated
    // surface normal, so there's no remaining GPU-side consumer of
    // per-vertex normals. (Requested directly: "the triangles vertices
    // must sit at the dem provided altitude" — the along-normal approach
    // only ever landed on the tangent PLANE through that point, not the
    // point itself, except when the surface was exactly flat.)

    // -------------------------------------------------------------------
    // Build the flat, non-indexed patch buffer: one GL_PATCHES quad per
    // leaf. For each of its 4 edges, classify the neighbor (same-level/
    // domain-boundary -> unconstrained=0; different-level -> constrained=1,
    // see dem_tess_mesh.h for what each value means to the TCS).
    // -------------------------------------------------------------------
    patchPositions.clear(); patchUVs.clear();
    patchHeightUVs.clear();
    patchEdgeConstraint.clear();
    patchPositions.reserve(leaves.size() * 4 * 3);
    patchUVs.reserve(leaves.size() * 4 * 2);
    patchHeightUVs.reserve(leaves.size() * 4 * 2);
    patchEdgeConstraint.reserve(leaves.size() * 4 * 4);

    auto addCorner = [&](double col, double row) {
        float elev = sampleElev(col, row);
        double wx = geo.C + geo.A * col;
        double wy = geo.F + geo.E * row;
        patchPositions.push_back(static_cast<float>((wx - worldCenter.x) * invScale));
        patchPositions.push_back(static_cast<float>((elev - worldCenter.z) * invScale));
        patchPositions.push_back(static_cast<float>(-(wy - worldCenter.y) * invScale));
        auto [u, v] = uvFor(col, row);
        patchUVs.push_back(u); patchUVs.push_back(v);
        auto [hu, hv] = demUVFor(col, row);
        patchHeightUVs.push_back(hu); patchHeightUVs.push_back(hv);
    };

    for (const auto& c : leaves) {
        double eps = std::min(c.cw, c.ch) * 0.01;
        double midC = c.col + c.cw * 0.5;
        double midR = c.row + c.ch * 0.5;
        // side order matches patchEdgeConstraint's x=bottom,y=right,z=top,w=left
        auto edgeConstraintFor = [&](double nc, double nr) -> float {
            const QuadCell* nbr = findLeafAt(leaves, nc, nr);
            if (!nbr) return 0.0f;               // domain boundary: free
            return (nbr->level != c.level) ? 1.0f : 0.0f;  // different level: constrained
        };
        float cRight  = edgeConstraintFor(c.col + c.cw + eps, midR);
        float cTop    = edgeConstraintFor(midC, c.row + c.ch + eps);
        float cLeft   = edgeConstraintFor(c.col - eps, midR);
        float cBottom = edgeConstraintFor(midC, c.row - eps);

        // CCW from bottom-left: 0=BL,1=BR,2=TR,3=TL — matches
        // DEMMesh's corner convention (dem_mesh.cpp's getVert() usage).
        addCorner(c.col,          c.row);
        addCorner(c.col + c.cw,   c.row);
        addCorner(c.col + c.cw,   c.row + c.ch);
        addCorner(c.col,          c.row + c.ch);
        for (int k = 0; k < 4; ++k) {
            patchEdgeConstraint.push_back(cBottom);
            patchEdgeConstraint.push_back(cRight);
            patchEdgeConstraint.push_back(cTop);
            patchEdgeConstraint.push_back(cLeft);
        }
    }
    patchCount = static_cast<int>(leaves.size());

    // Keep the full-res heightmap, pre-converted to GL-space Y, for
    // uploadGPU()'s texture. Nodata already resolved to 0 by sampleElev's
    // neighbors, but the raw array itself hasn't been clamped — do that
    // here so the TES's coarse/fine delta never sees a nodata sentinel
    // value.
    heightmapSrcW = static_cast<int>(w);
    heightmapSrcH = static_cast<int>(h);
    heightmapGLSpace.resize(elevs.size());
    for (size_t i = 0; i < elevs.size(); ++i) {
        float e = elevs[i];
        if (isNodataValue(e)) e = 0.0f;
        heightmapGLSpace[i] = static_cast<float>((e - worldCenter.z) * invScale);
    }

    // GL-space bbox (from patch corners — displacement can locally exceed
    // this by the same amount DEMMesh's per-vertex elevation already would
    // have captured exactly; this is a coarse approximation feeding
    // dynamic near/far, same caveat as DEMMesh's own bbox).
    float mnx=1e30f, mxx=-1e30f, mny=1e30f, mxy=-1e30f, mnz=1e30f, mxz=-1e30f;
    for (size_t i = 0; i < patchPositions.size(); i += 3) {
        float x = patchPositions[i], y = patchPositions[i+1], z = patchPositions[i+2];
        if (x<mnx) mnx=x; if (x>mxx) mxx=x;
        if (y<mny) mny=y; if (y>mxy) mxy=y;
        if (z<mnz) mnz=z; if (z>mxz) mxz=z;
    }
    glBBoxMin = glm::vec2(mnx, mnz);
    glBBoxMax = glm::vec2(mxx, mxz);
    glBBoxMinY = mny;
    glBBoxMaxY = mxy;

    loaded = true;
    std::cerr << "[dem-tess] built: " << leaves.size() << " adaptive patches ("
              << patchPositions.size() / 3 << " corner vertices, unshared)"
              << std::endl;
    return true;
}

// ---------------------------------------------------------------------------
// DEMTessMesh::uploadGPU — GL context MUST be current. Uploads patch VBOs/
// IBO, the heightmap texture (downsampled if oversized), and the color
// texture (if an orthophoto was provided).
// ---------------------------------------------------------------------------
bool DEMTessMesh::uploadGPU(const Orthophoto* ortho) {
    if (!loaded) {
        std::cerr << "ERROR: DEMTessMesh::uploadGPU called before loadFromDEM" << std::endl;
        return false;
    }

    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &posVBO);
    glGenBuffers(1, &uvVBO);
    glGenBuffers(1, &heightUVVBO);
    glGenBuffers(1, &edgeConstraintVBO);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, posVBO);
    glBufferData(GL_ARRAY_BUFFER, patchPositions.size() * sizeof(float),
                 patchPositions.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

    // NOTE: attribute location 1 is deliberately unused (was aNormal —
    // removed along with the along-normal displacement approach it fed;
    // see the TES's history in this file for why). Left as a gap rather
    // than renumbering locations 2-4, to keep this a minimal, low-risk
    // diff rather than touching every attribute location across both the
    // CPU upload code and the shader source for no functional benefit.

    glBindBuffer(GL_ARRAY_BUFFER, uvVBO);
    glBufferData(GL_ARRAY_BUFFER, patchUVs.size() * sizeof(float),
                 patchUVs.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);

    // Separate from uvVBO above — DEM-raster-relative UV for heightmap
    // sampling only, never the orthophoto-relative one. See dem_tess_mesh.h
    // and demUVFor() in loadFromDEM() for why these must not be conflated.
    glBindBuffer(GL_ARRAY_BUFFER, heightUVVBO);
    glBufferData(GL_ARRAY_BUFFER, patchHeightUVs.size() * sizeof(float),
                 patchHeightUVs.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);

    glBindBuffer(GL_ARRAY_BUFFER, edgeConstraintVBO);
    glBufferData(GL_ARRAY_BUFFER, patchEdgeConstraint.size() * sizeof(float),
                 patchEdgeConstraint.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);

    glBindVertexArray(0);

    // --- Heightmap texture, capped at MAX_HEIGHTMAP_TEXELS.
    if (!uploadHeightmapTexture(heightmapGLSpace, heightmapSrcW, heightmapSrcH,
                                MAX_HEIGHTMAP_TEXELS)) {
        return false;
    }

    // Not needed on the CPU after upload; rebuilds re-read the file.
    heightmapGLSpace.clear();
    heightmapGLSpace.shrink_to_fit();

    // --- Color texture (orthophoto), same RGBA8 upload main.cpp already
    // does for DEMMesh — done here instead so main.cpp's DEM-tessellation
    // integration stays to a handful of lines (see main.cpp). Gated on
    // orthoUsable too, not just having pixels: see that field's comment
    // in dem_tess_mesh.h for why an orthophoto whose own geo metadata says
    // it doesn't overlap the DEM should render untextured (elevation
    // ramp), not stretched to fit anyway. ---
    if (!orthoUsable && colorTex) {
        glDeleteTextures(1, &colorTex);
        colorTex = 0;
    }
    if (ortho && !ortho->pixels.empty() && orthoUsable && colorTex == 0) {
        glGenTextures(1, &colorTex);
        glBindTexture(GL_TEXTURE_2D, colorTex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, ortho->width, ortho->height, 0,
                     GL_RGBA, GL_UNSIGNED_BYTE, ortho->pixels.data());
        glGenerateMipmap(GL_TEXTURE_2D);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        std::cerr << "[dem-tess] color texture uploaded: " << ortho->width
                  << "x" << ortho->height << std::endl;
    } else if (ortho && !ortho->pixels.empty() && !orthoUsable) {
        std::cerr << "[dem-tess] orthophoto present but does not overlap this "
                     "DEM — skipping texture, using elevation color ramp"
                  << std::endl;
    }

    glBindTexture(GL_TEXTURE_2D, 0);
    valid = true;
    return true;
}

// ---------------------------------------------------------------------------
// DEMTessMesh::uploadHeightmapTexture — downsample (box filter, fractional
// mapping — NOT naive integer stride, see specs.md §6.3 for why that
// matters) to capTexels if needed, then upload as a single-channel float
// texture. Deletes any previously-uploaded heightmapTex first.
// ---------------------------------------------------------------------------
bool DEMTessMesh::uploadHeightmapTexture(const std::vector<float>& glSpaceData,
                                         int srcW, int srcH, int capTexels) {
    const std::vector<float>* srcData = &glSpaceData;
    std::vector<float> downsampled;
    int dW = srcW, dH = srcH;
    if (static_cast<int64_t>(srcW) * srcH > capTexels) {
        float scale = std::sqrt(static_cast<float>(capTexels) /
                                (static_cast<float>(srcW) * srcH));
        int dstW = std::max(1, static_cast<int>(srcW * scale));
        int dstH = std::max(1, static_cast<int>(srcH * scale));
        downsampled.resize(static_cast<size_t>(dstW) * dstH);
        for (int dy = 0; dy < dstH; ++dy) {
            double sy0 = (static_cast<double>(dy) / dstH) * srcH;
            double sy1 = (static_cast<double>(dy + 1) / dstH) * srcH;
            int iy0 = std::max(0, static_cast<int>(sy0));
            int iy1 = std::min(srcH - 1, static_cast<int>(std::ceil(sy1)) - 1);
            if (iy1 < iy0) iy1 = iy0;
            for (int dx = 0; dx < dstW; ++dx) {
                double sx0 = (static_cast<double>(dx) / dstW) * srcW;
                double sx1 = (static_cast<double>(dx + 1) / dstW) * srcW;
                int ix0 = std::max(0, static_cast<int>(sx0));
                int ix1 = std::min(srcW - 1, static_cast<int>(std::ceil(sx1)) - 1);
                if (ix1 < ix0) ix1 = ix0;
                double sum = 0.0; int count = 0;
                for (int sy = iy0; sy <= iy1; ++sy) {
                    for (int sx = ix0; sx <= ix1; ++sx) {
                        sum += glSpaceData[static_cast<size_t>(sy) * srcW + sx];
                        ++count;
                    }
                }
                downsampled[static_cast<size_t>(dy) * dstW + dx] =
                    static_cast<float>(count > 0 ? sum / count : 0.0);
            }
        }
        srcData = &downsampled;
        dW = dstW; dH = dstH;
        std::cerr << "[dem-tess] heightmap downsampled: " << srcW << "x" << srcH
                  << " -> " << dW << "x" << dH << std::endl;
    }

    if (heightmapTex) { glDeleteTextures(1, &heightmapTex); heightmapTex = 0; }
    glGenTextures(1, &heightmapTex);
    glBindTexture(GL_TEXTURE_2D, heightmapTex);
    // Kept as R32F rather than R16F — see the precision-tradeoff comment
    // this replaced (still applies): a coarser cap traded for correctness
    // was judged safer than more resolution with a real quantization risk
    // in the TES's coarse/fine subtraction, unverified either way on real
    // hardware.
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, dW, dH, 0, GL_RED, GL_FLOAT, srcData->data());
    // Mipmapped — the TES samples a density-dependent coarse mip level
    // (see kMeshTessEval in shaders.cpp) so displacement magnitude
    // actually shrinks as tessellation density increases toward native
    // resolution, instead of the "coarse" baseline always being the same
    // far-corner bilinear guess regardless of density.
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);

    return true;
}

// ---------------------------------------------------------------------------
// DEMTessMesh::render
// ---------------------------------------------------------------------------
void DEMTessMesh::render(GLuint tessProgram, const glm::mat4& V, const glm::mat4& P,
                         const glm::vec3& camPosGL, float fov, float viewportH,
                         float zScale, float targetPixelsPerSegment,
                         bool useDisplacement, bool showMasterEdges) const {
    if (!valid || tessProgram == 0) return;

    glUseProgram(tessProgram);
    glUniformMatrix4fv(glGetUniformLocation(tessProgram, "uView"), 1, GL_FALSE, glm::value_ptr(V));
    glUniformMatrix4fv(glGetUniformLocation(tessProgram, "uProj"), 1, GL_FALSE, glm::value_ptr(P));
    glUniform1f(glGetUniformLocation(tessProgram, "uZScale"), zScale);
    glUniform3fv(glGetUniformLocation(tessProgram, "uCamPos"), 1, glm::value_ptr(camPosGL));
    glUniform1f(glGetUniformLocation(tessProgram, "uViewportH"), viewportH);
    glUniform1i(glGetUniformLocation(tessProgram, "uShowMasterEdges"), showMasterEdges ? 1 : 0);
    float tanHalfFov = std::tan(glm::radians(fov) * 0.5f);
    glUniform1f(glGetUniformLocation(tessProgram, "uTanHalfFov"), tanHalfFov);

    // targetPixelsPerSegment (S/F-controlled triangle density, main.cpp)
    // drives the free formula directly. The constrained-edge (LOD-
    // transition) level must move in lockstep with it — both need to scale
    // by the same factor relative to their design-doc defaults (8px and
    // 4.0 respectively) for a constrained edge to stay crack-free at any
    // density setting, since both sides of that edge read the SAME
    // uConstrainedEdgeTessLevel uniform value each frame regardless of
    // what density is currently selected.
    // Lower bound dropped from 0.5 to 0.01 — 0.5 silently defeated the
    // density ceiling raise in gl_app.cpp's 'f' case (128x density gives
    // targetPixelsPerSegment as low as 0.0625, which a 0.5 floor here
    // would have clamped straight back up to 0.5, making that fix a
    // no-op). The floor still exists to avoid a literal division-by-zero-
    // adjacent value reaching the shader, not to cap how fine density can
    // usefully go.
    float clampedTargetPx = glm::clamp(targetPixelsPerSegment, 0.01f, 64.0f);
    glUniform1f(glGetUniformLocation(tessProgram, "uTargetPixelsPerSegment"), clampedTargetPx);
    float constrainedLevel = glm::clamp(4.0f * (8.0f / clampedTargetPx), 1.0f, 64.0f);
    glUniform1f(glGetUniformLocation(tessProgram, "uConstrainedEdgeTessLevel"), constrainedLevel);

    glUniform1i(glGetUniformLocation(tessProgram, "uDisplacementEnabled"), useDisplacement ? 1 : 0);

    glUniform1i(glGetUniformLocation(tessProgram, "uHeightmap"), 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, heightmapTex);

    glUniform1f(glGetUniformLocation(tessProgram, "uMinElev"), glBBoxMinY);
    glUniform1f(glGetUniformLocation(tessProgram, "uMaxElev"), glBBoxMaxY);

    if (colorTex) {
        glUniform1i(glGetUniformLocation(tessProgram, "uHasTexture"), 1);
        glUniform1i(glGetUniformLocation(tessProgram, "uTexture"), 1);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, colorTex);
    } else {
        glUniform1i(glGetUniformLocation(tessProgram, "uHasTexture"), 0);
    }

    glPatchParameteri(GL_PATCH_VERTICES, 4);
    glBindVertexArray(vao);
    glDrawArrays(GL_PATCHES, 0, patchCount * 4);
    glBindVertexArray(0);
}

void DEMTessMesh::releaseGeometryGL() {
    if (heightmapTex) glDeleteTextures(1, &heightmapTex);
    if (vao) glDeleteVertexArrays(1, &vao);
    if (posVBO) glDeleteBuffers(1, &posVBO);
    if (uvVBO) glDeleteBuffers(1, &uvVBO);
    if (heightUVVBO) glDeleteBuffers(1, &heightUVVBO);
    if (edgeConstraintVBO) glDeleteBuffers(1, &edgeConstraintVBO);
    vao = posVBO = uvVBO = heightUVVBO = edgeConstraintVBO = 0;
    heightmapTex = 0;
    valid = false;
}

void DEMTessMesh::destroy() {
    releaseGeometryGL();
    if (colorTex) glDeleteTextures(1, &colorTex);
    colorTex = 0;
}

// ---------------------------------------------------------------------------
// Background rebuild — see the header's comment on requestBackgroundBuild()/
// pollBackgroundBuild() for the full mechanism and reasoning.
// ---------------------------------------------------------------------------

DEMTessMesh::~DEMTessMesh() {
    // Block until any in-flight background build finishes before this
    // object is destroyed — the background thread captures `this` and
    // would otherwise risk touching freed memory. Same reasoning as
    // TileGrid::stop()/~TileGrid() in copc_streamer.cpp for the point-
    // cloud streaming thread. Does NOT call destroy() here — GPU resource
    // cleanup stays main.cpp's explicit responsibility, unchanged, since
    // destroying GL objects requires a current context, which isn't
    // guaranteed at arbitrary destruction time.
    if (bgThread.joinable()) bgThread.join();
}

void DEMTessMesh::requestBackgroundBuild(const std::string& path, const Orthophoto* ortho,
                                         double angleThresholdDeg, int maxLevelParam) {
    if (bgInProgress.load()) {
        // The currently running build is now obsolete — it's about to be
        // replaced by this request — so signal it to abort rather than
        // let it run to completion just to be discarded (an earlier
        // version of this mechanism did exactly that; changed on direct
        // request: "when a background task is obsolete it must be
        // stopped before starting the new one that rendered it
        // obsolete"). Cooperative, not immediate — loadFromDEM()'s
        // bottom-up collapse (dem_tess_mesh.cpp) checks this flag
        // periodically and stops early; see its own comment for why that
        // unwinds quickly rather than instantly.
        if (bgCancelFlag) {
            bgCancelFlag->store(true, std::memory_order_relaxed);
        }
        // Coalesce: remember these as the latest-requested params. The
        // actual new thread starts once the (now aborting) old one has
        // actually exited — see pollBackgroundBuild() — not immediately;
        // starting a second thread concurrently with the still-unwinding
        // old one would defeat "at most one background thread alive at a
        // time" for no benefit, since the old one is aborting quickly
        // anyway.
        bgHasPendingRequest = true;
        bgPendingPath = path;
        bgPendingOrtho = ortho;
        bgPendingAngle = angleThresholdDeg;
        bgPendingMaxLevel = maxLevelParam;
        return;
    }
    startBackgroundBuildNow(path, ortho, angleThresholdDeg, maxLevelParam);
}

void DEMTessMesh::startBackgroundBuildNow(const std::string& path, const Orthophoto* ortho,
                                          double angleThresholdDeg, int maxLevelParam) {
    // The only thread that could possibly be joinable here has already
    // finished (we only reach this point when bgInProgress is false,
    // which the background thread itself sets — as the very last thing it
    // does, right before returning — see the lambda below), so this join
    // is near-instant, not a real block.
    if (bgThread.joinable()) bgThread.join();

    // A fresh flag per build, owned via shared_ptr so reassigning
    // `bgCancelFlag` for a LATER build never affects an EARLIER build's
    // thread, which keeps its own captured copy alive independently (see
    // the member's comment in dem_tess_mesh.h).
    bgCancelFlag = std::make_shared<std::atomic<bool>>(false);
    auto cancelFlag = bgCancelFlag;

    bgInProgress = true;
    std::cerr << "[dem-tess] background rebuild START (angle="
              << angleThresholdDeg << "\u00b0, maxLevel=" << maxLevelParam
              << ")" << std::endl;
    bgThread = std::thread([this, path, ortho, angleThresholdDeg, maxLevelParam, cancelFlag]() {
        // Builds an entirely separate, temporary instance — reuses
        // loadFromDEM() completely unchanged. This touches no GL state and
        // no member of `this`, so it's safe to run concurrently with the
        // main thread rendering `this`'s CURRENT (old) data.
        auto tmp = std::make_unique<DEMTessMesh>();
        bool ok = tmp->loadFromDEM(path, ortho, angleThresholdDeg, maxLevelParam,
                                   cancelFlag.get());
        bool wasCancelled = cancelFlag->load(std::memory_order_relaxed);
        if (wasCancelled) {
            std::cerr << "[dem-tess] background rebuild ABORTED (superseded)"
                      << std::endl;
        } else if (ok) {
            std::cerr << "[dem-tess] background rebuild FINISHED ("
                      << tmp->patchCount << " patches, not yet swapped in)"
                      << std::endl;
            std::lock_guard<std::mutex> lk(bgMutex);
            bgPending = std::move(tmp);
            bgHasResult = true;
        } else {
            std::cerr << "[dem-tess] background rebuild FAILED" << std::endl;
        }
        bgInProgress = false;
    });
}

bool DEMTessMesh::pollBackgroundBuild(const Orthophoto* ortho) {
    bool swapped = false;
    if (bgHasResult.load()) {
        std::unique_ptr<DEMTessMesh> pending;
        {
            std::lock_guard<std::mutex> lk(bgMutex);
            pending = std::move(bgPending);
            bgHasResult = false;
        }
        if (pending && pending->loaded) {
            // Move the CPU-side build results from the temporary instance
            // into this one. GPU-side members (vao, textures, etc.) are
            // deliberately NOT touched here — uploadGPU() below replaces
            // them properly (it already deletes any previous ones first).
            collapseAngleDeg = pending->collapseAngleDeg;
            maxLevel = pending->maxLevel;
            patchPositions = std::move(pending->patchPositions);
            patchUVs = std::move(pending->patchUVs);
            patchHeightUVs = std::move(pending->patchHeightUVs);
            patchEdgeConstraint = std::move(pending->patchEdgeConstraint);
            patchCount = pending->patchCount;
            heightmapGLSpace = std::move(pending->heightmapGLSpace);
            heightmapSrcW = pending->heightmapSrcW;
            heightmapSrcH = pending->heightmapSrcH;
            worldCenter = pending->worldCenter;
            worldScale = pending->worldScale;
            bboxMin = pending->bboxMin;
            bboxMax = pending->bboxMax;
            glBBoxMin = pending->glBBoxMin;
            glBBoxMax = pending->glBBoxMax;
            glBBoxMinY = pending->glBBoxMinY;
            glBBoxMaxY = pending->glBBoxMaxY;
            orthoUsable = pending->orthoUsable;
            loaded = true;
            releaseGeometryGL();
            swapped = uploadGPU(ortho);
            std::cerr << "[dem-tess] background rebuild SWAPPED IN ("
                      << patchCount << " patches)" << std::endl;
        }
    }
    // If a newer request came in while we were building, start it now
    // that the previous build has finished (bgInProgress went false
    // inside the background thread, right before it exited).
    if (!bgInProgress.load() && bgHasPendingRequest) {
        bgHasPendingRequest = false;
        startBackgroundBuildNow(bgPendingPath, bgPendingOrtho, bgPendingAngle, bgPendingMaxLevel);
    }
    return swapped;
}
