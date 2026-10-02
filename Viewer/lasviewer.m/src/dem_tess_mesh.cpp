// dem_tess_mesh.cpp — GPU-tessellated DEM/DSM mesh with height
// displacement. See dem_tess_mesh.h and
// docs/design-tessellation-displacement.md (§6v for the current version).
//
// Verified on an Apple GPU (GL 4.1): crack-free at level transitions (a
// render from below the terrain shows no background through it; with the
// old 1:1 transition rule it did), vertices on the DEM, no hanging edges at
// nodata.
#include "dem_tess_mesh.h"
#include "dem_quadtree.h"
#include "raster.h"
// GLFW/OpenGL headers now come from dem_tess_mesh.h -> gl_platform.h (see
// that file for why this used to be a fragile per-file ad-hoc block, and
// specifically why relying on it silently broke for shaders.cpp).

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

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

// Adaptive quadtree parameters. Patch count = COARSE * 2^maxLevel
// per axis; a level-L leaf's world size is (initial COARSE cell) / 2^L.
//
// maxLevel is a runtime, per-instance member (DemLayer: F/S keys, UI) —
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
// GPU tessellation splits a patch edge into at most this many segments
// (GL's guaranteed minimum gl_MaxTessGenLevel), so a patch spans at most
// this many DEM pixels: every pixel stays reachable.
static const double MAX_TESS_SEGMENTS = 64.0;

// NOTE: the actual fixed tessellation level used for constrained (LOD-
// transition) edges lives in the shader as CONSTRAINED_EDGE_TESS_LEVEL
// (src/shaders.cpp, kMeshTessControl) — the CPU side here only classifies
// each patch edge as constrained (1.0) or unconstrained (0.0) via
// patchEdgeConstraint; the shader decides what value a constrained edge
// actually gets. No CPU-side constant needed (would be unused/dead code).

// Heightmap texture is capped at this many texels (downsampled via box
// filter if the source DEM is larger), matching the spirit of the existing
// orthophoto downsample cap in raster.cpp::loadOrthophoto.
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
// DEMTessMesh::loadFromDEM — CPU-only (no GL calls; runs on the background
// build thread). Reads the DEM, builds the adaptive patch set, and keeps the
// full-resolution heightmap (in GL-space Y) for uploadGPU() to texture.
// ---------------------------------------------------------------------------
bool DEMTessMesh::loadFromDEM(const DemSource& source, const Orthophoto* ortho,
                              const SceneFrame& frame_, double angleThresholdDeg,
                              int maxLevelParam,
                              const std::atomic<bool>* cancelFlag) {
    frame = frame_;
    collapseAngleDeg = angleThresholdDeg;
    maxLevel = maxLevelParam;
    orthoUsable = true; // reset each load — this object can be reloaded with
                        // a different DEM/ortho pairing; a stale false from
                        // a previous load must not persist
    DemSourceData data;
    if (!source(data)) return false;
    const DemRaster& dem = data.dem;
    if (!data.aux.empty() &&
        data.aux.size() != static_cast<size_t>(dem.width) * static_cast<size_t>(dem.height)) {
        std::cerr << "ERROR: DEM auxiliary raster does not match the DEM grid" << std::endl;
        return false;
    }
    const RasterGeo& geo = dem;
    const uint32_t w = static_cast<uint32_t>(dem.width);
    const uint32_t h = static_cast<uint32_t>(dem.height);
    const std::vector<float>& elevs = dem.elevations;
    auto isNodataValue = [&](float v) -> bool { return dem.isNodata(v); };

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
    const glm::dvec3 worldCenter = frame.center;
    double invScale = 1.0 / frame.scale;



    // UV mode: geo-matched when the ortho is georeferenced, stretch-fit
    // over the DEM extent otherwise. The ortho is already in the scene CRS
    // (raster.cpp warps it on load), as is this DEM.
    const Orthophoto* orthoEff = ortho;

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
                // Georeferenced but not overlapping: use the elevation ramp
                // rather than stretch unrelated imagery over the DEM
                // (stretching is only for an ortho with no georeferencing).
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
            // Ortho pixel (fractional, pixel-centre convention) under this
            // DEM point, then texel-centre texture coordinates: pixel i's
            // centre is at (i + 0.5) / width.
            double pc = (orthoEff->A != 0) ? ((geo.C + geo.A * col) - orthoEff->C) / orthoEff->A : 0;
            double pr = (orthoEff->E != 0) ? ((geo.F + geo.E * row) - orthoEff->F) / orthoEff->E : 0;
            return {static_cast<float>((pc + 0.5) / orthoEff->width),
                    static_cast<float>((pr + 0.5) / orthoEff->height)};
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
    //
    // Pixel (col, row)'s centre is texel centre ((col + 0.5) / w, ...). The
    // earlier col / (w - 1) put pixel centres up to half a pixel off the
    // texel centres (0 in the middle of the DEM, ±0.5 px at its edges). The
    // box downsample in uploadHeightmapTexture() keeps the raster's full
    // extent, so the same mapping holds for a downsampled heightmap.
    auto demUVFor = [&](double col, double row) -> std::pair<float,float> {
        return {static_cast<float>((col + 0.5) / w), static_cast<float>((row + 0.5) / h)};
    };

    // -------------------------------------------------------------------
    // Patches: an adaptive quadtree (dem_quadtree.h). A cell stays whole
    // when the flat patch through its 4 corners is within the collapse
    // angle of the DEM everywhere inside it (exact, every pixel checked) and
    // is small enough for GPU tessellation to reach every DEM pixel. Leaves
    // are then balanced (neighbours at most one level apart, checked along
    // whole edges) and each edge gets a code telling the TCS how to split it
    // so that both sides of a level transition produce the same vertices.
    // -------------------------------------------------------------------
    // Nodata pixels are filled with the nearest valid height: patch corners
    // and tessellated vertices next to the data's edge then stay level with
    // it (they used to take 0 m, hanging curtains down to sea level); the
    // nodata mask still drives the quadtree, and the fragment shader cuts
    // the surface at the data's edge (heightmap channel G, below).
    std::vector<uint8_t> nodataMask(elevs.size());
    for (size_t i = 0; i < elevs.size(); ++i) nodataMask[i] = isNodataValue(elevs[i]) ? 1 : 0;
    std::vector<float> filled(elevs);
    fillNodataNearest(filled, nodataMask, static_cast<int>(w), static_cast<int>(h));
    const DemGrid grid{filled.data(), nodataMask.data(), static_cast<int>(w), static_cast<int>(h)};
    const double cw0 = static_cast<double>(w - 1) / COARSE;
    const double ch0 = static_cast<double>(h - 1) / COARSE;

    std::vector<QuadCell> leaves =
        buildLeaves(grid, COARSE, maxLevel, collapseAngleDeg, std::abs(geo.A), std::abs(geo.E),
                    MAX_TESS_SEGMENTS, cancelFlag);
    if (cancelFlag && cancelFlag->load(std::memory_order_relaxed)) return false;
    auto logHistogram = [&](const char* what) {
        std::vector<int> counts(maxLevel + 1, 0);
        for (const QuadCell& c : leaves) counts[c.level]++;
        std::cerr << "[dem-tess] " << what << ": " << leaves.size() << " leaves (angle="
                  << collapseAngleDeg << "\u00b0, maxLevel=" << maxLevel << "):";
        for (int L = 0; L <= maxLevel; ++L) std::cerr << " L" << L << "=" << counts[L];
        std::cerr << std::endl;
    };
    logHistogram("subdivided");

    int passes = balanceLeaves(leaves, COARSE, maxLevel, cw0, ch0);
    // Balancing can split a cell into children lying over nodata: drop those
    // (same rule as the subdivision: no centre data, no patch).
    leaves.erase(std::remove_if(leaves.begin(), leaves.end(),
                                [&](const QuadCell& c) {
                                    int nd = cellNodataSamples(grid, c);
                                    if (nd == 0) return false;
                                    if (nd == 5) return true;
                                    int cc = std::clamp(static_cast<int>(std::lround(c.col + c.cw * 0.5)), 0, static_cast<int>(w) - 1);
                                    int rr = std::clamp(static_cast<int>(std::lround(c.row + c.ch * 0.5)), 0, static_cast<int>(h) - 1);
                                    return grid.isNodata(cc, rr);
                                }),
                 leaves.end());
    if (cancelFlag && cancelFlag->load(std::memory_order_relaxed)) return false;
    logHistogram(passes > 1 ? "balanced" : "balanced (no change)");

    // -------------------------------------------------------------------
    // Patch buffers: one GL_PATCHES quad per leaf, corners CCW from (col,
    // row): 0=(col,row) 1=(col+cw,row) 2=(col+cw,row+ch) 3=(col,row+ch).
    // -------------------------------------------------------------------
    patchPositions.clear(); patchUVs.clear();
    patchHeightUVs.clear();
    patchEdgeConstraint.clear();
    patchPositions.reserve(leaves.size() * 4 * 3);
    patchUVs.reserve(leaves.size() * 4 * 2);
    patchHeightUVs.reserve(leaves.size() * 4 * 2);
    patchEdgeConstraint.reserve(leaves.size() * 4 * 4);

    auto addCorner = [&](double col, double row) {
        float elev = grid.sample(col, row);
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

    const LeafIndex index(leaves, COARSE, maxLevel);
    for (const QuadCell& c : leaves) {
        float codes[4];
        edgeCodes(c, index, maxLevel, codes);
        addCorner(c.col,          c.row);
        addCorner(c.col + c.cw,   c.row);
        addCorner(c.col + c.cw,   c.row + c.ch);
        addCorner(c.col,          c.row + c.ch);
        for (int k = 0; k < 4; ++k)
            patchEdgeConstraint.insert(patchEdgeConstraint.end(), codes, codes + 4);
    }
    patchCount = static_cast<int>(leaves.size());

    // Keep the full-res heightmap for uploadGPU(): GL-space heights (nodata
    // filled, see above) and a validity channel (1 = data, 0 = nodata).
    heightmapSrcW = static_cast<int>(w);
    heightmapSrcH = static_cast<int>(h);
    demPixelW = std::abs(geo.A);
    demPixelH = std::abs(geo.E);
    heightmapGLSpace.resize(elevs.size());
    heightmapValid.resize(elevs.size());
    for (size_t i = 0; i < elevs.size(); ++i) {
        heightmapGLSpace[i] = static_cast<float>((filled[i] - worldCenter.z) * invScale);
        heightmapValid[i] = nodataMask[i] ? 0.0f : 1.0f;
    }
    auxData = std::move(data.aux);

    // GL-space bbox from patch corners (displacement can locally exceed it;
    // good enough for near/far and framing).
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

// Box filter (fractional mapping — NOT naive integer stride, see specs.md
// §6.3 for why that matters) of a w x h float grid down to at most
// capTexels. Returns an empty vector when no downsampling is needed;
// otherwise updates w and h to the new size.
static std::vector<float> boxDownsample(const std::vector<float>& src, int& w, int& h,
                                        int capTexels) {
    const int srcW = w, srcH = h;
    if (static_cast<int64_t>(srcW) * srcH <= capTexels) return {};
    float scale = std::sqrt(static_cast<float>(capTexels) /
                            (static_cast<float>(srcW) * srcH));
    int dstW = std::max(1, static_cast<int>(srcW * scale));
    int dstH = std::max(1, static_cast<int>(srcH * scale));
    std::vector<float> out(static_cast<size_t>(dstW) * dstH);
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
                    sum += src[static_cast<size_t>(sy) * srcW + sx];
                    ++count;
                }
            }
            out[static_cast<size_t>(dy) * dstW + dx] =
                static_cast<float>(count > 0 ? sum / count : 0.0);
        }
    }
    w = dstW;
    h = dstH;
    return out;
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
    if (!uploadHeightmapTexture(heightmapGLSpace, heightmapValid, heightmapSrcW, heightmapSrcH,
                                MAX_HEIGHTMAP_TEXELS)) {
        return false;
    }

    if (auxTex) { glDeleteTextures(1, &auxTex); auxTex = 0; }
    if (!auxData.empty()) {
        int aw = heightmapSrcW, ah = heightmapSrcH;
        std::vector<float> small = boxDownsample(auxData, aw, ah, MAX_HEIGHTMAP_TEXELS);
        glGenTextures(1, &auxTex);
        glBindTexture(GL_TEXTURE_2D, auxTex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, aw, ah, 0, GL_RED, GL_FLOAT,
                     small.empty() ? auxData.data() : small.data());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }

    // Not needed on the CPU after upload; rebuilds re-read the file.
    heightmapGLSpace.clear();
    heightmapGLSpace.shrink_to_fit();
    heightmapValid.clear();
    heightmapValid.shrink_to_fit();
    auxData.clear();
    auxData.shrink_to_fit();

    // --- Color texture (orthophoto), uploaded once and kept across
    // rebuilds. Skipped when the ortho's georeferencing says it doesn't
    // overlap the DEM (orthoUsable, see dem_tess_mesh.h). ---
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
// DEMTessMesh::uploadHeightmapTexture — box-filter downsample (fractional
// mapping, see specs.md §6.3) to capTexels if needed, then upload as RG32F:
// R = GL-space height, G = validity (1 data, 0 nodata; fractional where a
// downsampled or interpolated texel straddles the data's edge). Mipmapped,
// for the fragment shader's hill-shading far away. Replaces any previous
// heightmapTex.
// ---------------------------------------------------------------------------
bool DEMTessMesh::uploadHeightmapTexture(const std::vector<float>& glSpaceData,
                                         const std::vector<float>& validData,
                                         int srcW, int srcH, int capTexels) {
    int dW = srcW, dH = srcH;
    std::vector<float> heights = boxDownsample(glSpaceData, dW, dH, capTexels);
    int vW = srcW, vH = srcH;
    std::vector<float> valid = boxDownsample(validData, vW, vH, capTexels);
    const std::vector<float>& h = heights.empty() ? glSpaceData : heights;
    const std::vector<float>& v = valid.empty() ? validData : valid;
    if (!heights.empty()) {
        std::cerr << "[dem-tess] heightmap downsampled: " << srcW << "x" << srcH
                  << " -> " << dW << "x" << dH << std::endl;
    }
    std::vector<float> rg(static_cast<size_t>(dW) * dH * 2);
    for (size_t i = 0; i < static_cast<size_t>(dW) * dH; ++i) {
        rg[2 * i] = h[i];
        rg[2 * i + 1] = v[i];
    }

    heightmapTexW = dW;
    heightmapTexH = dH;
    if (heightmapTex) { glDeleteTextures(1, &heightmapTex); heightmapTex = 0; }
    glGenTextures(1, &heightmapTex);
    glBindTexture(GL_TEXTURE_2D, heightmapTex);
    // 32-bit floats: heights are GL-space values near 0 with centimetre
    // detail; half floats would quantize them.
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RG32F, dW, dH, 0, GL_RG, GL_FLOAT, rg.data());
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
                         bool useDisplacement, bool showMasterEdges,
                         const DemStyle& style) const {
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

    // targetPixelsPerSegment drives the free (same-level edge) formula. The
    // constrained (LOD-transition) level scales with it from the design
    // defaults (8 px ↔ 4.0) so both stay consistent; both sides of such an
    // edge read the same uniform, so it stays crack-free at any setting.
    float clampedTargetPx = glm::clamp(targetPixelsPerSegment, 0.01f, 64.0f);
    glUniform1f(glGetUniformLocation(tessProgram, "uTargetPixelsPerSegment"), clampedTargetPx);
    // Segments K of each fine half edge at a level transition (the coarse
    // side uses 2K, at most 64): an integer, so both sides split exactly.
    float transition = std::round(glm::clamp(4.0f * (8.0f / clampedTargetPx), 1.0f, 32.0f));
    glUniform1f(glGetUniformLocation(tessProgram, "uTransitionSegments"), transition);
    glUniform2f(glGetUniformLocation(tessProgram, "uHeightmapTexels"),
                static_cast<float>(heightmapTexW), static_cast<float>(heightmapTexH));
    glUniform2f(glGetUniformLocation(tessProgram, "uTexelGL"),
                static_cast<float>(demPixelW * heightmapSrcW / std::max(heightmapTexW, 1) / frame.scale),
                static_cast<float>(demPixelH * heightmapSrcH / std::max(heightmapTexH, 1) / frame.scale));
    glUniform1i(glGetUniformLocation(tessProgram, "uShade"), style.shade ? 1 : 0);

    glUniform1i(glGetUniformLocation(tessProgram, "uDisplacementEnabled"), useDisplacement ? 1 : 0);

    glUniform1i(glGetUniformLocation(tessProgram, "uHeightmap"), 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, heightmapTex);

    glUniform1f(glGetUniformLocation(tessProgram, "uMinElev"), glBBoxMinY);
    glUniform1f(glGetUniformLocation(tessProgram, "uMaxElev"), glBBoxMaxY);

    if (colorTex && showTexture) {
        glUniform1i(glGetUniformLocation(tessProgram, "uHasTexture"), 1);
        glUniform1i(glGetUniformLocation(tessProgram, "uTexture"), 1);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, colorTex);
    } else {
        glUniform1i(glGetUniformLocation(tessProgram, "uHasTexture"), 0);
    }

    glUniform1f(glGetUniformLocation(tessProgram, "uOpacity"), style.opacity);
    int auxMode = auxTex ? static_cast<int>(style.auxMode) : 0;
    glUniform1i(glGetUniformLocation(tessProgram, "uAuxMode"), auxMode);
    if (auxMode != 0) {
        glUniform1i(glGetUniformLocation(tessProgram, "uAux"), 2);
        glActiveTexture(GL_TEXTURE2);
        glBindTexture(GL_TEXTURE_2D, auxTex);
        glUniform1f(glGetUniformLocation(tessProgram, "uThreshold"), style.threshold);
        glUniform1f(glGetUniformLocation(tessProgram, "uFrameScale"),
                    static_cast<float>(frame.scale));
        glUniform1f(glGetUniformLocation(tessProgram, "uFrameCenterZ"),
                    static_cast<float>(frame.center.z));
    }
    glActiveTexture(GL_TEXTURE0);

    glPatchParameteri(GL_PATCH_VERTICES, 4);
    glBindVertexArray(vao);
    glDrawArrays(GL_PATCHES, 0, patchCount * 4);
    glBindVertexArray(0);
}

void DEMTessMesh::releaseGeometryGL() {
    if (heightmapTex) glDeleteTextures(1, &heightmapTex);
    if (auxTex) glDeleteTextures(1, &auxTex);
    auxTex = 0;
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
    // The build thread captures `this`: wait for it. GL resources are freed
    // by destroy() (the owner calls it with a current context).
    if (bgThread.joinable()) bgThread.join();
}

void DEMTessMesh::requestBackgroundBuild(const DemSource& source, const Orthophoto* ortho,
                                         const SceneFrame& frame_, double angleThresholdDeg,
                                         int maxLevelParam) {
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
        bgPendingSource = source;
        bgPendingOrtho = ortho;
        bgPendingFrame = frame_;
        bgPendingAngle = angleThresholdDeg;
        bgPendingMaxLevel = maxLevelParam;
        return;
    }
    startBackgroundBuildNow(source, ortho, frame_, angleThresholdDeg, maxLevelParam);
}

void DEMTessMesh::startBackgroundBuildNow(const DemSource& source, const Orthophoto* ortho,
                                          const SceneFrame& frame_, double angleThresholdDeg,
                                          int maxLevelParam) {
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
    bgThread = std::thread([this, source, ortho, frame_, angleThresholdDeg, maxLevelParam, cancelFlag]() {
        // Builds an entirely separate, temporary instance — reuses
        // loadFromDEM() completely unchanged. This touches no GL state and
        // no member of `this`, so it's safe to run concurrently with the
        // main thread rendering `this`'s CURRENT (old) data.
        auto tmp = std::make_unique<DEMTessMesh>();
        bool ok = tmp->loadFromDEM(source, ortho, frame_, angleThresholdDeg, maxLevelParam,
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
            heightmapValid = std::move(pending->heightmapValid);
            auxData = std::move(pending->auxData);
            heightmapSrcW = pending->heightmapSrcW;
            heightmapSrcH = pending->heightmapSrcH;
            demPixelW = pending->demPixelW;
            demPixelH = pending->demPixelH;
            frame = pending->frame;
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
        startBackgroundBuildNow(bgPendingSource, bgPendingOrtho, bgPendingFrame, bgPendingAngle,
                                bgPendingMaxLevel);
    }
    return swapped;
}
