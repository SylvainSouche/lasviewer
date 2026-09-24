// dem_tess_mesh.h — GPU hardware-tessellated DEM/DSM mesh with
// normal-directed displacement mapping.
//
// Isolated, OPTIONAL alternative to DEMMesh (dem_mesh.h). Requires an
// OpenGL 4.0+ context (tessellation control/evaluation shaders — core
// since GL4.0). macOS supports GL 4.1 core on both Intel and Apple Silicon,
// so this is usable on the primary target without a Vulkan/Metal backend.
// See docs/design-tessellation-displacement.md for the full design writeup
// and known open risks (this has NOT been tested on real GPU hardware).
//
// Design summary (SECOND REVISION — see design doc §4 for why the first
// revision's uniform patch grid was wrong: it discarded exactly the
// adaptivity that concentrates small patches around sharp/discontinuous
// features like buildings, causing visible over-smoothing there):
//   - CPU builds patches from the SAME adaptive quadtree DEMMesh uses
//     (subdivide by geometric error + balance pass, ≤1 level difference
//     between any two adjacent leaves) — one GL_PATCHES quad per leaf.
//   - Crack avoidance is split by tessellation-level type, since only
//     OUTER (edge) levels can ever cause a crack — INNER (interior)
//     tessellation is entirely private to a patch and needs no
//     cross-patch agreement at all, so it stays fully free/view-dependent:
//       * Edges bordering a SAME-level neighbor (or no neighbor — a
//         raster-domain boundary): free continuous screen-space formula.
//         Both sides independently compute identical corner endpoints, so
//         they independently arrive at identical values (see
//         kMeshTessControl in shaders.cpp for the determinism argument).
//       * Edges bordering a DIFFERENT-level neighbor (LOD transitions,
//         capped at exactly ±1 level by the balance pass): a FIXED,
//         level-derived value instead — two independently-computed
//         continuous estimates on differently-sized patches cannot be
//         trusted to land on matching segment counts.
//   - Each patch corner gets a real per-vertex normal (finite difference
//     on the full-resolution heightmap) for displacement direction.
//   - Geometry is culled to real DEM coverage: cells that are entirely
//     nodata are dropped (no patch built there at all — an orthophoto
//     covering that area does NOT cause geometry to be fabricated), and
//     cells straddling a nodata/valid boundary are subdivided further (up
//     to MAX_LEVEL) to localize that edge rather than rendering a coarse,
//     blocky boundary. See isNodataAt() in loadFromDEM().
//   - The full-resolution heightmap is uploaded as a single-channel float
//     texture (downsampled first if oversized — see uploadGPU()), storing
//     the SAME GL-space Y value a vertex would get:
//     (elev - worldCenter.z) / worldScale. This means the tessellation
//     evaluation shader can compute the vertical gap between the fine and
//     coarse GL-space Y values with no separate unit-conversion uniform
//     needed — see the next point for what's actually done with that gap
//     (NOT applied to displacement directly — that was a real, corrected
//     bug, see shaders.cpp's TES history / design doc §6f-§6h).
//   - Tessellation Evaluation Shader: bilinear-interpolates each generated
//     point's coarse position/normal/UV from the patch's 4 corners, samples
//     the heightmap at a SEPARATE DEM-raster-relative UV (see patchUVs vs.
//     patchHeightUVs below — conflating these was a real bug found on
//     first hardware test: the orthophoto and DEM generally cover
//     different extents, so sampling the heightmap with the orthophoto-
//     relative UV samples essentially the wrong location, producing large
//     wrong displacement and visibly broken geometry). The resulting
//     vertical gap (fine - coarse) is NOT applied as displacement
//     directly — an earlier version did exactly that, and separately an
//     even earlier "fix" divided by normal.y, both wrong for different
//     reasons (design doc §6f-§6h). The correct displacement distance is
//     a ray/tangent-plane intersection: verticalGap * normal.y — degenerates
//     to the vertical gap exactly when the normal is vertical, and
//     SHRINKS (not amplifies) on steep terrain. Patch CORNERS always get
//     this displacement == 0 by construction. KNOWN RESIDUAL (documented,
//     not fully solved): GPU-generated boundary vertices that are NOT
//     corners get displaced along a tilted (non-vertical) normal, which
//     nudges them slightly in X/Z as well as Y — so they won't land
//     bit-exactly on a finer neighbor's true corner position at steep LOD
//     transitions. Expected to be sub-pixel in practice (proportional to
//     the same geometric-error term the adaptive threshold already keeps
//     small there, and further shrunk by §6q's density scaling — smaller
//     triangles, smaller residual), but is a real, understood residual —
//     see design doc §9 for why this wasn't also "fixed," and check it
//     specifically at steep transitions on hardware.
#pragma once
#include "gl_platform.h"
#include <glm/glm.hpp>
#include <string>
#include <vector>
#include <cstdint>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>

struct Orthophoto; // forward decl (geotiff.h)

// True if the current GL context supports tessellation shaders (GL >= 4.0
// core). Must be called AFTER glfwMakeContextCurrent(). Callers should fall
// back to DEMMesh (dem_mesh.h) if this returns false.
bool demTessSupported();

struct DEMTessMesh {
    // --- CPU-side patch data (populated by loadFromDEM, GL-context-free
    //     so this can be called before a window/context exists, matching
    //     DEMMesh's calling convention in main.cpp).
    //
    //     REVISED from the first implementation: patches now come from the
    //     SAME adaptive quadtree DEMMesh uses (subdivide + balance pass),
    //     not a uniform grid — a uniform grid discarded exactly the
    //     adaptivity that concentrates small patches around sharp features
    //     (buildings, cliffs), which caused visible over-smoothing there.
    //     See docs/design-tessellation-displacement.md §4 ("Second
    //     revision") for the full writeup.
    //
    //     Patches are a FLAT, non-indexed array (4 corner vertices per
    //     patch, no cross-patch vertex sharing) rather than an indexed
    //     grid — variable patch sizes make shared-vertex indexing
    //     significantly more bookkeeping for a benefit (some VRAM) that
    //     doesn't matter at typical patch counts. Two same-level patches
    //     computing the "same" shared corner from identical (col,row)
    //     inputs still get bit-identical floats independently, so this
    //     doesn't reintroduce any crack risk. ---
    std::vector<float> patchPositions;      // 3 floats/vertex, 4 verts/patch
    std::vector<float> patchUVs;            // 2 floats/vertex — ORTHOPHOTO-relative
                                             // (color sampling only — see demUVFor()
                                             // in dem_tess_mesh.cpp for why this must
                                             // NOT be reused for heightmap sampling)
    std::vector<float> patchHeightUVs;      // 2 floats/vertex — DEM-raster-relative
                                             // (heightmap sampling only, always
                                             // col/(w-1),row/(h-1) regardless of any
                                             // orthophoto's own geo extent)
    // Per-patch edge constraint, replicated identically to all 4 corner
    // vertices of a patch (GLSL only has per-vertex attributes; the TCS
    // only reads index 0, since all 4 corners carry the same 4 values).
    // Order: x=bottom, y=right, z=top, w=left (matches corner winding
    // 0=BL,1=BR,2=TR,3=TL). Value per edge: 0 = unconstrained (neighbor is
    // same level, or this is a raster-domain boundary with no neighbor —
    // free continuous view-dependent tessellation is crack-safe here,
    // since both sides of a same-level shared edge compute identical
    // corner positions independently); 1 = constrained (neighbor is one
    // level coarser/finer, per the balance pass's guaranteed ≤1 level
    // difference — use a fixed, level-derived value instead of the
    // continuous formula, since two DIFFERENTLY-sized independent patches
    // cannot be trusted to agree on a continuous estimate).
    std::vector<float> patchEdgeConstraint; // 4 floats/vertex
    int patchCount = 0;

    // Full-resolution elevation raster, kept (unlike DEMMesh, which frees
    // it after CPU mesh build) because uploadGPU() needs it for the
    // heightmap texture. Stored pre-converted to GL-space Y so uploadGPU()
    // doesn't need worldCenter/worldScale again.
    std::vector<float> heightmapGLSpace;
    int heightmapSrcW = 0, heightmapSrcH = 0;

    glm::dvec3 worldCenter{0.0};
    double worldScale = 1.0;
    glm::dvec3 bboxMin{0.0}, bboxMax{0.0};
    glm::vec2 glBBoxMin{0.0f}, glBBoxMax{0.0f};
    float glBBoxMinY = 0.0f, glBBoxMaxY = 0.0f;
    bool loaded = false;   // CPU data ready
    bool valid = false;    // GPU resources ready (after uploadGPU)
    // False specifically when the orthophoto HAS geo tags but genuinely
    // does not geographically overlap the DEM — as opposed to the
    // orthophoto having no geo tags at all, a different, deliberately-
    // still-supported case (a convenience for manually pairing an
    // untagged image with a DEM, stretched to fit). When there IS geo
    // metadata and it says the two don't correspond, stretching the
    // orthophoto onto the DEM anyway is actively misleading — it would
    // show imagery from a completely unrelated location — reported
    // directly as "the full orthophoto is displayed" when it shouldn't
    // be. uploadGPU() checks this and skips texturing entirely (falls
    // back to the elevation color ramp) rather than force a fit that
    // isn't real.
    bool orthoUsable = true;

    // For increaseHeightmapResolution()'s reload-on-demand (see there):
    // the DEM file path (re-read from disk on reload rather than keeping
    // a second full-resolution CPU copy resident for the whole session),
    // the texel cap actually used for the currently-uploaded heightmap
    // texture, and whether that upload already reached the DEM's true
    // native resolution (once true, further density increases correctly
    // become a pure GPU-tessellation question — there's no more real
    // detail on disk left to fetch).
    std::string sourcePath;
    int heightmapCapTexels = 0;
    bool heightmapIsNative = false;
    bool heightmapAtSafetyCeiling = false; // see increaseHeightmapResolution()
    // Point-collapsing angle (I/O keys, main.cpp) — the ANGLE_THRESH_DEG
    // used by the last loadFromDEM()/reload() call. Since this drives a
    // LOAD-TIME quadtree subdivision decision (§4b of the design doc),
    // changing it requires a full reload() — see there — not just a
    // uniform update like the render-time density (S/F) controls.
    double collapseAngleDeg = 1.0;
    // Quadtree depth ceiling (S/F keys, main.cpp) — the MAX_LEVEL used by
    // the last loadFromDEM()/reload() call. Directly controls the
    // "how many coarse patches can this DEM ever have" number
    // (COARSE * 2^maxLevel per axis) — like collapseAngleDeg, this is a
    // LOAD-TIME structural parameter, not a render-time one, so changing
    // it requires a full reload(). S/F previously drove GPU tessellation
    // density (uTargetPixelsPerSegment) for this path instead; that
    // render-time control was replaced by this load-time one on direct
    // request ("that is the number I want to change with S and F") —
    // main.cpp no longer derives a tessellation-density target from
    // input.density for the DEM-tessellation path, only for point clouds.
    int maxLevel = 5;

    // --- GPU resources (owned by this instance — NOT file-static, unlike
    //     DEMMesh; this is a new module with no compatibility constraint to
    //     match, and instance ownership avoids the "only one instance ever
    //     safe" landmine file-statics create). ---
    GLuint vao = 0, posVBO = 0, uvVBO = 0, heightUVVBO = 0, edgeConstraintVBO = 0;
    GLuint heightmapTex = 0;
    GLuint colorTex = 0;   // orthophoto; 0 if none (elevation-ramp fallback)

    // Step 1 (no GL context required): read the DEM + compute the coarse
    // patch grid, normals, and heightmap. ortho may be null.
    // angleThresholdDeg is the point-collapsing angle (I/O keys, main.cpp)
    // — see collapseAngleDeg below and reload(). maxLevelParam is the
    // quadtree depth ceiling (S/F keys) — see maxLevel below. cancelFlag,
    // if non-null, is checked periodically during the (potentially slow,
    // §6k of the design doc) bottom-up collapse — if set, the build stops
    // early and returns false, letting a background build that's been
    // superseded (requestBackgroundBuild(), below) abort promptly instead
    // of running to completion just to be discarded.
    bool loadFromDEM(const std::string& path, const Orthophoto* ortho,
                     double angleThresholdDeg = 1.0, int maxLevelParam = 5,
                     const std::atomic<bool>* cancelFlag = nullptr);

    // Step 2 (GL context MUST be current): upload patch VBOs + heightmap
    // texture + color texture. Call once, after loadFromDEM() succeeds and
    // after confirming demTessSupported(). ortho must be the same pointer
    // passed to loadFromDEM (re-passed here rather than cached, since
    // Orthophoto pixel data may be large and the caller already owns it).
    bool uploadGPU(const Orthophoto* ortho);

    // Shared by uploadGPU() and increaseHeightmapResolution() — downsamples
    // to capTexels if needed and (re-)uploads the heightmap texture,
    // replacing any previous one. Not usually called directly.
    bool uploadHeightmapTexture(const std::vector<float>& glSpaceData,
                                int srcW, int srcH, int capTexels);

    // Reload-on-demand: re-reads the DEM from disk and doubles the
    // heightmap texture's effective resolution cap (clamped to the DEM's
    // true native resolution), replacing the texture in place. Call this
    // when the user asks for finer density (F key) — see the function's
    // own comment in dem_tess_mesh.cpp for why just increasing GPU
    // tessellation density on an unchanged, already-downsampled texture
    // doesn't add real detail. Geometry is untouched. Returns false (no-
    // op) once heightmapIsNative is already true, or on I/O failure.
    bool increaseHeightmapResolution();

    // Full rebuild with a new point-collapsing angle (I/O keys) and/or a
    // new quadtree depth ceiling (S/F keys) — unlike
    // increaseHeightmapResolution(), either of these changes the CPU-side
    // quadtree subdivision itself (§4b of the design doc), so the entire
    // patch set, not just the heightmap texture, must be regenerated and
    // re-uploaded. Re-reads the DEM from disk (same reasoning as
    // increaseHeightmapResolution() for not keeping a second full-
    // resolution CPU copy resident). Synchronous — will hitch the frame,
    // same tradeoff as the rest of this module's loading. Returns false
    // on I/O failure (logged); geometry is left in its previous state.
    bool reload(const Orthophoto* ortho, double newAngleThresholdDeg, int newMaxLevel);

    // Draw using a tessellation program (link via
    // shaders::linkTessProgram(kMeshTessVert, kMeshTessControl,
    // kMeshTessEval, kMeshFrag) — kMeshFrag is reused unmodified from the
    // existing DEMMesh path). fov is in degrees, matching Camera::fov and
    // TileGrid::render()'s convention. targetPixelsPerSegment controls
    // triangle density (S/F keys in main.cpp — smaller = denser, same
    // inverse relationship point-cloud density already uses); the
    // constrained-edge (LOD-transition) tessellation level is derived from
    // it automatically so the two stay in lockstep (see shaders.cpp,
    // kMeshTessControl). useDisplacement toggles the A key's plain-
    // tessellated-surface debug view.
    void render(GLuint tessProgram, const glm::mat4& V, const glm::mat4& P,
                const glm::vec3& camPosGL, float fov, float viewportH,
                float zScale, float targetPixelsPerSegment,
                bool useDisplacement, bool showMasterEdges) const;

    void destroy();

    // ---------------------------------------------------------------------
    // Background rebuild — requested directly: "load coarsest, and display
    // it while we compute the full picture. swap representation when
    // ready. same when changes require rebuilding the representation." A
    // follow-up request tightened this further: "when a background task is
    // obsolete it must be stopped before starting the new one that
    // rendered it obsolete" — see requestBackgroundBuild()'s comment below
    // for the resulting cancellation mechanism (this replaced an earlier
    // "let the superseded build run to completion, just discard the
    // result" design).
    //
    // Mechanism: the background thread builds an entirely SEPARATE,
    // temporary DEMTessMesh instance via its own loadFromDEM() call
    // (unchanged — reused exactly as-is, not duplicated) — it never
    // touches THIS instance's members, so there is no data race with the
    // main thread, which keeps rendering the current (old) state
    // uninterrupted until a finished background build is explicitly
    // consumed by pollBackgroundBuild(). The heavy CPU work (bottom-up
    // quadtree collapse, §6k of the design doc — can be genuinely slow at
    // high maxLevel) happens entirely off the main thread; only the final
    // GPU upload (uploadGPU(), fast by comparison) happens on it, inside
    // pollBackgroundBuild().
    //
    // At most one background thread is ever alive at a time. If a new
    // request arrives while one is already running (e.g. rapid I/O/S/F
    // key-repeat), the running build is signaled to abort (cooperative —
    // loadFromDEM()'s bottom-up collapse checks the flag periodically and
    // stops early, see dem_tess_mesh.cpp) rather than left to finish and
    // be discarded. The new request does NOT start immediately or block
    // the caller either way — the latest requested parameters are
    // remembered and automatically started by the next
    // pollBackgroundBuild() call once the (now aborting) build has
    // actually exited, coalescing any intermediate requests into just the
    // latest one.
    // ---------------------------------------------------------------------

    // Non-blocking. Kicks off (or queues, per the coalescing rule above) a
    // background CPU build at the given parameters. If a build is already
    // running, it is signaled to abort first.
    void requestBackgroundBuild(const std::string& path, const Orthophoto* ortho,
                                double angleThresholdDeg, int maxLevelParam);

    // Call once per frame regardless of whether a rebuild was explicitly
    // requested this frame — this is also how the initial "coarsest first,
    // then background full-detail" load (main.cpp) picks up its result.
    // If a background build has finished, uploads it to GPU (GL context
    // must be current — main thread only) and swaps it in, returning true.
    // Otherwise a cheap no-op, returning false.
    bool pollBackgroundBuild(const Orthophoto* ortho);

    // For UI feedback (e.g. an on-screen "rebuilding..." indicator).
    bool backgroundBuildInProgress() const { return bgInProgress.load(); }

    ~DEMTessMesh();

private:
    void startBackgroundBuildNow(const std::string& path, const Orthophoto* ortho,
                                 double angleThresholdDeg, int maxLevelParam);

    std::thread bgThread;
    std::mutex bgMutex;
    std::atomic<bool> bgInProgress{false};
    std::atomic<bool> bgHasResult{false};
    std::unique_ptr<DEMTessMesh> bgPending; // protected by bgMutex

    // Owned via shared_ptr (not a plain member) specifically so a NEW
    // build's flag is independent of whatever the PREVIOUS (now-aborting)
    // build's thread still holds a reference to — reassigning
    // `bgCancelFlag` to a fresh flag for the new build doesn't affect the
    // old thread's already-captured copy, which it needs to keep checking
    // (a flag it no longer has any way to reach otherwise) until it
    // actually exits.
    std::shared_ptr<std::atomic<bool>> bgCancelFlag;

    // Coalesced "latest requested params while a build was already
    // running" — see requestBackgroundBuild()'s comment above.
    bool bgHasPendingRequest = false;
    std::string bgPendingPath;
    const Orthophoto* bgPendingOrtho = nullptr;
    double bgPendingAngle = 1.0;
    int bgPendingMaxLevel = 5;
};
