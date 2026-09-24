// shaders.cpp — GLSL shader sources and GL compile/link helpers.
//
// All shader sources are kept as raw string literals in this TU. The
// compile/link helpers wrap the standard GL boilerplate and print the
// info-log on failure.
#include "shaders.h"

#include <iostream>

namespace shaders {

// ---------------------------------------------------------------------------
// Embedded GLSL shader sources
// ---------------------------------------------------------------------------

const char* kPointCloudVert = R"GLSL(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

uniform mat4 uView;
uniform mat4 uProj;
uniform float uPointSize;
uniform float uViewportH;
uniform float uZScale;       // vertical exaggeration factor (1.0 = true scale)
uniform float uTargetPixelSpacing;  // desired on-screen spacing in pixels
uniform float uTanHalfFov;   // tan(fov/2), precomputed on CPU
uniform float uDensity;      // point density in GL-space (pts per GL-unit²)
uniform float uDensityMul;   // user density multiplier (1.0 = default)
uniform float uDisableSubsampling;  // 1.0 = draw all points, 0.0 = normal
uniform float uOrtho;         // 1.0 = orthographic, 0.0 = perspective
uniform float uOrthoHeight;   // ortho view-box half-height in GL units (0 in persp)

out vec3 vColor;

// 3D hash function — returns pseudo-random value in [0,1).
// Stable per input position, varies smoothly in space.
float hash13(vec3 p) {
    p = fract(p * 0.1031);
    p += dot(p, p.zyx + 31.32);
    return fract((p.x + p.y) * p.z);
}

void main() {
    // Apply Z scale in GL space (Y axis after the Z-up -> Y-up swap).
    vec3 p = aPos;
    p.y *= uZScale;
    vec4 viewPos = uView * vec4(p, 1.0);
    float depth = max(-viewPos.z, 0.001);

    // --- Depth-based stochastic subsampling ---
    // Compute the desired world-space point spacing at this depth.
    //   pixelsPerWorldUnit = viewportH / (2 * depth * tan(fov/2))
    //   worldSpacing = targetPixelSpacing / pixelsPerWorldUnit
    //                = targetPixelSpacing * 2 * depth * tan(fov/2) / viewportH
    float worldSpacing = uTargetPixelSpacing * 2.0 * depth * uTanHalfFov
                         / (uViewportH * uDensityMul);

    // Each voxel of size worldSpacing contains ~N points:
    //   N = density * worldSpacing²
    // Keep each point with probability 1/N → statistically 1 point per voxel
    // → uniform screen-space density regardless of depth.
    float N = uDensity * worldSpacing * worldSpacing;
    float keepThreshold = clamp(1.0 / max(N, 1.0), 0.0, 1.0);

    // Per-point hash (stable per point position, varies spatially).
    // Using the original (unscaled) position so the hash doesn't change
    // when zScale changes — prevents popping during Z-scale adjustment.
    float pointHash = hash13(aPos * 1000.0);

    // Skip subsampling entirely when uDisableSubsampling is set (for labels).
    if (uDisableSubsampling < 0.5 && pointHash > keepThreshold) {
        // Discard this point — push it outside clip space.
        gl_Position = vec4(0.0, 0.0, 0.0, -1.0);
        gl_PointSize = 0.0;
        return;
    }

    gl_Position = uProj * viewPos;
    // Point size: perspective-scaled in perspective mode, fixed in ortho mode.
    if (uOrtho > 0.5) {
        gl_PointSize = uPointSize * uViewportH / (2.0 * max(uOrthoHeight, 0.001));
    } else {
        gl_PointSize = uPointSize * (uViewportH / depth);
    }
    vColor = aColor;
}
)GLSL";

const char* kPointCloudFrag = R"GLSL(
#version 330 core
in vec3 vColor;
out vec4 FragColor;

void main() {
    // Round the point sprite so it looks like a small disc, not a square.
    vec2 c = gl_PointCoord - vec2(0.5);
    float r2 = dot(c, c);
    if (r2 > 0.25) discard;
    FragColor = vec4(vColor, 1.0);
}
)GLSL";

// Simple line shader for wireframe bbox rendering.
const char* kLineVert = R"GLSL(
#version 330 core
layout (location = 0) in vec3 aPos;
uniform mat4 uView;
uniform mat4 uProj;
uniform float uZScale;
void main() {
    vec3 p = aPos;
    p.y *= uZScale;
    gl_Position = uProj * uView * vec4(p, 1.0);
}
)GLSL";

const char* kLineFrag = R"GLSL(
#version 330 core
out vec4 FragColor;
uniform vec3 uColor;
void main() {
    FragColor = vec4(uColor, 1.0);
}
)GLSL";

// Mesh shader: textured triangles for DEM surface.
// Supports 3 color modes:
//   uHasTexture == 1 : sample orthophoto texture
//   uHasTexture == 0 : elevation-based color ramp (dark blue-violet -> teal -> yellow)
const char* kMeshVert = R"GLSL(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aUV;
uniform mat4 uView;
uniform mat4 uProj;
uniform float uZScale;
out vec2 vUV;
out float vElev;
out float vEdgeDist;
void main() {
    vec3 p = aPos;
    p.y *= uZScale;
    gl_Position = uProj * uView * vec4(p, 1.0);
    vUV = aUV;
    vElev = aPos.y;  // GL Y = elevation (before zScale)
    // DEMMesh (this path) has no per-patch parametric (u,v) coordinate at
    // the vertex-shader stage — it's a single, already-fully-triangulated
    // static mesh, not built from GPU-tessellated patches. 1.0 ("far from
    // any edge") means the master-edge red highlight in kMeshFrag never
    // triggers here; that visualization is specific to the GPU-tessellated
    // DEMTessMesh path (kMeshTessEval), where "coarse patch edge" is a
    // meaningful, distinct concept from the rendered triangle edges.
    vEdgeDist = 1.0;
}
)GLSL";

const char* kMeshFrag = R"GLSL(
#version 330 core
in vec2 vUV;
in float vElev;
in float vEdgeDist;
out vec4 FragColor;
uniform sampler2D uTexture;
uniform int uHasTexture;
uniform float uMinElev;
uniform float uMaxElev;
uniform int uShowMasterEdges; // G key (main.cpp) — default OFF; this is a
                              // diagnostic overlay, not a rendering feature,
                              // so it shouldn't be in the way unless asked for

void main() {
    // Master (coarse patch) edges — see vEdgeDist's producer in
    // kMeshTessEval for the GPU-tessellated DEM path, and kMeshVert above
    // for why this never triggers on the CPU-only DEMMesh path. Drawn in
    // place of the orthophoto/elevation-ramp color, not blended over it,
    // so the boundary is unambiguous. MASTER_EDGE_THRESHOLD is in the
    // same parametric [0, 0.5] units as vEdgeDist (0 = exactly on an
    // edge, 0.5 = patch center) — a starting point, not tuned; the actual
    // on-screen line thickness scales with each patch's own size, since
    // this threshold is patch-relative, not a fixed world-space or
    // screen-space width.
    const float MASTER_EDGE_THRESHOLD = 0.01;
    if (uShowMasterEdges == 1 && vEdgeDist < MASTER_EDGE_THRESHOLD) {
        FragColor = vec4(1.0, 0.0, 0.0, 1.0);
        return;
    }

    if (uHasTexture == 1) {
        FragColor = texture(uTexture, vUV);
    } else {
        // Elevation-based color ramp: dark blue-violet (low) -> teal (mid)
        // -> yellow (high) — a viridis-inspired gradient. Replaced the
        // original green -> brown -> white (hypsometric-tinting) ramp on
        // direct request ("i never want that one") — same 2-segment
        // mix() structure, just different anchor colors, so this is a
        // drop-in palette swap, not a new blending mechanism.
        float range = max(uMaxElev - uMinElev, 0.0001);
        float t = clamp((vElev - uMinElev) / range, 0.0, 1.0);
        vec3 low  = vec3(0.267, 0.005, 0.329);  // dark blue-violet
        vec3 mid  = vec3(0.128, 0.567, 0.551);  // teal
        vec3 high = vec3(0.993, 0.906, 0.144);  // yellow
        vec3 c = (t < 0.5)
            ? mix(low,  mid,  t * 2.0)
            : mix(mid,  high, (t - 0.5) * 2.0);
        FragColor = vec4(c, 1.0);
    }
}
)GLSL";

// ---------------------------------------------------------------------------
// GPU-tessellated DEM mesh shaders (src/dem_tess_mesh.cpp).
//
// Pipeline: kMeshTessVert (passthrough, one invocation per patch corner)
//        -> kMeshTessControl (TCS: per-edge tessellation level)
//        -> kMeshTessEval (TES: bilinear-interpolates position/normal/UV,
//           samples the heightmap, displaces along the interpolated normal)
//        -> kMeshFrag (REUSED UNCHANGED — TES outputs the same vUV/vElev
//           interface kMeshVert does).
//
// SECOND REVISION — crack avoidance split by tessellation-level type, since
// only OUTER (edge) levels can ever cause a crack; INNER (interior) is
// entirely private to a patch and needs no cross-patch agreement, so it
// stays fully free/view-dependent regardless of what the edges do. See
// dem_tess_mesh.h's header banner and docs/design-tessellation-displacement.md
// §4-5 for the full writeup of why the first revision (uniform patch grid)
// was wrong, and why "free formula everywhere" doesn't work across patches
// of different sizes.
//   - Edge borders a SAME-level neighbor, or no neighbor (raster-domain
//     boundary) — aEdgeConstraint component == 0: free continuous
//     screen-space formula. Both patches sharing that edge independently
//     compute identical corner endpoints (built from identical (col,row)
//     inputs on the CPU), so they independently arrive at identical values
//     — no coordination needed, this is the same determinism argument the
//     first revision already had right.
//   - Edge borders a DIFFERENT-level neighbor (LOD transition, capped at
//     exactly ±1 level by DEMTessMesh::loadFromDEM()'s balance pass) —
//     aEdgeConstraint component == 1: use uConstrainedEdgeTessLevel, a
//     fixed constant BOTH the coarse patch and its finer neighbor's two
//     half-edges use identically — two independently-computed CONTINUOUS
//     estimates on differently-sized patches cannot be trusted to land on
//     matching segment counts, but two uses of the SAME fixed constant
//     trivially do.
//
// Requires GL 4.0+ (GL_TESS_CONTROL_SHADER / GL_TESS_EVALUATION_SHADER are
// core since GL4.0). Targets #version 410 core specifically because macOS's
// OpenGL ceiling is 4.1 core — see
// docs/design-tessellation-displacement.md §2.
//
// UNVERIFIED ON REAL GPU HARDWARE — see the banner comment at the top of
// dem_tess_mesh.cpp for known open risks before trusting this in production,
// including a documented residual at different-level transitions on steep
// terrain (tilted-normal displacement of non-corner boundary vertices).
// ---------------------------------------------------------------------------
const char* kMeshTessVert = R"GLSL(
#version 410 core
layout (location = 0) in vec3 aPos;
// location 1 deliberately unused — was aNormal, removed along with the
// along-normal displacement approach it fed. See the TES below for why:
// displacement is now a direct vertical correction against the true
// heightmap value, with no remaining use for a per-vertex normal.
layout (location = 2) in vec2 aUV;
layout (location = 3) in vec2 aHeightUV; // DEM-raster-relative — see
                                         // dem_tess_mesh.h/.cpp demUVFor():
                                         // NOT the same space as aUV, which
                                         // is orthophoto-relative.
layout (location = 4) in vec4 aEdgeConstraint; // x=bottom,y=right,z=top,w=left
out vec3 vPosVC;
out vec2 vUVVC;
out vec2 vHeightUVVC;
out vec4 vEdgeConstraintVC;
void main() {
    vPosVC = aPos;
    vUVVC = aUV;
    vHeightUVVC = aHeightUV;
    vEdgeConstraintVC = aEdgeConstraint;
}
)GLSL";

const char* kMeshTessControl = R"GLSL(
#version 410 core
layout(vertices = 4) out;

in vec3 vPosVC[];
in vec2 vUVVC[];
in vec2 vHeightUVVC[];
in vec4 vEdgeConstraintVC[];
out vec3 vPosTC[];
out vec2 vUVTC[];
out vec2 vHeightUVTC[];

uniform vec3 uCamPos;
uniform float uViewportH;
uniform float uTanHalfFov;
uniform float uZScale;
uniform float uTargetPixelsPerSegment;
uniform float uConstrainedEdgeTessLevel;

// This uniform's value is set from DEMTessMesh::render() to always match
// the free formula's overall density (both scale inversely with the same
// S/F-controlled density factor — see dem_tess_mesh.cpp render()) so a
// coarse patch and its finer neighbor's two half-edges keep agreeing on
// this value regardless of the current density setting; it is NOT a
// compile-time constant precisely because it must move in lockstep with
// uTargetPixelsPerSegment for that agreement to hold. See §5.1 of
// docs/design-tessellation-displacement.md for the full argument for why
// a SHARED value (constant or not) is what makes a constrained edge
// crack-free, regardless of what that shared value currently is.

// Free, continuous, view-dependent formula — ONLY valid on edges where
// both sides are guaranteed to compute the same corner endpoints (same-
// level neighbor, or a raster-domain boundary with no neighbor at all).
// See the file-level comment above for the crack-avoidance argument.
//
// Deliberately driven by screen-space size ALONE, not scaled by any
// CPU-side judgment of how "flat" or well-approximated a patch is (a
// per-patch density-scaling factor was tried and reverted — see design
// doc §6q's follow-up note — because it undermines the entire reason
// this feature exists: GPU tessellation + per-vertex displacement is
// supposed to catch and correct whatever the CPU's own, necessarily
// coarse and finite-resolution sampling (§4b/§6k) might have missed,
// and scaling GENERATED vertex count down based on that SAME coarse
// judgment reintroduces exactly the aliasing risk the bottom-up collapse
// was built to minimize. Every vertex's position should come from the
// DEM — the only point of displacement is to refine that on the fly for
// whatever vertices get generated, so generating fewer of them based on
// a possibly-wrong flatness call works against that, not with it).
float freeEdgeTessLevel(vec3 aGL, vec3 bGL) {
    vec3 a = vec3(aGL.x, aGL.y * uZScale, aGL.z);
    vec3 b = vec3(bGL.x, bGL.y * uZScale, bGL.z);
    vec3 mid = (a + b) * 0.5;
    float dist = max(length(mid - uCamPos), 0.0001);
    float pxPerUnit = uViewportH / (2.0 * dist * uTanHalfFov);
    float edgeLenGL = length(b - a);
    float edgeLenPx = edgeLenGL * pxPerUnit;
    return clamp(edgeLenPx / uTargetPixelsPerSegment, 1.0, 64.0);
}

float outerLevelFor(vec3 aGL, vec3 bGL, float constraint) {
    return (constraint > 0.5) ? uConstrainedEdgeTessLevel : freeEdgeTessLevel(aGL, bGL);
}

void main() {
    vPosTC[gl_InvocationID]      = vPosVC[gl_InvocationID];
    vUVTC[gl_InvocationID]       = vUVVC[gl_InvocationID];
    vHeightUVTC[gl_InvocationID] = vHeightUVVC[gl_InvocationID];

    // Must synchronize before invocation 0 reads all 4 corners' outputs.
    barrier();

    if (gl_InvocationID == 0) {
        // Corner order (CCW): 0=BL, 1=BR, 2=TR, 3=TL — matches
        // DEMTessMesh::loadFromDEM()'s patch corner winding.
        vec4 ec = vEdgeConstraintVC[0]; // x=bottom,y=right,z=top,w=left — same on all 4 corners
        float eBottom = outerLevelFor(vPosTC[0], vPosTC[1], ec.x);
        float eRight  = outerLevelFor(vPosTC[1], vPosTC[2], ec.y);
        float eTop    = outerLevelFor(vPosTC[2], vPosTC[3], ec.z);
        float eLeft   = outerLevelFor(vPosTC[3], vPosTC[0], ec.w);

        // NOTE: GLSL quad-domain OuterLevel[i] <-> physical-edge
        // correspondence (OuterLevel[0]=u=0/left, [1]=v=0/bottom,
        // [2]=u=1/right, [3]=v=1/top) is per the GLSL 4.x spec as currently
        // understood, but has NOT been visually verified on real hardware —
        // see docs/design-tessellation-displacement.md §9. If patches look
        // stretched/rotated wrong on first test, this mapping is the first
        // place to check.
        gl_TessLevelOuter[0] = eLeft;
        gl_TessLevelOuter[1] = eBottom;
        gl_TessLevelOuter[2] = eRight;
        gl_TessLevelOuter[3] = eTop;

        // INNER tessellation is entirely private to this patch — no
        // cross-patch agreement needed, so it stays fully free/continuous
        // regardless of whether any of this patch's edges are constrained.
        // Using the free formula on the FULL diagonal-ish span (via the
        // corner-to-corner distance already computed for outer edges)
        // keeps interior detail responsive even on patches that have a
        // constrained boundary.
        gl_TessLevelInner[0] = max(freeEdgeTessLevel(vPosTC[0], vPosTC[1]),
                                   freeEdgeTessLevel(vPosTC[2], vPosTC[3]));
        gl_TessLevelInner[1] = max(freeEdgeTessLevel(vPosTC[3], vPosTC[0]),
                                   freeEdgeTessLevel(vPosTC[1], vPosTC[2]));
    }
}
)GLSL";

const char* kMeshTessEval = R"GLSL(
#version 410 core
layout(quads, equal_spacing, ccw) in;

in vec3 vPosTC[];
in vec2 vUVTC[];
in vec2 vHeightUVTC[];
out vec2 vUV;
out float vElev;
out float vEdgeDist; // parametric distance to nearest coarse-patch edge —
                     // see kMeshFrag for the red master-edge visualization
                     // this feeds. 0 exactly on an edge/corner, 0.5 at the
                     // patch center.

uniform mat4 uView;
uniform mat4 uProj;
uniform float uZScale;
uniform sampler2D uHeightmap;
uniform int uDisplacementEnabled; // A key: 0 = show the plain tessellated
                                  // coarse surface (no correction applied) —
                                  // useful to visually isolate whether a
                                  // seam comes from tessellation-count
                                  // mismatch alone vs. from displacement,
                                  // see docs/design-tessellation-displacement.md §9

vec3 bilerp3(vec3 a, vec3 b, vec3 c, vec3 d, float u, float v) {
    // a=BL(0), b=BR(1), c=TR(2), d=TL(3)
    vec3 bottom = mix(a, b, u);
    vec3 top    = mix(d, c, u);
    return mix(bottom, top, v);
}
vec2 bilerp2(vec2 a, vec2 b, vec2 c, vec2 d, float u, float v) {
    vec2 bottom = mix(a, b, u);
    vec2 top    = mix(d, c, u);
    return mix(bottom, top, v);
}

void main() {
    float u = gl_TessCoord.x;
    float v = gl_TessCoord.y;

    vec3 coarsePos = bilerp3(vPosTC[0], vPosTC[1], vPosTC[2], vPosTC[3], u, v);
    // uv: ORTHOPHOTO-relative, for color sampling in the fragment shader
    // ONLY. heightUV: DEM-raster-relative, for uHeightmap ONLY. These are
    // NOT interchangeable — the orthophoto and DEM generally cover
    // different geographic extents, so using uv to sample uHeightmap (a
    // real bug in the first implementation) samples the wrong location
    // entirely, producing large, essentially uncorrelated displacement
    // deltas and visibly broken geometry.
    vec2 uv       = bilerp2(vUVTC[0], vUVTC[1], vUVTC[2], vUVTC[3], u, v);
    vec2 heightUV = bilerp2(vHeightUVTC[0], vHeightUVTC[1], vHeightUVTC[2], vHeightUVTC[3], u, v);

    // Requested directly: "the triangle's vertices must sit at the dem
    // provided altitude ... what is acceptable is to have a shader
    // controlled refinement with normal position driven by displacement
    // mapping. that is still not the case." Correct, and this replaces a
    // materially different (and materially wrong, for this specific use)
    // prior approach: an earlier version displaced each generated vertex
    // ALONG its interpolated surface normal, using a ray/tangent-plane
    // intersection to decide how far — carefully re-derived more than
    // once to be internally self-consistent, but solving the wrong
    // problem regardless. Moving along a TILTED normal changes X and Z,
    // not just Y, so that approach only ever landed the vertex on the
    // tangent PLANE through the true DEM point, not the point itself —
    // exactly right only where the surface happens to be flat
    // (normal.y == 1), increasingly wrong as terrain steepens.
    //
    // For a heightfield, elevation is ALWAYS a function of (X,Z) along
    // the fixed, global vertical axis — there is no local "surface
    // normal direction" involved in what "the DEM's altitude at this
    // point" even means. So the correct fix is simpler than what it
    // replaces, not more complex: keep X and Z exactly as given by the
    // coarse patch's own bilinear interpolation (that already correctly
    // identifies which (X,Z) this GPU-generated vertex is FOR — nothing
    // about that was ever wrong), and set Y directly to the true
    // elevation at that (X,Z), sampled from the full-resolution
    // heightmap. No normal, no projection, no per-slope approximation
    // error, no safety clamp needed (a direct sample can never produce an
    // implausible "overhang" the way a mis-scaled normal projection
    // could) — every displaced vertex sits EXACTLY at the DEM-provided
    // altitude for its (X,Z), for any slope.
    float fineElev = coarsePos.y;
    if (uDisplacementEnabled != 0) {
        // w is a smooth bump function that is EXACTLY ZERO along all four
        // patch edges (u or v == 0 or 1) and peaks at the patch center —
        // so at and along every boundary, fineElev == coarsePos.y exactly
        // BY CONSTRUCTION, regardless of any heightmap sampling
        // imprecision there (e.g. from downsampling — see
        // uploadHeightmapTexture()'s texel cap). That's what preserves
        // the crack-freedom guarantee (§5.1/5.2 of the design doc): a
        // coarse patch and any finer neighbor already agree exactly on
        // shared edge/corner positions before displacement ever runs, and
        // this never disturbs that agreement — only interior points
        // (where neighboring-patch agreement was never a concern in the
        // first place) move toward the true, full-resolution value.
        float w = 16.0 * u * (1.0 - u) * v * (1.0 - v);
        float fineElevRaw = textureLod(uHeightmap, heightUV, 0.0).r; // native res, true DEM value at this (X,Z)
        fineElev = mix(coarsePos.y, fineElevRaw, w);
    }

    vec3 displaced = vec3(coarsePos.x, fineElev, coarsePos.z);
    displaced.y *= uZScale;

    gl_Position = uProj * uView * vec4(displaced, 1.0);
    vUV = uv;
    vElev = fineElev;  // the ACTUAL displayed elevation (pre-zScale) — more
                       // accurate for the color ramp than the coarse
                       // bilinear guess now that the true value is
                       // available for free; matches coarsePos.y exactly
                       // when displacement is off, unchanged behavior
    vEdgeDist = min(min(u, 1.0 - u), min(v, 1.0 - v));
}
)GLSL";

// ---------------------------------------------------------------------------
// Hi-Z (max-mip depth pyramid) downsample shaders — see copc_streamer.h's
// TileGrid comment for the full occlusion-culling mechanism this feeds.
// ---------------------------------------------------------------------------

const char* kHiZVert = R"GLSL(
#version 330 core
// Procedural fullscreen triangle — no vertex buffer needed. A triangle
// covering (-1,-1) to (3,-1) to (-1,3) fully covers the NDC square
// (-1,-1)-(1,1) without needing a quad's extra 2 vertices; standard trick.
out vec2 vUV;
void main() {
    vec2 pos = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    vUV = pos;
    gl_Position = vec4(pos * 2.0 - 1.0, 0.0, 1.0);
}
)GLSL";

const char* kHiZCopyFrag = R"GLSL(
#version 330 core
// Initial capture pass: 1:1 copy from the REAL depth buffer (a
// GL_DEPTH_COMPONENT-format texture, readable directly as a normal
// sampler2D — depth textures are sampled just like color ones) into mip 0
// of the color-attachable (GL_R32F) Hi-Z pyramid. Kept as a separate,
// trivial pass rather than folded into the first downsample step, so mip
// 0 always represents "one real pixel per texel," unreduced — the
// downsample shader below only ever reduces FROM a previous Hi-Z mip,
// never from the raw depth texture directly.
in vec2 vUV;
out float oDepth;
uniform sampler2D uSrcDepth;
void main() {
    oDepth = texture(uSrcDepth, vUV).r;
}
)GLSL";

const char* kHiZDownsampleFrag = R"GLSL(
#version 330 core
// One mip level of max-reduction: each output texel takes the MAX (i.e.
// farthest, since GL depth is 0=near/1=far under the standard depth
// range) of the 4 texels below it in the previous mip. MAX, not average,
// is what makes this a conservative occlusion structure — see
// copc_streamer.h's TileGrid comment for why: a region is only safely
// cullable if EVERY pixel in it is occluded, so the worst case (farthest
// visible surface) across the region is the only value that can be
// compared against safely.
in vec2 vUV;
out float oDepth;
uniform sampler2D uSrcMip;
uniform vec2 uSrcTexelSize; // 1/srcWidth, 1/srcHeight, of uSrcMip specifically
void main() {
    float d0 = texture(uSrcMip, vUV + vec2(-0.5, -0.5) * uSrcTexelSize).r;
    float d1 = texture(uSrcMip, vUV + vec2( 0.5, -0.5) * uSrcTexelSize).r;
    float d2 = texture(uSrcMip, vUV + vec2(-0.5,  0.5) * uSrcTexelSize).r;
    float d3 = texture(uSrcMip, vUV + vec2( 0.5,  0.5) * uSrcTexelSize).r;
    oDepth = max(max(d0, d1), max(d2, d3));
}
)GLSL";

GLuint compileShader(GLenum type, const char* src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok = GL_FALSE;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (ok != GL_TRUE) {
        char log[2048];
        glGetShaderInfoLog(s, sizeof(log), nullptr, log);
        std::cerr << "Shader compile error:\n" << log << std::endl;
        glDeleteShader(s);
        return 0;
    }
    return s;
}

GLuint linkProgram(const char* vertSrc, const char* fragSrc) {
    GLuint v = compileShader(GL_VERTEX_SHADER, vertSrc);
    GLuint f = compileShader(GL_FRAGMENT_SHADER, fragSrc);
    if (!v || !f) return 0;
    GLuint p = glCreateProgram();
    glAttachShader(p, v);
    glAttachShader(p, f);
    glLinkProgram(p);
    glDeleteShader(v);
    glDeleteShader(f);
    GLint ok = GL_FALSE;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (ok != GL_TRUE) {
        char log[2048];
        glGetProgramInfoLog(p, sizeof(log), nullptr, log);
        std::cerr << "Program link error:\n" << log << std::endl;
        glDeleteProgram(p);
        return 0;
    }
    return p;
}

GLuint linkTessProgram(const char* vertSrc, const char* tcsSrc,
                       const char* tesSrc, const char* fragSrc) {
    GLuint v = compileShader(GL_VERTEX_SHADER, vertSrc);
    GLuint tc = compileShader(GL_TESS_CONTROL_SHADER, tcsSrc);
    GLuint te = compileShader(GL_TESS_EVALUATION_SHADER, tesSrc);
    GLuint f = compileShader(GL_FRAGMENT_SHADER, fragSrc);
    if (!v || !tc || !te || !f) {
        if (v) glDeleteShader(v);
        if (tc) glDeleteShader(tc);
        if (te) glDeleteShader(te);
        if (f) glDeleteShader(f);
        return 0;
    }
    GLuint p = glCreateProgram();
    glAttachShader(p, v);
    glAttachShader(p, tc);
    glAttachShader(p, te);
    glAttachShader(p, f);
    glLinkProgram(p);
    glDeleteShader(v);
    glDeleteShader(tc);
    glDeleteShader(te);
    glDeleteShader(f);
    GLint ok = GL_FALSE;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (ok != GL_TRUE) {
        char log[2048];
        glGetProgramInfoLog(p, sizeof(log), nullptr, log);
        std::cerr << "Tess program link error:\n" << log << std::endl;
        glDeleteProgram(p);
        return 0;
    }
    return p;
}

} // namespace shaders
