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

// DEM surface fragment shader (fed by kMeshTessEval).
//   uHasTexture == 1 : sample the orthophoto texture
//   uHasTexture == 0 : elevation color ramp (dark blue-violet -> teal -> yellow)
// uAuxMode relates the surface to the auxiliary raster uAux (meters, on the
// DEM grid, see DemAux in dem_tess_mesh.h):
//   1 : uAux is the ground — cut what is less than uThreshold above it
//   2 : uAux is the height of what stands here — no orthophoto where it is
//       above uThreshold (the image shows the top of that object, not the
//       ground): neutral grey instead, which reads through the translucent
//       objects above better than the elevation ramp
const char* kMeshFrag = R"GLSL(
#version 330 core
in vec2 vUV;
in vec2 vHeightUV;
in float vElev;
in float vEdgeDist;
out vec4 FragColor;
uniform sampler2D uTexture;
uniform int uHasTexture;
uniform float uMinElev;
uniform float uMaxElev;
uniform int uShowMasterEdges; // G key / DEM "Patch edges" — diagnostic, off by default
uniform float uOpacity;
uniform int uAuxMode;
uniform sampler2D uAux;
uniform float uThreshold;     // meters
uniform float uFrameScale;    // GL unit -> meters
uniform float uFrameCenterZ;  // meters at GL y = 0
uniform sampler2D uHeightmap; // R: GL-space heights, G: validity (see uploadHeightmapTexture)
uniform vec2 uHeightmapTexels;
uniform vec2 uTexelGL;        // GL size of one heightmap texel along x and z
uniform float uZScale;
uniform int uShade;           // hill-shading on/off

// Hill-shading from the heightmap gradient (light from the north-west, 45°
// up), normalised so flat ground keeps its colour: 1 on the flat, darker on
// slopes facing away, up to 1.3 on slopes facing the light.
float hillshade() {
    vec2 d = 1.0 / uHeightmapTexels;
    float hL = texture(uHeightmap, vHeightUV - vec2(d.x, 0.0)).r;
    float hR = texture(uHeightmap, vHeightUV + vec2(d.x, 0.0)).r;
    float hN = texture(uHeightmap, vHeightUV - vec2(0.0, d.y)).r; // row - 1: north, -GL z
    float hS = texture(uHeightmap, vHeightUV + vec2(0.0, d.y)).r;
    float hx = (hR - hL) / (2.0 * uTexelGL.x) * uZScale;
    float hz = (hS - hN) / (2.0 * uTexelGL.y) * uZScale;
    vec3 n = normalize(vec3(-hx, 1.0, -hz));
    const vec3 light = vec3(-0.5, 0.70710678, -0.5);
    float lit = max(dot(n, light), 0.0) / 0.70710678;
    return clamp(0.35 + 0.65 * lit, 0.0, 1.3);
}

void main() {
    // No surface where the DEM has no data (heightmap G = validity; the
    // heights there are only filled so that vertices stay level).
    if (texture(uHeightmap, vHeightUV).g < 0.5) discard;
    bool covered = false;
    if (uAuxMode == 1) {
        float aboveGround = vElev * uFrameScale + uFrameCenterZ - texture(uAux, vHeightUV).r;
        if (aboveGround < uThreshold) discard;
    } else if (uAuxMode == 2) {
        covered = uHasTexture == 1 && texture(uAux, vHeightUV).r > uThreshold;
    }

    // Master (coarse patch) edges — see vEdgeDist's producer in
    // kMeshTessEval. Drawn in
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

    if (covered) {
        FragColor = vec4(vec3(0.30), uOpacity);
    } else if (uHasTexture == 1) {
        FragColor = vec4(texture(uTexture, vUV).rgb, uOpacity);
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
        FragColor = vec4(c, uOpacity);
    }
    if (uShade == 1) FragColor.rgb *= hillshade();
}
)GLSL";

// ---------------------------------------------------------------------------
// GPU-tessellated DEM mesh shaders (src/dem_tess_mesh.cpp). Design and
// history: docs/design-tessellation-displacement.md (§6v for this version).
//
// Pipeline: kMeshTessVert (passthrough, one invocation per patch corner)
//        -> kMeshTessControl (TCS: per-edge tessellation levels)
//        -> kMeshTessEval (TES: bilinear position/UV within the patch, height
//           sampled from the heightmap for every vertex)
//        -> kMeshFrag (orthophoto or elevation ramp, hill-shading)
//
// Level of detail: the CPU quadtree (dem_quadtree.h) decides patch sizes;
// the TCS splits each patch edge by its on-screen length (one segment per
// uTargetPixelsPerSegment pixels), capped at the DEM pixels the edge spans.
//
// No cracks:
//   - an edge shared by two patches of the same level is split identically
//     by both (same endpoints, same formula), and every vertex takes its
//     height from the same heightmap texels;
//   - at a one-level transition (the most the CPU balance pass allows) the
//     coarse side splits its edge into 2K segments and each fine half edge
//     into K, so all vertices coincide.
//
// Requires GL 4.0+; targets #version 410 core (macOS's ceiling).
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
uniform float uTransitionSegments; // K, an integer: see outerLevelFor()
uniform vec2 uHeightmapTexels;     // heightmap size in texels

// Screen-space level for an edge, capped at the number of heightmap texels
// it spans: beyond one vertex per DEM pixel there is nothing new to sample.
// Both patches sharing an edge pass identical endpoints and UVs, so they
// compute identical levels (no crack).
float freeEdgeTessLevel(vec3 aGL, vec3 bGL, vec2 uvA, vec2 uvB) {
    vec3 a = vec3(aGL.x, aGL.y * uZScale, aGL.z);
    vec3 b = vec3(bGL.x, bGL.y * uZScale, bGL.z);
    float dist = max(length((a + b) * 0.5 - uCamPos), 0.0001);
    float pxPerUnit = uViewportH / (2.0 * dist * uTanHalfFov);
    float level = length(b - a) * pxPerUnit / uTargetPixelsPerSegment;
    float texels = length((uvB - uvA) * uHeightmapTexels);
    return clamp(min(level, max(texels, 1.0)), 1.0, 64.0);
}

// Edge codes from the CPU (dem_quadtree.h edgeCodes): 0 = free, 1 = this
// patch is the finer side of a one-level transition, 2 = the coarser side.
// The coarse side splits its edge into 2K segments and each fine half edge
// into K, so every vertex along the edge exists on both sides, at the same
// position and (sampled from the same heightmap) the same height.
float outerLevelFor(int i, int j, float code) {
    if (code > 1.5) return 2.0 * uTransitionSegments;
    if (code > 0.5) return uTransitionSegments;
    return freeEdgeTessLevel(vPosTC[i], vPosTC[j], vHeightUVTC[i], vHeightUVTC[j]);
}

void main() {
    vPosTC[gl_InvocationID]      = vPosVC[gl_InvocationID];
    vUVTC[gl_InvocationID]       = vUVVC[gl_InvocationID];
    vHeightUVTC[gl_InvocationID] = vHeightUVVC[gl_InvocationID];
    barrier();

    if (gl_InvocationID == 0) {
        // Corners 0..3 CCW from (col,row); edge codes x: 0→1, y: 1→2,
        // z: 2→3, w: 3→0 (the same on all 4 corners).
        vec4 ec = vEdgeConstraintVC[0];
        // Quad domain: OuterLevel[0] = u=0 edge (3→0), [1] = v=0 (0→1),
        // [2] = u=1 (1→2), [3] = v=1 (2→3).
        gl_TessLevelOuter[0] = outerLevelFor(3, 0, ec.w);
        gl_TessLevelOuter[1] = outerLevelFor(0, 1, ec.x);
        gl_TessLevelOuter[2] = outerLevelFor(1, 2, ec.y);
        gl_TessLevelOuter[3] = outerLevelFor(2, 3, ec.z);
        // Inner levels are private to the patch: free, capped the same way.
        gl_TessLevelInner[0] = max(freeEdgeTessLevel(vPosTC[0], vPosTC[1], vHeightUVTC[0], vHeightUVTC[1]),
                                   freeEdgeTessLevel(vPosTC[3], vPosTC[2], vHeightUVTC[3], vHeightUVTC[2]));
        gl_TessLevelInner[1] = max(freeEdgeTessLevel(vPosTC[0], vPosTC[3], vHeightUVTC[0], vHeightUVTC[3]),
                                   freeEdgeTessLevel(vPosTC[1], vPosTC[2], vHeightUVTC[1], vHeightUVTC[2]));
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
out vec2 vHeightUV;
out float vElev;
out float vEdgeDist; // parametric distance to nearest coarse-patch edge —
                     // see kMeshFrag for the red master-edge visualization
                     // this feeds. 0 exactly on an edge/corner, 0.5 at the
                     // patch center.

uniform mat4 uView;
uniform mat4 uProj;
uniform float uZScale;
uniform sampler2D uHeightmap;
uniform int uDisplacementEnabled; // 0: plain bilinear patches (diagnostic)

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

    // Every generated vertex, patch corners included, takes its height from
    // the heightmap at its own (X, Z): it sits on the DEM ("the triangle's
    // vertices must sit at the dem provided altitude"). Taking all heights
    // from the same texture also keeps shared edges watertight: two patches
    // generating a vertex at the same place sample the same texel values.
    // Off ("Displace" unchecked): the plain bilinear patch, a diagnostic.
    float fineElev = (uDisplacementEnabled != 0)
        ? textureLod(uHeightmap, heightUV, 0.0).r
        : coarsePos.y;

    vec3 displaced = vec3(coarsePos.x, fineElev, coarsePos.z);
    displaced.y *= uZScale;

    gl_Position = uProj * uView * vec4(displaced, 1.0);
    vUV = uv;
    vHeightUV = heightUV;
    vElev = fineElev;  // the displayed elevation, pre-zScale
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
