# Design: Hybrid Tessellation + Normal-Displacement for DEM/DSM Meshes

Status: **implemented and checked on an Apple GPU (GL 4.1) as of §6v**
(2026-10). The current design is summarized in §6v and `specs.md` §9.7;
the sections before it are the history of how it got there, several of
them superseded (each says so). Sections below are updated inline where the
implementation deviated from the original draft (search "IMPLEMENTED" /
"Deviation"). Companion to `specs.md` — once verified on real hardware, the
relevant points here should be folded into `specs.md` §9 (Rendering) as new
numbered subsections, and this file should be kept only as historical
rationale.

## 1. Problem statement

The current DEM/DSM path (`src/dem_mesh.cpp`) builds one static, adaptively
subdivided triangle mesh on the CPU at load time (quadtree, geometric-error +
texture-span driven, T-junction-welded, capped at `MAX_LEVEL=5`). This has
two structural limits:

1. **It's view-independent.** The mesh is built once from a fixed error
   threshold and never changes as the camera moves — unlike the COPC point
   streaming path (`src/copc_streamer.cpp`), which already recomputes desired
   detail every frame from the camera's actual view.
2. **Detail is binary, not scaled.** A cell either subdivides (pays full
   triangle cost) or doesn't (pays zero — flat facet). There's no cheap way
   to represent small-amplitude, high-frequency terrain (rock texture, scan
   noise, small ridges) without either overtessellating it or losing it.

Two techniques were considered and rejected as *complete* solutions before
landing on this design:

- **Normal/bump mapping** (perturb shading only, no depth change): fails at
  grazing/low-angle views, exactly where this viewer spends a lot of its
  time (§7.5/7.6 side/top view controls exist because low-angle inspection
  is a primary use case). Rejected.
- **Vertex displacement of the existing coarse mesh** (move existing
  vertices, add none): doesn't add silhouette detail between vertices, so it
  can't reduce triangle count in noisy zones without visible faceting.
  Rejected as the sole mechanism, but its core idea (push a vertex along a
  normal by a heightmap-sampled amount) is exactly right *if new vertices are
  generated first* — which is what hardware tessellation is for.

## 2. Chosen approach

**GPU hardware tessellation (TCS/TES) generating new vertices per patch,
each displaced along its interpolated normal by a heightmap texture sample.**
The existing CPU quadtree is *not* thrown away — it's repurposed as the
**coarse patch generator**, using a relaxed error threshold so it produces
far fewer, larger patches than today. Fine detail within a patch is filled
in at render time by the GPU, correctly, at every view angle, because the
generated points are real 3D geometry.

This requires **OpenGL 4.0+** (`GL_PATCHES` primitive, tessellation control
shader, tessellation evaluation shader — none exist before GL 4.0).
Confirmed acceptable: macOS has supported GL 4.1 core natively since OS X
10.9 (2013), on Intel and Apple Silicon alike, and continues to today despite
the 2018 API deprecation — so this doesn't shrink the realistic Mac support
window. Linux/Windows GL4 hardware has been standard since ~2010-2013.
macOS's ceiling is 4.1, so target **GL 4.1 core**, not higher.

## 3. Pipeline overview

```
CPU (load time, once per DEM):
  1. Build coarse patches (relaxed quadtree — see §4)
  2. Compute per-corner-vertex normals (finite-difference on heightmap)
  3. Upload heightmap as a texture (capped resolution — see §6)
  4. Upload patch corner positions + normals + UVs as GL_PATCHES (4 verts/patch)

GPU (every frame):
  5. Vertex shader:  pass-through (patches, no work per-vertex yet)
  6. Tessellation Control Shader (TCS): per patch, compute outer/inner
     tessellation levels from projected screen-space edge length (§5)
  7. Tessellation Evaluation Shader (TES): for each generated point,
     bilinear-interpolate position/normal/UV from the 4 patch corners,
     sample the heightmap at that UV, displace position along the
     interpolated normal by (fine_height - coarse_height) (§5.3)
  8. Fragment shader: unchanged — sample orthophoto or elevation ramp
```

## 4. CPU: coarse patch generation — IMPLEMENTED (revised from the original draft above)

**Status: implemented** in new, isolated files `src/dem_tess_mesh.h` /
`src/dem_tess_mesh.cpp` — `dem_mesh.cpp`/`DEMMesh` were left completely
untouched (beyond exposing `readDEMElevations()` for reuse) and remain the
GL 3.3 fallback. `main.cpp` negotiates a GL 4.1 core context first, falling
back to 3.3 core; `demTessSupported()` gates which path gets used.

**Deviation from the draft above, found during implementation:** the draft
proposed reusing the adaptive quadtree (relaxed thresholds) so each leaf
becomes one patch. This was changed to a **uniform (48×48 by default) grid
of patches** instead. Reason: an adaptive coarse quadtree can still produce
neighboring leaves at different levels (the existing balance pass only
bounds this to ≤1 level difference) — which reintroduces the exact
T-junction/crack problem DEMMesh's midpoint-welding exists to solve, just at
the coarse-patch granularity instead of the triangle granularity. The
original draft's §5.4 crack-avoidance argument (deterministic per-edge
tessellation factors) only prevents cracks *within* the fine tessellation
of same-level neighboring patches — it does nothing for a coarse-level
mismatch between a leaf and a smaller neighbor leaf.

Since GPU tessellation now supplies all view-dependent fine detail every
frame, the CPU coarse level's job changed from "capture geometry adaptively"
to "capture the DEM's large-scale shape at a fixed resolution" — for that,
a uniform grid is both simpler and crack-free by construction, since every
shared patch edge is bit-identical (same two grid vertices) rather than
needing to be reconciled between differently-sized neighbors. `PATCH_GRID`
(patches per side, default 48) is a tunable constant in
`dem_tess_mesh.cpp`, explicitly flagged there as needing real-hardware
tuning, not a value derived from any measurement.

As implemented (`DEMTessMesh::loadFromDEM()`, CPU-only, no GL calls —
called before the GL context exists, same convention as
`DEMMesh::loadFromDEM`):

```
function loadFromDEM(path, ortho):
    elevs = readDEMElevations(path)          # shared with dem_mesh.cpp
    (zMin, zMax, worldCenter, worldScale) = computeBoundsAndTransform(elevs)
    determine UV mode (geo-matched vs. stretch-fit) — same logic as
        DEMMesh::loadFromDEM, duplicated in dem_tess_mesh.cpp rather than
        factored into a shared helper, to keep this module's only
        cross-file dependency limited to readDEMElevations()

    N = PATCH_GRID  # 48
    for gy in 0..N, gx in 0..N:              # uniform (N+1)x(N+1) grid
        (col, row) = gridToRasterCoords(gx, gy, N)
        elev = bilinearSample(elevs, col, row)
        glPos = worldToGL(col, row, elev)     # same formula as DEMMesh
        normal = computeNormalGL(elevs, col, row)   # full-res, see below
        uv = uvFor(col, row)
        store (glPos, normal, uv) in the shared vertex grid

    build patchIndices: N*N patches, 4 grid-vertex indices each (CCW
        BL,BR,TR,TL) — no vertex duplication, no welding needed

    keep elevs, pre-converted to GL-space Y, for uploadGPU()'s heightmap
        texture (DEMMesh discards elevs after CPU mesh build; this module
        keeps it since the GPU needs it for displacement sampling)

function computeNormalGL(elevs, col, row):
    # Full derivation (see dem_tess_mesh.cpp for the worked-out comment):
    # the world->GL map is a uniform scale + axis permutation/reflection
    # (no shear), so normals transform the same way as position deltas —
    # the invScale factor cancels under normalize(), leaving only axis
    # correspondence/sign to handle correctly.
    dElev_dcol = (sampleElev(col+step, row) - sampleElev(col-step, row)) / (2*step)
    dElev_drow = (sampleElev(col, row+step) - sampleElev(col, row-step)) / (2*step)
    dElev_dwx = dElev_dcol / geo.A
    dElev_dwy = dElev_drow / geo.E   # dividing by geo.E (often negative)
                                       # folds in the row/Y-axis sign flip —
                                       # no separate manual negation needed
    return normalize(vec3(-dElev_dwx, 1.0, dElev_dwy))
```

Note the sign on the Z component: `+dElev_dwy`, not `-dzdy` as the original
draft's pseudocode had it — that draft was hand-wavy about the world→GL
axis mapping (specifically the Z-negation from `specs.md` §3.8) and the
implementation comment above derives it properly. Verified with
`test_dem_tess_normal` in `tests/test_basic.cpp` (flat terrain → straight
up; always unit length; deterministic).

### 4b. Angular (scale-invariant) geometric-error criterion

**Implemented** in response to user feedback framed exactly this way:
*"we can merge triangles when the merger doesn't change geometry: if a
point is [within] 1° of where the surface would lie without that vertex,
collapse it."* Replaced the original positional criterion — comparing a
sample's deviation from the bilinear-interpolated surface to
`(zMax-zMin) × 0.01`, an arbitrary fixed percentage of the DEM's TOTAL
elevation range — with an angular one: the same deviation is compared to
the tangent of a 1° angle, scaled by the CELL's own world size
(`worldCellSize × 0.5 × tan(1°)`). This is scale-invariant in the way a
fixed-fraction-of-total-range threshold isn't — a given absolute bump (say
10cm) means something different in a 1m cell (steep, must subdivide) than
in a 100m cell (negligible, stay coarse/"collapsed"), and the old
criterion couldn't distinguish those cases since it only compared against
the DEM's overall range, never the local cell size. Applied identically in
both `dem_tess_mesh.cpp` and `dem_mesh.cpp` (the GL 3.3 CPU-only
fallback), kept in sync. `ANGLE_THRESH_DEG = 1.0` is the user's stated
starting point, not independently tuned. Verified with
`test_angular_geom_error` in `tests/test_basic.cpp` (same absolute
deviation judged differently by cell size; monotonic in the threshold).

One explicit assumption worth restating: this assumes the DEM's
horizontal CRS units match its vertical (elevation) units — both meters,
true for the projected LiDAR-derived DEMs this app targets, not for an
unprojected lat/lon raster. Noted in the code, not otherwise handled.

## 5. GPU: tessellation control + evaluation

### 5.1 Tessellation Control Shader (TCS) — per-edge LOD — THIRD REVISION

**Crack avoidance is split by tessellation-level type**, because only
`gl_TessLevelOuter` (edges) can ever cause a visible crack —
`gl_TessLevelInner` (a patch's interior) is entirely private to that patch
and needs zero cross-patch agreement, so it stays fully free/continuous/
view-dependent no matter what the edges do:

- **Outer level, unconstrained edge** (neighbor is same quadtree level, or
  there's no neighbor — a raster-domain boundary): free continuous
  screen-space formula, same as originally drafted. Both patches sharing
  that edge independently compute *identical* corner endpoints (built from
  identical `(col,row)` inputs at CPU build time), so they independently
  arrive at identical tessellation levels — no coordination needed. This
  part of the original draft's argument was right and is unchanged.
- **Outer level, constrained edge** (neighbor is a *different* quadtree
  level — an LOD transition, e.g. right at a building's silhouette where
  the adaptive quadtree subdivides deeper; capped at exactly ±1 level by
  the balance pass, §4): a **fixed constant**
  (`CONSTRAINED_EDGE_TESS_LEVEL`, currently `4.0`, in `shaders.cpp`), used
  identically by the coarse patch AND both of its finer neighbor's
  half-edges. Two independently-computed *continuous* estimates on
  differently-sized patches cannot be trusted to land on matching segment
  counts (their raw formula results only differ by an exact factor of 2 in
  theory — in practice, clamping and GL's fractional `equal_spacing`
  subdivision rules break that exact relationship) — but two uses of the
  *same fixed constant* trivially match.

Each patch corner carries a 4-component `aEdgeConstraint` attribute
(`x`=bottom, `y`=right, `z`=top, `w`=left; `0`=free, `1`=fixed), computed
once on the CPU when the patch is built (`DEMTessMesh::loadFromDEM()`,
`edgeConstraintFor()`) by checking whether a same-position, different-level
leaf exists across that edge.

```
// per patch, 4 control points in, tessellation levels out
tcsMain():
    ec = edgeConstraint[0]  # same on all 4 corners of this patch

    for each of the 4 edges e in patch:
        if ec[e] > 0.5:
            outerTessLevel[e] = CONSTRAINED_EDGE_TESS_LEVEL   # fixed, e.g. 4.0
        else:
            midWorld   = (cornerPos[e.a] + cornerPos[e.b]) * 0.5
            midView    = viewMatrix * midWorld
            edgeLenGL  = length(cornerPos[e.b] - cornerPos[e.a])
            dist       = max(length(midView.xyz), epsilon)
            pxPerUnit  = viewportH / (2 * dist * tan(fov/2))
            edgeLenPx  = edgeLenGL * pxPerUnit
            outerTessLevel[e] = clamp(edgeLenPx / 8.0, 1.0, 64.0)  # free formula

    # INNER tessellation stays fully free/continuous regardless of any
    # edge's constraint — it's private to this patch, no crack risk.
    innerTessLevel[0] = max(freeFormula(edge_bottom), freeFormula(edge_top))
    innerTessLevel[1] = max(freeFormula(edge_left), freeFormula(edge_right))
```

**What this deliberately does NOT solve** (documented, not silently
ignored): even with matching *segment counts* on both sides of a
constrained edge, a coarse patch's GPU-*generated* (non-corner) boundary
vertices get displaced along a **tilted** normal (§5.3), which nudges them
in X/Z as well as Y — so they won't land bit-exactly on the finer
neighbor's true corner position. See §5.2's note and §9 for the full
explanation of why this residual exists and why it's expected to be small
in practice.

### 5.2 Tessellation Evaluation Shader (TES) — displacement

```
// gl_TessCoord = (u, v) barycentric/bilinear coord within the patch,
// provided by the GPU tessellator after TCS levels are applied.
tesMain(gl_TessCoord u, v):
    coarsePos    = bilerp(cornerPos[0..3], u, v)
    coarseNormal = normalize(bilerp(cornerNormal[0..3], u, v))
    uv           = bilerp(cornerUV[0..3], u, v)

    # "coarse elevation" = what the un-displaced coarse patch would give
    # at this (u,v) — i.e. bilinear interpolation of the 4 corner heights,
    # which is exactly what coarsePos.y already encodes (GL Y = elevation).
    coarseElev = coarsePos.y

    # IMPLEMENTED SIMPLIFICATION (see §5.3): the heightmap texture is
    # pre-converted to GL-space Y at upload time (DEMTessMesh::uploadGPU),
    # so no separate scale uniform is needed here — the sample IS already
    # in the same units as coarseElev.
    fineElevGL = texture(uHeightmap, uv).r      # full-res, GL-space Y
    delta      = fineElevGL - coarseElev

    displacedPos = coarsePos + coarseNormal * delta
    displacedPos.y *= uZScale   # same Z-exaggeration control as today

    gl_Position = uProj * uView * vec4(displacedPos, 1.0)
    vUV = uv
    vElev = coarsePos.y       # pre-zScale, for the elevation-ramp fallback
```

Key property: **at patch corners, `delta == 0` by construction** (the corner
was itself sampled from the heightmap when the patch was built), so corners
never move. On unconstrained (same-level) edges, this is sufficient for an
exact crack-free boundary — every generated boundary point uses linear
interpolation between two corners that are bit-identical on both
neighboring patches, and `delta` at any given (shared) UV samples the same
heightmap texel on both sides, so the displaced result matches exactly.

On **constrained** (different-level) edges, this guarantee only holds
exactly at the corners themselves — GPU-generated boundary points along the
coarse side interpolate `coarsePos`/`coarseNormal` from *its own*, larger,
2 far corners (a flatter approximation of the true surface than the finer
neighbor's own corner, which is an exact heightmap sample) and then correct
the *elevation* via `delta`, but the correction is applied along a
**tilted** `coarseNormal`, not straight up — so it also nudges X/Z. See §9.

### 5.3 Displacement direction and scale

- Direction: **interpolated per-vertex normal**, not fixed world-up — this
  was explicitly required (steep terrain must displace along the local
  surface normal, not vertically, or cliffs displace sideways-wrong). This
  is also the direct cause of the constrained-edge residual noted in §5.2 —
  a fixed world-up displacement would eliminate that residual entirely
  (X/Z would never move), but was explicitly rejected in favor of
  slope-correctness. Worth remembering as an available fallback if the
  residual turns out to be visually significant on real hardware.
- Scale: **implemented as a simplification of the original draft.** Rather
  than a separate `uElevScaleToGL` uniform multiplying a real-world-meters
  delta at render time, `DEMTessMesh::uploadGPU()` pre-converts every
  heightmap texel to GL-space Y (`(elev - worldCenter.z) / worldScale`,
  identical to what `coarsePos.y` already is) once, at load time. The TES
  then just subtracts two already-matching-unit values — one fewer uniform,
  one fewer multiply per generated vertex, and no risk of the two sides of
  the subtraction drifting out of unit-agreement.

### 5.4 Crack avoidance — summary (see §5.1 for the full mechanism)

Superseded by §5.1's split inner/outer, constraint-attribute-driven
strategy (third revision). The original draft's "deterministic edge
factor" argument only holds between same-size patches — it's preserved
exactly for unconstrained (same-level) edges, and replaced with a fixed
shared constant for constrained (different-level, LOD-transition) edges.
The CPU quadtree's `neighborLevel()`-style balance pass — initially
mentioned here as something that "can be retired for this mesh" — is in
fact **required**, both to build the adaptive patches in the first place
(§4) and to classify each edge as constrained/unconstrained.

## 6. Heightmap texture: resolution and format

The full-resolution heightmap must stay resident on the GPU for per-fragment
random access during tessellation (unlike the current code, which reads
`elevs` once during CPU mesh build and frees it, `dem_mesh.cpp` lines ~330).
A raw Float32 heightmap at typical DEM sizes is too large to keep resident
untouched — e.g. 10007×9878 × 4 bytes ≈ 380 MB, and VRAM budgets have to
share with the orthophoto texture too.

Mitigation, reusing an existing pattern: `geotiff.cpp` already box-filter
downsamples oversized orthophotos with correct affine rescaling (§6.3-6.4 of
`specs.md`). Apply the same treatment to the heightmap:

```
maxHeightmapPixels = 4096 * 4096   # ~16M texels
if (heightmapPixelCount > maxHeightmapPixels):
    downsample via box filter (reuse the existing fractional-mapping
                                downsample function, not integer division —
                                that was a real, previously-fixed bug, see
                                specs.md §6.3)
    rescale affine accordingly (as already done for orthophotos)
```

Format: `GL_R16F` (2 bytes/texel) rather than `GL_R32F` — half-float has
more than enough precision for a normalized, per-tile elevation range, and
halves VRAM cost again. At 4096×4096×2 bytes ≈ 32 MB, this is a
non-issue alongside a downsampled orthophoto texture.

**Implementation note**: R16F was NOT used (kept `GL_R32F` — see
`uploadHeightmapTexture()`'s comment in `dem_tess_mesh.cpp`) because the
TES's `delta = fineElevGL - coarseElev` subtracts two similar-magnitude
values, and R16F's ~3 decimal digits of relative precision per operand
risks visible quantization in that delta specifically for subtle relief
against a mostly-flat backdrop — exactly this project's motivating case.
Cap raised instead (4096²→6144², §"Implementation" below) as the safer of
the two available levers, unverified either way on real hardware.

### 6b. Reload-on-demand (implemented, in response to real hardware testing)

Real-hardware testing surfaced a design gap this section didn't originally
address: raising GPU tessellation density (S/F keys) past what the
heightmap texture's OWN resolution can represent doesn't add real detail —
it just smoothly interpolates a texture that already lost that detail at
load time (downsampled to the `MAX_HEIGHTMAP_TEXELS` cap). The user's own
framing: *"when user asks for finer grain, reload geotiff if needed, but
refining tessellation while keeping the same geometry makes no sense."*

Implemented as `DEMTessMesh::increaseHeightmapResolution()`: on every F
(density-increase) press, `main.cpp` calls it before rendering. It
re-reads the DEM file from disk (the full-resolution CPU array is
deliberately NOT kept resident between loads — see `uploadGPU()`'s
comment on why), doubles the effective heightmap resolution cap (clamped
to both the DEM's true native resolution and a hard safety ceiling of
16384² texels, to bound VRAM growth from repeated key-repeat on a very
large DEM), and re-uploads just the heightmap texture in place — patch
geometry (positions/normals/UVs) is untouched. Once native resolution is
reached, it correctly becomes a no-op and further density increases are
purely a GPU-tessellation-density question again, as intended.

Companion change: the CPU-side adaptive quadtree's `MAX_LEVEL` was reduced
from 6 to 3 — a deliberately much coarser BASE mesh (roughly 8x fewer
per-axis / 64x fewer patches at full subdivision, the closest power-of-
two approximation to the requested "~10x/~100x fewer"), on the reasoning
that once GPU tessellation + displacement can add real per-frame detail
from the (now reload-capable) heightmap texture, deep CPU-side
subdivision duplicates work the GPU path does better — the CPU quadtree's
remaining job is placing patch boundaries near sharp features and
providing per-vertex normals for displacement direction, not achieving
geometric fidelity by itself. `dem_mesh.cpp`'s CPU-only fallback path
(no GL 4.0+, §7) keeps `MAX_LEVEL = 6` unchanged — it has no GPU
tessellation to compensate, so coarsening it would be a straightforward
regression for users without tessellation-shader support.

### 6c. Density-tied coarse reference (implemented, in response to real hardware testing)

Real-hardware testing surfaced another design gap: displacement magnitude
never actually shrank as GPU tessellation density (S/F) increased, even at
maximum density. Root cause: `delta = fineElevGL - coarseElev` always used
`coarseElev = coarsePos.y`, the far-corner bilinear guess — completely
independent of how many extra triangles were generated within the patch.
More tessellation meant more *sample points*, not a better *baseline* to
measure the correction against, so displacement kept doing the same amount
of work regardless of density. The user's framing: *"when increasing the
tessellation to the max I should get closer to coarse tessellation +
displacement mapping [converging correctly]. At maximum tessellation,
displacement [magnitude] should be minimal."*

Fix, in `kMeshTessEval` (`shaders.cpp`): blend `coarseElev` toward a
**density-matched coarse mip level** of the (now mipmapped) heightmap
texture, using a weight `w = 16·u·(1-u)·v·(1-v)` — a bump function that is
**exactly zero along all four patch edges** and peaks at the center. Two
important consequences of that specific shape, not incidental:

- **Crack-freedom is completely unaffected.** Since `w == 0` everywhere on
  the boundary (not just at the 4 corners — the whole edge), every
  boundary point still computes `coarseElev` exactly the old way. The
  blend only ever touches the *interior*, where neighboring-patch
  agreement was never a concern in the first place.
- **No persistent accuracy error.** Both the base position's Y *and* the
  delta baseline use the same blended value
  (`basePos = vec3(coarsePos.x, coarseElevBlended, coarsePos.z)`), so
  `basePos.y + delta == fineElevGL` always holds regardless of how much of
  that sum comes from geometry (`coarseElevBlended`, density-dependent) vs.
  displacement (`delta`, shrinking as density increases). The
  decomposition shifts with density; final accuracy doesn't.

The coarse mip level itself: `max(0, log2(uTargetPixelsPerSegment))` — the
same density value the TCS's free-edge formula already uses, reused here
(no new CPU-side uniform). At the ~8px/segment default this is mip ≈3 (a
real, noticeable blur, roughly matching the original far-corner-only
behavior); as density increases toward the ~1px/segment clamp, the mip
level approaches 0 (native resolution) — the blended coarse reference gets
close to the true value, so `delta` shrinks toward zero, exactly the
requested property. A pleasant side effect: since the tilted-normal
boundary residual (§5.2/§9) is proportional to `delta`, it also shrinks at
high density in the interior, partially mitigating a previously-documented
open risk without any code aimed at that specifically.

`uploadHeightmapTexture()` now calls `glGenerateMipmap()` and uses
`GL_LINEAR_MIPMAP_LINEAR` (was plain `GL_LINEAR`, no mip chain) — applies
to both the initial upload and every `increaseHeightmapResolution()`
reload. Verified with `test_density_blend_weight` (edge-vanishing,
center-peaking) and `test_density_mip_level` (monotonic, converges to 0)
in `tests/test_basic.cpp` — the blend math, not the visual result, which
remains unverified on real GPU hardware like the rest of this feature.

### 6d. Horizon LOD floor + displacement overhangs (implemented, correcting an earlier over-correction)

Real-hardware testing reported two symptoms that turned out to share one
root cause: (1) increasing tessellation density had **no visible effect on
distant/horizon terrain** at all, and (2) displacement was producing
**geometrically impossible overhangs** — triangles displaced so far they
folded the surface over itself.

**Horizon floor, mechanism:** the TCS's free-edge formula
(`freeEdgeTessLevel`, §5.1) computes `edgeLenPx / uTargetPixelsPerSegment`,
clamped to `[1.0, 64.0]`. For a distant patch, `edgeLenPx` (its projected
on-screen size) is already tiny. The density ceiling was 8x, giving
`uTargetPixelsPerSegment` a floor of 1.0px — nowhere near small enough to
push an already-sub-pixel edge length above the tessellation-level floor
of 1.0. Once a patch is pinned at that floor, it gets **zero** interior
tessellation points (no subdivision at all, GL_PATCHES with outer/inner
level 1 generates only the 4 corners) — so no amount of further density
increase could ever add detail there; the knob simply couldn't reach far
enough. Fixed two ways: the density ceiling (`gl_app.cpp`'s `f` case) was
raised from 8x to 128x, AND a separate clamp inside
`DEMTessMesh::render()` (`clampedTargetPx`'s lower bound) was dropped from
0.5 to 0.01 — that second clamp was silently defeating the ceiling raise
by flooring the computed target straight back up to 0.5px regardless of
how high density went, which would have made the ceiling change a no-op
if left unfixed. Nearby geometry is unaffected either way — it's already
governed by the separate 64-segment upper clamp.

**Overhangs, root cause:** this traces directly back to §4b's `MAX_LEVEL`
reduction (6→3), made in direct response to an earlier "coarser base
tessellation" request on the theory that GPU tessellation + displacement
would gracefully absorb whatever detail the coarser CPU mesh no longer
captured. That theory doesn't hold at that level of coarsening: a
`MAX_LEVEL=3` patch can span enough real-world area to contain substantial
elevation variation entirely on its own, inside one patch. The resulting
displacement delta for interior points can exceed the patch's own
dimensions, and that delta is applied along `coarseNormal` — only a
bilinear blend of the 4 corner normals, an increasingly unreliable
direction estimate the larger and more varied a patch gets. Large
delta × unreliable direction folds the surface into an overhang. This
wasn't a subtle bug — it was a foreseeable consequence of the earlier
coarsening that should have been safeguarded against at the time and
wasn't.

Fixed three ways, not one — a safety net alone would have papered over a
real design mistake rather than correcting it:

1. **`MAX_LEVEL` walked back from 3 to 5.** Still coarser than the
   original 6 (roughly 2x fewer per axis / 4x fewer patches at full
   subdivision, vs. 3's 8x/64x), but nowhere near coarse enough to
   routinely produce patches displacement can't handle safely. A genuine
   partial reversal, not a full revert — the underlying idea (let GPU
   tessellation + displacement do more of the work) still holds, just not
   at the original level of aggressiveness.
2. **Displacement magnitude safety clamp**, added in the TES regardless
   of what `MAX_LEVEL` ends up being: `delta` is bounded to
   `patchDiag × MAX_DISPLACEMENT_FRACTION` (0.5, a starting point) before
   being applied, where `patchDiag` is computed from the patch's own
   corner positions (`max(length(vPosTC[2]-vPosTC[0]), length(vPosTC[3]-vPosTC[1]))`).
   This is a hard, unconditional safety net independent of how the base
   mesh ends up tuned in the future — a displacement larger than the
   patch itself is never geometrically sane, regardless of cause.
3. The horizon-floor fix above, since a coarser `MAX_LEVEL` alone doesn't
   help distant terrain gain detail from increased tessellation density
   without also being able to reach small enough target-pixel values.

Verified with `test_displacement_clamp` (bounds a pathological delta,
passes a legitimate one through unchanged, scales with patch size) in
`tests/test_basic.cpp` — again, the clamp math, not the visual result on
real hardware.

### 6e. Collapsing angle had no visible effect (implemented)

Reported directly: *"changing collapse angle doesn't change displayed
geometry at all."* Root cause: `dem_tess_mesh.cpp`'s quadtree subdivision
OR's together **three** independent criteria (`geomAngleExceeded`,
`texSpan > MAX_TEX_SPAN`, `normalVaries`) — but the I/O keys only ever
controlled `collapseAngleDeg`, which feeds `geomAngleExceeded` alone.
`normalVaries` was a second, independently-fixed ~20° threshold
(`NORMAL_ANGLE_THRESH_COS`). On real terrain with genuine local slope
variation, `normalVaries` alone could very plausibly be triggering
subdivision for most cells regardless of what `geomAngleExceeded`'s
threshold was set to — meaning changing the collapsing angle had no
visible effect whenever that was the case, since those cells subdivided
anyway via the other, untouched criterion. (`dem_mesh.cpp`, the CPU-only
fallback, doesn't have this third criterion at all, so this bug was
specific to the GPU-tessellated path.)

Fixed by making `NORMAL_ANGLE_THRESH_DEG` scale proportionally with
`collapseAngleDeg` relative to its 1.0° default
(`20.0 * (collapseAngleDeg / 1.0)`, clamped to `[1°, 89°]`) rather than
being a second, independently-fixed constant. At the default angle this
reproduces the original ~20° behavior exactly; the I key (finer) now
tightens both criteria together, the O key (coarser) loosens both
together — the user-controlled angle actually controls the full
subdivision decision, not just one of three OR'd conditions. Verified with
`test_normal_threshold_scales_with_collapse_angle` (reproduces the
original 20° at the default, scales proportionally, stays clamped) in
`tests/test_basic.cpp`.

### 6f. Displacement was using a vertical gap as if it were a normal-direction distance — SUPERSEDED, see §6s

**Superseded entirely by §6s.** §6f, §6g, and §6h below record real,
carefully-reasoned history — each correctly identified and fixed a bug in
the along-normal approach as it then stood — but the along-normal
approach itself turned out to be the wrong technique for a heightfield,
not something that needed a fourth correction. See §6s for why, and for
the simpler, exact replacement now in place. Left in place below,
unedited, as the honest record of how this was arrived at rather than
deleted — same practice as other superseded sections in this document
(e.g. §6q).

Reported directly, precisely, and correctly: *"you compute displacement
by looking at the vertical distance between the triangle and the DEM. you
must look at the normal distance."* This is a real, previously-unnoticed
bug distinct from (and more fundamental than) the size clamp added in
§6d — that clamp bounded the damage without fixing the underlying
miscalculation.

`fineElevGL` and `coarseElevBlended` are both elevation (Y) values sampled
at the same (X,Z) location — their difference (`verticalDelta`) is
therefore a **vertical** gap, not a distance along any particular
direction. The code was applying it directly as
`coarseNormal * verticalDelta`. That's only correct when the normal is
exactly vertical: moving a distance `d` along a unit normal changes Y by
`d × normal.y`, not `d`. On any sloped terrain (`normal.y < 1`), applying
the raw vertical gap as if it were already a normal-direction length:

- **Under-corrected the actual height reached** — the point never quite
  gets to `fineElevGL` vertically, by a factor of `normal.y`.
- **Simultaneously over-applied an uncontrolled horizontal (X/Z) shift** —
  `normal.x × verticalDelta` and `normal.z × verticalDelta`, amounts that
  were never validated against anything, since the vertical component was
  already wrong.

That combination — wrong magnitude, applied in a direction whose
horizontal component was never meant to scale with a raw vertical gap —
is a direct, mechanical explanation for genuinely impossible
(overhanging) displaced geometry, independent of and in addition to the
oversized-patch mechanism §6d addresses.

Fixed by solving for the actual along-normal distance that closes the
known vertical gap: `dispDist × normal.y == verticalDelta`, i.e.
`dispDist = verticalDelta / normal.y`, with `normal.y` floored at 0.1 to
avoid a division blowup on near-vertical terrain (`coarseNormal.y` is
always positive by construction — `computeNormalGL()` in
`dem_tess_mesh.cpp` normalizes `vec3(-dzdx, 1, dzdy)`, whose vertical
component starts at exactly 1 before normalizing, so it shrinks toward
but never reaches zero or goes negative). With this fix, `displaced.y`
correctly reaches `fineElevGL` whenever `normal.y` isn't floor-clamped —
previously it never did, on any non-flat terrain, even before the size
clamp or the floor kicked in. The §6d size clamp still applies afterward,
now clamping the *correct* quantity (the actual along-normal travel
distance) rather than the vertical gap that was being misused as one.

Verified with `test_displacement_normal_distance` in
`tests/test_basic.cpp` — confirms the corrected conversion exactly
reproduces the intended vertical gap when applied along a tilted normal,
and explicitly confirms the old (buggy) direct-application behavior would
not have.

### 6g. Clamp was scaled to the wrong reference size, and corners weren't actually exact under downsampling

**Superseded by §6s** — see §6f's note. The clamp this section fixes no
longer exists (removed entirely, not just re-tuned); the corners-exact
fix (the edge-vanishing weight `w`) is the one part of this section that
survives unchanged into the current formula.

Reported directly, immediately after §6f: *"I have displacement that are
one or two orders of magnitude the size of the triangles, and are not at
all null at vertex level. Default triangle size is a few meters, and I
have 100s of meters displacement."* Two distinct bugs, both real:

**Clamp reference size.** §6d's safety clamp bounded `dispDist` to a
fraction of `patchDiag` — the diagonal of the *coarse* patch, before GPU
tessellation subdivides it. A coarse patch can legitimately span tens to
hundreds of meters; GPU tessellation then subdivides that into many small
triangles a few meters wide (§5.1's screen-space density formula). A
clamp scaled to the whole coarse patch could therefore still let a single
small triangle displace by an amount vastly larger than itself — exactly
"1-2 orders of magnitude the size of the triangles." Made worse by §6f's
normal-distance fix, which can amplify the raw vertical gap up to 10x on
steep slopes (the 0.1 floor on `safeNormalY`).

Fixed by clamping against the actual **local tessellated segment size**
instead. The TCS now computes a per-patch estimate
(`vSegmentSizeTC = patchDiag / average(outer tessellation levels)`,
written once per corner by every invocation independently — cheap,
avoids a second `barrier()`) and passes it to the TES, which clamps
`dispDist` to `±vSegmentSizeTC × MAX_SEGMENT_MULTIPLE` (4.0, a starting
point) instead of a fraction of `patchDiag`. This bounds displacement to
something that can never look absurd relative to its own neighboring
geometry, regardless of coarse patch size or slope.

**Corners weren't actually exact.** The whole crack-avoidance design
(§5.1/§5.2) rests on `verticalDelta == 0` exactly at patch corners/edges,
which was implemented by blending `coarseElevBlended` toward
`coarsePos.y` there (§6c) — but `fineElevGL` was still sampled RAW
(`textureLod(uHeightmap, heightUV, 0.0)`, unblended), on the assumption
that the texture reproduces each corner's exact build-time elevation at
that exact UV. That assumption only holds if the heightmap texture is at
native resolution. If the DEM was large enough to trigger
`uploadHeightmapTexture()`'s downsample-to-cap (very plausible — the cap
exists specifically for large DEMs), the texture holds box-filtered
AVERAGES near a corner, not an exact point sample — so `verticalDelta`
was NOT actually zero at corners whenever downsampling occurred,
undermining the crack-avoidance guarantee at its foundation. Reported
directly: displacement "not at all null at vertex level."

Fixed by blending `fineElevGL` toward `coarsePos.y` with the SAME
edge-vanishing weight `w` used for `coarseElevBlended` — guaranteeing
`fineElevGL == coarseElevBlended == coarsePos.y` at every boundary point
BY CONSTRUCTION, independent of whatever the texture actually contains
there. The raw high-resolution sample is only actually used in the patch
interior, where real detail is wanted and boundary-matching was never a
concern.

Verified with `test_displacement_clamp_uses_segment_not_patch_size`
(confirms the new bound is strictly tighter than the old patch-diagonal
one for a small-triangle/large-patch scenario matching the report) and
`test_fine_elev_exact_at_boundary` (confirms exact equality at the
boundary regardless of raw-sample imprecision, and that interior points
still use real detail) in `tests/test_basic.cpp`.

### 6h. The displacement-distance formula itself was wrong — a THIRD correction (implemented)

**Superseded by §6s** — see §6f's note. This section's `verticalDelta *
normal.y` formula was a real, correctly-reasoned fix at the time, but the
along-normal approach it corrected turned out to be the wrong technique
entirely, not something a fourth correction should be layered onto.

Reported directly: *"you must not take the vertical gap between the
vertex coarse reference and the true elevation: you must take the
distance between the coarse vertex and the interpolated surface in the
direction of the normal at the vertex to the underlying geometry."* This
is a proper ray/surface-intersection framing, and it revealed that §6f's
"fix" — while correctly identifying that a vertical gap isn't a
normal-direction distance — solved the wrong equation.

§6f computed `dispDist = verticalDelta / normal.y`, which forces the
displaced point's Y to exactly equal `fineElevGL` — but `fineElevGL` was
sampled at the vertex's ORIGINAL (X,Z), and displacing along a tilted
normal changes X/Z too. "Exactly hitting a Y value sampled somewhere
you're no longer standing" is not the same as landing on the true
surface — it's an arbitrary target once you've moved. Worse, dividing by
a SMALL `normal.y` (steep terrain) AMPLIFIES the result — a direct,
mechanical explanation for the "1-2 orders of magnitude" oversized
displacement reported one exchange earlier, on top of (and possibly a
bigger contributor than) the wrong clamp reference size §6g fixed.

**Correct derivation**, matching the report precisely: model the true
surface locally as the tangent plane passing through
`Q = (X0, fineElevGL, Z0)` (the true height at the coarse vertex's OWN
X,Z) with the interpolated normal `n` as the plane's normal. The ray
`basePos + n·s` intersects this plane where `dot(basePos + n·s - Q, n) = 0`;
solving for `s` (with `n` already unit length):

```
s = dot(Q - basePos, n) = dot((0, verticalDelta, 0), n) = verticalDelta * n.y
```

**Multiply by `normal.y`, not divide.** Sanity check: at `normal.y = 1`
(flat terrain) this degenerates to `s = verticalDelta`, exactly matching
pure vertical displacement, as it must. On steep terrain it now *shrinks*
the along-normal distance relative to the raw vertical gap — the opposite,
self-limiting behavior compared to the divide formula — which is also why
the near-vertical-terrain division-blowup guard (`safeNormalY`, §6f) is
gone entirely: there's no division left to guard against. The zero-at-
corners guarantee (verticalDelta == 0 there, §6c/§6g) still holds
trivially for either formula, since both are linear in `verticalDelta`.

Verified with `test_displacement_normal_distance` in `tests/test_basic.cpp`
— confirms the flat-terrain degenerate case, confirms the corrected
formula shrinks rather than amplifies on steep terrain, explicitly
computes what the old divide formula would have given for the same inputs
to demonstrate the difference, and confirms zero-at-corners holds
regardless of slope.

### 6i. Master (coarse patch) edge visualization (implemented)

Requested directly: highlight the edges of the coarse patches — as
distinct from the GPU-tessellated triangle edges — in red, replacing
whatever color (orthophoto or elevation ramp) would otherwise show there.
Useful for directly seeing patch boundaries, independent of the wireframe
toggle (`W`, which shows every tessellated triangle edge, not just the
coarse patch boundaries) — e.g. for visually checking crack behavior at
constrained edges (§5.1) or how patch size varies across a DEM.

Implemented as a new varying, `vEdgeDist`, computed in `kMeshTessEval`
(the TES) as `min(u, 1-u, v, 1-v)` — the patch-parametric distance to the
nearest edge (0 exactly on a boundary, 0.5 at the patch center) — and
consumed in `kMeshFrag` (shared by both DEM paths): if `vEdgeDist` is
below `MASTER_EDGE_THRESHOLD` (0.03, a starting point), the fragment is
drawn solid red instead of sampling the orthophoto or elevation ramp.
Since `kMeshFrag` is linked into two different programs (`kMeshVert` for
the CPU-only `DEMMesh` path, `kMeshTessEval` for the GPU-tessellated
`DEMTessMesh` path — see `linkProgram()` vs `linkTessProgram()`),
`kMeshVert` also declares a matching `vEdgeDist` output, hardcoded to 1.0
("always far from an edge") — `DEMMesh` has no per-patch parametric
coordinate at the vertex-shader stage (it's a single, already-fully-
triangulated static mesh, not GPU-tessellated patches), so this
visualization is specific to `DEMTessMesh` and correctly never triggers
on the fallback path.

**Update:** was implemented unconditional (always on, no toggle), on
direct request at the time. That turned out to be the wrong default in
practice — asked directly how to turn it off. `uShowMasterEdges` (a new
uniform, set from `InputState::showMasterEdges`) now gates the check in
`kMeshFrag`, toggled by the `G` key, **default off** — this is a
diagnostic overlay, not a rendering feature, so it shouldn't be in the
way unless asked for (matching `W`'s wireframe toggle's own default-off
convention, not `A`'s displacement toggle, which defaults on because it's
a real rendering behavior). Threaded through both `DEMMesh::render()` and
`DEMTessMesh::render()`'s parameter lists — `DEMMesh`'s copy is a no-op
either way (`vEdgeDist` is hardcoded to 1.0 there regardless, per below),
kept explicit rather than relying on an un-set uniform defaulting to zero.

The line's on-screen thickness is patch-relative, not a fixed world-space
or screen-space width, since `MASTER_EDGE_THRESHOLD` is compared against
a parametric [0, 0.5] quantity — so larger coarse patches will show a
visually thicker red boundary than smaller ones for the same threshold
value. Verified with `test_master_edge_distance` in `tests/test_basic.cpp`
(zero at all four edges/corners, maximum at center, monotonic toward each
edge) — the distance math, not the visual result on real hardware.

### 6j. S/F reassigned from GPU tessellation density to the quadtree depth ceiling (implemented)

Requested directly, following a question about the max theoretical patch
count (`COARSE × 2^MAX_LEVEL` per axis, 256×256≈65k at the time):
*"that is the number I want to change with S and F."* `MAX_LEVEL` — until
now a fixed constant per DEM path (5 for `DEMTessMesh`, 6 for the
CPU-only `DEMMesh` fallback) — became a runtime, per-instance member
(`maxLevel`, mirroring how `collapseAngleDeg` was already made adjustable
for the I/O keys), threaded through `loadFromDEM()`'s and `reload()`'s
signatures in both classes.

`F` increments it, `S` decrements it, clamped to `[0, 10]` — there's no
smaller meaningful increment for a quadtree depth, so each press is a
2x jump in the per-axis patch-count ceiling. Like I/O, this triggers a
full `reload()` (re-reads the DEM from disk, rebuilds the whole patch
set), not a cheap render-time uniform update, since `MAX_LEVEL` bounds a
load-time subdivision decision. The two reload triggers (`collapseAngleChanged`,
`demMaxLevelChanged`) are checked together in `main.cpp` and reloaded
once with the current value of both, so pressing I/O and S/F in close
succession doesn't cause two separate reloads.

**This fully replaced, not layered onto, S/F's previous DEM meaning.**
S/F previously drove GPU tessellation density (`uTargetPixelsPerSegment`,
§6d/§6j's predecessor) for the DEM-tessellation path; that render-time
control is now fixed at the original 8px/segment baseline, and
`input.density` reverted to being exclusively a point-cloud (LAZ/COPC)
control again (including its ceiling, dropped back from the 128x §6d
introduced to the original 8x — that extension existed specifically for
the DEM horizon-floor problem, which S/F no longer touches).

**One capability was orphaned by this, not restored elsewhere, and is
worth flagging rather than burying:** `DEMTessMesh::increaseHeightmapResolution()`
— progressively fetching a higher-resolution heightmap *texture* from
disk (§6b) — was triggered by the same F key via a `densityIncreaseRequested`
flag that no longer gets set by anything. That was solving a genuinely
different problem (how much real detail is available in the texture
displacement samples from) than patch *count* (this section). The
function itself is untouched and still callable; nothing currently calls
it. `demMaxLevelChanged`'s `reload()` still rebuilds the heightmap at the
initial `MAX_HEIGHTMAP_TEXELS` cap each time (via `uploadGPU()`), not
progressively beyond it. Revisit if that capability is still wanted,
under a different key or folded into this one.

`demMaxLevel` initializes (`main.cpp`, right after `useDEMTess` is decided)
to whichever DEM path actually ended up active's current depth — `DEMMesh`
and `DEMTessMesh` have different defaults (6 vs 5) — so the first S/F
press starts from the real current value rather than an arbitrary shared
default. No new CPU-testable formula here (the clamp logic is a one-liner
integer increment/decrement) — the reload plumbing itself was the
substance of this change, not new math.

### 6k. Top-down subdivision replaced with bottom-up collapse (implemented — algorithm change, not a tuning fix)

**Superseded by §6t.** This section's move to bottom-up collapse was a
real, correctly-reasoned fix for a real problem (the aliasing risk
described below) — but it wasn't the only way to fix that problem, and it
came with a real, admitted cost (`O(4^MAX_LEVEL)` unconditionally, even
over enormous flat regions a single test could resolve immediately). §6t
replaces it with a top-down traversal again, fixed at the actual root
cause this time (what the test measures, not which direction the tree is
built), regaining early termination without reintroducing the aliasing
risk. Left in place below, unedited, as the honest record of why bottom-up
was chosen at the time — same practice as other superseded sections in
this document (§6f-h, §6q).

Requested directly: *"this is a repeating operation. collapse until there
is nothing to collapse."* The quadtree in both DEM paths was, until now,
built **top-down**: start at the COARSE grid, test each cell's own 5
sample points against the angle/normal/texspan criteria, subdivide only
if that cell's own samples show excess error, recurse into children. This
has a real, structural aliasing risk: a coarse cell's error can look
acceptable at its own sample points while hiding genuine fine detail
*between* them — the same class of problem the earlier switch from a
1-sample to a 5-sample geometric-error test (§4) partially addressed, just
recurring at a coarser granularity that more samples-per-cell alone can't
fully close.

Replaced with **bottom-up collapse**: for each initial COARSE cell,
recursively resolve all the way to `MAX_LEVEL` first (so every finest-
level cell always exists — nothing can be hidden), then repeatedly try to
merge each group of 4 sibling leaves into their parent, but *only* when
the parent cell itself would pass every criterion as a single piece.
Implemented as a single recursive function (`buildCollapse`, using
`std::function` for the self-reference) rather than an explicit
"repeat until no change" loop — processing proceeds naturally from
`MAX_LEVEL` down to each branch's stopping point in one pass, which *is*
the fixed point: a cell can't merge further once its parent fails the
test or `COARSE` (level 0) is reached. The existing balance pass
(iterative, ≤1-level-neighbor-difference invariant) is unchanged and runs
afterward on the resulting leaf set exactly as before — bottom-up merging
doesn't guarantee that invariant on its own (a large flat region could
merge much coarser than a neighboring detailed one), so it's still needed.

**Performance, stated plainly rather than glossed over:** this
necessarily visits every finest-level cell at least once — there's no way
to know a parent is safe to merge without first resolving what's really
inside it — so cost scales as `O(4^MAX_LEVEL)` per initial COARSE cell.
Now that `MAX_LEVEL` is user-adjustable up to 10 (§6j), that's up to ~1M
finest cells per COARSE cell, ~67M total across the full COARSE=8 grid, in
the worst case. No early-pruning heuristic is implemented to bound this —
reload was already documented as "will hitch the frame" (§6b), and a
longer hitch at extreme `MAX_LEVEL` is an honest, disclosed consequence of
asking for that much resolution combined with this more thorough
algorithm, not a silent regression. A cheap pre-pass bound (e.g. skip
recursing into a subtree once a fast upper-bound test proves it can't
possibly merge) would be the natural follow-up if this proves too slow at
high `MAX_LEVEL` in practice.

Applied identically to both `dem_tess_mesh.cpp` and `dem_mesh.cpp` (the
CPU-only fallback), kept in sync as always — `dem_mesh.cpp`'s
acceptability test stays single-sample with no normal-variation criterion
(it never had either), matching its existing behavior; only the traversal
direction changed there.

**Also this session:** the master-edge visualization's line thickness
(`MASTER_EDGE_THRESHOLD`, §6i) was reduced from 0.03 to 0.01 on direct
request ("have them thinner").

### 6l. Background threading — load coarsest, display it, swap when the full build is ready (implemented)

Called out directly as something that "should have been included from the
start": *"load coarsest, and display it while we compute the full
picture. swap representation when ready. same when changes require
rebuilding the representation."* Fair — every load and every I/O/S/F
rebuild had been synchronous up to this point, blocking the frame (§6b
already used the phrase "will hitch the frame" as an accepted tradeoff,
which this addresses directly rather than continuing to accept). This
became a much more pressing problem after §6k's bottom-up collapse, which
can genuinely take a long time at high `maxLevel`.

**Scope decision, stated upfront:** implemented fully for `DEMTessMesh`
(the GPU-tessellated path this whole feature has centered on).
`DEMMesh` (the CPU-only GL-3.3 fallback) stays synchronous — threading it
would mean duplicating the background-build infrastructure against a
different class with a different (file-static, lazy-init) GPU resource
pattern, on the less-tested, less-central path. `DEMMesh::reload()` still
hitches the frame on I/O/S/F, same as before. Flagged as a known,
deliberate gap, not an oversight.

**Mechanism.** The background thread builds an entirely **separate,
temporary `DEMTessMesh` instance**, via `loadFromDEM()` completely
unchanged (not duplicated, not re-derived) — it never touches the *real*
instance's members at all, so there is no data race with the main thread,
which keeps rendering the current (old) representation uninterrupted the
whole time. Only the final GPU upload (`uploadGPU()`, fast relative to a
slow bottom-up collapse) happens on the main thread, inside
`pollBackgroundBuild()`, moving the finished CPU-side results out of the
temporary instance and into the real one just before uploading.

```
requestBackgroundBuild(path, ortho, angle, maxLevel):
    if a build is already running:
        remember (angle, maxLevel) as "latest requested", return  # non-blocking
    else:
        start a new background thread now

background thread:
    tmp = new DEMTessMesh()          # fully isolated, no shared state
    ok = tmp.loadFromDEM(...)        # the SAME method, unchanged, CPU-only
    if ok: hand off `tmp` (mutex-protected)
    mark "no longer in progress"

pollBackgroundBuild(ortho):          # called every frame, unconditionally
    if a finished build is waiting:
        move its CPU-side fields into `this`
        this.uploadGPU(ortho)        # GL work — main thread only
    if nothing is running AND a request was coalesced while one was:
        start it now
```

**Why coalescing instead of queuing or cancelling.** At most one
background thread is ever alive at a time. A new request arriving while
one is already running does not start a second thread (bounding resource
use) and does not block the caller (the actual point of this feature) —
it overwrites "the latest requested parameters," which
`pollBackgroundBuild()` picks up and starts automatically the moment the
current build finishes. Rapid I/O/S/F key-repeat naturally collapses into
just the final requested state instead of processing every intermediate
step. True mid-flight cancellation (aborting a build partway through) was
not implemented — it would need cooperative check points scattered through
the recursive `buildCollapse` from §6k, adding real complexity for a
benefit (saving a discarded build's CPU time) that coalescing already
captures for the common case (the user settling on a final value, not the
transient intermediate ones).

**Initial load** now goes through the same mechanism: the window opens
after a synchronous build at `maxLevel=0` (just the initial `COARSE` grid,
no subdivision — always fast regardless of DEM size, so there is
something on screen immediately, not a wait proportional to the real
target detail), then immediately requests a background build at the real
default (`maxLevel=5`, `collapseAngleDeg=1.0`), which lands via the same
per-frame `pollBackgroundBuild()` call every other rebuild uses.

**Safety.** `~DEMTessMesh()` joins any in-flight background thread before
allowing the object to be destroyed (the thread captures `this` by
reference; letting the object die while it might still run would be a
use-after-free) — same reasoning `TileGrid::stop()`/`~TileGrid()` already
established for the point-cloud streaming thread in `copc_streamer.cpp`,
followed here rather than inventing a new shutdown convention. This can
cause a brief hang at program exit if a build is genuinely mid-flight —
an accepted, standard tradeoff for correctness over instant shutdown.

**Not implemented, worth naming explicitly:** no on-screen "rebuilding..."
UI indicator — `backgroundBuildInProgress()` exists on the class
specifically to make adding one straightforward, but progress is
currently only visible via the existing `[dem-tess]` console log messages
(`L` key overlay). Worth adding if the silence during a long background
build proves confusing in practice.

### 6m. Nodata detection only ever caught one hardcoded sentinel value (implemented — root-cause fix)

The "only display geometry where there is DEM data" request (originally
addressed in §6d's nodata-culling work) was raised again — correctly, it
turned out, because the ORIGINAL fix had a real, separate gap. Nodata
detection everywhere (`sampleElev()`'s per-corner clamping, `isNodataAt()`,
the elevation-range scan) only ever compared against a hardcoded
`< -9000.0f` threshold. That catches the common `-9999` convention, but a
DEM declaring a *different* nodata sentinel — `0`, a large-magnitude
negative float-min value under a different exact constant, or anything
else — would have those pixels silently treated as valid elevation,
generating real geometry exactly where there should have been none. That
looks precisely like "displays geometry where there's only orthophoto,"
without the bottom-up-collapse rewrite (§6k) or anything else being at
fault — the nodata *culling logic itself* was always structurally sound
(verified again by re-reading it end to end before concluding this); the
*detection* feeding into it was the gap.

Fixed by reading the DEM's own declared NODATA value — the `GDAL_NODATA`
tag (42113, a de facto standard, not part of the baseline TIFF spec, so
not a libtiff-predefined constant; stored as an ASCII string) — via a new
shared function, `readDEMNodataValue()` (`dem_mesh.h`/`.cpp`, alongside
the existing shared `readDEMElevations()`). Both DEM paths now build a
unified `isNodataValue(v)` check per load: exact-ish match (small relative
tolerance, for float round-trip through ASCII tag parsing) against the
declared value if the tag is present, falling back to the original
`< -9000.0f` heuristic only for files that don't declare one at all. Every
call site that previously hardcoded the threshold — both files'
`sampleElev()` and `isNodataAt()`, the elevation-range scan, the heightmap
texture conversion loop, and (for consistency, even though currently
unreachable — see §6j) `increaseHeightmapResolution()`'s own re-read path
— now goes through this shared check instead.

Deliberately NOT touched: the Terrain RGB decoding path's own
`< -9000.0f` check (`dem_mesh.cpp`, inside `decodePixel()`'s
`isTerrainRGB` branch) — that's a self-contained, format-specific
convention (values below -9000 after the R/G/B→elevation decode indicate
nodata, per the IGN MNS LiDAR HD encoding this app already documents
supporting) with no relationship to the general-purpose `GDAL_NODATA` tag
mechanism; changing it wasn't part of this fix and would have been out of
scope.

Verified with `test_nodata_declared_value` in `tests/test_basic.cpp` —
specifically includes the `0.0f`-as-nodata case, since that's the exact
scenario the old hardcoded threshold would have missed (`0` is not less
than `-9000`), alongside confirming the legacy fallback still works
unchanged for files with no declared tag.

### 6n. True cancellation instead of let-it-finish coalescing (implemented — correction to §6l)

§6l's background rebuild deliberately chose "coalesce, let the superseded
build run to completion, just discard the result" over true cancellation,
reasoning that cooperative cancellation would need check points scattered
through the recursive `buildCollapse` for a benefit (saving one discarded
build's CPU time) coalescing already captured for the common case. That
reasoning was overruled directly: *"when a background task is obsolete it
must be stopped before starting the new one that rendered it obsolete."*
Fair, especially given §6k's bottom-up collapse can be genuinely slow at
high `maxLevel` — letting an already-obsolete build run to full completion
wastes real time and competes for CPU with the build that actually matters.

Implemented via a shared cancellation flag
(`std::shared_ptr<std::atomic<bool>>`, one per build — not a single reused
flag, specifically so reassigning it for a NEW build never affects an
OLDER build's thread, which keeps its own captured copy alive
independently until it actually exits). `loadFromDEM()` gained an optional
`const std::atomic<bool>* cancelFlag` parameter (`nullptr` by default,
preserving existing behavior for every synchronous call site), checked:

- On every single `buildCollapse` invocation (the dominant cost per §6k)
  — cheap (an atomic read is nanoseconds against a per-cell cost of
  several elevation samples), so this doesn't meaningfully slow an
  uncancelled build, while making a cancelled one stop branching almost
  immediately — unwind time is bounded by current recursion depth
  (≤`MAX_LEVEL`), not by however much of the exploration remained.
- Once, immediately after `buildCollapse` completes, before the balance
  pass and patch-building work (also non-trivial, no point running either
  on a result about to be discarded).
- Once per balance-pass iteration.

`requestBackgroundBuild()` now sets the flag when superseding a running
build, in addition to the existing coalescing (which is still needed —
the new request still doesn't start immediately, since starting a second
thread concurrently with the still-unwinding old one would defeat "at
most one background thread alive at a time" for no benefit).

**Logging, requested directly** ("background task start finish and abort
must appear in logs"): every transition now has its own explicit,
distinct `[dem-tess]` message — `background rebuild START`, `FINISHED`
(built but not yet swapped in), `SWAPPED IN` (consumed by
`pollBackgroundBuild()`), `ABORTED` (superseded, cooperative cancellation
completed), `FAILED` (I/O or other error). Previously these were less
consistently distinguished (a single "starting..." and a single "ready,
swapped in" message, with no distinct abort/failure wording).

### 6o. Whether the collapsing angle "really works" — ANSWERED in §6v: it did not (the test measured the cell's elevation range, so every sloped cell failed); fixed there — a diagnostic, and an honest hypothesis (implemented: histogram logging; not implemented: a fix, because none is confirmed necessary)

Raised as a direct doubt after §6e's fix (which corrected a real bug: the
normal-variation criterion dominating independently of the user-controlled
angle). Re-reading the current merge logic end to end did not turn up a
second logic bug of that kind — but that doesn't mean the angle is
behaving the way it might be expected to, and guessing further without a
way to check would just be another unverified claim added to a long list
of them this session. Two things instead:

**A concrete, testable hypothesis**, worth stating plainly rather than
hedging: for genuinely noisy real terrain, `MAX_LEVEL` (S/F) may be the
dominant factor determining the final patch count, not the collapsing
angle (I/O), across a wide "reasonable" range of the angle. Reasoning: the
geometric-error threshold scales with each cell's own world size
(§4b — that's the whole point of the angular, scale-invariant
reformulation), but real terrain roughness does not shrink proportionally
at every scale the way a perfectly smooth or gently-curving surface would
— natural ground typically has *some* irregularity at nearly every scale
(a well-known property of real terrain, related to its fractal-like
roughness). If that roughness exceeds the angular threshold at the
finest tested scale regardless of whether the angle is 0.5° or 5°, a
merge will keep failing at that scale either way, and subdivision will
bottom out at `MAX_LEVEL` — a hard ceiling the angle test can never
override — almost everywhere, making the angle's practical effect small
within an ordinary range and only really significant at the extremes
(near 0°, or loose enough — tens of degrees — to tolerate the DEM's actual
noise floor).

**What was actually built: a way to check this empirically, per-DEM,
instead of trusting either the mechanism or this hypothesis blindly.**
Both DEM paths now log a leaf-count-by-level histogram every build/rebuild
(`[dem-tess] leaf level histogram (angle=X°, maxLevel=Y): L0=... L1=...`).
If most leaves sit at exactly `MAX_LEVEL` regardless of how far I/O is
pushed within a normal range, that directly confirms the hypothesis for
your specific data — and the corresponding fix would be a different one
than "the angle math is broken": either the angle needs to be pushed much
further than expected to see an effect (already possible, just
non-obvious without this histogram), or the merge criteria need a
different formulation for noisy terrain specifically (e.g. testing
against local *variance* rather than a single worst-sample deviation) —
a real follow-up, not attempted here without first confirming which
situation actually applies via the histogram.

### 6p. CRASH: unregistered custom TIFF tag caused a type-confusion EXC_BAD_ACCESS (fixed — §6m regression)

> **Resolved differently (GDAL raster I/O).** Declared nodata values are now read with GDAL's `GetNoDataValue()`; the libtiff-based reading described in §6p/§6p-2 no longer exists. Kept as history.

§6m's nodata fix (`readDEMNodataValue()`) crashed on real hardware:

```
EXC_BAD_ACCESS (code=1, address=0x3ff0000000000000)
frame #0: libtiff.6.dylib`_TIFFVGetField + 924
frame #2: lasviewer`readDEMNodataValue(tiff*, float&) + 48
frame #3: lasviewer`DEMMesh::loadFromDEM(...) + 772
```

**Root cause.** `TIFFTAG_GDAL_NODATA` (42113) is a GDAL-private tag, not
part of the baseline TIFF spec. libtiff does not know its type unless
explicitly told via `TIFFMergeFieldInfo()` first — calling `TIFFGetField()`
on an unregistered custom tag is unsafe, because libtiff's varargs-based
field dispatch (`_TIFFVGetField`) has no way to know what type of value
to write through the pointer it's given, and can write the wrong
type/size into it. The crash address itself is the tell: `0x3ff0000000000000`
is exactly the IEEE-754 bit pattern of the double `1.0`, misinterpreted as
a pointer — a textbook type-confusion signature. `readDEMNodataValue()`
passed a `char**` expecting an ASCII string; libtiff, not knowing tag
42113's real type, wrote something else through it.

**Fix**, in `dem_mesh.cpp`: register the tag as `TIFF_ASCII` via
`TIFFMergeFieldInfo()` before ever calling `TIFFGetField()` on it —

```cpp
static const TIFFFieldInfo nodataFieldInfo[] = {
    { TIFFTAG_GDAL_NODATA, -1, -1, TIFF_ASCII, FIELD_CUSTOM, 1, 0,
      const_cast<char*>("GDALNoDataValue") }
};
if (TIFFMergeFieldInfo(tif, nodataFieldInfo, 1) != 0) {
    return false; // do NOT proceed to TIFFGetField on this failure path
}
```

This is not an invented workaround — it's the same standard registration
GDAL's own libtiff-based readers use for this exact tag (field_tag,
readcount=-1/writecount=-1 meaning "count determined by the tag's own
data", type=`TIFF_ASCII`, `FIELD_CUSTOM` marking it as dynamically
registered rather than a fixed core field, okToChange=true,
passCount=false). `-1` is used as a literal rather than the `TIFF_VARIABLE`
macro specifically to avoid any risk of that symbol not being exported by
a particular libtiff header version — its numeric meaning is stable and
documented. If `TIFFMergeFieldInfo()` doesn't cleanly return success, the
function falls back to the legacy heuristic rather than proceeding to the
unsafe `TIFFGetField()` call regardless — there's no reliable,
version-independent way to distinguish "tag already safely known" from
"something is actually wrong" from the return code alone, and the cost of
guessing wrong is a crash, not a cosmetic issue, so any non-success case
is treated as "don't risk it."

**Same pattern found elsewhere, deliberately NOT touched.**
`geotiff.cpp`'s `readGeoTIFFTags()` reads `ModelPixelScaleTag` (33550) and
`ModelTiepointTag` (33922) — both GeoTIFF-spec tags, also not baseline
TIFF — via bare `TIFFGetField()`, the identical unregistered-tag pattern.
This has NOT crashed across this entire session's DEM/orthophoto loads;
the likely explanation is that GeoTIFF's core tags are common and
standard enough that many libtiff builds bundle them into their own
internal extended-tag table by default, unlike the more GDAL-specific
`GDAL_NODATA` tag — an inference from observed behavior, not a guarantee
for every libtiff build this app might run against. Flagged with a
comment at the call site pointing to this fix as the pattern to apply if
it ever does crash, rather than preemptively changing working, stable
code immediately after a real crash in a related area — that would add
risk without a demonstrated need.

No new unit test for this one: the bug is in libtiff's *type dispatch*
for an unregistered tag, not in any of this codebase's own formulas — not
something the dependency-free CPU test suite can exercise without linking
real libtiff, which it deliberately doesn't (see the test suite's own
"no external deps" design goal, `tests/test_basic.cpp`).

### 6p-2. SAME CRASH RECURRED after the §6p fix — feature disabled, not fixed a third time

The `TIFFMergeFieldInfo()` registration in §6p was a real, standard,
targeted fix — and it crashed again anyway, in exactly the same place,
same crash-address signature (`0x3ff0000000000000`), on real hardware.
Critically, the registration call itself was not rejected (the code path
that logs "registration did not succeed" and safely bails out before ever
reaching `TIFFGetField()` was NOT what ran — the crash trace shows
execution reaching `TIFFGetField` → `_TIFFVGetField` exactly as before).
That means `TIFFMergeFieldInfo()` reported success, yet the tag still
wasn't safely typed afterward — a different, more specific problem than
"forgot to register," one that can't be diagnosed further without a real
libtiff instance and a debugger to iterate against, which isn't available
in this environment.

**Decision: disable the feature entirely rather than attempt a third
variant of the same mechanism from the same position of being unable to
verify it.** `readDEMNodataValue()` is now a permanent no-op — it always
returns `false`, regardless of the TIFF passed to it, with no
`TIFFGetField` call of any kind. Every caller already treats `false` as
"no declared value found, fall back to the legacy `< -9000` heuristic"
(§6m) — that heuristic is the original, long-proven-safe behavior from
before this feature existed, and every DEM load now unconditionally uses
it. `test_nodata_declared_value` in `tests/test_basic.cpp` still passes
unchanged — it tests the `isNodataValue` comparison *logic*, not the
disabled TIFF-reading function, and that logic is still correct and still
exercised (via the always-taken heuristic branch).

This does reintroduce the original, narrower correctness gap this
feature was meant to close — a DEM declaring an unusual nodata sentinel
(e.g. `0`) won't be detected, exactly the case that motivated §6m in the
first place. That's accepted as the right tradeoff: a correctness gap
under a specific, narrow condition is a strictly better failure mode than
a crash on every load. If this is worth revisiting, it needs a
fundamentally different, independently-verifiable approach — parsing the
TIFF IFD directly at the byte level (tag/type/count/value-or-offset,
handling both byte orders), bypassing libtiff's per-tag type dispatch
entirely so there's no "wrong type" for it to guess — not another
attempt at registering the tag through the same `TIFFGetField()`
mechanism that has now failed twice under two different, independently
reasoned fixes.

### 6q. GPU tessellation density scaled by CPU-judged flatness — implemented, then reverted (architectural conflict with the whole point of this feature)

**Originally raised**, correctly: *"I don't see that we subdivide where
the surface is far from the DEM and keep coarse where the surface is
correctly approximated by large triangles."* GPU tessellation density
(`freeEdgeTessLevel`, §5.1) was, and is again, driven purely by
screen-space pixel size, with no relationship to whether the underlying
surface actually needed more triangles or was already fine as a flat
patch — a large flat patch close to the camera gets tessellated for no
geometric reason; a highly-detailed patch far away gets almost none.

**What was tried**: reuse `geomErr` relative to its own angular threshold
(already computed, previously discarded, during the CPU-side bottom-up
collapse, §6k) as a per-patch `deviationRatio`, uploaded as a new GPU
attribute, and used in the TCS to scale `freeEdgeTessLevel`'s effective
target-pixels-per-segment down (never up) for patches the CPU judged
well-approximated. Constrained (LOD-transition) edges were deliberately
exempted, to avoid breaking crack-freedom. A follow-up tuning pass
lowered the scaling's floor from a 10x to a 50x maximum reduction, after
it was reported that large flat areas still showed a lot of triangles
even with the mechanism active.

**Reverted entirely, on direct architectural grounds**: *"every vertex
should have its position from the dem. the only interest of having
displacement mapping is to refine on the fly and apply displacement to
generated vertices."* This is the crux, and it's a real conflict, not a
matter of degree — §6k's bottom-up collapse and the whole reason this
project uses GPU tessellation + per-vertex displacement rather than
relying purely on the CPU-side adaptive quadtree is to avoid trusting a
coarse, finite-resolution judgment about where detail exists: the CPU's
5-sample test can only ever check a handful of points at any given level,
and even `MAX_LEVEL` has a finite floor — real detail between those
sample points can be missed, at any level, no matter how good the
bottom-up collapse gets. GPU tessellation generating many vertices, each
individually corrected against the *full-resolution* heightmap via
displacement, is what's supposed to catch whatever that necessarily
limited CPU sampling missed.

§6q broke that guarantee by construction: it reduced how many vertices
get generated using the *same* CPU judgment the whole mechanism exists to
be independent of. If the CPU's flatness call was wrong — real detail
existing between its sample points, exactly the failure mode the
bottom-up collapse minimizes but can never fully eliminate — §6q meant
fewer vertices would be generated there, so fewer chances for
displacement to catch and correct what was missed. The fix optimized away
the safety net using the same signal the safety net was supposed to be
independent of. Reducing polygon count based on an *actual*, verified
measurement of local geometric complexity (rather than the CPU's own
prior judgment) would be a legitimate, different kind of optimization;
scaling based on the same coarse signal the GPU stage exists to
double-check is not.

**Fully reverted**, not just neutered — `QuadCell::deviationRatio`,
`patchDeviationRatio` (CPU array, VBO, attribute location 5),
`aDeviationRatio`/`vDeviationRatioVC` (vert/TCS), the `densityScale`
parameter threading through `freeEdgeTessLevel`/`outerLevelFor`, and
`test_deviation_based_tess_density` are all removed rather than left in
place disabled — dead plumbing for a reverted mechanism would only
confuse future maintenance. `freeEdgeTessLevel` is back to depending on
`uTargetPixelsPerSegment` and screen-space distance alone, exactly as
before §6q.

### 6r. Non-overlapping orthophoto was stretched onto the DEM instead of being skipped (implemented)

Reported directly: *"when the [ortho]tif is bigger than the dem, the full
orthophoto is displayed; only the dem should be displayed textured."*
Root cause was a real, if narrow, policy bug — not the geometry-culling
mechanism (§6m/§6d), which is unaffected and still correct, and not (after
careful re-derivation, twice, of the overlap-check math itself) a
precision bug in the overlap test — both DEM and orthophoto bounds are
already properly min/max-normalized before the comparison.

The actual issue: when the orthophoto **has** geo tags but its extent
turns out not to geographically overlap the DEM at all (e.g. covers a
much larger, different area), both `DEMTessMesh::loadFromDEM()` and
`DEMMesh::loadFromDEM()` set `orthoStretch = true` — the SAME fallback
used for the genuinely different case of an orthophoto with **no** geo
tags at all. That's the bug: with no geo tags, there's no correspondence
information to contradict, so stretch-to-fit is a reasonable, deliberate
convenience. With geo tags that say the two rasters don't correspond,
stretching anyway is actively misleading — it shows imagery from a
completely unrelated location, which is exactly "the full orthophoto is
displayed" when it shouldn't be.

Fixed by separating the two cases properly. New field, `orthoUsable`
(mirrored in both `DEMTessMesh` and `DEMMesh`, default `true`, reset to
`true` at the start of every `loadFromDEM()` call since these objects can
be reloaded with a different DEM/ortho pairing): set to `false` **only**
in the has-geo-but-no-overlap branch, leaving the no-geo-tags branch's
`orthoStretch = true` completely untouched. `uploadGPU()`
(`DEMTessMesh`) and `main.cpp`'s DEMMesh texture-upload block both now
check this flag and skip texture upload entirely when it's `false` — the
mesh renders with the elevation color ramp instead, same as if no
orthophoto had been provided at all. Also threaded into
`DEMTessMesh::pollBackgroundBuild()`'s move-assignment list (§6l), since
it's populated by `loadFromDEM()` on the temporary background-build
instance and needs to transfer into the real instance before
`uploadGPU()` runs.

No new unit test — the overlap-check math itself is unchanged (confirmed
correct on inspection, twice), this is a policy/wiring change (what
happens once "no overlap" is determined), not a new formula.

### 6s. Along-normal displacement replaced with direct vertical correction — the along-normal approach was never the right technique (implemented — supersedes §6f-6h; its "vertices exactly on the DEM" claim only became true in §6v, which removed the edge-vanishing blend)

Raised directly, and correctly: *"if i have a triangle, the triangles
vertices must sit at the dem provided altitude. what is acceptable is to
have a shader controlled refinement with normal position driven by
displacement mapping. that is still not the case."* True — and re-
deriving the math confirms exactly why, precisely: §6h's "corrected"
formula (`dispDist = verticalDelta * normal.y`, a proper ray/tangent-
plane intersection) does **not** land the displaced vertex at the true
DEM point. It lands on the tangent *plane* through that point — a
different location whenever the surface isn't exactly flat. Working
through what `basePos + normal * dispDist` actually evaluates to:

```
displaced.y = coarseElevBlended + normal.y * (verticalDelta * normal.y)
            = coarseElevBlended + verticalDelta * normal.y²
```

That equals `coarseElevBlended + verticalDelta` (the true value) only
when `normal.y == 1` — exactly flat terrain. On any real slope, the
result falls short by a factor of `normal.y²`, growing worse as terrain
steepens. §6f, §6g, and §6h each correctly fixed a real bug in the along-
normal approach as it then stood (a wrong axis, a wrong clamp reference,
a wrong scaling direction) — but the approach itself was never going to
get to "vertices sit at the DEM-provided altitude," because moving along
a *tilted* normal inherently changes X and Z as well as Y, so even a
"correct" along-normal formula can only ever land near the true point,
never exactly on it, except in the flat degenerate case.

**The actual fix is simpler than what it replaces, not more complex.**
For a heightfield, elevation is always a function of `(X, Z)` along the
fixed global vertical axis — there's no reason to involve a local,
tilted surface normal at all; that's a technique for general bump-mapped
surfaces where there's no fixed "up," not for terrain. The corrected TES
(`kMeshTessEval`, `shaders.cpp`):

```glsl
vec3 coarsePos = bilerp3(...);           // X, Z here were always correct
float w = 16.0 * u*(1-u) * v*(1-v);      // same edge-vanishing weight as before
float fineElevRaw = textureLod(uHeightmap, heightUV, 0.0).r;
float fineElev = mix(coarsePos.y, fineElevRaw, w);
vec3 displaced = vec3(coarsePos.x, fineElev, coarsePos.z);  // X, Z unchanged; Y set directly
```

No normal, no ray/plane intersection, no `verticalDelta`, no per-slope
attenuation — the displaced vertex's Y is *exactly* the true heightmap
value at its `(X, Z)`, for any slope, not an approximation that happens
to be good when flat. Corners and edges still match exactly (`w == 0`
there, unchanged from §6g's fix), preserving crack-freedom by the same
mechanism as before — displacement never touches shared boundaries,
regardless of which formula computes the interior.

**Also removed, not left in place unused**: the entire per-vertex normal
pipeline that existed only to feed the along-normal projection —
`patchNormals` (CPU array), `normalVBO`, `aNormal`/`vNormalVC`/`vNormalTC`
(GPU attribute chain), `coarseNormal` (TES) — and `vSegmentSizeTC` (TCS
output that existed only to size the now-gone displacement clamp; a
direct sample can't produce an "impossible overhang" the way a mis-scaled
normal projection could, so no clamp is needed at all anymore).
`computeNormalGL` itself is kept — it's still correctly used for the
CPU-side `normalVaries` quadtree-subdivision criterion (§4b), a genuinely
separate purpose from the removed GPU pipeline. Attribute location 1
(formerly `aNormal`) is left as a documented gap rather than renumbering
locations 2-4, to keep this a minimal, lower-risk diff.

As a small additional correctness improvement enabled by having the true
value available for free: `vElev` (feeding the elevation color ramp when
no orthophoto texture is present) now carries the actual displayed
elevation (`fineElev`) rather than the coarse bilinear guess — matches
`coarsePos.y` exactly when displacement is off, so no behavior change in
that case, but strictly more accurate when displacement is on.

Three tests for the removed mechanism were deleted rather than kept
disabled — `test_displacement_clamp` (an even earlier, already-superseded
patch-diagonal clamp), `test_displacement_normal_distance` (the §6h
ray/plane-intersection formula), and
`test_displacement_clamp_uses_segment_not_patch_size` (the segment-based
clamp, §6g) — none of these mechanisms exist anymore. `test_fine_elev_
exact_at_boundary` was kept and its comment corrected (it still
accurately describes the current, simpler formula's boundary behavior).
A new test, `test_displaced_y_exact_regardless_of_slope`, directly
verifies the property that was requested: the displaced Y matches the
true heightmap value exactly regardless of slope, and explicitly computes
what the old formula would have given for the same inputs to demonstrate
that it fell meaningfully short on steep terrain.

### 6t. Back to top-down subdivision — fixed at the root cause this time, plus OpenMP (implemented — supersedes §6k; its min/max bound replaced by an exact deviation in §6v)

A three-part conversation led here, worth recording in order since each
part changed the conclusion:

1. **"What if we tested top-down, subdividing only on failure?"** —
   raised as a genuine question, not a demand. Answered honestly: this is
   exactly what §6k's bottom-up collapse replaced, and for a real reason
   — a coarse cell's 5-point sample test can pass while real detail hides
   between those points, invisible to it. Bottom-up avoids that by
   exploring every branch to `MAX_LEVEL` before ever merging back up, at
   real, admitted `O(4^MAX_LEVEL)` cost even over trivially flat regions.
2. **"Bottom-up doesn't actually test against the real DEM either —
   there's no tradeoff to actually test geometry against highest-precision
   DEM."** Correct, and the sharper framing: bottom-up's test is the
   *same* 5-point sample, just applied at whatever depth it happens to
   stop at. Its correctness is bounded by `MAX_LEVEL`'s granularity
   relative to the DEM's true resolution, not absolute — it doesn't
   eliminate the aliasing blind spot, it relocates it to a finer scale and
   pays a lot of redundant exploration to get there. The actual fix was
   never about which direction the tree gets built; it's about what the
   test measures.
3. **"Let's explore parallelism and SIMD first"**, then **"migrate to top
   down with OpenMP."** Once the test itself became exact (below), the
   original objection to top-down no longer applied, and its early-
   termination advantage — a real, substantial one, previously given up
   entirely — became safe to take back.

**The fix: a min/max elevation pyramid, built once per load, queried in
O(1) per cell.** Same structure as the Hi-Z depth pyramid already built
for occlusion culling (§6l is a different feature, but the reuse of the
technique is deliberate) — level 0 = native DEM resolution, each level up
a 2×2 min/max reduction of the level below. A bilinear surface through 4
corners is convex, so its own min/max over a footprint is just the min/max
of those 4 corners (`planeMin`/`planeMax`). Given the pyramid-derived TRUE
min/max anywhere in that footprint (`trueMin`/`trueMax`), the worst-case
deviation anywhere in the footprint is bounded by
`max(trueMax - planeMin, planeMax - trueMin)`. This bound can be loose
(the true extremes and the plane's extremes need not occur at the same
point) but is always *safe*: if it's within tolerance, the actual surface
is guaranteed to fit everywhere in the footprint, not just at sample
points; looseness only ever means subdividing somewhat more than the
tightest-possible answer would, never missing real detail. Verified with a
standalone extraction of the actual pyramid code, compiled and run
directly (`g++ -Wall -Wextra`, outside the full GL/PDAL build this sandbox
can't compile) — confirming, concretely, that a spike hidden at the dead
center of a large flat region (invisible to any coarser sampling) is
correctly caught, that flat regions correctly report zero error, and that
nodata handling (fully and partially nodata regions) behaves as intended.
Mirrored as proper project tests: `test_minmax_pyramid_flat_dem`,
`test_minmax_pyramid_catches_hidden_detail`,
`test_minmax_pyramid_nodata_handling` in `tests/test_basic.cpp`.

**Nodata handling in the pyramid**: a nodata pixel contributes `+inf` to
the min pyramid and `-inf` to the max pyramid — the identity elements for
those operations — so an entirely-nodata region naturally resolves to
`(+inf, -inf)` with no separate validity mask needed, and a partially
nodata region correctly reflects only its valid pixels (nodata's `±inf`
never wins a min/max against a real value). The query function's caller
checks for a non-finite result and treats it as "nothing to verify
geometrically here" — nodata cells are still decided by the existing,
separate `nodataCount()`/`isNodataAt()` mechanism (§6m), unchanged.

**The traversal (`buildTopDown`, replacing `buildCollapse`)**: test first,
recurse only on failure — no merge step at all, since a top-down
traversal never descends past the point where staying coarse stopped
being justified, so there's nothing to reassemble afterward. The angular
threshold and normal-variation criterion are unchanged from the bottom-up
version, evaluated from the same 4 corners as before; only the positional
test's source of truth changed, from 5 discrete samples to the exact
pyramid bound.

**OpenMP, over the independent COARSE-grid cells** — the same detection
macro and `#pragma omp parallel for` convention already used in
`geotiff.cpp` and `point_cloud.cpp`, not a new threading approach.
`schedule(dynamic)`, not `static`: with early termination restored,
per-cell workload is now highly uneven (a flat cell resolves in a handful
of O(1) checks; a detailed one still recurses deeply), unlike the flat,
uniform per-point loops OpenMP was already used for elsewhere — static
scheduling would let one thread get stuck with several expensive cells
while others sit idle. Each of the `COARSE*COARSE` top-level cells writes
into its own local vector (`perCellLeaves[i]`), merged sequentially after
the parallel region completes, avoiding any shared-vector data race.

Applied identically to both `dem_tess_mesh.cpp` and `dem_mesh.cpp`, kept
in sync as always. `dem_tess_mesh.cpp`'s background-build cancellation
flag (§6l) is still checked on every `buildTopDown` invocation, now from
potentially multiple OpenMP worker threads simultaneously — safe, since
`std::atomic<bool>` reads are inherently thread-safe regardless of how
many threads read them concurrently; `dem_mesh.cpp` has no such flag to
begin with (no background-build cancellation exists for that path), so no
equivalent change was needed there.

**Not verified**: real performance measurement on actual multi-core
hardware (this sandbox reports a single core) — the parallelization is
reasoned to be safe and embarrassingly parallel, not benchmarked. Also
not implemented: SIMD beyond what a compiler auto-vectorizes at `-O2` —
raised and discussed as a real opportunity specifically for pyramid
construction (a regular, branch-free grid reduction, a much better SIMD
fit than the traversal itself), but not hand-vectorized with intrinsics
in this pass.

### 6u. CRS mismatch: read both files' actual CRS, reproject with PROJ when they differ (implemented)

> **Superseded (GDAL raster I/O).** Raster reading moved to GDAL (`src/raster.*`). The scene now has one CRS, and orthophotos and DEMs in another CRS are warped into it pixel by pixel on load, so the corner-based affine refit below, the hand-written GeoKey parsing and the direct PROJ dependency are gone. Kept as history.

§6r's "orthophoto doesn't overlap the DEM" fallback (skip texturing rather
than stretch a mismatched image) was working as designed — but it never
answered *why* a DEM and orthophoto that genuinely cover the same ground
could numerically fail to overlap. Traced directly to the actual cause:
this codebase read `ModelPixelScaleTag`/`ModelTiepointTag` (the affine
transform) from both files, but never read the tag that identifies *what
CRS those coordinates are even in* — `GeoKeyDirectoryTag` (34735). The
overlap check was comparing raw numeric coordinates on the unstated
assumption that both files already shared a CRS; if they don't (e.g. one
in a projected system, one in geographic lat/lon, or two different
projected systems), the numbers simply don't correspond to the same
real-world locations, regardless of whether the ground truth overlaps.

**Detection**: `readEPSGCode()` (`geotiff.cpp`) parses
`GeoKeyDirectoryTag` for `GTModelTypeGeoKey` (1024) plus
`ProjectedCSTypeGeoKey` (3072) or `GeographicTypeGeoKey` (2048) — the
common case of a CRS referenced by a standard EPSG code directly
(`TIFFTagLocation == 0`, an inline SHORT value). Deliberately does not
attempt to resolve a custom, hand-defined CRS (which would additionally
need `GeoDoubleParamsTag`/`GeoAsciiParamsTag` parsing) — the large
majority of real-world GeoTIFFs reference a standard EPSG code. New
`Orthophoto::epsg` field (0 = unknown), shared by both the orthophoto and
the DEM's own `geo` transform (both reuse this struct type already).

**Same tag-reading risk class as a real, confirmed crash this session —
addressed by reusing the pattern that's already proven safe, not the one
that crashed.** `GeoKeyDirectoryTag` is an unregistered custom tag from
libtiff's point of view, the same category as `TIFFTAG_GDAL_NODATA`
(§6p/6p-2's `EXC_BAD_ACCESS`). But it's read here using the exact "count +
pointer" `TIFFGetField()` calling convention (with the same defensive
single-pointer fallback) already used for
`ModelPixelScaleTag`/`ModelTiepointTag` in this same file — a pattern
that has not crashed across this entire session's loads — rather than the
registration-based approach that crashed twice for `GDAL_NODATA`. Core
GeoTIFF-spec tags appear to be commonly pre-known to libtiff builds in a
way `GDAL_NODATA` (a more GDAL-specific extension) isn't; that's an
inference from observed behavior for `ModelPixelScaleTag`/`ModelTiepointTag`,
extended here on the same reasoning, not a new, untested assumption — but
still the same risk class, still not something to treat as fully
guaranteed safe without real hardware verification.

**Reprojection**: `reprojectToMatchCRS()` — when both files have a known,
*differing* EPSG code, transforms the orthophoto's 4 corners via
[PROJ](https://proj.org) (`proj_create_crs_to_crs` + `proj_trans`) into
the DEM's CRS, then re-fits a still axis-aligned (no rotation/shear —
matching this codebase's affine model, which never supported those even
before this) A/E/C/F from the reprojected corners, averaging both edges
per axis so a small asymmetric distortion splits evenly rather than
biasing toward one edge. This is a **local linear approximation**, not a
general-purpose non-linear reprojection — correct where the true
transform between two CRSs is itself close to affine over a small area
(a single orthophoto tile's extent), which is the expected case; not
survey-grade accuracy over arbitrarily large areas or CRS pairs with
severe local distortion.

Both `dem_tess_mesh.cpp` and `dem_mesh.cpp`'s `loadFromDEM()` now
introduce a local `orthoEff` pointer (defaulting to the original `ortho`)
and `orthoReprojected` copy, used in place of the original `ortho`
parameter throughout the overlap check, `uvFor()`, and the texSpan
criterion — **not** mutating `ortho` in place, since it's a `const`,
externally-owned object also read concurrently by
`DEMTessMesh`'s background-build thread (§6l); reprojecting into a
private copy avoids that entirely rather than needing new synchronization.

**PROJ is a genuinely optional dependency**, matching this codebase's
existing OpenMP/FreeType convention (Makefile, `pkg-config --exists
proj`) — without it, a CRS mismatch is still detected and reported (both
files' EPSG codes printed, `gdalwarp` suggested with the DEM's *actual*
EPSG code — the old hardcoded `EPSG:2154` suggestion in `dem_mesh.cpp`
was also fixed here, a separate small bug this touched in passing), just
not automatically corrected. The app builds and runs fully without PROJ.

**Verification**: real compiler checks (not brace-counting) were run for
this feature specifically, given the crash history above — `glm`,
`libtiff-dev`, and `libproj-dev` were installed from apt in the sandbox
used to develop this, plus a minimal hand-written GLFW stub (the real
`libglfw3-dev` package failed to install here on an unrelated broken mesa
dependency) sufficient to satisfy `gl_platform.h`'s include. `g++
-fsyntax-only` against the real PROJ/glm/tiff headers, in all four
combinations of `{with, without} LASVIEWER_HAS_PROJ` × `{dem_mesh.cpp,
dem_tess_mesh.cpp}` (plus `geotiff.cpp`, `main.cpp`), came back clean —
meaning the actual PROJ API calls (`proj_create_crs_to_crs`, `proj_trans`,
etc.) are confirmed syntactically correct against the real header, not
guessed. `test_crs_reproject_affine_refit` in `tests/test_basic.cpp`
covers the one piece of this that's testable without real PROJ/CRS data —
the axis-aligned affine refit formula (a pure-translation case, and an
asymmetric-distortion case confirming the two edges get averaged rather
than one being picked arbitrarily).

**Not verified**: the actual PROJ transform results against real-world
CRS pairs and real DEM/orthophoto files — `proj_create_crs_to_crs`'s
behavior for specific EPSG pairs, and whether the resulting affine fit is
visually accurate on real hardware, are both unconfirmed. Also not
implemented: resolving custom (non-EPSG-catalogued) CRS definitions via
`GeoDoubleParamsTag`/`GeoAsciiParamsTag` — flagged as a known, narrower
gap in `readEPSGCode()`'s own comment, not silently unhandled.

### 6v. Level of detail and displacement, reviewed and fixed (implemented 2026-10 — supersedes the quadtree test of §6t, the constrained-edge rule of §5.1, and the blend weight kept from §6g)

A review of what the code did against what this document intended found
five problems. Each was measured or reproduced before fixing.

1. **The adaptive quadtree did not adapt.** Every leaf ended at the maximum
   level (`L5=65140` on a 2000×2000 MNT, with or without orthophoto). §6t's
   "safe bound" `max(trueMax − planeMin, planeMax − trueMin)` is the cell's
   elevation *range*, not its deviation from the patch: on a tilted plane
   it equals the whole rise across the cell. Against a 1° tolerance
   (3.4 cm for a 3.9 m cell) any slope above about 1 % failed. This is the
   answer to §6o: the collapse angle had no effect.
   **Fix:** the exact deviation `max |DEM − bilinear patch|` over every
   pixel of the cell (`cellDeviation`, one pass over the cell's pixels;
   with top-down early termination, at most one pass over the raster per
   level). The normal-variation criterion (it fed the along-normal
   displacement removed in §6s) and the texture-span criterion (UVs are
   interpolated exactly, so it bought nothing) are gone. A new criterion
   caps a cell at 64 pixels across, so the GPU's 64 segments per edge can
   still reach every DEM pixel. Result: 1° → 230k leaves, 5° → 166k,
   20° → 24k at max level 6 on the same MNT; a plane of any slope stays at
   level 0.
2. **Balancing and edge classification were quadratic.** Both found a
   neighbour by scanning every leaf: 6.7 s of a 7.4 s build. Balancing also
   only probed edge midpoints, which cannot guarantee one-level balance
   along a whole edge. **Fix:** `LeafIndex`, a hash of (level, ix, iy), so a
   lookup is O(maxLevel); balance checks every finest-level cell along each
   edge. The build now takes about 0.1 s.
3. **Generated vertices did not sit on the DEM.** The TES used
   `mix(linear, heightmap, 16u(1−u)v(1−v))`: exact only at the patch
   centre, 44 % linear a quarter of the way in, fully linear on edges. The
   weight came from §6g (crack-freedom for along-normal displacement and
   downsampled corners). **Fix:** every vertex takes its height from the
   heightmap. Same-level edges stay watertight because both patches
   generate the same vertices and sample the same texels.
4. **Level transitions would crack.** §5.1 gave both sides of a transition
   the same level K: the coarse edge got K segments, the two fine half edges
   K each (2K in total). The fine side's middle corner is a DEM sample, the
   coarse side's edge a straight line there: a T-junction crack as large as
   the deviation that caused the split. Invisible only because finding 1
   kept every leaf at the same level. **Fix:** edges are coded free / finer
   side / coarser side; the coarse side splits into 2K, each fine half
   edge into K, so every vertex exists on both sides, at the same place
   and sampled from the same texels.
5. **Heightmap sampling was off by up to half a pixel.** UVs were
   `col / (w − 1)`; texel centres are at `(col + 0.5) / w` (0 error at the
   DEM's centre, ±0.5 px at its edges). The orthophoto UVs had the same
   offset. **Fix:** pixel centres map to texel centres for both.

Two more things surfaced while fixing these:

6. **Nodata pulled vertices down to 0 m.** The heightmap stored nodata as
   0 m. With full displacement, vertices of patches at the data's edge
   hung "curtains" down to sea level (the old blend had mostly hidden it).
   The elevation-ramp range was polluted the same way (corners at 0 m), so
   the colours were compressed. **Fix:** nodata texels are filled with the
   nearest valid height (`fillNodataNearest`, breadth-first) and the
   heightmap became RG32F with a validity channel; the fragment shader
   discards where validity < 0.5, cutting the surface exactly at the data's
   edge.
7. **The DEM was unlit.** **Added:** hill-shading in the fragment shader from
   the heightmap gradient (light NW, 45°; flat ground unchanged), per layer,
   on by default.

Also: the TCS caps each edge's segments at the DEM pixels it spans
(beyond one vertex per pixel there is nothing new to sample), and
`--dem-lod angle,level` sets the collapse angle and maximum level from the
command line.

**Verification.**
- `dem_test` (10 cases, on the real code):
  - a plane has zero deviation and stays coarse at any slope;
  - a single raised pixel is found exactly;
  - a spike subdivides only locally;
  - the patch count falls monotonically with the angle;
  - balance holds over the whole finest grid, and the leaves tile it exactly;
  - edge codes agree across every edge (0/0, 2/1);
  - the 2K:K vertex sets coincide;
  - `LeafIndex` lookups;
  - nodata fill;
  - a 2000×2000 build in under 3 s, with UVs on texel centres.
- **Cracks on the GPU:** rendered from *below* the terrain looking up, any
  background pixel enclosed by surface is a hole through the mesh. On the
  gap-filled MNT at a 20° collapse angle (large coarse patches, many
  transitions), 6 views had 0 holes. As a control, the old 1:1 transition
  rule gave holes in 5 of the 6 views (70 pixels), so the test does see
  cracks.

The tests that mirrored removed mechanisms (normals, blend weight, density
mip, normal threshold, min/max pyramid) were deleted from `basic_test`.

## 7. Fallback path (no GL 4.0/4.1)

The `Makefile`/`main.cpp` currently request GL 3.3 core explicitly
(`GLFW_CONTEXT_VERSION_MAJOR/MINOR`, `specs.md` §1.3). Plan:

- Bump the requested context to 4.1 core **only for the DEM mesh path**'s
  shader variant selection, not globally — the point-cloud/COPC paths have
  no need for GL 4.x and should keep working identically on GL 3.3 hardware.
- At startup, query the actual context version GLFW/GL reports. If < 4.0,
  fall back to today's CPU-only adaptive-quadtree mesh (`loadFromDEM` as it
  exists now) with a one-line stderr notice
  (`[dem-mesh] GL 4.0+ not available, using CPU-tessellated mesh`).
- This means `main.cpp` needs to request the *highest* context it can get
  rather than a fixed 3.3, then branch — GLFW supports requesting a version
  and reporting what was actually granted; needs testing on real hardware to
  confirm the exact negotiation behavior on MacPorts GLFW.

## 8. Files touched (implementation checklist)

- [x] `src/dem_tess_mesh.h/.cpp` — **new, isolated files** (not a
  modification of `dem_mesh.h/.cpp`, which was left untouched apart from
  exposing `readDEMElevations()` for reuse — see §4's "Deviation" note for
  why the coarse grid is uniform rather than adaptive). Implements
  `demTessSupported()`, `DEMTessMesh::loadFromDEM()` (CPU-only),
  `uploadGPU()` (GL calls, separate step — `loadFromDEM` is called before
  the GL context exists in `main.cpp`, same as `DEMMesh`), `render()`,
  `destroy()`.
- [x] `src/shaders.h/.cpp` — added `kMeshTessVert`, `kMeshTessControl`,
  `kMeshTessEval` source strings and `linkTessProgram()` (compiles/links
  vert+TCS+TES+frag, reusing `kMeshFrag` unmodified). `linkProgram()`
  itself was left unchanged.
- [x] `main.cpp` — context negotiation now tries GL 4.1 core first, falls
  back to 3.3 core (§7); after shader linking, builds+uploads a
  `DEMTessMesh` when `useDEMMesh && demTessSupported()`, with `useDEMTess`
  gating the render-loop branch and a fallback to the existing `DEMMesh`
  path if tessellated setup fails for any reason. Known inefficiency,
  accepted deliberately: the DEM/orthophoto files get read twice (once for
  `DEMMesh`, once for `DEMTessMesh`) rather than restructuring the
  load-before-window-creation flow — revisit once verified on hardware.
- [x] `Makefile` — no changes needed; `SOURCES` already wildcards
  `src/*.cpp`, so `dem_tess_mesh.cpp` is picked up automatically. No new
  external dependency (tessellation shaders are core GL 4.0+).
- [x] `tests/test_basic.cpp` — added `test_dem_tess_edge_level` (mirrors
  the free/unconstrained edge formula, including a determinism/order-
  independence check for same-level-neighbor crack avoidance) and
  `test_dem_tess_normal` (mirrors `computeNormalGL()`: flat→straight-up,
  unit-length, deterministic). Both use a small local `Vec3` rather than
  `glm`, keeping the test target dependency-free per §12.3 of `specs.md`.
  11/11 tests pass (`make test CXX_OVERRIDE=g++`). NOT yet covered: the
  constrained-edge fixed-value path, or the edge-classification logic
  itself (`edgeConstraintFor()`) — worth adding once the third revision is
  verified on hardware.
- [ ] `specs.md` — **not yet updated**. Should get new §9.10+ subsections
  once this is verified on real hardware, describing the adaptive-quadtree-
  plus-GPU-tessellation system and the split inner/outer crack-avoidance
  strategy (§5.1) precisely, and §1.3 should describe the two-tier GL
  3.3/4.1 requirement. Deliberately held off until real-hardware
  verification confirms the design as built actually works — no point
  documenting untested behavior as fact in the numbered spec.
- [ ] `README.md` — **not yet updated**, same reasoning as `specs.md` above.

## 9. Open risks / things to verify on real hardware (not verifiable in this sandbox — no GPU here)

- Actual GLFW context-version negotiation behavior on MacPorts GLFW when
  requesting 4.1 vs what's granted on older Intel Macs.
- **Real seam behavior specifically at constrained (LOD-transition) edges**
  — §5.1's fixed `CONSTRAINED_EDGE_TESS_LEVEL = 4.0` guarantees matching
  segment *counts* by construction, but §5.2/§5.3 document a real, expected-
  small-but-nonzero position residual from tilted-normal displacement at
  non-corner boundary points on those edges. This is the single most
  important thing to visually check first, specifically around a sharp
  feature like a building (the case that motivated this whole revision) —
  if the residual is visible rather than sub-pixel, the documented fallback
  is restricting displacement direction closer to vertical near constrained
  edges (blending toward world-up), at the cost of some slope-correctness
  right at LOD transitions.
- `CONSTRAINED_EDGE_TESS_LEVEL = 4.0` is a guess, not a measurement — too
  low and LOD transitions will look faceted/blocky; too high and they waste
  GPU work on every transition edge regardless of camera distance (since,
  unlike the free formula, this value is NOT view-dependent). May need to
  become distance-aware itself eventually (e.g. a small fixed set of
  discrete levels selected by rough camera-distance bucket, still shared
  identically by both sides of a constrained edge) if a single constant
  proves too coarse a compromise.
- Real seam behavior under camera motion at UNCONSTRAINED (same-level)
  edges — the deterministic-edge-factor technique is standard and the
  math checks out, but should still be visually confirmed against this
  codebase's specific edge/patch geometry.
- Fragment-shader/TES texture bandwidth cost of the heightmap sample at high
  tessellation factors — may need to drop `targetSegments`' pixel target
  (currently 8px/segment, §5.1) if this is a bottleneck in practice.
- Whether patch corner normals (computed from the *full-res* heightmap, per
  §4) ever visibly disagree with the *coarse* triangle's own face normal
  enough to look wrong before displacement kicks in — untested assumption.
