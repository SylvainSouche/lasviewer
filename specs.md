# lasviewer — Specification

This document tracks every requirement gathered during development. Each numbered item is a spec point that the implementation must satisfy. Items are grouped by category.

---

## 1. Platform & Dependencies

### 1.1
The application is a C/C++ desktop app targeting **macOS** (primary) and **Linux** (secondary).

### 1.2
Windowing toolkit: **GLFW** (minimal, lightweight, no native widgets required).

### 1.3
3D API: **Modern OpenGL 3.3 core profile** (shaders, VBO/VAO, no fixed-function).

### 1.4
LAS/LAZ reader: **PDAL** (`readers.copc`, `readers.las`, etc.).

### 1.5
TIFF/GeoTIFF reader: **libtiff** (with manual GeoTIFF tag reading via `TIFFGetField` — no libgeotiff dependency).

### 1.6
Math library: **glm** (header-only).

### 1.6b
On-screen text (help overlay, console log overlay): **FreeType** (`libfreetype`). A glyph atlas covering ASCII 32–126 is rasterized once at startup and drawn as textured quads via a dedicated text shader. (Earlier iterations used a hand-rolled 5×7 `GL_POINTS` bitmap font; this was replaced with FreeType so the full printable ASCII set renders correctly instead of a hardcoded per-character pixel grid.)

### 1.7
Build system: **plain Makefile** with multi-OS detection (MacPorts, Homebrew, Linux system packages).

### 1.8
Package manager: **MacPorts** (`/opt/local`) on macOS. Homebrew and Linux system packages also supported.

### 1.9
macOS OpenGL: must include `<OpenGL/gl3.h>` and define `GL_SILENCE_DEPRECATION` before `<GLFW/glfw3.h>` to get core-profile function declarations.

### 1.10
OpenMP: enabled by default (`-fopenmp`) for parallel point cloud loading and colorization. Can be disabled with `make OPENMP=0`.

---

## 2. Input Formats

### 2.1
Load **LAZ/LAS point clouds** via PDAL (supports `.las`, `.laz`, `.copc.laz`).

### 2.2
Load **GeoTIFF DEMs / DSMs** (elevation rasters). DEM input is rendered as an **adaptive triangulated mesh**, not a point cloud — see §9.7–9.9. Supported sample formats: Float32/64 and Int16/32/UInt16/Byte via scanline or tile reads (never `TIFFReadRGBAImageOriented`, which corrupts numeric elevation values by routing them through an RGBA decode). Also supports **Terrain RGB** encoding (8-bit, 3+ band, unsigned): `elevation = (R*65536 + G*256 + B) * 0.1 - 10000`, matching IGN's MNS LiDAR HD raster convention; values below -9000 are treated as nodata and clamped to 0.

The LiDAR point-cloud path (§2.1) with orthophoto coloring remains the primary/default workflow; the DEM/DSM mesh path is a parallel, actively-developed alternative for terrain-surface visualization from raster elevation data rather than point clouds.

### 2.3
Load **GeoTIFF orthophotos** (RGB/RGBA) for point cloud colorization. Supports JPEG-compressed and LZW-compressed TIFFs.

### 2.4
Read **GeoTIFF tags** (`ModelPixelScaleTag` 33550, `ModelTiepointTag` 33922) directly via `TIFFGetField`. No `.tfw` sidecar required, but `.tfw` is a fallback if GeoTIFF tags are absent.

### 2.5
GeoTIFF Area raster convention: tiepoint is the top-left CORNER of the pixel. Offset by half a pixel to get pixel-center origin (matching `.tfw` convention).

### 2.6
GeoTIFF `ModelPixelScaleTag` stores absolute values. For north-up orthophotos, the effective Y scale is **negative** (rows go down, world Y goes up). Must negate `scaleY` when synthesizing the affine.

### 2.7
Command-line interface:
- `./lasviewer cloud.laz [ortho.tif]` — bare positional (first = input, second = ortho)
- `./lasviewer -cop cloud.laz [-o ortho.tif]` — explicit COPC flag
- `./lasviewer -d dem.tif [-o ortho.tif]` — explicit DEM flag
- `./lasviewer -h` or `--help` — print usage and exit
- Bare `.tif` as first positional → treated as DEM; bare `.laz`/`.las` → treated as point cloud

---

## 3. Coordinate System & Georeferencing

### 3.1
Work in the data's native CRS (e.g., Lambert-93 / EPSG:2154). No reprojection.

### 3.2
World-to-GL transform:
```
GL_X = (worldX - center.x) / scale       ← easting
GL_Y = (worldZ - center.z) / scale       ← elevation (up)
GL_Z = -(worldY - center.y) / scale      ← northing (NEGATED for north-up)
```

### 3.3
Z scale defaults to **1.0× (true metric scale)**. The data CRS uses meters for both XY and Z, so 1 unit XY = 1 unit Z. No automatic Z scaling based on terrain type.

### 3.4
Z exaggeration adjustable at runtime: `E` (increase ×1.2), `D` (decrease ÷1.2), `U` (reset to 1.0×). Applied in the vertex shader via `uZScale` uniform.

### 3.5
Cloud recentered to GL origin and rescaled by bbox diagonal so it fits in roughly [-1, 1]³.

### 3.6
Orthophoto affine transformed to GL space (including Z negation) for colorization purposes only. The orthophoto quad is NOT rendered as a flat plane (see spec 9.6).

### 3.7
Orthophoto colorization: each point's world (X, Y) is inverse-transformed through the GeoTIFF affine to find the corresponding pixel, then bilinearly sampled. Points outside the orthophoto extent are colored mid-dark gray (0.35, 0.35, 0.35).

### 3.8
Orthophoto colorization must account for the Z negation: `worldY = -gz × scale + centerY`. The negation is baked into the inverse-affine coefficients (`kColZ`, `kRowZ`).

### 3.9
**Coverage gate**: before colorizing, the loader computes the overlap between the orthophoto's extent and the point cloud's XY bbox. If the orthophoto covers less than **25%** of the cloud's area, loading fails with an error (this catches wrong/mismatched ortho files early instead of producing a mostly-gray cloud). Coverage percentage is logged regardless of outcome.

---

## 4. Point Cloud Subsampling

### 4.1
Clouds larger than **2M points** are subsampled at load time using a **spatial grid**: one point per cell, where `cellSize = sqrt(area / 2M)`. This guarantees uniform spatial coverage regardless of input point order (LiDAR points are stored in scan order, so naive "every Nth point" drops entire scan lines).

### 4.2
Grid dimensions capped at 2048×2048 to avoid excessive memory. `cellSize` recomputed to match the capped grid.

### 4.3
At render time, the **vertex shader** stochastically discards points based on their depth from the camera:
- Closer points → kept at higher density
- Farther points → subsampled more aggressively
- Per-point hash (`hash13`) for spatial coherence (homogeneous coverage, no clustering)
- Formula: `worldSpacing = targetPixelSpacing × 2 × depth × tan(fov/2) / viewportH`; `N = density × worldSpacing²`; `keepThreshold = clamp(1/N, 0, 1)`; keep if `hash < keepThreshold`

### 4.4
Target pixel spacing default: **2.83px** (gives ~2× density vs 4px). Adjustable at runtime: `S` (sparser ÷1.5), `F` (fuller ×1.5).

### 4.5
Point size adjustable at runtime: `N` (smaller ÷1.3), `M` (bigger ×1.3). Applied via `uPointSize` uniform.

### 4.6
`uDisableSubsampling` uniform: when set to 1.0, the shader skips the discard logic entirely (used for help overlay text rendering).

---

## 5. Streaming COPC

### 5.1
For `.copc.laz` files, use **async tile loading** instead of loading the entire file. A background `TileLoader` thread processes a request queue.

### 5.2
**8×8 grid** (64 tiles) dividing the cloud's XY extent. Each tile is a PDAL spatial query with `bounds` + `resolution` options.

### 5.3
**Coarse first render**: on startup, request ALL tiles at a coarse resolution targeting **~500 points/tile** (`coarseRes = sqrt(tileArea / 500)`, clamped to [1m, 50m]) for fast initial display. No waiting for full-resolution data.

### 5.4
**Distance-based, apparent-size-driven LOD** (not culling): per-tile resolution is computed from the tile's *projected on-screen area*, not just camera distance:
1. On upload, each tile's points are PCA'd (Jacobi eigendecomposition of the position covariance) to get an oriented bounding box (OBB): 3 principal axes + extents, and a point-spacing estimate (`cbrt(obbVolume / pointCount)`).
2. Each frame, `desiredResolution(tile, camPos, fov, viewportH)` projects all 3 OBB faces toward the camera (`Σ face.dim1 × face.dim2 × |dot(face.normal, sightDir)|`) to estimate the tile's projected area in pixels.
3. If projected area < 25px², the tile is skipped (`resolution = 0`).
4. Otherwise, target point count = `projectedAreaPx / 25`, clamped to [100, 500000]; resolution = `sqrt(tileAreaMeters / targetPoints)`, clamped to [0.1m, 50m] (>50m → skip).
5. If a tile's loaded resolution is coarser than desired by more than 1.5×, it's re-requested at the finer resolution.

This replaced an earlier, simpler `tileWorldMeters / (tilePixelSize / 3.0)` radius-based formula — the OBB/projected-area approach better matches actual on-screen footprint for elongated or tilted tiles.

### 5.5
**LRU eviction**: tiles not used in the last 10 seconds are unloaded (GPU buffers deleted, state reset to UNLOADED).

### 5.6
**Occlusion culling — implemented via Hi-Z (max-mip depth pyramid), replacing the disabled query-based scaffolding.** The original approach (GL occlusion query objects, `glGenQueries`, allocated on init but never issued via `glBeginQuery`/`glEndQuery`) is still allocated — kept as documented scaffolding, not removed — but is not what's actually used: it was tried and caused visible tile flickering, due to occlusion-query result latency (the GPU answer for "was this tile visible" often isn't available until a later frame, so a decision ends up made from a stale answer to a question about a different camera position).

The working mechanism instead: `TileGrid` builds a conservative MAX-reduced depth mip pyramid from each frame's own rendered depth (`captureAndBuildHiZ()` — blit the real depth buffer into a texture, then a fragment-shader copy pass into mip 0 of a `GL_R32F` pyramid, then successive 2×2 max-reduction passes per mip), and uses it on the *next* frame (`isTileOccludedByHiZ()`) to skip tiles whose entire screen footprint is already known to be behind closer geometry. This is deliberately one-frame-stale too, but a fixed, deterministic offset rather than an unbounded, driver-dependent query latency — the only failure mode is a tile that should be visible this frame not being drawn for one frame during fast camera motion, which self-corrects immediately next frame; a tile that's actually occluded being wrongly drawn is not a failure mode at all (just wasted work, same as no culling). Since this app's DEM and point-cloud-streaming render paths are mutually exclusive (only one is ever active — see `main.cpp`), the only thing tiles can occlude here is *other tiles*, not DEM geometry.

Deliberately reads back only a small, fixed-size (~64×64) coarse mip level to the CPU once per frame, rather than a proper GPU-side per-tile test — `TileGrid::render()`'s draw/skip decision already happens on the CPU (a simple loop issuing `glDrawArrays` calls), and a GPU→CPU readback for every one of `gridX*gridY` tiles individually would stall far more than it saves. `InputState::useOcclusion` (default `true`) is the actual, working kill-switch now — no key currently toggles it (`O` remains repurposed to the DEM mesh's point-collapsing angle, §10.1), so it's effectively always on unless changed at the code level.

This was implemented at one point (proxy geometry drawn between begin/end query, tested against the depth buffer) but caused visible tile flickering — occluded tiles toggled visibility frame-to-frame as the 1-frame-latency query result interacted with the streaming refinement logic, and it was removed rather than debugged further at the time. The scaffolding (query objects, proxy VAO, `O` key) was left in place for a future retry rather than torn out. The toggle currently has no visible effect either way.

### 5.7
Max **16** concurrent tile loads (`maxConcurrentLoads`). Pending loads tracked; new requests only issued when `pendingLoads < maxConcurrentLoads`.

### 5.8
Non-COPC files (regular `.las`/`.laz`, DEMs) use the existing single-VBO path — streaming only activates for `.copc.laz` files (detected by `.copc.` in filename).

---

## 6. TIFF Handling

### 6.1
TIFF dimensions capped at 100,000 × 100,000 and 2 GB raw raster. Larger files are refused.

### 6.2
Orthophoto pixel buffer capped at ~64M pixels (256 MB RGBA) to keep memory and colorization time reasonable. Larger images are downsampled.

### 6.3
**Box-filter downsampling** with correct fractional pixel mapping:
```
sx0 = floor(x * srcW / dstW)
sx1 = floor((x+1) * srcW / dstW)
```
Must cover the full source image [0, srcW) with no gaps. Integer division (`bx = srcW / dstW`) is FORBIDDEN — it truncates and discards up to 26% of the image.

### 6.4
After downsampling, the GeoTIFF affine must be **rescaled** to match the new pixel size: `A *= srcW/dstW`, `E *= srcH/dstH`. C and F (origin) don't change.

### 6.5
DEM loading uses `TIFFReadScanline` (scanline-based, not `TIFFReadRGBAImage`). Supports multiple sample formats (IEEEFP, INT, UINT) and bit depths (8, 16, 32, 64).

### 6.6
Nodata values (typically -9999 or -32768) are clamped to 0.

---

## 7. Camera & Navigation

### 7.1
**Orbit camera**: left mouse drag (without Shift) rotates yaw/pitch around a target point. Pitch clamped to ±89°. Shift+left drag instead turns the viewer's head — see §7.1b.

### 7.1b
**Head turn** (Shift+left drag): rotates the look direction (yaw/pitch) while keeping the eye position fixed, rather than orbiting around the target. Implemented by recomputing `target` after the yaw/pitch change so `position()` (target + a fixed yaw/pitch/distance offset) stays put: `target_new = eye_old - offset(newYaw, newPitch)`.

### 7.2
**Pan**: right mouse drag or middle mouse drag. Translates the camera target along screen-space right/up vectors. Pan speed scales with camera distance. (Shift+left drag no longer pans — see §7.1b.)

### 7.3
**Move forward/backward** (mouse wheel; also Shift+Up/Down arrow keys, §10.2): translates the whole camera rig (both eye and target together) along the view direction — `target += fwd * (distance × 0.1 × dy)`. Deliberately NOT a distance-clamped zoom: an earlier version scaled `camera->distance` exponentially and clamped it to [0.001, 1000], which capped how far the camera could ever get from (or how close it could get to) the target — since this moves eye and target together, `distance` (the orbit radius, still used for pan/arrow step sizing and near/far computation) stays constant and there's no artificial travel limit.

### 7.3b
**Double-click-to-focus**: double-clicking (< 0.4s between presses, < 5px screen-space movement between them — tracked in `mouseButtonCallback`, `gl_app.cpp`) picks the point under the cursor and recenters on it **without moving the eye**: `target` becomes the picked point, and `yaw`/`pitch`/`distance` are solved so that `position()` (= target + offset(yaw,pitch,distance)) lands exactly back on the eye's position from *before* the pick — `distance = |eye - picked|`, `pitch = asin(dir.y)`, `yaw = atan2(dir.x, dir.z)` where `dir = normalize(eye - picked)`. This is the same math as the Shift+drag head-turn (§7.1b), applied once instead of continuously per mouse-move.

An earlier version instead kept `yaw`/`pitch`/`distance` unchanged and only reassigned `target` — which looks similar but isn't: since the picked point is generally at a different depth than the old target, preserving the *old* distance from a *new* point means `position()` recomputes to somewhere else — a disguised dolly/zoom along the view axis, not a pure recenter. Corrected per explicit user feedback: double-click "should [not] change zoom, [not] move along the view axis, just recenter and change rotation center." Pan/arrow-key step size (`distance × 0.05`, §7.2/§10.2) still adapts automatically afterward, since `distance` is updated to the *true* eye-to-target separation either way.

Picking uses depth-buffer readback + `Camera::unproject()` (matrix inverse of `proj() × view()`), not CPU-side ray-geometry intersection — deliberately, since it works uniformly across every data source this app renders (LAZ/COPC points, the CPU-triangulated DEM mesh, and the GPU-tessellated+displaced DEM mesh) without needing per-type intersection code, and critically the GPU-tessellated path's CPU side only has the coarse, pre-displacement patch corners — a CPU ray test there would hit the wrong (undisplaced) surface entirely.

Implementation detail worth noting: the actual `glReadPixels` call happens in `main.cpp`'s render loop, right after that frame's own rendering and before `glfwSwapBuffers` — not inside `mouseButtonCallback` (which only detects the double-click and records screen coordinates in `InputState::pickRequested/pickX/pickY`). Reading depth from inside the input callback would need `GL_FRONT` (since a swap may have already invalidated `GL_BACK`'s prior contents), which has real cross-platform reliability issues (compositors, some drivers, macOS quirks); reading the default `GL_BACK` immediately after this frame's draw calls is unambiguous. One-frame-old click coordinates at worst, imperceptible in practice. A depth of 1.0 (nothing rendered at that pixel — background) is treated as a miss and doesn't change `target`. A picked point coinciding with the eye itself (near-zero distance) is also treated as a no-op, to avoid a degenerate direction solve.

Verified self-consistent with `test_recenter_keeps_eye_fixed` in `tests/test_basic.cpp` (recomputing `position()` from the solved yaw/pitch/distance reproduces the original eye to within float tolerance) — the math, not the on-screen result, which remains unverified on real hardware.

### 7.4
**Reset** (`R`): yaw=0.6, pitch=1.2 (~69° from horizontal), distance=2.0, target=origin.

### 7.5
**Side view** (`V`): pitch=0.05 (nearly horizontal).

### 7.6
**Top view** (`T`): pitch=1.54 (looking straight down).

### 7.7
**Perspective/orthographic toggle** (`P`): perspective uses `glm::perspective(fov, aspect, near, far)`. Orthographic uses `glm::ortho(-w, w, -h, h, near, far)` where `h = orthoSize × (distance / 2)`.

### 7.8
In orthographic mode, point size is fixed (no depth scaling): `gl_PointSize = uPointSize × viewportH / (2 × uOrthoHeight)`.

### 7.9
Default camera: pitch=1.2 (near-top-down, ~69°), distance=2.0. Near-top-down minimizes perceived Z exaggeration from perspective foreshortening.

---

## 8. Near/Far Plane Computation

### 8.1
Near/far planes computed **dynamically** from the cloud's bounding box, not static values.

### 8.2
Transform the 8 bbox corners to view space, find min/max positive Z (distance in front of camera).

### 8.3
**Camera outside bbox**: `nearP = max(0.001, minDist × 0.9)`, `farP = maxDist × 1.5 + 0.01`.

### 8.4
**Camera inside bbox** (fewer than 8 corners in front): use small nearP = `max(0.001, camDist × 0.005)` where camDist is distance to target.

### 8.5
Only recompute when camera state (yaw, pitch, distance, target) or zScale changes. Cache last values; skip recompute if unchanged. `nearFarDirty` flag set by key handlers that change camera or Z scale.

---

## 9. Rendering

### 9.1
Point cloud drawn as `GL_POINTS` with perspective-correct sizing: `gl_PointSize = uPointSize × (viewportH / depth)`. Points are circular (fragment shader discards `gl_PointCoord` outside radius 0.5).

### 9.2
Color modes: **elevation gradient** (blue → cyan → green → yellow → red, 5-stop ramp) and **orthophoto-sampled colors**. Toggle with `C` key. Default is orthophoto when available.

### 9.3
Color toggle is instant: re-uploads the color VBO with `glBufferData`, no grid rebuild.

### 9.4
Background: dark gray (0.10, 0.11, 0.13).

### 9.5
Depth test enabled. MSAA (4×) enabled via `GLFW_SAMPLES`.

### 9.6
Orthophoto quad NOT rendered as a flat plane in the point-cloud path — only used for point colorization there. (The flat-plane rendering was removed per user request; only the colorization remains for LiDAR point clouds. The DEM/DSM path, §9.7–9.9, does render a textured surface — that is a different code path, not a reintroduction of the removed flat quad.)

### 9.7
**DEM/DSM adaptive mesh** (the DEM alternative to the point-cloud+orthophoto workflow, §2.2): elevation rasters are triangulated into a `GL_TRIANGLES` mesh instead of loaded as points.
1. **Adaptive quadtree subdivision**: starting from an 8×8 coarse grid over the raster, each cell recursively subdivides (up to level 6) if either (a) its geometric error vs. bilinear interpolation of its 4 corners exceeds 1% of the elevation range, or (b) its texture footprint spans more than 64 orthophoto pixels.
2. **Balance pass**: any leaf whose neighbor is 2+ levels finer is subdivided; repeated until stable (capped at 40 iterations) so no two adjacent leaves differ by more than one level.
3. **T-junction-free welding**: each leaf gets a midpoint vertex on any edge whose neighbor is same-level or finer, so same-level neighbors share the midpoint and finer neighbors weld their corner to it.
4. **Fan triangulation**: each leaf is triangulated as a fan from its center vertex to its boundary (4 corners + 0–4 edge midpoints).

### 9.8
DEM mesh texturing: both the DEM's and the orthophoto's actual CRS (EPSG code) are read from each file's GeoKeyDirectoryTag (`readEPSGCode()`, geotiff.cpp) — not just their affine transforms, which alone can't reveal a CRS difference. If they differ and PROJ is available (optional dependency, see specs.md §12.1/Makefile), the orthophoto's 4 corners are reprojected via PROJ into the DEM's CRS and a new affine is re-fit (`reprojectToMatchCRS()`) before the normal world→UV transform is used. If PROJ isn't available, or the reprojection pipeline itself fails, or the two rasters still don't overlap even after a successful reprojection (same CRS now, but genuinely different ground coverage), texturing is skipped entirely rather than stretched — stretching would show imagery from a completely unrelated location, worse than no texture (console prints a warning with both extents, both EPSG codes if known, and a `gdalwarp` command using the DEM's *actual* EPSG code). If the orthophoto has no geo tags at all, it's stretched over the DEM extent instead — a deliberate convenience for pairing an untagged image with a DEM, since there's no correspondence metadata to contradict there. If no orthophoto is supplied, or the one supplied isn't usable per the above, the mesh shader falls back to an elevation color ramp: **dark blue-violet → teal → yellow** (3-stop, viridis-inspired) — distinct from the point cloud's 5-stop blue→cyan→green→yellow→red ramp (§9.2).

### 9.9
DEM elevation is read via `TIFFReadScanline`/`TIFFReadTile` (never `TIFFReadRGBAImageOriented`, which would corrupt numeric elevation through an RGBA/YCbCr decode step). Supports UInt8/16/32, Int16/32, Float32/64, and Terrain RGB encoding (§2.2).

---

## 10. Controls (Layout-Independent Letter Shortcuts)

### 10.1
All keyboard shortcuts use **letters only** (no symbols), and are handled
via GLFW's character callback (`glfwSetCharCallback`), not the key
callback. This distinction matters and was previously wrong: GLFW's key
callback identifies a PHYSICAL key position fixed to a US QWERTY layout —
`GLFW_KEY_W`/`GLFW_KEY_A` fire based on where those letters sit on a US
keyboard, regardless of the user's actual OS layout. On AZERTY (French)
keyboards specifically, A/Q and W/Z are physically swapped and M relocates
— so key-callback-based shortcuts for `W` and `A` actually fired when an
AZERTY user pressed the keys labeled Z and Q respectively, not W and A.
"Letters only" alone (the original claim here) only avoids the
symbol/shift-state class of layout problems, not this one. The character
callback reports the actual Unicode codepoint the OS's active layout
produces for the physical key pressed, which is correct for AZERTY,
QWERTZ, Dvorak, or any other layout. See `src/gl_app.h`'s banner comment
for the full explanation.

| Key | Action |
|-----|--------|
| `R` | Reset camera |
| `E` | Increase Z exaggeration (×1.2) |
| `D` | Decrease Z exaggeration (÷1.2) |
| `U` | Reset Z to 1.0× (true scale) |
| `S` | Decrease point density (Sparser, ÷1.5) |
| `F` | Increase point density (Fuller, ×1.5) |
| `N` | Decrease point size (÷1.3) |
| `M` | Increase point size (×1.3) |
| `C` | Toggle colors (orthophoto / elevation) |
| `P` | Toggle perspective / orthographic |
| `I` | Decrease point-collapsing angle (finer, more subdivision — DEM mesh only, triggers a reload) |
| `O` | Increase point-collapsing angle (coarser, more merging — DEM mesh only, triggers a reload). Repurposed from the old occlusion-culling toggle (§5.6) — occlusion culling is now implemented (Hi-Z, §5.6) but has no key-based toggle; it's controlled only by `InputState::useOcclusion`'s default (`true`) |
| `G` | Toggle the red master (coarse patch) edge overlay — default off (diagnostic-only, not a rendering feature) |
| `V` | Side view |
| `T` | Top view |
| `B` | Toggle tile bounding-box wireframe overlay (streaming mode) |
| `L` | Toggle on-screen console log overlay |
| `H` | Show on-screen help (10 seconds) |
| `ESC` | Quit |

### 10.2
Arrow keys translate the eye (work on PRESS and REPEAT for continuous
movement): Left/Right always strafe left/right; Up/Down translate
vertically, unless Shift is held, in which case Up/Down move
forward/backward along the view direction instead. Handled via the key
callback (not the char callback) since arrows have no character codepoint
and need PRESS+REPEAT semantics for continuous movement, not a discrete
character-typed event.

### 10.3
Shift key tracked continuously. Shift+left-drag turns the viewer's head
(rotates the look direction while keeping the eye position fixed —
`target` is recomputed each frame so `position()` stays put), rather than
panning. Right-drag and middle-drag still pan (translate the target in the
screen plane).

---

## 11. On-Screen Help & Console Log

### 11.1
`H` key displays an on-screen help overlay for **10 seconds**, then auto-hides.

### 11.2
Text (help overlay and console log, §11.5) is rendered via **FreeType** (`TextRenderer`): a glyph atlas covering ASCII 32–126 is rasterized once at startup into a single texture, then each `drawText()` call batches one textured-quad draw call using a dedicated text shader (screen-space, origin top-left, +Y down). This replaced an earlier hand-rolled 5×7 `GL_POINTS` bitmap font (which only covered uppercase A-Z, digits, space, and punctuation) — FreeType renders the full printable ASCII set without hardcoding per-character pixel grids.

### 11.3
Text rendering is independent of the point-cloud vertex shader path — it has its own VAO/VBO/program and is unaffected by point subsampling, `uPointSize`, or `uDisableSubsampling`.

### 11.4
`-h` / `--help` command-line flag prints all controls to stderr and exits.

### 11.5
**Console log overlay** (`L` key toggle): `std::cerr` is intercepted at startup by a `LogCapture` streambuf that splits output on `\n`, stores up to the last 200 lines in a global buffer, and still forwards every line to the real stderr (so terminal output is unchanged). When toggled on, the last ~25 lines are drawn as green, semi-transparent text in the bottom-left of the window, scrolling as new log lines arrive — giving a live in-app view of load progress, tile streaming stats, and errors without needing a terminal.

---

## 12. Build System

### 12.1
Single `Makefile` with multi-OS detection (macOS MacPorts, macOS Homebrew, Linux system packages). Auto-detects via `$(wildcard)`.

### 12.2
Targets:
- `make` / `make build` — compile main.cpp + src/*.cpp → lasviewer
- `make debug` — same, but `-g` instead of `-O2` → lasviewer-debug, in a separate `.build/obj-debug` object dir (so `make build`/`make debug` can't serve stale cross-mode object files to each other)
- `make test` — compile + run unit tests (self-contained, no external deps)
- `make lint` — static analysis (clang-tidy or cppcheck fallback)
- `make fmt` — format source with clang-format
- `make dist` — create source tarball
- `make doc` — generate Doxygen HTML documentation
- `make clean` — remove all build artifacts
- `make run ARGS="..."` — build + run with arguments
- `make help` — show all targets

### 12.2b
`make build` requires GLFW, PDAL, libtiff, glm, **and FreeType** (`libfreetype`/`freetype2`, for §11.2 text rendering). The Makefile probes for freetype via `pkg-config --cflags/--libs freetype2` and links `-lfreetype`.

### 12.2c
**PROJ** (`proj`) is a genuinely optional dependency, unlike the above — detected via `pkg-config --exists proj` (same pattern as OpenMP's compiler-capability probe), setting `LASVIEWER_HAS_PROJ` when found. Used only by `reprojectToMatchCRS()` (geotiff.cpp) to auto-correct a detected DEM/orthophoto CRS mismatch (§9.2 texturing description above). The app builds and runs fully without it — a mismatch is still detected and reported either way, just not automatically reprojected.

### 12.3
Test target does NOT require GLFW/PDAL/libtiff/FreeType — tests are self-contained math verification (see §13 — some tests replicate formulas from `src/*.cpp` locally rather than linking against them).

### 12.4
`.clang-tidy` config: bugprone-*, cert-*, misc-*, modernize-*, performance-*, readability-* checks. Magic numbers and identifier length suppressed.

### 12.5
`.clang-format` config: LLVM base, C++17, 100-column limit, 4-space indent, attached braces.

### 12.6
`docs/Doxyfile` for Doxygen documentation generation.

---

## 13. Testing

### 13.1
Unit tests in `tests/test_basic.cpp` verify core math formulas independently (no linking against main.cpp).

### 13.2
Test coverage:
1. Coordinate transform (world → GL, including Z negation)
2. GeoTIFF affine inverse (world → pixel)
3. Spatial grid subsampling cell size
4. Depth-based subsampling formula
5. Box-filter downsample pixel mapping (full image coverage)
6. Morton (Z-order) code interleaving
7. Near/far plane computation from bbox corners
8. Elevation gradient color ramp
9. Tile LOD resolution computation — mirrors the current PCA/OBB projected-area algorithm in `src/copc_streamer.cpp` (§5.4), not the earlier radius-based formula. Keeping this test in sync with `desiredResolution()` whenever that function changes is a manual step since the test replicates the formula rather than linking against it.

### 13.3
Tests use assert-based macros with RUN/PASS output. Exit code 0 on all pass, 1 on any failure.

---

## 14. Documentation

### 14.1
`README.md`: presentation, features, quick start, controls, sample data sources (IGN France, USGS 3DEP, OpenTopography, Copernicus DEM), build system, project structure, technical details, limitations.

### 14.2
`LICENSE.md`: BSD 3-Clause license.

### 14.3
`specs.md`: this document — all numbered spec points.

---

## 15. Performance

### 15.1
Point cloud loading parallelized with OpenMP: both spatial grid subsampling (atomic cell claiming + parallel compaction) and orthophoto colorization use `#pragma omp parallel for`.

### 15.2
Single `glDrawArrays` call for the entire point cloud (no per-cell draw calls in non-streaming mode). GPU handles subsampling in the vertex shader.

### 15.3
Streaming mode: per-tile draw calls, but only loaded tiles are drawn. Max 16 concurrent background loads (`maxConcurrentLoads`, §5.7).

### 15.4
Stats printed to stderr once per second: point count, tile count, camera distance.

---

## 16. Error Handling

### 16.1
TIFF open failure: print error, return false, continue without orthophoto.

### 16.2
PDAL driver inference failure: print error, return false, exit.

### 16.3
GeoTIFF tag read failure: fall back to `.tfw` sidecar lookup. If no `.tfw` either, warn and continue with unaligned orthophoto.

### 16.4
Shader compile/link failure: print error log, return 0, exit.

### 16.5
TIFF dimensions suspicious (>100k or 0): refuse to load, print error.

### 16.6
Memory allocation failure (`_TIFFmalloc` returns null): print error, close TIFF, return false.

---

## 17. File Structure

```
lasviewer/
├── Makefile              # Multi-OS build (build/test/lint/fmt/dist/doc/clean/help)
├── main.cpp              # Orchestration layer: argv parsing, GLFW/GL setup, render loop
├── src/
│   ├── camera.h/.cpp          # Orbit camera, projection, dynamic near/far
│   ├── point_cloud.h/.cpp     # LAZ/LAS load (PDAL), spatial subsampling, GPU upload
│   ├── geotiff.h/.cpp         # GeoTIFF tags, .tfw, TIFF load/downsample, ortho colorization
│   ├── copc_streamer.h/.cpp   # Async COPC tile streaming, PCA/OBB LOD, LRU eviction
│   ├── dem_mesh.h/.cpp        # Adaptive quadtree DEM/DSM mesh triangulation + texturing
│   ├── shaders.h/.cpp         # Embedded GLSL sources (point/line/mesh) + compile/link
│   ├── text_renderer.h/.cpp   # FreeType glyph atlas + textured-quad text rendering
│   ├── gl_app.h/.cpp          # GLFW callbacks, InputState, LogCapture (console overlay)
│   └── gl_platform.h          # Canonical GLFW+platform-GL include (see file header)
├── README.md             # User documentation
├── LICENSE.md            # BSD 3-Clause
├── specs.md              # This specification
├── .clang-format         # Code formatting config
├── .clang-tidy           # Static analysis config
├── tests/
│   └── test_basic.cpp    # 9 unit tests
└── docs/
    └── Doxyfile          # Doxygen configuration
```
