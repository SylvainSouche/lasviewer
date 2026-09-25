# lasviewer — Specification

This document tracks every requirement gathered during development. Each numbered item is a spec point that the implementation must satisfy. Items are grouped by category.

---

## 1. Platform & Dependencies

### 1.1
The application is a C/C++ desktop app targeting **macOS** (primary) and **Linux** (secondary).

### 1.2
Windowing toolkit: **GLFW** (minimal, lightweight, no native widgets required).

### 1.3
3D API: **modern OpenGL, core profile**. A 4.1 core context is requested first (needed for DEM tessellation), with a fallback to 3.3 core, where point clouds still work and DEM layers are skipped.

### 1.4
LAS/LAZ reader: **PDAL** (`readers.copc`, `readers.las`, etc.).

### 1.5
Raster reader: **GDAL** (orthophotos, DEMs, georeferencing, CRS, reprojection), in `Geo/libgeo.m/src/raster.cpp`. GDAL is already a PDAL dependency. It replaced hand-written GeoTIFF tag parsing on libtiff plus a direct PROJ dependency; neither libtiff nor PROJ is linked directly any more.

### 1.6
Math library: **glm** (header-only).

### 1.6b
UI and on-screen text: **Dear ImGui** (v1.91.9b, MIT; core + GLFW and OpenGL3 backends), fetched from its release tag and compiled by the build (§12.2). It replaced the FreeType glyph-atlas text renderer, so FreeType is no longer a dependency.

### 1.7
Build system: **bmake-it** (BSD make; https://github.com/SylvainSouche/bmake-it). The repository is a bmake-it workspace; see §12. A plain GNU `GNUmakefile` building the same binary from the same tree is kept temporarily for comparison.

### 1.8
Package manager: **MacPorts** (`/opt/local`) on macOS. Homebrew and Linux system packages also supported.

### 1.9
macOS OpenGL: must include `<OpenGL/gl3.h>` and define `GL_SILENCE_DEPRECATION` before `<GLFW/glfw3.h>` to get core-profile function declarations.

### 1.10
OpenMP: `OPENMP=yes` on the `libgeo.m` and `lasviewer.m` modules (point-cloud thinning and colorization, DEM quadtree build). bmake-it probes that the compiler accepts `-fopenmp` and fails with a clear error otherwise.

---

## 2. Input Formats

### 2.1
Load **LAZ/LAS point clouds** via PDAL (supports `.las`, `.laz`, `.copc.laz`).

### 2.2
Load **DEMs / DSMs** from any raster GDAL reads, rendered as a GPU-tessellated adaptive mesh (§9.7). Band 1 is read as Float32. An 8-bit raster with ≥3 bands is decoded as **Terrain RGB**: `elevation = (R*65536 + G*256 + B) * 0.1 - 10000` (IGN MNS LiDAR HD); decoded values below -9000 become nodata (-9999). The file's declared nodata value is honoured; when none is declared, values below -9000 (and NaN) are nodata.

### 2.3
Load **orthophotos** from any raster GDAL reads (RGB/RGBA, grayscale replicated to RGB), as RGBA8. Non-8-bit imagery is clamped to 0–255 (a warning is logged).

### 2.4
Georeferencing comes from GDAL: GeoTIFF tags, world files (`.tfw`, `.wld`), or the format's own metadata. An orthophoto without any georeferencing is still accepted (stretched over a DEM, §9.8; not usable to color points).

### 2.5
GDAL's geotransform uses the top-left **corner** of the top-left pixel; the viewer's affine (`RasterGeo`: A, B, C, D, E, F) uses its **center**: `C = gt0 + gt1/2 + gt2/2`, `F = gt3 + gt4/2 + gt5/2`. This is computed after any downsampling, so the center origin follows the resampled pixel size.

### 2.6
Rotated or sheared grids (B ≠ 0 or D ≠ 0) are resampled to north-up on load (GDAL warp), since the rest of the viewer assumes north-up rasters.

### 2.7
Command-line interface:
- `./lasviewer [options] <file>...` loads any number of point clouds and DEMs into one scene.
- `.las` / `.laz` → point cloud loaded in full; names containing `.copc.` → streamed COPC layer.
- `.tif` / `.tiff` → classified by content: 8-bit with ≥3 bands is the **orthophoto**; anything else is a **DEM**. More than one orthophoto is an error.
- `-o ortho.tif` sets the orthophoto; `-d dem.tif` adds a DEM explicitly (required for 8-bit Terrain-RGB DEMs, which look like imagery); `-cop cloud.laz` adds a point cloud explicitly.
- `--snapshot out.ppm` renders until every layer is idle, writes the frame as a binary PPM, and exits.
- `-h` / `--help` prints usage and exits.
- The old forms `lasviewer cloud.laz ortho.tif` and `lasviewer dem.tif ortho.tif` keep working, through the content rule.

---

## 3. Coordinate System & Georeferencing

### 3.1
One **scene CRS**: the first input that declares one, point clouds first, then DEMs, then the orthophoto. CRSs are compared on their **horizontal** part only (a compound `EPSG:2154+5720` equals `EPSG:2154`). Orthophotos and DEMs in another horizontal CRS are **warped into the scene CRS on load** (GDAL, bilinear; DEM gaps become nodata, orthophoto gaps get alpha 0), so every layer and the ortho texture share one coordinate system. Point clouds are not reprojected: one in another CRS is reported and will be misplaced. EPSG codes are shown in the UI.

### 3.2
World-to-GL transform, **shared by every layer of a scene** (`SceneFrame`, `Geo/include/scene_frame.h`):
```
GL_X = (worldX - center.x) / scale       ← easting
GL_Y = (worldZ - center.z) / scale       ← elevation (up)
GL_Z = -(worldY - center.y) / scale      ← northing (NEGATED for north-up)
```
`SceneFrame::toWorld()` is the inverse (used for picked coordinates and the camera target readout).

### 3.3
Z scale defaults to **1.0× (true metric scale)**. The data CRS uses meters for both XY and Z, so 1 unit XY = 1 unit Z. No automatic Z scaling based on terrain type.

### 3.4
Z exaggeration adjustable at runtime: `E` (increase ×1.2), `D` (decrease ÷1.2), `U` (reset to 1.0×). Applied in the vertex shader via `uZScale` uniform.

### 3.5
The frame is fixed **before any layer loads**, from the union of all inputs' header extents (PDAL `bounds` metadata; GeoTIFF tags for DEMs): `center` = centre of the union, `scale` = its diagonal. DEM elevations aren't known before loading, so a DEM-only scene has `center.z = 0`. The frame also carries the union Z range, used by every point elevation ramp, so all tiles and files map the same elevation to the same color.

### 3.6
Orthophoto affine transformed to GL space (including Z negation) for colorization purposes only. The orthophoto quad is NOT rendered as a flat plane (see spec 9.6).

### 3.7
Orthophoto colorization: each point's world (X, Y) is inverse-transformed through the orthophoto's affine (full 2×2 inverse) to a pixel, then bilinearly sampled. Points outside the orthophoto are colored mid-dark gray (0.35, 0.35, 0.35).

### 3.8
Orthophoto colorization must account for the Z negation: `worldY = -gz × scale + centerY`. The negation is baked into the inverse-affine coefficients (`kColZ`, `kRowZ`).

### 3.9
**Coverage gate** (per point-cloud layer): the orthophoto colors a cloud only if it is georeferenced, the cloud is in the scene CRS (the ortho always is, §3.1), and the ortho covers at least **25%** of the cloud's XY extent. Otherwise that layer uses elevation (or file RGB) colors and a message is logged. Coverage is logged for every layer.

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
`uDisableSubsampling` uniform: when set to 1.0, the shader skips the discard logic entirely (currently unused by any layer).

---

## 5. Streaming COPC

### 5.1
For `.copc.laz` files, use **async tile loading** instead of loading the whole file. A background loader thread per COPC layer turns immutable `LoadRequest`s (tile index, resolution, bounds) into `LoadResult`s (GL-space positions, elevation colors and, when an orthophoto applies, orthophoto colors). **The main thread owns all `Tile` state**; the loader never touches a `Tile`, so there is no shared mutable state apart from the two mutex-protected queues.

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
No LRU eviction: every tile stays resident at its last loaded resolution. (The previous 10-second eviction never fired, because every tile was drawn and marked "used" each frame.) A memory budget is planned together with octree-based streaming.

### 5.6
**Culling.** Each frame, a tile is skipped if its box (Y scaled by the Z exaggeration) is entirely outside one frustum plane, or if the **scene-wide Hi-Z pyramid** (`HiZ`, `Viewer/lasviewer.m/src/hiz.cpp`) says it is occluded. After all layers have drawn, the viewer max-reduces the frame's depth buffer into an R32F mip pyramid and reads back one small level (~64×64) once per frame. The next frame tests tile boxes against it. Any layer's depth (including DEM meshes) can therefore occlude tiles. The pyramid is one frame stale by design: the only failure mode is a newly revealed tile missing for one frame. It is invalidated on frames where it isn't rebuilt. It can be toggled from the UI ("Occlusion culling").

### 5.7
Max **16** outstanding tile loads per layer (`maxConcurrentLoads`). A failed load is retried at twice the resolution value (coarser), up to 4 attempts; a tile keeps its previous geometry while a refinement is in flight.

### 5.8
Non-COPC files (regular `.las`/`.laz`, DEMs) use the existing single-VBO path — streaming only activates for `.copc.laz` files (detected by `.copc.` in filename).

---

## 6. Raster Handling (GDAL)

### 6.1
Rasters are opened with GDAL (`GDALDataset::Open`, read-only); an unreadable file or one without bands is an error for that input only.

### 6.2
The orthophoto is capped at **64M pixels**: larger images are read at a reduced size with GDAL's **averaging** resampler, which uses the file's own overviews when present (e.g. COG), so large orthophotos load fast without reading every full-resolution pixel. Warped orthophotos go through a lazy warped VRT: pixels are warped as they are read, from the source's overviews when it has them; without overviews the warp runs at full resolution before averaging (slower for very large images).

### 6.3
After downsampling, the affine is rescaled from the source geotransform (§2.5); the pixel-center origin moves to the center of the new, larger top-left pixel.

### 6.4
DEMs are read in full at native resolution (the GPU heightmap has its own texel cap, §9.7). A warped DEM is warped to an in-memory Float32 raster with nodata −9999 (or the source's declared value).

### 6.5
Terrain-RGB DEMs are decoded before any warp (warping encoded RGB would be meaningless): the decoded elevations are wrapped in an in-memory dataset, which is then warped.

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
**Double-click-to-focus**: double-clicking (< 0.4s between presses, < 5px screen-space movement between them — tracked in `CameraController::onMouseButton`) picks the point under the cursor and recenters on it **without moving the eye**: `target` becomes the picked point, and `yaw`/`pitch`/`distance` are solved so that `position()` (= target + offset(yaw,pitch,distance)) lands exactly back on the eye's position from *before* the pick — `distance = |eye - picked|`, `pitch = asin(dir.y)`, `yaw = atan2(dir.x, dir.z)` where `dir = normalize(eye - picked)`. This is the same math as the Shift+drag head-turn (§7.1b), applied once instead of continuously per mouse-move.

An earlier version instead kept `yaw`/`pitch`/`distance` unchanged and only reassigned `target` — which looks similar but isn't: since the picked point is generally at a different depth than the old target, preserving the *old* distance from a *new* point means `position()` recomputes to somewhere else — a disguised dolly/zoom along the view axis, not a pure recenter. Corrected per explicit user feedback: double-click "should [not] change zoom, [not] move along the view axis, just recenter and change rotation center." Pan/arrow-key step size (`distance × 0.05`, §7.2/§10.2) still adapts automatically afterward, since `distance` is updated to the *true* eye-to-target separation either way.

Picking uses depth-buffer readback + `Camera::unproject()` (matrix inverse of `proj() × view()`), not CPU-side ray-geometry intersection — deliberately, since it works uniformly across every data source this app renders (LAZ/COPC points, the CPU-triangulated DEM mesh, and the GPU-tessellated+displaced DEM mesh) without needing per-type intersection code, and critically the GPU-tessellated path's CPU side only has the coarse, pre-displacement patch corners — a CPU ray test there would hit the wrong (undisplaced) surface entirely.

The `glReadPixels` call happens in `ViewerApp::handlePick()`, right after that frame's scene rendering and before the UI and `glfwSwapBuffers`. It does not happen in the mouse callback, which only detects the double-click and records the screen coordinates. Reading depth from inside the input callback would need `GL_FRONT` (since a swap may have already invalidated `GL_BACK`'s prior contents), which has real cross-platform reliability issues (compositors, some drivers, macOS quirks); reading the default `GL_BACK` immediately after this frame's draw calls is unambiguous. One-frame-old click coordinates at worst, imperceptible in practice. A depth of 1.0 (nothing rendered at that pixel — background) is treated as a miss and doesn't change `target`. A picked point coinciding with the eye itself (near-zero distance) is also treated as a no-op, to avoid a degenerate direction solve.

Verified self-consistent with `test_recenter_keeps_eye_fixed` in `tests/test_basic.cpp` (recomputing `position()` from the solved yaw/pitch/distance reproduces the original eye to within float tolerance) — the math, not the on-screen result, which remains unverified on real hardware.

### 7.4
**Reset** (`R` / Reset button): yaw=0.6, pitch=1.2, target = centre of the visible layers' bounds (Y scaled by the Z exaggeration), distance = 1.3 × their diagonal.

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
Near/far are recomputed every frame (8 corner transforms) from the union of the visible layers' bounds, so they follow streaming and background DEM rebuilds.

---

## 9. Rendering

### 9.1
Point cloud drawn as `GL_POINTS` with perspective-correct sizing: `gl_PointSize = uPointSize × (viewportH / depth)`. Points are circular (fragment shader discards `gl_PointCoord` outside radius 0.5).

### 9.2
Color modes, per layer: **elevation gradient** (blue → cyan → green → yellow → red, 5 stops, over the scene frame's Z range) or the file's RGB, versus **orthophoto-sampled colors**. Set per layer in the panel; `C` toggles every layer. Orthophoto is the default when available.

### 9.3
Color toggle is instant. Full-load clouds re-upload their color VBO; COPC tiles keep both color buffers and re-point attribute 1; DEMs switch between texture and ramp in the shader.

### 9.4
Background: dark gray (0.10, 0.11, 0.13).

### 9.5
Depth test enabled. MSAA (4×) enabled via `GLFW_SAMPLES`.

### 9.6
Orthophoto quad NOT rendered as a flat plane in the point-cloud path — only used for point colorization there. (The flat-plane rendering was removed per user request; only the colorization remains for LiDAR point clouds. The DEM/DSM path, §9.7–9.9, does render a textured surface — that is a different code path, not a reintroduction of the removed flat quad.)

### 9.7
**DEM/DSM mesh**: a GPU-tessellated adaptive quadtree (`DEMTessMesh`; design in `docs/design-tessellation-displacement.md`). The CPU builds patches (subdivision by angular geometric error and texture span, balance pass, nodata-aware); the tessellation shaders refine them and displace them from an R32F heightmap. The coarsest level (maxLevel 0) is shown immediately and the full mesh is built on a background thread, then swapped in; parameter changes (max level, collapsing angle) rebuild the same way. The old CPU-triangulated fallback (`DEMMesh`) was removed: GL 4.1 is available on all supported macOS hardware.

### 9.8
DEM mesh texturing: the orthophoto and the DEM are both in the scene CRS (§3.1), so the texture coordinates follow directly from their affines. If a georeferenced ortho doesn't overlap the DEM, texturing is skipped for that DEM (elevation ramp) rather than stretching unrelated imagery over it. An ortho with no georeferencing at all is stretched over the DEM extent. Without an orthophoto the mesh uses an elevation ramp: **dark blue-violet → teal → yellow** (3 stops), distinct from the point cloud's 5-stop ramp (§9.2).

### 9.9
DEM elevations are read through GDAL as Float32 (`GDALRasterBand::RasterIO`), for any sample type; Terrain RGB per §2.2.

---

## 10. Controls (Layout-Independent Letter Shortcuts)

### 10.1
Letter shortcuts use GLFW's **character callback**, so they follow the active keyboard layout (AZERTY, QWERTZ, Dvorak, …) instead of US physical key positions. Non-character keys (arrows, Tab, Esc, Shift) use the key callback. Keys and clicks that ImGui wants (focus in a widget, cursor over a window) aren't passed to the viewer; mouse releases always are, so a drag ending over a window can't leave a button stuck.

| Key | Action |
|-----|--------|
| `R` | Reset view |
| `E` / `D` / `U` | Z exaggeration ×1.2 / ÷1.2 / reset |
| `F` / `S` | Point density ×1.5 / ÷1.5, and DEM max level +1 / −1 |
| `M` / `N` | Point size ×1.3 / ÷1.3 |
| `C` | Toggle colors on all layers |
| `P` | Perspective / orthographic |
| `I` / `O` | DEM collapsing angle ÷1.3 / ×1.3 (background rebuild) |
| `W` / `A` / `G` | DEM wireframe / displacement / coarse-patch edges |
| `V` / `T` | Side / top view |
| `B` | COPC tile boxes (yellow = loaded, orange = refining) |
| `L` / `H` | Log window / help window |
| `Tab` | Show / hide the side panel |
| `ESC` | Quit |

Every setting above is also in the side panel.

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
**Side panel** (ImGui, `Viewer/lasviewer.m/src/viewer_ui.cpp`):
- *Layers*: a visibility checkbox per layer, a status line (points drawn, tiles loading, patches, rebuilding), and the layer's own settings (colors; for DEMs: max level, collapsing angle, pixels/segment, wireframe, displacement, patch edges). The file path and EPSG code are in a tooltip. Orthophoto name, size and EPSG code are listed below the layers.
- *View*: reset/top/side, projection, Z exaggeration, point size, point density, occlusion culling, tile boxes.
- *Info*: fps, camera target and distance in world units, last picked point.

### 11.2
Help (`H`) is a window listing all controls; it replaces the 10-second text overlay.

### 11.3
While loading, a centred "Loading …" message is drawn between steps.

### 11.4
`-h` / `--help` command-line flag prints all controls to stderr and exits.

### 11.5
**Log window** (`L`): `std::cerr` is intercepted by `LogCapture` (thread-safe; loader threads log too), keeping the last 1000 lines and forwarding everything to the real stderr. The window scrolls with new lines unless the user has scrolled up; lines starting with `ERROR` / `WARNING` are colored. The per-second stats lines on stderr were removed; that information is in the panel.

---

## 12. Build System

### 12.1
Workspace layout (bmake-it): frameworks at the repository root, each with a `makefile` declaring `PREREQS=`; modules are `NAME.m` directories with `src/`, optional `tests/` and `mk/`.

| Framework | PREREQS | Module(s) |
|---|---|---|
| `GIS` | — | imported: `libpdalcpp.m` (PDAL), `libgdal.m` (GDAL), `libglm.m` (glm, header-only) |
| `GUI` | — | imported `libglfw.m` (GLFW); fetched `libimgui.m` (Dear ImGui: `IMPORT=fetch:`, static, `WARN=none`, `LIBS=glfw`) |
| `Geo` | GIS | `libgeo.m`: static, `OPENMP=yes`, `LIBS=pdalcpp gdal` |
| `Viewer` | Geo GIS GUI | `lasviewer.m`: `PROG=lasviewer`, `OPENMP=yes`, `LIBS=geo imgui glfw pdalcpp` |

Third-party libraries are grouped by subject, one module per library. Header visibility is per framework, so this keeps `Geo` (no OpenGL, no GUI) from seeing GLFW or ImGui headers; the Viewer sees everything.

Output: `build/<os>-<arch>/bin/lasviewer` at the workspace root (e.g. `build/macos-arm64/`).

### 12.2
**External libraries are ordinary library modules.** Each one imports the installed library instead of compiling it (`IMPORT=`), staging only the headers listed in `IMPORT_HEADERS=` into the framework's public include path, so consumers use `PREREQS=` and `LIBS=` exactly as for the project's own libraries. External frameworks set `PUBLIC_HEADERS_SYSTEM=yes` (their headers reach consumers via `-isystem`).
- PDAL: `IMPORT_HEADERS=pdal`. GLFW: `IMPORT_HEADERS=GLFW` via pkg-config `glfw3`.
- Dear ImGui: `IMPORT=fetch:imgui`. The release archive (`FETCH_URL=`, GitHub tag `v1.91.9b`) is downloaded once into `distfiles/`, checked against the committed `distinfo` (SHA-256 and size), extracted into `work/`, and its `SRCS=` (core + the GLFW/OpenGL3 backends) compiled by bmake-it; `IMPORT_HEADERS=` are staged like an imported library's. Only the makefile, `distinfo` and `mk/` hook are committed.
- GDAL installs its headers loose, so its module lists the 49 headers lasviewer's code reaches (the list and the command that regenerates it are in `GIS/libgdal.m/makefile`).
- glm is header-only with no pkg-config file: its include directory comes from per-target hooks (`GIS/libglm.m/mk/pre.<os>.mk`), and no library is staged.

### 12.2b
Per-target settings are bmake-it `mk/` hook files next to the module: `Viewer/lasviewer.m/mk/local.macos.mk` (`-framework OpenGL Cocoa IOKit`), `local.linux.mk` (`-lGL`), `GIS/libglm.m/mk/pre.<os>.mk`.

### 12.2c
**Workarounds for current bmake-it issues** (to remove when bmake-it is fixed):
1. Imported link lists: bmake-it records pkg-config's `--static` list minus the library's own `-L` directory, so the transitive libraries can't be found. PDAL and GDAL are therefore resolved with `IMPORT_PREFIX=/opt/local` in `mk/pre.macos.mk` (their link line is then just `-lpdalcpp` / `-lgdal`, which is all a shared library needs).
2. (Not needed with the current grouping, still a bmake-it bug:) a static library's transitive `-l` flags reach consumers without their library directory. `libgeo.a` passes `-lgdal` on; the Viewer finds it only because it lists `GIS` (for PDAL), whose library directory also holds GDAL.
3. `IMPORT=fetch:` doesn't create `obj/` subdirectories for `SRCS` in subdirectories of the fetched tree, and doesn't put the tree's root on the include path when compiling it. `GUI/libimgui.m/mk/local.mk` does both.
4. Not worked around: every build recompiles a `fetch:` module (the fetch step is a `.PHONY` prerequisite of its compile rules), about 10 s for ImGui.

### 12.3
`bmake test` builds and runs the atf-c++ tests with Kyua: `Geo/libgeo.m/tests/raster_test.cpp` (linked against `libgeo.a`) and `Viewer/lasviewer.m/tests/{basic_test,scene_test}.cpp` (linked against the viewer's objects except `main.o`). JUnit results go to `build/<key>/runs/<run>/test-results.xml` in each module.

### 12.4
`.clang-tidy` config: bugprone-*, cert-*, misc-*, modernize-*, performance-*, readability-* checks. Magic numbers and identifier length suppressed.

### 12.5
`.clang-format` config: LLVM base, C++17, 100-column limit, 4-space indent, attached braces.

### 12.6
`docs/Doxyfile` for Doxygen documentation generation.

---

## 13. Testing

### 13.1
Three atf-c++ test programs, run by `bmake test` (§12.3): `basic_test` re-derives formulas locally; `scene_test` tests the real frame and camera code; `raster_test` tests the real GDAL raster I/O on GeoTIFFs it writes itself: pixel-center georeferencing, downsampling and affine rescale, `.tfw` world files, declared and undeclared nodata, Terrain RGB decoding, horizontal CRS comparison and extent transformation, warping a DEM from EPSG:4326 into EPSG:2154 (value checked at a transformed point), and a rotated grid made north-up.

### 13.2
Test coverage:
1. Coordinate transform (world → GL, including Z negation)
2. Raster affine inverse (world → pixel)
3. Spatial grid subsampling cell size
4. Depth-based subsampling formula
6. Morton (Z-order) code interleaving
7. Near/far plane computation from bbox corners
8. Elevation gradient color ramp
9. Tile LOD resolution computation — mirrors the current PCA/OBB projected-area algorithm in `Viewer/lasviewer.m/src/copc_streamer.cpp` (§5.4), not the earlier radius-based formula. Keeping this test in sync with `desiredResolution()` whenever that function changes is a manual step since the test replicates the formula rather than linking against it.

### 13.3
Tests use atf-c++ (`ATF_TEST_CASE`, `ATF_REQUIRE`); Kyua runs each test case in its own scratch directory and reports pass/fail per case.

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
Stats are shown in the UI: fps, points drawn per layer, tiles drawn and culled, tiles loading, DEM patch counts.

---

## 16. Error Handling

### 16.1
An orthophoto GDAL can't read is reported; the scene loads without it.

### 16.2
An input PDAL can't open (driver inference failure, missing or corrupt file) is reported and skipped; the viewer exits with an error only if no layer at all could be loaded.

### 16.3
An orthophoto without georeferencing is accepted: it can't color points (logged per layer) and is stretched over DEMs (§9.8). A DEM without georeferencing is rejected.

### 16.4
Shader compile/link failure: print error log, return 0, exit.

### 16.5
A raster in another CRS that GDAL can't warp into the scene CRS is reported and skipped.

### 16.6
Raster read failures (`RasterIO` errors) are reported per input; the input is skipped.

---

## 17. File Structure

bmake-it workspace layout (§12.1); see README.md → Architecture for the per-file map.

---

## 18. Scene and Layers

### 18.1
A **Scene** (`Viewer/local/include/scene.h`) holds the shared `SceneFrame` (§3.2), at most one orthophoto (shared by every layer, outliving them) and an ordered list of **layers**. Loading (`Scene::load`): read all header extents → fix the frame → warn on CRS disagreement → load the orthophoto → create one layer per input. An input that fails is reported and skipped.

### 18.2
The **Layer** interface (`Viewer/local/include/layer.h`): `bounds()` (GL space), `update(ctx)` (every frame, visible or not, to drain background work), `render(ctx)`, `renderOverlay(ctx)`, `wantsHiZ()`, `drawUI()` (ImGui settings), `status()`, `busy()`, and `handleAction(LayerAction)` for keyboard actions that apply to every layer (colors, DEM detail, …). Implementations: `PointCloudLayer`, `CopcLayer`, `DemLayer`.

### 18.3
`RenderContext` carries the view/projection matrices, camera position, FOV, viewport, projection mode, global `ViewSettings` (Z exaggeration, point size, point density, occlusion, tile boxes), the shared `Programs`, and the previous frame's Hi-Z pyramid.

### 18.4
Frame order (`ViewerApp::renderFrame`): near/far from the visible bounds → `update` all layers → clear → `render` visible layers → build Hi-Z if a visible layer wants it (otherwise invalidate it) → overlays → resolve a pending double-click pick from the depth buffer → ImGui.

### 18.5
Navigation is in `CameraController` (`Viewer/lasviewer.m/src/camera_controller.cpp`), separate from GLFW callbacks and unit-tested: orbit, head turn, pan (scaled by window height, the cursor's coordinate space), fly, arrows, home, top/side, `focusOn()` (§7.3b), double-click detection.
