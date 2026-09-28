# lasviewer

A lightweight C++ viewer for **LAZ/LAS/COPC point clouds** and **GeoTIFF DEMs**, with orthophoto draping. Built on GLFW + OpenGL, laz-perf and copc-lib for point clouds, GDAL for rasters and Dear ImGui for the interface.

![lasviewer](https://img.shields.io/badge/platform-macOS%20%7C%20Linux-blue)
![language](https://img.shields.io/badge/language-C%2B%2B17-orange)
![license](https://img.shields.io/badge/license-BSD--3-blue)

---

## Features

- **Several files in one scene**: adjacent LiDAR tiles, point clouds on top of a DEM, and so on. All layers share one coordinate frame, so they line up exactly.
- **Layer panel** (Dear ImGui): per-layer visibility, colors and settings; view settings; log; help.
- **LAZ/LAS**: loaded in full through laz-perf (LAS 1.0–1.4, point formats 0–10), thinned on an XY grid to at most 2M points.
- **COPC streaming**: each file is split into 8×8 tiles, loaded asynchronously, and refined based on how large each tile appears on screen (per-tile PCA/OBB). Tiles outside the view or behind closer geometry are skipped; occlusion comes from a scene-wide Hi-Z depth pyramid.
- **Orthophoto coloring**: points sample a GeoTIFF orthophoto, which must cover at least 25% of the cloud and use the same CRS. The colors can be switched between the orthophoto and elevation (or the file's RGB) at any time.
- **DEM/DSM terrain**: an adaptive quadtree of GPU-tessellated patches with height displacement, textured by the orthophoto. Supports any raster GDAL reads (Float/Int elevation, IGN Terrain-RGB), with its declared nodata value.
- **GPU depth subsampling**: gives an even screen-space point density.
- **Navigation**: orbit, look-around, pan, fly, and double-click to focus. Perspective and orthographic projections, Z exaggeration.
- **One CRS per scene**: taken from the first input that declares one (point clouds first). Orthophotos and DEMs in another CRS are warped into it on load (GDAL, bilinear). Point clouds aren't reprojected; one in another CRS is reported.

---

## Quick Start

### Prerequisites

A dependency that can be downloaded from one single source for every supported platform is fetched by the build itself; anything else is a prerequisite, installed with the platform's own package manager before building.

| Prerequisite | MacPorts | Homebrew | pkgsrc (macOS, NetBSD, Linux) | FreeBSD pkg | Debian/Ubuntu apt | Alpine apk | Windows (OSGeo4W) |
|---|---|---|---|---|---|---|---|
| GDAL | `gdal` | `gdal` | `geography/gdal-lib` | `gdal` | `libgdal-dev` | `gdal-dev` | `gdal-devel` |
| GLFW | `glfw` | `glfw` | `graphics/glfw` | `glfw` | `libglfw3-dev` | `glfw-dev` | — |
| glm (headers) | `glm` | `glm` | `graphics/glm` | `glm` | `libglm-dev` | `glm-dev` | — |
| OpenMP runtime | `libomp` (with clang) | `libomp` | with the compiler | with the compiler | with gcc (`libgomp`) | with gcc | with MSVC |
| Build tools | `bmake cmake pkgconfig` | `bmake cmake pkg-config` | `bmake cmake pkgconf` | `cmake pkgconf` (base `make` is bmake) | `bmake cmake pkg-config` | `bmake cmake pkgconf` | — |
| Tests only | `atf kyua` | `atf kyua` | `atf kyua` | in base | — | — | — |

For example, on macOS with MacPorts:
```bash
sudo port install gdal glfw glm libomp bmake cmake pkgconfig atf kyua
```

Fetched and built by the first `bmake` (network access needed once; checked against each module's `distinfo`, nothing committed): **laz-perf** 3.4.0 and **copc-lib** 2.6.3 (point-cloud reading, built with their own CMake) and **Dear ImGui** 1.91.9b. All dependencies must come from one source: don't mix, e.g., a GDAL from MacPorts with one from Homebrew.

The build uses [bmake-it](https://github.com/SylvainSouche/bmake-it), a BSD-make build system. Get it once and let bmake find its `mk/` files:

```bash
git clone https://github.com/SylvainSouche/bmake-it.git
sh bmake-it/scripts/install-env.sh      # adds MAKESYSPATH to your shell rc; or:
export MAKESYSPATH=/path/to/bmake-it/mk:/opt/local/share/mk
```

Use a bmake-it that includes `IMPORT=fetch:` (merged into its `main`, PR #2).

### Build

```bash
bmake            # → build/macos-arm64/bin/lasviewer (build/<os>-<arch>/ on other hosts)
```

### Run

```bash
# One point cloud (LAS/LAZ loaded in full, .copc.laz streamed)
build/macos-arm64/bin/lasviewer cloud.laz
build/macos-arm64/bin/lasviewer cloud.copc.laz ortho.tif

# Several tiles + one orthophoto
build/macos-arm64/bin/lasviewer tile_a.copc.laz tile_b.copc.laz -o ortho.tif

# A DEM with a point cloud on top
build/macos-arm64/bin/lasviewer dem.tif cloud.copc.laz -o ortho.tif
```

On the command line, `.tif` files are sorted by content: 8-bit RGB(A) imagery is the orthophoto, and elevation rasters are DEMs. 8-bit Terrain-RGB DEMs look like imagery, so pass them with `-d`. Run `lasviewer -h` for all options.

---

## Controls

The side panel (toggle with **Tab**) holds the layer list and view settings. The keyboard shortcuts follow your keyboard layout (AZERTY, QWERTZ, and so on):

| Input | Action |
|-----|--------|
| **Left drag** | Orbit |
| **Shift + left drag** | Look around (eye fixed) |
| **Right / middle drag** | Pan |
| **Wheel** | Move forward / back |
| **Double-click** | Focus on the point under the cursor (new orbit pivot, eye unchanged); its world coordinates appear under *Info* |
| **Arrows** | Move; Shift+Up/Down = forward/back |
| `R` | Reset view to the whole scene |
| `T` / `V` | Top / side view |
| `P` | Perspective / orthographic |
| `E` / `D` / `U` | Z exaggeration up / down / reset |
| `F` / `S` | Finer / coarser: point density, and DEM quadtree depth |
| `M` / `N` | Bigger / smaller points |
| `C` | Toggle colors (orthophoto ↔ elevation/RGB) on all layers |
| `I` / `O` | DEM collapsing angle finer / coarser (rebuilds in the background) |
| `W` / `A` / `G` | DEM wireframe / displacement / coarse-patch edges |
| `B` | COPC tile bounding boxes |
| `L` / `H` | Log window / help window |
| `Tab` | Show / hide the side panel |
| `Esc` | Quit |

---

## Sample Data

| Dataset | Format | Source |
|---------|--------|--------|
| **IGN LiDAR HD** | `.copc.laz`, EPSG:2154 | [IGN LiDAR HD](https://ignf.github.io/ignf-lidar-hd/) |
| **IGN orthophotos** | GeoTIFF, EPSG:2154 | [IGN Géoservices](https://geoservices.ign.fr/) |
| **IGN RGE Alti** | GeoTIFF DEM, 1 m | [IGN RGE Alti](https://ign.fr/rgealti) |
| **USGS 3DEP** | `.laz`, COPC | [AWS open data](https://registry.opendata.aws/usgs-lidar/) |
| **OpenTopography** | `.laz`, `.las` | [OpenTopography](https://opentopography.org/) |
| **Copernicus DEM** | GeoTIFF (GLO-30) | [Copernicus](https://spacedata.copernicus.eu/) |

To make a DSM from a point cloud for testing, with PDAL's command-line tool if you have it (lasviewer itself doesn't use PDAL):
```bash
pdal translate cloud.copc.laz dsm.tif --readers.copc.resolution=0.5 \
  --writers.gdal.resolution=1 --writers.gdal.output_type=max \
  --writers.gdal.data_type=float32 --writers.gdal.nodata=-9999
```

---

## Build System

The build is [bmake-it](https://github.com/SylvainSouche/bmake-it): a workspace (this directory) of frameworks, each holding modules (`*.m`). Third-party libraries are grouped by subject in two frameworks, `GIS` and `GUI`, one module per library. An external library's module either imports the installed library (`IMPORT=pkg:`/`prefix:`) or downloads and compiles its release (`IMPORT=fetch:`, Dear ImGui), so it's used exactly like the project's own libraries. Downloads go to `<module>/distfiles/` and `<module>/work/`, never committed.

```bash
bmake                 # build everything for the host (build/<os>-<arch>/)
bmake test            # atf-c++ tests via Kyua (raster_test, basic_test, scene_test)
bmake clean
bmake help
cd Geo && bmake       # build / test one framework (or one module: cd Geo/libgeo.m)
```

| Framework | Contents | Notes |
|---|---|---|
| `GIS` | laz-perf and copc-lib (fetched, built with CMake); GDAL and glm (imported) | glm is header-only |
| `GUI` | GLFW imported; Dear ImGui fetched and compiled (`libimgui.a`) | ImGui comes from its release archive (`IMPORT=fetch:`), warnings off |
| `Geo` | `libgeo.a`: point-cloud (laz-perf) and raster (GDAL) I/O | no OpenGL; `raster_test`, `las_test` |
| `Viewer` | the `lasviewer` program | `basic_test`, `scene_test` |

Headers follow bmake-it's visibility rules: `<fw>/include/` is public (reached through `PREREQS=`), `<fw>/local/include/` is shared by that framework's modules, `<module>/include/` is private.

Per-target settings live in `mk/` hook files next to the module, for example `Viewer/lasviewer.m/mk/local.macos.mk` (OpenGL frameworks) or `GIS/libglm.m/mk/pre.macos.mk` (where glm's headers are).

`GNUmakefile` (plain GNU make, `make build`) builds the same binary from the same tree; it is kept temporarily, for comparison.

`--snapshot out.ppm` renders until every layer has finished loading, saves the frame, and exits. It's useful for scripted visual checks.

---

## Architecture

```
makefile                     bmake-it workspace
GIS/                         liblazperf.m, libcopc.m (fetched + CMake), libgdal.m, libglm.m (imported)
GUI/                         libglfw.m (imported), libimgui.m (Dear ImGui, fetched: makefile + distinfo)
Geo/
  include/                   public: point_cloud.h, raster.h, scene_frame.h
  libgeo.m/src/              point_cloud.cpp (laz-perf), las_format.cpp (records, CRS), raster.cpp (GDAL)
  libgeo.m/tests/            raster_test.cpp, las_test.cpp
Viewer/
  local/include/             headers shared inside Viewer
  lasviewer.m/src/
    main.cpp                 command line → LoadPlan → ViewerApp
    viewer_app.cpp           window, GL context, frame loop, input routing, picking
    viewer_ui.cpp            ImGui panels (layers, view, info, log, help)
    camera.cpp               orbit camera, projection, near/far from bounds
    camera_controller.cpp    mouse/keyboard navigation, double-click focus
    scene.cpp                Scene: layers + shared frame + orthophoto; loading
    point_cloud_layer.cpp    LAS/LAZ loaded in full
    copc_layer.cpp           COPC streaming layer (wraps TileGrid)
    copc_streamer.cpp        TileGrid: copc-lib loader thread, tile LOD, upload, draw
    dem_layer.cpp            DEM layer (wraps DEMTessMesh)
    dem_tess_mesh.cpp        adaptive quadtree + GPU tessellation, background rebuilds
    hiz.cpp                  scene-wide Hi-Z occlusion pyramid, frustum test
    shaders.cpp              embedded GLSL + compile/link helpers
    log_capture.cpp          std::cerr → in-app log (thread-safe)
  lasviewer.m/tests/         basic_test.cpp (formulas), scene_test.cpp (frame + camera)
docs/                        design notes (DEM tessellation), Doxyfile
```

**Loading.** The scene first reads every input's header (LAS header and CRS records; GDAL for rasters), picks the scene CRS, expresses every extent in it, and fixes one `SceneFrame` from their union:

```
GL_X =  (worldX - center.x) / scale     easting
GL_Y =  (worldZ - center.z) / scale     elevation (up)
GL_Z = -(worldY - center.y) / scale     northing, negated for north-up
```

It then loads the orthophoto once (warped into the scene CRS if needed) and creates one layer per input; DEMs are warped the same way when they load.

**Each frame.** Near/far planes are computed from the union of the visible layers. Then:
1. Every layer gets `update()`, which drains background work.
2. Every visible layer gets `render()`.
3. The Hi-Z pyramid is built from the frame's depth, for the next frame.
4. Overlays are drawn.
5. Any pending double-click pick is resolved from the depth buffer.
6. The UI is drawn.

Adding a new data type means writing one `Layer` subclass (in `Viewer/`); the `Layer` interface is in `Viewer/local/include/layer.h`.

---

## Limitations

- One orthophoto per scene, downsampled to ≤64 Mpx on load (GDAL averaging, using the file's overviews when it has them). It isn't streamed at higher resolution when zooming in.
- COPC streaming still splits each file into a fixed 8×8 grid of tiles: one loader thread answers each tile from copc-lib's octree nodes at the requested depth (decompressed nodes cached, 512 MB), but tiles aren't yet replaced by per-node loading, and there's no GPU memory budget.
- Point attributes other than XYZ/RGB (classification, intensity, returns) aren't used yet.
- Point clouds are not reprojected: a cloud in another CRS than the scene's is reported and will be misplaced.
- DEM display needs an OpenGL 4.0+ context (macOS provides 4.1). On a 3.3-only context, DEMs are skipped and point clouds still work.
- No measurements or exports beyond `--snapshot`.

---

## License

BSD 3-Clause License. See [LICENSE.md](LICENSE.md). Dear ImGui is MIT-licensed (its `LICENSE.txt` is in the fetched archive).
