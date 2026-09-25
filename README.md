# lasviewer

A lightweight C++ viewer for **LAZ/LAS/COPC point clouds** and **GeoTIFF DEMs**, with orthophoto draping. Built on GLFW + OpenGL, PDAL for point cloud I/O, GDAL for rasters and Dear ImGui for the interface.

![lasviewer](https://img.shields.io/badge/platform-macOS%20%7C%20Linux-blue)
![language](https://img.shields.io/badge/language-C%2B%2B17-orange)
![license](https://img.shields.io/badge/license-BSD--3-blue)

---

## Features

- **Several files in one scene**: adjacent LiDAR tiles, point clouds on top of a DEM, and so on. All layers share one coordinate frame, so they line up exactly.
- **Layer panel** (Dear ImGui): per-layer visibility, colors and settings; view settings; log; help.
- **LAZ/LAS**: loaded in full through PDAL, thinned on an XY grid to at most 2M points.
- **COPC streaming**: each file is split into 8×8 tiles, loaded asynchronously, and refined based on how large each tile appears on screen (per-tile PCA/OBB). Tiles outside the view or behind closer geometry are skipped; occlusion comes from a scene-wide Hi-Z depth pyramid.
- **Orthophoto coloring**: points sample a GeoTIFF orthophoto, which must cover at least 25% of the cloud and use the same CRS. The colors can be switched between the orthophoto and elevation (or the file's RGB) at any time.
- **DEM/DSM terrain**: an adaptive quadtree of GPU-tessellated patches with height displacement, textured by the orthophoto. Supports any raster GDAL reads (Float/Int elevation, IGN Terrain-RGB), with its declared nodata value.
- **GPU depth subsampling**: gives an even screen-space point density.
- **Navigation**: orbit, look-around, pan, fly, and double-click to focus. Perspective and orthographic projections, Z exaggeration.
- **One CRS per scene**: taken from the first input that declares one (point clouds first). Orthophotos and DEMs in another CRS are warped into it on load (GDAL, bilinear). Point clouds aren't reprojected; one in another CRS is reported.

---

## Quick Start

### Install dependencies

**macOS (MacPorts):**
```bash
sudo port install glfw pdal gdal glm pkgconfig
```

**macOS (Homebrew):**
```bash
brew install glfw pdal gdal glm pkg-config
```

**Linux (Debian/Ubuntu):**
```bash
sudo apt install libglfw3-dev libpdal-dev libgdal-dev libglm-dev pkg-config
```

GDAL is also a PDAL dependency, so it's normally installed already. Dear ImGui is vendored in `third_party/imgui` (v1.91.9b, MIT), so there's nothing to install for it.

### Build

```bash
make build
```

### Run

```bash
# One point cloud (LAS/LAZ loaded in full, .copc.laz streamed)
./lasviewer cloud.laz
./lasviewer cloud.copc.laz ortho.tif

# Several tiles + one orthophoto
./lasviewer tile_a.copc.laz tile_b.copc.laz -o ortho.tif

# A DEM with a point cloud on top
./lasviewer dem.tif cloud.copc.laz -o ortho.tif
```

On the command line, `.tif` files are sorted by content: 8-bit RGB(A) imagery is the orthophoto, and elevation rasters are DEMs. 8-bit Terrain-RGB DEMs look like imagery, so pass them with `-d`. Run `./lasviewer -h` for all options.

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

To make a DSM from a point cloud for testing:
```bash
pdal translate cloud.copc.laz dsm.tif --readers.copc.resolution=0.5 \
  --writers.gdal.resolution=1 --writers.gdal.output_type=max \
  --writers.gdal.data_type=float32 --writers.gdal.nodata=-9999
```

---

## Build System

```bash
make            # build (default)
make debug      # -g build → lasviewer-debug
make test       # unit tests (test_basic: formulas; test_scene: frame + camera; test_raster: GDAL raster I/O)
make lint       # clang-tidy / cppcheck
make fmt        # clang-format
make dist       # source tarball
make doc        # Doxygen
make clean
make run ARGS="cloud.copc.laz ortho.tif"
```

`--snapshot out.ppm` renders until every layer has finished loading, saves the frame, and exits. It's useful for scripted visual checks.

---

## Architecture

```
main.cpp                 command line → LoadPlan → ViewerApp
src/
  viewer_app.h/.cpp      window, GL context, frame loop, input routing, picking
  viewer_ui.cpp          ImGui panels (layers, view, info, log, help)
  camera.h/.cpp          orbit camera, projection, near/far from bounds
  camera_controller.*    mouse/keyboard navigation, double-click focus
  scene.h/.cpp           Scene: layers + shared frame + orthophoto; loading
  scene_frame.h          world ↔ GL transform shared by all layers
  layer.h                Layer interface, RenderContext, ViewSettings
  point_cloud_layer.*    LAS/LAZ loaded in full
  copc_layer.*           COPC streaming layer (wraps TileGrid)
  copc_streamer.*        TileGrid: loader thread, tile LOD, upload, draw
  dem_layer.*            DEM layer (wraps DEMTessMesh)
  dem_tess_mesh.*        adaptive quadtree + GPU tessellation, background rebuilds
  hiz.*                  scene-wide Hi-Z occlusion pyramid, frustum test
  point_cloud.*          PDAL loading, header reading
  raster.*               GDAL raster I/O: orthophotos, DEMs, CRS, warping
  shaders.*              embedded GLSL + compile/link helpers
  log_capture.*          std::cerr → in-app log (thread-safe)
third_party/imgui/       Dear ImGui core + GLFW/OpenGL3 backends
tests/                   test_basic.cpp, test_scene.cpp, test_raster.cpp
docs/                    design notes (DEM tessellation), Doxyfile
```

**Loading.** The scene first reads every input's header (PDAL metadata; GDAL for rasters), picks the scene CRS, expresses every extent in it, and fixes one `SceneFrame` from their union:

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

Adding a new data type means writing one `Layer` subclass.

---

## Limitations

- One orthophoto per scene, downsampled to ≤64 Mpx on load (GDAL averaging, using the file's overviews when it has them). It isn't streamed at higher resolution when zooming in.
- COPC streaming uses a fixed 8×8 grid of PDAL `bounds` + `resolution` queries rather than the COPC octree, with one loader thread and no memory budget.
- Point attributes other than XYZ/RGB (classification, intensity, returns) aren't used yet.
- Point clouds are not reprojected: a cloud in another CRS than the scene's is reported and will be misplaced.
- DEM display needs an OpenGL 4.0+ context (macOS provides 4.1). On a 3.3-only context, DEMs are skipped and point clouds still work.
- No measurements or exports beyond `--snapshot`.

---

## License

BSD 3-Clause License. See [LICENSE.md](LICENSE.md). Dear ImGui is MIT-licensed (`third_party/imgui/LICENSE.txt`).
