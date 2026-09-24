# lasviewer

A lightweight, single-file C++ viewer for **LAZ/LAS point clouds** and **GeoTIFF DEMs**, with optional orthophoto-based point coloring. Built on GLFW + OpenGL 3.3, using PDAL for point cloud I/O and libtiff for raster I/O.

![lasviewer](https://img.shields.io/badge/platform-macOS%20%7C%20Linux-blue)
![language](https://img.shields.io/badge/language-C%2B%2B17-orange)
![license](https://img.shields.io/badge/license-BSD--3-blue)

---

## Features

**Primary workflow: LiDAR point clouds + orthophoto coloring**
- **LAZ/LAS loading** via PDAL (supports COPC, compressed, and standard formats)
- **Orthophoto coloring** — samples a GeoTIFF orthophoto to color each LiDAR point by its real-world appearance (requires ≥25% area overlap with the cloud)
- **Streaming COPC support** — loads tiles asynchronously with apparent-size-driven LOD (per-tile PCA/OBB projected onto the view); only fetches the data needed for the current view
- **GPU-based depth subsampling** — the vertex shader stochastically discards points based on depth, giving uniform screen-space density
- **Elevation gradient coloring** — blue → cyan → green → yellow → red

**Alternative workflow: DEM/DSM terrain mesh** (newer, actively developed alongside the point-cloud path)
- **Adaptive triangulated mesh** — GeoTIFF DEMs/DSMs are triangulated with a quadtree (geometry-error + texture-span driven, T-junction-free) instead of rendered as a point cloud
- **Terrain RGB decoding** — supports the IGN MNS LiDAR HD 8-bit RGB elevation encoding, in addition to standard Float32/Int16/etc. rasters
- Textured from an orthophoto, with the DEM's and orthophoto's actual CRS (EPSG code) read from each file and compared — a real CRS mismatch is auto-reprojected if PROJ is available (see Install dependencies), otherwise reported and texturing is skipped rather than stretched onto the wrong location. Stretch-fit fallback is still used only when the orthophoto has no geo tags at all (no correspondence info to contradict). Elevation color ramp (dark blue-violet → teal → yellow) if no orthophoto is given or usable

**Shared**
- **Dynamic near/far planes** — computed from the bbox every frame for optimal depth precision
- **Perspective + orthographic projection** — toggle with a key
- **On-screen help & console log overlays** — rendered with FreeType, toggle with `H` / `L`
- Occlusion culling for streamed point-cloud tiles, via a Hi-Z (max-mip depth pyramid) built from each frame's own depth — see [Limitations](#limitations) for the one-frame-stale tradeoff

---

## Quick Start

### Install dependencies

**macOS (MacPorts):**
```bash
sudo port install glfw pdal tiff glm pkgconfig freetype
# Optional, for automatic CRS reprojection when a DEM and orthophoto use
# different coordinate systems (see Limitations) — builds and runs fine
# without it, CRS mismatches are just detected/reported, not auto-fixed:
sudo port install proj7
```

**macOS (Homebrew):**
```bash
brew install glfw pdal libtiff glm pkg-config freetype
# Optional (see note above):
brew install proj
```

**Linux (Debian/Ubuntu):**
```bash
sudo apt install libglfw3-dev libpdal-dev libtiff-dev libglm-dev libfreetype-dev pkg-config
# Optional (see note above):
sudo apt install libproj-dev
```

### Build

```bash
make build
```

The Makefile auto-detects your OS and package manager (MacPorts, Homebrew, or system packages). No manual configuration needed.

### Run

```bash
# Point cloud (LAZ/LAS/COPC)
./lasviewer cloud.laz

# Point cloud + orthophoto (colors each point from the aerial photo)
./lasviewer cloud.laz ortho.tif

# GeoTIFF DEM/DSM (elevation raster → adaptive triangulated mesh)
./lasviewer dem.tif

# DEM + orthophoto
./lasviewer dem.tif ortho.tif

# Explicit flags
./lasviewer -cop cloud.laz -o ortho.tif
./lasviewer -d dem.tif -o ortho.tif

# COPC streaming (async tile loading with LOD)
./lasviewer -cop data.copc.laz
```

---

## Controls

| Key | Action |
|-----|--------|
| **Left drag** | Orbit (rotate around target) |
| **Right / Middle / Shift+Left drag** | Pan (translate) |
| **Arrow keys** | Pan |
| **Mouse wheel** | Zoom |
| `R` | Reset camera |
| `E` / `D` | Increase / decrease Z exaggeration |
| `U` | Reset Z to 1.0× (true scale) |
| `S` / `F` | Decrease / increase point density (LAZ/COPC), or DEM quadtree depth ceiling — the coarse-patch resolution limit (triggers a mesh reload) |
| `N` / `M` | Decrease / increase point size |
| `C` | Toggle colors (orthophoto / elevation gradient) |
| `P` | Toggle perspective / orthographic projection |
| `I` | Decrease point-collapsing angle (finer, more subdivision — DEM mesh only, triggers a reload) |
| `O` | Increase point-collapsing angle (coarser, more merging — DEM mesh only, triggers a reload). Repurposed from the old occlusion-culling toggle — occlusion culling is now implemented (Hi-Z depth pyramid) but has no key-based toggle; see Limitations |
| `V` | Side view |
| `T` | Top view |
| `B` | Toggle tile bounding-box wireframe overlay (streaming mode) |
| `L` | Toggle on-screen console log overlay |
| `H` | Show on-screen help (10 seconds) |
| `ESC` | Quit |

Run `./lasviewer -h` for command-line help.

---

## Sample Data

The viewer works with standard LiDAR and raster formats. Here are public data sources for testing:

### French LiDAR (LHD / IGN RGE Alti)

| Dataset | Format | Source |
|---------|--------|--------|
| **LHD COPC** | `.copc.laz` (classified LiDAR, EPSG:2154) | [IGN LiDAR HD](https://ignf.github.io/ignf-lidar-hd/) |
| **Orthophoto** | `.tif` (GeoTIFF, JPEG-compressed, EPSG:2154) | [IGN Orthophotos](https://geoservices.ign.fr/) |
| **RGE Alti DEM** | `.tif` (GeoTIFF, 1m resolution, EPSG:2154) | [IGN RGE Alti](https://ign.fr/rgealti) |

Example file from this project's testing:
```
LHD_FXX_1010_6549_PTS_C_LAMB93_IGN69.copc.laz   (19.2M points, 1km × 1km, EPSG:2154)
monzone.tif                                       (20000 × 20000, 0.2m/px, EPSG:2154)
```

### International LiDAR

| Source | Region | Format |
|--------|--------|--------|
| [USGS 3DEP](https://registry.opendata.aws/usgs-lidar/) | USA | `.laz`, COPC |
| [OpenTopography](https://opentopography.org/) | Global | `.laz`, `.las` |
| [ARTIIGN BD Ortho](https://geoservices.ign.fr/) | France | GeoTIFF orthophotos |
| [Copernicus DEM](https://spacedata.copernicus.eu/) | Global | GeoTIFF DEM (GLO-30) |

### Quick test with sample data

```bash
# Download a small sample (if you have curl + wget)
# Replace URL with your data source
wget https://example.com/sample.copc.laz
wget https://example.com/sample_ortho.tif

# View
./lasviewer sample.copc.laz sample_ortho.tif
```

---

## Build System

The project uses a single `Makefile` with multi-OS detection and multiple targets:

```bash
make            # Build the viewer (default)
make build      # Same as above
make debug      # Compile with -g instead of -O2 → lasviewer-debug
make test       # Compile + run unit tests (no external deps needed)
make lint       # Static analysis (clang-tidy or cppcheck)
make fmt        # Format source code with clang-format
make dist       # Create source tarball (lasviewer.tar.gz)
make doc        # Generate Doxygen HTML documentation
make clean      # Remove all build artifacts
make help       # Show all available targets
make run ARGS="cloud.laz"  # Build + run with arguments
```

### Running tests

```bash
make test
```

Output:
```
[test] compiling tests/test_basic.cpp...
[test] running unit tests...
  [RUN ] test_coord_transform
  [PASS] test_coord_transform
  [RUN ] test_affine_inverse
  [PASS] test_affine_inverse
  ...
=== lasviewer unit tests ===
Ran 9 tests, 9 passed, 0 failed.
All tests passed!
```

Tests are self-contained (no GLFW/PDAL/libtiff needed) — they verify the core math formulas: coordinate transforms, GeoTIFF affine inverse, spatial subsampling, depth-based subsampling, Morton codes, near/far computation, elevation colors, and tile LOD resolution.

---

## Project Structure

```
lasviewer/
├── Makefile              # Multi-OS build system (build/test/lint/fmt/dist/doc/clean)
├── main.cpp              # Orchestration layer: argv, GLFW/GL setup, render loop
├── src/
│   ├── camera.h/.cpp          # Orbit camera, projection, dynamic near/far
│   ├── point_cloud.h/.cpp     # LAZ/LAS load (PDAL), spatial subsampling, GPU upload
│   ├── geotiff.h/.cpp         # GeoTIFF tags, .tfw, TIFF load, ortho colorization
│   ├── copc_streamer.h/.cpp   # Async COPC tile streaming, PCA/OBB LOD, eviction
│   ├── dem_mesh.h/.cpp        # Adaptive quadtree DEM/DSM mesh + texturing
│   ├── shaders.h/.cpp         # Embedded GLSL sources + compile/link helpers
│   ├── text_renderer.h/.cpp   # FreeType glyph atlas + text rendering
│   └── gl_app.h/.cpp          # GLFW callbacks, InputState, console log capture
├── README.md             # This file
├── LICENSE.md            # BSD 3-Clause license
├── specs.md              # Full numbered specification
├── .clang-format         # Code formatting config
├── .clang-tidy           # Static analysis config
├── tests/
│   └── test_basic.cpp    # 9 unit tests
└── docs/
    └── Doxyfile          # Doxygen configuration
```

---

## Technical Details

### Coordinate System

The viewer works in the data's native CRS (e.g., Lambert-93 / EPSG:2154 for French data). The world-to-GL transform is:

```
GL_X = (worldX - center.x) / scale      ← easting
GL_Y = (worldZ - center.z) / scale      ← elevation (up)
GL_Z = -(worldY - center.y) / scale     ← northing (negated for north-up)
```

Z is always at true 1:1 metric scale (1 unit XY = 1 unit Z). Press `E`/`D` to exaggerate for inspection.

### Point Cloud Subsampling

1. **Spatial grid subsampling** (at load time): clouds >2M points are subsampled using a uniform XY grid — one point per cell, where `cellSize = sqrt(area / 2M)`. This guarantees uniform spatial coverage regardless of input point order.

2. **GPU depth-based subsampling** (at render time): the vertex shader stochastically discards points based on their depth from the camera — closer points are kept at higher density, farther points are subsampled more aggressively. This gives uniform screen-space density with no cell boundaries.

### Streaming COPC

For `.copc.laz` files, the viewer uses async tile loading:
- 8×8 grid of spatial tiles (64 tiles)
- Coarse pass: all tiles loaded at a resolution targeting ~500 points/tile, for a fast first render
- Refinement: per-tile PCA gives an oriented bounding box; its projected on-screen area (toward the camera) drives the target resolution — this accounts for tile shape/orientation, not just distance to center
- LRU eviction: tiles not used in 10 seconds are unloaded
- Occlusion culling for streamed point-cloud tiles: implemented via a Hi-Z (max-mip depth pyramid) built from each frame's own depth, used on the next frame to skip tiles fully behind closer geometry — see Limitations for the one-frame-stale tradeoff and why (this replaced the disabled query-based scaffolding, which caused flickering from occlusion-query latency)

### DEM / DSM Terrain Mesh

The alternative path for GeoTIFF elevation rasters (see `-d dem.tif`): instead of converting pixels to points, the raster is triangulated into an adaptive mesh:
- Quadtree subdivision driven by geometric error (vs. bilinear interpolation) and texture footprint, up to 6 levels deep
- A balance pass guarantees no two adjacent triangulated cells differ by more than one subdivision level
- Edge midpoints are welded between neighboring cells to avoid T-junction cracks
- Textured from an orthophoto — a genuine CRS mismatch between DEM and orthophoto is detected (both files' EPSG codes are read and compared) and auto-reprojected via PROJ if available, otherwise reported and texturing skipped rather than stretched. Stretch-fit is still used only when the orthophoto has no geo tags at all. Elevation ramp (dark blue-violet → teal → yellow) otherwise
- Supports standard numeric rasters (Float32/64, Int16/32, UInt16, Byte) and IGN's Terrain RGB 8-bit encoding

### Orthophoto Colorization

Each LiDAR point's world (X, Y) is inverse-transformed through the GeoTIFF affine to find the corresponding pixel in the orthophoto, then bilinearly sampled. Points outside the orthophoto extent are colored mid-dark gray. The orthophoto must cover at least 25% of the point cloud's area, or loading fails with an error. Press `C` to toggle between orthophoto and elevation-gradient colors.

---

## Limitations

- Single file at a time (no multi-file mosaics yet)
- No measurement tools (distance, area, profile)
- No export (screenshots only)
- OpenGL 3.3 minimum (no Vulkan/Metal backend)
- Occlusion culling for streaming tiles is implemented via a Hi-Z (max-mip depth pyramid) mechanism, not the query-based scaffolding (GL query objects still allocated but unused — kept as documented scaffolding). Deliberately one-frame-stale: each frame's rendered depth is reduced into a conservative max-mip pyramid, and a small (~64×64) coarse mip is read back once per frame (not per tile) to decide next frame's tile visibility. Since this app's DEM and point-cloud-streaming paths are mutually exclusive, tiles can only occlude *other tiles* here, not DEM geometry. No key currently toggles this (`O` is repurposed to the DEM mesh's point-collapsing angle) — it's controlled only by `InputState::useOcclusion`'s default (`true`). Unverified on real hardware — the query-based predecessor's flicker was caused by unbounded query-result latency, which this design avoids structurally (a fixed one-frame offset instead), but the actual visual/performance behavior hasn't been checked on a real GPU.

---

## License

BSD 3-Clause License. See [LICENSE.md](LICENSE.md) for details.
