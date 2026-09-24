// geotiff.h — GeoTIFF tag reading, .tfw parsing, TIFF loading, orthophoto
#pragma once
#include <glm/glm.hpp>
#include <string>
#include <vector>
#include <cstdint>
#include <tiffio.h>

struct PointCloud; // forward decl from point_cloud.h
struct WorldBounds; // scene_frame.h

struct Orthophoto {
    int width = 0, height = 0;
    std::vector<uint8_t> pixels;
    double A = 1.0, D = 0.0, B = 0.0, E = -1.0, C = 0.0, F = 0.0;
    bool hasGeo = false;
    // EPSG code for this file's CRS (0 = unknown/not found — either the
    // file has no GeoKeyDirectoryTag, or it uses a custom, non-EPSG-
    // catalogued CRS definition that readEPSGCode() doesn't attempt to
    // resolve). Used to detect a CRS MISMATCH between the DEM and an
    // orthophoto (both reuse this same struct type — see loadFromDEM()'s
    // local `geo` variable in dem_tess_mesh.cpp) and, when
    // PROJ is available, to reproject one into the other's CRS.
    int epsg = 0;
};

// Reads the GeoKeyDirectoryTag (34735) and extracts the EPSG code for the
// file's CRS — ProjectedCSTypeGeoKey for a projected CRS,
// GeographicTypeGeoKey for a geographic one. Only resolves the common
// case of a standard, EPSG-catalogued CRS referenced directly by code
// (TIFFTagLocation == 0, a SHORT value stored inline) — does not attempt
// to resolve a custom, hand-defined CRS (which would additionally need
// GeoDoubleParamsTag/GeoAsciiParamsTag parsing), since the vast majority
// of real-world GeoTIFFs reference a standard EPSG code directly. Returns
// false (0) if no usable code was found.
bool readEPSGCode(TIFF* tif, int& outEpsg);

// If `ortho` and `demGeo` both have a known, DIFFERING EPSG code, and
// PROJ is available (LASVIEWER_HAS_PROJ — see the Makefile), reprojects
// `ortho`'s affine transform (A/C/E/F) so it's expressed in `demGeo`'s
// CRS instead of its own: transforms ortho's 4 corner points via PROJ,
// then re-fits a still axis-aligned (no rotation/shear — matching this
// codebase's existing affine model, which never supported those even
// before this) affine to the reprojected results. This is a local linear
// approximation of what may be, for very different CRS pairs, a
// genuinely non-linear transform — expected to be accurate enough for a
// single orthophoto tile's extent, not survey-grade for arbitrarily large
// areas. Returns true if a reprojection was actually performed (ortho's
// A/C/E/F/epsg modified in place); false if none was needed (same CRS,
// or either unknown) or possible (PROJ unavailable, or PROJ found no
// valid transform pipeline for this EPSG pair — logs why either way).
bool reprojectToMatchCRS(Orthophoto& ortho, const Orthophoto& demGeo);

// Read GeoTIFF ModelPixelScaleTag + ModelTiepointTag.
bool readGeoTIFFTags(TIFF* tif, Orthophoto& ortho);

// Load a .tfw world file.
bool loadTFW(const std::string& path, Orthophoto& ortho);

// Load a TIFF orthophoto (handles tiled/striped, JPEG/LZW, downsamples).
bool loadTIFF(const std::string& path, Orthophoto& ortho, int maxPixels);

// loadTIFF + ".tfw" sidecar fallback when the file has no GeoTIFF tags.
bool loadOrthophoto(const std::string& path, Orthophoto& ortho, int maxPixels);

// Georeferenced XY extent (pixel centers) and EPSG code of a raster, from its
// tags only. Returns false if unreadable or not georeferenced.
bool readRasterExtent(const std::string& path, WorldBounds& out, int& epsg);

// True if the TIFF looks like imagery (8-bit, ≥3 bands) rather than an
// elevation raster. Used to tell an orthophoto from a DEM on the command
// line; Terrain-RGB DEMs are 8-bit RGB too and must be passed with -d.
bool tiffLooksLikeImage(const std::string& path);

// XY extent of a georeferenced orthophoto (pixel centers).
void orthoExtent(const Orthophoto& o, double& minX, double& minY, double& maxX, double& maxY);

// Bilinear sample the orthophoto at fractional pixel coords.
glm::vec3 sampleOrthoBilinear(const Orthophoto& ortho, double col, double row);

// Colorize point cloud from orthophoto (CPU, OpenMP-parallel).
void colorizeFromOrthophoto(PointCloud& cloud, const Orthophoto& ortho);
