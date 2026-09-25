// raster.h — raster I/O through GDAL: orthophotos, DEMs, extents, CRS.
//
// Every raster is delivered north-up in the scene's CRS: a raster in another
// horizontal CRS, or on a rotated grid, is warped on load (GDAL, bilinear),
// so callers never deal with reprojection or rotation. Georeferencing comes
// from whatever GDAL finds (GeoTIFF tags, .tfw/.wld world files, other
// formats).
#pragma once
#include <glm/glm.hpp>
#include <cstdint>
#include <string>
#include <vector>

struct PointCloud;  // point_cloud.h
struct WorldBounds; // scene_frame.h

// A georeferenced raster's affine, pixel-CENTER convention:
//   worldX = C + A*col + B*row
//   worldY = F + D*col + E*row
// where (col, row) = (0, 0) is the center of the top-left pixel.
struct RasterGeo {
    double A = 1.0, D = 0.0, B = 0.0, E = -1.0, C = 0.0, F = 0.0;
    bool hasGeo = false;
    int epsg = 0;       // horizontal EPSG code, 0 = unknown / not EPSG
    std::string wkt;    // CRS as WKT, empty = unknown
};

// An RGBA8 image, usually an orthophoto.
struct Orthophoto : RasterGeo {
    int width = 0, height = 0;
    std::vector<uint8_t> pixels; // width*height*4, row 0 = top
    bool reprojected = false;    // warped on load (other CRS, or rotated grid)
};

// Elevation raster (band 1, or decoded Terrain RGB), in the scene CRS.
struct DemRaster : RasterGeo {
    int width = 0, height = 0;
    std::vector<float> elevations; // width*height, row 0 = top
    bool hasNodata = false;        // declared by the file (or set by the warp)
    float nodata = 0.0f;
    bool terrainRGB = false;
    bool reprojected = false;
    // True for a nodata sample: the declared value if any, else < -9000
    // (common undeclared sentinels such as -9999).
    bool isNodata(float v) const;
};

// Registers GDAL drivers once; safe to call repeatedly and from any thread.
void rasterInit();

// Header-only information: size, georeferencing, CRS.
struct RasterInfo : RasterGeo {
    int width = 0, height = 0, bands = 0;
    bool byteImage = false; // 8-bit with >= 3 bands (imagery, or Terrain RGB)
};
bool readRasterInfo(const std::string& path, RasterInfo& out);

// XY extent (pixel centers) of a georeferenced raster, in its own CRS.
void rasterExtent(const RasterGeo& g, int width, int height,
                  double& minX, double& minY, double& maxX, double& maxY);

// Reprojects an XY extent between two CRSs (dense edge sampling). Returns
// false if either CRS is unknown or the transform fails.
bool transformExtent(WorldBounds& b, const std::string& fromWkt, const std::string& toWkt);

// True when both CRSs are known and describe the same horizontal system
// (vertical components ignored, e.g. EPSG:2154 vs EPSG:2154+5720).
bool sameHorizontalCRS(const std::string& wktA, const std::string& wktB);

// Horizontal EPSG code of a WKT CRS, 0 if not identifiable.
int horizontalEPSG(const std::string& wkt);

// Loads an image as RGBA8, downsampled (GDAL averaging, using overviews when
// present) so that width*height <= maxPixels. If sceneWkt is set and the
// image has a different known CRS, it is warped into sceneWkt.
bool loadOrthophoto(const std::string& path, Orthophoto& ortho, int maxPixels,
                    const std::string& sceneWkt);

// Loads a DEM (band 1 as float, or Terrain RGB when the file is 8-bit with
// >= 3 bands). Warped into sceneWkt when its CRS differs.
bool loadDEM(const std::string& path, DemRaster& dem, const std::string& sceneWkt);

// XY extent of a georeferenced orthophoto (pixel centers).
void orthoExtent(const Orthophoto& o, double& minX, double& minY, double& maxX, double& maxY);

// Bilinear sample at fractional pixel (col, row); mid-grey outside.
glm::vec3 sampleOrthoBilinear(const Orthophoto& ortho, double col, double row);

// Colors every point of the cloud from the orthophoto (OpenMP-parallel).
void colorizeFromOrthophoto(PointCloud& cloud, const Orthophoto& ortho);
