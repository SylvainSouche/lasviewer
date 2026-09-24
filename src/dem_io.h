// dem_io.h — raw DEM raster reading shared by the DEM mesh builders.
#pragma once
#include <cstdint>
#include <tiffio.h>
#include <vector>

// Reads the first band of a (stripped or tiled) TIFF as float elevations,
// row-major, w*h values. Handles UInt8/16/32, Int16/32, Float32/64, and the
// IGN Terrain RGB encoding (8-bit RGB: h = (R*65536 + G*256 + B)*0.1 - 10000).
// Never uses TIFFReadRGBAImage, which would corrupt numeric samples.
bool readDEMElevations(TIFF* tif, uint32_t w, uint32_t h,
                       std::vector<float>& out, uint16_t& outSpp);

// Declared nodata value, if any. Currently always false (see dem_io.cpp);
// callers then treat values < -9000 as nodata.
bool readDEMNodataValue(TIFF* tif, float& outNodata);
