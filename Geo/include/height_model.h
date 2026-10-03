// height_model.h — combining a ground model (DTM, IGN "MNT") with what stands
// on it: a height model (DHM, IGN "MNH": height above ground) or a surface
// model (DSM, IGN "MNS": absolute elevation of the top surface).
//
// All rasters are DemRasters already in the scene CRS and north-up (loadDEM
// guarantees both), so combining them is plain resampling, no reprojection.
#pragma once
#include "raster.h"

#include <vector>

// How to read the raster laid over the ground.
enum class AboveGroundKind {
    Height,  // DHM: values are heights above the ground
    Surface, // DSM: values are absolute elevations
};

// Bilinear resampling of `src` at the pixel centers of a width x height grid
// with affine `grid`. Nodata neighbors are left out of the weights; a sample
// with no valid neighbor, or more than half a pixel outside `src`, is NaN.
std::vector<float> resampleOnGrid(const DemRaster& src, const RasterGeo& grid, int width,
                                  int height);

// Height above ground on `ground`'s grid (NaN where unknown): the DHM value,
// or DSM - DTM.
std::vector<float> heightAboveGround(const DemRaster& ground, const DemRaster& above,
                                     AboveGroundKind kind);

// The top surface on `above`'s own grid:
//   out          absolute elevations: ground + max(height, 0), so the ground
//                itself where nothing stands on it; nodata (-9999) where the
//                ground or the height is unknown;
//   outGround    the ground elevation on the same grid (NaN where unknown),
//                so a renderer can cut the surface at any height above it
//                (objects keep their sides down to the cut).
// Returns false if the two rasters do not overlap at all.
bool composeAboveGround(const DemRaster& ground, const DemRaster& above, AboveGroundKind kind,
                        DemRaster& out, std::vector<float>& outGround);
