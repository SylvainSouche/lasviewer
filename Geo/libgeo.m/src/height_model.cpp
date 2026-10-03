// height_model.cpp — see height_model.h.
#include "height_model.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kNodata = -9999.0f;

} // namespace

std::vector<float> resampleOnGrid(const DemRaster& src, const RasterGeo& grid, int width,
                                  int height) {
    std::vector<float> out(static_cast<size_t>(width) * height, kNaN);
    if (src.width <= 0 || src.height <= 0 || src.A == 0.0 || src.E == 0.0) return out;
    const int sw = src.width, sh = src.height;
#pragma omp parallel for schedule(static)
    for (int r = 0; r < height; ++r) {
        double y = grid.F + grid.E * r;
        double row = (y - src.F) / src.E;
        if (row < -0.5 || row > sh - 0.5) continue;
        for (int c = 0; c < width; ++c) {
            double x = grid.C + grid.A * c;
            double col = (x - src.C) / src.A;
            if (col < -0.5 || col > sw - 0.5) continue;
            double cc = std::clamp(col, 0.0, static_cast<double>(sw - 1));
            double rr = std::clamp(row, 0.0, static_cast<double>(sh - 1));
            int c0 = static_cast<int>(cc), r0 = static_cast<int>(rr);
            int c1 = std::min(c0 + 1, sw - 1), r1 = std::min(r0 + 1, sh - 1);
            double fx = cc - c0, fy = rr - r0;
            const int cs[4] = {c0, c1, c0, c1};
            const int rs[4] = {r0, r0, r1, r1};
            const double ws[4] = {(1 - fx) * (1 - fy), fx * (1 - fy), (1 - fx) * fy, fx * fy};
            double sum = 0.0, wsum = 0.0;
            for (int k = 0; k < 4; ++k) {
                float v = src.elevations[static_cast<size_t>(rs[k]) * sw + cs[k]];
                if (src.isNodata(v) || ws[k] <= 0.0) continue;
                sum += v * ws[k];
                wsum += ws[k];
            }
            if (wsum > 0.0)
                out[static_cast<size_t>(r) * width + c] = static_cast<float>(sum / wsum);
        }
    }
    return out;
}

std::vector<float> heightAboveGround(const DemRaster& ground, const DemRaster& above,
                                     AboveGroundKind kind) {
    std::vector<float> h = resampleOnGrid(above, ground, ground.width, ground.height);
    if (kind == AboveGroundKind::Surface) {
        for (size_t i = 0; i < h.size(); ++i) {
            float g = ground.elevations[i];
            h[i] = ground.isNodata(g) ? kNaN : h[i] - g; // NaN stays NaN
        }
    }
    return h;
}

bool composeAboveGround(const DemRaster& ground, const DemRaster& above, AboveGroundKind kind,
                        DemRaster& out, std::vector<float>& outGround) {
    const int w = above.width, h = above.height;
    outGround = resampleOnGrid(ground, above, w, h);

    // Height above ground per pixel (NaN = unknown).
    std::vector<float> height(outGround.size(), kNaN);
    bool any = false;
    for (size_t i = 0; i < height.size(); ++i) {
        float v = above.elevations[i];
        float g = outGround[i];
        if (above.isNodata(v) || std::isnan(g)) continue;
        height[i] = (kind == AboveGroundKind::Height) ? v : v - g;
        any = true;
    }

    static_cast<RasterGeo&>(out) = above;
    out.width = w;
    out.height = h;
    out.hasNodata = true;
    out.nodata = kNodata;
    out.terrainRGB = false;
    out.reprojected = above.reprojected;
    out.elevations.assign(height.size(), kNodata);
    for (size_t i = 0; i < height.size(); ++i) {
        // Negative heights (noise under the ground) are clamped onto it.
        if (!std::isnan(height[i])) out.elevations[i] = outGround[i] + std::max(height[i], 0.0f);
    }
    return any;
}
