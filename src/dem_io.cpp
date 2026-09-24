// dem_io.cpp — raw DEM raster reading (libtiff).
#include "dem_io.h"

#include <algorithm>
#include <cstring>
#include <iostream>

// Declared nodata (GDAL_NODATA, tag 42113) is not read: libtiff crashed on
// this unregistered custom tag (type confusion in TIFFGetField), both with
// and without TIFFMergeFieldInfo registration — see design doc §6p. Callers
// fall back to the "< -9000" heuristic. Reading rasters through GDAL would
// remove this limitation.
bool readDEMNodataValue(TIFF* /*tif*/, float& /*outNodata*/) {
    return false;
}

bool readDEMElevations(TIFF* tif, uint32_t w, uint32_t h,
                       std::vector<float>& out,
                       uint16_t& outSpp) {
    uint16_t bps = 8, sf = SAMPLEFORMAT_UINT, spp = 1;
    TIFFGetFieldDefaulted(tif, TIFFTAG_BITSPERSAMPLE,  &bps);
    TIFFGetFieldDefaulted(tif, TIFFTAG_SAMPLEFORMAT,   &sf);
    TIFFGetFieldDefaulted(tif, TIFFTAG_SAMPLESPERPIXEL, &spp);

    std::cerr << "[dem] sample format: bps=" << bps << " sf=" << sf
              << " spp=" << spp << std::endl;

    out.resize(static_cast<size_t>(w) * h);

    // Determine the byte size of one sample.
    size_t bytesPerSample = bps / 8;
    if (bytesPerSample == 0) bytesPerSample = 1;

    // Detect tiled vs stripped.
    uint32_t tileW = 0, tileH = 0;
    bool isTiled = TIFFIsTiled(tif) != 0;
    if (isTiled) {
        TIFFGetField(tif, TIFFTAG_TILEWIDTH,  &tileW);
        TIFFGetField(tif, TIFFTAG_TILELENGTH, &tileH);
    }

    // Helper: interpret one raw sample as a float.
    auto interpretSample = [&](const uint8_t* raw) -> float {
        if (sf == SAMPLEFORMAT_IEEEFP) {
            if (bytesPerSample == 4) {
                float v;
                std::memcpy(&v, raw, 4);
                return v;
            } else if (bytesPerSample == 8) {
                double v;
                std::memcpy(&v, raw, 8);
                return static_cast<float>(v);
            }
        }
        // Integer sample formats.
        int64_t iv = 0;
        bool isSigned = (sf == SAMPLEFORMAT_INT);
        // Read little-endian (libtiff's native byte order is preserved in
        // the scanline buffer; for cross-endian files libtiff handles it).
        for (size_t b = 0; b < bytesPerSample; ++b) {
            iv |= static_cast<int64_t>(raw[b]) << (8 * b);
        }
        if (isSigned && bytesPerSample < 8) {
            // Sign-extend.
            int64_t signBit = static_cast<int64_t>(1) << (8 * bytesPerSample - 1);
            if (iv & signBit) iv |= ~((static_cast<int64_t>(1) << (8 * bytesPerSample)) - 1);
        }
        return static_cast<float>(iv);
    };

    // For multi-band 8-bit TIFFs (spp >= 3, bps=8), the elevation may be
    // encoded as Terrain RGB: height = (R*65536 + G*256 + B) * 0.1 - 10000.
    // This is the standard IGN MNS LiDAR HD encoding.
    bool isTerrainRGB = (spp >= 3 && bps == 8 && sf == SAMPLEFORMAT_UINT);
    if (isTerrainRGB) {
        std::cerr << "[dem] Terrain RGB encoding detected (spp=" << spp
                  << ", bps=8). Decoding: h = (R*65536+G*256+B)*0.1 - 10000" << std::endl;
    }

    auto decodePixel = [&](const uint8_t* p) -> float {
        if (isTerrainRGB) {
            // Read R, G, B bands (bands are interleaved: RGBRGB...)
            int r = p[0];
            int g = p[1];
            int b = p[2];
            // Terrain RGB: height = (R*65536 + G*256 + B) * 0.1 - 10000
            float h = (static_cast<float>(r) * 65536.0f +
                       static_cast<float>(g) * 256.0f +
                       static_cast<float>(b)) * 0.1f - 10000.0f;
            if (h < -9000.0f) h = 0.0f;  // nodata
            return h;
        }
        return interpretSample(p);
    };

    if (!isTiled) {
        // Stripped TIFF — read scanline by scanline.
        // TIFFScanlineSize returns bytes per scanline (across all samples).
        tmsize_t scanlineSize = TIFFScanlineSize(tif);
        if (scanlineSize <= 0) {
            std::cerr << "ERROR: TIFFScanlineSize returned " << scanlineSize << std::endl;
            return false;
        }
        std::vector<uint8_t> buf(static_cast<size_t>(scanlineSize));
        for (uint32_t row = 0; row < h; ++row) {
            if (TIFFReadScanline(tif, buf.data(), row, 0) < 0) {
                std::cerr << "ERROR: TIFFReadScanline failed at row " << row << std::endl;
                return false;
            }
            const uint8_t* p = buf.data();
            for (uint32_t col = 0; col < w; ++col) {
                out[static_cast<size_t>(row) * w + col] = decodePixel(p);
                p += bytesPerSample * spp;
            }
        }
        outSpp = spp;
        return true;
    }

    // Tiled TIFF — read tile by tile.
    if (tileW == 0 || tileH == 0) {
        std::cerr << "ERROR: invalid tile dimensions (" << tileW << "x" << tileH << ")" << std::endl;
        return false;
    }
    tmsize_t tileSize = TIFFTileSize(tif);
    if (tileSize <= 0) {
        std::cerr << "ERROR: TIFFTileSize returned " << tileSize << std::endl;
        return false;
    }
    std::vector<uint8_t> buf(static_cast<size_t>(tileSize));
    for (uint32_t ty = 0; ty < h; ty += tileH) {
        for (uint32_t tx = 0; tx < w; tx += tileW) {
            if (TIFFReadTile(tif, buf.data(), tx, ty, 0, 0) < 0) {
                std::cerr << "ERROR: TIFFReadTile failed at (" << tx << "," << ty << ")" << std::endl;
                return false;
            }
            uint32_t copyW = std::min(tileW, w - tx);
            uint32_t copyH = std::min(tileH, h - ty);
            for (uint32_t r = 0; r < copyH; ++r) {
                const uint8_t* rowPtr = buf.data() + (r * tileW * bytesPerSample * spp);
                for (uint32_t c = 0; c < copyW; ++c) {
                    out[static_cast<size_t>(ty + r) * w + (tx + c)] = decodePixel(rowPtr);
                    rowPtr += bytesPerSample * spp;
                }
            }
        }
    }
    outSpp = spp;
    return true;
}
