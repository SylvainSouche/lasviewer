// las_format.h — LAS point-record decoding and CRS lookup, shared by the
// full-file reader (point_cloud.cpp, laz-perf) and the COPC streamer
// (copc-lib). Both libraries hand back standard, uncompressed LAS records.
#pragma once
#include <glm/glm.hpp>
#include <cstdint>
#include <cstring>
#include <string>

// Where the fields of one LAS point record are (LAS 1.0-1.4, formats 0-10).
struct LasRecordLayout {
    int format = 0;          // point data format id (compression bits masked)
    int recordLength = 0;    // bytes per record, extra bytes included
    int rgbOffset = -1;      // byte offset of Red, Green, Blue (uint16 each); -1 if none
    glm::dvec3 scale{1.0}, offset{0.0};

    bool hasRGB() const { return rgbOffset >= 0; }

    // X, Y, Z are the first three int32 fields in every format.
    void xyz(const char* rec, double& x, double& y, double& z) const {
        int32_t v[3];
        std::memcpy(v, rec, sizeof(v)); // LAS is little-endian, as are all supported hosts
        x = v[0] * scale.x + offset.x;
        y = v[1] * scale.y + offset.y;
        z = v[2] * scale.z + offset.z;
    }
    // Red, Green, Blue as stored (0-65535).
    void rgb(const char* rec, uint16_t out[3]) const {
        std::memcpy(out, rec + rgbOffset, 3 * sizeof(uint16_t));
    }
};

// Layout for a header's point format id (bits 6-7, used by LAZ, are masked)
// and record length. Returns false for an unknown format.
bool lasRecordLayout(int formatId, int recordLength, const glm::dvec3& scale,
                     const glm::dvec3& offset, LasRecordLayout& out);

// The file's CRS as WKT, from its VLRs or extended VLRs: the OGC WKT record
// (LASF_Projection 2112, LAS 1.4), else the GeoTIFF key directory (34735,
// LAS <= 1.3), converted to WKT from its EPSG code through GDAL. Empty if the
// file declares none.
std::string lasCrsWkt(const std::string& path);
