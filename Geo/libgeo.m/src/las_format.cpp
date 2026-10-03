// las_format.cpp — see las_format.h.
#include "las_format.h"

#include "raster.h"

#include <cpl_conv.h>
#include <ogr_spatialref.h>

#include <fstream>
#include <vector>

bool lasRecordLayout(int formatId, int recordLength, const glm::dvec3& scale,
                     const glm::dvec3& offset, LasRecordLayout& out) {
    // Byte offset of RGB per point data format (LAS 1.4 R15, section 2.6);
    // -1 = the format has no RGB.
    static const int kRgbOffset[11] = {-1, -1, 20, 28, -1, 28, -1, 30, 30, -1, 30};
    static const int kMinLength[11] = {20, 28, 26, 34, 57, 63, 30, 36, 38, 59, 67};
    int format = formatId & 0x3F; // LAZ sets bits 6-7 on compressed files
    if (format < 0 || format > 10 || recordLength < kMinLength[format]) return false;
    out.format = format;
    out.recordLength = recordLength;
    out.rgbOffset = kRgbOffset[format];
    out.scale = scale;
    out.offset = offset;
    return true;
}

namespace {

template <typename T> bool readAt(std::ifstream& f, uint64_t pos, T& value) {
    f.seekg(static_cast<std::streamoff>(pos));
    f.read(reinterpret_cast<char*>(&value), sizeof(T));
    return static_cast<bool>(f);
}

struct Record {
    std::string userId;
    uint16_t recordId = 0;
    std::vector<char> data;
};

// VLRs (54-byte headers, after the file header) and, for LAS 1.4, EVLRs
// (60-byte headers, after the point data).
std::vector<Record> readVariableRecords(const std::string& path) {
    std::vector<Record> out;
    std::ifstream f(path, std::ios::binary);
    char magic[4] = {};
    if (!f.read(magic, 4) || std::memcmp(magic, "LASF", 4) != 0) return out;
    uint8_t minor = 0;
    uint16_t headerSize = 0;
    uint32_t vlrCount = 0;
    if (!readAt(f, 25, minor) || !readAt(f, 94, headerSize) || !readAt(f, 100, vlrCount))
        return out;

    auto readRecord = [&](uint64_t pos, bool extended, uint64_t& next) -> bool {
        char user[17] = {};
        uint16_t recordId = 0;
        uint64_t length = 0;
        f.seekg(static_cast<std::streamoff>(pos + 2));
        if (!f.read(user, 16)) return false;
        if (!readAt(f, pos + 18, recordId)) return false;
        uint64_t dataPos;
        if (extended) {
            if (!readAt(f, pos + 20, length)) return false;
            dataPos = pos + 60;
        } else {
            uint16_t len16 = 0;
            if (!readAt(f, pos + 20, len16)) return false;
            length = len16;
            dataPos = pos + 54;
        }
        Record r;
        r.userId = user;
        r.recordId = recordId;
        if ((r.userId == "LASF_Projection") && length < (64u << 20)) {
            r.data.resize(length);
            f.seekg(static_cast<std::streamoff>(dataPos));
            if (!f.read(r.data.data(), static_cast<std::streamsize>(length))) return false;
        }
        out.push_back(std::move(r));
        next = dataPos + length;
        return true;
    };

    uint64_t pos = headerSize;
    for (uint32_t i = 0; i < vlrCount; ++i) {
        uint64_t next = 0;
        if (!readRecord(pos, false, next)) break;
        pos = next;
    }
    if (minor >= 4 && headerSize >= 247) {
        uint64_t evlrOffset = 0;
        uint32_t evlrCount = 0;
        if (readAt(f, 235, evlrOffset) && readAt(f, 243, evlrCount) && evlrOffset > 0) {
            pos = evlrOffset;
            for (uint32_t i = 0; i < evlrCount; ++i) {
                uint64_t next = 0;
                if (!readRecord(pos, true, next)) break;
                pos = next;
            }
        }
    }
    return out;
}

// EPSG code from a GeoTIFF key directory (uint16 entries: version, revision,
// minor, key count, then key id / tag location / count / value per key).
int epsgFromGeoKeys(const std::vector<char>& data) {
    size_t n = data.size() / 2;
    std::vector<uint16_t> k(n);
    std::memcpy(k.data(), data.data(), n * 2);
    if (n < 4) return 0;
    size_t keys = k[3];
    int projected = 0, geographic = 0;
    for (size_t i = 0; i < keys && 4 + i * 4 + 3 < n; ++i) {
        uint16_t id = k[4 + i * 4], location = k[5 + i * 4], value = k[7 + i * 4];
        if (location != 0 || value == 0 || value == 32767) continue; // not an inline EPSG code
        if (id == 3072) projected = value;                           // ProjectedCSTypeGeoKey
        if (id == 2048) geographic = value;                          // GeographicTypeGeoKey
    }
    return projected ? projected : geographic;
}

} // namespace

std::string lasCrsWkt(const std::string& path) {
    std::vector<Record> records = readVariableRecords(path);
    for (const Record& r : records) {
        if (r.userId == "LASF_Projection" && r.recordId == 2112 && !r.data.empty())
            return std::string(r.data.data(), strnlen(r.data.data(), r.data.size()));
    }
    for (const Record& r : records) {
        if (r.userId == "LASF_Projection" && r.recordId == 34735) {
            int epsg = epsgFromGeoKeys(r.data);
            if (!epsg) break;
            rasterInit();
            OGRSpatialReference srs;
            if (srs.importFromEPSG(epsg) != OGRERR_NONE) break;
            char* wkt = nullptr;
            srs.exportToWkt(&wkt);
            std::string s = wkt ? wkt : "";
            CPLFree(wkt);
            return s;
        }
    }
    return {};
}
