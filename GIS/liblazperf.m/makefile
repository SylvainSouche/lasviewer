# laz-perf (Hobu, Apache-2.0): LAS/LAZ reading and LAZ decompression.
# Fetched from its release tag and built with its own CMake. Its CMake
# installs only the shared library (the static one isn't installed).
LIB=lazperf
IMPORT=fetch:laz-perf
FETCH_URL=https://github.com/hobuinc/laz-perf/archive/refs/tags/3.4.0.tar.gz
FETCH_BUILD=cmake
REQUIRES=cmake
FETCH_BUILD_ARGS=-DCMAKE_BUILD_TYPE=Release -DWITH_TESTS=OFF
IMPORT_HEADERS=lazperf
WARN=none
.include "mk.lib.mk"
