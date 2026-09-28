# copc-lib (Rock Robotic, BSD): direct access to a COPC file's octree.
# Fetched from its release tag and built with its own CMake; static library.
# It needs laz-perf: the release archive doesn't include its bundled copy
# (a git submodule), so CMake is pointed at liblazperf.m's install prefix.
# (bmake-it doesn't pass a dependency's prefix to an upstream build yet.)
# LIBS=lazperf makes liblazperf.m build first.
LIB=copc-lib
IMPORT=fetch:copc-lib
FETCH_URL=https://github.com/RockRobotic/copc-lib/archive/refs/tags/v2.6.3.tar.gz
FETCH_BUILD=cmake
FETCH_BUILD_ARGS=-DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF -DWITH_TESTS=OFF \
    -DWITH_PYTHON=OFF -DCMAKE_PREFIX_PATH=${.CURDIR}/../liblazperf.m/work/_install
IMPORT_HEADERS=copc-lib
LIBS=lazperf
WARN=none
.include "mk.lib.mk"
