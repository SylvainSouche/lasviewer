#!/bin/sh
# lint.sh — check the project's own C++ sources: formatting (clang-format,
# .clang-format) and naming (clang-tidy, .clang-tidy). Exits non-zero on any
# deviation. Third-party code (fetched or imported) is not checked.
#
#   scripts/lint.sh          check
#   scripts/lint.sh --fix    reformat in place and apply naming fixes
#
# Needs a prior `bmake` (clang-tidy reads the headers bmake-it staged under
# GIS/build and GUI/build). Tools: CLANG_FORMAT / CLANG_TIDY override the
# defaults (clang-format, clang-tidy); CI pins both to the same version as
# the development machine (see .github/workflows/ci.yml).
set -eu
cd "$(dirname "$0")/.."

fix=no
[ "${1:-}" = --fix ] && fix=yes
cf=${CLANG_FORMAT:-clang-format}
ct=${CLANG_TIDY:-clang-tidy}

files=$(git ls-files -co --exclude-standard 'Geo/*.cpp' 'Geo/*.h' 'Viewer/*.cpp' 'Viewer/*.h')
headers=$(git ls-files -co --exclude-standard 'Geo/*.h' 'Viewer/*.h')
sources=$(git ls-files -co --exclude-standard 'Geo/*.cpp' 'Viewer/*.cpp')

# --- formatting
status=0
if [ $fix = yes ]; then
    $cf -i $files
else
    $cf --dry-run --Werror $files || status=1
fi

# --- naming: the bmake-it build key whose staged headers to use
key=$(ls GIS/build 2>/dev/null | head -1)
if [ -z "$key" ] || [ ! -d "GIS/build/$key/include" ]; then
    echo "lint.sh: run bmake first (no staged headers under GIS/build/<key>/include)" >&2
    exit 2
fi
# atf-c++ (tests) still uses std::auto_ptr, removed in C++17 (bmake-it adds the same define).
flags="-std=c++17 -D_LIBCPP_ENABLE_CXX17_REMOVED_AUTO_PTR -IGeo/include -IViewer/local/include
       -isystem GIS/build/$key/include -isystem GUI/build/$key/include"
for d in /opt/local/include /opt/local/include/libomp /opt/homebrew/include \
         /opt/homebrew/opt/libomp/include /usr/local/include /usr/include/gdal; do
    [ -d "$d" ] && flags="$flags -isystem $d"
done
echo "lint.sh: clang-tidy flags: $flags"
tidyFix=""
[ $fix = yes ] && tidyFix="--fix-errors"
# Headers are checked through the sources that include them
# (HeaderFilterRegex in .clang-tidy); tests are sources too.
for f in $sources; do
    $ct --quiet $tidyFix "$f" -- $flags 2>/dev/null || status=1
done
[ -n "$headers" ] # (listed for documentation; checked via includes)

[ $status = 0 ] && echo "lint.sh: OK"
exit $status
