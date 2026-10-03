#!/bin/sh
# ci.sh — what CI runs (.github/workflows/ci.yml), and what you can run on a
# fresh Debian/Ubuntu machine or container to reproduce it:
#
#   scripts/ci.sh deps     install the build, test and lint prerequisites (apt, root)
#   scripts/ci.sh build    fetch bmake-it at the pinned commit, bmake, bmake test
#   scripts/ci.sh lint     bmake lint with clang-format/clang-tidy 22 (pip)
#
# BMAKE_IT_REF pins bmake-it (a commit of github.com/SylvainSouche/bmake-it);
# WORK is where bmake-it and the lint tools go (default: ./.ci, git-ignored).
set -eu
cd "$(dirname "$0")/.."

BMAKE_IT_REF=${BMAKE_IT_REF:-943b3b2}
WORK=${WORK:-$PWD/.ci}

case "${1:-}" in
deps)
    # The prerequisites listed in README → Prerequisites (Debian/Ubuntu
    # column), plus the test (atf, kyua) and CI tools.
    export DEBIAN_FRONTEND=noninteractive
    apt-get update -qq
    apt-get install -y -qq --no-install-recommends \
        bmake make clang libomp-dev cmake pkg-config git curl ca-certificates \
        libgdal-dev libglfw3-dev libglm-dev libgl-dev \
        libatf-dev atf-sh kyua python3 python3-venv
    ;;
build)
    mkdir -p "$WORK"
    if [ ! -d "$WORK/bmake-it/.git" ]; then
        git clone -q https://github.com/SylvainSouche/bmake-it.git "$WORK/bmake-it"
    fi
    git -C "$WORK/bmake-it" fetch -q origin
    git -C "$WORK/bmake-it" checkout -q "$BMAKE_IT_REF"
    export MAKESYSPATH="$WORK/bmake-it/mk:/usr/share/mk"
    if [ "$(uname -s)" = Linux ]; then
        # Workarounds for bmake-it 943b3b2 on Linux, to remove once it is
        # fixed (specs.md §12.2c):
        # - its non-macOS FETCH_BUILD prologue lacks a ";" before the next
        #   command (laz-perf, copc-lib fail with a dash syntax error);
        # - an import whose headers are in /usr/include gets no include
        #   directory from `pkg-config --cflags` (glm, GLFW not staged);
        # - staging a versioned .so whose files are symlinks loops them, so
        #   GLFW is given as a prefix holding real copies.
        glfwPrefix="$WORK/glfw-prefix"
        rm -rf "$glfwPrefix"
        mkdir -p "$glfwPrefix/include" "$glfwPrefix/lib"
        cp -R /usr/include/GLFW "$glfwPrefix/include/"
        cp -L "$(pkg-config --variable=libdir glfw3)/libglfw.so" "$glfwPrefix/lib/"
        export GLFW_PREFIX="$glfwPrefix" GLM_CFLAGS=-I/usr/include
        set -- "_FETCH_DEPLOY_EXPORT=:;"
    else
        set --
    fi
    bmake "$@"
    bmake test "$@"
    ;;
lint)
    if [ ! -x "$WORK/venv/bin/clang-format" ]; then
        python3 -m venv "$WORK/venv"
        "$WORK/venv/bin/pip" install -q clang-format==22.1.8 clang-tidy==22.1.8
    fi
    export MAKESYSPATH="$WORK/bmake-it/mk:/usr/share/mk"
    CLANG_FORMAT="$WORK/venv/bin/clang-format" CLANG_TIDY="$WORK/venv/bin/clang-tidy" bmake lint
    ;;
*)
    echo "usage: scripts/ci.sh deps|build|lint" >&2
    exit 2
    ;;
esac
