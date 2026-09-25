# Makefile for lasviewer
# ----------------------
# Multi-OS, multi-target build system.
#
# Targets:
#   make            build the viewer (default = build)
#   make build      compile main.cpp → lasviewer
#   make debug      compile with -g instead of -O2 → lasviewer-debug
#   make test       compile + run unit tests
#   make lint       static analysis (clang-tidy / cppcheck)
#   make fmt        format source with clang-format
#   make dist       create source tarball
#   make doc        generate Doxygen documentation
#   make clean      remove all build artifacts
#   make run        build + run with ARGS="..."
#   make help       show available targets
#
# OS support:
#   macOS (MacPorts):  sudo port install glfw pdal gdal glm pkgconfig
#   macOS (Homebrew):  brew install glfw pdal gdal glm pkg-config
#   Linux (apt):       sudo apt install libglfw3-dev libpdal-dev libgdal-dev libglm-dev
#
# Dear ImGui is vendored in third_party/imgui (no install needed).

# ===========================================================================
# OS detection (lightweight — no errors here, deferred to build target)
# ===========================================================================

UNAME_S := $(shell uname -s)
UNAME_M := $(shell uname -m)

ifeq ($(UNAME_S),Darwin)
  OS := macos
else ifeq ($(UNAME_S),Linux)
  OS := linux
else
  OS := unknown
endif

# ===========================================================================
# Compiler settings (shared by all targets)
# ===========================================================================
#
# IMPORTANT: We use `override` because GNU Make pre-defines CXX to 'c++' from
# its built-in rules. `CXX ?= clang++` (conditional assignment) does NOT
# override Make's built-in default — it only fires if CXX is truly undefined.
# On macOS, Make's built-in CXX='c++' resolves to Apple's clang (/usr/bin/c++),
# which does NOT support OpenMP. We force the override here so the user can
# still pass CXX_OVERRIDE=g++ or CXX_OVERRIDE=clang-mp-19 on the command line.
#
# To use a different compiler:  make CXX_OVERRIDE=g++
# To use MacPorts clang:        make CXX_OVERRIDE=clang-mp-19
#
# NOTE: plain `make CXX=g++` will NOT work — the `override CXX := $(CXX_OVERRIDE)`
# below unconditionally replaces any command-line CXX= value. CXX_OVERRIDE is
# the only variable that actually controls the compiler.

ifndef CXX_OVERRIDE
  CXX_OVERRIDE := clang++
endif
override CXX := $(CXX_OVERRIDE)
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -pedantic

# --- OpenMP ---
# Apple's default clang (/usr/bin/clang++) does NOT support OpenMP.
# MacPorts clang (clang-mp-XX) and Homebrew llvm clang DO support it.
# g++ also supports it. We detect at build time: try compiling a tiny
# OpenMP test; if it fails, disable OpenMP automatically.
ifeq ($(OPENMP),0)
  CXXFLAGS += -DLASVIEWER_NO_OPENMP
else
  # Auto-detect: check if the compiler accepts -fopenmp
  OPENMP_TEST := $(shell echo 'int main(){return 0;}' | $(CXX) -fopenmp -x c++ - -o /dev/null 2>/dev/null && echo yes || echo no)
  ifeq ($(OPENMP_TEST),yes)
    CXXFLAGS += -fopenmp
    OMP_LDFLAGS := -fopenmp
  else
    CXXFLAGS += -DLASVIEWER_NO_OPENMP
    OMP_LDFLAGS :=
    $(info [config] OpenMP not supported by $(CXX) — building single-threaded. Use CXX_OVERRIDE=clang-mp-19 or CXX_OVERRIDE=g++ for OpenMP.)
  endif
endif

# --- GDAL ---
# All raster I/O (orthophotos, DEMs, CRS, reprojection) goes through GDAL,
# which PDAL already depends on.
GDAL_CFLAGS  := $(shell pkg-config --cflags gdal 2>/dev/null | sed 's/-I/-isystem /g')
GDAL_LDFLAGS := $(shell pkg-config --libs gdal 2>/dev/null)
ifeq ($(GDAL_LDFLAGS),)
  GDAL_LDFLAGS := -lgdal
endif

# ===========================================================================
# File paths
# ===========================================================================

TARGET    := lasviewer
# Same source tree as the bmake-it build (makefile); this file is kept only
# to compare against it until the GNU make build is retired.
SRC       := Viewer/lasviewer.m/src/main.cpp
SRC_DIR   := Viewer/lasviewer.m/src
GEO_DIR   := Geo/libgeo.m/src
IMGUI_DIR := ImGui/libimgui.m/src
IMGUI_SOURCES := $(addprefix $(IMGUI_DIR)/,imgui.cpp imgui_draw.cpp imgui_tables.cpp \
                 imgui_widgets.cpp imgui_demo.cpp imgui_impl_glfw.cpp imgui_impl_opengl3.cpp)
SOURCES   := $(wildcard $(SRC_DIR)/*.cpp) $(wildcard $(GEO_DIR)/*.cpp) $(IMGUI_SOURCES)
OBJ_DIR   := .build/obj
OBJECTS   := $(patsubst %.cpp,$(OBJ_DIR)/%.o,$(SOURCES))
TEST_SRC  := tests/test_basic.cpp
TEST_BIN  := .build/test_runner
SCENE_TEST_SRC := tests/test_scene.cpp $(SRC_DIR)/camera.cpp $(SRC_DIR)/camera_controller.cpp
SCENE_TEST_BIN := .build/test_scene
RASTER_TEST_SRC := tests/test_raster.cpp $(SRC_DIR)/raster.cpp
RASTER_TEST_BIN := .build/test_raster
# Header-only deps of the scene tests (glm, GLFW key constants).
TEST_INC  := $(firstword $(foreach d,/opt/local /opt/homebrew /usr/local /usr,$(if $(wildcard $(d)/include/glm/glm.hpp),$(d)/include)))
BUILD_DIR := .build

# ===========================================================================
# Phony targets
# ===========================================================================

.PHONY: all build debug test lint fmt dist doc clean help run

all: build

# ===========================================================================
# Build target (requires GLFW/PDAL/GDAL/glm)
# ===========================================================================

define detect_libs
  ifneq ($$(wildcard /opt/local/include/GLFW/glfw3.h),)
    PORTS := /opt/local
    PKG_MANAGER := macports
  else ifneq ($$(wildcard /opt/homebrew/include/GLFW/glfw3.h),)
    PORTS := /opt/homebrew
    PKG_MANAGER := homebrew
  else ifneq ($$(wildcard /usr/local/include/GLFW/glfw3.h),)
    PORTS := /usr/local
    PKG_MANAGER := homebrew-intel
  else ifneq ($$(wildcard /usr/include/GLFW/glfw3.h),)
    PORTS := /usr
    PKG_MANAGER := system
  else ifneq ($$(wildcard /usr/local/include/GLFW/glfw3.h),)
    PORTS := /usr/local
    PKG_MANAGER := system
  else
    $$(error "GLFW not found. Install: sudo port install glfw pdal gdal glm pkgconfig (MacPorts) OR brew install glfw pdal gdal glm pkg-config (Homebrew) OR sudo apt install libglfw3-dev libpdal-dev libgdal-dev libglm-dev (Linux)")
  endif
endef

build: $(TARGET)

# ===========================================================================
# Debug build — same flags as the default build (see CXXFLAGS above), with
# -O2 swapped for -g (debug symbols, no optimization — the implicit default
# when no -O flag is given). Uses a SEPARATE object directory and output
# binary name (.build/obj-debug, lasviewer-debug) rather than reusing the
# release build's — Make's dependency tracking is based on file mtimes, not
# on whether CXXFLAGS changed, so reusing the same .o files between `make
# build` and `make debug` could silently serve stale, wrongly-optimized (or
# wrongly-unoptimized) objects if a source file hadn't been touched since
# the other mode's last build. Implemented as a thin recursive-make wrapper
# around the existing `build` target rather than a parallel copy of it —
# CXXFLAGS/OBJ_DIR/TARGET are all plain `:=`/`?=` assignments (not
# `override`), so command-line values passed here correctly take priority,
# same mechanism CXX_OVERRIDE already relies on elsewhere in this file.
.PHONY: debug
debug:
	@$(MAKE) --no-print-directory build \
		CXXFLAGS="-std=c++17 -g -Wall -Wextra -pedantic" \
		OBJ_DIR=.build/obj-debug \
		TARGET=lasviewer-debug
	@echo "[debug] built: lasviewer-debug (run with lldb/gdb)"

# Per-source object compile rule — discovers PORTS/PKG_CONFIG flags once via
# the _detect_libs helper target and reuses the cached values via the
# .build/config.mk file. This keeps per-file compilation fast while still
# auto-detecting the system's library paths.
#
# Third-party include paths use -isystem, not -I: this suppresses warnings
# that originate INSIDE library headers (PDAL in particular is warning-
# heavy) from being reported as if they were this project's own — e.g. an
# "unused parameter" or "sign-compare" warning pointing into a PDAL/GLFW/
# glm header is not something we can or should fix here. Only our own
# code's include paths (-I. -I$(SRC_DIR)) stay as plain -I, so warnings in
# main.cpp/src/*.cpp still show up normally.
$(OBJ_DIR)/$(IMGUI_DIR)/%.o: $(IMGUI_DIR)/%.cpp | $(OBJ_DIR) .build/config.mk
	@echo "[cc] $<"
	@mkdir -p $(dir $@)
	@$(CXX) -std=c++17 $(if $(findstring -g,$(CXXFLAGS)),-g,-O2) -w -IImGui/include -isystem $(PORTS)/include \
	        $(if $(filter macos,$(OS)),-DGL_SILENCE_DEPRECATION,) -c $< -o $@

$(OBJ_DIR)/%.o: %.cpp | $(OBJ_DIR) .build/config.mk
	@echo "[cc] $<"
	@mkdir -p $(dir $@)
	@CXXFLAGS_OBJ="$(CXXFLAGS) -IGeo/include -IViewer/local/include -isystem ImGui/include -isystem $(PORTS) -isystem $(PORTS)/include"; \
	if [ "$(OS)" = "macos" ]; then \
	        CXXFLAGS_OBJ="$$CXXFLAGS_OBJ -DGL_SILENCE_DEPRECATION"; \
	fi; \
	GLM_CFLAGS=$$(pkg-config --cflags glm 2>/dev/null | sed 's/-I/-isystem /g'); \
	if [ -z "$$GLM_CFLAGS" ]; then GLM_CFLAGS="-isystem $(PORTS)/include"; fi; \
	PDAL_CFLAGS=$$(pkg-config --cflags pdal 2>/dev/null | sed 's/-I/-isystem /g'); \
	$(CXX) $$CXXFLAGS_OBJ $$GLM_CFLAGS $$PDAL_CFLAGS $(GDAL_CFLAGS) -MMD -MP -c $< -o $@

# Header dependencies generated by -MMD.
-include $(OBJECTS:.o=.d)

$(OBJ_DIR):
	@mkdir -p $(OBJ_DIR)/$(SRC_DIR)

# Detect library paths once and cache to .build/config.mk.
.PHONY: _detect_libs
.build/config.mk:
	@mkdir -p .build
	@$(eval $(detect_libs))
	@echo "PORTS=$(PORTS)" > .build/config.mk
	@echo "PKG_MANAGER=$(PKG_MANAGER)" >> .build/config.mk
	@echo "[config] detected: $(PKG_MANAGER) at $(PORTS)"

# Only detect libs when building the main target (not for test/clean/help).
ifneq ($(filter-out test clean help fmt lint dist doc,$(MAKECMDGOALS)),)
-include .build/config.mk
endif

$(TARGET): $(OBJECTS)
	@$(MAKE) --no-print-directory _link_final TARGET=$(TARGET)

.PHONY: _link_final
_link_final: $(OBJECTS)
	@$(eval $(detect_libs))
	@echo "[link] $(OBJECTS) → $(TARGET) ($(OS)/$(UNAME_M), $(PKG_MANAGER))"
	@if [ "$(OS)" = "macos" ]; then \
	        GL_LIBS="-framework OpenGL -framework Cocoa -framework IOKit"; \
	else \
	        GL_LIBS="-lGL -lX11 -lXrandr -lXi -lpthread"; \
	fi; \
	PDAL_LIBS=$$(pkg-config --libs pdal 2>/dev/null); \
	if [ -z "$$PDAL_LIBS" ]; then PDAL_LIBS="-L$(PORTS)/lib -lPDAL"; fi; \
	case " $$PDAL_LIBS " in *" -lgdal "*) EXTRA_GDAL="";; *) EXTRA_GDAL="$(GDAL_LDFLAGS)";; esac; \
	$(CXX) $(OBJECTS) -o $(TARGET) \
	        $$PDAL_LIBS $$GL_LIBS -L$(PORTS)/lib -lglfw $$EXTRA_GDAL $(OMP_LDFLAGS) \
	        -Wl,-rpath,$(PORTS)/lib || true
	@if [ -f $(TARGET) ]; then echo "[build] done: ./$(TARGET)"; fi

run: build
	./$(TARGET) $(ARGS)

# ===========================================================================
# Test target (no external deps needed)
# ===========================================================================

test:
	@echo "[test] the tests are atf-c++ modules now: run 'bmake test' (bmake-it build)"
	@exit 1

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# ===========================================================================
# Lint (static analysis)
# ===========================================================================

lint:
	@echo "[lint] running static analysis on $(SRC)..."
	@if command -v clang-tidy >/dev/null 2>&1; then \
	        echo "[lint] using clang-tidy"; \
	        PORTS=""; \
	        for d in /opt/local /opt/homebrew /usr/local /usr; do \
	                if [ -f "$$d/include/GLFW/glfw3.h" ]; then PORTS="$$d"; break; fi; \
	        done; \
	        clang-tidy $(SRC) -- -std=c++17 -I$${PORTS:-/usr}/include 2>&1 \
	                | grep -v 'note:\|command line\|command-line\|^Use -' \
	                | head -80; \
	        echo "[lint] clang-tidy done (see .clang-tidy for config)"; \
	elif command -v cppcheck >/dev/null 2>&1; then \
	        echo "[lint] using cppcheck"; \
	        cppcheck --enable=warning,style,performance --suppress=missingIncludeSystem \
	                --std=c++17 $(SRC) 2>&1 | head -80; \
	        echo "[lint] cppcheck done"; \
	else \
	        echo "[lint] WARNING: no linter found."; \
	        echo "[lint]   macOS:  sudo port install clang-tidy OR brew install llvm"; \
	        echo "[lint]   Linux:  sudo apt install clang-tidy cppcheck"; \
	fi

# ===========================================================================
# Format (clang-format)
# ===========================================================================

fmt:
	@if command -v clang-format >/dev/null 2>&1; then \
	        echo "[fmt] formatting $(SRC) and $(TEST_SRC)..."; \
	        clang-format -i $(SRC) $(TEST_SRC); \
	        echo "[fmt] done"; \
	else \
	        echo "[fmt] clang-format not found. Install: sudo port install clang-format"; \
	fi

# ===========================================================================
# Distribution tarball
# ===========================================================================

VERSION    := $(shell git describe --tags 2>/dev/null || echo "v0.1.0")
DIST_NAME  := lasviewer

dist:
	@echo "[dist] creating $(DIST_NAME).tar.gz from the committed tree (git archive)..."
	@git archive --format=tar.gz --prefix=$(DIST_NAME)/ -o $(DIST_NAME).tar.gz HEAD
	@echo "[dist] created: $(DIST_NAME).tar.gz"

# ===========================================================================
# Documentation (Doxygen)
# ===========================================================================

doc:
	@echo "[doc] generating documentation..."
	@if command -v doxygen >/dev/null 2>&1; then \
	        mkdir -p docs/html; \
	        doxygen docs/Doxyfile 2>&1 | tail -5; \
	        echo "[doc] HTML docs: open docs/html/index.html"; \
	else \
	        echo "[doc] doxygen not found."; \
	        echo "[doc]   macOS:  sudo port install doxygen"; \
	        echo "[doc]   Linux:  sudo apt install doxygen graphviz"; \
	fi

# ===========================================================================
# Clean
# ===========================================================================

clean:
	@echo "[clean] removing build artifacts..."
	rm -f $(TARGET) lasviewer-debug $(TEST_BIN) compile_commands.json
	rm -rf $(BUILD_DIR) docs/html docs/latex
	rm -f *.tar.gz
	@echo "[clean] done"

# ===========================================================================
# Help
# ===========================================================================

help:
	@echo "lasviewer — Makefile targets"
	@echo ""
	@echo "  make            build the viewer (default)"
	@echo "  make build      compile main.cpp → lasviewer"
	@echo "  make debug      compile with -g instead of -O2 → lasviewer-debug"
	@echo "  make test       compile + run unit tests (no external deps)"
	@echo "  make lint       static analysis (clang-tidy / cppcheck)"
	@echo "  make fmt        format source with clang-format"
	@echo "  make dist       create source tarball"
	@echo "  make doc        generate Doxygen documentation"
	@echo "  make clean      remove all build artifacts"
	@echo "  make run        build + run (ARGS=\"...\")"
	@echo "  make help       show this help"
	@echo ""
	@echo "Environment:"
	@echo "  OS:        $(OS) ($(UNAME_M))"
	@echo "  Compiler:  $(CXX)"
	@echo "  OpenMP:    auto-detected (use OPENMP=0 to disable)"
	@echo "  Version:   $(VERSION)"
	@echo ""
	@echo "Install deps:"
	@echo "  macOS (MacPorts): sudo port install glfw pdal gdal glm pkgconfig"
	@echo "  macOS (Homebrew): brew install glfw pdal gdal glm pkg-config"
	@echo "  Linux (apt):      sudo apt install libglfw3-dev libpdal-dev libgdal-dev libglm-dev"
	@echo ""
	@echo "OpenMP note: Apple's default clang (/usr/bin/clang++) does NOT support OpenMP."
	@echo "  For multi-threaded loading: make CXX_OVERRIDE=clang-mp-19  (MacPorts)"
	@echo "                              make CXX_OVERRIDE=g++          (if gcc installed)"
	@echo "  Or disable: make OPENMP=0"
