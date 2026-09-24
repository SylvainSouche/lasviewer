// lasviewer — main entry point.
//
// This file is now an orchestration layer only: it parses arguments, prints
// the banner, displays file metadata, loads the appropriate data sources
// (LAZ/LAS point cloud, COPC stream, or GeoTIFF DEM mesh), creates the GLFW
// window, compiles the shaders, and runs the render loop. All heavy lifting
// lives in the src/*.cpp files.
//
// Build:   see Makefile
// Run:     ./lasviewer path/to/cloud.laz [ortho.tif]
//          ./lasviewer -d dem.tif [-o ortho.tif]
//          ./lasviewer -cop cloud.copc.laz [-o ortho.tif]
//
// Controls:
//   Left mouse drag    : orbit (rotate around target)
//   Double-click        : focus — picks the clicked point (depth-buffer
//                         readback + unproject, works for points, CPU
//                         DEM mesh, and GPU-tessellated+displaced DEM
//                         mesh alike), recenters it on screen, and makes
//                         it the new orbit pivot, WITHOUT moving the eye
//                         (yaw/pitch/distance are solved so position()
//                         lands back exactly where it was — no dolly/
//                         zoom along the view axis). Pan/arrow step size
//                         (5% of eye-to-target distance) adapts
//                         automatically since it's already distance-
//                         relative.
//   Shift+Left drag    : turn the viewer's head (look around, eye fixed)
//   Right mouse drag   : pan (translate target in screen plane)
//   Middle mouse drag  : pan
//   Arrow keys         : translate eye left/right/up/down
//   Shift+Up/Down       : move eye forward/backward
//   Mouse wheel        : move forward/backward (not a distance-clamped zoom)
//   R                   : reset camera to default
//   E / D               : increase / decrease Z exaggeration
//   U                   : reset Z to 1.0x (true scale)
//   S / F               : decrease / increase point density (LAZ/COPC), or
//                         DEM quadtree depth ceiling — the coarse-patch
//                         resolution ceiling (triggers a mesh reload)
//   N / M               : decrease / increase point size (smaller / bigger)
//   C                   : toggle colors (orthophoto / elevation)
//   P                   : toggle perspective / orthographic
//   V                   : side view
//   T                   : top view
//   B                   : toggle tile bbox wireframe (streaming)
//   W                   : toggle wireframe display (triangulated/tessellated DEM)
//   A                   : toggle displacement mapping (DEM tessellation)
//   I / O               : decrease / increase point-collapsing angle
//                         (DEM mesh only — triggers a full reload)
//   G                   : toggle master (coarse patch) edge overlay, red
//   L                   : toggle on-screen console log
//   H                   : toggle on-screen help (10s)
//   ESC                 : quit

// GLFW/OpenGL headers: single canonical include, see src/gl_platform.h for
// why this used to be a fragile per-file ad-hoc block that silently broke
// for shaders.cpp specifically.
#include "src/gl_platform.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "src/camera.h"
#include "src/point_cloud.h"
#include "src/geotiff.h"
#include "src/copc_streamer.h"
#include "src/dem_mesh.h"
#include "src/dem_tess_mesh.h"
#include "src/shaders.h"
#include "src/text_renderer.h"
#include "src/gl_app.h"

// ---------------------------------------------------------------------------
// Print usage and exit.
// ---------------------------------------------------------------------------
static void printUsage(const char* prog) {
    std::cerr << "Usage: " << prog << " [options] [cloud.laz ortho.tif]\n\n"
              << "Options:\n"
              << "  -cop <file.laz>  : load a COPC/LAZ point cloud\n"
              << "  -d   <dem.tif>   : load a GeoTIFF DEM (elevation raster)\n"
              << "  -o   <ortho.tif> : load an orthophoto for point coloring\n"
              << "\nBare positional args (no flags):\n"
              << "  lasviewer cloud.laz [ortho.tif]\n"
              << "  lasviewer dem.tif [ortho.tif]\n\n"
              << "Controls:\n"
              << "  Left drag       : orbit (rotate)\n"
              << "  Dbl-click       : focus on picked point (new orbit pivot)\n"
              << "  Shift+Left drag : turn head (look around, eye fixed)\n"
              << "  Right/Mid drag  : pan\n"
              << "  Arrows          : translate eye (Shift+Up/Down = forward/back)\n"
              << "  Wheel           : move forward/back (not a distance-clamped zoom)\n"
              << "  R               : reset camera\n"
              << "  E / D           : increase / decrease Z exaggeration\n"
              << "  U               : reset Z to 1.0x (true scale)\n"
              << "  S / F           : decrease / increase point density (LAZ/COPC)\n"
              << "                    or DEM quadtree depth ceiling (reloads mesh)\n"
              << "  N / M           : decrease / increase point size\n"
              << "  C               : toggle colors (orthophoto / elevation)\n"
              << "  P               : toggle perspective / orthographic\n"
              << "  V               : side view   T : top view\n"
              << "  B               : toggle tile bbox wireframe (streaming)\n"
              << "  W               : toggle wireframe (triangulated/tessellated DEM)\n"
              << "  A               : toggle displacement mapping (DEM tessellation)\n"
              << "  I / O           : decrease / increase point-collapsing angle (DEM mesh)\n"
              << "  G               : toggle master (coarse patch) edge overlay, red\n"
              << "  L               : toggle console log overlay\n"
              << "  H               : show on-screen help (10s)\n"
              << "  ESC             : quit\n";
}

// ---------------------------------------------------------------------------
// Parse argv into (copPath, demPath, orthoPath).
// ---------------------------------------------------------------------------
static bool parseArgs(int argc, char** argv,
                      std::string& copPath, std::string& demPath,
                      std::string& orthoPath) {
    bool hasExplicitInput = false;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            std::exit(0);
        } else if (arg == "-cop" && i + 1 < argc) {
            copPath = argv[++i];
            hasExplicitInput = true;
        } else if (arg == "-d" && i + 1 < argc) {
            demPath = argv[++i];
            hasExplicitInput = true;
        } else if (arg == "-o" && i + 1 < argc) {
            orthoPath = argv[++i];
        } else if (arg[0] != '-') {
            if (!hasExplicitInput && copPath.empty() && demPath.empty()) {
                std::string ext = arg.substr(arg.find_last_of('.') + 1);
                std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
                if (ext == "tif" || ext == "tiff") demPath = arg;
                else                               copPath = arg;
                hasExplicitInput = true;
            } else if (orthoPath.empty()) {
                orthoPath = arg;
            }
        }
    }
    return !(copPath.empty() && demPath.empty());
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    std::string demPath, copPath, orthoPath;
    if (!parseArgs(argc, argv, copPath, demPath, orthoPath)) {
        std::cerr << "Usage: " << argv[0]
                  << " [-cop cloud.laz | -d dem.tif] [-o ortho.tif]\n"
                  << "       " << argv[0] << " cloud.laz [ortho.tif]\n"
                  << "       " << argv[0] << " dem.tif [ortho.tif]\n"
                  << "Run with -h for full help." << std::endl;
        return 1;
    }

    // Banner — printed before LogCapture so it always shows on real stderr.
    std::cerr << "lasviewer v0.1.0 — LiDAR + DEM + orthophoto 3D viewer\n"
              << "Copyright (c) 2024 lasviewer contributors. BSD 3-Clause.\n"
              << std::endl;

    LogCapture logCapture(&g_logBuffer);

    // --- Display file metadata before loading ---
    std::cerr << "=== Input Files ===" << std::endl;
    if (!copPath.empty()) {
        std::cerr << "  Point cloud: " << copPath << std::endl;
        double hMinX, hMinY, hMinZ, hMaxX, hMaxY, hMaxZ;
        if (getCloudBounds(copPath, hMinX, hMinY, hMinZ, hMaxX, hMaxY, hMaxZ)) {
            std::cerr << "    bbox: X[" << hMinX << ", " << hMaxX << "]"
                      << "  Y[" << hMinY << ", " << hMaxY << "]"
                      << "  Z[" << hMinZ << ", " << hMaxZ << "]" << std::endl;
        }
    }
    if (!demPath.empty()) std::cerr << "  DEM:         " << demPath << std::endl;
    if (!orthoPath.empty()) std::cerr << "  Orthophoto:  " << orthoPath << std::endl;
    std::cerr << std::endl << "=== Loading ===" << std::endl;

    // --- Load data sources ---
    PointCloud cloud;
    bool useStreaming = false;
    bool useDEMMesh = false;
    DEMMesh demMesh;
    Orthophoto demOrthoStorage;
    const Orthophoto* demOrthoForUpload = nullptr;
    TileGrid tileGrid;

    if (!copPath.empty()) {
        bool isCopc = (copPath.find(".copc.") != std::string::npos);
        if (isCopc) {
            double bMinX, bMinY, bMinZ, bMaxX, bMaxY, bMaxZ;
            if (!getCloudBounds(copPath, bMinX, bMinY, bMinZ, bMaxX, bMaxY, bMaxZ)) {
                std::cerr << "ERROR: could not read COPC bounds" << std::endl;
                return 1;
            }
            std::cerr << "[stream] COPC bbox: (" << bMinX << "," << bMinY << "," << bMinZ
                      << ") -> (" << bMaxX << "," << bMaxY << "," << bMaxZ << ")" << std::endl;
            cloud.bboxMin = glm::dvec3(bMinX, bMinY, bMinZ);
            cloud.bboxMax = glm::dvec3(bMaxX, bMaxY, bMaxZ);
            cloud.worldCenter = (cloud.bboxMin + cloud.bboxMax) * 0.5;
            cloud.worldScale = glm::length(cloud.bboxMax - cloud.bboxMin);
            double invScale = 1.0 / cloud.worldScale;
            cloud.glBBoxMin = glm::vec2(
                static_cast<float>((bMinX - cloud.worldCenter.x) * invScale),
                static_cast<float>(-(bMaxY - cloud.worldCenter.y) * invScale));
            cloud.glBBoxMax = glm::vec2(
                static_cast<float>((bMaxX - cloud.worldCenter.x) * invScale),
                static_cast<float>(-(bMinY - cloud.worldCenter.y) * invScale));
            cloud.glBBoxMinY = static_cast<float>((bMinZ - cloud.worldCenter.z) * invScale);
            cloud.glBBoxMaxY = static_cast<float>((bMaxZ - cloud.worldCenter.z) * invScale);
            cloud.glDensity = 5.0f;
            cloud.pointCount = 0;
            useStreaming = true;
        } else {
            if (!loadPointCloud(copPath, cloud)) return 1;
        }
    } else {
        // DEM mode: build a textured mesh instead of a point cloud.
        Orthophoto orthoForMesh;
        if (!orthoPath.empty()) {
            if (loadTIFF(orthoPath, orthoForMesh, 64'000'000) && !orthoForMesh.hasGeo) {
                std::string tfwPath = orthoPath;
                size_t dot = tfwPath.find_last_of('.');
                if (dot != std::string::npos) tfwPath = tfwPath.substr(0, dot) + ".tfw";
                else                          tfwPath += ".tfw";
                loadTFW(tfwPath, orthoForMesh);
            }
        }
        // Use the orthophoto as a texture whether or not it has GeoTIFF
        // tags. If it has no geo, dem_mesh.cpp stretches it over the DEM
        // extent. We only skip it if the load entirely failed (no pixels).
        if (!orthoForMesh.pixels.empty()) {
            demOrthoStorage = std::move(orthoForMesh);
            demOrthoForUpload = &demOrthoStorage;
        }
        if (!demMesh.loadFromDEM(demPath, demOrthoForUpload)) return 1;

        cloud.bboxMin = demMesh.bboxMin;
        cloud.bboxMax = demMesh.bboxMax;
        cloud.worldCenter = demMesh.worldCenter;
        cloud.worldScale = demMesh.worldScale;
        cloud.glBBoxMin = demMesh.glBBoxMin;
        cloud.glBBoxMax = demMesh.glBBoxMax;
        cloud.glBBoxMinY = demMesh.glBBoxMinY;
        cloud.glBBoxMaxY = demMesh.glBBoxMaxY;
        cloud.glDensity = 1.0f;
        cloud.pointCount = 0;
        useDEMMesh = true;
    }

    // --- Load orthophoto (skip in DEM mode — texture already loaded). ---
    Orthophoto ortho;
    if (!orthoPath.empty() && !useDEMMesh) {
        if (!loadTIFF(orthoPath, ortho, 64'000'000)) {
            std::cerr << "ERROR: could not load orthophoto: " << orthoPath << std::endl;
            return 1;
        }
        if (!ortho.hasGeo) {
            std::string tfwPath = orthoPath;
            size_t dot = tfwPath.find_last_of('.');
            if (dot != std::string::npos) tfwPath = tfwPath.substr(0, dot) + ".tfw";
            else                          tfwPath += ".tfw";
            if (!loadTFW(tfwPath, ortho)) {
                std::cerr << "ERROR: orthophoto has no GeoTIFF tags and no .tfw sidecar ("
                          << tfwPath << ")" << std::endl;
                return 1;
            }
        }

        // Coverage check: the orthophoto must cover ≥25% of the cloud XY extent.
        double orthoMinX = ortho.C;
        double orthoMaxX = ortho.C + ortho.A * (ortho.width - 1);
        double orthoMinY = ortho.F + ortho.E * (ortho.height - 1);
        double orthoMaxY = ortho.F;
        if (orthoMinX > orthoMaxX) std::swap(orthoMinX, orthoMaxX);
        if (orthoMinY > orthoMaxY) std::swap(orthoMinY, orthoMaxY);
        double overlapMinX = std::max(orthoMinX, cloud.bboxMin.x);
        double overlapMaxX = std::min(orthoMaxX, cloud.bboxMax.x);
        double overlapMinY = std::max(orthoMinY, cloud.bboxMin.y);
        double overlapMaxY = std::min(orthoMaxY, cloud.bboxMax.y);
        double overlapArea = std::max(0.0, overlapMaxX - overlapMinX) *
                             std::max(0.0, overlapMaxY - overlapMinY);
        double cloudArea = (cloud.bboxMax.x - cloud.bboxMin.x) *
                           (cloud.bboxMax.y - cloud.bboxMin.y);
        double coverageRatio = (cloudArea > 0) ? overlapArea / cloudArea : 0.0;
        std::cerr << "[ortho] coverage: " << (coverageRatio * 100.0)
                  << "% of cloud area" << std::endl;
        if (coverageRatio < 0.25) {
            std::cerr << "ERROR: orthophoto covers only " << (coverageRatio * 100.0)
                      << "% of the point cloud area (minimum 25% required)." << std::endl;
            return 1;
        }

        if (!useStreaming) {
            colorizeFromOrthophoto(cloud, ortho);
            if (!cloud.hasOrthoColors) {
                std::cerr << "ERROR: orthophoto colorization failed." << std::endl;
                return 1;
            }
        } else {
            cloud.hasOrthoColors = true;
            std::cerr << "[ortho] streaming mode — tiles will sample orthophoto on load"
                      << std::endl;
        }
    }

    // --- GLFW / OpenGL setup ---
    if (!glfwInit()) { std::cerr << "ERROR: glfwInit failed" << std::endl; return 1; }
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GLFW_TRUE);

    // Try GL 4.1 core first (enables the GPU-tessellated DEM path, §2 of
    // docs/design-tessellation-displacement.md — macOS supports 4.1 core on
    // both Intel and Apple Silicon). A 4.1 core context still runs #version
    // 330 core shaders fine (core profile doesn't remove anything from
    // 3.3), so the point-cloud/COPC paths are unaffected either way. Falls
    // back to 3.3 core, the previous fixed requirement, if 4.1 isn't
    // available — the DEM path then falls back to DEMMesh's CPU mesh (see
    // demTessSupported() usage below).
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    GLFWwindow* window = glfwCreateWindow(1280, 800,
                                          "lasviewer — LAZ/LAS + Orthophoto",
                                          nullptr, nullptr);
    if (!window) {
        std::cerr << "[gl] GL 4.1 core context not available, falling back to 3.3 core "
                     "(GPU-tessellated DEM mesh will not be available)" << std::endl;
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        window = glfwCreateWindow(1280, 800,
                                  "lasviewer — LAZ/LAS + Orthophoto",
                                  nullptr, nullptr);
    }
    if (!window) { std::cerr << "ERROR: could not create GLFW window" << std::endl;
                   glfwTerminate(); return 1; }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    Camera camera;
    camera.target = glm::vec3(0.0f);
    camera.distance = 2.0f;
    camera.yaw = 0.6f;
    camera.pitch = 1.2f;
    camera.fov = 55.0f;
    camera.aspect = 1280.0f / 800.0f;
    camera.nearP = 0.05f;
    camera.farP = 1000.0f;

    InputState input;
    input.camera = &camera;
    input.cloud = &cloud;
    input.logLines = g_logBuffer;
    input.zScale = 1.0f;
    int fbW = 0, fbH = 0;
    glfwGetFramebufferSize(window, &fbW, &fbH);
    input.viewportW = static_cast<float>(fbW);
    input.viewportH = static_cast<float>(fbH);
    camera.aspect = static_cast<float>(fbW) / static_cast<float>(fbH > 0 ? fbH : 1);

    glfwSetWindowUserPointer(window, &input);
    glfwSetCursorPosCallback(window, cursorPosCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetScrollCallback(window, scrollCallback);
    glfwSetFramebufferSizeCallback(window, resizeCallback);
    glfwSetKeyCallback(window, keyCallback);
    glfwSetCharCallback(window, charCallback);

    // Compile shaders (uses the shaders:: namespace helpers).
    GLuint pointProgram = shaders::linkProgram(shaders::kPointCloudVert, shaders::kPointCloudFrag);
    GLuint lineProgram  = shaders::linkProgram(shaders::kLineVert, shaders::kLineFrag);
    GLuint meshProgram  = shaders::linkProgram(shaders::kMeshVert, shaders::kMeshFrag);
    if (!pointProgram) { glfwTerminate(); return 1; }

    // --- GPU-tessellated DEM path (optional; see dem_tess_mesh.h and
    // docs/design-tessellation-displacement.md). Re-reads the same DEM/
    // orthophoto files DEMMesh already loaded above — a deliberate,
    // acknowledged inefficiency (double TIFF read) chosen to avoid
    // restructuring the load-before-window-creation flow elsewhere in this
    // file; revisit once this path is verified on real hardware. Falls
    // back to the existing DEMMesh CPU rendering (already fully loaded) if
    // the context isn't GL 4.0+, or if the tessellated build/upload fails
    // for any reason. ---
    DEMTessMesh demTessMesh;
    GLuint tessProgram = 0;
    bool useDEMTess = false;
    // The real target detail level — what the window actually opens
    // "aiming for," and what every subsequent I/O/S/F rebuild starts back
    // from. Matches loadFromDEM()'s own defaults.
    const double kDefaultCollapseAngle = 1.0;
    const int kDefaultMaxLevel = 5;
    if (useDEMMesh && demTessSupported()) {
        tessProgram = shaders::linkTessProgram(shaders::kMeshTessVert,
                                               shaders::kMeshTessControl,
                                               shaders::kMeshTessEval,
                                               shaders::kMeshFrag);
        // Load coarsest first, display it immediately, compute the full
        // picture in the background — requested directly, and applies
        // here to the VERY FIRST load, not just later rebuilds. maxLevel=0
        // (just the initial COARSE grid, no subdivision at all) is always
        // fast regardless of DEM size, so the window opens showing
        // something right away instead of blocking on what could be a
        // genuinely slow bottom-up collapse (§6k of the design doc) at the
        // real target maxLevel.
        if (tessProgram &&
            demTessMesh.loadFromDEM(demPath, demOrthoForUpload,
                                    kDefaultCollapseAngle, /*maxLevelParam=*/0) &&
            demTessMesh.uploadGPU(demOrthoForUpload)) {
            useDEMTess = true;
            std::cerr << "[dem-tess] using GPU-tessellated DEM path "
                         "(fast coarse pass shown; full detail building "
                         "in the background)" << std::endl;
            demTessMesh.requestBackgroundBuild(demPath, demOrthoForUpload,
                                               kDefaultCollapseAngle, kDefaultMaxLevel);
        } else {
            std::cerr << "[dem-tess] GPU-tessellated DEM setup failed, "
                         "falling back to CPU mesh (DEMMesh)" << std::endl;
            if (tessProgram) { glDeleteProgram(tessProgram); tessProgram = 0; }
        }
    }

    // S/F now controls demMaxLevel (see gl_app.h) — initialize it to
    // whichever DEM path actually ended up active's current TARGET depth
    // (DEMMesh and DEMTessMesh have different defaults, 6 vs 5), so the
    // first press starts from the real intended value. For DEMTessMesh
    // specifically this is NOT demTessMesh.maxLevel right now (that's
    // still 0, the fast coarse pass shown immediately — see above; the
    // real target is being built in the background and hasn't landed yet).
    input.demMaxLevel = useDEMTess ? kDefaultMaxLevel : demMesh.maxLevel;

    // Upload DEM orthophoto texture now that the GL context is ready.
    // The orthophoto is used as a texture whether or not it has GeoTIFF
    // tags — when it has none, the UV computation in dem_mesh.cpp stretches
    // it over the DEM extent.
    //
    // Skipped entirely when useDEMTess is true: demMesh isn't rendered in
    // that case (DEMTessMesh already uploaded its own color texture from
    // the same ortho above), so uploading a second, unused texture here
    // would just waste GPU memory and upload time.
    //
    // Deliberately does NOT free demOrthoStorage.pixels after upload
    // (previously did) — DEMMesh::reload() and DEMTessMesh::reload() (I/O
    // keys, point-collapsing angle) both need to re-upload a color
    // texture from these same pixels on every reload, and re-reading the
    // orthophoto from disk each time was judged more complexity than the
    // bounded CPU RAM cost of keeping it resident (it's already capped at
    // load time by loadTIFF's own downsample logic, geotiff.cpp).
    if (useDEMMesh && !useDEMTess && demOrthoForUpload && !demOrthoForUpload->pixels.empty()
        && demMesh.orthoUsable) {
        demMesh.texture = 0;
        glGenTextures(1, &demMesh.texture);
        glBindTexture(GL_TEXTURE_2D, demMesh.texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8,
                     demOrthoForUpload->width, demOrthoForUpload->height, 0,
                     GL_RGBA, GL_UNSIGNED_BYTE, demOrthoForUpload->pixels.data());
        glGenerateMipmap(GL_TEXTURE_2D);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        demMesh.texW = demOrthoForUpload->width;
        demMesh.texH = demOrthoForUpload->height;
        std::cerr << "[dem-mesh] texture uploaded: " << demMesh.texW << "x"
                  << demMesh.texH << " (hasGeo="
                  << (demOrthoForUpload->hasGeo ? 1 : 0) << ")" << std::endl;
    } else if (useDEMMesh && !useDEMTess && demOrthoForUpload && !demOrthoForUpload->pixels.empty()
               && !demMesh.orthoUsable) {
        std::cerr << "[dem-mesh] orthophoto present but does not overlap this "
                     "DEM — skipping texture, using elevation color ramp"
                  << std::endl;
    } else if (useDEMMesh && !useDEMTess) {
        std::cerr << "[dem-mesh] no orthophoto texture — using elevation color ramp"
                  << std::endl;
    }

    TextRenderer textR;
    textR.init(32);

    GLuint pointVAO = 0;
    if (useStreaming) {
        const Orthophoto* orthoForTiles =
            (!orthoPath.empty() && cloud.hasOrthoColors) ? &ortho : nullptr;
        tileGrid.init(copPath,
                      cloud.bboxMin.x, cloud.bboxMin.y, cloud.bboxMin.z,
                      cloud.bboxMax.x, cloud.bboxMax.y, cloud.bboxMax.z,
                      orthoForTiles);
    } else if (!useDEMMesh) {
        if (cloud.hasOrthoColors) input.useOrthoColors = true;
        GLuint colorVBO = 0;
        pointVAO = uploadPointCloudGL(cloud, input.useOrthoColors, &colorVBO);
        input.colorVBO = colorVBO;
    }

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_MULTISAMPLE);

    std::cout << "\n--- lasviewer ---\n"
              << "  Left drag       : orbit\n"
              << "  Dbl-click       : focus on picked point (new orbit pivot)\n"
              << "  Shift+Left drag : turn head (look around)\n"
              << "  Right/Mid drag  : pan\n"
              << "  Arrows          : translate eye (Shift+Up/Down = forward/back)\n"
              << "  Wheel           : move forward/back (not a distance-clamped zoom)\n"
              << "  R  reset camera\n"
              << "  E D  Z exaggeration   U  reset Z (now " << input.zScale << "x)\n"
              << "  S F  density (points) / max-level (DEM)   N M  point size\n"
              << "  C  toggle colors      P  perspective/ortho\n"
              << "  V  side view   T  top view\n"
              << "  B  tile boxes  W  wireframe   A  displacement\n"
              << "  I O  collapse angle (DEM mesh, reloads)   G  master edges\n"
              << "  L  console log\n"
              << "  H  show this help     ESC  quit\n"
              << std::endl;

    // --- Render loop ---
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        glClearColor(0.10f, 0.11f, 0.13f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 V = camera.view();

        // Recompute near/far planes only when the camera moves or zScale changes.
        if (input.nearFarDirty ||
            camera.yaw      != input.lastYaw     ||
            camera.pitch    != input.lastPitch   ||
            camera.distance != input.lastDist    ||
            camera.target   != input.lastTarget  ||
            input.zScale    != input.lastZScale) {
            input.nearFarDirty = false;
            input.lastYaw = camera.yaw;
            input.lastPitch = camera.pitch;
            input.lastDist = camera.distance;
            input.lastTarget = camera.target;
            input.lastZScale = input.zScale;
            glm::vec3 bboxMin(cloud.glBBoxMin.x, cloud.glBBoxMinY, cloud.glBBoxMin.y);
            glm::vec3 bboxMax(cloud.glBBoxMax.x, cloud.glBBoxMaxY, cloud.glBBoxMax.y);
            computeNearFar(camera, bboxMin, bboxMax, input.zScale,
                           camera.nearP, camera.farP);
        }
        glm::mat4 P = camera.proj();

        // Consume the I/O (collapsing angle) and/or S/F (quadtree depth
        // ceiling) rebuild requests. Both drive LOAD-TIME quadtree
        // decisions (§4b of the design doc), so either needs a full mesh
        // rebuild, not just a render-time uniform update. Checked together
        // and requested once with the CURRENT value of both parameters, so
        // pressing both in close succession doesn't trigger two separate
        // rebuilds. Whichever DEM path is currently active is rebuilt; a
        // no-op for the LAZ/COPC point-cloud path (neither concept applies
        // to discrete points).
        //
        // ASYNC for DEMTessMesh (requested directly: "load coarsest, and
        // display it while we compute the full picture. swap
        // representation when ready. same when changes require rebuilding
        // the representation") — requestBackgroundBuild() returns
        // immediately, the OLD representation keeps rendering uninterrupted
        // (no hitch, unlike the synchronous reload() this replaced for
        // this path), and pollBackgroundBuild() below (called every frame,
        // unconditionally) swaps in the new one once it's ready, however
        // many frames later that turns out to be — see §6k of the design
        // doc for why that can be genuinely slow at high maxLevel.
        //
        // STILL SYNCHRONOUS for DEMMesh (the CPU-only fallback,
        // dem_mesh.cpp/.h) — a deliberate scope decision, not an
        // oversight: threading it would need the same background-build
        // infrastructure duplicated against a different class with a
        // different (file-static, lazy-init) GPU resource pattern, and
        // DEMMesh is the less-tested, less-central path this whole feature
        // has focused on. Still hitches the frame on I/O/S/F for that
        // path, same as before.
        //
        // NOTE: this replaced an earlier F-key mechanism
        // (DEMTessMesh::increaseHeightmapResolution(), progressively
        // fetching a higher-resolution heightmap TEXTURE from disk) that
        // is now orphaned — S/F no longer sets any flag that triggers it.
        // That was a genuinely different concern (texture detail available
        // for displacement to sample, vs. the coarse patch COUNT this
        // section handles) and is not restored by this change; every
        // rebuild here still starts the heightmap at the initial
        // MAX_HEIGHTMAP_TEXELS cap, not progressively beyond it. Worth
        // revisiting if that capability is still wanted under a different
        // key.
        if (input.collapseAngleChanged || input.demMaxLevelChanged) {
            input.collapseAngleChanged = false;
            input.demMaxLevelChanged = false;
            double newAngle = static_cast<double>(input.pointCollapseAngleDeg);
            int newMaxLevel = input.demMaxLevel;
            if (useDEMTess) {
                demTessMesh.requestBackgroundBuild(demPath, demOrthoForUpload,
                                                   newAngle, newMaxLevel);
            } else if (useDEMMesh) {
                demMesh.reload(demOrthoForUpload, newAngle, newMaxLevel);
            }
        }

        // Poll every frame, unconditionally — this is also how the
        // initial "coarsest first, then background full-detail" load
        // (right after window creation, above) picks up its result, not
        // just explicit I/O/S/F rebuilds. Cheap no-op when nothing is
        // pending or still building.
        if (useDEMTess) {
            demTessMesh.pollBackgroundBuild(demOrthoForUpload);
        }

        // --- Render the active data source ---
        if (useDEMTess) {
            glm::vec3 camPos = camera.position();
            // Fixed at the design doc's original 8px/segment baseline —
            // S/F (input.density) no longer drives GPU tessellation
            // density for this path; it now controls demMaxLevel (the
            // quadtree depth ceiling, via reload() above) instead, on
            // direct request. input.density is back to being exclusively
            // a point-cloud (LAZ/COPC) control, its original purpose.
            float targetPx = 8.0f;
            if (input.tessWireframe) glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            demTessMesh.render(tessProgram, V, P, camPos, camera.fov,
                               input.viewportH, input.zScale, targetPx,
                               input.useDisplacement, input.showMasterEdges);
            if (input.tessWireframe) glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        } else if (useDEMMesh) {
            if (input.tessWireframe) glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            demMesh.render(meshProgram, V, P, input.zScale, input.showMasterEdges);
            if (input.tessWireframe) glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        } else if (useStreaming) {
            glm::vec3 camPos = camera.position();
            double now = glfwGetTime();
            tileGrid.update(camPos, camera.fov, input.viewportH, now);
            tileGrid.render(pointProgram, V, P, camPos, camera.fov,
                            input.viewportW, input.viewportH,
                            input.zScale, input.pointSizeMul, now, input.useOcclusion);
        } else {
            glUseProgram(pointProgram);
            glUniformMatrix4fv(glGetUniformLocation(pointProgram, "uView"), 1, GL_FALSE, glm::value_ptr(V));
            glUniformMatrix4fv(glGetUniformLocation(pointProgram, "uProj"), 1, GL_FALSE, glm::value_ptr(P));
            glUniform1f(glGetUniformLocation(pointProgram, "uPointSize"), 0.0015f * input.pointSizeMul);
            glUniform1f(glGetUniformLocation(pointProgram, "uViewportH"), input.viewportH);
            glUniform1f(glGetUniformLocation(pointProgram, "uZScale"), input.zScale);
            glUniform1f(glGetUniformLocation(pointProgram, "uTargetPixelSpacing"), 2.83f);
            glUniform1f(glGetUniformLocation(pointProgram, "uTanHalfFov"),
                        std::tan(glm::radians(camera.fov) * 0.5f));
            glUniform1f(glGetUniformLocation(pointProgram, "uDensity"), cloud.glDensity);
            glUniform1f(glGetUniformLocation(pointProgram, "uDensityMul"), input.density);
            glUniform1f(glGetUniformLocation(pointProgram, "uDisableSubsampling"), 0.0f);
            glUniform1f(glGetUniformLocation(pointProgram, "uOrtho"), camera.ortho ? 1.0f : 0.0f);
            glUniform1f(glGetUniformLocation(pointProgram, "uOrthoHeight"), camera.orthoHeight());
            glBindVertexArray(pointVAO);
            glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(cloud.pointCount));
            glBindVertexArray(0);
        }

        // --- Per-second stats printout ---
        {
            static double lastReport = 0.0;
            double now = glfwGetTime();
            if (now - lastReport >= 1.0) {
                if (useStreaming) {
                    int loaded = 0, requested = 0, empty = 0;
                    for (auto& t : tileGrid.tiles) {
                        if (t.state == TileState::LOADED) {
                            if (t.pointCount > 0) ++loaded; else ++empty;
                        } else if (t.state == TileState::REQUESTED) ++requested;
                    }
                    std::cerr << "[stream] " << loaded << " loaded + " << requested
                              << " refining + " << empty << " empty = "
                              << loaded + requested + empty << "/"
                              << tileGrid.gridX * tileGrid.gridY
                              << " tiles, " << tileGrid.pendingLoads << " pending" << std::endl;
                } else if (!useDEMMesh) {
                    std::cerr << "[points] sent " << cloud.pointCount << " pts to GPU" << std::endl;
                }
                lastReport = now;
            }
        }

        // --- On-screen help overlay (10s after pressing H) ---
        if (input.helpUntil > 0.0 && glfwGetTime() < input.helpUntil) {
            const char* help =
                "LASVIEWER CONTROLS\n"
                "Left drag       orbit\n"
                "Dbl-click       focus on picked point (new pivot)\n"
                "Shift+Left drag turn head (look around)\n"
                "Right/Mid drag  pan\n"
                "Arrows  translate eye  (Shift+Up/Down = forward/back)\n"
                "Wheel           move forward/back\n"
                "R  reset camera\n"
                "E D  Z exaggeration   U  reset Z\n"
                "S F  density (points) / max-level (DEM)   N M  point size\n"
                "C  toggle colors      P  perspective/ortho\n"
                "V  side view   T  top view\n"
                "B  tile boxes  W  wireframe   A  displacement\n"
                "I O  collapse angle (DEM mesh)   G  master edges\n"
                "L  console log\n"
                "H  show this help     ESC  quit";
            textR.drawText(help, 20, 20,
                           static_cast<int>(input.viewportW),
                           static_cast<int>(input.viewportH),
                           1.0f, 1.0f, 1.0f, 1.0f);
        }

        // --- Tile bbox wireframe (toggle with B, streaming mode only) ---
        if (input.showTileBoxes && useStreaming) {
            glUseProgram(lineProgram);
            glUniformMatrix4fv(glGetUniformLocation(lineProgram, "uView"), 1, GL_FALSE, glm::value_ptr(V));
            glUniformMatrix4fv(glGetUniformLocation(lineProgram, "uProj"), 1, GL_FALSE, glm::value_ptr(P));
            glUniform1f(glGetUniformLocation(lineProgram, "uZScale"), input.zScale);
            for (const auto& t : tileGrid.tiles) {
                bool isEvicting = (t.state == TileState::EVICTING);
                if (t.pointCount == 0 && !isEvicting) continue;
                if (isEvicting) {
                    glUniform3f(glGetUniformLocation(lineProgram, "uColor"), 1.0f, 0.0f, 0.0f);
                    glLineWidth(3.0f);
                } else {
                    glUniform3f(glGetUniformLocation(lineProgram, "uColor"), 1.0f, 1.0f, 0.0f);
                    glLineWidth(1.0f);
                }
                float c[8][3] = {
                    {t.glMin.x, t.glMin.y, t.glMin.z}, {t.glMax.x, t.glMin.y, t.glMin.z},
                    {t.glMax.x, t.glMin.y, t.glMax.z}, {t.glMin.x, t.glMin.y, t.glMax.z},
                    {t.glMin.x, t.glMax.y, t.glMin.z}, {t.glMax.x, t.glMax.y, t.glMin.z},
                    {t.glMax.x, t.glMax.y, t.glMax.z}, {t.glMin.x, t.glMax.y, t.glMax.z},
                };
                int edges[24] = {0,1,1,2,2,3,3,0, 4,5,5,6,6,7,7,4, 0,4,1,5,2,6,3,7};
                float verts[72];
                for (int e = 0; e < 24; ++e) {
                    verts[e*3]=c[edges[e]][0];
                    verts[e*3+1]=c[edges[e]][1];
                    verts[e*3+2]=c[edges[e]][2];
                }
                GLuint bVAO, bVBO;
                glGenVertexArrays(1, &bVAO);
                glGenBuffers(1, &bVBO);
                glBindVertexArray(bVAO);
                glBindBuffer(GL_ARRAY_BUFFER, bVBO);
                glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW);
                glEnableVertexAttribArray(0);
                glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void*)0);
                glDrawArrays(GL_LINES, 0, 24);
                glBindVertexArray(0);
                glDeleteVertexArrays(1, &bVAO);
                glDeleteBuffers(1, &bVBO);
            }
        }

        // --- Console log overlay (toggle with L) ---
        if (input.showLog) {
            input.logLines = g_logBuffer;
            size_t maxLines = 25;
            size_t startIdx = (input.logLines.size() > maxLines)
                            ? input.logLines.size() - maxLines : 0;
            size_t numLines = input.logLines.size() - startIdx;
            int lineH = 32 + 6;
            int startY = static_cast<int>(input.viewportH) - numLines * lineH - 10;
            for (size_t li = 0; li < numLines; ++li) {
                textR.drawText(input.logLines[startIdx + li],
                               10, static_cast<float>(startY + li * lineH),
                               static_cast<int>(input.viewportW),
                               static_cast<int>(input.viewportH),
                               0.1f, 0.9f, 0.1f, 0.85f);
            }
        }

        // --- Double-click-to-focus: consume the pick request set by
        // mouseButtonCallback, right after this frame's own rendering and
        // before the swap — glReadPixels here reads GL_BACK (the default),
        // which at this exact point unambiguously holds what was just
        // drawn. Reading depth from inside the input callback instead
        // would need GL_FRONT, which has real cross-platform reliability
        // issues (compositors, some drivers, macOS quirks) — deliberately
        // avoided. One-frame-old click coordinates at worst (the click
        // happened during glfwPollEvents at the top of this same frame),
        // imperceptible in practice. ---
        if (input.pickRequested) {
            input.pickRequested = false;
            int winW = 0, winH = 0;
            glfwGetWindowSize(window, &winW, &winH);
            if (winW > 0 && winH > 0) {
                // Screen coords (GLFW cursor) -> framebuffer pixels: on
                // HiDPI/Retina displays these differ by the OS scale
                // factor, derived here from GLFW's own reported window
                // vs. framebuffer size rather than assuming e.g. a fixed
                // 2x, so it's correct on any display.
                float scaleX = input.viewportW / static_cast<float>(winW);
                float scaleY = input.viewportH / static_cast<float>(winH);
                int fbX = static_cast<int>(input.pickX * scaleX);
                int fbYTop = static_cast<int>(input.pickY * scaleY);
                // glReadPixels' Y is bottom-up; GLFW's cursor Y is top-down.
                int glY = static_cast<int>(input.viewportH) - 1 - fbYTop;
                if (fbX >= 0 && fbX < static_cast<int>(input.viewportW) &&
                    glY >= 0 && glY < static_cast<int>(input.viewportH)) {
                    float depth = 1.0f;
                    glReadPixels(fbX, glY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
                    if (depth < 0.9999999f) {
                        float ndcX = (2.0f * fbX) / input.viewportW - 1.0f;
                        float ndcY = (2.0f * glY) / input.viewportH - 1.0f;
                        float ndcZ = depth * 2.0f - 1.0f;
                        glm::vec3 worldPos = camera.unproject(ndcX, ndcY, ndcZ);
                        // Recenter WITHOUT moving the eye: solve for the
                        // yaw/pitch/distance that make position() (= target
                        // + offset(yaw,pitch,distance)) land exactly back on
                        // the CURRENT eye position, with target = the picked
                        // point. This is the same "keep eye fixed, rotate
                        // look direction" math as Shift+drag's head-turn
                        // (gl_app.cpp) — deliberately NOT the simpler
                        // "just set target, keep yaw/pitch/distance"
                        // version tried first, which effectively dollies
                        // the eye along the view axis to preserve the OLD
                        // distance from a point generally at a different
                        // depth — a disguised zoom, not a pure recenter.
                        glm::vec3 eye = camera.position(); // BEFORE any change
                        glm::vec3 toEye = eye - worldPos;
                        float newDistance = glm::length(toEye);
                        if (newDistance > 1e-6f) {
                            glm::vec3 dir = toEye / newDistance;
                            float newPitch = std::asin(glm::clamp(dir.y, -1.0f, 1.0f));
                            float newYaw = std::atan2(dir.x, dir.z);
                            camera.target = worldPos;
                            camera.distance = newDistance;
                            camera.yaw = newYaw;
                            camera.pitch = newPitch;
                            input.nearFarDirty = true;
                            std::cerr << "[pick] focused on (" << worldPos.x << ", "
                                      << worldPos.y << ", " << worldPos.z << ")"
                                      << std::endl;
                        } else {
                            std::cerr << "[pick] clicked point coincides with the eye, "
                                         "no refocus" << std::endl;
                        }
                    } else {
                        std::cerr << "[pick] double-click missed geometry, no refocus"
                                  << std::endl;
                    }
                }
            }
        }

        glfwSwapBuffers(window);
    }

    demMesh.destroy();
    demTessMesh.destroy();
    if (tessProgram) glDeleteProgram(tessProgram);
    textR.destroy();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
