// gl_app.h — GLFW window, input handling, render loop
#pragma once
#include "gl_platform.h"
#include <glm/glm.hpp>
#include <string>
#include <vector>
#include <streambuf>

struct Camera;
struct PointCloud;
struct TextRenderer;

struct InputState {
    bool leftDown = false, rightDown = false, middleDown = false;
    bool shiftDown = false;
    double lastX = 0.0, lastY = 0.0;
    Camera* camera = nullptr;
    float viewportW = 1280.0f, viewportH = 800.0f;
    PointCloud* cloud = nullptr;
    GLuint colorVBO = 0;
    bool useOrthoColors = false;
    float zScale = 1.0f;
    float density = 1.0f;
    float pointSizeMul = 1.0f;
    double helpUntil = 0.0;
    bool useOcclusion = true;
    bool showTileBoxes = false;
    bool showLog = false;
    bool tessWireframe = false;   // W: wireframe toggle for triangulated/tessellated DEM surfaces
    bool showMasterEdges = false; // G: red coarse-patch-boundary overlay toggle — default
                                  // off, this is a diagnostic overlay, not a rendering
                                  // feature, so it shouldn't be in the way unasked
    bool useDisplacement = true;  // A: toggle GPU displacement mapping (DEM tessellation path only)
    // Double-click-to-focus. mouseButtonCallback only detects the
    // double-click and records WHERE (screen coords); it deliberately
    // does NOT call glReadPixels itself — see main.cpp's render loop for
    // why (GL_FRONT readback from inside an input callback has real
    // platform-reliability issues; reading GL_BACK right after this
    // frame's own render, before the buffer swap, is unambiguous).
    double lastClickTime = 0.0;
    double lastClickX = 0.0, lastClickY = 0.0;
    bool pickRequested = false;
    double pickX = 0.0, pickY = 0.0;
    // I/O keys: point-collapsing angle (§4b of the design doc). Unlike
    // density (S/F), this drives a LOAD-TIME quadtree decision, so
    // changing it triggers a full mesh rebuild (main.cpp consumes this
    // flag, calling DEMMesh::reload()/DEMTessMesh::reload() as
    // appropriate) rather than just updating a render-time uniform.
    float pointCollapseAngleDeg = 1.0f;
    bool collapseAngleChanged = false;
    // S/F keys: quadtree depth ceiling for the DEM path (either DEMMesh or
    // DEMTessMesh) — replaces what S/F previously did for DEM rendering
    // (GPU tessellation density, uTargetPixelsPerSegment) on direct
    // request: "that is the number I want to change with S and F,"
    // referring to the max-patch-count ceiling (COARSE * 2^maxLevel per
    // axis). `density` below is now exclusively a point-cloud (LAZ/COPC)
    // control again. main.cpp initializes demMaxLevel to match whichever
    // DEM path is actually active right after that's decided, so the
    // first S/F press starts from the correct current baseline (DEMMesh
    // and DEMTessMesh have different default depths, 6 vs 5).
    int demMaxLevel = 5;
    bool demMaxLevelChanged = false;
    std::vector<std::string> logLines;
    float lastNearP = -1.0f, lastFarP = -1.0f;
    float lastYaw = 0.0f, lastPitch = 0.0f, lastDist = 0.0f;
    glm::vec3 lastTarget{0.0f};
    float lastZScale = -1.0f;
    bool nearFarDirty = true;
};

// GLFW callbacks.
//
// keyCallback uses GLFW's `key` parameter, which — despite the naming of
// constants like GLFW_KEY_W or GLFW_KEY_A — identifies a PHYSICAL key
// position on a standard US QWERTY layout, not the character actually
// printed on the keycap or produced by the user's active OS keyboard
// layout. On AZERTY (French) keyboards specifically, the physical keys
// swap for A/Q and W/Z, and M relocates to where QWERTY has semicolon —
// so matching `key == GLFW_KEY_W` fires when an AZERTY user presses the
// key labeled 'Z', not 'W'. This is a real, previously-unaddressed bug in
// this codebase: specs.md §10.1 claimed "letters only" was sufficient for
// AZERTY compatibility, but that only avoids the SYMBOL-shift-state class
// of layout problems, not this physical-position mismatch.
//
// charCallback uses GLFW's Unicode codepoint callback instead, which DOES
// go through the OS's active keyboard layout — the codepoint reported is
// the actual character produced, correctly, for AZERTY, QWERTZ, Dvorak, or
// any other layout. All printable single-letter shortcuts (R, E, D, U, S,
// F, N, M, C, P, O, V, T, B, L, H, W, A) are handled here. keyCallback is
// kept only for non-printable/functional keys that have no character
// codepoint at all: Shift tracking, arrow keys (which also need
// PRESS+REPEAT semantics for continuous movement, not a single character
// event), and ESC.
void cursorPosCallback(GLFWwindow* win, double x, double y);
void mouseButtonCallback(GLFWwindow* win, int button, int action, int mods);
void scrollCallback(GLFWwindow* win, double dx, double dy);
void resizeCallback(GLFWwindow* win, int w, int h);
void keyCallback(GLFWwindow* win, int key, int sc, int action, int mods);
void charCallback(GLFWwindow* win, unsigned int codepoint);

// Log capture (redirects stderr for on-screen console).
struct LogCapture : std::streambuf {
    std::streambuf* original;
    std::string line;
    std::vector<std::string>* captured;
    explicit LogCapture(std::vector<std::string>* cap);
    ~LogCapture() override;
    int overflow(int c) override;
    std::streamsize xsputn(const char* s, std::streamsize n) override;
private:
    void putChar(char c);
};
extern std::vector<std::string> g_logBuffer;
std::vector<std::string> logSnapshot(); // thread-safe copy of the log
