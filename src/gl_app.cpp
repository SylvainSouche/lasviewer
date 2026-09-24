// gl_app.cpp — GLFW callbacks + LogCapture (stderr → on-screen console).
//
// InputState lives in gl_app.h and is shared with main.cpp. The log buffer
// is also defined here so LogCapture can write into it directly.
#include "gl_app.h"
#include "camera.h"
#include "point_cloud.h"
// GLFW/OpenGL headers now come from gl_app.h -> gl_platform.h (see that
// file for why this used to be a fragile per-file ad-hoc block).

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <streambuf>

// Global log buffer — accessed by InputState (copy snapshot each frame) and
// written to by LogCapture (which intercepts std::cerr).
std::vector<std::string> g_logBuffer;

// ---------------------------------------------------------------------------
// LogCapture — redirects std::cerr through a streambuf that splits on '\n'
// and stores each line in the global log buffer. Lines are also forwarded to
// the real stderr so terminal output is unchanged.
// ---------------------------------------------------------------------------

LogCapture::LogCapture(std::vector<std::string>* cap) : captured(cap) {
    original = std::cerr.rdbuf();
    std::cerr.rdbuf(this);
}

LogCapture::~LogCapture() {
    std::cerr.rdbuf(original);
}

int LogCapture::overflow(int c) {
    if (c == '\n') {
        captured->push_back(line);
        if (captured->size() > 200) captured->erase(captured->begin());
        original->sputn(line.c_str(), line.size());
        original->sputn("\n", 1);
        line.clear();
    } else if (c != '\r') {
        line += static_cast<char>(c);
    }
    return c;
}

// ---------------------------------------------------------------------------
// GLFW callbacks
// ---------------------------------------------------------------------------

void cursorPosCallback(GLFWwindow* win, double x, double y) {
    InputState* s = static_cast<InputState*>(glfwGetWindowUserPointer(win));
    if (!s || !s->camera) return;
    double dx = x - s->lastX;
    double dy = y - s->lastY;
    s->lastX = x;
    s->lastY = y;

    if (s->leftDown && !s->shiftDown) {
        // Plain drag: orbit the model around the target (unchanged).
        s->camera->yaw   -= static_cast<float>(dx) * 0.005f;
        s->camera->pitch += static_cast<float>(dy) * 0.005f;
        s->camera->pitch = glm::clamp(s->camera->pitch, -1.55f, 1.55f);
    } else if (s->leftDown && s->shiftDown) {
        // Shift+drag: turn the viewer's head — rotate the look direction
        // while keeping the EYE position fixed (first-person look), rather
        // than orbiting around the target. The Camera struct only stores
        // (target, distance, yaw, pitch) with position() derived from
        // those, so "keep the eye fixed" is implemented by recomputing
        // target after the yaw/pitch change so the eye stays where it was:
        // target_new = eye_old - offset(newYaw, newPitch).
        glm::vec3 eye = s->camera->position();
        s->camera->yaw   -= static_cast<float>(dx) * 0.005f;
        s->camera->pitch += static_cast<float>(dy) * 0.005f;
        s->camera->pitch = glm::clamp(s->camera->pitch, -1.55f, 1.55f);
        glm::vec3 offset(
            s->camera->distance * std::cos(s->camera->pitch) * std::sin(s->camera->yaw),
            s->camera->distance * std::sin(s->camera->pitch),
            s->camera->distance * std::cos(s->camera->pitch) * std::cos(s->camera->yaw));
        s->camera->target = eye - offset;
        s->nearFarDirty = true;
    } else if (s->rightDown || s->middleDown) {
        glm::vec3 fwd   = glm::normalize(s->camera->target - s->camera->position());
        glm::vec3 right = glm::normalize(glm::cross(fwd, glm::vec3(0, 1, 0)));
        glm::vec3 up    = glm::cross(right, fwd);

        float worldPerPixel = (2.0f * s->camera->distance *
                               std::tan(glm::radians(s->camera->fov) * 0.5f))
                              / s->viewportH;
        glm::vec3 delta = (right * static_cast<float>(-dx) +
                           up    * static_cast<float>( dy)) * worldPerPixel;
        s->camera->target += delta;
    }
}

void mouseButtonCallback(GLFWwindow* win, int button, int action, int mods) {
    InputState* s = static_cast<InputState*>(glfwGetWindowUserPointer(win));
    if (!s) return;
    s->shiftDown = (mods & GLFW_MOD_SHIFT) != 0;
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        s->leftDown = (action == GLFW_PRESS);
        if (action == GLFW_PRESS) {
            double now = glfwGetTime();
            double x, y;
            glfwGetCursorPos(win, &x, &y);
            const double DOUBLE_CLICK_INTERVAL = 0.4; // seconds — standard-ish
            const double DOUBLE_CLICK_PIXEL_TOL = 5.0; // screen-space px
            bool isDoubleClick =
                (now - s->lastClickTime) < DOUBLE_CLICK_INTERVAL &&
                std::abs(x - s->lastClickX) < DOUBLE_CLICK_PIXEL_TOL &&
                std::abs(y - s->lastClickY) < DOUBLE_CLICK_PIXEL_TOL;
            s->lastClickTime = now;
            s->lastClickX = x;
            s->lastClickY = y;
            if (isDoubleClick) {
                s->pickRequested = true;
                s->pickX = x;
                s->pickY = y;
            }
        }
    } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        s->rightDown = (action == GLFW_PRESS);
    } else if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
        s->middleDown = (action == GLFW_PRESS);
    }
    double x, y;
    glfwGetCursorPos(win, &x, &y);
    s->lastX = x;
    s->lastY = y;
}

void scrollCallback(GLFWwindow* win, double /*dx*/, double dy) {
    InputState* s = static_cast<InputState*>(glfwGetWindowUserPointer(win));
    if (!s || !s->camera) return;
    // Move forward/backward (dolly), not a distance-clamped zoom — same
    // translate-the-whole-rig mechanic as Shift+Up/Down arrow keys (see
    // keyCallback): translating target moves the eye by the same delta
    // (position() = target + a fixed yaw/pitch/distance offset), so this
    // actually moves the camera through space rather than just shrinking
    // the orbit radius toward a fixed target — which is what made the old
    // zoom feel capped (it was clamped to distance in [0.001, 1000] from
    // the target and could never move past it).
    glm::vec3 fwd = glm::normalize(s->camera->target - s->camera->position());
    float step = s->camera->distance * 0.1f * static_cast<float>(dy);
    s->camera->target += fwd * step;
    s->nearFarDirty = true;
}

void resizeCallback(GLFWwindow* win, int w, int h) {
    glViewport(0, 0, w, h);
    InputState* s = static_cast<InputState*>(glfwGetWindowUserPointer(win));
    if (s && s->camera && h > 0) {
        s->camera->aspect = static_cast<float>(w) / static_cast<float>(h);
        s->viewportW = static_cast<float>(w);
        s->viewportH = static_cast<float>(h);
    }
}

void keyCallback(GLFWwindow* win, int key, int /*sc*/, int action, int /*mods*/) {
    InputState* s = static_cast<InputState*>(glfwGetWindowUserPointer(win));
    if (!s) return;

    if (key == GLFW_KEY_LEFT_SHIFT || key == GLFW_KEY_RIGHT_SHIFT) {
        s->shiftDown = (action != GLFW_RELEASE);
    }

    // Arrow keys: translate the eye (PRESS + REPEAT for continuous motion).
    // Left/Right always strafe left/right. Up/Down translate vertically —
    // UNLESS Shift is held, in which case Up/Down move forward/backward
    // along the view direction instead.
    if (s->camera && (action == GLFW_PRESS || action == GLFW_REPEAT) &&
        (key == GLFW_KEY_UP || key == GLFW_KEY_DOWN ||
         key == GLFW_KEY_LEFT || key == GLFW_KEY_RIGHT)) {
        glm::vec3 fwd   = glm::normalize(s->camera->target - s->camera->position());
        glm::vec3 right = glm::normalize(glm::cross(fwd, glm::vec3(0, 1, 0)));
        glm::vec3 up    = glm::cross(right, fwd);
        float step = s->camera->distance * 0.05f;
        glm::vec3 delta(0.0f);
        if (key == GLFW_KEY_LEFT)  delta -= right * step;
        if (key == GLFW_KEY_RIGHT) delta += right * step;
        if (s->shiftDown) {
            if (key == GLFW_KEY_UP)   delta += fwd * step;  // forward
            if (key == GLFW_KEY_DOWN) delta -= fwd * step;  // backward
        } else {
            if (key == GLFW_KEY_UP)   delta += up * step;
            if (key == GLFW_KEY_DOWN) delta -= up * step;
        }
        // Translating target moves the eye by the same delta (position()
        // is target + a fixed yaw/pitch/distance offset), so this moves
        // the whole rig together — i.e. actually translates the eye, not
        // just the orbit pivot.
        s->camera->target += delta;
        s->nearFarDirty = true;
        return;
    }

    if (action != GLFW_PRESS) return;
    if (key == GLFW_KEY_ESCAPE) {
        glfwSetWindowShouldClose(win, GLFW_TRUE);
    }
    // All printable letter shortcuts (R, E, D, U, S, F, N, M, C, P, O, V,
    // T, B, L, H, W, A) moved to charCallback() — see gl_app.h for why
    // that's the layout-correct place for them (GLFW_KEY_* tokens here are
    // fixed to US-QWERTY physical positions, not the user's actual layout).
}

// ---------------------------------------------------------------------------
// charCallback — layout-aware letter shortcuts. See gl_app.h for the full
// explanation of why this exists separately from keyCallback.
// ---------------------------------------------------------------------------
void charCallback(GLFWwindow* win, unsigned int codepoint) {
    InputState* s = static_cast<InputState*>(glfwGetWindowUserPointer(win));
    if (!s) return;

    // NOTE: unlike the old keyCallback-based implementation (which
    // explicitly filtered to action == GLFW_PRESS only), GLFW's char
    // callback fires once per character generated, including OS-level key
    // repeat if a key is held past the repeat threshold. For the toggle
    // actions below (w, a, c, l, b, o), holding the key down past that
    // threshold will flicker the toggle repeatedly rather than firing
    // once. Minor, self-evident if it happens (tap instead of hold), and
    // not worth the extra debounce-state complexity to fully eliminate.
    unsigned int c = codepoint;
    if (c >= 'A' && c <= 'Z') c += ('a' - 'A'); // case-insensitive

    switch (c) {
    case 'h':
        s->helpUntil = glfwGetTime() + 10.0;
        break;
    case 'l':
        s->showLog = !s->showLog;
        break;
    case 'b':
        s->showTileBoxes = !s->showTileBoxes;
        break;
    case 'w':
        s->tessWireframe = !s->tessWireframe;
        std::cerr << "[wireframe] " << (s->tessWireframe ? "on" : "off") << std::endl;
        break;
    case 'g':
        s->showMasterEdges = !s->showMasterEdges;
        std::cerr << "[master-edges] " << (s->showMasterEdges ? "on" : "off") << std::endl;
        break;
    case 'a':
        s->useDisplacement = !s->useDisplacement;
        std::cerr << "[displacement] " << (s->useDisplacement ? "on" : "off")
                  << " (DEM tessellation path only)" << std::endl;
        break;
    case 'c':
        if (s->cloud && s->cloud->hasOrthoColors) {
            s->useOrthoColors = !s->useOrthoColors;
            const std::vector<float>& buf =
                s->useOrthoColors ? s->cloud->orthoColors : s->cloud->colors;
            glBindBuffer(GL_ARRAY_BUFFER, s->colorVBO);
            glBufferData(GL_ARRAY_BUFFER, buf.size() * sizeof(float),
                         buf.data(), GL_STATIC_DRAW);
            std::cerr << "[color] " << (s->useOrthoColors ? "orthophoto" : "elevation") << std::endl;
        }
        break;
    case 'n':
        s->pointSizeMul /= 1.3f;
        if (s->pointSizeMul < 0.1f) s->pointSizeMul = 0.1f;
        std::cerr << "[pointsize] " << s->pointSizeMul << "x" << std::endl;
        break;
    case 'm':
        s->pointSizeMul *= 1.3f;
        if (s->pointSizeMul > 10.0f) s->pointSizeMul = 10.0f;
        std::cerr << "[pointsize] " << s->pointSizeMul << "x" << std::endl;
        break;
    case 'r':
        if (s->camera) {
            s->camera->yaw = 0.6f;
            s->camera->pitch = 1.2f;
            s->camera->distance = 2.0f;
            s->camera->target = glm::vec3(0.0f);
            s->nearFarDirty = true;
        }
        break;
    case 'e':
        s->zScale *= 1.2f;
        if (s->zScale > 100.0f) s->zScale = 100.0f;
        s->nearFarDirty = true;
        std::cerr << "[zscale] " << s->zScale << "x" << std::endl;
        break;
    case 'd':
        s->zScale /= 1.2f;
        if (s->zScale < 0.01f) s->zScale = 0.01f;
        s->nearFarDirty = true;
        std::cerr << "[zscale] " << s->zScale << "x" << std::endl;
        break;
    case 'u':
        s->zScale = 1.0f;
        s->nearFarDirty = true;
        std::cerr << "[zscale] reset to 1.0x" << std::endl;
        break;
    case 'f':
        // Point-cloud (LAZ/COPC) density only, as originally. The 128x
        // ceiling this briefly had was specifically to help GPU
        // tessellation density escape a horizon-detail floor for the DEM
        // path — now moot here, since S/F no longer drives DEM
        // tessellation density at all (see demMaxLevel below). Reverted
        // to the original 8x ceiling, appropriate for points.
        s->density *= 1.5f;
        if (s->density > 8.0f) s->density = 8.0f;
        std::cerr << "[density] " << s->density << "x" << std::endl;
        // DEM quadtree depth ceiling — see InputState::demMaxLevel's
        // comment for why this replaced GPU-tessellation-density as what
        // S/F controls for DEM rendering. +1 level doubles the per-axis
        // patch-count ceiling (COARSE * 2^level) — a big jump per press,
        // but there's no smaller meaningful increment for a quadtree
        // depth. Clamped to 10 (COARSE=8 * 2^10 = 8192 per axis,
        // 67M patches at the absolute ceiling — deliberately generous;
        // real DEMs won't come close to fully subdividing that deep, see
        // the adaptive criteria that decide whether a cell actually needs
        // to go that far).
        s->demMaxLevel += 1;
        if (s->demMaxLevel > 10) s->demMaxLevel = 10;
        s->demMaxLevelChanged = true;
        std::cerr << "[dem-max-level] " << s->demMaxLevel << std::endl;
        break;
    case 's':
        s->density /= 1.5f;
        if (s->density < 0.1f) s->density = 0.1f;
        std::cerr << "[density] " << s->density << "x" << std::endl;
        s->demMaxLevel -= 1;
        if (s->demMaxLevel < 0) s->demMaxLevel = 0;
        s->demMaxLevelChanged = true;
        std::cerr << "[dem-max-level] " << s->demMaxLevel << std::endl;
        break;
    case 'i':
        // Decrease the point-collapsing angle (finer/stricter — more
        // subdivision, closer to the original surface). Triggers a full
        // DEM mesh rebuild — see InputState::collapseAngleChanged.
        s->pointCollapseAngleDeg /= 1.3f;
        if (s->pointCollapseAngleDeg < 0.05f) s->pointCollapseAngleDeg = 0.05f;
        s->collapseAngleChanged = true;
        std::cerr << "[collapse-angle] " << s->pointCollapseAngleDeg << "\xC2\xB0" << std::endl;
        break;
    case 'o':
        // Increase the point-collapsing angle (coarser/looser — less
        // subdivision, more triangles merged/collapsed). NOTE: this key
        // previously toggled occlusion culling; that toggle is dropped
        // here rather than kept elsewhere, since occlusion culling was
        // never actually wired into the render path and the flag had no
        // visible effect regardless of its state (specs.md §5.6) — there
        // was nothing working to preserve.
        s->pointCollapseAngleDeg *= 1.3f;
        if (s->pointCollapseAngleDeg > 30.0f) s->pointCollapseAngleDeg = 30.0f;
        s->collapseAngleChanged = true;
        std::cerr << "[collapse-angle] " << s->pointCollapseAngleDeg << "\xC2\xB0" << std::endl;
        break;
    case 'p':
        if (s->camera) {
            s->camera->ortho = !s->camera->ortho;
            s->nearFarDirty = true;
            std::cerr << "[proj] " << (s->camera->ortho ? "orthographic" : "perspective")
                      << std::endl;
        }
        break;
    case 'v':
        if (s->camera) {
            s->camera->pitch = 0.05f;
            s->nearFarDirty = true;
            std::cerr << "[camera] side view (pitch=0.05)" << std::endl;
        }
        break;
    case 't':
        if (s->camera) {
            s->camera->pitch = 1.54f;
            s->nearFarDirty = true;
            std::cerr << "[camera] top view (pitch=1.54)" << std::endl;
        }
        break;
    default:
        break;
    }
}
