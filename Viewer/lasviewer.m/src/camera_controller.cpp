// camera_controller.cpp — see camera_controller.h.
#include "camera_controller.h"

#include "gl_platform.h"

#include <cmath>

namespace {
constexpr float kRotateSpeed = 0.005f; // radians per pixel
constexpr float kPitchLimit = 1.55f;
constexpr double kDoubleClickInterval = 0.4;  // s
constexpr double kDoubleClickTolerance = 5.0; // px

glm::vec3 orbitOffset(float yaw, float pitch, float distance) {
    return glm::vec3(distance * std::cos(pitch) * std::sin(yaw), distance * std::sin(pitch),
                     distance * std::cos(pitch) * std::cos(yaw));
}
} // namespace

void CameraController::onMouseButton(int button, bool pressed, double x, double y, double time) {
    lastX_ = x;
    lastY_ = y;
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        left_ = pressed;
        if (pressed) {
            bool isDouble = (time - lastClickTime_) < kDoubleClickInterval &&
                            std::abs(x - lastClickX_) < kDoubleClickTolerance &&
                            std::abs(y - lastClickY_) < kDoubleClickTolerance;
            lastClickTime_ = isDouble ? -1.0 : time; // a triple click is not two doubles
            lastClickX_ = x;
            lastClickY_ = y;
            if (isDouble) {
                pickRequested_ = true;
                pickX_ = x;
                pickY_ = y;
            }
        }
    } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        right_ = pressed;
    } else if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
        middle_ = pressed;
    }
}

void CameraController::onCursor(double x, double y, float viewportH) {
    float dx = static_cast<float>(x - lastX_);
    float dy = static_cast<float>(y - lastY_);
    lastX_ = x;
    lastY_ = y;

    if (left_) {
        glm::vec3 eye = cam_.position();
        cam_.yaw -= dx * kRotateSpeed;
        cam_.pitch = glm::clamp(cam_.pitch + dy * kRotateSpeed, -kPitchLimit, kPitchLimit);
        // Head turn: move the target so that the eye stays put.
        if (shift_) cam_.target = eye - orbitOffset(cam_.yaw, cam_.pitch, cam_.distance);
    } else if (right_ || middle_) {
        glm::vec3 fwd = glm::normalize(cam_.target - cam_.position());
        glm::vec3 right = glm::normalize(glm::cross(fwd, glm::vec3(0, 1, 0)));
        glm::vec3 up = glm::cross(right, fwd);
        float worldPerPixel =
            2.0f * cam_.distance * std::tan(glm::radians(cam_.fov) * 0.5f) / viewportH;
        cam_.target += (right * -dx + up * dy) * worldPerPixel;
    }
}

void CameraController::onScroll(double dy) {
    // Translate eye and target together: no minimum distance to the target.
    glm::vec3 fwd = glm::normalize(cam_.target - cam_.position());
    cam_.target += fwd * (cam_.distance * 0.1f * static_cast<float>(dy));
}

void CameraController::onArrowKey(int key) {
    glm::vec3 fwd = glm::normalize(cam_.target - cam_.position());
    glm::vec3 right = glm::normalize(glm::cross(fwd, glm::vec3(0, 1, 0)));
    glm::vec3 up = glm::cross(right, fwd);
    float step = cam_.distance * 0.05f;
    glm::vec3 delta(0.0f);
    if (key == GLFW_KEY_LEFT) delta -= right * step;
    if (key == GLFW_KEY_RIGHT) delta += right * step;
    if (key == GLFW_KEY_UP) delta += (shift_ ? fwd : up) * step;
    if (key == GLFW_KEY_DOWN) delta -= (shift_ ? fwd : up) * step;
    cam_.target += delta;
}

void CameraController::home(const GLBounds& bounds, float zScale) {
    cam_.yaw = 0.6f;
    cam_.pitch = 1.2f;
    if (!bounds.valid()) {
        cam_.target = glm::vec3(0.0f);
        cam_.distance = 2.0f;
        return;
    }
    glm::vec3 lo = bounds.min, hi = bounds.max;
    lo.y *= zScale;
    hi.y *= zScale;
    cam_.target = (lo + hi) * 0.5f;
    cam_.distance = std::max(1.3f * glm::length(hi - lo), 1e-3f);
}

void CameraController::topView() {
    cam_.pitch = 1.54f;
}
void CameraController::sideView() {
    cam_.pitch = 0.05f;
}

bool CameraController::focusOn(const glm::vec3& p) {
    glm::vec3 toEye = cam_.position() - p;
    float dist = glm::length(toEye);
    if (dist < 1e-6f) return false;
    glm::vec3 dir = toEye / dist;
    cam_.target = p;
    cam_.distance = dist;
    cam_.pitch = std::asin(glm::clamp(dir.y, -1.0f, 1.0f));
    cam_.yaw = std::atan2(dir.x, dir.z);
    return true;
}

bool CameraController::takePickRequest(double& x, double& y) {
    if (!pickRequested_) return false;
    pickRequested_ = false;
    x = pickX_;
    y = pickY_;
    return true;
}
