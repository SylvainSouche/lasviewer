// camera_controller.cpp — see camera_controller.h.
#include "camera_controller.h"

#include "gl_platform.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr float kRotateSpeed = 0.005f;        // radians per pixel
constexpr double kDoubleClickInterval = 0.4;  // s
constexpr double kDoubleClickTolerance = 5.0; // px

glm::vec3 orbitOffset(float yaw, float pitch, float distance) {
    return glm::vec3(distance * std::cos(pitch) * std::sin(yaw), distance * std::sin(pitch),
                     distance * std::cos(pitch) * std::cos(yaw));
}

// Fly stick: a dead zone around the centre, then linear up to full rate.
float stickShape(float v) {
    constexpr float kDeadZone = 0.05f;
    float a = std::abs(v);
    if (a <= kDeadZone) return 0.0f;
    return std::copysign(std::min((a - kDeadZone) / (1.0f - kDeadZone), 1.0f), v);
}
} // namespace

glm::vec3 CameraController::forward() const {
    return -glm::normalize(orbitOffset(cam_.yaw, cam_.pitch, 1.0f));
}

// The camera looks at a target 1 m ahead of the eye in Fly and Walk.
void CameraController::placeEye(const glm::vec3& eye) {
    cam_.target = eye + forward() * cam_.distance;
}

bool CameraController::groundGL(const glm::vec3& eye, float zScale, const GroundFn& ground,
                                float& y) const {
    if (!ground) return false;
    glm::dvec3 w = frame_.toWorld(glm::vec3(eye.x, eye.y / zScale, eye.z));
    double z = 0.0;
    if (!ground(w.x, w.y, z)) return false;
    y = static_cast<float>((z - frame_.center.z) / frame_.scale * zScale);
    return true;
}

glm::dvec3 CameraController::eyeWorld(float zScale) const {
    glm::vec3 eye = cam_.position();
    return frame_.toWorld(glm::vec3(eye.x, eye.y / zScale, eye.z));
}

void CameraController::setMode(NavMode mode, float zScale, const GroundFn& ground) {
    if (mode == mode_) return;
    glm::vec3 eye = cam_.position();
    if (mode_ == NavMode::Orbit) orbitDistance_ = cam_.distance;
    mode_ = mode;
    if (mode == NavMode::Orbit) {
        // Same eye, same direction: the target goes back to the distance it
        // had when Orbit was left.
        cam_.distance = std::max(orbitDistance_, 1e-3f);
        placeEye(eye);
        return;
    }
    cam_.ortho = false;
    cam_.distance = static_cast<float>(1.0 / frame_.scale);
    if (mode == NavMode::Walk) {
        cam_.pitch = 0.0f; // level gaze
        float g = 0.0f;
        onGround_ = groundGL(eye, zScale, ground, g);
        if (onGround_) eye.y = g + static_cast<float>(walkEyeHeight / frame_.scale) * zScale;
    }
    placeEye(eye);
}

void CameraController::update(double dt, const NavInput& in, float zScale, const GroundFn& ground) {
    if (mode_ == NavMode::Orbit || dt <= 0.0) return;
    dt = std::min(dt, 0.25); // a stalled frame must not jump
    const float perMeter = static_cast<float>(1.0 / frame_.scale);
    glm::vec3 eye = cam_.position();
    if (mode_ == NavMode::Fly) {
        if (in.steering) {
            float turn = static_cast<float>(kFlyTurnRate * dt);
            cam_.yaw -= stickShape(in.stickX) * turn;
            // Cursor below the centre (stickY > 0) looks further down: dive.
            float pitchStick = stickShape(in.stickY) * (invertFlyPitch ? -1.0f : 1.0f);
            cam_.pitch = glm::clamp(cam_.pitch + pitchStick * turn, -kPitchLimit, kPitchLimit);
        }
        eye += forward() * static_cast<float>(flySpeed_ * dt) * perMeter;
        float g = 0.0f;
        if (groundGL(eye, zScale, ground, g))
            eye.y = std::max(eye.y, g + static_cast<float>(kFlyClearance) * perMeter * zScale);
    } else {
        glm::vec3 f(-std::sin(cam_.yaw), 0.0f, -std::cos(cam_.yaw)); // level forward
        glm::vec3 r = glm::normalize(glm::cross(f, glm::vec3(0, 1, 0)));
        float step = static_cast<float>(kWalkSpeed * (in.run ? 4.0 : 1.0) * dt) * perMeter;
        eye += (f * (float(in.forward) - float(in.back)) + r * (float(in.right) - float(in.left))) *
               step;
        float g = 0.0f;
        onGround_ = groundGL(eye, zScale, ground, g);
        if (onGround_) eye.y = g + static_cast<float>(walkEyeHeight) * perMeter * zScale;
    }
    placeEye(eye);
}

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

    if (mode_ == NavMode::Walk) {
        // Turn the head; the eye stays.
        if (left_) {
            glm::vec3 eye = cam_.position();
            cam_.yaw -= dx * kRotateSpeed;
            cam_.pitch = glm::clamp(cam_.pitch + dy * kRotateSpeed, -kPitchLimit, kPitchLimit);
            placeEye(eye);
        }
        return;
    }
    if (mode_ == NavMode::Fly) return; // steering is sampled by update()
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
    if (mode_ == NavMode::Fly) {
        onArrowKey(dy > 0 ? GLFW_KEY_UP : GLFW_KEY_DOWN);
        return;
    }
    if (mode_ == NavMode::Walk) return;
    // Translate eye and target together: no minimum distance to the target.
    glm::vec3 fwd = glm::normalize(cam_.target - cam_.position());
    cam_.target += fwd * (cam_.distance * 0.1f * static_cast<float>(dy));
}

void CameraController::onArrowKey(int key) {
    if (mode_ == NavMode::Fly) {
        if (key == GLFW_KEY_UP)
            flySpeed_ = std::min(flySpeed_ < 1.0 ? 5.0 : flySpeed_ * 1.5, kFlyMaxSpeed);
        if (key == GLFW_KEY_DOWN) flySpeed_ = flySpeed_ / 1.5 < 1.0 ? 0.0 : flySpeed_ / 1.5;
        return;
    }
    if (mode_ == NavMode::Walk) return; // sampled by update()
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
    mode_ = NavMode::Orbit;
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
    if (mode_ == NavMode::Orbit) cam_.pitch = 1.54f;
}
void CameraController::sideView() {
    if (mode_ == NavMode::Orbit) cam_.pitch = 0.05f;
}

bool CameraController::focusOn(const glm::vec3& p) {
    if (mode_ != NavMode::Orbit) return false;
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
