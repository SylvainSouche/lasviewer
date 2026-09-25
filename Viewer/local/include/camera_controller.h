// camera_controller.h — mouse/keyboard navigation for the orbit Camera.
//
//   Left drag           orbit around the target
//   Shift + left drag   turn the head (look around, eye fixed)
//   Right/middle drag   pan in the screen plane
//   Wheel               move forward/backward (eye and target together)
//   Arrows              strafe / move up-down; Shift+Up/Down = forward/back
//   Double-click        request a pick; focusOn() then recenters on the
//                       picked point without moving the eye
#pragma once
#include "camera.h"
#include "scene_frame.h"

class CameraController {
public:
    explicit CameraController(Camera& cam) : cam_(cam) {}

    void onMouseButton(int button, bool pressed, double x, double y, double time);
    void onCursor(double x, double y, float viewportH);
    void onScroll(double dy);
    void onArrowKey(int glfwKey);
    void setShift(bool down) { shift_ = down; }

    // Frame `bounds` (GL space, Y unscaled) from the default oblique angle.
    void home(const GLBounds& bounds, float zScale);
    void topView();
    void sideView();
    // Make `p` the orbit pivot while keeping the eye where it is.
    bool focusOn(const glm::vec3& p);

    bool takePickRequest(double& x, double& y);
    bool anyButtonDown() const { return left_ || right_ || middle_; }

private:
    Camera& cam_;
    bool left_ = false, right_ = false, middle_ = false, shift_ = false;
    double lastX_ = 0.0, lastY_ = 0.0;
    double lastClickTime_ = -1.0, lastClickX_ = 0.0, lastClickY_ = 0.0;
    bool pickRequested_ = false;
    double pickX_ = 0.0, pickY_ = 0.0;
};
