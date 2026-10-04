// camera_controller.h — mouse/keyboard navigation for the Camera, in three
// modes (NavMode):
//
// Orbit (GIS/CAD), event-driven:
//   Left drag           orbit around the target
//   Shift + left drag   turn the head (look around, eye fixed)
//   Right/middle drag   pan in the screen plane
//   Wheel               move forward/backward (eye and target together)
//   Arrows              strafe / move up-down; Shift+Up/Down = forward/back
//   Double-click        request a pick; focusOn() then recenters on the
//                       picked point without moving the eye
// Fly, continuous (update() every frame): the eye moves forward at flySpeed();
//   Up / Down           faster / slower (onArrowKey); Space stops (flyStop)
//   Left button held    steer: the cursor's offset from the view centre is a
//                       stick (turn rate grows with the offset; dead zone)
//   The eye never goes below kFlyClearance above the ground.
// Walk, continuous: the eye stays walkEyeHeight above the ground (DTM)
//   Left drag           turn the head
//   Up / Down           walk forward / back (Shift: run); Left / Right: step aside
// Switching (setMode) keeps the eye where it is (Walk puts it on the ground).
#pragma once
#include "camera.h"
#include "scene_frame.h"

#include <functional>

enum class NavMode { Orbit, Fly, Walk };

// Ground elevation (scene CRS, meters) under world point (x, y); false where
// there is no terrain model.
using GroundFn = std::function<bool(double x, double y, double& z)>;

// Continuous input for Fly and Walk, sampled once per frame.
struct NavInput {
    bool forward = false, back = false, left = false, right = false;
    bool run = false;             // Walk: Shift
    bool steering = false;        // Fly: left button held over the 3D view
    float stickX = 0, stickY = 0; // Fly: cursor offset from the view centre, -1..1 (y down)
};

class CameraController {
  public:
    explicit CameraController(Camera& cam) : cam_(cam) {}

    // Meters <-> GL units for Fly and Walk speeds and heights.
    void setFrame(const SceneFrame& frame) { frame_ = frame; }

    NavMode mode() const { return mode_; }
    void setMode(NavMode mode, float zScale, const GroundFn& ground);
    // Fly and Walk: advance by dt seconds.
    void update(double dt, const NavInput& in, float zScale, const GroundFn& ground);

    double flySpeed() const { return flySpeed_; } // m/s
    void flyStop() { flySpeed_ = 0.0; }
    bool invertFlyPitch = false; // false: cursor above centre = climb
    double walkEyeHeight = 2.0;  // m above the ground
    // Walk: whether the last update found ground under the eye.
    bool walkOnGround() const { return onGround_; }
    // The eye in the scene CRS (meters).
    glm::dvec3 eyeWorld(float zScale) const;

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

    static constexpr float kPitchLimit = 1.55f;  // rad, about 89°
    static constexpr double kWalkSpeed = 1.4;    // m/s (x4 running)
    static constexpr double kFlyClearance = 1.0; // m above the ground, at least
    static constexpr double kFlyTurnRate = 1.2;  // rad/s at full stick
    static constexpr double kFlyDefaultSpeed = 20.0, kFlyMaxSpeed = 2000.0; // m/s

  private:
    glm::vec3 forward() const; // viewing direction (GL, unit)
    void placeEye(const glm::vec3& eye);
    bool groundGL(const glm::vec3& eye, float zScale, const GroundFn& ground, float& y) const;

    Camera& cam_;
    SceneFrame frame_;
    NavMode mode_ = NavMode::Orbit;
    double flySpeed_ = kFlyDefaultSpeed;
    float orbitDistance_ = 1.0f; // restored when returning to Orbit
    bool onGround_ = false;
    bool left_ = false, right_ = false, middle_ = false, shift_ = false;
    double lastX_ = 0.0, lastY_ = 0.0;
    double lastClickTime_ = -1.0, lastClickX_ = 0.0, lastClickY_ = 0.0;
    bool pickRequested_ = false;
    double pickX_ = 0.0, pickY_ = 0.0;
};
