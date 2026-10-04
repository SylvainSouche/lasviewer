// scene_test.cpp — tests of the real scene-frame and camera code (atf-c++).
// Needs glm and the GLFW header (key constants), no GL context.
#include "camera.h"
#include "camera_controller.h"
#include "gl_platform.h" // GLFW mouse button constants
#include "scene_frame.h"

#include <atf-c++.hpp>

#include <cmath>
#include <cstdio>

#define CHECK(cond) ATF_REQUIRE(cond)

static bool near(double a, double b, double tol) {
    return std::abs(a - b) <= tol;
}

// World → GL → world round trip, in Lambert-93-sized coordinates.
ATF_TEST_CASE_WITHOUT_HEAD(test_frame_roundtrip);
ATF_TEST_CASE_BODY(test_frame_roundtrip) {
    WorldBounds b;
    b.extendXY(991000.0, 6557000.0, 992000.0, 6559000.0);
    b.extendZ(746.0, 1677.0);
    SceneFrame f = SceneFrame::fromBounds(b);
    CHECK(near(f.center.x, 991500.0, 1e-9));
    CHECK(near(f.center.y, 6558000.0, 1e-9));
    const double pts[][3] = {{991000.0, 6557000.0, 746.0}, {991733.25, 6558421.5, 1203.7}};
    for (const auto& p : pts) {
        glm::vec3 gl = f.toGL(p[0], p[1], p[2]);
        glm::dvec3 w = f.toWorld(gl);
        // Float GL coordinates in [-1, 1] over a ~2.4 km scene: sub-mm.
        CHECK(near(w.x, p[0], 1e-3));
        CHECK(near(w.y, p[1], 1e-3));
        CHECK(near(w.z, p[2], 1e-3));
    }
    // North is -Z, up is +Y.
    glm::vec3 north = f.toGL(f.center.x, f.center.y + 100.0, f.center.z);
    CHECK(north.z < 0.0f && near(north.x, 0.0, 1e-6));
    glm::vec3 up = f.toGL(f.center.x, f.center.y, f.center.z + 100.0);
    CHECK(up.y > 0.0f);
}

// Two adjacent tiles transformed by one frame share their common edge.
ATF_TEST_CASE_WITHOUT_HEAD(test_frame_shared_between_tiles);
ATF_TEST_CASE_BODY(test_frame_shared_between_tiles) {
    WorldBounds a, b, all;
    a.extendXY(991000.0, 6557000.0, 992000.0, 6558000.0);
    b.extendXY(991000.0, 6558000.0, 992000.0, 6559000.0);
    all.extendXY(a.min.x, a.min.y, a.max.x, a.max.y);
    all.extendXY(b.min.x, b.min.y, b.max.x, b.max.y);
    SceneFrame f = SceneFrame::fromBounds(all);
    glm::vec3 edgeA = f.toGL(991500.0, a.max.y, 1000.0);
    glm::vec3 edgeB = f.toGL(991500.0, b.min.y, 1000.0);
    CHECK(edgeA == edgeB);
}

// Without Z data the frame is centered at z = 0 and still valid.
ATF_TEST_CASE_WITHOUT_HEAD(test_frame_without_z);
ATF_TEST_CASE_BODY(test_frame_without_z) {
    WorldBounds b;
    b.extendXY(0.0, 0.0, 300.0, 400.0);
    SceneFrame f = SceneFrame::fromBounds(b);
    CHECK(near(f.scale, 500.0, 1e-9));
    CHECK(near(f.center.z, 0.0, 1e-12));
    CHECK(f.zMax > f.zMin);
}

// focusOn() makes the point the pivot without moving the eye.
ATF_TEST_CASE_WITHOUT_HEAD(test_focus_keeps_eye_fixed);
ATF_TEST_CASE_BODY(test_focus_keeps_eye_fixed) {
    Camera cam;
    cam.target = glm::vec3(0.1f, -0.05f, 0.2f);
    cam.distance = 1.7f;
    cam.yaw = 0.6f;
    cam.pitch = 1.1f;
    CameraController ctl(cam);
    glm::vec3 eye = cam.position();
    glm::vec3 picked(-0.3f, 0.02f, 0.15f);
    CHECK(ctl.focusOn(picked));
    CHECK(glm::length(cam.target - picked) < 1e-6f);
    CHECK(glm::length(cam.position() - eye) < 1e-5f);
    // Picking the eye itself is refused.
    CHECK(!ctl.focusOn(cam.position()));
}

// home() centers on the bounds (with Z exaggeration applied).
ATF_TEST_CASE_WITHOUT_HEAD(test_home_frames_bounds);
ATF_TEST_CASE_BODY(test_home_frames_bounds) {
    Camera cam;
    CameraController ctl(cam);
    GLBounds b;
    b.min = glm::vec3(-0.5f, 0.1f, -0.25f);
    b.max = glm::vec3(0.5f, 0.3f, 0.25f);
    ctl.home(b, 2.0f);
    CHECK(glm::length(cam.target - glm::vec3(0.0f, 0.4f, 0.0f)) < 1e-6f);
    CHECK(cam.distance > glm::length(b.max - b.min));
    GLBounds empty;
    ctl.home(empty, 1.0f);
    CHECK(cam.target == glm::vec3(0.0f));
}

// A double click is two presses within the time and distance limits; a
// third press right after does not count as another double click.
ATF_TEST_CASE_WITHOUT_HEAD(test_double_click_detection);
ATF_TEST_CASE_BODY(test_double_click_detection) {
    Camera cam;
    CameraController ctl(cam);
    double x = 0, y = 0;
    ctl.onMouseButton(GLFW_MOUSE_BUTTON_LEFT, true, 100, 100, 1.00);
    ctl.onMouseButton(GLFW_MOUSE_BUTTON_LEFT, false, 100, 100, 1.05);
    CHECK(!ctl.takePickRequest(x, y));
    ctl.onMouseButton(GLFW_MOUSE_BUTTON_LEFT, true, 102, 101, 1.20);
    CHECK(ctl.takePickRequest(x, y));
    CHECK(x == 102 && y == 101);
    ctl.onMouseButton(GLFW_MOUSE_BUTTON_LEFT, true, 102, 101, 1.30);
    CHECK(!ctl.takePickRequest(x, y));
    // Too slow.
    ctl.onMouseButton(GLFW_MOUSE_BUTTON_LEFT, true, 10, 10, 5.0);
    ctl.onMouseButton(GLFW_MOUSE_BUTTON_LEFT, true, 10, 10, 5.6);
    CHECK(!ctl.takePickRequest(x, y));
}

// --- Navigation modes (Fly, Walk). A 100 m-scale frame, Z exaggeration 1.

namespace {
SceneFrame navFrame() {
    SceneFrame f;
    f.center = glm::dvec3(1000.0, 2000.0, 100.0);
    f.scale = 100.0;
    return f;
}
// A camera looking level toward world north (GL -Z) from world (1000, 1900, 150).
void levelCamera(Camera& cam, const SceneFrame& f) {
    cam.yaw = 0.0f;
    cam.pitch = 0.0f;
    cam.distance = 1.0f;
    glm::vec3 eye = f.toGL(1000.0, 1900.0, 150.0);
    cam.target = eye + glm::vec3(0, 0, -1) * cam.distance;
}
GroundFn slopeGround() { // z = 100 + 0.1 * (x - 1000), everywhere
    return [](double x, double, double& z) {
        z = 100.0 + 0.1 * (x - 1000.0);
        return true;
    };
}
} // namespace

// Switching modes keeps the eye and the view direction; back to Orbit, the
// orbit distance it had is restored.
ATF_TEST_CASE_WITHOUT_HEAD(test_nav_switch_keeps_eye);
ATF_TEST_CASE_BODY(test_nav_switch_keeps_eye) {
    Camera cam;
    CameraController ctl(cam);
    SceneFrame f = navFrame();
    ctl.setFrame(f);
    levelCamera(cam, f);
    cam.pitch = 0.4f;
    cam.target = f.toGL(1000.0, 1900.0, 150.0) -
                 glm::normalize(glm::vec3(0, std::sin(0.4f), std::cos(0.4f)));
    glm::vec3 eye = cam.position();
    ctl.setMode(NavMode::Fly, 1.0f, nullptr);
    CHECK(glm::length(cam.position() - eye) < 1e-5f);
    CHECK(near(cam.pitch, 0.4, 1e-6) && cam.distance < 0.02f);
    ctl.setMode(NavMode::Orbit, 1.0f, nullptr);
    CHECK(glm::length(cam.position() - eye) < 1e-5f);
    CHECK(near(cam.distance, 1.0, 1e-6));
}

// Fly: the eye moves forward at the speed; Up/Down change the speed in
// steps of x1.5, from 0 (stopped) up to the maximum.
ATF_TEST_CASE_WITHOUT_HEAD(test_fly_moves_and_speed);
ATF_TEST_CASE_BODY(test_fly_moves_and_speed) {
    Camera cam;
    CameraController ctl(cam);
    SceneFrame f = navFrame();
    ctl.setFrame(f);
    levelCamera(cam, f);
    ctl.setMode(NavMode::Fly, 1.0f, nullptr);
    glm::dvec3 before = ctl.eyeWorld(1.0f);
    ctl.update(0.2, NavInput{}, 1.0f, nullptr); // 20 m/s for 0.2 s, northward
    glm::dvec3 after = ctl.eyeWorld(1.0f);
    CHECK(near(after.y - before.y, 4.0, 1e-3) && near(after.x, before.x, 1e-3));
    ctl.onArrowKey(GLFW_KEY_UP);
    CHECK(near(ctl.flySpeed(), 30.0, 1e-9));
    for (int i = 0; i < 40; ++i) ctl.onArrowKey(GLFW_KEY_UP);
    CHECK(ctl.flySpeed() == CameraController::kFlyMaxSpeed);
    for (int i = 0; i < 40; ++i) ctl.onArrowKey(GLFW_KEY_DOWN);
    CHECK(ctl.flySpeed() == 0.0);
    ctl.onArrowKey(GLFW_KEY_UP);
    CHECK(ctl.flySpeed() == 5.0); // restarts from 5 m/s
    ctl.flyStop();
    before = ctl.eyeWorld(1.0f);
    ctl.update(1.0, NavInput{}, 1.0f, nullptr);
    CHECK(glm::length(ctl.eyeWorld(1.0f) - before) < 1e-6);
}

// Fly steering: a stick to the right turns right (yaw decreases), a stick
// above the centre climbs; inside the dead zone nothing turns.
ATF_TEST_CASE_WITHOUT_HEAD(test_fly_steering);
ATF_TEST_CASE_BODY(test_fly_steering) {
    Camera cam;
    CameraController ctl(cam);
    SceneFrame f = navFrame();
    ctl.setFrame(f);
    levelCamera(cam, f);
    ctl.setMode(NavMode::Fly, 1.0f, nullptr);
    ctl.flyStop();
    NavInput in;
    in.steering = true;
    in.stickX = 0.03f; // dead zone
    ctl.update(0.1, in, 1.0f, nullptr);
    CHECK(cam.yaw == 0.0f && cam.pitch == 0.0f);
    in.stickX = 1.0f;
    in.stickY = -1.0f; // full right, full up
    ctl.update(0.1, in, 1.0f, nullptr);
    CHECK(near(cam.yaw, -CameraController::kFlyTurnRate * 0.1, 1e-5));
    CHECK(cam.pitch < 0.0f); // looking up
    ctl.invertFlyPitch = true;
    float p = cam.pitch;
    ctl.update(0.1, in, 1.0f, nullptr);
    CHECK(cam.pitch > p);
}

// Fly never goes below kFlyClearance above the ground.
ATF_TEST_CASE_WITHOUT_HEAD(test_fly_ground_clearance);
ATF_TEST_CASE_BODY(test_fly_ground_clearance) {
    Camera cam;
    CameraController ctl(cam);
    SceneFrame f = navFrame();
    ctl.setFrame(f);
    levelCamera(cam, f);
    cam.pitch = 1.2f;        // diving
    cam.target = cam.target; // direction set through pitch below
    ctl.setMode(NavMode::Fly, 1.0f, nullptr);
    GroundFn ground = slopeGround();
    for (int i = 0; i < 100; ++i) ctl.update(0.1, NavInput{}, 1.0f, ground);
    glm::dvec3 e = ctl.eyeWorld(1.0f);
    double g = 0;
    ground(e.x, e.y, g);
    CHECK(e.z >= g + CameraController::kFlyClearance - 1e-3);
}

// Walk: the eye goes to 2 m above the ground, walks at 1.4 m/s level
// (x4 with Shift), and follows the ground; without ground the height stays.
ATF_TEST_CASE_WITHOUT_HEAD(test_walk_follows_ground);
ATF_TEST_CASE_BODY(test_walk_follows_ground) {
    Camera cam;
    CameraController ctl(cam);
    SceneFrame f = navFrame();
    ctl.setFrame(f);
    levelCamera(cam, f);
    cam.yaw = -1.5707963f; // looking east (+x), up the slope
    cam.target = f.toGL(1000.0, 1900.0, 150.0) + glm::vec3(1, 0, 0) * cam.distance;
    GroundFn ground = slopeGround();
    ctl.setMode(NavMode::Walk, 1.0f, ground);
    CHECK(ctl.walkOnGround() && cam.pitch == 0.0f);
    glm::dvec3 e = ctl.eyeWorld(1.0f);
    CHECK(near(e.z, 100.0 + 0.1 * (e.x - 1000.0) + 2.0, 1e-3));
    NavInput in;
    in.forward = true;
    ctl.update(1.0 / 8, in, 1.0f, ground); // 8 frames of a second
    for (int i = 1; i < 8; ++i) ctl.update(1.0 / 8, in, 1.0f, ground);
    glm::dvec3 e2 = ctl.eyeWorld(1.0f);
    CHECK(near(e2.x - e.x, CameraController::kWalkSpeed, 1e-3) && near(e2.y, e.y, 1e-3));
    CHECK(near(e2.z, 100.0 + 0.1 * (e2.x - 1000.0) + 2.0, 1e-3)); // still 2 m above
    in.run = true;
    ctl.update(0.25, in, 1.0f, ground);
    CHECK(near(ctl.eyeWorld(1.0f).x - e2.x, CameraController::kWalkSpeed * 4 * 0.25, 1e-3));

    // No terrain model: walks at a constant height.
    GroundFn none = [](double, double, double&) { return false; };
    double z = ctl.eyeWorld(1.0f).z;
    ctl.update(0.5, in, 1.0f, none);
    CHECK(!ctl.walkOnGround() && near(ctl.eyeWorld(1.0f).z, z, 1e-3));
}

// Walk with a Z exaggeration: the eye is 2 real meters above the ground.
ATF_TEST_CASE_WITHOUT_HEAD(test_walk_with_z_exaggeration);
ATF_TEST_CASE_BODY(test_walk_with_z_exaggeration) {
    Camera cam;
    CameraController ctl(cam);
    SceneFrame f = navFrame();
    ctl.setFrame(f);
    levelCamera(cam, f);
    GroundFn ground = slopeGround();
    ctl.setMode(NavMode::Walk, 3.0f, ground);
    glm::dvec3 e = ctl.eyeWorld(3.0f);
    CHECK(near(e.z, 100.0 + 0.1 * (e.x - 1000.0) + 2.0, 1e-3));
}

ATF_INIT_TEST_CASES(tcs) {
    ATF_ADD_TEST_CASE(tcs, test_frame_roundtrip);
    ATF_ADD_TEST_CASE(tcs, test_frame_shared_between_tiles);
    ATF_ADD_TEST_CASE(tcs, test_frame_without_z);
    ATF_ADD_TEST_CASE(tcs, test_focus_keeps_eye_fixed);
    ATF_ADD_TEST_CASE(tcs, test_home_frames_bounds);
    ATF_ADD_TEST_CASE(tcs, test_double_click_detection);
    ATF_ADD_TEST_CASE(tcs, test_nav_switch_keeps_eye);
    ATF_ADD_TEST_CASE(tcs, test_fly_moves_and_speed);
    ATF_ADD_TEST_CASE(tcs, test_fly_steering);
    ATF_ADD_TEST_CASE(tcs, test_fly_ground_clearance);
    ATF_ADD_TEST_CASE(tcs, test_walk_follows_ground);
    ATF_ADD_TEST_CASE(tcs, test_walk_with_z_exaggeration);
}
