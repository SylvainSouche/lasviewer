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

ATF_INIT_TEST_CASES(tcs) {
    ATF_ADD_TEST_CASE(tcs, test_frame_roundtrip);
    ATF_ADD_TEST_CASE(tcs, test_frame_shared_between_tiles);
    ATF_ADD_TEST_CASE(tcs, test_frame_without_z);
    ATF_ADD_TEST_CASE(tcs, test_focus_keeps_eye_fixed);
    ATF_ADD_TEST_CASE(tcs, test_home_frames_bounds);
    ATF_ADD_TEST_CASE(tcs, test_double_click_detection);
}
