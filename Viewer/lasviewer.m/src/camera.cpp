// camera.cpp — orbit camera implementation + dynamic near/far computation.
#include "camera.h"

#include <algorithm>
#include <cmath>

glm::vec3 Camera::position() const {
    return target + glm::vec3(distance * std::cos(pitch) * std::sin(yaw),
                              distance * std::sin(pitch),
                              distance * std::cos(pitch) * std::cos(yaw));
}

glm::mat4 Camera::view() const {
    return glm::lookAt(position(), target, glm::vec3(0, 1, 0));
}

glm::mat4 Camera::proj() const {
    if (ortho) {
        float h = orthoSize * (distance / 2.0f);
        float w = h * aspect;
        return glm::ortho(-w, w, -h, h, nearP, farP);
    }
    return glm::perspective(glm::radians(fov), aspect, nearP, farP);
}

float Camera::orthoHeight() const {
    return orthoSize * (distance / 2.0f);
}

// Converts a Normalized Device Coordinate point (each component in
// [-1, 1], with ndcZ derived from a depth-buffer sample as
// `depth * 2.0 - 1.0`) back to a world-space GL position. Used for
// double-click-to-focus picking — see ViewerApp::handlePick() for how ndcZ
// is obtained via glReadPixels.
glm::vec3 Camera::unproject(float ndcX, float ndcY, float ndcZ) const {
    glm::mat4 invVP = glm::inverse(proj() * view());
    glm::vec4 clip(ndcX, ndcY, ndcZ, 1.0f);
    glm::vec4 world = invVP * clip;
    if (std::abs(world.w) > 1e-8f) world /= world.w;
    return glm::vec3(world);
}

void computeNearFar(const Camera& cam, const glm::vec3& bboxMin, const glm::vec3& bboxMax,
                    float zScale, float& nearP, float& farP, float minNear) {
    glm::mat4 V = cam.view();
    glm::vec4 corners[8] = {
        {bboxMin.x, bboxMin.y * zScale, bboxMin.z, 1},
        {bboxMax.x, bboxMin.y * zScale, bboxMin.z, 1},
        {bboxMax.x, bboxMin.y * zScale, bboxMax.z, 1},
        {bboxMin.x, bboxMin.y * zScale, bboxMax.z, 1},
        {bboxMin.x, bboxMax.y * zScale, bboxMin.z, 1},
        {bboxMax.x, bboxMax.y * zScale, bboxMin.z, 1},
        {bboxMax.x, bboxMax.y * zScale, bboxMax.z, 1},
        {bboxMin.x, bboxMax.y * zScale, bboxMax.z, 1},
    };
    float minDist = 1e30f, maxDist = 1e-30f;
    int frontCount = 0;
    for (int i = 0; i < 8; ++i) {
        glm::vec4 viewPos = V * corners[i];
        float dist = -viewPos.z;
        if (dist > 0) {
            ++frontCount;
            if (dist < minDist) minDist = dist;
            if (dist > maxDist) maxDist = dist;
        }
    }

    if (frontCount < 8) {
        // Camera is inside (or intersecting) the bbox.
        float camDist = glm::length(cam.position() - cam.target);
        nearP = std::max(minNear, camDist * 0.005f);
    } else {
        nearP = std::max(minNear, minDist * 0.9f);
    }
    farP = maxDist * 1.5f + 0.01f;
}
