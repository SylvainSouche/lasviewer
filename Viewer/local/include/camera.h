// camera.h — orbit camera with perspective/ortho, dynamic near/far
#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

struct Camera {
    glm::vec3 target{0.0f};
    float distance = 2.0f;
    float yaw = 0.6f;
    float pitch = 1.2f;
    float fov = 55.0f;
    float aspect = 1.6f;
    float nearP = 0.05f;
    float farP = 1000.0f;
    bool ortho = false;
    float orthoSize = 0.7f;

    glm::vec3 position() const;
    glm::mat4 view() const;
    glm::mat4 proj() const;
    float orthoHeight() const;
    glm::vec3 unproject(float ndcX, float ndcY, float ndcZ) const;
};

struct BBoxCorners {
    glm::vec3 min, max;
    bool cameraInside = false;
};

// Compute dynamic near/far from bbox corners in view space.
// Returns {near, far}. Sets cameraInside if camera is inside the bbox.
void computeNearFar(const Camera& cam, const glm::vec3& bboxMin,
                    const glm::vec3& bboxMax, float zScale,
                    float& nearP, float& farP);
