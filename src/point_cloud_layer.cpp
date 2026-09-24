// point_cloud_layer.cpp — see point_cloud_layer.h.
#include "point_cloud_layer.h"
#include "geotiff.h"

#include <glm/gtc/type_ptr.hpp>
#include <imgui.h>

#include <cmath>
#include <iostream>

PointCloudLayer::PointCloudLayer(const std::string& path, const Orthophoto* ortho)
    : Layer(path.substr(path.find_last_of('/') + 1), path), ortho_(ortho) {}

PointCloudLayer::~PointCloudLayer() {
    if (vao_) glDeleteVertexArrays(1, &vao_);
    if (vboPos_) glDeleteBuffers(1, &vboPos_);
    if (vboCol_) glDeleteBuffers(1, &vboCol_);
}

bool PointCloudLayer::load(const SceneFrame& frame) {
    if (!loadPointCloud(path(), frame, cloud_)) return false;
    if (ortho_) {
        colorizeFromOrthophoto(cloud_, *ortho_);
        useOrtho_ = cloud_.hasOrthoColors;
    }

    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);
    glGenBuffers(1, &vboPos_);
    glBindBuffer(GL_ARRAY_BUFFER, vboPos_);
    glBufferData(GL_ARRAY_BUFFER, cloud_.positions.size() * sizeof(float),
                 cloud_.positions.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glGenBuffers(1, &vboCol_);
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, vboCol_);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glBindVertexArray(0);
    setUseOrtho(useOrtho_);

    // Positions stay on the CPU only for bounds; release them.
    cloud_.positions.clear();
    cloud_.positions.shrink_to_fit();
    return true;
}

void PointCloudLayer::setUseOrtho(bool useOrtho) {
    useOrtho_ = useOrtho && cloud_.hasOrthoColors;
    const std::vector<float>& buf = useOrtho_ ? cloud_.orthoColors : cloud_.colors;
    glBindBuffer(GL_ARRAY_BUFFER, vboCol_);
    glBufferData(GL_ARRAY_BUFFER, buf.size() * sizeof(float), buf.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

GLBounds PointCloudLayer::bounds() const {
    GLBounds b;
    if (cloud_.pointCount == 0) return b;
    b.min = glm::vec3(cloud_.glBBoxMin.x, cloud_.glBBoxMinY, cloud_.glBBoxMin.y);
    b.max = glm::vec3(cloud_.glBBoxMax.x, cloud_.glBBoxMaxY, cloud_.glBBoxMax.y);
    return b;
}

void PointCloudLayer::render(const RenderContext& ctx) {
    if (!vao_) return;
    const ViewSettings& vs = *ctx.settings;
    GLuint prog = ctx.programs->point;
    glUseProgram(prog);
    glUniformMatrix4fv(glGetUniformLocation(prog, "uView"), 1, GL_FALSE, glm::value_ptr(ctx.view));
    glUniformMatrix4fv(glGetUniformLocation(prog, "uProj"), 1, GL_FALSE, glm::value_ptr(ctx.proj));
    glUniform1f(glGetUniformLocation(prog, "uPointSize"), 0.0015f * vs.pointSizeMul);
    glUniform1f(glGetUniformLocation(prog, "uViewportH"), ctx.viewportH);
    glUniform1f(glGetUniformLocation(prog, "uZScale"), vs.zScale);
    glUniform1f(glGetUniformLocation(prog, "uTargetPixelSpacing"), 2.83f);
    glUniform1f(glGetUniformLocation(prog, "uTanHalfFov"), std::tan(glm::radians(ctx.fovDeg) * 0.5f));
    glUniform1f(glGetUniformLocation(prog, "uDensity"), cloud_.glDensity);
    glUniform1f(glGetUniformLocation(prog, "uDensityMul"), vs.pointDensityMul);
    glUniform1f(glGetUniformLocation(prog, "uDisableSubsampling"), 0.0f);
    glUniform1f(glGetUniformLocation(prog, "uOrtho"), ctx.ortho ? 1.0f : 0.0f);
    glUniform1f(glGetUniformLocation(prog, "uOrthoHeight"), ctx.orthoHeight);
    glBindVertexArray(vao_);
    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(cloud_.pointCount));
    glBindVertexArray(0);
}

void PointCloudLayer::drawUI() {
    if (cloud_.hasOrthoColors) {
        int mode = useOrtho_ ? 1 : 0;
        const char* base = cloud_.hasRGB ? "File RGB" : "Elevation";
        bool changed = ImGui::RadioButton(base, &mode, 0);
        ImGui::SameLine();
        changed |= ImGui::RadioButton("Orthophoto", &mode, 1);
        if (changed) setUseOrtho(mode == 1);
    } else {
        ImGui::TextDisabled("Colors: %s", cloud_.hasRGB ? "file RGB" : "elevation");
    }
}

std::string PointCloudLayer::status() const {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.2fM pts", cloud_.pointCount / 1e6);
    return buf;
}

void PointCloudLayer::handleAction(LayerAction a) {
    if (a == LayerAction::ToggleColors && cloud_.hasOrthoColors) setUseOrtho(!useOrtho_);
}
