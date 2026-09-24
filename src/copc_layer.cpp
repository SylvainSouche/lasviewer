// copc_layer.cpp — see copc_layer.h.
#include "copc_layer.h"
#include "geotiff.h"

#include <glm/gtc/type_ptr.hpp>
#include <imgui.h>

#include <cstdio>

CopcLayer::CopcLayer(const std::string& path, const WorldBounds& bounds, uint64_t pointCount,
                     const Orthophoto* ortho)
    : Layer(path.substr(path.find_last_of('/') + 1), path),
      worldBounds_(bounds), filePointCount_(pointCount), ortho_(ortho) {}

CopcLayer::~CopcLayer() {
    grid_.reset(); // joins the loader thread, frees tile buffers
    if (boxVAO_) glDeleteVertexArrays(1, &boxVAO_);
    if (boxVBO_) glDeleteBuffers(1, &boxVBO_);
}

void CopcLayer::start(const SceneFrame& frame) {
    grid_ = std::make_unique<TileGrid>();
    grid_->init(path(), worldBounds_, ortho_, frame);
}

GLBounds CopcLayer::bounds() const {
    GLBounds b;
    if (!grid_) return b;
    for (const Tile& t : grid_->tiles) {
        GLBounds tb;
        tb.min = t.glMin;
        tb.max = t.glMax;
        b.extend(tb);
    }
    return b;
}

void CopcLayer::update(const RenderContext& ctx) {
    if (grid_) grid_->update(ctx.camPos, ctx.fovDeg, ctx.viewportH);
}

void CopcLayer::render(const RenderContext& ctx) {
    if (grid_) grid_->render(ctx);
}

void CopcLayer::renderOverlay(const RenderContext& ctx) {
    if (!grid_ || !ctx.settings->showTileBoxes) return;
    static const int kEdges[24] = {0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6,
                                   6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7};
    // Loaded tiles first (yellow), then tiles being refined (orange).
    std::vector<float> verts;
    size_t loadedVerts = 0;
    for (int pass = 0; pass < 2; ++pass) {
        for (const Tile& t : grid_->tiles) {
            if (!t.hasGeometry() || t.inFlight != (pass == 1)) continue;
            const glm::vec3& a = t.glMin;
            const glm::vec3& b = t.glMax;
            const glm::vec3 c[8] = {{a.x, a.y, a.z}, {b.x, a.y, a.z}, {b.x, a.y, b.z}, {a.x, a.y, b.z},
                                    {a.x, b.y, a.z}, {b.x, b.y, a.z}, {b.x, b.y, b.z}, {a.x, b.y, b.z}};
            for (int e : kEdges) verts.insert(verts.end(), {c[e].x, c[e].y, c[e].z});
        }
        if (pass == 0) loadedVerts = verts.size() / 3;
    }
    if (verts.empty()) return;

    if (!boxVAO_) {
        glGenVertexArrays(1, &boxVAO_);
        glGenBuffers(1, &boxVBO_);
        glBindVertexArray(boxVAO_);
        glBindBuffer(GL_ARRAY_BUFFER, boxVBO_);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    }
    glBindVertexArray(boxVAO_);
    glBindBuffer(GL_ARRAY_BUFFER, boxVBO_);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STREAM_DRAW);

    GLuint prog = ctx.programs->line;
    glUseProgram(prog);
    glUniformMatrix4fv(glGetUniformLocation(prog, "uView"), 1, GL_FALSE, glm::value_ptr(ctx.view));
    glUniformMatrix4fv(glGetUniformLocation(prog, "uProj"), 1, GL_FALSE, glm::value_ptr(ctx.proj));
    glUniform1f(glGetUniformLocation(prog, "uZScale"), ctx.settings->zScale);
    GLint colorLoc = glGetUniformLocation(prog, "uColor");
    glUniform3f(colorLoc, 1.0f, 1.0f, 0.0f);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(loadedVerts));
    size_t total = verts.size() / 3;
    if (total > loadedVerts) {
        glUniform3f(colorLoc, 1.0f, 0.5f, 0.0f);
        glDrawArrays(GL_LINES, static_cast<GLint>(loadedVerts),
                     static_cast<GLsizei>(total - loadedVerts));
    }
    glBindVertexArray(0);
}

void CopcLayer::drawUI() {
    if (!grid_) return;
    bool hasOrtho = ortho_ && ortho_->hasGeo;
    if (hasOrtho) {
        int mode = grid_->useOrthoColors ? 1 : 0;
        bool changed = ImGui::RadioButton("Elevation", &mode, 0);
        ImGui::SameLine();
        changed |= ImGui::RadioButton("Orthophoto", &mode, 1);
        if (changed) grid_->setUseOrthoColors(mode == 1);
    } else {
        ImGui::TextDisabled("Colors: elevation");
    }
    ImGui::Text("Drawn: %.2fM pts in %zu tiles (%zu culled)",
                grid_->drawnPoints / 1e6, grid_->drawnTiles, grid_->culledTiles);
    ImGui::Text("File: %.1fM pts", filePointCount_ / 1e6);
}

std::string CopcLayer::status() const {
    if (!grid_) return {};
    int loaded = 0, loading = 0;
    for (const Tile& t : grid_->tiles) {
        if (t.hasGeometry()) ++loaded;
        if (t.inFlight) ++loading;
    }
    char buf[96];
    if (loading > 0)
        std::snprintf(buf, sizeof(buf), "%d/%zu tiles, %d loading", loaded,
                      grid_->tiles.size(), loading);
    else
        std::snprintf(buf, sizeof(buf), "%.2fM pts drawn", grid_->drawnPoints / 1e6);
    return buf;
}

bool CopcLayer::busy() const {
    if (!grid_) return false;
    for (const Tile& t : grid_->tiles)
        if (t.inFlight) return true;
    return false;
}

void CopcLayer::handleAction(LayerAction a) {
    if (a == LayerAction::ToggleColors && grid_ && ortho_ && ortho_->hasGeo)
        grid_->setUseOrthoColors(!grid_->useOrthoColors);
}
