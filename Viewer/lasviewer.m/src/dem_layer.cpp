// dem_layer.cpp — see dem_layer.h.
#include "dem_layer.h"

#include <imgui.h>

#include <algorithm>
#include <cstdio>
#include <iostream>

DemLayer::DemLayer(const std::string& path, const Orthophoto* ortho, DemSource source, DemRole role,
                   const char* kindName)
    : Layer(path.substr(path.find_last_of('/') + 1), path), ortho_(ortho),
      source_(std::move(source)), role_(role), kindName_(kindName) {
    if (role_ != DemRole::AboveGround) opacity_ = 1.0f;
}

DemLayer::~DemLayer() {
    mesh_.destroy();
}

bool DemLayer::load(const SceneFrame& frame) {
    frame_ = frame;
    // maxLevel 0 (the coarse grid only) is fast whatever the DEM size.
    if (!mesh_.loadFromDEM(source_, ortho_, frame_, collapseAngleDeg_, 0) ||
        !mesh_.uploadGPU(ortho_)) {
        return false;
    }
    requestRebuild();
    return true;
}

void DemLayer::requestRebuild() {
    mesh_.requestBackgroundBuild(source_, ortho_, frame_, collapseAngleDeg_, maxLevel_);
}

GLBounds DemLayer::bounds() const {
    GLBounds b;
    if (!mesh_.valid) return b;
    b.min = glm::vec3(mesh_.glBBoxMin.x, mesh_.glBBoxMinY, mesh_.glBBoxMin.y);
    b.max = glm::vec3(mesh_.glBBoxMax.x, mesh_.glBBoxMaxY, mesh_.glBBoxMax.y);
    return b;
}

void DemLayer::update(const RenderContext&) {
    mesh_.pollBackgroundBuild(ortho_);
}

DemStyle DemLayer::style(const RenderContext& ctx) const {
    DemStyle st;
    st.opacity = opacity_;
    st.shade = shade_;
    st.threshold = ctx.settings->heightThreshold;
    if (role_ == DemRole::AboveGround) {
        st.auxMode = DemAux::Ground;
    } else if (role_ == DemRole::Ground) {
        bool anyVisible = false;
        for (const Layer* l : above_) anyVisible |= l->visible;
        if (anyVisible) st.auxMode = DemAux::HeightAbove;
    }
    return st;
}

void DemLayer::render(const RenderContext& ctx) {
    if (!ctx.programs->tess) return;
    DemStyle st = style(ctx);
    auto draw = [&] {
        mesh_.render(ctx.programs->tess, ctx.view, ctx.proj, ctx.camPos, ctx.fovDeg, ctx.viewportH,
                     ctx.settings->zScale, pixelsPerSegment_, displacement_, masterEdges_, st);
    };
    if (wireframe_) glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    if (st.opacity < 1.0f) {
        // Depth pre-pass, then blend only the nearest layer of this surface:
        // without it, the far sides of objects would show through the near
        // ones in whatever order the patches happen to be drawn.
        glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
        draw();
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        glDepthMask(GL_FALSE);
        glDepthFunc(GL_LEQUAL);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        draw();
        glDisable(GL_BLEND);
        glDepthFunc(GL_LESS);
        glDepthMask(GL_TRUE);
    } else {
        draw();
    }
    if (wireframe_) glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void DemLayer::drawUI() {
    if (role_ == DemRole::AboveGround) {
        ImGui::SetNextItemWidth(140);
        ImGui::SliderFloat("Opacity", &opacity_, 0.05f, 1.0f, "%.2f");
    }
    if (mesh_.colorTex) {
        int mode = mesh_.showTexture ? 1 : 0;
        bool changed = ImGui::RadioButton("Elevation", &mode, 0);
        ImGui::SameLine();
        changed |= ImGui::RadioButton("Orthophoto", &mode, 1);
        if (changed) mesh_.showTexture = (mode == 1);
    } else {
        ImGui::TextDisabled("Colors: elevation%s",
                            (ortho_ && !mesh_.orthoUsable) ? " (ortho does not overlap)" : "");
    }

    // Rebuild when a slider is released, not on every value while dragging.
    ImGui::SetNextItemWidth(140);
    ImGui::SliderInt("Max level", &maxLevel_, 0, 10);
    if (ImGui::IsItemDeactivatedAfterEdit()) requestRebuild();
    ImGui::SetNextItemWidth(140);
    ImGui::SliderFloat("Collapse angle", &collapseAngleDeg_, 0.05f, 30.0f, "%.2f deg",
                       ImGuiSliderFlags_Logarithmic);
    if (ImGui::IsItemDeactivatedAfterEdit()) requestRebuild();
    ImGui::SetNextItemWidth(140);
    ImGui::SliderFloat("Px / segment", &pixelsPerSegment_, 2.0f, 32.0f, "%.1f");
    ImGui::Checkbox("Wireframe", &wireframe_);
    ImGui::SameLine();
    ImGui::Checkbox("Displace", &displacement_);
    ImGui::SameLine();
    ImGui::Checkbox("Patch edges", &masterEdges_);
    ImGui::Checkbox("Shade", &shade_);
}

std::string DemLayer::status() const {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "%d patches, level %d%s", mesh_.patchCount, mesh_.maxLevel,
                  busy() ? ", rebuilding" : "");
    return buf;
}

void DemLayer::handleAction(LayerAction a) {
    switch (a) {
    case LayerAction::ToggleColors:
        mesh_.showTexture = !mesh_.showTexture;
        break;
    case LayerAction::Finer:
        maxLevel_ = std::min(maxLevel_ + 1, 10);
        requestRebuild();
        break;
    case LayerAction::Coarser:
        maxLevel_ = std::max(maxLevel_ - 1, 0);
        requestRebuild();
        break;
    case LayerAction::CollapseFiner:
        collapseAngleDeg_ = std::max(collapseAngleDeg_ / 1.3f, 0.05f);
        requestRebuild();
        break;
    case LayerAction::CollapseCoarser:
        collapseAngleDeg_ = std::min(collapseAngleDeg_ * 1.3f, 30.0f);
        requestRebuild();
        break;
    case LayerAction::ToggleWireframe:
        wireframe_ = !wireframe_;
        break;
    case LayerAction::ToggleDisplacement:
        displacement_ = !displacement_;
        break;
    case LayerAction::ToggleMasterEdges:
        masterEdges_ = !masterEdges_;
        break;
    }
}
