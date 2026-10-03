// viewer_ui.cpp — ImGui panels: layers, view settings, log, help.
#include "dem_layer.h"
#include "log_capture.h"
#include "raster.h"
#include "viewer_app.h"

#include <imgui.h>

#include <cstdio>

namespace {

const char* kHelpRows[][2] = {
    {"Left drag", "Orbit"},
    {"Shift + left drag", "Look around (eye fixed)"},
    {"Right / middle drag", "Pan"},
    {"Wheel", "Move forward / back"},
    {"Double-click", "Focus on the point under the cursor"},
    {"Arrows", "Move; Shift+Up/Down = forward/back"},
    {"R", "Reset view"},
    {"T / V", "Top / side view"},
    {"P", "Perspective / orthographic"},
    {"E / D / U", "Z exaggeration up / down / reset"},
    {"F / S", "Finer / coarser (point density, DEM depth)"},
    {"M / N", "Bigger / smaller points"},
    {"C", "Toggle colors (orthophoto / elevation)"},
    {"I / O", "DEM collapsing angle finer / coarser"},
    {"W / A / G", "DEM wireframe / displacement / patch edges"},
    {"B", "COPC tile boxes"},
    {"Tab", "Show / hide the side panel"},
    {"L / H", "Log / this help"},
    {"Esc", "Quit"},
};

} // namespace

void ViewerApp::drawUI() {
    if (ui_.showPanel) drawMainPanel();
    if (ui_.showLog) drawLogWindow();
    if (ui_.showHelp) drawHelpWindow();
}

void ViewerApp::drawMainPanel() {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + 10, vp->WorkPos.y + 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(340, 0), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("lasviewer", &ui_.showPanel)) {
        ImGui::End();
        return;
    }

    if (ImGui::CollapsingHeader("Layers", ImGuiTreeNodeFlags_DefaultOpen)) {
        for (size_t i = 0; i < scene_.layers.size(); ++i) {
            Layer& layer = *scene_.layers[i];
            ImGui::PushID(static_cast<int>(i));
            ImGui::Checkbox("##visible", &layer.visible);
            ImGui::SameLine();
            bool open = ImGui::TreeNodeEx("##node", ImGuiTreeNodeFlags_DefaultOpen, "%s  [%s]",
                                          layer.name().c_str(), layer.kind());
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("%s%s", layer.path().c_str(),
                                  layer.epsg ? ("\nEPSG:" + std::to_string(layer.epsg)).c_str()
                                             : "");
            }
            if (open) {
                std::string status = layer.status();
                if (!status.empty()) {
                    if (layer.busy())
                        ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%s", status.c_str());
                    else
                        ImGui::TextDisabled("%s", status.c_str());
                }
                layer.drawUI();
                ImGui::TreePop();
            }
            ImGui::PopID();
        }
        if (scene_.ortho) {
            ImGui::Separator();
            std::string name = scene_.orthoPath.substr(scene_.orthoPath.find_last_of('/') + 1);
            ImGui::TextDisabled("Orthophoto: %s", name.c_str());
            ImGui::TextDisabled(
                "  %dx%d px%s", scene_.ortho->width, scene_.ortho->height,
                scene_.ortho->epsg ? (", EPSG:" + std::to_string(scene_.ortho->epsg)).c_str() : "");
        }
    }

    if (ImGui::CollapsingHeader("View", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("Reset")) resetView();
        ImGui::SameLine();
        if (ImGui::Button("Top")) controller_.topView();
        ImGui::SameLine();
        if (ImGui::Button("Side")) controller_.sideView();
        ImGui::SameLine();
        int proj = camera_.ortho ? 1 : 0;
        if (ImGui::RadioButton("Persp", &proj, 0)) camera_.ortho = false;
        ImGui::SameLine();
        if (ImGui::RadioButton("Ortho", &proj, 1)) camera_.ortho = true;

        ImGui::SetNextItemWidth(160);
        ImGui::SliderFloat("Z exaggeration", &settings_.zScale, 0.1f, 20.0f, "%.2fx",
                           ImGuiSliderFlags_Logarithmic);
        ImGui::SameLine();
        if (ImGui::SmallButton("1x")) settings_.zScale = 1.0f;
        ImGui::SetNextItemWidth(160);
        ImGui::SliderFloat("Point size", &settings_.pointSizeMul, 0.1f, 10.0f, "%.2fx",
                           ImGuiSliderFlags_Logarithmic);
        ImGui::SetNextItemWidth(160);
        ImGui::SliderFloat("Point density", &settings_.pointDensityMul, 0.1f, 8.0f, "%.2fx",
                           ImGuiSliderFlags_Logarithmic);
        bool anyAbove = false;
        for (const auto& l : scene_.layers) {
            const auto* d = dynamic_cast<const DemLayer*>(l.get());
            anyAbove |= d && d->role() == DemRole::AboveGround;
        }
        if (anyAbove) {
            ImGui::SetNextItemWidth(160);
            ImGui::SliderFloat("Height threshold", &settings_.heightThreshold, 0.1f, 50.0f,
                               "%.1f m", ImGuiSliderFlags_Logarithmic);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Above-ground surfaces (DHM/DSM) are shown only above this "
                                  "height;\nthe DTM has no orthophoto under them");
        }
        ImGui::Checkbox("Occlusion culling", &settings_.useOcclusion);
        ImGui::SameLine();
        ImGui::Checkbox("Tile boxes", &settings_.showTileBoxes);
    }

    if (ImGui::CollapsingHeader("Info")) {
        ImGui::Text("%.0f fps", fps_);
        glm::dvec3 t = scene_.frame.toWorld(
            glm::vec3(camera_.target.x, camera_.target.y / settings_.zScale, camera_.target.z));
        ImGui::Text("Target  %.2f  %.2f  %.2f", t.x, t.y, t.z);
        ImGui::Text("Distance  %.1f m", camera_.distance * scene_.frame.scale);
        if (hasPick_)
            ImGui::Text("Picked  %.2f  %.2f  %.2f", lastPick_.x, lastPick_.y, lastPick_.z);
        else
            ImGui::TextDisabled("Double-click a point to pick it");
    }

    ImGui::Separator();
    if (ImGui::SmallButton(ui_.showLog ? "Hide log" : "Log")) ui_.showLog = !ui_.showLog;
    ImGui::SameLine();
    if (ImGui::SmallButton("Help")) ui_.showHelp = !ui_.showHelp;
    ImGui::SameLine();
    ImGui::TextDisabled("Tab hides this panel");
    ImGui::End();
}

void ViewerApp::drawLogWindow() {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + 10, vp->WorkPos.y + vp->WorkSize.y - 260),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(vp->WorkSize.x * 0.6f, 250), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Log", &ui_.showLog)) {
        ImGui::End();
        return;
    }
    size_t total = logLineCount();
    std::vector<std::string> lines = logSnapshot();
    ImGui::BeginChild("##lines", ImVec2(0, 0), ImGuiChildFlags_None,
                      ImGuiWindowFlags_HorizontalScrollbar);
    ImGuiListClipper clipper;
    clipper.Begin(static_cast<int>(lines.size()));
    while (clipper.Step()) {
        for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i) {
            const std::string& l = lines[static_cast<size_t>(i)];
            bool isError = l.rfind("ERROR", 0) == 0;
            bool isWarn = l.rfind("WARNING", 0) == 0;
            if (isError)
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.4f, 0.4f, 1.0f));
            else if (isWarn)
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.8f, 0.3f, 1.0f));
            ImGui::TextUnformatted(l.c_str());
            if (isError || isWarn) ImGui::PopStyleColor();
        }
    }
    // Follow new lines unless the user scrolled up.
    if (total != ui_.logLinesSeen && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 1.0f)
        ImGui::SetScrollHereY(1.0f);
    ui_.logLinesSeen = total;
    ImGui::EndChild();
    ImGui::End();
}

void ViewerApp::drawHelpWindow() {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(
        ImVec2(vp->WorkPos.x + vp->WorkSize.x * 0.5f, vp->WorkPos.y + vp->WorkSize.y * 0.5f),
        ImGuiCond_FirstUseEver, ImVec2(0.5f, 0.5f));
    if (!ImGui::Begin("Controls", &ui_.showHelp, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::End();
        return;
    }
    if (ImGui::BeginTable("##keys", 2, ImGuiTableFlags_RowBg)) {
        for (const auto& row : kHelpRows) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(row[0]);
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(row[1]);
        }
        ImGui::EndTable();
    }
    ImGui::End();
}
