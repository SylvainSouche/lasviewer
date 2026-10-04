// viewer_ui.cpp — ImGui panels: layers, view settings, log, help.
#include "dem_layer.h"
#include "log_capture.h"
#include "raster.h"
#include "viewer_app.h"

#include <imgui.h>

#include <cstdio>

namespace {

const char* kHelpRows[][2] = {
    {"1 / 2 / 3", "Navigation: Orbit (GIS/CAD) / Fly / Walk"},
    {"Orbit:", ""},
    {"Left drag", "Orbit"},
    {"Shift + left drag", "Look around (eye fixed)"},
    {"Right / middle drag", "Pan"},
    {"Wheel", "Move forward / back"},
    {"Double-click", "Focus on the point under the cursor"},
    {"Arrows", "Move; Shift+Up/Down = forward/back"},
    {"Fly:", "the viewpoint keeps moving forward"},
    {"Up / Down, wheel", "Faster / slower"},
    {"Space", "Stop"},
    {"Hold left button", "Steer toward the cursor (like a stick)"},
    {"Walk:", "eye 2 m above the terrain model"},
    {"Up / Down", "Walk forward / back (Shift: run)"},
    {"Left / Right", "Step aside"},
    {"Left drag", "Turn the head"},
    {"All modes:", ""},
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
    drawNavHud();
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
        int nav = static_cast<int>(controller_.mode());
        ImGui::TextUnformatted("Navigation");
        ImGui::SameLine();
        if (ImGui::RadioButton("Orbit", &nav, 0)) setNavMode(NavMode::Orbit);
        ImGui::SameLine();
        if (ImGui::RadioButton("Fly", &nav, 1)) setNavMode(NavMode::Fly);
        ImGui::SameLine();
        if (ImGui::RadioButton("Walk", &nav, 2)) setNavMode(NavMode::Walk);
        if (controller_.mode() == NavMode::Fly) {
            ImGui::Text("Speed %.1f m/s", controller_.flySpeed());
            ImGui::SameLine();
            if (ImGui::SmallButton("Stop")) controller_.flyStop();
            ImGui::SameLine();
            ImGui::Checkbox("Invert pitch", &controller_.invertFlyPitch);
        } else if (controller_.mode() == NavMode::Walk) {
            float h = static_cast<float>(controller_.walkEyeHeight);
            ImGui::SetNextItemWidth(160);
            if (ImGui::SliderFloat("Eye height", &h, 0.5f, 20.0f, "%.1f m"))
                controller_.walkEyeHeight = h;
        }

        if (ImGui::Button("Reset")) resetView();
        if (controller_.mode() == NavMode::Orbit) {
            ImGui::SameLine();
            if (ImGui::Button("Top")) controller_.topView();
            ImGui::SameLine();
            if (ImGui::Button("Side")) controller_.sideView();
            ImGui::SameLine();
            int proj = camera_.ortho ? 1 : 0;
            if (ImGui::RadioButton("Persp", &proj, 0)) camera_.ortho = false;
            ImGui::SameLine();
            if (ImGui::RadioButton("Ortho", &proj, 1)) camera_.ortho = true;
        }

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

// ---------------------------------------------------------------------------
// Navigation HUD: which mode, and what the input does, while the viewpoint is
// being manipulated. Orbit shows it only then (and fades it out); Fly and Walk
// always show it, brighter while steering or walking.
// ---------------------------------------------------------------------------

void ViewerApp::drawNavHud() {
    const NavMode mode = controller_.mode();
    const double idle = lastNavInput_ < 0.0 ? 1e9 : glfwGetTime() - lastNavInput_;
    float alpha;
    if (mode == NavMode::Orbit) {
        if (idle > 2.0) return;
        alpha = idle < 1.0 ? 1.0f : static_cast<float>(2.0 - idle); // fade out
    } else {
        alpha = idle < 1.0 ? 1.0f : 0.55f;
    }

    static const ImVec4 kColor[] = {{0.35f, 0.65f, 1.0f, 1.0f}, // Orbit: blue
                                    {1.0f, 0.62f, 0.2f, 1.0f},  // Fly: orange
                                    {0.4f, 0.85f, 0.4f, 1.0f}}; // Walk: green
    const ImVec4 color = kColor[static_cast<int>(mode)];
    const ImU32 colorU32 = ImGui::GetColorU32(ImVec4(color.x, color.y, color.z, alpha));

    // Height above the terrain, when there is terrain under the eye.
    glm::dvec3 eye = controller_.eyeWorld(settings_.zScale);
    double groundZ = 0.0;
    bool hasGround = mode != NavMode::Orbit && groundAt(eye.x, eye.y, groundZ);

    char title[64], detail[128], hint[160];
    switch (mode) {
    case NavMode::Orbit:
        std::snprintf(title, sizeof title, "ORBIT");
        std::snprintf(detail, sizeof detail, "GIS / CAD view");
        std::snprintf(hint, sizeof hint,
                      "drag: orbit   shift+drag: look   right drag: pan   "
                      "wheel: forward   2: fly   3: walk");
        break;
    case NavMode::Fly:
        std::snprintf(title, sizeof title, "FLY");
        if (hasGround)
            std::snprintf(detail, sizeof detail, "%.1f m/s   %.0f m above ground",
                          controller_.flySpeed(), eye.z - groundZ);
        else
            std::snprintf(detail, sizeof detail, "%.1f m/s", controller_.flySpeed());
        std::snprintf(hint, sizeof hint,
                      "up/down: speed   space: stop   hold left button: "
                      "steer   1: orbit   3: walk");
        break;
    case NavMode::Walk:
        std::snprintf(title, sizeof title, "WALK");
        if (controller_.walkOnGround())
            std::snprintf(detail, sizeof detail, "eye %.1f m above the terrain",
                          controller_.walkEyeHeight);
        else
            std::snprintf(detail, sizeof detail, "no terrain model here: constant height");
        std::snprintf(hint, sizeof hint,
                      "up/down: walk (shift: run)   left/right: step aside   "
                      "drag: look   1: orbit   2: fly");
        break;
    }

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(
        ImVec2(vp->WorkPos.x + vp->WorkSize.x * 0.5f, vp->WorkPos.y + vp->WorkSize.y - 16.0f),
        ImGuiCond_Always, ImVec2(0.5f, 1.0f));
    ImGui::SetNextWindowBgAlpha(0.55f * alpha);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 2.0f);
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(color.x, color.y, color.z, alpha));
    ImGui::Begin("##navhud", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                     ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoSavedSettings |
                     ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav);
    ImGui::SetWindowFontScale(1.5f);
    ImGui::TextColored(ImVec4(color.x, color.y, color.z, alpha), "%s", title);
    ImGui::SetWindowFontScale(1.0f);
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(1, 1, 1, alpha), "  %s", detail);
    ImGui::TextColored(ImVec4(0.75f, 0.75f, 0.75f, alpha), "%s", hint);
    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar();

    // Reticles, over the 3D view (behind the UI windows).
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    ImVec2 c(vp->Pos.x + vp->Size.x * 0.5f, vp->Pos.y + vp->Size.y * 0.5f);
    if (mode == NavMode::Fly) {
        float r = std::min(vp->Size.x, vp->Size.y) * 0.5f * 0.05f; // the stick's dead zone
        dl->AddCircle(c, r, colorU32, 32, 1.5f);
        dl->AddLine(ImVec2(c.x - 2.5f * r, c.y), ImVec2(c.x - r, c.y), colorU32, 1.5f);
        dl->AddLine(ImVec2(c.x + r, c.y), ImVec2(c.x + 2.5f * r, c.y), colorU32, 1.5f);
        if (controller_.anyButtonDown()) { // steering: the stick
            ImVec2 m = ImGui::GetIO().MousePos;
            dl->AddLine(c, m, colorU32, 2.0f);
            dl->AddCircleFilled(m, 4.0f, colorU32);
        }
    } else if (mode == NavMode::Walk) {
        dl->AddLine(ImVec2(c.x - 8, c.y), ImVec2(c.x + 8, c.y), colorU32, 1.5f);
        dl->AddLine(ImVec2(c.x, c.y - 8), ImVec2(c.x, c.y + 8), colorU32, 1.5f);
    }
}
