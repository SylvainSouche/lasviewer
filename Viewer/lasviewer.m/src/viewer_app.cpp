// viewer_app.cpp — window, GL setup, frame loop and input routing.
// The ImGui panels live in viewer_ui.cpp.
#include "viewer_app.h"

#include "dem_layer.h"
#include "dem_tess_mesh.h"
#include "shaders.h"

#include <imgui.h>

#include <cmath>
#include <cstdio>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <iostream>
#include <vector>

namespace {

ViewerApp* appFrom(GLFWwindow* w) {
    return static_cast<ViewerApp*>(glfwGetWindowUserPointer(w));
}

void keyCb(GLFWwindow* w, int key, int, int action, int mods) {
    appFrom(w)->onKey(key, action, mods);
}
void charCb(GLFWwindow* w, unsigned int c) {
    appFrom(w)->onChar(c);
}
void mouseButtonCb(GLFWwindow* w, int b, int action, int mods) {
    appFrom(w)->onMouseButton(b, action, mods);
}
void cursorCb(GLFWwindow* w, double x, double y) {
    appFrom(w)->onCursor(x, y);
}
void scrollCb(GLFWwindow* w, double, double dy) {
    appFrom(w)->onScroll(dy);
}
void fbSizeCb(GLFWwindow* w, int width, int height) {
    appFrom(w)->onFramebufferSize(width, height);
}

// A proportional TrueType font if one is installed; ImGui's built-in
// bitmap font otherwise.
void loadUiFont(float sizePx) {
    ImGuiIO& io = ImGui::GetIO();
    const char* candidates[] = {
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/Library/Fonts/Arial.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/TTF/DejaVuSans.ttf",
    };
    for (const char* path : candidates) {
        if (FILE* f = std::fopen(path, "rb")) {
            std::fclose(f);
            if (io.Fonts->AddFontFromFileTTF(path, sizePx)) return;
        }
    }
    ImFontConfig cfg;
    cfg.SizePixels = sizePx;
    io.Fonts->AddFontDefault(&cfg);
}

} // namespace

ViewerApp::ViewerApp() {
    camera_.fov = 55.0f;
}

ViewerApp::~ViewerApp() {
    if (!window_) return;
    glfwMakeContextCurrent(window_);
    scene_.layers.clear();
    if (hizOk_) hiz_.destroy();
    if (programs_.point) glDeleteProgram(programs_.point);
    if (programs_.line) glDeleteProgram(programs_.line);
    if (programs_.tess) glDeleteProgram(programs_.tess);
    if (imguiReady_) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
    }
    glfwDestroyWindow(window_);
    glfwTerminate();
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

bool ViewerApp::createWindow(const std::string& title) {
    if (!glfwInit()) {
        std::cerr << "ERROR: glfwInit failed" << std::endl;
        return false;
    }
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GLFW_TRUE);
    if (!snapshotPath_.empty()) {
        // Scripted capture: don't take keyboard focus from whatever the user
        // is doing (their keystrokes would otherwise change the view).
        glfwWindowHint(GLFW_FOCUSED, GLFW_FALSE);
        glfwWindowHint(GLFW_FOCUS_ON_SHOW, GLFW_FALSE);
    }
    // 4.1 core enables DEM tessellation; 3.3 core still shows point clouds.
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    window_ = glfwCreateWindow(1400, 900, title.c_str(), nullptr, nullptr);
    if (!window_) {
        std::cerr << "[gl] no GL 4.1 core context, falling back to 3.3 (no DEM display)"
                  << std::endl;
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        window_ = glfwCreateWindow(1400, 900, title.c_str(), nullptr, nullptr);
    }
    if (!window_) {
        std::cerr << "ERROR: could not create a window" << std::endl;
        glfwTerminate();
        return false;
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);
    glfwGetFramebufferSize(window_, &fbW_, &fbH_);
    camera_.aspect = static_cast<float>(fbW_) / static_cast<float>(std::max(fbH_, 1));

    glfwSetWindowUserPointer(window_, this);
    glfwSetKeyCallback(window_, keyCb);
    glfwSetCharCallback(window_, charCb);
    glfwSetMouseButtonCallback(window_, mouseButtonCb);
    glfwSetCursorPosCallback(window_, cursorCb);
    glfwSetScrollCallback(window_, scrollCb);
    glfwSetFramebufferSizeCallback(window_, fbSizeCb);

    // ImGui chains to the callbacks installed above.
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 4.0f;
    style.FrameRounding = 3.0f;
    style.Colors[ImGuiCol_WindowBg].w = 0.92f;

    // Retina: window coords are logical points, the framebuffer is larger —
    // rasterize the font at framebuffer resolution and scale it back down.
    // Other HiDPI setups report a content scale instead: scale the UI up.
    int winW = 1, winH = 1;
    glfwGetWindowSize(window_, &winW, &winH);
    float fbRatio = static_cast<float>(fbW_) / static_cast<float>(std::max(winW, 1));
    float contentScale = 1.0f;
    glfwGetWindowContentScale(window_, &contentScale, nullptr);
    const float baseFontPx = 15.0f;
    if (fbRatio > 1.01f) {
        loadUiFont(baseFontPx * fbRatio);
        io.FontGlobalScale = 1.0f / fbRatio;
    } else {
        loadUiFont(baseFontPx * contentScale);
        style.ScaleAllSizes(contentScale);
    }

    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 330");
    imguiReady_ = true;
    return true;
}

bool ViewerApp::compilePrograms() {
    programs_.point = shaders::linkProgram(shaders::kPointCloudVert, shaders::kPointCloudFrag);
    programs_.line = shaders::linkProgram(shaders::kLineVert, shaders::kLineFrag);
    if (!programs_.point || !programs_.line) return false;
    if (demTessSupported()) {
        programs_.tess = shaders::linkTessProgram(shaders::kMeshTessVert, shaders::kMeshTessControl,
                                                  shaders::kMeshTessEval, shaders::kMeshFrag);
    }
    hizOk_ = hiz_.init();
    return true;
}

bool ViewerApp::init(const LoadPlan& plan) {
    std::string first = !plan.clouds.empty() ? plan.clouds.front() : plan.dems.front();
    std::string title = "lasviewer — " + first.substr(first.find_last_of('/') + 1);
    size_t n = plan.clouds.size() + plan.dems.size();
    if (n > 1) title += " (+" + std::to_string(n - 1) + ")";

    if (!createWindow(title)) return false;
    if (!compilePrograms()) {
        std::cerr << "ERROR: shader setup failed" << std::endl;
        return false;
    }
    if (!scene_.load(plan, programs_, [this](const std::string& m) { drawLoadingFrame(m); })) {
        std::cerr << "ERROR: nothing could be loaded" << std::endl;
        return false;
    }
    controller_.setFrame(scene_.frame);
    groundFn_ = [this](double x, double y, double& z) { return groundAt(x, y, z); };
    resetView();
    if (initialView_) {
        glm::vec3 t = scene_.frame.toGL(viewTarget_.x, viewTarget_.y, viewTarget_.z);
        t.y *= settings_.zScale;
        camera_.target = t;
        camera_.distance = static_cast<float>(viewDistance_ / scene_.frame.scale);
        camera_.yaw = static_cast<float>(glm::radians(viewYawDeg_));
        camera_.pitch = static_cast<float>(glm::radians(viewPitchDeg_));
    }
    if (initialNav_ != NavMode::Orbit) setNavMode(initialNav_);
    return true;
}

// ---------------------------------------------------------------------------
// Navigation modes
// ---------------------------------------------------------------------------

bool ViewerApp::groundAt(double x, double y, double& z) const {
    // The terrain: visible DEMs that aren't above-ground layers (DHM/DSM).
    for (const auto& l : scene_.layers) {
        const auto* dem = dynamic_cast<const DemLayer*>(l.get());
        if (dem && l->visible && dem->role() != DemRole::AboveGround && dem->groundAt(x, y, z))
            return true;
    }
    return false;
}

void ViewerApp::setNavMode(NavMode mode) {
    controller_.setMode(mode, settings_.zScale, groundFn_);
    lastNavInput_ = glfwGetTime();
    static const char* kNames[] = {"orbit", "fly", "walk"};
    std::cerr << "[nav] " << kNames[static_cast<int>(mode)] << " mode" << std::endl;
}

// Fly and Walk read the keys and the cursor every frame (smooth, frame-rate
// independent movement) instead of reacting to key-repeat events.
NavInput ViewerApp::sampleNavInput() {
    NavInput in;
    if (!snapshotPath_.empty() || controller_.mode() == NavMode::Orbit) return in;
    if (!ImGui::GetIO().WantCaptureKeyboard) {
        auto down = [&](int k) { return glfwGetKey(window_, k) == GLFW_PRESS; };
        in.forward = down(GLFW_KEY_UP);
        in.back = down(GLFW_KEY_DOWN);
        in.left = down(GLFW_KEY_LEFT);
        in.right = down(GLFW_KEY_RIGHT);
        in.run = down(GLFW_KEY_LEFT_SHIFT) || down(GLFW_KEY_RIGHT_SHIFT);
    }
    if (controller_.mode() == NavMode::Fly && controller_.anyButtonDown()) {
        double x = 0, y = 0;
        int w = 1, h = 1;
        glfwGetCursorPos(window_, &x, &y);
        glfwGetWindowSize(window_, &w, &h);
        in.steering = true;
        in.stickX = static_cast<float>((x - w * 0.5) / (w * 0.5));
        in.stickY = static_cast<float>((y - h * 0.5) / (h * 0.5));
    }
    bool walking = controller_.mode() == NavMode::Walk &&
                   (in.forward || in.back || in.left || in.right || controller_.anyButtonDown());
    if (in.steering || walking) lastNavInput_ = glfwGetTime();
    return in;
}

void ViewerApp::drawLoadingFrame(const std::string& message) {
    glfwPollEvents();
    glViewport(0, 0, fbW_, fbH_);
    glClearColor(0.10f, 0.11f, 0.13f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(
        ImVec2(vp->WorkPos.x + vp->WorkSize.x * 0.5f, vp->WorkPos.y + vp->WorkSize.y * 0.5f),
        ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::Begin("##loading", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                     ImGuiWindowFlags_NoSavedSettings);
    ImGui::TextUnformatted(message.c_str());
    ImGui::End();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window_);
}

// ---------------------------------------------------------------------------
// Frame loop
// ---------------------------------------------------------------------------

void ViewerApp::run() {
    double lastFrame = glfwGetTime();
    double fpsWindowStart = glfwGetTime();
    double startTime = fpsWindowStart;
    int frames = 0;
    while (!glfwWindowShouldClose(window_)) {
        glfwPollEvents();

        ++frames;
        double now = glfwGetTime();
        controller_.update(now - lastFrame, sampleNavInput(), settings_.zScale, groundFn_);
        lastFrame = now;
        if (now - fpsWindowStart >= 0.5) {
            fps_ = frames / (now - fpsWindowStart);
            frames = 0;
            fpsWindowStart = now;
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        renderFrame();
        drawUI();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        if (!snapshotPath_.empty()) {
            bool busy = false;
            for (auto& layer : scene_.layers) busy |= layer->busy();
            double elapsed = glfwGetTime() - startTime;
            if ((!busy && elapsed > 2.0) || elapsed > 120.0) {
                saveSnapshot(snapshotPath_);
                glfwSetWindowShouldClose(window_, GLFW_TRUE);
            }
        }
        glfwSwapBuffers(window_);
    }
}

bool ViewerApp::saveSnapshot(const std::string& path) {
    std::vector<unsigned char> px(static_cast<size_t>(fbW_) * fbH_ * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadBuffer(GL_BACK);
    glReadPixels(0, 0, fbW_, fbH_, GL_RGB, GL_UNSIGNED_BYTE, px.data());
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) {
        std::cerr << "ERROR: cannot write " << path << std::endl;
        return false;
    }
    std::fprintf(f, "P6\n%d %d\n255\n", fbW_, fbH_);
    for (int y = fbH_ - 1; y >= 0; --y) // GL rows are bottom-up
        std::fwrite(px.data() + static_cast<size_t>(y) * fbW_ * 3, 1, static_cast<size_t>(fbW_) * 3,
                    f);
    std::fclose(f);
    std::cerr << "[snapshot] wrote " << path << " (" << fbW_ << "x" << fbH_ << ")" << std::endl;
    return true;
}

RenderContext ViewerApp::makeContext() {
    RenderContext ctx;
    ctx.view = camera_.view();
    ctx.proj = camera_.proj();
    ctx.camPos = camera_.position();
    ctx.fovDeg = camera_.fov;
    ctx.viewportW = static_cast<float>(fbW_);
    ctx.viewportH = static_cast<float>(fbH_);
    ctx.ortho = camera_.ortho;
    ctx.orthoHeight = camera_.ortho ? camera_.orthoHeight() : 0.0f;
    ctx.settings = &settings_;
    ctx.programs = &programs_;
    ctx.hiz = hizOk_ ? &hiz_ : nullptr;
    return ctx;
}

void ViewerApp::renderFrame() {
    GLBounds bounds = scene_.bounds(true);
    if (!bounds.valid()) bounds = scene_.bounds(false);
    if (bounds.valid()) {
        // Walk and Fly: near plane down to 10 cm, so nothing at the
        // viewer's feet is clipped.
        float minNear = controller_.mode() == NavMode::Orbit
                            ? 0.001f
                            : static_cast<float>(0.1 / scene_.frame.scale);
        computeNearFar(camera_, bounds.min, bounds.max, settings_.zScale, camera_.nearP,
                       camera_.farP, minNear);
    }
    RenderContext ctx = makeContext();

    for (auto& layer : scene_.layers) layer->update(ctx);

    glViewport(0, 0, fbW_, fbH_);
    glClearColor(0.10f, 0.11f, 0.13f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_MULTISAMPLE);

    bool wantHiZ = false;
    for (auto& layer : scene_.layers) {
        if (!layer->visible || layer->transparent()) continue;
        layer->render(ctx);
        wantHiZ |= layer->wantsHiZ();
    }
    // Next frame's occlusion culling uses this frame's depth, opaque layers
    // only.
    if (wantHiZ && hizOk_ && settings_.useOcclusion)
        hiz_.build(fbW_, fbH_);
    else if (hizOk_)
        hiz_.invalidate();
    for (auto& layer : scene_.layers)
        if (layer->visible && layer->transparent()) layer->render(ctx);

    for (auto& layer : scene_.layers)
        if (layer->visible) layer->renderOverlay(ctx);

    handlePick();
}

// Double-click focus: read back the depth under the cursor right after this
// frame's scene drawing (GL_BACK still holds it), unproject, and make that
// point the orbit pivot without moving the eye.
void ViewerApp::handlePick() {
    double x = 0, y = 0;
    if (!controller_.takePickRequest(x, y)) return;
    int winW = 1, winH = 1;
    glfwGetWindowSize(window_, &winW, &winH);
    int fbX = static_cast<int>(x * fbW_ / std::max(winW, 1));
    int glY = fbH_ - 1 - static_cast<int>(y * fbH_ / std::max(winH, 1));
    if (fbX < 0 || fbX >= fbW_ || glY < 0 || glY >= fbH_) return;

    float depth = 1.0f;
    glReadPixels(fbX, glY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
    if (depth >= 0.9999999f) {
        std::cerr << "[pick] missed geometry" << std::endl;
        return;
    }
    glm::vec3 p =
        camera_.unproject(2.0f * fbX / fbW_ - 1.0f, 2.0f * glY / fbH_ - 1.0f, depth * 2.0f - 1.0f);
    if (!controller_.focusOn(p)) return;
    glm::vec3 unscaled(p.x, p.y / settings_.zScale, p.z);
    lastPick_ = scene_.frame.toWorld(unscaled);
    hasPick_ = true;
    std::cerr << "[pick] focused on " << std::fixed << lastPick_.x << ", " << lastPick_.y << ", "
              << lastPick_.z << std::defaultfloat << std::endl;
}

void ViewerApp::applyToLayers(LayerAction a) {
    for (auto& layer : scene_.layers) layer->handleAction(a);
}

void ViewerApp::resetView() {
    GLBounds b = scene_.bounds(true);
    if (!b.valid()) b = scene_.bounds(false);
    controller_.home(b, settings_.zScale);
}

// ---------------------------------------------------------------------------
// Input
// ---------------------------------------------------------------------------

void ViewerApp::onKey(int key, int action, int mods) {
    if (!snapshotPath_.empty()) return; // scripted capture: no user input
    if (key == GLFW_KEY_LEFT_SHIFT || key == GLFW_KEY_RIGHT_SHIFT) {
        controller_.setShift(action != GLFW_RELEASE);
        return;
    }
    controller_.setShift((mods & GLFW_MOD_SHIFT) != 0);
    if (action == GLFW_RELEASE || ImGui::GetIO().WantCaptureKeyboard) return;

    switch (key) {
    case GLFW_KEY_ESCAPE:
        glfwSetWindowShouldClose(window_, GLFW_TRUE);
        break;
    case GLFW_KEY_TAB:
        if (action == GLFW_PRESS) ui_.showPanel = !ui_.showPanel;
        break;
    case GLFW_KEY_UP:
    case GLFW_KEY_DOWN:
    case GLFW_KEY_LEFT:
    case GLFW_KEY_RIGHT:
        controller_.onArrowKey(key); // Orbit: move; Fly: speed; Walk: sampled per frame
        lastNavInput_ = glfwGetTime();
        break;
    case GLFW_KEY_SPACE:
        if (controller_.mode() == NavMode::Fly) {
            controller_.flyStop();
            lastNavInput_ = glfwGetTime();
        }
        break;
    default:
        break;
    }
}

// Letter shortcuts go through the character callback so they follow the
// active keyboard layout (AZERTY, QWERTZ, …) rather than US key positions.
void ViewerApp::onChar(unsigned int c) {
    if (!snapshotPath_.empty()) return;
    if (ImGui::GetIO().WantCaptureKeyboard) return;
    if (c >= 'A' && c <= 'Z') c += 'a' - 'A';
    switch (c) {
    case 'h':
        ui_.showHelp = !ui_.showHelp;
        break;
    case '1':
        setNavMode(NavMode::Orbit);
        break;
    case '2':
        setNavMode(NavMode::Fly);
        break;
    case '3':
        setNavMode(NavMode::Walk);
        break;
    case 'l':
        ui_.showLog = !ui_.showLog;
        break;
    case 'b':
        settings_.showTileBoxes = !settings_.showTileBoxes;
        break;
    case 'c':
        applyToLayers(LayerAction::ToggleColors);
        break;
    case 'w':
        applyToLayers(LayerAction::ToggleWireframe);
        break;
    case 'a':
        applyToLayers(LayerAction::ToggleDisplacement);
        break;
    case 'g':
        applyToLayers(LayerAction::ToggleMasterEdges);
        break;
    case 'i':
        applyToLayers(LayerAction::CollapseFiner);
        break;
    case 'o':
        applyToLayers(LayerAction::CollapseCoarser);
        break;
    case 'f':
        settings_.pointDensityMul = std::min(settings_.pointDensityMul * 1.5f, 8.0f);
        applyToLayers(LayerAction::Finer);
        break;
    case 's':
        settings_.pointDensityMul = std::max(settings_.pointDensityMul / 1.5f, 0.1f);
        applyToLayers(LayerAction::Coarser);
        break;
    case 'n':
        settings_.pointSizeMul = std::max(settings_.pointSizeMul / 1.3f, 0.1f);
        break;
    case 'm':
        settings_.pointSizeMul = std::min(settings_.pointSizeMul * 1.3f, 10.0f);
        break;
    case 'e':
        settings_.zScale = std::min(settings_.zScale * 1.2f, 100.0f);
        break;
    case 'd':
        settings_.zScale = std::max(settings_.zScale / 1.2f, 0.01f);
        break;
    case 'u':
        settings_.zScale = 1.0f;
        break;
    case 'r':
        resetView();
        break;
    case 'p':
        if (controller_.mode() == NavMode::Orbit) camera_.ortho = !camera_.ortho;
        break;
    case 'v':
        controller_.sideView();
        break;
    case 't':
        controller_.topView();
        break;
    default:
        break;
    }
}

void ViewerApp::onMouseButton(int button, int action, int mods) {
    if (!snapshotPath_.empty()) return;
    bool pressed = action == GLFW_PRESS;
    // Presses over UI belong to ImGui; releases always reach the controller
    // so a drag that ends over a window doesn't leave a button stuck.
    if (pressed && ImGui::GetIO().WantCaptureMouse) return;
    controller_.setShift((mods & GLFW_MOD_SHIFT) != 0);
    double x = 0, y = 0;
    glfwGetCursorPos(window_, &x, &y);
    controller_.onMouseButton(button, pressed, x, y, glfwGetTime());
    lastNavInput_ = glfwGetTime();
}

void ViewerApp::onCursor(double x, double y) {
    // Cursor deltas are in window coordinates, so pan against window height.
    int winW = 1, winH = 1;
    glfwGetWindowSize(window_, &winW, &winH);
    controller_.onCursor(x, y, static_cast<float>(std::max(winH, 1)));
    if (controller_.anyButtonDown()) lastNavInput_ = glfwGetTime();
}

void ViewerApp::onScroll(double dy) {
    if (!snapshotPath_.empty()) return;
    if (ImGui::GetIO().WantCaptureMouse) return;
    controller_.onScroll(dy);
    lastNavInput_ = glfwGetTime();
}

void ViewerApp::onFramebufferSize(int w, int h) {
    fbW_ = std::max(w, 1);
    fbH_ = std::max(h, 1);
    camera_.aspect = static_cast<float>(fbW_) / static_cast<float>(fbH_);
}
