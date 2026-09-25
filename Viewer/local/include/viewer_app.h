// viewer_app.h — window, frame loop, input routing and UI.
#pragma once
#include "camera.h"
#include "camera_controller.h"
#include "hiz.h"
#include "layer.h"
#include "scene.h"

#include <string>

class ViewerApp {
public:
    ViewerApp();
    ~ViewerApp();

    bool init(const LoadPlan& plan); // window + GL + UI + scene
    // Save the first frame rendered once all layers are idle to a binary PPM,
    // then quit (for scripted checks). The window then opens without focus
    // and ignores keyboard/mouse input. Call before init().
    void setSnapshotPath(const std::string& path) { snapshotPath_ = path; }
    void run();

    // GLFW callback entry points.
    void onKey(int key, int action, int mods);
    void onChar(unsigned int codepoint);
    void onMouseButton(int button, int action, int mods);
    void onCursor(double x, double y);
    void onScroll(double dy);
    void onFramebufferSize(int w, int h);

private:
    struct UiState {
        bool showPanel = true;
        bool showLog = false;
        bool showHelp = false;
        bool scrollLogToBottom = true;
        size_t logLinesSeen = 0;
    };

    bool createWindow(const std::string& title);
    bool compilePrograms();
    void drawLoadingFrame(const std::string& message);
    RenderContext makeContext();
    void renderFrame();
    void handlePick();
    void applyToLayers(LayerAction a);
    void resetView();
    bool saveSnapshot(const std::string& path);

    // UI (viewer_ui.cpp)
    void drawUI();
    void drawMainPanel();
    void drawLogWindow();
    void drawHelpWindow();

    GLFWwindow* window_ = nullptr;
    int fbW_ = 1, fbH_ = 1;
    Camera camera_;
    CameraController controller_{camera_};
    ViewSettings settings_;
    Programs programs_;
    HiZ hiz_;
    bool hizOk_ = false;
    Scene scene_;
    UiState ui_;
    bool imguiReady_ = false;

    // Last double-click pick, world coordinates.
    bool hasPick_ = false;
    glm::dvec3 lastPick_{0.0};
    double fps_ = 0.0;
    std::string snapshotPath_;
};
