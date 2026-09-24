// layer.h — the common interface of everything a scene can display.
//
// A Layer owns one dataset (a point cloud, a streamed COPC file, a DEM mesh)
// and its GPU resources. The viewer drives all layers the same way each
// frame: update() for background work, render() for drawing, and drawUI()
// for the layer's own settings in the side panel.
#pragma once
#include "gl_platform.h"
#include "scene_frame.h"
#include <glm/glm.hpp>
#include <string>

class HiZ;

// Global display settings, edited from the UI and keyboard.
struct ViewSettings {
    float zScale = 1.0f;          // vertical exaggeration
    float pointSizeMul = 1.0f;
    float pointDensityMul = 1.0f; // screen-space point density (points layers)
    bool useOcclusion = true;     // Hi-Z culling of streamed tiles
    bool showTileBoxes = false;
};

// Linked programs shared by all layers (0 when unavailable).
struct Programs {
    GLuint point = 0;
    GLuint line = 0;
    GLuint tess = 0;
};

struct RenderContext {
    glm::mat4 view{1.0f}, proj{1.0f};
    glm::vec3 camPos{0.0f};
    float fovDeg = 55.0f;
    float viewportW = 1.0f, viewportH = 1.0f;
    bool ortho = false;
    float orthoHeight = 0.0f;
    const ViewSettings* settings = nullptr;
    const Programs* programs = nullptr;
    const HiZ* hiz = nullptr; // last frame's depth pyramid; may be not ready
};

// Keyboard-driven actions a layer may respond to.
enum class LayerAction {
    ToggleColors,
    Finer,             // F — more detail (DEM quadtree depth)
    Coarser,           // S
    CollapseFiner,     // I — smaller collapsing angle (DEM)
    CollapseCoarser,   // O
    ToggleWireframe,   // W
    ToggleDisplacement,// A
    ToggleMasterEdges, // G
};

class Layer {
public:
    Layer(std::string name, std::string path) : name_(std::move(name)), path_(std::move(path)) {}
    virtual ~Layer() = default;
    Layer(const Layer&) = delete;
    Layer& operator=(const Layer&) = delete;

    const std::string& name() const { return name_; }
    const std::string& path() const { return path_; }

    bool visible = true;
    int epsg = 0; // horizontal CRS, 0 = unknown

    virtual const char* kind() const = 0;
    // GL-space extent of what is currently loaded.
    virtual GLBounds bounds() const = 0;
    // Called every frame, visible or not (drains background work).
    virtual void update(const RenderContext&) {}
    virtual void render(const RenderContext&) = 0;
    // Debug overlays drawn after all layers (e.g. tile boxes).
    virtual void renderOverlay(const RenderContext&) {}
    // Whether this layer consults the Hi-Z pyramid (so the viewer builds it).
    virtual bool wantsHiZ() const { return false; }
    // ImGui widgets for this layer's settings.
    virtual void drawUI() {}
    // One-line status for the layer list (point counts, loading state).
    virtual std::string status() const { return {}; }
    virtual bool busy() const { return false; }
    virtual void handleAction(LayerAction) {}

private:
    std::string name_, path_;
};
