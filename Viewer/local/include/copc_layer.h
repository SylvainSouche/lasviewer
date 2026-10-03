// copc_layer.h — a COPC file streamed tile by tile (see copc_streamer.h).
#pragma once
#include "copc_streamer.h"
#include "layer.h"

#include <memory>

struct Orthophoto;

class CopcLayer : public Layer {
  public:
    // bounds: header extent. ortho: used for coloring when non-null.
    CopcLayer(const std::string& path, const WorldBounds& bounds, uint64_t pointCount,
              const Orthophoto* ortho);
    ~CopcLayer() override;

    void start(const SceneFrame& frame); // GL context current; starts the loader

    const char* kind() const override { return "COPC"; }
    GLBounds bounds() const override;
    void update(const RenderContext& ctx) override;
    void render(const RenderContext& ctx) override;
    void renderOverlay(const RenderContext& ctx) override;
    bool wantsHiZ() const override { return true; }
    void drawUI() override;
    std::string status() const override;
    bool busy() const override;
    void handleAction(LayerAction a) override;

  private:
    WorldBounds worldBounds_;
    uint64_t filePointCount_;
    const Orthophoto* ortho_;
    std::unique_ptr<TileGrid> grid_;
    GLuint boxVAO_ = 0, boxVBO_ = 0;
};
