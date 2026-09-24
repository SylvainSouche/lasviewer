// dem_layer.h — a GeoTIFF DEM/DSM drawn as a GPU-tessellated mesh.
#pragma once
#include "dem_tess_mesh.h"
#include "layer.h"

struct Orthophoto;

class DemLayer : public Layer {
public:
    // ortho: draped as a texture when non-null (the mesh itself checks the
    // CRS and overlap, reprojecting when PROJ is available).
    DemLayer(const std::string& path, const Orthophoto* ortho);
    ~DemLayer() override;

    // GL context current. Shows the coarsest level at once and builds the
    // full-detail mesh in the background.
    bool load(const SceneFrame& frame);

    const char* kind() const override { return "DEM"; }
    GLBounds bounds() const override;
    void update(const RenderContext& ctx) override;
    void render(const RenderContext& ctx) override;
    void drawUI() override;
    std::string status() const override;
    bool busy() const override { return mesh_.backgroundBuildInProgress(); }
    void handleAction(LayerAction a) override;

private:
    void requestRebuild();

    const Orthophoto* ortho_;
    SceneFrame frame_;
    DEMTessMesh mesh_;
    // Target build parameters (the displayed mesh may still be older).
    float collapseAngleDeg_ = 1.0f;
    int maxLevel_ = 5;
    float pixelsPerSegment_ = 8.0f;
    bool wireframe_ = false;
    bool displacement_ = true;
    bool masterEdges_ = false;
};
