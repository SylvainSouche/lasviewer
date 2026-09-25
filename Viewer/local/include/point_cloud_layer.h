// point_cloud_layer.h — a LAS/LAZ file loaded in full (thinned to ≤2M points).
#pragma once
#include "layer.h"
#include "point_cloud.h"

struct Orthophoto;

class PointCloudLayer : public Layer {
public:
    // ortho: used for coloring when non-null (caller checks it covers the cloud).
    PointCloudLayer(const std::string& path, const Orthophoto* ortho);
    ~PointCloudLayer() override;

    bool load(const SceneFrame& frame); // CPU load + GPU upload (GL context current)

    const char* kind() const override { return "Points"; }
    GLBounds bounds() const override;
    void render(const RenderContext& ctx) override;
    void drawUI() override;
    std::string status() const override;
    void handleAction(LayerAction a) override;

private:
    void setUseOrtho(bool useOrtho);

    const Orthophoto* ortho_;
    PointCloud cloud_;
    GLuint vao_ = 0, vboPos_ = 0, vboCol_ = 0;
    bool useOrtho_ = false;
};
