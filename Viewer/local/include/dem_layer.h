// dem_layer.h — a GeoTIFF DEM/DSM drawn as a GPU-tessellated mesh.
//
// A DEM can play one of three roles:
//   Plain        any elevation raster on its own;
//   Ground       a terrain model (DTM, IGN "MNT") that above-ground layers
//                stand on: where one of them shows something taller than the
//                height threshold, the ground is not textured with the
//                orthophoto, whose pixels there show the object's top;
//   AboveGround  what stands on a Ground layer, from a height model (DHM,
//                IGN "MNH") or a surface model (DSM, IGN "MNS"): drawn
//                semi-transparent, cut below the height threshold.
#pragma once
#include "dem_tess_mesh.h"
#include "layer.h"

#include <vector>

struct Orthophoto;

enum class DemRole { Plain, Ground, AboveGround };

class DemLayer : public Layer {
  public:
    // source: produces the raster (and auxiliary raster, see DemSourceData)
    // for every build. ortho: draped as a texture when non-null.
    DemLayer(const std::string& path, const Orthophoto* ortho, DemSource source,
             DemRole role = DemRole::Plain, const char* kindName = "DEM");
    ~DemLayer() override;

    DemRole role() const { return role_; }
    // Elevation range of the current mesh (GL-space Y, unscaled); false
    // before the first mesh is ready.
    bool elevationRangeGL(float& lo, float& hi) const {
        if (!mesh_.valid) return false;
        lo = mesh_.glBBoxMinY;
        hi = mesh_.glBBoxMaxY;
        return true;
    }
    // Ground elevation at world (x, y) from the current mesh's DEM; false
    // outside it or on nodata.
    bool groundAt(double x, double y, double& z) const {
        return mesh_.ground && mesh_.ground->at(x, y, z);
    }
    // Initial level-of-detail settings (before load()); the panel can still
    // change them.
    void setLod(double collapseAngleDeg, int maxLevel) {
        collapseAngleDeg_ = static_cast<float>(collapseAngleDeg);
        maxLevel_ = maxLevel;
    }
    // Ground layers: an above-ground layer standing on this one. The ground
    // hides its orthophoto under objects only while one of them is visible.
    void addAboveLayer(const Layer* above) { above_.push_back(above); }

    // GL context current. Shows the coarsest level at once and builds the
    // full-detail mesh in the background.
    bool load(const SceneFrame& frame);

    const char* kind() const override { return kindName_; }
    bool transparent() const override { return role_ == DemRole::AboveGround && opacity_ < 1.0f; }
    GLBounds bounds() const override;
    void update(const RenderContext& ctx) override;
    void render(const RenderContext& ctx) override;
    void drawUI() override;
    std::string status() const override;
    bool busy() const override { return mesh_.backgroundBuildInProgress(); }
    void handleAction(LayerAction a) override;

  private:
    void requestRebuild();

    DemStyle style(const RenderContext& ctx) const;

    const Orthophoto* ortho_;
    DemSource source_;
    DemRole role_;
    const char* kindName_;
    std::vector<const Layer*> above_;
    float opacity_ = 0.5f; // AboveGround only
    SceneFrame frame_;
    DEMTessMesh mesh_;
    // Target build parameters (the displayed mesh may still be older).
    float collapseAngleDeg_ = 1.0f;
    int maxLevel_ = 5;
    float pixelsPerSegment_ = 8.0f;
    bool wireframe_ = false;
    bool displacement_ = true;
    bool masterEdges_ = false;
    bool shade_ = true;
};
