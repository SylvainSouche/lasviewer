// scene.h — the set of layers on screen, their shared frame and orthophoto.
#pragma once
#include "layer.h"
#include "scene_frame.h"
#include <functional>
#include <memory>
#include <string>
#include <vector>

struct Orthophoto;

struct LoadPlan {
    std::vector<std::string> clouds; // .las / .laz / .copc.laz
    std::vector<std::string> dems;   // GeoTIFF elevation rasters
    std::string ortho;               // optional GeoTIFF orthophoto
};

struct Scene {
    SceneFrame frame;
    std::string orthoPath;
    std::unique_ptr<Orthophoto> ortho; // shared by all layers; outlives them
    std::vector<std::unique_ptr<Layer>> layers;

    Scene();
    ~Scene(); // destroys layers first (GL context must be current)

    GLBounds bounds(bool visibleOnly) const;

    // Builds all layers of `plan`. Requires a current GL context. `progress`
    // is called before each slow step (e.g. to draw a "Loading…" frame).
    // Inputs that fail to load are reported and skipped; returns false only
    // if no layer could be loaded.
    bool load(const LoadPlan& plan, const Programs& programs,
              const std::function<void(const std::string&)>& progress);
};
