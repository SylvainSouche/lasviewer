// lasviewer — 3D viewer for LiDAR point clouds (LAS/LAZ/COPC) and GeoTIFF
// DEMs, with orthophoto draping. This file only starts the application; see
// cli.h for the command line, viewer_app.h for the application and scene.h
// for loading.
#include "cli.h"
#include "log_capture.h"
#include "viewer_app.h"

#include <iostream>

int main(int argc, char** argv) {
    CommandLine cmd;
    switch (parseCommandLine(argc, argv, cmd, std::cerr, std::cerr)) {
    case CliStatus::Help:
        return 0;
    case CliStatus::Error:
        std::cerr << "Run " << argv[0] << " -h for help.\n";
        return 1;
    case CliStatus::Run:
        break;
    }
    std::cerr << "lasviewer " << kLasviewerVersion << " — LiDAR + DEM + orthophoto 3D viewer"
              << std::endl;

    LogCapture logCapture; // mirrors std::cerr into the in-app log window
    ViewerApp app;
    app.setSnapshotPath(cmd.snapshot);
    if (cmd.viewSet)
        app.setInitialView(glm::dvec3(cmd.view[0], cmd.view[1], cmd.view[2]), cmd.view[3],
                           cmd.view[4], cmd.view[5]);
    if (!app.init(cmd.plan)) return 1;
    app.run();
    return 0;
}
