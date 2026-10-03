// cli.h — lasviewer's command line: what to load, and how to start.
#pragma once
#include "scene.h"

#include <ostream>
#include <string>

constexpr const char* kLasviewerVersion = "0.3.0";

struct CommandLine {
    LoadPlan plan;
    std::string snapshot; // --snapshot: save a frame once loaded, then quit
    bool viewSet = false; // --view x,y,z,distance,yaw,pitch
    double view[6] = {};
};

enum class CliStatus {
    Run,   // arguments valid: start the viewer
    Help,  // -h / --help: usage printed, exit successfully
    Error, // invalid arguments: reason printed, exit with an error
};

// Parses argv (argv[0] is the program name). Usage goes to `out`, errors to
// `err`. Positional .tif/.tiff files are classified by content: 8-bit
// imagery with at least 3 bands is the orthophoto, anything else a DEM
// (reading the file header through GDAL); other files are point clouds.
CliStatus parseCommandLine(int argc, const char* const* argv, CommandLine& out,
                           std::ostream& outStream, std::ostream& err);

void printUsage(const char* prog, std::ostream& out);
