// shaders.h — GLSL shader sources and compile/link helpers
#pragma once
#include "gl_platform.h"

namespace shaders {

GLuint compileShader(GLenum type, const char* src);
GLuint linkProgram(const char* vertSrc, const char* fragSrc);

// Links a 4-stage tessellation program (vert -> TCS -> TES -> frag).
// Requires an OpenGL 4.0+ context (GL_TESS_CONTROL_SHADER /
// GL_TESS_EVALUATION_SHADER are core since GL 4.0). Returns 0 on failure
// (check the context version with demTessSupported() before calling this —
// see dem_tess_mesh.h).
GLuint linkTessProgram(const char* vertSrc, const char* tcsSrc,
                       const char* tesSrc, const char* fragSrc);

extern const char* kPointCloudVert;
extern const char* kPointCloudFrag;
extern const char* kLineVert;
extern const char* kLineFrag;
extern const char* kMeshVert;
extern const char* kMeshFrag;

// GPU-tessellated DEM mesh shaders (dem_tess_mesh.cpp). kMeshTessEval
// outputs the same vUV/vElev interface as kMeshVert, so kMeshFrag is reused
// unmodified for both the CPU-only and GPU-tessellated DEM paths.
extern const char* kMeshTessVert;
extern const char* kMeshTessControl;
extern const char* kMeshTessEval;

// Hi-Z (max-mip depth pyramid) downsample shaders — copc_streamer.cpp's
// occlusion culling for streamed point-cloud tiles. See copc_streamer.h's
// TileGrid comment for the full mechanism. kHiZCopyFrag does the initial
// 1:1 capture from the real depth buffer into mip 0 of a color-attachable
// (GL_R32F) pyramid texture; kHiZDownsampleFrag does the subsequent 2x2
// max-reduction for each further mip level. Both share kHiZVert, a
// fullscreen triangle generated procedurally from gl_VertexID — no vertex
// buffer needed, just an empty bound VAO (core profile requires one to be
// bound for glDrawArrays, even with zero enabled attributes).
extern const char* kHiZVert;
extern const char* kHiZCopyFrag;
extern const char* kHiZDownsampleFrag;

} // namespace shaders
