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
GLuint linkTessProgram(const char* vertSrc, const char* tcsSrc, const char* tesSrc,
                       const char* fragSrc);

extern const char* kPointCloudVert;
extern const char* kPointCloudFrag;
extern const char* kLineVert;
extern const char* kLineFrag;
extern const char* kMeshFrag;

// GPU-tessellated DEM mesh shaders (dem_tess_mesh.cpp); kMeshTessEval feeds
// kMeshFrag (vUV, vElev, vEdgeDist).
extern const char* kMeshTessVert;
extern const char* kMeshTessControl;
extern const char* kMeshTessEval;

// Hi-Z pyramid passes (hiz.cpp): kHiZCopyFrag copies the depth buffer into
// mip 0, kHiZDownsampleFrag max-reduces each further mip. kHiZVert draws a
// fullscreen triangle from gl_VertexID (bind an empty VAO).
extern const char* kHiZVert;
extern const char* kHiZCopyFrag;
extern const char* kHiZDownsampleFrag;

} // namespace shaders
