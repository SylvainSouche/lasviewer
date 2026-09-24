// gl_platform.h — the single canonical place to include GLFW plus the
// correct platform OpenGL header.
//
// Why this exists: on macOS, GLFW's own default include path (when
// GLFW_INCLUDE_NONE is NOT defined) pulls in the legacy <OpenGL/gl.h>,
// which does not declare core-profile 3.2+ symbols, let alone GL 4.0+
// tessellation-shader symbols (GL_TESS_CONTROL_SHADER etc.). Previously,
// several .cpp files worked around this individually with an inline
// `#define GLFW_INCLUDE_NONE` + `#include <OpenGL/gl3.h>` block — but
// each of those files' OWN header (dem_mesh.h, gl_app.h, etc.) still did a
// plain `#include <GLFW/glfw3.h>`. Because C++ header guards mean a header
// is only truly processed on its FIRST inclusion in a given translation
// unit, whichever file's include order "won" that race determined whether
// the TU got modern GL symbols or not — this worked by accident wherever
// the owning .cpp's ad-hoc block happened to run first, and silently broke
// for any file (like shaders.cpp) that never had the workaround, or any
// new file that includes a GL-using header before setting up the block
// itself.
//
// Fix: every file that needs GL types/constants includes THIS header
// instead of <GLFW/glfw3.h> directly — there is now exactly one place
// this logic lives, so there's nothing left to race.
#pragma once
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#ifdef __APPLE__
#  define GL_SILENCE_DEPRECATION 1
#  include <OpenGL/gl3.h>
#endif
// Linux/Windows: not verified here (no GPU/GLFW in this sandbox — see
// specs.md §12.3). GLFW_INCLUDE_NONE + linking -lGL (Linux, see Makefile)
// gets the header DECLARATIONS this project needs from most distributions'
// dev packages, but whether the GL 4.0+ tessellation entry points resolve
// as directly-linkable symbols (vs. needing glXGetProcAddress / a loader
// like GLAD/GLEW, which this project does not currently use) has NOT been
// confirmed on real Linux hardware. If Linux linking fails on
// glCreateShader(GL_TESS_CONTROL_SHADER)-family calls specifically (link
// error, not a missing-declaration compile error), that's the likely
// cause — a loader would need to be added.
