// gl_platform.h — the single canonical place to include GLFW plus the
// correct platform OpenGL header.
//
// On macOS, GLFW's default include pulls in the legacy <OpenGL/gl.h>, which
// lacks core-profile (3.2+) and tessellation symbols. Every file that needs
// GL includes this header instead of <GLFW/glfw3.h> directly, so the right
// declarations are seen regardless of include order.
#pragma once
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION 1
#include <OpenGL/gl3.h>
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
