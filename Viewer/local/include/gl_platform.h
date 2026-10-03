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
#else
// Linux (Mesa / libglvnd): the core 3.x/4.x entry points are declared by
// glext.h with GL_GLEXT_PROTOTYPES and exported by libGL, so no loader is
// needed. Builds and links in CI; not yet run on a Linux GPU.
#define GL_GLEXT_PROTOTYPES 1
#include <GL/gl.h>
#include <GL/glext.h>
#endif
