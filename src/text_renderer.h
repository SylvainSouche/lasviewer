// text_renderer.h — FreeType-based 2D text rendering
#pragma once
#include "gl_platform.h"
#include <string>

struct TextRenderer {
    GLuint program = 0;
    GLuint atlasTex = 0;
    GLuint textVAO = 0;
    GLuint textVBO = 0;
    int atlasW = 0, atlasH = 0;
    int fontSize = 16;
    bool initialized = false;

    bool init(int fontSize = 16);
    void drawText(const std::string& text, float x, float y,
                  int screenW, int screenH,
                  float r, float g, float b, float a);
    void destroy();
};
