// text_renderer.cpp — FreeType-based 2D text rendering.
//
// Renders a glyph atlas (ASCII 32..126) at init time, then draws text as
// batched textured quads (one VAO, one draw call per text block). The text
// shader runs in screen space (origin = top-left, +Y down).
#include "text_renderer.h"
#include "shaders.h"
// GLFW/OpenGL headers now come from text_renderer.h -> gl_platform.h (see
// that file for why this used to be a fragile per-file ad-hoc block).

#include <ft2build.h>
#include FT_FREETYPE_H

#include <iostream>
#include <vector>
#include <string>
#include <cstring>

// Per-glyph metrics + atlas UVs.
struct GlyphInfo {
    int advanceX, advanceY;
    int bitmapW, bitmapH;
    int bearingX, bearingY;
    float texU0, texV0, texU1, texV1;
};

// File-static glyph table (ASCII 32..126). Kept here rather than in the
// header so the FreeType headers don't leak into the rest of the codebase.
static GlyphInfo g_glyphs[128];

// Text shader sources (kept in this TU — they're text-specific).
static const char* kTextVertSrc = R"GLSL(
#version 330 core
layout (location = 0) in vec2 aPos;
layout (location = 1) in vec2 aUV;
uniform vec2 uScreenSize;
out vec2 vUV;
void main() {
    vec2 ndc = (aPos / uScreenSize) * 2.0 - 1.0;
    ndc.y = -ndc.y;
    gl_Position = vec4(ndc, 0.0, 1.0);
    vUV = aUV;
}
)GLSL";

static const char* kTextFragSrc = R"GLSL(
#version 330 core
in vec2 vUV;
out vec4 FragColor;
uniform sampler2D uAtlas;
uniform vec4 uColor;
void main() {
    float a = texture(uAtlas, vUV).a;
    FragColor = vec4(uColor.rgb, a * uColor.a);
}
)GLSL";

// ---------------------------------------------------------------------------
// init — load a system font via FreeType, rasterize glyphs into a horizontal
// atlas, upload the atlas as a GL texture, and compile the text shader.
// ---------------------------------------------------------------------------

bool TextRenderer::init(int fontSize_) {
    fontSize = fontSize_;
    FT_Library ft;
    if (FT_Init_FreeType(&ft)) {
        std::cerr << "[text] FreeType init failed" << std::endl;
        return false;
    }

    // Find a system font. Try common paths on macOS and Linux.
    const char* fontPaths[] = {
        "/System/Library/Fonts/Menlo.ttc",
        "/System/Library/Fonts/Monaco.ttf",
        "/System/Library/Fonts/Courier.dfont",
        "/Library/Fonts/Menlo.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf",
        "/usr/share/fonts/TTF/DejaVuSansMono.ttf",
        nullptr
    };
    FT_Face face = nullptr;
    for (int i = 0; fontPaths[i]; ++i) {
        if (FT_New_Face(ft, fontPaths[i], 0, &face) == 0) {
            std::cerr << "[text] using font: " << fontPaths[i] << std::endl;
            break;
        }
    }
    if (!face) {
        std::cerr << "[text] no system font found" << std::endl;
        FT_Done_FreeType(ft);
        return false;
    }

    FT_Set_Pixel_Sizes(face, 0, fontSize);

    // First pass: measure total atlas width needed.
    int atlasWidth = 0;
    for (int c = 32; c < 127; ++c) {
        if (FT_Load_Char(face, c, FT_LOAD_RENDER)) continue;
        atlasWidth += face->glyph->bitmap.width + 2;
    }
    atlasW = atlasWidth;
    atlasH = fontSize * 2;

    std::vector<unsigned char> atlasData(static_cast<size_t>(atlasW) * atlasH, 0);

    // Second pass: rasterize glyphs into the atlas.
    int penX = 0;
    for (int c = 32; c < 127; ++c) {
        if (FT_Load_Char(face, c, FT_LOAD_RENDER)) continue;
        FT_GlyphSlot g = face->glyph;
        FT_Bitmap* bmp = &g->bitmap;
        int bw = bmp->width;
        int bh = bmp->rows;
        int pitch = bmp->pitch;
        if (pitch < 0) pitch = -pitch;

        for (int row = 0; row < bh; ++row) {
            std::memcpy(&atlasData[static_cast<size_t>(penX) + row * atlasW],
                        &bmp->buffer[static_cast<size_t>(row) * pitch], bw);
        }

        g_glyphs[c].advanceX = g->advance.x >> 6;
        g_glyphs[c].advanceY = g->advance.y >> 6;
        g_glyphs[c].bitmapW = bw;
        g_glyphs[c].bitmapH = bh;
        g_glyphs[c].bearingX = g->bitmap_left;
        g_glyphs[c].bearingY = g->bitmap_top;
        g_glyphs[c].texU0 = static_cast<float>(penX) / atlasW;
        g_glyphs[c].texV0 = 0.0f;
        g_glyphs[c].texU1 = static_cast<float>(penX + bw) / atlasW;
        g_glyphs[c].texV1 = static_cast<float>(bh) / atlasH;

        penX += bw + 2;
    }

    // Upload atlas as RGBA (alpha = grayscale glyph). GL_R8 can fail on some
    // macOS drivers, so we use RGBA8 for compatibility.
    std::vector<unsigned char> rgbaData(static_cast<size_t>(atlasW) * atlasH * 4, 0);
    for (int i = 0; i < atlasW * atlasH; ++i) {
        rgbaData[i * 4 + 3] = atlasData[i];
    }
    glGenTextures(1, &atlasTex);
    glBindTexture(GL_TEXTURE_2D, atlasTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, atlasW, atlasH, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, rgbaData.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    // Compile the text shader (uses the shared linkProgram helper).
    program = shaders::linkProgram(kTextVertSrc, kTextFragSrc);

    // Dynamic VBO for text quads (6 verts × 4 floats = 24 floats per char).
    glGenVertexArrays(1, &textVAO);
    glGenBuffers(1, &textVBO);
    glBindVertexArray(textVAO);
    glBindBuffer(GL_ARRAY_BUFFER, textVBO);
    glBufferData(GL_ARRAY_BUFFER, 200000 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float),
                          (void*)(2 * sizeof(float)));
    glBindVertexArray(0);

    FT_Done_Face(face);
    FT_Done_FreeType(ft);
    initialized = true;
    std::cerr << "[text] atlas " << atlasW << "x" << atlasH
              << ", font size " << fontSize << std::endl;
    return true;
}

// ---------------------------------------------------------------------------
// drawText — render `text` at (x, y) in screen pixels (origin = top-left).
// Supports '\n' newlines. Color is RGBA (0..1).
// ---------------------------------------------------------------------------

void TextRenderer::drawText(const std::string& text, float x, float y,
                            int screenW, int screenH,
                            float r, float g, float b, float a) {
    if (!initialized) return;

    std::vector<float> verts;
    float penX = x;
    float penY = y;
    int lineHeight = fontSize + 4;

    for (char c : text) {
        if (c == '\n') {
            penX = x;
            penY += lineHeight;
            continue;
        }
        if (c < 32 || c >= 127) continue;
        const GlyphInfo& gi = g_glyphs[static_cast<unsigned char>(c)];

        float px = penX + gi.bearingX;
        float py = penY + (fontSize - gi.bearingY);
        float pw = gi.bitmapW;
        float ph = gi.bitmapH;

        float quad[24] = {
            px,      py,      gi.texU0, gi.texV0,
            px + pw, py,      gi.texU1, gi.texV0,
            px + pw, py + ph, gi.texU1, gi.texV1,
            px,      py,      gi.texU0, gi.texV0,
            px + pw, py + ph, gi.texU1, gi.texV1,
            px,      py + ph, gi.texU0, gi.texV1,
        };
        for (float f : quad) verts.push_back(f);

        penX += gi.advanceX;
    }

    if (verts.empty()) return;

    glUseProgram(program);
    glUniform2f(glGetUniformLocation(program, "uScreenSize"),
                static_cast<float>(screenW), static_cast<float>(screenH));
    glUniform1i(glGetUniformLocation(program, "uAtlas"), 0);
    glUniform4f(glGetUniformLocation(program, "uColor"), r, g, b, a);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, atlasTex);

    glBindVertexArray(textVAO);
    glBindBuffer(GL_ARRAY_BUFFER, textVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0,
                    verts.size() * sizeof(float), verts.data());

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(verts.size() / 4));
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glBindVertexArray(0);
}

void TextRenderer::destroy() {
    if (program) glDeleteProgram(program);
    if (atlasTex) glDeleteTextures(1, &atlasTex);
    if (textVAO) glDeleteVertexArrays(1, &textVAO);
    if (textVBO) glDeleteBuffers(1, &textVBO);
    initialized = false;
}
