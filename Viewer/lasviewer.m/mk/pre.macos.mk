# System frameworks for OpenGL and GLFW's Cocoa window. Workaround (specs.md
# §12.2c-1): a pre-hook with one-word -Wl, flags, because bmake-it 173f44a
# drops LDFLAGS added by local.mk hooks and splits "-framework X" pairs.
# Back to local.macos.mk with "-framework OpenGL ..." once fixed.
LDFLAGS += -Wl,-framework,OpenGL -Wl,-framework,Cocoa -Wl,-framework,IOKit
