# Workaround (specs.md §12.2c-2) for a gap in bmake-it's IMPORT=fetch: the
# fetched tree's root isn't on the include path when compiling its own
# sources (the backends include "imgui.h" from the root).
CXXFLAGS += -I${.CURDIR}/work/_resolved
