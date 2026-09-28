# Workaround for a gap in bmake-it's IMPORT=fetch: (remove once fixed): the
# fetched tree's root isn't on the include path when compiling its own
# sources (the backends include "imgui.h" from the root).
CXXFLAGS += -I${.CURDIR}/work/_resolved
