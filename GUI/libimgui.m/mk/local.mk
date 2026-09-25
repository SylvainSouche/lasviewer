# Workarounds for two gaps in bmake-it's IMPORT=fetch: (remove once fixed):
# - obj/ subdirectories aren't created for SRCS entries in subdirectories
#   of the fetched tree (backends/*.cpp);
# - the fetched tree's root isn't on the include path when compiling its
#   own sources (the backends include "imgui.h" from the root).
_IMGUI_OBJ_SUBDIRS != mkdir -p ${_OBJDIR}/backends && echo ok
CXXFLAGS += -I${.CURDIR}/work/_resolved
