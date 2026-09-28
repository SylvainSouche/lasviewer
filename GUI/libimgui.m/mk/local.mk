# Workarounds for two gaps in bmake-it's IMPORT=fetch: (remove once fixed):
# - obj/ subdirectories aren't created for SRCS entries in subdirectories
#   of the fetched tree (backends/*.cpp). The directory is made just before
#   those objects compile: extracting the archive clears obj/, so creating it
#   when the makefile is read isn't enough on a first build.
# - the fetched tree's root isn't on the include path when compiling its
#   own sources (the backends include "imgui.h" from the root).
_imgui_backends_objdir: .PHONY
	@mkdir -p ${_OBJDIR}/backends
${_OBJDIR}/backends/imgui_impl_glfw.o ${_OBJDIR}/backends/imgui_impl_opengl3.o: _imgui_backends_objdir
CXXFLAGS += -I${.CURDIR}/work/_resolved
