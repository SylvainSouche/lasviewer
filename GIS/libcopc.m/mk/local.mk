# Workaround: bmake-it passes this module's own consumer-side flags to the
# upstream CMake build: -l<LIBS> (breaks CMake's compiler check, since the
# library isn't on its search path) and -D<LIB>_BUILDING, which is not a
# valid macro name for LIB=copc-lib. copc-lib finds laz-perf through
# CMAKE_PREFIX_PATH instead. Remove once bmake-it stops exporting them.
LDFLAGS  := ${LDFLAGS:N-l*}
CFLAGS   := ${CFLAGS:N-D*_BUILDING}
CXXFLAGS := ${CXXFLAGS:N-D*_BUILDING}
