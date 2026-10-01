# Workaround: glm is header-only and has no pkg-config file, and bmake-it has
# no header-only import form (specs.md §12.2c-3). bmake-it's prefix probe
# happens to find MacPorts' optional libglm.dylib, but a header-only glm
# would not be found, so the include directories are given here.
IMPORT_CFLAGS=-I/opt/local/include -I/opt/homebrew/include -I/usr/local/include
