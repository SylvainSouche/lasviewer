# Workaround (specs.md §12.2c-1): resolve from the MacPorts prefix instead
# of pkg-config. Hard-coded: a Homebrew or pkgsrc host must edit it.
# bmake-it records pkg-config's --static link list minus the library's own
# -L directory, which is also where all those dependencies live, so the
# consumer's link fails. Linking the shared library needs only -l<LIB>.
IMPORT_PREFIX=/opt/local
