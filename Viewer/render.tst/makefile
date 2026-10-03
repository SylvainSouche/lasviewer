# Render checks: the test scripts run `lasviewer --snapshot` and measure the
# image (render_tool). They need a GPU and a display, which the CI runners
# don't have: macOS only for now.
PLATFORMS=macos
REASON.macos=render tests need a GPU and a display (CI's Linux runners have neither)
PROG=render_tool
LIBS=geo
.include "mk.tst.mk"
