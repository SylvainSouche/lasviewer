# The viewer application: scene, layers, rendering, UI.
# GDAL is listed only so that libgeo's transitive -lgdal finds GDAL's lib
# directory: bmake-it passes a static library's link deps on (-l) but not
# their search paths. Remove once that is fixed; Viewer includes no GDAL
# header.
PREREQS=Geo ImGui GLFW GLM PDAL GDAL
.include "mk.framework.mk"
