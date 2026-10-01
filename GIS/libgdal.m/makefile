LIB=gdal
IMPORT=pkg:gdal
REQUIRES=gdal
# GDAL installs its headers loose in the prefix's include/ (no gdal/
# subdirectory): stage its own families of headers by pattern.
IMPORT_HEADERS=gdal*.h cpl_*.h ogr*.h
.include "mk.lib.mk"
