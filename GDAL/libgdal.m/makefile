LIB=gdal
IMPORT=pkg:gdal
# GDAL installs its headers loose in the prefix's include/ (no gdal/
# subdirectory), so every header lasviewer's code reaches is listed: the
# ones it includes (cpl_conv.h cpl_string.h gdal_priv.h gdal_utils.h
# ogr_spatialref.h) and everything those include. Regenerate after a GDAL
# upgrade with:
#   inc=$(pkg-config --variable=includedir gdal)
#   printf '#include <%s>\n' cpl_conv.h cpl_string.h gdal_priv.h gdal_utils.h \
#     ogr_spatialref.h | clang++ -x c++ -std=c++17 -I"$inc" -E -H - -o /dev/null \
#     2>&1 | grep -oE "$inc/[^ ]+" | sed "s|$inc/||" | sort -u
IMPORT_HEADERS= \
    cpl_atomic_ops.h cpl_config.h cpl_conv.h cpl_error.h cpl_json.h \
    cpl_minixml.h cpl_multiproc.h cpl_port.h cpl_progress.h cpl_string.h \
    cpl_virtualmem.h cpl_vsi.h gdal.h gdal_asyncreader.h \
    gdal_colortable.h gdal_computedrasterband.h gdal_cpp_functions.h \
    gdal_dataset.h gdal_defaultoverviews.h gdal_driver.h \
    gdal_drivermanager.h gdal_frmts.h gdal_fwd.h gdal_gcp.h \
    gdal_geotransform.h gdal_majorobject.h gdal_maskbands.h \
    gdal_multidim.h gdal_multidim_cpp.h gdal_multidomainmetadata.h \
    gdal_openinfo.h gdal_pam_multidim.h gdal_priv.h gdal_raster_cpp.h \
    gdal_rasterband.h gdal_rasterblock.h gdal_relationship.h gdal_utils.h \
    gdal_version.h gdalsubdatasetinfo.h ogr_api.h ogr_core.h \
    ogr_feature.h ogr_featurestyle.h ogr_geomcoordinateprecision.h \
    ogr_geometry.h ogr_refcountedptr.h ogr_spatialref.h ogr_srs_api.h
.include "mk.lib.mk"
