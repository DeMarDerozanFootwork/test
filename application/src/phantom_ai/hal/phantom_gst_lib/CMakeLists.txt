project(phantom_gst)

find_library(GSTREAMER_LIB gstreamer-1.0 REQUIRED)
find_library(GSTAPP_LIB gstapp-1.0 REQUIRED)
find_library(GSTBASE_LIB gstbase-1.0 REQUIRED)
find_library(GOBJECT_LIB gobject-2.0 REQUIRED)
find_library(GLIB_LIB glib-2.0 REQUIRED)

set(PHANTOM_GST_INCLUDE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${TI_PSDK_BASE_DIR}/vision_apps/apps/dl_demos/phantom_common/gst_stream
    CACHE INTERNAL "" FORCE)
include_directories(${PHANTOM_GST_INCLUDE_DIR})

add_library(phantom_gst STATIC IMPORTED GLOBAL)

set_target_properties(phantom_gst PROPERTIES
  IMPORTED_LOCATION ${TI_VISION_LIB_PATH}/libphantom_gst.a
  INTERFACE_INCLUDE_DIRECTORIES ${TI_PSDK_BASE_DIR}/vision_apps/apps/dl_demos/phantom_common/gst_stream
  INTERFACE_LINK_LIBRARIES
    "${GSTREAMER_LIB};${GSTAPP_LIB};${GSTBASE_LIB};${GOBJECT_LIB};${GLIB_LIB};-Wl,--as-needed"
  IMPORTED_LINK_INTERFACE_LANGUAGES "C"
)