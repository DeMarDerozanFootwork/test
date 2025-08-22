ifeq ($(TARGET_CPU),$(filter $(TARGET_CPU), x86_64 A72))

include $(PRELUDE)

TARGET      := phantom_gst
TARGETTYPE  := library
CSOURCES 	:= gst_stream.c

IDIRS += $(VISION_APPS_KERNELS_IDIRS)
IDIRS += $(VISION_APPS_MODULES_IDIRS)
IDIRS += $(VISION_APPS_PATH)/apps/dl_demos/phantom_common/multi_cam/
IDIRS += $(LINUX_FS_PATH)/usr/include/gstreamer-1.0/
IDIRS += $(LINUX_FS_PATH)/usr/include/glib-2.0/
IDIRS += $(LINUX_FS_PATH)/usr/lib/glib-2.0/include/

STATIC_LIBS += $(VISION_APPS_KERNELS_LIBS)
STATIC_LIBS += $(VISION_APPS_MODULES_LIBS)

SHARED_LIBS += gstreamer-1.0
SHARED_LIBS += gstapp-1.0
SHARED_LIBS += gstbase-1.0
SHARED_LIBS += gobject-2.0
SHARED_LIBS += glib-2.0

STATIC_LIBS += app_utils_codec_wrapper
STATIC_LIBS += app_utils_gst_wrapper

APP_LDFLAGS += -lgstapp-1.0 -lgstreamer-1.0 -lgobject-2.0 -lglib-2.0

include $(FINALE)

endif
