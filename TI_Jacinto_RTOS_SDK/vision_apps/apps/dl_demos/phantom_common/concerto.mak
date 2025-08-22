ifeq ($(TARGET_CPU),$(filter $(TARGET_CPU), x86_64 A72))

include $(PRELUDE)

TARGET      := ti_phantom
CSOURCES    := display_tivx.c tidl_tivx.c preproc_cpu.c iccom.c
CSOURCES    += multi_cam/app_aewb_module.c  multi_cam/app_ldc_module.c multi_cam/app_viss_module.c multi_cam/app_capture_module.c multi_cam/app_img_mosaic_module.c multi_cam/app_sensor_module.c multi_cam/multi_cam_tivx.c multi_cam/ringbuffer.c

IDIRS       += $(VISION_APPS_PATH)/apps/dl_demos/phantom_common/gst_stream
IDIRS       += $(TIOVX_PATH)/tutorial
IDIRS       += $(TIOVX_PATH)/tutorial/ch01_common
IDIRS       += $(TIOVX_PATH)/tutorial/ch03_graph


ifeq ($(TARGET_CPU),x86_64)

TARGETTYPE  := exe
CSOURCES    += main.c main_x86.c

include $(VISION_APPS_PATH)/apps/concerto_x86_64_inc.mak

endif

ifeq ($(TARGET_CPU),A72)
ifeq ($(TARGET_OS),$(filter $(TARGET_OS), LINUX QNX))

#TARGETTYPE  := exe
#CSOURCES    += main.c main_linux_arm.c

TARGETTYPE  := library
CSOURCES    += ti_init_arm.c

include $(VISION_APPS_PATH)/apps/concerto_mpu_inc.mak

endif
endif

ifeq ($(TARGET_OS),SYSBIOS)
ifeq ($(TARGET_CPU),$(filter $(TARGET_CPU), A72))
TARGETTYPE  := library
endif
endif

IDIRS += $(IMAGING_IDIRS)
IDIRS += $(VISION_APPS_KERNELS_IDIRS)

STATIC_LIBS += $(IMAGING_LIBS)
STATIC_LIBS += $(VISION_APPS_KERNELS_LIBS)


include $(FINALE)

endif
