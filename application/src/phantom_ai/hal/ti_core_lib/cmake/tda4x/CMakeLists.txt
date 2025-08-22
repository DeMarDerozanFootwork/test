phantom_project(ti_core)

# AUTOBRAIN is defined by "cmake .. DCUSTOMER_SPECIFIC_APP=AUTOBRAIN"
# Default: CUSTOMER_SPECIFIC_APPCUSTOMER_SPECIFIC_APP = "UNDEFINED"
SET(CUSTOMER_SPECIFIC_APP "UNDEFINED" CACHE STRING "Customer Specific App")

if (NOT DEFINED ENV{TI_PSDK_SOURCE_DIR})
  message("Environment variable TI_PSDK_SOURCE_DIR not defined. Setting to default folder...")
  SET(TI_PSDK_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "")
else()
  SET(TI_PSDK_BASE_DIR $ENV{TI_PSDK_SOURCE_DIR} CACHE INTERNAL "")
endif()

SET(USER_DIR $ENV{USER})

message("TI PSDK = ${TI_PSDK_BASE_DIR}")

############## Specify SoC and TIDL directory by CMake define ###############
# 'TI_SOC_NAME' and 'TIDL_VER_DIR' is defined by CMake's input arguemnt below.
# ex1) if "CMake -DPHANTOM_AI_TI_SOC=J721E"     (J721E  = TDA4VM's SOC')
#      then "TI_SOC_NAME = J7", "TIDL_VER_DIR = tidl_j7_08_01_00_05", ...
# ex2) if "CMake -DPHANTOM_AI_TI_SOC=J784s4"    (J784s4 = TDA4VH's SOC')
#      then "TI_SOC_NAME = J784S4", "TIDL_VER_DIR=tidl_j784s4_08_02_02_01", ...
# ex3) For another new SDK
#      Find out the output folder name of New SDK like below "J7" or "J784S4" and so on
#       - ${TI_PSDK_BASE_DIR}/vision_apps/out/J7/A72/LINUX/release     => J7
#       - ${TI_PSDK_BASE_DIR}/vision_apps/out/J784S4/A72/LINUX/release => J784S4
#       - ${TI_PSDK_BASE_DIR}/tidl_j7_08_01_00_05/ti_dl/inc            => tidl_j7_08_01_00_05
#       - ${TI_PSDK_BASE_DIR}/tidl_j784s4_08_02_02_01/ti_dl/inc        => tidl_j784s4_08_02_02_01
if (${PHANTOM_AI_TI_SOC} STREQUAL "J721E")
  SET(TI_SOC_NAME "J721E")
  SET(TIDL_VER_DIR "c7x-mma-tidl")
  SET(TI_SDK_VER 9.2.0)
  SET(MPU_CORE_NAME "A72")
elseif (${PHANTOM_AI_TI_SOC} STREQUAL "J721S2")
  SET(TI_SOC_NAME "J721S2")
  SET(MPU_CORE_NAME "A72")
  if(DEFINED TARGET_BOARD_EVM_AL_86)
    SET(TIDL_VER_DIR "tidl_j721s2_08_06_00_10")
    SET(TI_SDK_VER 8.6.0)
  else()
    SET(TIDL_VER_DIR "c7x-mma-tidl")
    SET(TI_SDK_VER 10.1.0)
  endif()
elseif (${PHANTOM_AI_TI_SOC} STREQUAL "J722S")
  SET(TI_SOC_NAME "J722S")
  SET(TIDL_VER_DIR "c7x-mma-tidl")
  SET(TI_SDK_VER 10.1.0)
  SET(MPU_CORE_NAME "A53")
elseif (${PHANTOM_AI_TI_SOC} STREQUAL "J784S4")
  SET(TI_SOC_NAME "J784S4")
  SET(TIDL_VER_DIR "c7x-mma-tidl")
  SET(TI_SDK_VER 10.1.0)
  SET(MPU_CORE_NAME "A72")
  if (PHANTOM_AI_TI_SDK)
    # 'TI_SDK_VER' can be defined by CMake's input arguemnt below.
    # ex)"CMake -PHANTOM_AI_TI_SDK=8.2.2"
    if (${PHANTOM_AI_TI_SDK} STREQUAL "8.2.2")
      SET(TIDL_VER_DIR "tidl_j784s4_08_02_02_01")
      SET(TI_SDK_VER ${PHANTOM_AI_TI_SDK})
    endif()
  endif()
else()
  message(FATAL_ERROR "Unsupported value of PHANTOM_AI_TI_SOC ${PHANTOM_AI_TI_SOC}")
endif()

# Check if SDK_PROFILE environment variable is defined
if (DEFINED ENV{SDK_PROFILE})
  set(SDK_PROFILE "$ENV{SDK_PROFILE}")
else()
  set(SDK_PROFILE "release")
endif()
set(TI_VISION_LIB_PATH "${TI_PSDK_BASE_DIR}/vision_apps/out/${TI_SOC_NAME}/${MPU_CORE_NAME}/LINUX/${SDK_PROFILE}" CACHE INTERNAL "")
set(TI_IMAGING_LIB_PATH "${TI_PSDK_BASE_DIR}/imaging/lib/${TI_SOC_NAME}/${MPU_CORE_NAME}/LINUX/${SDK_PROFILE}")
set(TIOVX_LIB_PATH "${TI_PSDK_BASE_DIR}/tiovx/lib/${TI_SOC_NAME}/${MPU_CORE_NAME}/LINUX/${SDK_PROFILE}")
set(TI_TARGET_LIB_PATH "${TI_PSDK_BASE_DIR}/targetfs/usr/lib")
set(TI_OVX_DIR "${TI_PSDK_BASE_DIR}/tiovx")
if (TI_SDK_VER VERSION_GREATER_EQUAL 9.0.0)
  set(TI_APP_UTILS_PATH "${TI_PSDK_BASE_DIR}/app_utils/lib/${TI_SOC_NAME}/${MPU_CORE_NAME}/LINUX/${SDK_PROFILE}")
  set(TI_C7X_MMA_TIDL_PATH "${TI_PSDK_BASE_DIR}/c7x-mma-tidl/arm-tidl/tiovx_kernels/lib/${TI_SOC_NAME}/${MPU_CORE_NAME}/LINUX/${SDK_PROFILE}")
  set(TI_APP_UTILS_ISS_PATH ${TI_IMAGING_LIB_PATH})
  set(TI_VX_KERNELS_HWA_PATH ${TI_IMAGING_LIB_PATH})
  set(TI_VIDEO_IO_PATH "${TI_PSDK_BASE_DIR}/video_io/lib/${TI_SOC_NAME}/${MPU_CORE_NAME}/LINUX/${SDK_PROFILE}")
else()
  set(TI_APP_UTILS_PATH "${TI_PSDK_BASE_DIR}/vision_apps/out/${TI_SOC_NAME}/${MPU_CORE_NAME}/LINUX/${SDK_PROFILE}")
  set(TI_C7X_MMA_TIDL_PATH ${TIOVX_LIB_PATH})
  set(TI_APP_UTILS_ISS_PATH ${TI_VISION_LIB_PATH})
  set(TI_VX_KERNELS_HWA_PATH ${TIOVX_LIB_PATH})
endif()

if (${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN")
  set(AB_TARGET_INC_PATH ${TI_PSDK_BASE_DIR}/../DRV_TDA4X_HAT_MASTER_SDK_v1.4.1_220921_347162/Libs/inc
                          ${TI_PSDK_BASE_DIR}/../DRV_TDA4X_HAT_MASTER_SDK_v1.4.1_220921_347162/Libs/lib/linux_a72/osal/inc)
  set(AB_TARGET_LIB_PATH ${TI_PSDK_BASE_DIR}/../DRV_TDA4X_HAT_MASTER_SDK_v1.4.1_220921_347162/Libs/lib)
elseif (${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_ECO")
  set(AB_TARGET_INC_PATH ${TI_PSDK_BASE_DIR}/../autobrain_sdk/Libs/inc
                          ${TI_PSDK_BASE_DIR}/../autobrain_sdk/Libs/lib/linux_a72/osal/inc)
  set(AB_TARGET_LIB_PATH ${TI_PSDK_BASE_DIR}/../autobrain_sdk/Libs/lib)
elseif (${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_VH")
  set(AB_TARGET_INC_PATH ${TI_PSDK_BASE_DIR}/../autobrain_sdk/Libs/inc
                          ${TI_PSDK_BASE_DIR}/../autobrain_sdk/Libs/lib/vBMgt/inc/
                          ${TI_PSDK_BASE_DIR}/../autobrain_sdk/Libs/lib/module_manager/inc/
                          ${TI_PSDK_BASE_DIR}/../autobrain_sdk/Libs/lib/log/inc/
                          ${TI_PSDK_BASE_DIR}/../autobrain_sdk/Libs/lib/linux_a72/osal/inc
                          ${TI_PSDK_BASE_DIR}/../autobrain_sdk/Libs/sdk/bios_libs/include/sys
                          ${TI_PSDK_BASE_DIR}/../autobrain_sdk/Libs/sdk/bios_libs/include/drv
                          ${TI_PSDK_BASE_DIR}/../autobrain_sdk/Libs/sdk/bios_libs/drv.log/include)
  set(AB_TARGET_LIB_PATH ${TI_PSDK_BASE_DIR}/../autobrain_sdk/Libs/lib)
  set(AB_TARGET_BIOS_LIB_PATH ${TI_PSDK_BASE_DIR}/../autobrain_sdk/Libs/sdk/bios_libs)
else()
  set(AB_TARGET_INC_PATH "")
  set(AB_TARGET_LIB_PATH "")
endif()

if (TI_SDK_VER VERSION_GREATER_EQUAL 9.0.0)
  set(TI_OVX_INC_DIR ${TI_OVX_DIR}/include
  ${TI_OVX_DIR}/kernels/include
  ${TI_OVX_DIR}/kernels_j7/include
  ${TI_PSDK_BASE_DIR}/${TIDL_VER_DIR}/arm-tidl/tiovx_kernels/include
  ${TI_PSDK_BASE_DIR}/${TIDL_VER_DIR}/arm-tidl/rt/inc
  ${TI_PSDK_BASE_DIR}/${TIDL_VER_DIR}/ti_dl/inc
  ${TI_PSDK_BASE_DIR}/ivision
  ${TI_OVX_DIR}/utils/include
  ${TI_OVX_DIR}/source/include
  ${TI_OVX_DIR}/tutorial/
  ${TI_OVX_DIR}/conformance_tests/
  ${TI_OVX_DIR}/tutorial/ch01_common
  ${TI_PSDK_BASE_DIR}/vision_apps/kernels/fileio/include/TI
  ${TI_PSDK_BASE_DIR}/vision_apps/kernels/fileio/host
  ${TI_PSDK_BASE_DIR}/vision_apps/utils/hwa/include)
else()
  set(TI_OVX_INC_DIR ${TI_OVX_DIR}/include
    ${TI_OVX_DIR}/kernels/include
    ${TI_OVX_DIR}/kernels_j7/include
    ${TI_PSDK_BASE_DIR}/${TIDL_VER_DIR}/ti_dl/inc
    ${TI_PSDK_BASE_DIR}/ivision
    ${TI_OVX_DIR}/utils/include
    ${TI_OVX_DIR}/source/include
    ${TI_OVX_DIR}/tutorial/
    ${TI_OVX_DIR}/conformance_tests/
    ${TI_OVX_DIR}/tutorial/ch01_common
    ${TI_PSDK_BASE_DIR}/vision_apps/kernels/fileio/include/TI
    ${TI_PSDK_BASE_DIR}/vision_apps/kernels/fileio/host
    ${TI_PSDK_BASE_DIR}/vision_apps/utils/hwa/include)
endif()

set(TI_CORE_INCLUDE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${TI_PSDK_BASE_DIR}/vision_apps
    ${TI_PSDK_BASE_DIR}/vision_apps/apps/dl_demos/phantom_common
    ${TI_PSDK_BASE_DIR}/vision_apps/apps/dl_demos/phantom_common/multi_cam
    ${TI_PSDK_BASE_DIR}/vision_apps/apps/basic_demos/multi_cam
    ${TI_PSDK_BASE_DIR}/vision_apps/apps/basic_demos/phantom_multi_cam_hat
    ${TI_PSDK_BASE_DIR}/vision_apps/modules/include
    ${TI_PSDK_BASE_DIR}/vision_apps/phantom-openvx-extensions/include
    ${TI_OVX_INC_DIR}
    ${AB_TARGET_INC_PATH}
    CACHE INTERNAL "" FORCE)
include_directories(${TI_CORE_INCLUDE_DIR} ${PHANTOM_AI_CORE_INCLUDE_DIR})

##########################################################

set(TDA4X_IMPORTED_LIBS
    vx_framework
    vx_platform_psdk_j7_linux
    vx_kernels_hwa
    vx_kernels_host_utils
    vx_kernels_openvx_core
    vx_kernels_target_utils
    vx_kernels_tidl
    vx_kernels_imaging
    vx_target_kernels_source_sink
    vx_utils
    vx_vxu
    vx_kernels_img_proc
    vx_target_kernels_img_proc_a72
    vx_kernels_fileio
    vx_target_kernels_fileio
    app_utils_hwa
    app_utils_perf_stats
    app_utils_draw2d
    app_utils_grpx
    app_utils_console_io
    app_utils_ipc
    app_utils_iss
    app_utils_mem
    app_utils_remote_service
    app_tirtos_linux_mpu1_common
    ti_rpmsg_char
    ti_phantom
    pvx_decode_host
    CACHE INTERNAL "" FORCE)

if(TI_SDK_VER VERSION_GREATER_EQUAL 9.2.0)
  # Add libs
  set(TDA4X_IMPORTED_LIBS
      ${TDA4X_IMPORTED_LIBS}
      app_utils_timer
      )
endif()

if(TI_SDK_VER VERSION_GREATER_EQUAL 9.0.0)
  # Add libs
  set(TDA4X_IMPORTED_LIBS
      ${TDA4X_IMPORTED_LIBS}
      vx_kernels_openvx_ext
      vx_nested_kernels_tidl
      vx_kernels_test_kernels
      vx_kernels_video_io
      app_utils_file_io
      )
endif()

if(TI_SDK_VER VERSION_GREATER_EQUAL 8.5.0)
  # Add libs
  set(TDA4X_IMPORTED_LIBS
      ${TDA4X_IMPORTED_LIBS}
      vx_hwa_target_kernels
      )
endif()

if (${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN" OR ${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_ECO" OR ${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_VH" )
  # Add libs
  set(TDA4X_IMPORTED_LIBS
      ${TDA4X_IMPORTED_LIBS}
      hal.gtc
      HAT_MDL_IPC
      HAT_SUBIPC_COMMON
      HAT_MQSTAR
      HAT_AD_OSAL
      HAT_CHKSUM
      HAT_RSQ
      HAT_MISC
      HAT_SHAREMEM
      HAT_MQUEUE
      HAT_MDL_CFG
      HAT_CRC
      HAT_FIFO
      HAT_MDL_ADVANCE_IPC
      HAT_MODULE_MANAGER
      HAT_MRADAR
      HAT_PRTLVL
      HAT_SYSTEM_CMD
      HAT_MCUMSG_COMMON
      HAT_MCUMSG_RWAPI
      HAT_AD_LOG
      HAT_SYSVERSION
      HAT_SHAREBUF_PHYMEM
      HAT_ALG_MODEL_LOAD
      HAT_MPBUF
      HAT_SHAREBUF
      HAT_VBMGT
      HAL_LOG
      HAL_SYSTEM
     )

  if (${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_ECO" OR ${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_VH" )
    set(TDA4X_IMPORTED_LIBS
        ${TDA4X_IMPORTED_LIBS}
        HAT_MDL_SUBIPC
        vx_app_modules
        HAL_MCUPROT
        HAT_DSPBOOT
        ${TI_PSDK_BASE_DIR}/../autobrain_sdk/Libs/hat/task/process20/out/libprocess20.a
       )
    set(LIBHAT_MDL_SUBIPC_A    "subipc/linux_a72/libHAT_SUBIPC.a")
    set(LIBHAL_GTC_A           "drv.gtc/lib/LINUX/A72/libhal.gtc.a")
    set(LIBHAT_CRC_A           "crc/linux_a72/libHAT_CRC.a")
    set(LIBHAT_MDL_CFG_A       "cfg/linux_a72/libHAT_MDL_CFG.a")
    set(LIBHAT_MQUEUE_A        "mqueue/linux_a72/libHAT_MQUEUE.a")
    set(LIBHAT_SHAREMEM_A      "sharemem/linux_a72/libHAT_SHAREMEM.a")
    set(LIBHAT_MISC_A          "misc/linux_a72/libHAT_MISC.a")
    set(LIBHAT_RSQ_A           "rsq/linux_a72/libHAT_RSQ.a")
    set(LIBHAT_CHKSUM_A        "chksum/linux_a72/libHAT_CHKSUM.a")
    set(LIBHAT_AD_OSAL_A       "osal/linux_a72/libHAT_AD_OSAL.a")
    set(LIBHAT_MQSTAR_A        "mqstar/linux_a72/libHAT_MQSTAR.a")
    set(LIBHAT_SUBIPC_COMMON_A "subipc_common/linux_a72/libHAT_SUBIPC_COMMON.a")
    set(LIBHAT_MDL_IPC_A       "ipc/linux_a72/libHAT_MDL_IPC.a")
    set(LIBHAT_FIFO_A          "fifo/linux_a72/libHAT_FIFO.a")
    set(LIBHAT_MDL_ADVANCE_IPC_A "advance_ipc/linux_a72/libHAT_MDL_ADVANCE_IPC.a")
    set(LIBHAT_MODULE_MANAGER_A "module_manager/linux_a72/libHAT_MODULE_MANAGER.a")
    set(LIBHAT_MRADAR_A        "mradar/linux_a72/libHAT_MRADAR.a")
    set(LIBHAT_PRTLVL_A        "prtlvl/linux_a72/libHAT_PRTLVL.a")
    set(LIBHAT_SYSTEM_CMD_A    "system_cmd/linux_a72/libHAT_SYSTEM_CMD.a")
    set(LIBHAT_MCUMSG_COMMON_A "mcumsg_common/linux_a72/libHAT_MCUMSG_COMMON.a")
    set(LIBHAT_MCUMSG_RWAPI_A  "mcumsg_rwapi/linux_a72/libHAT_MCUMSG_RWAPI.a")
    set(LIBHAT_AD_LOG_A        "log/linux_a72/libHAT_AD_LOG.a")
    set(LIBHAT_SYSVERSION_A    "sysVersion/linux_a72/libHAT_SYSVERSION.a")
    set(LIBHAT_SHAREBUF_PHYMEM_A "sharebuf_phymem/linux_a72/libHAT_SHAREBUF_PHYMEM.a")
    set(LIBHAT_ALG_MODEL_LOAD_A  "alg_model_load/linux_a72/libHAT_MDL_ALG_MODEL_LOAD.a")
    set(LIBHAT_MPBUF_A         "mpbuf/linux_a72/libHAT_MPBUF.a")
    set(LIBHAT_SHAREBUF_A      "sharebuf/linux_a72/libHAT_SHAREBUF.a")
    set(LIBHAT_VBMGT_A         "vBMgt/linux_a72/libHAT_VBMGT.a")
    set(LIBHAL_LOG_A           "drv.log/lib/LINUX/A72/libhal.log.a")
    set(LIBHAL_SYSTEM_A        "os.sys/lib/LINUX/A72/libsystem.a")
    set(LIBHAL_MCUPROT_A        "mcuprot/linux_a72/libHAT_MCUPROT.a")
    set(LIBHAT_DSPBOOT_A        "dspBoot/linux_a72/libHAT_DSPBOOT.a")

  else() # AUTOBRAIN
    # Add libs
    set(TDA4X_IMPORTED_LIBS
        ${TDA4X_IMPORTED_LIBS}
        HAT_MDL_SUBIPC
       )
    set(LIBHAT_MDL_SUBIPC_A    "linux_a72/subipc/libHAT_MDL_SUBIPC.a")
    set(LIBHAL_GTC_A           "linux_a72/osal/libhal.gtc.a")
    set(LIBHAT_CRC_A           "linux_a72/crc/libHAT_CRC.a")
    set(LIBHAT_MDL_CFG_A       "linux_a72/cfg/libHAT_MDL_CFG.a")
    set(LIBHAT_MQUEUE_A        "linux_a72/mqueue/libHAT_MQUEUE.a")
    set(LIBHAT_SHAREMEM_A      "linux_a72/sharemem/libHAT_SHAREMEM.a")
    set(LIBHAT_MISC_A          "linux_a72/misc/libHAT_MISC.a")
    set(LIBHAT_RSQ_A           "linux_a72/rsq/libHAT_RSQ.a")
    set(LIBHAT_CHKSUM_A        "linux_a72/chksum/libHAT_CHKSUM.a")
    set(LIBHAT_AD_OSAL_A       "linux_a72/osal/libHAT_AD_OSAL.a")
    set(LIBHAT_MQSTAR_A        "linux_a72/mqstar/libHAT_MQSTAR.a")
    set(LIBHAT_SUBIPC_COMMON_A "linux_a72/subipc_common/libHAT_SUBIPC_COMMON.a")
    set(LIBHAT_MDL_IPC_A       "linux_a72/ipc/libHAT_MDL_IPC.a")
  endif()

  add_library(HAT_MDL_SUBIPC STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_MDL_SUBIPC PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_MDL_SUBIPC_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(hal.gtc STATIC IMPORTED GLOBAL)
  set_target_properties(hal.gtc PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_BIOS_LIB_PATH}/${LIBHAL_GTC_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_CRC STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_CRC PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_CRC_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_MDL_CFG STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_MDL_CFG PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_MDL_CFG_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_MQUEUE STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_MQUEUE PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_MQUEUE_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_SHAREMEM STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_SHAREMEM PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_SHAREMEM_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_MISC STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_MISC PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_MISC_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_RSQ STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_RSQ PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_RSQ_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_CHKSUM STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_CHKSUM PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_CHKSUM_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_AD_OSAL STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_AD_OSAL PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_AD_OSAL_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_MQSTAR STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_MQSTAR PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_MQSTAR_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_SUBIPC_COMMON STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_SUBIPC_COMMON PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_SUBIPC_COMMON_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_MDL_IPC STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_MDL_IPC PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_MDL_IPC_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_FIFO STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_FIFO PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_FIFO_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_MDL_ADVANCE_IPC STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_MDL_ADVANCE_IPC PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_MDL_ADVANCE_IPC_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_MODULE_MANAGER STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_MODULE_MANAGER PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_MODULE_MANAGER_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_MRADAR STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_MRADAR PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_MRADAR_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_PRTLVL STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_PRTLVL PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_PRTLVL_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_SYSTEM_CMD STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_SYSTEM_CMD PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_SYSTEM_CMD_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_MCUMSG_COMMON STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_MCUMSG_COMMON PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_MCUMSG_COMMON_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_MCUMSG_RWAPI STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_MCUMSG_RWAPI PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_MCUMSG_RWAPI_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_AD_LOG STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_AD_LOG PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_AD_LOG_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_SYSVERSION STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_SYSVERSION PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_SYSVERSION_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_SHAREBUF_PHYMEM STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_SHAREBUF_PHYMEM PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_SHAREBUF_PHYMEM_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_ALG_MODEL_LOAD STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_ALG_MODEL_LOAD PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_ALG_MODEL_LOAD_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_MPBUF STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_MPBUF PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_MPBUF_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_SHAREBUF STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_SHAREBUF PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_SHAREBUF_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_VBMGT STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_VBMGT PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_VBMGT_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")


  add_library(HAL_LOG STATIC IMPORTED GLOBAL)
  set_target_properties(HAL_LOG PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_BIOS_LIB_PATH}/${LIBHAL_LOG_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAL_SYSTEM STATIC IMPORTED GLOBAL)
  set_target_properties(HAL_SYSTEM PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_BIOS_LIB_PATH}/${LIBHAL_SYSTEM_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAL_MCUPROT STATIC IMPORTED GLOBAL)
  set_target_properties(HAL_MCUPROT PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAL_MCUPROT_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")

  add_library(HAT_DSPBOOT STATIC IMPORTED GLOBAL)
  set_target_properties(HAT_DSPBOOT PROPERTIES
                        IMPORTED_LOCATION ${AB_TARGET_LIB_PATH}/${LIBHAT_DSPBOOT_A}
                        IMPORTED_LINK_INTERFACE_LANGUAGES "C")
endif()

if(${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "TDA4X")
    if(DEFINED BUILD_TI_IMAGE_KERNEL)

      set(TDA4X_IMPORTED_LIBS
          ${TDA4X_IMPORTED_LIBS}
          vx_tutorial
          ti_phantom_vision_kernel_abstract
          CACHE INTERNAL "" FORCE)

      add_library(ti_phantom_vision_kernel_abstract STATIC IMPORTED GLOBAL)
      set_target_properties(ti_phantom_vision_kernel_abstract PROPERTIES
                            IMPORTED_LOCATION ${TI_VISION_LIB_PATH}/libti_phantom_vision_kernel_abstract.a
                            IMPORTED_LINK_INTERFACE_LANGUAGES "C")

      add_library(vx_tutorial STATIC IMPORTED GLOBAL)
      set_target_properties(vx_tutorial PROPERTIES
                            IMPORTED_LOCATION ${TIOVX_LIB_PATH}/libvx_tutorial.a
                            IMPORTED_LINK_INTERFACE_LANGUAGES "C")

    endif()
    if(TI_SDK_VER VERSION_GREATER_EQUAL 8.5.0)
      if (TI_SDK_VER VERSION_GREATER_EQUAL 9.0.0)
        add_library(vx_hwa_target_kernels STATIC IMPORTED GLOBAL)
        set_target_properties(vx_hwa_target_kernels PROPERTIES
                              IMPORTED_LOCATION ${TIOVX_LIB_PATH}/libvx_target_kernels_openvx_ext.a
                              IMPORTED_LINK_INTERFACE_LANGUAGES "C")
        add_library(vx_kernels_openvx_ext STATIC IMPORTED GLOBAL)
        set_target_properties(vx_kernels_openvx_ext PROPERTIES
                              IMPORTED_LOCATION ${TIOVX_LIB_PATH}/libvx_kernels_openvx_ext.a
                              IMPORTED_LINK_INTERFACE_LANGUAGES "C")
        add_library(vx_nested_kernels_tidl STATIC IMPORTED GLOBAL)
        set_target_properties(vx_nested_kernels_tidl PROPERTIES
                              IMPORTED_LOCATION ${TI_C7X_MMA_TIDL_PATH}/libvx_nested_kernels_tidl.a
                              IMPORTED_LINK_INTERFACE_LANGUAGES "C")
        add_library(vx_kernels_test_kernels STATIC IMPORTED GLOBAL)
        set_target_properties(vx_kernels_test_kernels PROPERTIES
                              IMPORTED_LOCATION ${TIOVX_LIB_PATH}/libvx_kernels_test_kernels.a
                              IMPORTED_LINK_INTERFACE_LANGUAGES "C")
        add_library(vx_kernels_video_io STATIC IMPORTED GLOBAL)
        set_target_properties(vx_kernels_video_io PROPERTIES
                              IMPORTED_LOCATION ${TI_VIDEO_IO_PATH}/libvx_kernels_video_io.a
                              IMPORTED_LINK_INTERFACE_LANGUAGES "C")
        add_library(app_utils_file_io STATIC IMPORTED GLOBAL)
        set_target_properties(app_utils_file_io PROPERTIES
                              IMPORTED_LOCATION ${TI_APP_UTILS_PATH}/libapp_utils_file_io.a
                              IMPORTED_LINK_INTERFACE_LANGUAGES "C")
      else()
        add_library(vx_hwa_target_kernels STATIC IMPORTED GLOBAL)
        set_target_properties(vx_hwa_target_kernels PROPERTIES
                              IMPORTED_LOCATION ${TIOVX_LIB_PATH}/libvx_hwa_target_kernels.a
                              IMPORTED_LINK_INTERFACE_LANGUAGES "C")
      endif()
    endif()
endif()

add_library(ti_phantom STATIC IMPORTED GLOBAL)
set_target_properties(ti_phantom PROPERTIES
                      IMPORTED_LOCATION ${TI_VISION_LIB_PATH}/libti_phantom.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(app_utils_perf_stats STATIC IMPORTED GLOBAL)
set_target_properties(app_utils_perf_stats PROPERTIES
                      IMPORTED_LOCATION ${TI_APP_UTILS_PATH}/libapp_utils_perf_stats.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(app_utils_grpx STATIC IMPORTED GLOBAL)
set_target_properties(app_utils_grpx PROPERTIES
                      IMPORTED_LOCATION ${TI_VISION_LIB_PATH}/libapp_utils_grpx.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(app_utils_draw2d STATIC IMPORTED GLOBAL)
set_target_properties(app_utils_draw2d PROPERTIES
                      IMPORTED_LOCATION ${TI_VISION_LIB_PATH}/libapp_utils_draw2d.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(app_utils_console_io STATIC IMPORTED GLOBAL)
set_target_properties(app_utils_console_io PROPERTIES
                      IMPORTED_LOCATION ${TI_APP_UTILS_PATH}/libapp_utils_console_io.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

if (TI_SDK_VER VERSION_GREATER_EQUAL 9.2.0)
  add_library(app_utils_timer STATIC IMPORTED GLOBAL)
  set_target_properties(app_utils_timer PROPERTIES
                      IMPORTED_LOCATION ${TI_APP_UTILS_PATH}/libapp_utils_timer.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")
endif()

add_library(app_utils_ipc STATIC IMPORTED GLOBAL)
set_target_properties(app_utils_ipc PROPERTIES
                      IMPORTED_LOCATION ${TI_APP_UTILS_PATH}/libapp_utils_ipc.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(app_utils_iss STATIC IMPORTED GLOBAL)
set_target_properties(app_utils_iss PROPERTIES
                      IMPORTED_LOCATION ${TI_APP_UTILS_ISS_PATH}/libapp_utils_iss.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(app_utils_mem STATIC IMPORTED GLOBAL)
set_target_properties(app_utils_mem PROPERTIES
                      IMPORTED_LOCATION ${TI_APP_UTILS_PATH}/libapp_utils_mem.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(app_utils_remote_service STATIC IMPORTED GLOBAL)
set_target_properties(app_utils_remote_service PROPERTIES
                      IMPORTED_LOCATION ${TI_APP_UTILS_PATH}/libapp_utils_remote_service.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(app_tirtos_linux_mpu1_common STATIC IMPORTED GLOBAL)
set_target_properties(app_tirtos_linux_mpu1_common PROPERTIES
                      IMPORTED_LOCATION ${TI_VISION_LIB_PATH}/libapp_rtos_linux_mpu1_common.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(vx_kernels_img_proc STATIC IMPORTED GLOBAL)
set_target_properties(vx_kernels_img_proc PROPERTIES
                      IMPORTED_LOCATION ${TI_VISION_LIB_PATH}/libvx_kernels_img_proc.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(vx_target_kernels_img_proc_a72 STATIC IMPORTED GLOBAL)
set_target_properties(vx_target_kernels_img_proc_a72 PROPERTIES
                      IMPORTED_LOCATION ${TI_VISION_LIB_PATH}/libvx_target_kernels_img_proc_a72.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(vx_kernels_fileio STATIC IMPORTED GLOBAL)
set_target_properties(vx_kernels_fileio PROPERTIES
                      IMPORTED_LOCATION ${TI_VISION_LIB_PATH}/libvx_kernels_fileio.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(vx_target_kernels_fileio STATIC IMPORTED GLOBAL)
set_target_properties(vx_target_kernels_fileio PROPERTIES
                      IMPORTED_LOCATION ${TI_VISION_LIB_PATH}/libvx_target_kernels_fileio.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(app_utils_hwa STATIC IMPORTED GLOBAL)
set_target_properties(app_utils_hwa PROPERTIES
                      IMPORTED_LOCATION ${TI_APP_UTILS_ISS_PATH}/libapp_utils_hwa.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(vx_framework STATIC IMPORTED GLOBAL)
set_target_properties(vx_framework PROPERTIES
                      IMPORTED_LOCATION ${TIOVX_LIB_PATH}/libvx_framework.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(vx_kernels_hwa STATIC IMPORTED GLOBAL)
set_target_properties(vx_kernels_hwa PROPERTIES
                      IMPORTED_LOCATION ${TI_VX_KERNELS_HWA_PATH}/libvx_kernels_hwa.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(vx_kernels_host_utils STATIC IMPORTED GLOBAL)
set_target_properties(vx_kernels_host_utils PROPERTIES
                      IMPORTED_LOCATION ${TIOVX_LIB_PATH}/libvx_kernels_host_utils.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(vx_kernels_openvx_core STATIC IMPORTED GLOBAL)
set_target_properties(vx_kernels_openvx_core PROPERTIES
                      IMPORTED_LOCATION ${TIOVX_LIB_PATH}/libvx_kernels_openvx_core.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(vx_kernels_target_utils STATIC IMPORTED GLOBAL)
set_target_properties(vx_kernels_target_utils PROPERTIES
                      IMPORTED_LOCATION ${TIOVX_LIB_PATH}/libvx_kernels_target_utils.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(vx_kernels_tidl STATIC IMPORTED GLOBAL)
set_target_properties(vx_kernels_tidl PROPERTIES
                      IMPORTED_LOCATION ${TI_C7X_MMA_TIDL_PATH}/libvx_kernels_tidl.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(vx_kernels_imaging STATIC IMPORTED GLOBAL)
set_target_properties(vx_kernels_imaging PROPERTIES
                      IMPORTED_LOCATION ${TI_IMAGING_LIB_PATH}/libvx_kernels_imaging.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(vx_target_kernels_source_sink STATIC IMPORTED GLOBAL)
set_target_properties(vx_target_kernels_source_sink PROPERTIES
                      IMPORTED_LOCATION ${TIOVX_LIB_PATH}/libvx_target_kernels_source_sink.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(vx_platform_psdk_j7_linux STATIC IMPORTED GLOBAL)
set_target_properties(vx_platform_psdk_j7_linux PROPERTIES
                      IMPORTED_LOCATION ${TIOVX_LIB_PATH}/libvx_platform_psdk_j7.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(vx_utils STATIC IMPORTED GLOBAL)
set_target_properties(vx_utils PROPERTIES
                      IMPORTED_LOCATION ${TIOVX_LIB_PATH}/libvx_utils.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(vx_vxu STATIC IMPORTED GLOBAL)
set_target_properties(vx_vxu PROPERTIES
                      IMPORTED_LOCATION ${TIOVX_LIB_PATH}/libvx_vxu.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(ti_rpmsg_char SHARED IMPORTED GLOBAL)
set_target_properties(ti_rpmsg_char PROPERTIES
                      IMPORTED_LOCATION ${TI_TARGET_LIB_PATH}/libti_rpmsg_char.so
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(vx_app_modules STATIC IMPORTED GLOBAL)
set_target_properties(vx_app_modules PROPERTIES
                      IMPORTED_LOCATION ${TI_VISION_LIB_PATH}/libvx_app_modules.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(pvx_decode_host STATIC IMPORTED GLOBAL)
set_target_properties(pvx_decode_host PROPERTIES
                      IMPORTED_LOCATION ${TI_VISION_LIB_PATH}/libpvx_decode_host.a
                      IMPORTED_LINK_INTERFACE_LANGUAGES "CXX")

# Make the C lib for all the TIVX stuff
set(CMAKE_SHARED_LINKER_FLAGS "-fPIC")
if (${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN" OR ${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_ECO" OR ${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_VH" )
  add_library(ti_core SHARED
      ../../src/hal_hw.c
      ../../src/ab_api.c
      ../../src/ab_log_api.c
      ../../src/tidl_interface.cpp
      ../../src/pvx_interface.cpp
      )
else ()
  add_library(ti_core SHARED ../../src/hal_hw.c ../../src/tidl_interface.cpp ../../src/pvx_interface.cpp)
endif ()
if(PHANTOM_AI_BUILD_ASAN)
  message("AddressSanitizer is enabled for ti_core.")
  target_compile_options(ti_core PRIVATE -fsanitize=address -fsanitize-recover=address)
  target_link_libraries(ti_core asan)
endif()

if(PHANTOM_AI_BUILD_TSAN)
  message("ThreadSanitizer is enabled for ti_core.")
  target_compile_options(ti_core PRIVATE -fsanitize=thread)
  target_link_options(ti_core PRIVATE -fsanitize=thread)
endif()

target_link_libraries(ti_core -Wl,--whole-archive ${TDA4X_IMPORTED_LIBS} -Wl,--no-whole-archive core -lrt)

## Mark executables and/or libraries for installation.
install(
  TARGETS ti_core
  RUNTIME DESTINATION ${PHANTOM_OS_INSTALL_BIN_DIR}
  LIBRARY DESTINATION ${PHANTOM_OS_INSTALL_LIB_DIR}
)
