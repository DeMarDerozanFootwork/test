phantom_project(phantom_linux)

if (NOT DEFINED ENV{TI_PSDK_SOURCE_DIR})
  message("Environment variable TI_PSDK_SOURCE_DIR not defined. Setting to default folder...")
  SET(TI_PSDK_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
else()
  SET(TI_PSDK_BASE_DIR $ENV{TI_PSDK_SOURCE_DIR})
endif()

if (${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN" OR ${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_ECO" OR ${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_VH" )
  set(DEFINE_ENABLE_PROTOBUF_MSG "-DENABLE_PROTOBUF_MSG")
  set(AUTOBRAIN_LINKER_SCRIPT "${TI_PSDK_BASE_DIR}/../autobrain_sdk/Libs/hat/task/process20/lscript.lds")
else ()
  set(DEFINE_ENABLE_PROTOBUF_MSG "")
endif()

if (PHANTOM_AI_USE_CAM_PARAM_INPUT_FOR_VISION_INIT)
  add_definitions(-DUSE_CAM_PARAM_INPUT_FOR_VISION_INIT)
endif()

# Finding catkin below clobbers the CMAKE output target for libs and directories.
# Since we want this defined at the top level CMAKE, back it up here,
# and restore it after the catkin steps.
set(BIN_OUTPUT_DIR ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
set(LIB_OUTPUT_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0 glib-2.0)

if (DEFINE_ENABLE_PROTOBUF_MSG)
  set(Protobuf_INCLUDE_DIR "/usr/include")
endif ()

# Setup Boost Libraries
set(Boost_USE_STATIC_LIBS       OFF)
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME    OFF)

if (USE_ZF_VEHICLE_STATES)
  add_compile_definitions(USE_ZF_VEHICLE_STATES)
endif()

# Minimum Boost version is 1.58, which is installed as a dep for ROS Kinetic.
find_package(Boost 1.58.0)

#message("HAL_INCLUDE_DIR = ${HAL_INCLUDE_DIR}")
#message("TI_CORE_INCLUDE_DIR = ${TI_CORE_INCLUDE_DIR}")

###########################################################################
## include directories
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
  ${PHANTOM_AI_COMMON_INCLUDE_DIR}
  ${PHANTOM_AI_CONTROL_INCLUDE_DIR}
  ${PHANTOM_AI_CORE_INCLUDE_DIR}
  ${PHANTOM_AI_TASK_ENGINE_INCLUDE_DIR}
  ${PHANTOM_AI_DBC_INCLUDE_DIR}
  ${PHANTOM_AI_CAN_INCLUDE_DIR}
  ${PHANTOM_AI_CAN_MSG_PACK_INCLUDE_DIR}
  ${PHANTOM_AI_DISPLAY_INCLUDE_DIR}
  ${PHANTOM_AI_PHANTOM_VISION_INCLUDE_DIR}
  ${PHANTOM_AI_PHANTOM_VISION2_INCLUDE_DIR}
  ${PHANTOM_AI_SENSOR_FUSION_INCLUDE_DIR}
  ${PHANTOM_AI_SUPERVISOR_INCLUDE_DIR}
  ${PHANTOM_AI_TOOLS_INCLUDE_DIR}
  ${PHANTOM_AI_UTILS_INCLUDE_DIR}
  ${PHANTOM_AI_VEHICLE_STATE_PARSER_INCLUDE_DIR}
  ${PHANTOM_AI_ZF_VEHICLE_STATE_PARSER_INCLUDE_DIR}
  ${PHANTOM_AI_PHANTOM_CALIBRATION_INCLUDE_DIR}
  ${PHANTOM_AI_WRAPPERS_INCLUDE_DIR}
  ${embedded_network_INCLUDE_DIR}
  ${PHANTOM_AI_FUNCTIONAL_SAFETY_INCLUDE_DIR}
  ${PHANTOM_AI_TEST_INCLUDE_DIR}
  ${ZEROMQ_INCLUDE_DIR}
  ${PHANTOM_GST_INCLUDE_DIR}
  ${GSTREAMER_INCLUDE_DIRS}
)
if (DEFINE_ENABLE_PROTOBUF_MSG)
  include_directories(${PHANTOM_AI_PROTOBUF_MSGS_INCLUDE_DIR} ${PHANTOM_AI_PROTOBUF_MSGS_MSGS_INCLUDE_DIR} ${Protobuf_INCLUDE_DIRS})
endif ()

# Libraries

if (DEFINE_ENABLE_PROTOBUF_MSG)
  set(protobuf_wrapper "protobuf_wrapper")
  set(ecu_key_manager_wrapper "ecu_key_manager_wrapper")
else ()
  set(protobuf_wrapper "")
  set(ecu_key_manager_wrapper "")
endif ()

if (SW_RELEASE_TARGET STREQUAL "EXTERNAL")
  add_definitions(-DDISABLE_PHANTOM_ARGUMENTS)
  add_definitions(-DENABLE_ECU_KEY_MANAGER)
  set(ecu_key_manager_wrapper "ecu_key_manager_wrapper")
else ()
  set(ecu_key_manager_wrapper "")
endif()

if(PHANTOM_AI_BUILD_ASAN)
  message("AddressSanitizer is enabled for Apps.")
  set(asan_compile_flag "-fsanitize=address" "-fsanitize-recover=address")
  set(asan_lib "asan")
else ()
  set(asan_compile_flag "")
  set(asan_lib "")
endif ()

if(PHANTOM_AI_BUILD_TSAN)
  message("ThreadSanitizer is enabled for Apps.")
  set(tsan_compile_flag "-fsanitize=thread")
  set(tsan_lib "tsan")
else ()
  set(tsan_compile_flag "")
  set(tsan_lib "")
endif()

#
# PhantomVision"2" executables
#
if (PHANTOM_AI_BUILD_PERCEPTION)

if(${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "TDA4X" OR ${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "AM62A")
    include_directories(${HAL_INCLUDE_DIR} ${TI_CORE_INCLUDE_DIR})
    set(HAL_INTERFACE_LIB "hal_interface_lib")
    set(HAL_PROCESS_LIB "hal_process_lib")
endif()

if(ENABLE_ZEROMQ)
  set(zmq_wrapper_lib "zmq_wrapper")
else()
  set(zmq_wrapper_lib "")
endif()

macro(generate_apps)
  if ((mode STREQUAL "_6cam" OR mode STREQUAL "_8mp") AND fullres STREQUAL "_fullres") # Skip full resolution app in 6cam and 8mp mode.
    message(vision${mode}${fullres}${logging}${protobuf} " app is skipped.")
  else()
    add_executable(vision${mode}${fullres}${logging}${protobuf} src/vision_demo2.cpp src/system_mode.cpp)
    target_compile_options(vision${mode}${fullres}${logging}${protobuf} PRIVATE ${asan_compile_flag} ${tsan_compile_flag})
    SET_TARGET_PROPERTIES(vision${mode}${fullres}${logging}${protobuf} PROPERTIES COMPILE_FLAGS "  ${MODE} ${FULLRES_LOG} ${LOGGING}" )
    if (AUTOBRAIN_LINKER_SCRIPT)
      target_link_options(vision${mode}${fullres}${logging}${protobuf} PRIVATE "-T${AUTOBRAIN_LINKER_SCRIPT}")
    endif()
    target_link_libraries(vision${mode}${fullres}${logging}${protobuf} ${asan_lib} ${tsan_lib} vision_wrapper2 ${HAL_PROCESS_LIB}
                          vehicle_state_parser zf_vehicle_state_parser camera_wrapper can_wrapper
                          dbc_vehicle_gateway utils network_tx_wrapper
                          ${protobuf_wrapper} ${ecu_key_manager_wrapper} functional_safety
                          ${zmq_wrapper_lib} ${phantom_gst_lib} ${GSTREAMER_LIBRARIES})
    list(APPEND PHANTOM_LINUX_INSTALL_TARGETS_TDA4X vision${mode}${fullres}${logging}${protobuf})
  endif()
endmacro()

if (${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "TDA4X" OR ${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "AM62A")
  # Separate and customize for AM62A as needed
  # valid APP_MODES definition: "-DAPP_MODE_6CAM", "-DAPP_MODE_FRONT_8MP", "-DAPP_MODE_FRONT_2MP"
  list(APPEND APP_MODES "-DAPP_MODE_FRONT_8MP" "-DAPP_MODE_FRONT_2MP")
  list(APPEND ENABLE_LOGGING "-DENABLE_VISION_LOGGING" "")
  list(APPEND ENABLE_PROTOBUF ${DEFINE_ENABLE_PROTOBUF_MSG} "")
  list(APPEND ENABLE_FULLRES_LOGGING "-DENABLE_FULLRES_IMG_LOGGING" "")

  foreach(MODE IN LISTS APP_MODES)
    if(MODE STREQUAL "-DAPP_MODE_6CAM")
      set(mode "_6cam")
    elseif(MODE STREQUAL "-DAPP_MODE_FRONT_8MP")
      set(mode "_8mp")
    elseif(MODE STREQUAL "-DAPP_MODE_FRONT_2MP")
      set(mode "_2mp")
    else()
      set(mode "")
    endif()
    foreach(FULLRES_LOG IN LISTS ENABLE_FULLRES_LOGGING)
      if(FULLRES_LOG STREQUAL "-DENABLE_FULLRES_IMG_LOGGING")
        set(fullres "_fullres")
      else()
        set(fullres "")
      endif()
      foreach(LOGGING IN LISTS ENABLE_LOGGING)
        if(LOGGING STREQUAL "-DENABLE_VISION_LOGGING")
          set(logging "_logging2")
        else()
          set(logging "_demo2")
        endif()
        if (${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN" OR ${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_ECO" OR ${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_VH")
          foreach(PROTOBUF IN LISTS ENABLE_PROTOBUF)
            if(PROTOBUF STREQUAL "-DENABLE_PROTOBUF_MSG")
              set(protobuf "_protobuf")
            else()
              set(protobuf "")
            endif()
            generate_apps()
          endforeach()
        else()
          generate_apps()
        endif()
      endforeach()
    endforeach()
  endforeach()

  add_executable(resource_monitor src/resource_monitor.cpp)
  if (AUTOBRAIN_LINKER_SCRIPT)
    target_link_options(resource_monitor PRIVATE "-T${AUTOBRAIN_LINKER_SCRIPT}")
  endif()
  target_link_libraries(resource_monitor utils)

  add_executable(print_stats src/print_stats.cpp)
  if (AUTOBRAIN_LINKER_SCRIPT)
    target_link_options(print_stats PRIVATE "-T${AUTOBRAIN_LINKER_SCRIPT}")
  endif()
  target_link_libraries(print_stats ${HAL_PROCESS_LIB} utils)

  # Dual logging Single Camera Vision App
  add_executable(vision_8mp_dual_logging2 src/vision_demo2.cpp src/system_mode.cpp)
  target_compile_options(vision_8mp_dual_logging2 PRIVATE ${asan_compile_flag} ${tsan_compile_flag})
  SET_TARGET_PROPERTIES(vision_8mp_dual_logging2 PROPERTIES COMPILE_FLAGS "-DAPP_MODE_FRONT_8MP -DENABLE_DUAL_LOGGING -DENABLE_VISION_LOGGING")
  if (AUTOBRAIN_LINKER_SCRIPT)
    target_link_options(vision_8mp_dual_logging2 PRIVATE "-T${AUTOBRAIN_LINKER_SCRIPT}")
  endif()
  target_link_libraries(vision_8mp_dual_logging2 ${asan_lib} ${tsan_lib} vision_wrapper2 ${HAL_PROCESS_LIB}
                        vehicle_state_parser zf_vehicle_state_parser camera_wrapper can_wrapper
                        dbc_vehicle_gateway utils network_tx_wrapper
                        ${protobuf_wrapper} ${ecu_key_manager_wrapper} functional_safety ${zmq_wrapper_lib} ${phantom_gst_lib} ${GSTREAMER_LIBRARIES})

  if (${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN" OR ${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_ECO" OR ${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_VH" )
    add_executable(vision_8mp_dual_logging2_protobuf src/vision_demo2.cpp src/system_mode.cpp)
    target_compile_options(vision_8mp_dual_logging2_protobuf PRIVATE ${asan_compile_flag} ${tsan_compile_flag})
    SET_TARGET_PROPERTIES(vision_8mp_dual_logging2_protobuf PROPERTIES COMPILE_FLAGS "-DAPP_MODE_FRONT_8MP -DENABLE_DUAL_LOGGING -DENABLE_VISION_LOGGING -DENABLE_PROTOBUF_MSG")
    if (AUTOBRAIN_LINKER_SCRIPT)
      target_link_options(vision_8mp_dual_logging2_protobuf PRIVATE "-T${AUTOBRAIN_LINKER_SCRIPT}")
    endif()
    target_link_libraries(vision_8mp_dual_logging2_protobuf ${asan_lib} ${tsan_lib} vision_wrapper2 ${HAL_PROCESS_LIB}
                          vehicle_state_parser zf_vehicle_state_parser camera_wrapper can_wrapper
                          dbc_vehicle_gateway utils network_tx_wrapper
                          ${protobuf_wrapper} ${ecu_key_manager_wrapper} functional_safety)
  endif()

  # Dual logging Single Camera Vision App
  add_executable(vision_2mp_dual_logging2 src/vision_demo2.cpp src/system_mode.cpp)
  target_compile_options(vision_2mp_dual_logging2 PRIVATE ${asan_compile_flag} ${tsan_compile_flag})
  SET_TARGET_PROPERTIES(vision_2mp_dual_logging2 PROPERTIES COMPILE_FLAGS "-DAPP_MODE_FRONT_2MP -DENABLE_DUAL_LOGGING -DENABLE_VISION_LOGGING")
  if (AUTOBRAIN_LINKER_SCRIPT)
    target_link_options(vision_2mp_dual_logging2 PRIVATE "-T${AUTOBRAIN_LINKER_SCRIPT}")
  endif()
  target_link_libraries(vision_2mp_dual_logging2 ${asan_lib} ${tsan_lib} vision_wrapper2 ${HAL_PROCESS_LIB}
                        vehicle_state_parser zf_vehicle_state_parser camera_wrapper can_wrapper
                        dbc_vehicle_gateway utils network_tx_wrapper
                        ${protobuf_wrapper} ${ecu_key_manager_wrapper} functional_safety ${zmq_wrapper_lib} ${phantom_gst_lib} ${GSTREAMER_LIBRARIES})

  if (${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN" OR ${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_ECO" OR ${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_VH" )
    add_executable(vision_2mp_dual_logging2_protobuf src/vision_demo2.cpp src/system_mode.cpp)
    target_compile_options(vision_2mp_dual_logging2_protobuf PRIVATE ${asan_compile_flag} ${tsan_compile_flag})
    SET_TARGET_PROPERTIES(vision_2mp_dual_logging2_protobuf PROPERTIES COMPILE_FLAGS "-DAPP_MODE_FRONT_2MP -DENABLE_DUAL_LOGGING -DENABLE_VISION_LOGGING -DENABLE_PROTOBUF_MSG")
    if (AUTOBRAIN_LINKER_SCRIPT)
      target_link_options(vision_2mp_dual_logging2_protobuf PRIVATE "-T${AUTOBRAIN_LINKER_SCRIPT}")
    endif()
    target_link_libraries(vision_2mp_dual_logging2_protobuf ${asan_lib} ${tsan_lib} vision_wrapper2 ${HAL_PROCESS_LIB}
                          vehicle_state_parser zf_vehicle_state_parser camera_wrapper can_wrapper
                          dbc_vehicle_gateway utils network_tx_wrapper
                          ${protobuf_wrapper} ${ecu_key_manager_wrapper} functional_safety)
  endif()

  add_executable(vision_2mp_simulation src/vision_network_app2.cpp)
  SET_TARGET_PROPERTIES(vision_2mp_simulation PROPERTIES COMPILE_FLAGS "-DAPP_MODE_FRONT_2MP")
  target_link_libraries(vision_2mp_simulation vision_wrapper2 ${HAL_PROCESS_LIB}
                        data_feeder_wrapper network_rx_wrapper network_tx_wrapper utils)

  add_executable(vision_2mp_2crops_simulation src/vision_network_app2.cpp)
  SET_TARGET_PROPERTIES(vision_2mp_2crops_simulation PROPERTIES COMPILE_FLAGS "-DAPP_MODE_FRONT_2MP -DAPP_MODE_FRONT_2MP_2CROPS")
  target_link_libraries(vision_2mp_2crops_simulation vision_wrapper2 ${HAL_PROCESS_LIB}
                        data_feeder_wrapper network_rx_wrapper network_tx_wrapper utils)

  add_executable(vision_8mp_simulation src/vision_network_app2.cpp)
  SET_TARGET_PROPERTIES(vision_8mp_simulation PROPERTIES COMPILE_FLAGS "-DAPP_MODE_FRONT_8MP")
  if (AUTOBRAIN_LINKER_SCRIPT)
    target_link_options(vision_8mp_simulation PRIVATE "-T${AUTOBRAIN_LINKER_SCRIPT}")
  endif()
  target_link_libraries(vision_8mp_simulation vision_wrapper2 ${HAL_PROCESS_LIB}
                        data_feeder_wrapper network_rx_wrapper network_tx_wrapper utils)

  add_executable(vision_8mp_crop_simulation src/vision_network_app2.cpp)
  SET_TARGET_PROPERTIES(vision_8mp_crop_simulation PROPERTIES COMPILE_FLAGS "-DAPP_MODE_FRONT_8MP -DAPP_MODE_FRONT_8MP_CROP")
  if (AUTOBRAIN_LINKER_SCRIPT)
    target_link_options(vision_8mp_crop_simulation PRIVATE "-T${AUTOBRAIN_LINKER_SCRIPT}")
  endif()
  target_link_libraries(vision_8mp_crop_simulation vision_wrapper2 ${HAL_PROCESS_LIB}
                        data_feeder_wrapper network_rx_wrapper network_tx_wrapper utils)

  add_executable(vision_8mp_middle_crop_simulation src/vision_network_app2.cpp)
  SET_TARGET_PROPERTIES(vision_8mp_middle_crop_simulation PROPERTIES COMPILE_FLAGS "-DAPP_MODE_FRONT_8MP -DAPP_MODE_FRONT_8MP_MIDDLE_CROP")
  if (AUTOBRAIN_LINKER_SCRIPT)
    target_link_options(vision_8mp_middle_crop_simulation PRIVATE "-T${AUTOBRAIN_LINKER_SCRIPT}")
  endif()
  target_link_libraries(vision_8mp_middle_crop_simulation vision_wrapper2 ${HAL_PROCESS_LIB}
                        data_feeder_wrapper network_rx_wrapper network_tx_wrapper utils)

  add_executable(vision_8mp_whole_crop_simulation src/vision_network_app2.cpp)
  SET_TARGET_PROPERTIES(vision_8mp_whole_crop_simulation PROPERTIES COMPILE_FLAGS "-DAPP_MODE_FRONT_8MP -DAPP_MODE_FRONT_8MP_WHOLE_CROP")
  if (AUTOBRAIN_LINKER_SCRIPT)
    target_link_options(vision_8mp_whole_crop_simulation PRIVATE "-T${AUTOBRAIN_LINKER_SCRIPT}")
  endif()
  target_link_libraries(vision_8mp_whole_crop_simulation vision_wrapper2 ${HAL_PROCESS_LIB}
                        data_feeder_wrapper network_rx_wrapper network_tx_wrapper utils)

  #add_executable(vision_6cam_simulation src/vision_network_app2.cpp)
  #SET_TARGET_PROPERTIES(vision_6cam_simulation PROPERTIES COMPILE_FLAGS "-DAPP_MODE_6CAM")
  #if (AUTOBRAIN_LINKER_SCRIPT)
  #  target_link_options(vision_6cam_simulation PRIVATE "-T${AUTOBRAIN_LINKER_SCRIPT}")
  #endif()
  #target_link_libraries(vision_6cam_simulation vision_wrapper2 ${HAL_PROCESS_LIB}
  #                      data_feeder_wrapper network_rx_wrapper network_tx_wrapper utils)

  #add_executable(vision_6cam_middle_crop_simulation src/vision_network_app2.cpp)
  #SET_TARGET_PROPERTIES(vision_6cam_middle_crop_simulation PROPERTIES COMPILE_FLAGS "-DAPP_MODE_6CAM -DAPP_MODE_6CAM_MIDDLE_CROP")
  #if (AUTOBRAIN_LINKER_SCRIPT)
  #  target_link_options(vision_6cam_middle_crop_simulation PRIVATE "-T${AUTOBRAIN_LINKER_SCRIPT}")
  #endif()
  #target_link_libraries(vision_6cam_middle_crop_simulation vision_wrapper2 ${HAL_PROCESS_LIB}
  #                      data_feeder_wrapper network_rx_wrapper network_tx_wrapper utils)

  #add_executable(vision_6cam_whole_crop_simulation src/vision_network_app2.cpp)
  #SET_TARGET_PROPERTIES(vision_6cam_whole_crop_simulation PROPERTIES COMPILE_FLAGS "-DAPP_MODE_6CAM -DAPP_MODE_6CAM_WHOLE_CROP")
  #if (AUTOBRAIN_LINKER_SCRIPT)
  #  target_link_options(vision_6cam_whole_crop_simulation PRIVATE "-T${AUTOBRAIN_LINKER_SCRIPT}")
  #endif()
  #target_link_libraries(vision_6cam_whole_crop_simulation vision_wrapper2 ${HAL_PROCESS_LIB}
  #                      data_feeder_wrapper network_rx_wrapper network_tx_wrapper utils)

  #
  # Phantom Static Calibration Executable
  #
  if (${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN" OR ${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_ECO" OR ${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_VH" )
    add_executable(static_calibrator_6v src/static_calibration_app.cpp)
    SET_TARGET_PROPERTIES(static_calibrator_6v PROPERTIES COMPILE_FLAGS "${PROTOBUF} -DENABLE_8MP_WIDE -DENABLE_FULL_SIZE_IMAGE ${DEFINE_ENABLE_PROTOBUF_MSG}" )
    if (AUTOBRAIN_LINKER_SCRIPT)
      target_link_options(static_calibrator_6v PRIVATE "-T${AUTOBRAIN_LINKER_SCRIPT}")
    endif()
    target_link_libraries(static_calibrator_6v calibration_wrapper ${HAL_PROCESS_LIB}
                          camera_wrapper vision_wrapper2 utils network_tx_wrapper ${protobuf_wrapper})
    list(APPEND PHANTOM_LINUX_INSTALL_TARGETS_TDA4X static_calibrator_6v)
  endif()
endif()
endif() #PHANTOM_AI_BUILD_PERCEPTION

#
# Network & CAN executables
#
add_executable(can_rx_test src/can_test.cpp)
SET_TARGET_PROPERTIES(can_rx_test PROPERTIES COMPILE_FLAGS "-DRUN_RX_TEST")
if (AUTOBRAIN_LINKER_SCRIPT)
target_link_options(can_rx_test PRIVATE "-T${AUTOBRAIN_LINKER_SCRIPT}")
endif()
target_link_libraries(can_rx_test can_wrapper
                      utils vehicle_state_parser zf_vehicle_state_parser dbc_vehicle_gateway ${HAL_INTERFACE_LIB})

add_executable(can_tx_test src/can_test.cpp)
SET_TARGET_PROPERTIES(can_tx_test PROPERTIES COMPILE_FLAGS "-DRUN_TX_TEST")
if (AUTOBRAIN_LINKER_SCRIPT)
target_link_options(can_tx_test PRIVATE "-T${AUTOBRAIN_LINKER_SCRIPT}")
endif()
target_link_libraries(can_tx_test can_wrapper
                      utils vehicle_state_parser zf_vehicle_state_parser dbc_vehicle_gateway ${HAL_INTERFACE_LIB})

if(NOT ${PHANTOM_AI_TI_SOC} STREQUAL "AM62A")
  add_executable(network_test src/network_test.cpp)
  if (AUTOBRAIN_LINKER_SCRIPT)
    target_link_options(network_test PRIVATE "-T${AUTOBRAIN_LINKER_SCRIPT}")
  endif()
  target_link_libraries(network_test network_rx_wrapper ${HAL_INTERFACE_LIB} utils)
endif()

#
# Camera executables
#
add_executable(camera_stream src/camera_stream.cpp)
if (AUTOBRAIN_LINKER_SCRIPT)
  target_link_options(camera_stream PRIVATE "-T${AUTOBRAIN_LINKER_SCRIPT}")
endif()
target_link_libraries(camera_stream network_tx_wrapper camera_wrapper can_wrapper vehicle_state_parser zf_vehicle_state_parser dbc_vehicle_gateway ${HAL_INTERFACE_LIB} utils)

# 8 Megapixel cameras (OnSemi AR0820 and OmniVision OX08B40) front camera
add_executable(camera_8mp_stream src/camera_stream.cpp)
SET_TARGET_PROPERTIES(camera_8mp_stream PROPERTIES COMPILE_FLAGS "-DENABLE_8MP_WIDE ${DEFINE_ENABLE_PROTOBUF_MSG}")
if (AUTOBRAIN_LINKER_SCRIPT)
  target_link_options(camera_8mp_stream PRIVATE "-T${AUTOBRAIN_LINKER_SCRIPT}")
endif()
target_link_libraries(camera_8mp_stream network_tx_wrapper camera_wrapper can_wrapper vehicle_state_parser zf_vehicle_state_parser dbc_vehicle_gateway ${HAL_INTERFACE_LIB} utils ${protobuf_wrapper})

add_executable(camera_fullres_stream src/camera_stream.cpp)
SET_TARGET_PROPERTIES(camera_fullres_stream PROPERTIES COMPILE_FLAGS "-DENABLE_FULL_SIZE_IMAGE")
if (AUTOBRAIN_LINKER_SCRIPT)
  target_link_options(camera_fullres_stream PRIVATE "-T${AUTOBRAIN_LINKER_SCRIPT}")
endif()
target_link_libraries(camera_fullres_stream network_tx_wrapper camera_wrapper can_wrapper vehicle_state_parser zf_vehicle_state_parser dbc_vehicle_gateway ${HAL_INTERFACE_LIB} utils)

add_executable(camera_fullres_extra_stream src/camera_stream.cpp)
SET_TARGET_PROPERTIES(camera_fullres_extra_stream PROPERTIES COMPILE_FLAGS "-DENABLE_FULL_SIZE_IMAGE -DENABLE_EXTRA_STREAM")
target_link_libraries(camera_fullres_extra_stream network_tx_wrapper camera_wrapper can_wrapper vehicle_state_parser zf_vehicle_state_parser dbc_vehicle_gateway ${HAL_INTERFACE_LIB} utils)

add_executable(camera_fullres_5fps_stream src/camera_stream.cpp)
SET_TARGET_PROPERTIES(camera_fullres_5fps_stream PROPERTIES COMPILE_FLAGS "-DENABLE_FULL_SIZE_IMAGE -DENABLE_FULL_SIZE_5FPS_IMAGE")
target_link_libraries(camera_fullres_5fps_stream network_tx_wrapper camera_wrapper can_wrapper vehicle_state_parser zf_vehicle_state_parser dbc_vehicle_gateway ${HAL_INTERFACE_LIB} utils)

add_executable(cpu_utilization src/cpu_utilization.cpp)
if (AUTOBRAIN_LINKER_SCRIPT)
target_link_options(cpu_utilization PRIVATE "-T${AUTOBRAIN_LINKER_SCRIPT}")
endif()
target_link_libraries(cpu_utilization ${HAL_INTERFACE_LIB})

# Unfortunately there is no automagic CMake variable that tracks all the executables for a project.  We need a list
# like this for the `test_launch_files`, so we will create it here and reuse it to specify the install targets.
if (PHANTOM_AI_BUILD_PERCEPTION)
  if(${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "TDA4X" OR ${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "AM62A")
    if(DEFINED BUILD_TI_IMAGE_KERNEL)
      add_definitions(-DTI_DSP_OVX_ENABLED)
      include_directories(${TI_PHANTOM_IMAGE_KERNEL_INCLUDE_DIR})
    endif()
    list(APPEND PHANTOM_LINUX_INSTALL_TARGETS_TDA4X
      vision_2mp_simulation
      vision_2mp_2crops_simulation
      vision_2mp_dual_logging2
      vision_8mp_dual_logging2
      vision_8mp_simulation
      vision_8mp_crop_simulation
      vision_8mp_middle_crop_simulation
      vision_8mp_whole_crop_simulation
      #vision_6cam_simulation
      #vision_6cam_middle_crop_simulation
      #vision_6cam_whole_crop_simulation
    )
  endif()
endif()

if (${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN" OR ${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_ECO" OR ${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_VH" )
  list(APPEND PHANTOM_LINUX_INSTALL_TARGETS_TDA4X
    vision_2mp_dual_logging2_protobuf
    vision_8mp_dual_logging2_protobuf
  )
endif()

set(PHANTOM_LINUX_INSTALL_TARGETS
  can_rx_test
  can_tx_test
  camera_stream
  camera_fullres_stream
  camera_fullres_5fps_stream
  camera_fullres_extra_stream
  camera_8mp_stream
  print_stats
  cpu_utilization
  resource_monitor
  ${PHANTOM_LINUX_INSTALL_TARGETS_TDA4X}
  ${DEMO_APPLICATION}
)

if(NOT ${PHANTOM_AI_TI_SOC} STREQUAL "AM62A")
  set(PHANTOM_LINUX_INSTALL_TARGETS "${PHANTOM_LINUX_INSTALL_TARGETS}"
      network_test)
endif()

#message_info("PHANTOM_LINUX_EXECUTABLES = ${PHANTOM_LINUX_EXECUTABLES}")

## Mark executables and/or libraries for installation.
install(
  TARGETS ${PHANTOM_LINUX_INSTALL_TARGETS}
  RUNTIME DESTINATION ${PHANTOM_OS_INSTALL_BIN_DIR}
  LIBRARY DESTINATION ${PHANTOM_OS_INSTALL_LIB_DIR}
)
