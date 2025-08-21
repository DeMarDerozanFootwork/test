cmake_minimum_required(VERSION 2.8.3)

project(wrappers)

find_package(OpenCV REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PHANTOM_AI_CAN_INCLUDE_DIR}
  ${PHANTOM_AI_CAN_MSG_PACK_INCLUDE_DIR}
  ${PHANTOM_AI_COMMON_INCLUDE_DIR}
  ${PHANTOM_AI_CORE_INCLUDE_DIR}
  ${PHANTOM_AI_DBC_INCLUDE_DIR}
  ${PHANTOM_AI_PHANTOM_VISION_INCLUDE_DIR}
  ${PHANTOM_AI_PHANTOM_VISION2_INCLUDE_DIR}
  ${PHANTOM_AI_PHANTOM_CALIBRATION_INCLUDE_DIR}
  ${PHANTOM_AI_TOOLS_INCLUDE_DIR}
  ${PHANTOM_AI_UTILS_INCLUDE_DIR}
  ${embedded_network_INCLUDE_DIR}
  ${PHANTOM_AI_FUNCTIONAL_SAFETY_INCLUDE_DIR}
  ${EIGEN_INCLUDE_DIR}
  ${PHANTOM_AI_TEST_INCLUDE_DIR}
  ${PHANTOM_AI_VEHICLE_STATE_PARSER_INCLUDE_DIR}
  ${PHANTOM_AI_ZF_VEHICLE_STATE_PARSER_INCLUDE_DIR}
  ${ZEROMQ_INCLUDE_DIR}
  ${PHANTOM_GST_INCLUDE_DIR}
)

if(${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "TDA4X" OR ${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "AM62A")
    include_directories(${HAL_INCLUDE_DIR} ${TI_CORE_INCLUDE_DIR})
    set(HAL_INTERFACE_LIB "hal_interface_lib")
    set(HAL_PROCESS_LIB "hal_process_lib")
endif()

if(PHANTOM_AI_BUILD_ASAN)
  message("AddressSanitizer is enabled for all wrapper")
  set(asan_compile_flag "-fsanitize=address" "-fsanitize-recover=address")
  set(asan_lib "asan")
else ()
  set(asan_compile_flag "")
  set(asan_lib "")
endif ()
if(PHANTOM_AI_BUILD_TSAN)
  message("ThreadSanitizer is enabled for wrapper.")
  set(tsan_compile_flag "-fsanitize=thread")
  set(tsan_lib "tsan")
else ()
  set(tsan_compile_flag "")
  set(tsan_lib "")
endif()

if(ENABLE_ZEROMQ)
  set(ZMQ_WRAPPER "zmq_wrapper")
  set(zmq_lib "zmq")
  add_library(zmq STATIC IMPORTED)
  set_target_properties(zmq PROPERTIES
    IMPORTED_LOCATION ${ZMQ_STATIC_LIB_PATH})
else ()
  set(ZMQ_WRAPPER "")
  set(zmq_lib "")
endif ()

message("========== src/phantom_ai/wrappers/CMakeLists.txt ENABLE_GSTREAMER=${ENABLE_GSTREAMER}") #WOOSEOK
if(${PHANTOM_AI_TI_SOC} STREQUAL "J721E")
  set(phantom_gst_lib "phantom_gst")
else()
  set(phantom_gst_lib "")
endif()

if (SW_RELEASE_TARGET STREQUAL "EXTERNAL")
  add_definitions(-DCUSTOMER_RELEASE_MODE)
endif()

add_library(can_wrapper src/can_wrapper.cpp)
target_compile_options(can_wrapper PRIVATE ${asan_compile_flag} ${tsan_compile_flag})
target_link_libraries(can_wrapper ${asan_lib} ${tsan_lib} core can systest)

add_library(data_feeder_wrapper src/data_feeder_wrapper.cpp)
target_compile_options(data_feeder_wrapper PRIVATE ${asan_compile_flag} ${tsan_compile_flag})
target_link_libraries(data_feeder_wrapper ${asan_lib} ${tsan_lib} core ${OpenCV_LIBRARIES})

add_library(network_rx_wrapper src/network_rx_wrapper.cpp)
target_compile_options(network_rx_wrapper PRIVATE ${asan_compile_flag} ${tsan_compile_flag})
target_link_libraries(network_rx_wrapper ${asan_lib} ${tsan_lib} core embedded_network ${OpenCV_LIBRARIES})

add_library(network_tx_wrapper src/network_tx_wrapper.cpp)
target_compile_options(network_tx_wrapper PRIVATE ${asan_compile_flag} ${tsan_compile_flag})
target_link_libraries(network_tx_wrapper ${asan_lib} ${tsan_lib} core common utils embedded_network image_conversion_helper ${OpenCV_LIBRARIES} ${phantom_gst_lib})

add_library(camera_wrapper src/camera_wrapper.cpp)
target_compile_options(camera_wrapper PRIVATE ${asan_compile_flag} ${tsan_compile_flag})
target_link_libraries(camera_wrapper ${asan_lib} ${tsan_lib} core systest ${HAL_INTERFACE_LIB} common ${OpenCV_LIBRARIES})

if (TARGET tda4x_ticamdrv_interface_lib)
  add_library(camera_wrapper_ticamdrv src/camera_wrapper.cpp)
  target_compile_options(camera_wrapper_ticamdrv PRIVATE ${asan_compile_flag} ${tsan_compile_flag})
  target_link_libraries(camera_wrapper_ticamdrv ${asan_lib} ${tsan_lib} core tda4x_ticamdrv_interface_lib common ${OpenCV_LIBRARIES})
endif()

add_library(ecu_key_manager_wrapper src/ecu_key_manager_wrapper.cpp)
target_compile_options(ecu_key_manager_wrapper PRIVATE ${asan_compile_flag} ${tsan_compile_flag})
target_link_libraries(ecu_key_manager_wrapper ${asan_lib} ${tsan_lib} utils core ${HAL_PROCESS_LIB})

if(ENABLE_ZEROMQ)
    add_library(zmq_wrapper src/zmq_wrapper.cpp)
    target_compile_options(zmq_wrapper PRIVATE ${asan_compile_flag} ${tsan_compile_flag})
    target_link_libraries(zmq_wrapper ${asan_lib} ${tsan_lib} core can vehicle_state_parser zf_vehicle_state_parser ${zmq_lib})
endif()

set(PHANTOM_AI_WRAPPERS
  can_wrapper
  data_feeder_wrapper
  network_rx_wrapper
  network_tx_wrapper
  camera_wrapper
  ecu_key_manager_wrapper
  ${ZMQ_WRAPPER}
)

if (PHANTOM_AI_BUILD_PERCEPTION)
  add_library(vision_wrapper2 src/vision_wrapper2.cpp)
  target_compile_options(vision_wrapper2 PRIVATE ${asan_compile_flag} ${tsan_compile_flag})
  target_link_libraries(vision_wrapper2 ${asan_lib} ${tsan_lib} core systest ${PHANTOM_VISION2_LIB_NAME} vision_can_message_pack ${HAL_INTERFACE_LIB} ${OpenCV_LIBRARIES})
  if(${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "TDA4X" OR ${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "AM62A")
    if(DEFINED BUILD_TI_IMAGE_KERNEL)
      add_definitions(-DTI_DSP_OVX_ENABLED)
      include_directories(${TI_PHANTOM_IMAGE_KERNEL_INCLUDE_DIR})
    endif()
    list(APPEND PHANTOM_AI_WRAPPERS
      vision_wrapper2
    )
  endif()
endif()

# Add protobuf wrappers for Autobrain
if (${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN" OR ${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_ECO" OR ${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_VH" )
  include_directories(${PHANTOM_AI_PROTOBUF_MSGS_INCLUDE_DIR} ${PHANTOM_AI_PROTOBUF_MSGS_MSGS_INCLUDE_DIR} ${Protobuf_INCLUDE_DIRS})

  add_library(protobuf_wrapper src/protobuf_wrapper.cpp)
  target_compile_options(protobuf_wrapper PRIVATE ${asan_compile_flag} ${tsan_compile_flag})
  target_link_libraries(protobuf_wrapper ${asan_lib} ${tsan_lib} utils core ${HAL_INTERFACE_LIB} protobuf_msgs)

  #debug version so customer can check CAN signals are received on application side
  add_library(protobuf_wrapper_debug src/protobuf_wrapper.cpp)
  target_compile_options(protobuf_wrapper_debug PRIVATE ${asan_compile_flag} ${tsan_compile_flag})
  SET_TARGET_PROPERTIES(protobuf_wrapper_debug PROPERTIES COMPILE_FLAGS "-DVEHICLE_STATE_DEBUG_DUMP")
  target_link_libraries(protobuf_wrapper_debug ${asan_lib} ${tsan_lib} utils core ${HAL_INTERFACE_LIB} protobuf_msgs)

  add_library(calibration_wrapper src/calibration_wrapper.cpp)
  target_compile_options(calibration_wrapper PRIVATE ${asan_compile_flag} ${tsan_compile_flag})
  target_link_libraries(calibration_wrapper ${asan_lib} ${tsan_lib} core ${PHANTOM_VISION2_LIB_NAME} ${PHANTOM_STATIC_CALIBRATION_LIB_NAME} ${OpenCV_LIBRARIES})
  list(APPEND PHANTOM_AI_WRAPPERS calibration_wrapper)

  set(PHANTOM_AI_WRAPPERS
    ${PHANTOM_AI_WRAPPERS}
    protobuf_wrapper
    protobuf_wrapper_debug
  )

  if (TARGET tda4x_ticamdrv_interface_lib)
    set(PHANTOM_AI_WRAPPERS
      ${PHANTOM_AI_WRAPPERS}
      camera_wrapper_ticamdrv
    )
  endif ()
endif ()

set(${PROJECT_NAME}_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)

install(
  TARGETS ${PHANTOM_AI_WRAPPERS}
  RUNTIME DESTINATION ${PHANTOM_OS_INSTALL_BIN_DIR}
  LIBRARY DESTINATION ${PHANTOM_OS_INSTALL_LIB_DIR}
)

set(PHANTOM_AI_WRAPPERS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE INTERNAL "" FORCE)
