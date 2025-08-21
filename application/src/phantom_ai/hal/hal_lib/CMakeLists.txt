phantom_project(hal_process_lib)

find_package(OpenCV REQUIRED)
find_package(Boost 1.58.0)

###########################################################################
## include directories
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}

  ${PHANTOM_AI_COMMON_INCLUDE_DIR}
  ${PHANTOM_AI_CORE_INCLUDE_DIR}
  ${PHANTOM_AI_TASK_ENGINE_INCLUDE_DIR}
  ${PHANTOM_AI_PHANTOM_VISION2_INCLUDE_DIR}
  ${PHANTOM_AI_TEST_INCLUDE_DIR}
  ${EIGEN_INCLUDE_DIR}
)

set(HAL_INCLUDE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/../
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    CACHE INTERNAL "" FORCE)
include_directories(${HAL_INCLUDE_DIR} ${TI_CORE_INCLUDE_DIR} ${PHANTOM_AI_TASK_ENGINE_INCLUDE_DIR} ${PHANTOM_AI_PHANTOM_VISION2_INCLUDE_DIR})

##########################################################
# create Hal C++ library
add_library(hal_interface_lib
  src/hal_display.cpp
  src/hal_base_hw.cpp
  src/hal_stats.cpp
)

if (${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "TDA4X")
  target_sources(hal_interface_lib PRIVATE
    src/tda4x_camera.cpp)
elseif (${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "AM62A")
  target_sources(hal_interface_lib PRIVATE
    src/hal_camera.cpp)
endif()

if(PHANTOM_AI_BUILD_ASAN)
  message("AddressSanitizer is enabled for hal_interface_lib.")
  target_compile_options(hal_interface_lib PRIVATE -fsanitize=address -fsanitize-recover=address)
  target_link_libraries(hal_interface_lib asan systest)
endif()

if(PHANTOM_AI_BUILD_TSAN)
  message("ThreadSanitizer is enabled for hal_interface_lib.")
  target_compile_options(hal_interface_lib PRIVATE -fsanitize=thread)
  target_link_options(hal_interface_lib PRIVATE -fsanitize=thread)
endif()

target_link_libraries(hal_interface_lib core systest ${OpenCV_LIBRARIES} ti_core)
if (${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_VH")
  message("Using default HIK Camera API.")
  target_compile_definitions(hal_interface_lib PRIVATE USE_HIK_CAM_API)
else()
  message("Using TI camera API.")
  target_compile_definitions(hal_interface_lib PRIVATE USE_TI_CAM_API)
endif()

if(TARGET_BOARD_EVM_AL_86)
  add_definitions(-DTARGET_BOARD_EVM_AL_86)
endif()

add_library(hal_process_lib
  src/intercam_nms.cpp
  src/tidl_inferencer.cpp
  src/tidl_kernels.cpp
  src/tidl_kernels_neon.cpp
  src/tidl_preproc_neon.cpp
  src/tidl_decode_neon.cpp
  src/mmio.cpp
  src/hal_ecu_key_manager.cpp)

target_link_libraries(hal_process_lib core systest ${OpenCV_LIBRARIES} ti_core hal_interface_lib task_engine ${PHANTOM_VISION2_LIB_NAME})

## Mark executables and/or libraries for installation.
install(
  TARGETS hal_interface_lib
  RUNTIME DESTINATION ${PHANTOM_OS_INSTALL_BIN_DIR}
  LIBRARY DESTINATION ${PHANTOM_OS_INSTALL_LIB_DIR}
)

if (TARGET hal_ticamdrv_interface_lib)
  install(
    TARGETS hal_ticamdrv_interface_lib
    RUNTIME DESTINATION ${PHANTOM_OS_INSTALL_BIN_DIR}
    LIBRARY DESTINATION ${PHANTOM_OS_INSTALL_LIB_DIR}
  )
endif()

install(
  TARGETS hal_process_lib
  RUNTIME DESTINATION ${PHANTOM_OS_INSTALL_BIN_DIR}
  LIBRARY DESTINATION ${PHANTOM_OS_INSTALL_LIB_DIR}
)

if(PHANTOM_OS_TEST)
   add_subdirectory(test)
endif(PHANTOM_OS_TEST)
