##########################################################
## Header for CMakeLists
cmake_minimum_required(VERSION 3.0.0)
phantom_project(phantom_ai)
##########################################################

# Generate a compile_commands.json file to support IDEs like lsp-mode.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

find_package(Boost REQUIRED COMPONENTS filesystem iostreams python system thread program_options regex)
include_directories(${Boost_INCLUDE_DIRS})

############ library type shared or static?
if (PHANTOM_AI_BUILD_SHARED)
    set(PHANTOM_AI_BUILD_LIBRARY_TYPE SHARED)
else(PHANTOM_AI_BUILD_SHARED)
    set(PHANTOM_AI_BUILD_LIBRARY_TYPE)
endif(PHANTOM_AI_BUILD_SHARED)


if (${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "AGX" OR ${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "AGX2" OR ${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "PX2")
  list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/3rd_party/nvmedia/cmake)
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} PARENT_SCOPE)
endif()

if (SW_RELEASE_TARGET STREQUAL "EXTERNAL")
  add_definitions(-DDISABLE_RESOURCE_STATUS_DISPLAY)
endif()

###############################################################################
## for 3rd party
###############################################################################

## Find 3rd party packages
if (${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "AGX" OR ${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "AGX2" OR ${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "PX2")
  find_package(NvMedia MODULE REQUIRED)
endif()


# Remove circular dependency with Hal by defining the
# phantom_vision2 include path here.
set(PHANTOM_AI_PHANTOM_VISION2_INCLUDE_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/perception/phantom_vision2/include CACHE
  INTERNAL "" FORCE)

set(PHANTOM_AI_PHANTOM_CALIBRATION_INCLUDE_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/perception/phantom_static_calibration/include CACHE
  INTERNAL "" FORCE)

##########################################################
## building each modules
## Note: these are ordered by depedency

if (${CUSTOMER_SPECIFIC_APP} STREQUAL "UNDEFINED")
  add_subdirectory(core)
  add_subdirectory(geometry)
  add_subdirectory(common)
  add_subdirectory(tools)
  add_subdirectory(display)
  add_subdirectory(localization)
  add_subdirectory(supervisor)
  add_subdirectory(test)
  add_subdirectory(sensors)
  add_subdirectory(mapper)
  add_subdirectory(map_server)
  add_subdirectory(control)
  add_subdirectory(sensor_fusion)
  add_subdirectory(task_engine)
  add_subdirectory(hmi)
elseif (${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN" OR ${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_ECO" OR ${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_VH")
  add_subdirectory(core)
  add_subdirectory(geometry)
  add_subdirectory(common)
  add_subdirectory(tools)
  add_subdirectory(display)
  add_subdirectory(supervisor)
  add_subdirectory(test)
  add_subdirectory(sensors)
  add_subdirectory(task_engine)
  add_subdirectory(system)
endif()

if (${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "V3H" OR ${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "TDA4X" OR ${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "AM62A")
  add_subdirectory(hal)
  add_subdirectory(utils)
  add_subdirectory(protobuf_msgs)
  add_subdirectory(wrappers)
  add_subdirectory(functional_safety)
endif()


if (PHANTOM_AI_BUILD_PERCEPTION)
  add_subdirectory(perception)
endif()


###############################################################################

############ Artificial target to make some IDEs (such as QtCreator) work better with
FILE(GLOB_RECURSE INCLUDE_FILES "*.h" "*.hpp" "*.yaml" "*.json")
add_custom_target(${PROJECT_NAME}_headers SOURCES ${INCLUDE_FILES})

if(PHANTOM_AI_BUILD_ENCRYPT_YAML)
  ## Copy build/params directory.
  install(DIRECTORY ${CMAKE_BINARY_DIR}/params/
    DESTINATION ${PHANTOM_OS_INSTALL_SHARE_DIR}/params
    PATTERN "phantom_vision2" EXCLUDE
    PATTERN "phantom_static_calibration" EXCLUDE
    PATTERN "param_test" EXCLUDE
    PATTERN "encrypt.sh" EXCLUDE
    PATTERN "hmi" EXCLUDE
    PATTERN "localization" EXCLUDE
    PATTERN "map_server" EXCLUDE
    PATTERN "node_manager" EXCLUDE
    PATTERN "planning" EXCLUDE
    PATTERN "sensors" EXCLUDE
    PATTERN "sensor_fusion" EXCLUDE
    PATTERN "phantom_ros" EXCLUDE
  )

  if (NOT (${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "V3H" OR ${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "TDA4X" OR ${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "AM62A"))
    install(DIRECTORY ${CMAKE_BINARY_DIR}/params/phantom_ros/
      DESTINATION ${PHANTOM_OS_INSTALL_SHARE_DIR}/params/phantom_ros
  )
  endif()

  install(DIRECTORY ${CMAKE_BINARY_DIR}/params/sensors/camera/
    DESTINATION ${PHANTOM_OS_INSTALL_SHARE_DIR}/params/sensors/camera/
  )

  if (PHANTOM_AI_BUILD_PERCEPTION)
    install(DIRECTORY ${CMAKE_BINARY_DIR}/params/perception/phantom_vision2/
        DESTINATION ${PHANTOM_OS_INSTALL_SHARE_DIR}/params/perception/${PHANTOM_VISION2_LIB_NAME}/
        PATTERN "user" EXCLUDE
        PATTERN "export" EXCLUDE
      )
    ## Copy Perception ICON files
    install(DIRECTORY perception/params/perception/phantom_vision2/icons/
      DESTINATION ${PHANTOM_OS_INSTALL_SHARE_DIR}/params/perception/${PHANTOM_VISION2_LIB_NAME}/icons
      FILES_MATCHING
      PATTERN "*.png"
      PATTERN "*.json"
    )
    ## Create empty Perception dirs (for user copied files)
    install(DIRECTORY DESTINATION ${PHANTOM_OS_INSTALL_SHARE_DIR}/params/perception/${PHANTOM_VISION2_LIB_NAME}/pie_models)
    install(DIRECTORY DESTINATION ${PHANTOM_OS_INSTALL_SHARE_DIR}/params/perception/${PHANTOM_VISION2_LIB_NAME}/trt_models)
    ## Copy Camera .CONF files
    install(DIRECTORY params/sensors/camera/
      DESTINATION ${PHANTOM_OS_INSTALL_SHARE_DIR}/params/sensors/camera
      FILES_MATCHING
      PATTERN "*.conf"
    )
    ## Copy model files
    install(DIRECTORY perception/params/perception/phantom_vision2/tda4x/models/
      DESTINATION ${PHANTOM_OS_INSTALL_SHARE_DIR}/params/perception/${PHANTOM_VISION2_LIB_NAME}/tda4x/models
      FILES_MATCHING PATTERN "*.bin"
    )

    install(DIRECTORY ${CMAKE_BINARY_DIR}/params/perception/phantom_static_calibration/
        DESTINATION ${PHANTOM_OS_INSTALL_SHARE_DIR}/params/perception/${PHANTOM_STATIC_CALIBRATION_LIB_NAME}/
      )
  endif()
else()
  ## Copy params directory.
  install(DIRECTORY params/
    DESTINATION ${PHANTOM_OS_INSTALL_SHARE_DIR}/params
    PATTERN "param_test" EXCLUDE
    PATTERN "hmi" EXCLUDE
    PATTERN "localization" EXCLUDE
    PATTERN "map_server" EXCLUDE
    PATTERN "node_manager" EXCLUDE
    PATTERN "planning" EXCLUDE
    PATTERN "sensors" EXCLUDE
    PATTERN "sensor_fusion" EXCLUDE
  )

  install(DIRECTORY params/sensors/camera/
    DESTINATION ${PHANTOM_OS_INSTALL_SHARE_DIR}/params/sensors/camera/
  )

  if (PHANTOM_AI_BUILD_PERCEPTION)
    ## Copy perception params directory to params dir.
    install(DIRECTORY perception/params/perception/phantom_vision2/
      DESTINATION ${PHANTOM_OS_INSTALL_SHARE_DIR}/params/perception/${PHANTOM_VISION2_LIB_NAME}/
      PATTERN "user" EXCLUDE
      PATTERN "export" EXCLUDE
      PATTERN "encrypt.sh" EXCLUDE
    )

    install(DIRECTORY perception/params/perception/phantom_static_calibration/
      DESTINATION ${PHANTOM_OS_INSTALL_SHARE_DIR}/params/perception/${PHANTOM_STATIC_CALIBRATION_LIB_NAME}/
    )
  endif()
endif()

## Copy assets
if (PHANTOM_AI_INSTALL_ASSESTS)
install(DIRECTORY assets/
  DESTINATION ${PHANTOM_OS_INSTALL_SHARE_DIR}/assets
)
endif()

## Copy dataset
if (PHANTOM_AI_INSTALL_DATASET)
install(DIRECTORY dataset/
  DESTINATION ${PHANTOM_OS_INSTALL_SHARE_DIR}/dataset
)
endif()

## Copy schema
install(DIRECTORY schema/
  DESTINATION ${PHANTOM_OS_INSTALL_SHARE_DIR}/schema
)
