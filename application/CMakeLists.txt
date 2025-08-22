cmake_minimum_required(VERSION 2.8.3)

# Silence cmake 3.0+ warning
if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

project(phantom_os)

# Fancy color for stdout messages
if(NOT WIN32)
	string(ASCII 27 Esc)
	set(ColourReset "${Esc}[m")
	set(ColourBold  "${Esc}[1m")
	set(Red         "${Esc}[31m")
	set(Green       "${Esc}[32m")
	set(Yellow      "${Esc}[33m")
	set(Blue        "${Esc}[34m")
	set(Magenta     "${Esc}[35m")
	set(Cyan        "${Esc}[36m")
	set(White       "${Esc}[37m")
	set(BoldRed     "${Esc}[1;31m")
	set(BoldGreen   "${Esc}[1;32m")
	set(BoldYellow  "${Esc}[1;33m")
	set(BoldBlue    "${Esc}[1;34m")
	set(BoldMagenta "${Esc}[1;35m")
	set(BoldCyan    "${Esc}[1;36m")
	set(BoldWhite   "${Esc}[1;37m")
endif()

macro(message_build message)
	message("-- ${message}")
endmacro()

macro(message_debug message)
	message(${Cyan}${message}${ColourReset})
endmacro()

macro(message_info message)
	message(${BoldBlue}${message}${ColourReset})
endmacro()

macro(message_warn message)
	message(${BoldYellow}${message}${ColourReset})
endmacro()

macro(message_error message)
	message(${BoldRed}${message}${ColourReset})
endmacro()

option(ENABLE_HIDING "Enable/Disable hiding 'phantom' from package." OFF)

macro(phantom_project project_name)
	project(${project_name})
	message_debug("Adding project ${project_name}.")

	# Nest the generated libs and binaries into their own project directories
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${project_name})
	set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${project_name})
endmacro()

# Option to enable Phantom OS unit tests (excluding Eigen tests).
option(PHANTOM_OS_TEST "Enable/Disable Phantom OS tests." OFF)
if(PHANTOM_OS_TEST)
	message_info("Enabling Phantom OS tests.")
	enable_testing()
	find_package(GTest REQUIRED)
endif()

option(ENABLE_PRINT_STATS "Enable/Disable output of performance and memory usage stats." OFF)
if(ENABLE_PRINT_STATS)
 message_info("Enabling output of performance and memory usage stats.")
 add_definitions(-DPRINT_PERF_MEM_STATS=1)
else()
 add_definitions(-DPRINT_PERF_MEM_STATS=0)
endif()

option(PHANTOM_OS_PROFILE "Enable/Disable Phantom OS profiling." OFF)
if(PHANTOM_OS_PROFILE)
 message_info("Enabling Phantom OS profiling.")
 add_definitions(-DPHANTOM_AI_ENABLE_PROFILER=1)
else()
 add_definitions(-DPHANTOM_AI_ENABLE_PROFILER=0)
endif()

option(ENABLE_ZEROMQ "Enable/Disable ZeroMQ communication" OFF)
if(ENABLE_ZEROMQ)
 message_info("Enabling ZeroMQ communication")
 add_definitions(-DENABLE_ZEROMQ=1)
else()
 add_definitions(-DENABLE_ZEROMQ=0)
endif()

option(PHANTOM_OS_TEST_COVERAGE "Enable/Disable Phantom OS test coverage." OFF)

# Option to enable Eigen unit tests.
option(ENABLE_EIGEN_TESTS "Enable Eigen tests." OFF)

# Option to enable Eigen unit tests.
option(ENABLE_3RD_PARTY_ROS_TESTS "Enable 3rd party ROS tests." OFF)

# Option to use ZF's Generic CAN DBC.
option(USE_ZF_VEHICLE_STATES "Use ZF's Generic CAN DBC" OFF)

# Option to make all compiler warnings into errors. Jenkins will enable this.
option(STRICT_BUILD "Turn all compiler warnings ito errors." OFF)
if (STRICT_BUILD)
	message_warn("Performing strict build!")
	add_compile_options(-Werror)
  if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 9.0)
    # WA for GCC 9.x, too many places
    message_info(" Disabling deprecated-copy warnings")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-copy")
  endif()
endif()

# Globally set the base path for generated libs and bins.
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Set build target platform if not defined in CACHE.  Valid options are PC, PX2, AGX, V3H, TDA4X, AM62A UNDEFINED (default).
SET(PHANTOM_AI_BUILD_PLATFORM "UNDEFINED" CACHE STRING "Build target platform")

set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)
set(PHANTOM_OS_INSTALL_LIB_DIR lib)
if (ENABLE_HIDING)
  set(PHANTOM_OS_INSTALL_BIN_DIR bin)
  set(PHANTOM_OS_INSTALL_SHARE_DIR share/perception)
  set(PHANTOM_OS_INSTALL_INCLUDE_DIR include/perception)
else()
  if (NOT (${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "V3H" OR ${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "TDA4X" OR ${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "AM62A"))
    set(PHANTOM_OS_INSTALL_BIN_DIR lib/phantom_ros)
    set(PHANTOM_OS_INSTALL_SHARE_DIR share/phantom_ros)
    set(PHANTOM_OS_INSTALL_INCLUDE_DIR include/phantom_ros)
  else()
    set(PHANTOM_OS_INSTALL_BIN_DIR bin)
    set(PHANTOM_OS_INSTALL_SHARE_DIR share/phantom_os)
    set(PHANTOM_OS_INSTALL_INCLUDE_DIR include/phantom_os)
  endif()
endif()
set(PHANTON_OS_INSTALL_PYTHON_DIR lib)

# Set C++17 as a requirement.  Setting 'CMAKE_CXX_STANDARD_REQUIRED' is required as the default behaviour is to decay
# to an earlier standard without issuing a warning.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Turn on all the compiler warnings.  Wswitch-enum adds to Wall and Wextra by complaining if an switch statement using
# a enum does not cover all the possible values even if a default case is defined.
add_compile_options(-fPIC -pthread -Wall -Wextra -Wswitch-enum -Werror=return-type)
add_definitions(-DEIGEN_MPL2_ONLY)

# By default, include the debug symbols with all binaries.  To be used with debugging with core dumps.
option(ENABLE_DEBUG_SYMBOLS "Build with debug symbols" ON)
if (ENABLE_DEBUG_SYMBOLS)
	message_info("Building with debug symbols.")
	add_compile_options(-g)
endif()

# Linker options: Don't link against anything that actually isn't needed.
# There isn't a clean CMake-y way of setting linker flags until newer versions, so we will brute force by appending
# directly to the variables CMAKE maintains.
set(ADDITIONAL_LINKER_FLAGS "-Wl,--no-allow-shlib-undefined")  #Had to remove 'no-undefined' to allow libtda4x/hal_interface_lib.so to build
set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} ${ADDITIONAL_LINKER_FLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${ADDITIONAL_LINKER_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ADDITIONAL_LINKER_FLAGS}")


# If we are doing a debug build, disable compiler optimizations.
if(CMAKE_BUILD_TYPE MATCHES Debug)
  message_info("Disabling compiler optimizations.")
  add_compile_options(-ggdb -O0)
else()
  message_info("Enabling compiler optimizations.")
	# https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html
	# -O3 enables vectorization, which speeds up Eigen dramatically.
	add_compile_options(-O3)

	# Disable Eigen run-time checking, good for ~8% speedup.
	# https://eigen.tuxfamily.org/dox/TopicPreprocessorDirectives.html
	add_definitions(-DEIGEN_NO_DEBUG)
endif()

set(PHANTOM_AI_VERSION_MAJOR 0)
set(PHANTOM_AI_VERSION_MINOR 1)

option(PHANTOM_AI_BUILD_SHARED "" ON)
option(PHANTOM_AI_BUILD_PERCEPTION "" ON)
option(PHANTOM_AI_BUILD_BENCHMARK "" OFF)
option(PHANTOM_AI_BUILD_VISION "" OFF)
option(PHANTOM_AI_BUILD_XAVI "" OFF)
option(PHANTOM_AI_BUILD_HMI "" OFF)
option(PHANTOM_AI_BUILD_IPMI "" OFF)
option(PHANTOM_AI_BUILD_SYSCHECK "" OFF)
option(PHANTOM_AI_BUILD_ENCRYPT_YAML "" OFF)
option(PHANTOM_AI_BUILD_ASAN "" OFF)
option(PHANTOM_AI_BUILD_TSAN "" OFF)
option(PHANTOM_AI_CONFIG_SOCKETCAN "" OFF)
option(PHANTOM_AI_ENABLE_VISUALIZATION "" ON)
option(PHANTOM_AI_USE_CAM_PARAM_INPUT_FOR_VISION_INIT "" OFF)

# For 'make install' / debian build
option(PHANTOM_AI_INSTALL_ASSESTS "" OFF)
option(PHANTOM_AI_INSTALL_DATASET "" OFF)

# Set the path for benchmark schema parsing and data dumping
if(PHANTOM_AI_BUILD_BENCHMARK)
  add_definitions(-DCURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
endif()

# Set customer specific applications if not defined in CACHE. Valid options are
# AUTOBRAIN, AUTOBRAIN_ECO, UNDEFINED (default).
# It can be defined as "cmake .. DCUSTOMER_SPECIFIC_APP=AUTOBRAIN"
SET(CUSTOMER_SPECIFIC_APP "UNDEFINED" CACHE STRING "Customer specific applications")
message_info("CUSTOMER_SPECIFIC_APP = ${CUSTOMER_SPECIFIC_APP}")
if (${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_VH")
  add_definitions(-DAUTOBRAIN_VH_SPECIFIC_APP)
endif()

# Set release target based on these build flag values:
#  INTERNAL(1) - for internal validation
#  INTERNAL(0) - for customer release
SET(SW_RELEASE_TARGET "INTERNAL" CACHE STRING "Software Release Target")
message_info("SW_RELEASE_TARGET = ${SW_RELEASE_TARGET}")

if (${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN" OR ${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_ECO" OR ${CUSTOMER_SPECIFIC_APP} STREQUAL "AUTOBRAIN_VH" )
  find_package(Protobuf REQUIRED)
endif()

SET(DEMO_TARGET "UNDEFINED" CACHE STRING "Target demonstration")
message_info("DEMO_TARGET = ${DEMO_TARGET}")

# TI SDK 9 support
if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64-oe")
  set(CMAKE_SYSTEM_PROCESSOR "aarch64")
  add_compile_definitions(LATER_THAN_TI_SDK_9)
endif()

# Set cmake flag for build target
if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64" OR ${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "TDA4X" OR ${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "AM62A")
  # For ARM builds, use platform flags to set target platform
  if (${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "AGX")
    add_definitions(-DPLATFORM_AGX)
  elseif (${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "AGX2")
    # AGX with newer drive 5.2+ software
    add_definitions(-DPLATFORM_AGX2)
  elseif (${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "V3H")
    add_definitions(-DPLATFORM_V3H)
  elseif (${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "TDA4X" OR ${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "AM62A")

    # Set the TI SoC version.
    # This setting only has effect if PHANTOM_AI_BUILD_PLATFORM equals TDA4X.
    # Supported values include J721E, J721S2, J722S, J784S4, and UNDEFINED (default).
    SET(PHANTOM_AI_TI_SOC "UNDEFINED" CACHE STRING "TI SoC")

    # Specify the exact SoC to prevent a build error by SDK.
    if (${PHANTOM_AI_TI_SOC} STREQUAL "J721E")
      add_definitions(-DPLATFORM_TDA4X)
      add_compile_definitions(SOC_J721E)
      add_compile_definitions(SKIP_3RD_FRAME)
      add_definitions(-DGST_ENABLE)
    elseif (${PHANTOM_AI_TI_SOC} STREQUAL "J721S2")
      add_definitions(-DPLATFORM_TDA4X)
      add_compile_definitions(SOC_J721S2)
      add_compile_definitions(SKIP_3RD_FRAME)
    elseif (${PHANTOM_AI_TI_SOC} STREQUAL "J722S")
      add_definitions(-DPLATFORM_TDA4X)
      add_compile_definitions(SOC_J722S)
      add_compile_definitions(SKIP_3RD_FRAME)
    elseif (${PHANTOM_AI_TI_SOC} STREQUAL "J784S4")
      add_definitions(-DPLATFORM_TDA4X)
      add_compile_definitions(SOC_J784S4)
    elseif (${PHANTOM_AI_TI_SOC} STREQUAL "AM62A")
      add_definitions(-DPLATFORM_AM62A)
      add_compile_definitions(SOC_AM62A)
      add_compile_definitions(SKIP_3RD_FRAME)
      # Experimental feature to submit TIDL jobs earlier
      add_compile_definitions(EARLY_TIDL_SCHEDULING)
    else() # UNDEFINED
      message(FATAL_ERROR "Unsupported value of PHANTOM_AI_TI_SOC ${PHANTOM_AI_TI_SOC}")
    endif()
    message("TI SoC = ${PHANTOM_AI_TI_SOC}")

    if(${PHANTOM_AI_TI_SOC} STREQUAL "J721E")
      set(phantom_gst_lib "phantom_gst")
      add_compile_definitions(ENABLE_GSTREAMER)
      message("========== CMakeLists.txt ENABLE_GSTREAMER=${ENABLE_GSTREAMER}") #WOOSEOK
    else()
      set(phantom_gst_lib "")
    endif()
  else()
    # default to PX2
    add_definitions(-DPLATFORM_PX2)
  endif()
  if (${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "UNDEFINED")
    # Build platform hasn't been set or is wrong, force it to PX2 (ARM default)
    SET(PHANTOM_AI_BUILD_PLATFORM "PX2" CACHE STRING "Default build target platform" FORCE)
  endif()
else()
  if (${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "UNDEFINED")
    # Build platform hasn't been set or is wrong, force it to PC
    SET(PHANTOM_AI_BUILD_PLATFORM "PC" CACHE STRING "Default build target platform" FORCE)
  endif()
endif()

message_info("Target Platform = ${PHANTOM_AI_BUILD_PLATFORM}")
message_info("Target ARCH = ${CMAKE_SYSTEM_PROCESSOR}")

if (CAMERA_SWAP_UV)
  if(${CAMERA_SWAP_UV} STREQUAL "ON")
    add_definitions(-DENABLE_CAMERA_SWAP_UV)
  endif()
endif()

# Remove circular dependency between 3rd_party and and phantom_ai/core headers.
set(spdlog_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/phantom_ai/3rd_party/spdlog/include
    CACHE INTERNAL "spdlog: Include Directories" FORCE)

set(PHANTOM_AI_CORE_INCLUDE_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/src/phantom_ai/core/include
  ${spdlog_INCLUDE_DIR}
  CACHE INTERNAL "" FORCE)

set(PHANTOM_AI_TEST_INCLUDE_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/src/phantom_ai/test/include
  CACHE INTERNAL "" FORCE)
# Turn on to enable logging for interface test
option(INTERFACE_TEST_LOG "Enable InterfaceTestLog definition" OFF)
if(INTERFACE_TEST_LOG)
  add_definitions(-DINTERFACE_TEST_LOG)
endif()

if (ENABLE_HIDING)
  set(PHANTOM_VISION2_LIB_NAME "vision")
  set(PHANTOM_STATIC_CALIBRATION_LIB_NAME "perception_calibration")
  set(PHANTOM_AI_DBC_HMI_LIB "dbc_perception_hmi")
  set(PHANTOM_AI_DBC_PV_LIB "dbc_vision")
else()
  set(PHANTOM_VISION2_LIB_NAME "phantom_vision2")
  set(PHANTOM_STATIC_CALIBRATION_LIB_NAME "phantom_static_calibration")
  set(PHANTOM_AI_DBC_HMI_LIB "dbc_phantom_hmi")
  set(PHANTOM_AI_DBC_PV_LIB "dbc_phantomvision")
endif()
add_definitions(-DPHANTOM_VISION2_LIB_NAME="${PHANTOM_VISION2_LIB_NAME}")
add_definitions(-DPHANTOM_STATIC_CALIBRATION_LIB_NAME="${PHANTOM_STATIC_CALIBRATION_LIB_NAME}")

if (${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "TDA4X")
  set(PHANTOM_TIDL_VERSION "tidl86" CACHE STRING "Specify tidl sdk version")
  message_info("PHANTOM_TIDL_VERSION = ${PHANTOM_TIDL_VERSION}")
elseif (${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "AM62A")
  set(PHANTOM_TIDL_VERSION "tidl91" CACHE STRING "Specify tidl sdk version")
  message_info("PHANTOM_TIDL_VERSION = ${PHANTOM_TIDL_VERSION}")
endif()

add_definitions(-DPHANTOM_TIDL_VERSION="${PHANTOM_TIDL_VERSION}")

message_info("####################")
message_info("3rd Party")
message_info("####################")
# Expliticty tell yaml-cpp to build shared libs.
SET(BUILD_SHARED_LIBS ON CACHE BOOL "Build yaml-cpp shared library" FORCE)
add_subdirectory(src/phantom_ai/3rd_party)

message_info("####################")
message_info("Phantom AI")
message_info("####################")
add_subdirectory(src/phantom_ai)

if (${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "V3H" OR ${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "TDA4X" OR ${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "AM62A")
  message_info("####################")
  message_info("Phantom Linux")
  message_info("####################")
  add_definitions(-DBUILD_PHANTOM_LINUX)
  add_subdirectory(src/phantom_linux)
else()
  message_info("####################")
  message_info("Phantom ROS")
  message_info("####################")
  add_definitions(-DBUILD_PHANTOM_ROS)
  add_subdirectory(src/phantom_ros)
endif()

if(PHANTOM_OS_TEST_COVERAGE)
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
  include(CodeCoverage)

  # Make sure to use the appropriate compiler flags for coverage
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fprofile-arcs -ftest-coverage")

  set(COVERAGE_LCOV_EXCLUDES "'/usr/*'" "'/opt/ros/*'" "'*/build/*'" "'*/3rd_party/*'")
  SETUP_TARGET_FOR_COVERAGE_LCOV(
    NAME phantom_coverage
    EXECUTABLE ctest -j5
    WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
    DEPENDENCIES
      test_launch_files
      test_can_frame_serialization
      test_shape_shifter_msg_sub
      test_paccar_message_converter
      test_ford_message_converter
      test_phantom_bag_player_node
      test_embedded_capture
  )
endif()

## Copy scripts
install(DIRECTORY scripts/
        DESTINATION scripts
)
