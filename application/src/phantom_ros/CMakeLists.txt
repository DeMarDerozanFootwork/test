phantom_project(phantom_ros)

if (PHANTOM_ROS_BUILD_SHARED)
    set(PHANTOM_ROS_BUILD_LIBRARY_TYPE SHARED)
else(PHANTOM_ROS_BUILD_SHARED)
    set(PHANTOM_ROS_BUILD_LIBRARY_TYPE)
endif(PHANTOM_ROS_BUILD_SHARED)

add_compile_options(-Wno-error=switch-enum)
if(PHANTOM_OS_TEST)
  configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
  execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
  if(result)
    message(FATAL_ERROR "CMake step for googletest failed: ${result}")
  endif()
  execute_process(COMMAND ${CMAKE_COMMAND} --build .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
  if(result)
    message(FATAL_ERROR "Build step for googletest failed: ${result}")
  endif()

  # Prevent overriding the parent project's compiler/linker
  # settings on Windows
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

  # Add googletest directly to our build. This defines
  # the gtest and gtest_main targets.
  add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
                  ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
                  EXCLUDE_FROM_ALL)

  include_directories(
      ${gtest_SOURCE_DIR}/include
      /usr/include
      include
  )
endif()

# Disable the logger for now because the build issue
# add_subdirectory(tools)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -std=c++11)

# Finding catkin below clobbers the CMAKE output target for libs and directories.
# Since we want this defined at the top level CMAKE, back it up here,
# and restore it after the catkin steps.
set(BIN_OUTPUT_DIR ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
set(LIB_OUTPUT_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# Disable all the message generators that we don't use.
set(ENV{ROS_LANG_DISABLE} "genlisp:gennodejs:geneus:genlisp")

find_package(
    catkin
    REQUIRED
    COMPONENTS
    cmake_modules
    class_loader
    roscpp
    rosbag
    genmsg
    std_msgs
    geometry_msgs
    nav_msgs
    message_generation
    tf
    sensor_msgs
    pluginlib

    # Used by 3rd_party/usb_cam
    image_transport
    std_srvs
    sensor_msgs
    roscpp_serialization
    rosconsole

    # Used by 3rd_party/image_view
    cv_bridge
    dynamic_reconfigure
    message_filters
    nodelet
    stereo_msgs
)

find_package(orocos_kdl)
find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})

find_package(OpenCV REQUIRED)

find_library(UUID_LIBRARIES NAMES uuid)

# Setup Boost Libraries
set(Boost_USE_STATIC_LIBS       OFF)
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME    OFF)

# Minimum Boost version is 1.58, which is installed as a dep for ROS Kinetic.
find_package(Boost 1.58.0 REQUIRED COMPONENTS filesystem python system thread iostreams)
include_directories(${Boost_INCLUDE_DIRS})

if (${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "AGX" OR ${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "AGX2" OR ${PHANTOM_AI_BUILD_PLATFORM} STREQUAL "PX2")
  find_package(NvMedia REQUIRED)
endif()

###########################################################################
# A "separate" catkin package.
add_subdirectory(3rd_party/ford)
add_subdirectory(3rd_party/parking)
add_subdirectory(3rd_party/paccar)

###########################################################################

## include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${NvMedia_INCLUDE_DIRS}
  ${ROBOWITS_INCLUDE_DIRECTORY}
  ${FORD_INCLUDE_DIRECTORY}
  ${PARKING_INCLUDE_DIRECTORY}
  ${PACCAR_INCLUDE_DIRECTORY}  

  ${PHANTOM_AI_AB_DYNAMICS_RC_PARSER_INCLUDE_DIR}
  ${PHANTOM_AI_CAN_INCLUDE_DIR}
  ${PHANTOM_AI_CAN_MSG_PACK_INCLUDE_DIR}
  ${PHANTOM_AI_COMMON_INCLUDE_DIR}
  ${PHANTOM_AI_CONTI_ARS4G0_INCLUDE_DIR}
  ${PHANTOM_AI_CONTI_MFC4T0_INCLUDE_DIR}
  ${PHANTOM_AI_CONTI_SRR520_INCLUDE_DIR}
  ${PHANTOM_AI_CONTI_VEHICLE_STATE_PARSER_INCLUDE_DIR}
  ${PHANTOM_AI_CONTROL_INCLUDE_DIR}
  ${PHANTOM_AI_CORE_INCLUDE_DIR}
  ${PHANTOM_AI_DBC_INCLUDE_DIR}
  ${PHANTOM_AI_DELPHI_MRR_PARSER_INCLUDE_DIR}
  ${PHANTOM_AI_DELPHI_SRR_PARSER_INCLUDE_DIR}
  ${PHANTOM_AI_DISPLAY_INCLUDE_DIR}
  ${PHANTOM_AI_HMI_INCLUDE_DIR}
  ${PHANTOM_AI_MANDO_FRM_TRANSFORM_INCLUDE_DIR}
  ${PHANTOM_AI_MANDO_FRM_PARSER_INCLUDE_DIR}
  ${PHANTOM_AI_MANDO_LRR_20_FCM_PARSER_INCLUDE_DIR}
  ${PHANTOM_AI_MANDO_FUSION_PARSER_INCLUDE_DIR}
  ${PHANTOM_AI_MANDO_VISION_PARSER_INCLUDE_DIR}
  ${PHANTOM_AI_FORD_L3_LOCALIZATION_INCLUDE_DIR}
  ${PHANTOM_AI_MAP_SERVER_INCLUDE_DIR}
  ${PHANTOM_AI_MAPPER_INCLUDE_DIR}
  ${PHANTOM_AI_MOBILEYE_PARSER_INCLUDE_DIR}
  ${PHANTOM_AI_NETWORK_IO_INCLUDE_DIR}
  ${PHANTOM_AI_NMEA_SERIAL_INCLUDE_DIR}
  ${PHANTOM_AI_OXTS_INCLUDE_DIR}
  ${PHANTOM_AI_PACCAR_VEHICLE_STATE_PARSER_INCLUDE_DIR}
  ${PHANTOM_AI_FORD_VEHICLE_STATE_PARSER_INCLUDE_DIR}
  ${PHANTOM_AI_PHANTOM_VISION_INCLUDE_DIR}
  ${PHANTOM_AI_PHANTOM_VISION2_INCLUDE_DIR}
  ${PHANTOM_AI_PLANNING_INCLUDE_DIR}
  ${PHANTOM_AI_POINT_CLOUD_MOTION_COMPENSATION_INCLUDE_DIR}
  ${PHANTOM_AI_GENESYS_ADMA_GPS_PARSER_INCLUDE_DIR}
  ${PHANTOM_AI_SENSOR_FUSION_INCLUDE_DIR}
  ${PHANTOM_AI_SUPERVISOR_INCLUDE_DIR}
  ${PHANTOM_AI_SPORTAGE_SENSOR_PARSER_INCLUDE_DIR}
  ${PHANTOM_AI_SPORTAGE_VEHICLE_PARSER_INCLUDE_DIR}
  ${PHANTOM_AI_TOOLS_INCLUDE_DIR}
  ${PHANTOM_AI_TRAFFIC_LIGHT_HANDLER_INCLUDE_DIR}
  ${PHANTOM_AI_UMS_GATEWAY_INCLUDE_DIR}
  ${PHANTOM_AI_VEHICLE_STATE_PARSER_INCLUDE_DIR}
  ${PHANTOM_AI_VEHICLE_STATE_PROVIDER_INCLUDE_DIR}
  ${PHANTOM_AI_VELODYNE_PARSER_INCLUDE_DIR}
  ${PHANTOM_AI_EXODUS_FRONT_SENSOR_PARSER_INCLUDE_DIR}
  ${PHANTOM_AI_MANDO_LRR_25_FRM_PARSER_INCLUDE_DIR}
  ${ZEROMQ_INCLUDE_DIR}
  ${EIGEN_INCLUDE_DIR}
#  ${ibeo_INCLUDE_DIR}
  ${json_INCLUDE_DIR}
  ${embedded_network_INCLUDE_DIR}
  ${tilib_INCLUDE_DIR}
  ${PHANTOM_AI_PHANTOM_CALIBRATION_INCLUDE_DIR}
#  ${Ceres_INCLUDE_DIR}
  ${nlohmann_json_schema_validator_INCLUDE_DIR}
)

# Expose the phantom_ros include directory.
set(PHANTOM_ROS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

###################################################
## Generate messages in the 'msg' folder
add_message_files(
   FILES
   AccessoryButtons.msg
   AdvancedDriverAssist.msg
   BasicState.msg
   Can.msg
   CanFrame.msg
   ClusterAndButtons1.msg
   ClusterAndButtons2.msg
   ClusterAndButtons3.msg
   ClusterAndButtons4.msg
   VehicleStateCollection.msg
   ContiArs4g0Object.msg
   ContiArs4g0ObjectList.msg
   ContiMfc4t0Lane.msg
   ContiMfc4t0Object.msg
   ContiMfc4t0Measurement.msg
   ContiSRR520Object.msg
   ContiSRR520Measurement.msg
   DelphiMRRTrackObject.msg
   DelphiMRRTrackObjectList.msg
   DelphiMRRRawObject.msg
   DelphiMRRRawObjectList.msg
   DelphiMRRTrackPreprocessorObject.msg
   DelphiMRRTrackPreprocessorInfo.msg
   DelphiMRRRawPreprocessorObject.msg
   DelphiMRRRawPreprocessorInfo.msg
   DelphiSRRRawObject.msg
   DelphiSRRRawObjectList.msg
   DelphiSRRRawPreprocessorCandidate.msg
   DelphiSRRRawPreprocessorCandidateList.msg
   DelphiSRRRawPreprocessorDebug.msg
   DelphiSRRRawPreprocessorDebugObject.msg
   DestinationArrivalFlag.msg
   ElectronicBrakeInfo.msg
   EngineManagementSystem.msg
   FusionObject.msg
   FusionObjectList.msg
   FutureState.msg
   HdMapLaneWidthInfo.msg
   HealthInfo.msg
   HmiMsg01.msg
   HmiMsg02.msg
   HmiMsg03.msg
   HmiMsg04.msg
   HmiMsg05.msg
   IbeoObject.msg
   IbeoObjectList.msg
   KeyValue.msg
   KeyValueArray.msg
   Level4FusionObject.msg
   Level4FusionMeasurement.msg
   LidarPerceptionObject.msg
   LidarPerceptionRays.msg
   LidarPerceptionMeasurement.msg
   LineSegment.msg
   Localization.msg
   LogButtonMonitor.msg
   LogStatus.msg
   LongAccelControlInfo.msg
   MainHmi.msg
   MandoFRMTrackObject.msg
   MandoFRMTrackObjectList.msg
   MandoLRR20FCMTrackObject.msg
   MandoLRR20FCMTrackObjectList.msg
   MandoLRR25FRMRadarGeneral.msg
   MandoLRR25FRMRadarObject.msg
   MandoLRR25FRMRadarObjectList.msg
   MandoVisionDebug.msg
   MandoVisionObject.msg
   MandoVisionObjectList.msg
   MandoVisionOcgm.msg
   MandoVisionOcgmCell.msg
   MandoVisionOcgmLayer.msg
   MandoVisionOcgmPoint.msg
   Map.msg
   MobileyeEyeQ2Lane.msg
   MobileyeEyeQ2Measurement.msg
   MobileyeEyeQ2Obstacle.msg
   MobileyeEyeQ3CameraStatus.msg
   MobileyeEyeQ3HighBeamAssist.msg
   MobileyeEyeQ3Lane.msg
   MobileyeEyeQ3LaneAdditionalData.msg
   MobileyeEyeQ3Measurement.msg
   MobileyeEyeQ3Obstacle.msg
   MobileyeEyeQ3ObstacleStatus.msg
   MobileyeEyeQ3SignType.msg
   MobileyeEyeQ3SupplementarySignType.msg
   MobileyeEyeQ3TrafficSign.msg
   MobileyeEyeQ3TrafficSignRecognition.msg
   MobisMobileyeEyeQ3Lane.msg
   MobisMobileyeEyeQ3Measurement.msg
   MobisMobileyeEyeQ3Obstacle.msg
   Motion.msg
   NetworkFrame.msg
   NodeState.msg
   OCamCalib.msg
   PathPoint.msg
   PathLaneLinks.msg
   PathViz.msg
   PhantomCameraInfo.msg
   PhantomVisionPoints.msg
   PhantomVisionAEBFlag.msg
   PhantomVisionAEB.msg
   PhantomVisionCenterPath.msg
   PhantomVisionConstructionZone.msg
   PhantomVisionFreeSpace.msg
   PhantomCameraExtraInfoH3aConfig.msg
   PhantomCameraExtraInfo.msg
   PhantomCameraExtraInfoFrame.msg
   PhantomVisionHBA.msg
   PhantomVisionLane.msg
   PhantomVisionLaneBlob.msg
   PhantomVisionLaneBlobList.msg
   PhantomVisionMeasurement.msg
   PhantomVisionNetMsg.msg
   PhantomVisionObject.msg
   PhantomVisionTrafficSign.msg
   PhantomVisionRamp.msg
   PhantomVisionStatus.msg
   PhantomLidarObject.msg
   PhantomLidarObjectList.msg
   VehicleStateFailSafeInformation.msg
   VisionPhantomnetData.msg
   VisionPhantomnetDataList.msg
   VisionPhantomnetDetection.msg
   VisionPhantomnetHorizon.msg
   VisionPhantomnetLaneBlob.msg
   VisionPhantomnetObject.msg
   VisionPhantomnetObjectCuboid.msg
   VisionPhantomnetObjectPart.msg
   VisionFeaturePosition.msg
   VisionFeatureTrackData.msg
   VisionFeatureTrackDataList.msg
   VisionLaneTrackData.msg
   VisionLaneTrackDataList.msg
   VisionTaskTrigger.msg
   PoseMeasurement.msg
   PowerSteering1.msg
   PowerSteering2.msg
   Prediction.msg
   PredictionList.msg
   ProcessHealthState.msg
   ProcessHealthStatus.msg
   RequiredTrafficLightInfo.msg
   RoadBoundarySegment.msg
   RoadEvent.msg
   SimStatus.msg
   SportageSensorDbgOut1.msg
   SportageSensorDbgOut2.msg
   SportageSensorDbgOut3.msg
   SportageSensorDbgOut4.msg
   SportageSensorDbgOut5.msg
   SportageSensorDbgOut6.msg
   SportageSensorDbgOut7.msg
   SportageSensorObstacle.msg
   SportageSensorLane.msg
   SportageSensorMeasurement.msg
   SportageVehicleAdasCmd1020Ms.msg
   SportageVehicleAdasCmd2020Ms.msg
   SportageVehicleAdasCmd3010Ms.msg
   SportageVehicleAdasCmd3150Ms.msg
   SportageVehicleAdas.msg
   SportageVehicleTsrClu.msg
   SportageVehicleTsrNavi.msg
   SteeringAngle.msg
   SmartCruiseControl.msg
   SystemState.msg
   TrafficLightInfo.msg
   UMSGatewayState.msg
   WheelSpeedMeasurements.msg
   EspMeasurements.msg
   GearState.msg
   ABDynamicsRobotMotionPack.msg
   GenesysADMAGPSPack.msg
   NodeStateWithCategory.msg
   NodeManagerControlMsg.msg
   NodeManagerStatesMsg.msg
   Pose3D.msg
   SensorConfig.msg
   PhantomVisionDynamicCalibrationOutput.msg
   PhantomVisionDynamicCalibrationInput.msg
   PhantomVisionDynamicCalibrationCameraInput.msg
   PhantomVisionDynamicCalibrationOutputList.msg
   PhantomVisionVehicleStates.msg
   AdasCmd1020Ms.msg
   ExodusFrontSensorCameraObject.msg
   ExodusFrontSensorCameraObjectList.msg
   ExodusFrontSensorCameraGeneral.msg
   ExodusFrontSensorLaneInfo.msg
   ExodusFrontSensorCamera3DObject.msg
   ExodusFrontSensorRadarGeneral.msg
   ExodusFrontSensorRadarOccupancyDistanceMap.msg
   ExodusFrontSensorRadarObject.msg
   ExodusFrontSensorRadarObjectList.msg
   FunctionalSafetyOutput.msg
   CameraFailSafeInformation.msg   
   PhantomVisionObjectGeneralInfo.msg
 )

add_service_files(
  FILES
  PhantomXAutoDrivingCmd.srv
  PhantomXCapturedEventInfo.srv
  PhantomXDestinationInfo.srv
  PhantomXLogSessionInfo.srv
  PhantomXPopupMessages.srv
  LogCmd.srv
  NoteCmd.srv

  NodeletList.srv
  NodeletLoad.srv
  NodeletUnload.srv

  SimCompleteSession.srv
  SimSessionInfo.srv
  SimStartCommand.srv
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs
   nav_msgs
   sensor_msgs
 )

# Ensure that the message files do not contain unicode characters, which cause issues with Python 2.7.
# Stop generation if we find a message that is not ASCII.
foreach (message_file ${phantom_ros_INSTALLED_MESSAGE_FILES})
  set(FULL_MSG_PATH "${CMAKE_CURRENT_SOURCE_DIR}/${message_file}")
  execute_process(
    # `file` returns the type of file based on the contents.  In this case, we are expecting 'ASCII text' or
    # 'UTF-8 Unicode text' to be returned.  The '-b' is for 'brief', do not print the filename along with the result.
    COMMAND file -b ${FULL_MSG_PATH}
    OUTPUT_VARIABLE file_type_string
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  if (NOT "${file_type_string}" STREQUAL "ASCII text")
    message(
      FATAL_ERROR
      "ROS message '${message_file}' contains unicode characters. Remove them by doing something like this: "
      "`iconv -c -f utf-8 -t ascii ${FULL_MSG_PATH} -o ${FULL_MSG_PATH}`"
    )
  endif()
endforeach()

catkin_package(
        CATKIN_DEPENDS
        roscpp
        std_msgs
        geometry_msgs
        message_runtime
        tf
        sensor_msgs
)

# Nab the path that Catkin wanted to put the binaries, and revert back to the path set in the parent.
set(CATKIN_BIN_TARGET_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
set(CATKIN_LIB_TARGET_DIR ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BIN_OUTPUT_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${LIB_OUTPUT_DIR})

# Utilities like `rosrun` and `roslaunch` expect a certain layout under devel output dir.  Symlink
# the binaries from our bin directory back to where catkin would normally place them.
macro(symlink_catkin_bin node_name)
  add_custom_command (
    TARGET ${node_name} POST_BUILD
    COMMAND mkdir -p ${CATKIN_BIN_TARGET_DIR}
    COMMAND ln -sf ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${node_name} ${CATKIN_BIN_TARGET_DIR}/${node_name}
    # COMMENT "Symlinking ${node_name} back to the catkin workspace."
  )
endmacro()

macro(symlink_catkin_lib node_name)
  add_custom_command (
    TARGET ${node_name} POST_BUILD
    COMMAND mkdir -p ${CATKIN_BIN_TARGET_DIR}
    COMMAND ln -sf ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/lib${node_name}.so ${CATKIN_BIN_TARGET_DIR}/lib${node_name}.so
    # COMMENT "Symlinking lib${node_name}.so back to the catkin workspace."
  )
endmacro()

if (PHANTOM_AI_BUILD_HMI)
  message_info("Building HMI")
  add_subdirectory(phantom_hmi)
  set(PHANTOM_HMI_LIB phantom_hmi_display)
endif()

# Still 3rd_party, but built using some phantom_os constructs.
add_subdirectory(3rd_party/camera_calibration_parsers)
add_subdirectory(3rd_party/camera_info_manager)
add_subdirectory(3rd_party/usb_cam)
add_subdirectory(3rd_party/image_view)

add_subdirectory(3rd_party/ros_msg_parser)

# Node manager library
add_subdirectory(node_manager)
add_subdirectory(phantom_bag_player)
include_directories(
  ${PHANTOM_ROS_NODE_MANAGER_INCLUDE_DIR}
  ${ROS_MSG_PARSER_INCLUDE_DIRECTORY}
  ${PHANTOM_ROS_PHANTOM_BAG_PLAYER_INCLUDE_DIR}
)

# Phantom ROS node/nodelet libs, and our version of the nodelet manager.
add_library(phantom_ros ${PHANTOM_ROS_BUILD_LIBRARY_TYPE} src/phantom_ros.cpp)
add_dependencies(phantom_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(phantom_ros common core oxts nlohmann_json_schema_validator json ${catkin_LIBRARIES})

add_library(phantom_ros_node
  src/callback_queue.cpp
  src/callback_queue_manager.cpp
  src/phantom_ros_node.cpp
  src/phantom_ros_nodelet.cpp
  src/phantom_ros_nodelet_loader.cpp
)
target_link_libraries(phantom_ros_node phantom_ros core ${catkin_LIBRARIES} ${BOOST_LIBRARIES})
symlink_catkin_lib(phantom_ros_node)

add_executable(phantom_ros_nodelet_manager src/phantom_ros_nodelet_manager.cpp)
target_link_libraries(phantom_ros_nodelet_manager core phantom_ros_node ${UUID_LIBRARIES} ${catkin_LIBRARIES}
  ${BOOST_LIBRARIES})
symlink_catkin_bin(phantom_ros_nodelet_manager)

add_library(rviz_helpers ${PHANTOM_ROS_BUILD_LIBRARY_TYPE} src/rviz_helpers.cpp)
target_link_libraries(rviz_helpers phantom_ros common ${catkin_LIBRARIES} dbc_phantomfusion)

add_executable(clpe_image_convert src/clpe_image_convert_node.cpp)
target_link_libraries(clpe_image_convert phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(clpe_image_convert)

add_library(clpe_image_convert_nodelet src/clpe_image_convert_nodelet.cpp)
target_link_libraries(clpe_image_convert_nodelet ${OpenCV_LIBRARIES} phantom_ros phantom_ros_node image_conversion_helper)
symlink_catkin_lib(clpe_image_convert_nodelet)

# Health message and supervisor.
add_library(health_message_handler src/health_message_handler.cpp)
target_link_libraries(health_message_handler phantom_ros)

add_executable(supervisor_node src/supervisor_node.cpp)
target_link_libraries(supervisor_node phantom_ros supervisor phantom_ros_node health_message_handler)
symlink_catkin_bin(supervisor_node)

add_library(supervisor_nodelet src/supervisor_nodelet.cpp)
target_link_libraries(supervisor_nodelet phantom_ros supervisor phantom_ros_node health_message_handler)
symlink_catkin_lib(supervisor_nodelet)

add_executable(process_supervisor_node src/process_supervisor_node.cpp)
target_link_libraries(process_supervisor_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(process_supervisor_node)

add_library(process_supervisor_nodelet src/process_supervisor_nodelet.cpp)
target_link_libraries(process_supervisor_nodelet process_supervisor_gui supervisor phantom_ros phantom_ros_node)
symlink_catkin_lib(process_supervisor_nodelet)

# Delphi SRR parser node/nodelet.
add_executable(delphi_srr_raw_parser_node src/delphi_srr_raw_parser_node.cpp)
target_link_libraries(delphi_srr_raw_parser_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(delphi_srr_raw_parser_node)

add_library(delphi_srr_raw_parser_nodelet src/delphi_srr_raw_parser_nodelet.cpp)
target_link_libraries(delphi_srr_raw_parser_nodelet phantom_ros delphi_srr_parser phantom_ros_node)
symlink_catkin_lib(delphi_srr_raw_parser_nodelet)

add_executable(delphi_srr_raw_viz_node src/delphi_srr_raw_viz_node.cpp)
target_link_libraries(delphi_srr_raw_viz_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(delphi_srr_raw_viz_node)

add_library(delphi_srr_raw_viz_nodelet src/delphi_srr_raw_viz_nodelet.cpp)
target_link_libraries(delphi_srr_raw_viz_nodelet phantom_ros phantom_ros_node)
symlink_catkin_lib(delphi_srr_raw_viz_nodelet)

# CAN Node/Nodelet.
add_executable(can_node src/can_node.cpp)
target_link_libraries(can_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(can_node)

add_library(can_nodelet src/can_nodelet.cpp)
target_link_libraries(can_nodelet can phantom_ros phantom_ros_node)
symlink_catkin_lib(can_nodelet)


# Mobileye Node/Nodelets.
add_executable(mobileye_eyeq2_parser_node src/mobileye_eyeq2_parser_node.cpp)
target_link_libraries(mobileye_eyeq2_parser_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(mobileye_eyeq2_parser_node)

add_library(mobileye_eyeq2_parser_nodelet src/mobileye_eyeq2_parser_nodelet.cpp)
target_link_libraries(mobileye_eyeq2_parser_nodelet phantom_ros mobileye_eyeq2_parser phantom_ros_node)
symlink_catkin_lib(mobileye_eyeq2_parser_nodelet)

add_executable(mobileye_eyeq2_viz_node src/mobileye_eyeq2_viz_node.cpp)
target_link_libraries(mobileye_eyeq2_viz_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(mobileye_eyeq2_viz_node)

add_executable(mobileye_eyeq3_parser_node src/mobileye_eyeq3_parser_node.cpp)
target_link_libraries(mobileye_eyeq3_parser_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(mobileye_eyeq3_parser_node)

add_library(mobileye_eyeq3_parser_nodelet src/mobileye_eyeq3_parser_nodelet.cpp)
target_link_libraries(mobileye_eyeq3_parser_nodelet phantom_ros mobileye_eyeq3_parser phantom_ros_node)
symlink_catkin_lib(mobileye_eyeq3_parser_nodelet)

add_executable(mobileye_eyeq3_viz_node src/mobileye_eyeq3_viz_node.cpp)
target_link_libraries(mobileye_eyeq3_viz_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(mobileye_eyeq3_viz_node)

add_executable(mobis_mobileye_eyeq3_parser_node src/mobis_mobileye_eyeq3_parser_node.cpp)
target_link_libraries(mobis_mobileye_eyeq3_parser_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(mobis_mobileye_eyeq3_parser_node)

add_library(mobis_mobileye_eyeq3_parser_nodelet src/mobis_mobileye_eyeq3_parser_nodelet.cpp)
target_link_libraries(mobis_mobileye_eyeq3_parser_nodelet phantom_ros mobis_mobileye_eyeq3_parser phantom_ros_node)
symlink_catkin_lib(mobis_mobileye_eyeq3_parser_nodelet)

add_executable(mobis_mobileye_eyeq3_viz_node src/mobis_mobileye_eyeq3_viz_node.cpp)
target_link_libraries(mobis_mobileye_eyeq3_viz_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(mobis_mobileye_eyeq3_viz_node)

add_executable(sportage_sensor_parser_node src/sportage_sensor_parser_node.cpp)
target_link_libraries(sportage_sensor_parser_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(sportage_sensor_parser_node)

add_library(sportage_sensor_parser_nodelet src/sportage_sensor_parser_nodelet.cpp)
target_link_libraries(sportage_sensor_parser_nodelet phantom_ros sportage_sensor_parser phantom_ros_node)
symlink_catkin_lib(sportage_sensor_parser_nodelet)

add_executable(sportage_vehicle_parser_node src/sportage_vehicle_parser_node.cpp)
target_link_libraries(sportage_vehicle_parser_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(sportage_vehicle_parser_node)

add_library(sportage_vehicle_parser_nodelet src/sportage_vehicle_parser_nodelet.cpp)
target_link_libraries(sportage_vehicle_parser_nodelet phantom_ros sportage_vehicle_parser phantom_ros_node)
symlink_catkin_lib(sportage_vehicle_parser_nodelet)

add_executable(sportage_sensor_viz_node src/sportage_sensor_viz_node.cpp)
target_link_libraries(sportage_sensor_viz_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(sportage_sensor_viz_node)

add_executable(exodus_front_sensor_parser_node src/exodus_front_sensor_parser_node.cpp)
target_link_libraries(exodus_front_sensor_parser_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(exodus_front_sensor_parser_node)

add_library(exodus_front_sensor_parser_nodelet src/exodus_front_sensor_parser_nodelet.cpp)
target_link_libraries(exodus_front_sensor_parser_nodelet phantom_ros exodus_front_sensor_parser phantom_ros_node)
symlink_catkin_lib(exodus_front_sensor_parser_nodelet)

add_executable(exodus_front_sensor_viz_node src/exodus_front_sensor_viz_node.cpp)
target_link_libraries(exodus_front_sensor_viz_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(exodus_front_sensor_viz_node)

add_executable(mando_lrr_25_frm_parser_node src/mando_lrr_25_frm_parser_node.cpp)
target_link_libraries(mando_lrr_25_frm_parser_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(mando_lrr_25_frm_parser_node)

add_library(mando_lrr_25_frm_parser_nodelet src/mando_lrr_25_frm_parser_nodelet.cpp)
target_link_libraries(mando_lrr_25_frm_parser_nodelet phantom_ros mando_lrr_25_frm_parser phantom_ros_node)
symlink_catkin_lib(mando_lrr_25_frm_parser_nodelet)

add_executable(mando_lrr_25_frm_viz_node src/mando_lrr_25_frm_viz_node.cpp)
target_link_libraries(mando_lrr_25_frm_viz_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(mando_lrr_25_frm_viz_node)

# Map server nodelet and node.
add_library(map_server_nodelet src/map_server_nodelet.cpp)
target_link_libraries(map_server_nodelet phantom_ros phantom_ros_node ford map_server)
symlink_catkin_lib(map_server_nodelet)

add_executable(map_server_node src/map_server_node.cpp)
target_link_libraries(map_server_node phantom_ros phantom_ros_node)
symlink_catkin_bin(map_server_node)

# TF Node.
add_executable(tf_node src/tf_node.cpp)
target_link_libraries(tf_node phantom_ros phantom_ros_node)
symlink_catkin_bin(tf_node)


# Ibeo Node.
#add_executable(ibeo_node src/ibeo_node.cpp)
#target_link_libraries(ibeo_node phantom_ros ibeosdk phantom_ros_node)
#symlink_catkin_bin(ibeo_node)

#add_executable(ibeo_viz_node src/ibeo_viz_node.cpp)
#target_link_libraries(ibeo_viz_node phantom_ros rviz_helpers phantom_ros_node)
#symlink_catkin_bin(ibeo_viz_node)


# OxTS node.
add_executable(oxts_node src/oxts_node.cpp)
target_link_libraries(oxts_node phantom_ros phantom_ros_node)
symlink_catkin_bin(oxts_node)

add_library(oxts_nodelet src/oxts_nodelet.cpp)
target_link_libraries(oxts_nodelet oxts phantom_ros phantom_ros_node)
symlink_catkin_lib(oxts_nodelet)


# Delphi MRR Node/Nodelet.
add_executable(delphi_mrr_track_parser_node src/delphi_mrr_track_parser_node.cpp)
target_link_libraries(delphi_mrr_track_parser_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(delphi_mrr_track_parser_node)

add_library(delphi_mrr_track_parser_nodelet src/delphi_mrr_track_parser_nodelet.cpp)
target_link_libraries(delphi_mrr_track_parser_nodelet phantom_ros delphi_mrr_parser phantom_ros_node)
symlink_catkin_lib(delphi_mrr_track_parser_nodelet)

add_executable(delphi_mrr_raw_parser_node src/delphi_mrr_raw_parser_node.cpp)
target_link_libraries(delphi_mrr_raw_parser_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(delphi_mrr_raw_parser_node)

add_library(delphi_mrr_raw_parser_nodelet src/delphi_mrr_raw_parser_nodelet.cpp)
target_link_libraries(delphi_mrr_raw_parser_nodelet core phantom_ros_node delphi_mrr_parser ${catkin_LIBRARIES})
symlink_catkin_lib(delphi_mrr_raw_parser_nodelet)

add_executable(delphi_mrr_track_viz_node src/delphi_mrr_track_viz_node.cpp)
target_link_libraries(delphi_mrr_track_viz_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(delphi_mrr_track_viz_node)

add_library(delphi_mrr_track_viz_nodelet src/delphi_mrr_track_viz_nodelet.cpp)
target_link_libraries(delphi_mrr_track_viz_nodelet core phantom_ros_node)
symlink_catkin_lib(delphi_mrr_track_viz_nodelet)

add_executable(delphi_mrr_raw_viz_node src/delphi_mrr_raw_viz_node.cpp)
target_link_libraries(delphi_mrr_raw_viz_node phantom_ros phantom_ros_node)
symlink_catkin_bin(delphi_mrr_raw_viz_node)


# Velodyne Node/Nodelet
add_executable(velodyne_parser_node src/velodyne_parser_node.cpp)
target_link_libraries(velodyne_parser_node phantom_ros phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(velodyne_parser_node)

add_library(velodyne_parser_nodelet src/velodyne_parser_nodelet.cpp)
target_link_libraries(velodyne_parser_nodelet velodyne_parser phantom_ros phantom_ros_node)
symlink_catkin_lib(velodyne_parser_nodelet)

# Conti ARS radar and MFC camera node/nodelets.
add_executable(conti_ars4g0_parser_node src/continental/conti_ars4g0_parser_node.cpp)
target_link_libraries(conti_ars4g0_parser_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(conti_ars4g0_parser_node)

add_library(conti_ars4g0_parser_nodelet src/continental/conti_ars4g0_parser_nodelet.cpp)
target_link_libraries(conti_ars4g0_parser_nodelet phantom_ros conti_ars4g0_parser phantom_ros_node)
symlink_catkin_lib(conti_ars4g0_parser_nodelet)

add_executable(conti_ars4g0_viz_node src/continental/conti_ars4g0_viz_node.cpp)
target_link_libraries(conti_ars4g0_viz_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(conti_ars4g0_viz_node)

add_executable(conti_mfc4t0_parser_node src/continental/conti_mfc4t0_parser_node.cpp)
target_link_libraries(conti_mfc4t0_parser_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(conti_mfc4t0_parser_node)

add_library(conti_mfc4t0_parser_nodelet src/continental/conti_mfc4t0_parser_nodelet.cpp)
target_link_libraries(conti_mfc4t0_parser_nodelet phantom_ros conti_mfc4t0_parser phantom_ros_node)
symlink_catkin_lib(conti_mfc4t0_parser_nodelet)

add_executable(conti_mfc4t0_viz_node src/continental/conti_mfc4t0_viz_node.cpp)
target_link_libraries(conti_mfc4t0_viz_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(conti_mfc4t0_viz_node)

add_executable(conti_srr520_parser_node src/continental/conti_srr520_parser_node.cpp)
target_link_libraries(conti_srr520_parser_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(conti_srr520_parser_node)

add_library(conti_srr520_parser_nodelet src/continental/conti_srr520_parser_nodelet.cpp)
target_link_libraries(conti_srr520_parser_nodelet phantom_ros conti_srr520_parser phantom_ros_node)
symlink_catkin_lib(conti_srr520_parser_nodelet)

add_executable(conti_srr520_viz_node src/continental/conti_srr520_viz_node.cpp)
target_link_libraries(conti_srr520_viz_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(conti_srr520_viz_node)

# Sensor Fusion + related Node/nodelets.
add_executable(sensor_fusion_node src/sensor_fusion_node.cpp)
target_link_libraries(sensor_fusion_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(sensor_fusion_node)

add_library(sensor_fusion_nodelet src/sensor_fusion_nodelet.cpp)
target_link_libraries(sensor_fusion_nodelet phantom_ros rviz_helpers sensor_fusion phantom_ros_node)
symlink_catkin_lib(sensor_fusion_nodelet)

add_executable(sensor_fusion_viz_node src/sensor_fusion_viz_node.cpp)
target_link_libraries(sensor_fusion_viz_node phantom_ros_node rviz_helpers ${catkin_LIBRARIES})
symlink_catkin_bin(sensor_fusion_viz_node)

add_library(sensor_fusion_viz_nodelet src/sensor_fusion_viz_nodelet.cpp)
target_link_libraries(sensor_fusion_viz_nodelet phantom_ros rviz_helpers phantom_ros_node)
symlink_catkin_lib(sensor_fusion_viz_nodelet)

add_executable(delphi_mrr_track_preprocessor_viz_node src/delphi_mrr_track_preprocessor_viz_node.cpp)
target_link_libraries(delphi_mrr_track_preprocessor_viz_node phantom_ros phantom_ros_node)
symlink_catkin_bin(delphi_mrr_track_preprocessor_viz_node)

add_executable(delphi_mrr_raw_preprocessor_viz_node src/delphi_mrr_raw_preprocessor_viz_node.cpp)
target_link_libraries(delphi_mrr_raw_preprocessor_viz_node phantom_ros phantom_ros_node)
symlink_catkin_bin(delphi_mrr_raw_preprocessor_viz_node)

add_executable(delphi_srr_raw_preprocessor_debug_viz_node src/delphi_srr_raw_preprocessor_debug_viz_node.cpp)
target_link_libraries(delphi_srr_raw_preprocessor_debug_viz_node core common phantom_ros phantom_ros_node)
symlink_catkin_bin(delphi_srr_raw_preprocessor_debug_viz_node)


# NMEA GPS node.
add_executable(nmea_gps_node src/nmea_gps_node.cpp)
target_link_libraries(nmea_gps_node phantom_ros nmea nmea_serial phantom_ros_node)
symlink_catkin_bin(nmea_gps_node)


# UMS Gateway node.
add_executable(ums_gateway_node src/ums_gateway_node.cpp)
target_link_libraries(ums_gateway_node phantom_ros ums_gateway phantom_ros_node)
symlink_catkin_bin(ums_gateway_node)

add_library(ums_gateway_nodelet src/ums_gateway_nodelet.cpp)
target_link_libraries(ums_gateway_nodelet phantom_ros ums_gateway phantom_ros_node)
symlink_catkin_lib(ums_gateway_nodelet)

# Vehicle state parser.
add_executable(vehicle_state_parser_node src/vehicle_state_parser_node.cpp)
target_link_libraries(vehicle_state_parser_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(vehicle_state_parser_node)

add_library(vehicle_state_parser_nodelet src/vehicle_state_parser_nodelet.cpp)
target_link_libraries(vehicle_state_parser_nodelet phantom_ros vehicle_state_parser phantom_ros_node)
symlink_catkin_lib(vehicle_state_parser_nodelet)

add_executable(conti_vehicle_state_parser_node src/continental/conti_vehicle_state_parser_node.cpp)
target_link_libraries(conti_vehicle_state_parser_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(conti_vehicle_state_parser_node)

add_library(conti_vehicle_state_parser_nodelet src/continental/conti_vehicle_state_parser_nodelet.cpp)
target_link_libraries(conti_vehicle_state_parser_nodelet conti_vehicle_state_parser phantom_ros phantom_ros_node)
symlink_catkin_lib(conti_vehicle_state_parser_nodelet)

add_executable(paccar_vehicle_state_parser_node src/paccar_vehicle_state_parser_node.cpp)
target_link_libraries(paccar_vehicle_state_parser_node phantom_ros phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(paccar_vehicle_state_parser_node)

add_library(paccar_vehicle_state_parser_nodelet src/paccar_vehicle_state_parser_nodelet.cpp)
target_link_libraries(paccar_vehicle_state_parser_nodelet paccar_vehicle_state_parser phantom_ros phantom_ros_node)
symlink_catkin_lib(paccar_vehicle_state_parser_nodelet)

add_executable(paccar_viz_node src/paccar_viz_node.cpp)
target_link_libraries(paccar_viz_node phantom_ros phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(paccar_viz_node)

add_library(paccar_viz_nodelet src/paccar_viz_nodelet.cpp)
target_link_libraries(paccar_viz_nodelet phantom_ros phantom_ros_node paccar)
symlink_catkin_lib(paccar_viz_nodelet)

add_executable(ford_vehicle_state_parser_node src/ford_vehicle_state_parser_node.cpp)
target_link_libraries(ford_vehicle_state_parser_node phantom_ros phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(ford_vehicle_state_parser_node)

add_library(ford_vehicle_state_parser_nodelet src/ford_vehicle_state_parser_nodelet.cpp)
target_link_libraries(ford_vehicle_state_parser_nodelet ford_vehicle_state_parser phantom_ros phantom_ros_node)
symlink_catkin_lib(ford_vehicle_state_parser_nodelet)

add_executable(vehicle_state_provider_node src/vehicle_state_provider_node.cpp)
target_link_libraries(vehicle_state_provider_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(vehicle_state_provider_node)

add_library(vehicle_state_provider_nodelet src/vehicle_state_provider_nodelet.cpp)
target_link_libraries(vehicle_state_provider_nodelet phantom_ros vehicle_state_provider phantom_ros_node)
symlink_catkin_lib(vehicle_state_provider_nodelet)

# Mando FRM parser node/nodelet
add_executable(mando_frm_track_parser_node src/mando_frm_track_parser_node.cpp)
target_link_libraries(mando_frm_track_parser_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(mando_frm_track_parser_node)

add_library(mando_frm_track_parser_nodelet src/mando_frm_track_parser_nodelet.cpp)
target_link_libraries(mando_frm_track_parser_nodelet phantom_ros mando_frm_parser phantom_ros_node)
symlink_catkin_lib(mando_frm_track_parser_nodelet)

add_executable(mando_frm_track_viz_node src/mando_frm_track_viz_node.cpp)
target_link_libraries(mando_frm_track_viz_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(mando_frm_track_viz_node)

add_library(mando_frm_track_viz_nodelet src/mando_frm_track_viz_nodelet.cpp)
target_link_libraries(mando_frm_track_viz_nodelet core phantom_ros_node)
symlink_catkin_lib(mando_frm_track_viz_nodelet)

# Mando LRR20FCM parser node/nodelet
add_executable(mando_lrr_20_fcm_track_parser_node src/mando_lrr_20_fcm_track_parser_node.cpp)
target_link_libraries(mando_lrr_20_fcm_track_parser_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(mando_lrr_20_fcm_track_parser_node)

add_library(mando_lrr_20_fcm_track_parser_nodelet src/mando_lrr_20_fcm_track_parser_nodelet.cpp)
target_link_libraries(mando_lrr_20_fcm_track_parser_nodelet phantom_ros mando_lrr_20_fcm_parser phantom_ros_node)
symlink_catkin_lib(mando_lrr_20_fcm_track_parser_nodelet)

add_executable(mando_lrr_20_fcm_track_viz_node src/mando_lrr_20_fcm_track_viz_node.cpp)
target_link_libraries(mando_lrr_20_fcm_track_viz_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(mando_lrr_20_fcm_track_viz_node)

add_library(mando_lrr_20_fcm_track_viz_nodelet src/mando_lrr_20_fcm_track_viz_nodelet.cpp)
target_link_libraries(mando_lrr_20_fcm_track_viz_nodelet core phantom_ros_node)
symlink_catkin_lib(mando_lrr_20_fcm_track_viz_nodelet)

# Mando vis node.
add_executable(mando_fusion_viz_node src/mando_fusion_viz_node.cpp)
target_link_libraries(mando_fusion_viz_node phantom_ros mando_fusion_parser rviz_helpers common phantom_ros_node)
symlink_catkin_bin(mando_fusion_viz_node)

# CSI camera node/nodelet.
add_executable(csi_camera_node src/csi_camera_node.cpp)
target_link_libraries(csi_camera_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(csi_camera_node)

add_library(csi_camera_nodelet src/csi_camera_nodelet.cpp)
target_link_libraries(csi_camera_nodelet phantom_ros ${NvMedia_LIBRARIES} core common phantom_ros_node)
symlink_catkin_lib(csi_camera_nodelet)

add_executable(csi_snapshot_node src/csi_snapshot_node.cpp)
target_link_libraries(csi_snapshot_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(csi_snapshot_node)

add_library(csi_snapshot_nodelet src/csi_snapshot_nodelet.cpp)
target_link_libraries(csi_snapshot_nodelet phantom_ros ${NvMedia_LIBRARIES} phantom_ros_node)
symlink_catkin_lib(csi_snapshot_nodelet)

add_executable(nvsipl_camera_node src/nvsipl_camera_node.cpp)
target_link_libraries(nvsipl_camera_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(nvsipl_camera_node)

add_library(nvsipl_camera_nodelet src/nvsipl_camera_nodelet.cpp)
target_compile_definitions(nvsipl_camera_nodelet PRIVATE -DNVMEDIA_NVSCI_ENABLE)
target_link_libraries(nvsipl_camera_nodelet phantom_ros core ${NvMedia_LIBRARIES} phantom_ros_node)
symlink_catkin_lib(nvsipl_camera_nodelet)

# Camera logging node/nodelet.
add_executable(camera_sync_logging_node src/camera_sync_logging_node.cpp)
target_link_libraries(camera_sync_logging_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(camera_sync_logging_node)

add_library(camera_sync_logging_nodelet src/camera_sync_logging_nodelet.cpp)
target_link_libraries(camera_sync_logging_nodelet phantom_ros core image_conversion_helper phantom_ros_node)
symlink_catkin_lib(camera_sync_logging_nodelet)

# Phantom Vision node/nodelet.
if (PHANTOM_AI_BUILD_PERCEPTION)
  # PhantomVision2
  add_executable(vision_node2 src/vision_node2.cpp)
  target_link_libraries(vision_node2 phantom_ros_node ${catkin_LIBRARIES})
  symlink_catkin_bin(vision_node2)

  if (ENABLE_ZEROMQ)
    set(zmq_lib "zmq")
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(ZMQ REQUIRED libzmq)
    include_directories(${ZMQ_INCLUDE_DIRS})
  else()
    set(zmq_lib "")
  endif()
  add_library(vision_nodelet2 src/vision_nodelet2.cpp src/vision_message_internal.cpp)
  target_link_libraries(vision_nodelet2 vision_can_message_pack ${zmq_lib} phantom_ros ${PHANTOM_VISION2_LIB_NAME} phantom_ros_node)
  symlink_catkin_lib(vision_nodelet2)

  add_executable(vision_viewer_node2 src/vision_viewer_node2.cpp)
  target_link_libraries(vision_viewer_node2 phantom_ros_node ${catkin_LIBRARIES})
  symlink_catkin_bin(vision_viewer_node2)

  add_library(vision_viewer_nodelet2 src/vision_viewer_nodelet2.cpp src/vision_message_internal.cpp)
  target_link_libraries(vision_viewer_nodelet2 phantom_ros ${PHANTOM_VISION2_LIB_NAME} phantom_ros_node)
  symlink_catkin_lib(vision_viewer_nodelet2)

  # Phantom Static Calibration
  add_executable(calibration_node src/calibration_node.cpp)
  target_link_libraries(calibration_node phantom_ros_node ${catkin_LIBRARIES})
  symlink_catkin_bin(calibration_node)

  add_library(calibration_nodelet src/calibration_nodelet.cpp src/vision_message_internal.cpp)
  target_link_libraries(calibration_nodelet phantom_ros ${PHANTOM_STATIC_CALIBRATION_LIB_NAME} phantom_ros_node)
  symlink_catkin_lib(calibration_nodelet)
endif()

# Phantom vision viz node/nodelet
add_executable(phantom_vision_viz_node src/phantom_vision_viz_node.cpp)
target_link_libraries(phantom_vision_viz_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(phantom_vision_viz_node)

add_library(phantom_vision_viz_nodelet src/phantom_vision_viz_nodelet.cpp)
target_link_libraries(phantom_vision_viz_nodelet core phantom_ros_node)
symlink_catkin_lib(phantom_vision_viz_nodelet)


# Log server node/nodelets.
add_executable(log_button_monitor_node src/log_button_monitor_node.cpp)
target_link_libraries(log_button_monitor_node core common tools phantom_ros phantom_ros_node)
symlink_catkin_bin(log_button_monitor_node)

add_library(log_server_nodelet src/log_server_nodelet.cpp src/log_server.cpp src/aeb_mileage_manager.cpp)
target_link_libraries(log_server_nodelet core tools common phantom_ros phantom_ros_node ros_msg_parser nlohmann_json_schema_validator ${catkin_LIBRARIES})
symlink_catkin_lib(log_server_nodelet)

add_executable(log_server_node src/log_server_node.cpp src/log_server.cpp src/aeb_mileage_manager.cpp)
target_link_libraries(log_server_node core tools common phantom_ros_node ros_msg_parser log_server_nodelet nlohmann_json_schema_validator ${catkin_LIBRARIES})
symlink_catkin_bin(log_server_node)

# Log Status GUI nodes
add_executable(log_status_gui_node src/log_status_gui_node.cpp)
target_link_libraries(log_status_gui_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(log_status_gui_node)

add_library(log_status_gui_nodelet src/log_status_gui_nodelet.cpp)
target_link_libraries(log_status_gui_nodelet log_status_gui core phantom_ros phantom_ros_node)
symlink_catkin_lib(log_status_gui_nodelet)

# Log Bandwidth Tester nodes
add_executable(bandwidth_tester_node src/bandwidth_tester_node.cpp)
target_link_libraries(bandwidth_tester_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(bandwidth_tester_node)

add_library(bandwidth_tester_nodelet src/bandwidth_tester_nodelet.cpp)
target_link_libraries(bandwidth_tester_nodelet core phantom_ros phantom_ros_node)
symlink_catkin_lib(bandwidth_tester_nodelet)

# HMI nodes.
add_executable(main_hmi_node src/main_hmi_node.cpp)
target_link_libraries(main_hmi_node common hmi phantom_ros phantom_ros_node ${OpenCV_LIBRARIES})
symlink_catkin_bin(main_hmi_node)

add_executable(main_hmi_display_node src/main_hmi_display_node.cpp)
target_compile_definitions(main_hmi_display_node PRIVATE -DPHANTOM_AI_BUILD_HMI=$<BOOL:${PHANTOM_AI_BUILD_HMI}>)
target_link_libraries(main_hmi_display_node core ${PHANTOM_HMI_LIB})
symlink_catkin_bin(main_hmi_display_node)

add_executable(hmi_viz_node src/hmi_viz_node.cpp)
target_link_libraries(hmi_viz_node phantom_ros hmi ${OpenCV_LIBRARIES} phantom_ros_node)
symlink_catkin_bin(hmi_viz_node)


# TI node/nodelets.
add_executable(time_sender_node src/time_sender_node.cpp)
target_link_libraries(time_sender_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(time_sender_node)

add_library(time_sender_nodelet src/time_sender_nodelet.cpp)
target_link_libraries(time_sender_nodelet embedded_network phantom_ros phantom_ros_node)
symlink_catkin_lib(time_sender_nodelet)

add_executable(ti_video_capture_node src/ti_video_capture_node.cpp)
target_link_libraries(ti_video_capture_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(ti_video_capture_node)

add_library(ti_video_capture_nodelet src/ti_video_capture_nodelet.cpp)
target_link_libraries(ti_video_capture_nodelet tilib ${OpenCV_LIBRARIES} phantom_ros phantom_ros_node)
symlink_catkin_lib(ti_video_capture_nodelet)

add_executable(ti_video_streamer_node src/ti_video_streamer_node.cpp)
target_link_libraries(ti_video_streamer_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(ti_video_streamer_node)

add_library(ti_video_streamer_nodelet src/ti_video_streamer_nodelet.cpp)
target_link_libraries(ti_video_streamer_nodelet tilib ${OpenCV_LIBRARIES} phantom_ros phantom_ros_node)
symlink_catkin_lib(ti_video_streamer_nodelet)

add_executable(embedded_capture_node src/embedded_capture_node.cpp)
target_link_libraries(embedded_capture_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(embedded_capture_node)

add_library(embedded_capture_nodelet src/embedded_capture_nodelet.cpp)
target_link_libraries(embedded_capture_nodelet embedded_network ${OpenCV_LIBRARIES} phantom_ros phantom_ros_node)
symlink_catkin_lib(embedded_capture_nodelet)

add_executable(embedded_streamer_node src/embedded_streamer_node.cpp)
target_link_libraries(embedded_streamer_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(embedded_streamer_node)

add_library(embedded_streamer_nodelet src/embedded_streamer_nodelet.cpp)
target_link_libraries(embedded_streamer_nodelet embedded_network ${OpenCV_LIBRARIES} phantom_ros phantom_ros_node)
symlink_catkin_lib(embedded_streamer_nodelet)

add_executable(phantom_rosbag_converter_node src/phantom_rosbag_converter_node.cpp)
target_link_libraries(phantom_rosbag_converter_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(phantom_rosbag_converter_node)

add_library(phantom_rosbag_converter_nodelet src/phantom_rosbag_converter_nodelet.cpp)
target_link_libraries(phantom_rosbag_converter_nodelet ${OpenCV_LIBRARIES} phantom_ros phantom_ros_node)
symlink_catkin_lib(phantom_rosbag_converter_nodelet)

# Mando Vision Project
add_executable(ti_viz_node src/mando/ti_viz_node.cpp)
target_link_libraries(ti_viz_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(ti_viz_node)

add_library(ti_viz_nodelet src/mando/ti_viz_nodelet.cpp)
target_link_libraries(ti_viz_nodelet ${OpenCV_LIBRARIES} phantom_ros phantom_ros_node)
symlink_catkin_lib(ti_viz_nodelet)

add_executable(mando_vision_parser_node src/mando/mando_vision_parser_node.cpp)
target_link_libraries(mando_vision_parser_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(mando_vision_parser_node)

add_library(mando_vision_parser_nodelet src/mando/mando_vision_parser_nodelet.cpp)
target_link_libraries(mando_vision_parser_nodelet mando_vision_parser phantom_ros phantom_ros_node)
symlink_catkin_lib(mando_vision_parser_nodelet)

# Image view.
add_executable(nv_image_view_node src/nv_image_view_node.cpp)
target_link_libraries(nv_image_view_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(nv_image_view_node)

add_library(nv_image_view_nodelet src/nv_image_view_nodelet.cpp)
target_link_libraries(nv_image_view_nodelet ${OpenCV_LIBRARIES} phantom_ros phantom_ros_node display)
symlink_catkin_lib(nv_image_view_nodelet)

add_executable(image_convert_node src/image_convert_node.cpp)
target_link_libraries(image_convert_node phantom_ros phantom_ros_node)
symlink_catkin_bin(image_convert_node)

add_library(image_convert_nodelet src/image_convert_nodelet.cpp)
target_link_libraries(image_convert_nodelet ${OpenCV_LIBRARIES} phantom_ros phantom_ros_node image_conversion_helper)
symlink_catkin_lib(image_convert_nodelet)

# Traffic light handler node.
add_executable(traffic_light_handler_node src/traffic_light_handler_node.cpp)
target_link_libraries(traffic_light_handler_node traffic_light_handler phantom_ros phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(traffic_light_handler_node)

add_library(traffic_light_handler_nodelet src/traffic_light_handler_nodelet.cpp)
target_link_libraries(traffic_light_handler_nodelet traffic_light_handler phantom_ros phantom_ros_node)
symlink_catkin_lib(traffic_light_handler_nodelet)

# Network IO node/nodelet.
add_executable(network_io_node src/network_io_node.cpp)
target_link_libraries(network_io_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(network_io_node)

add_library(network_io_nodelet src/network_io_nodelet.cpp)
target_link_libraries(network_io_nodelet core phantom_ros phantom_ros_node network_io)
symlink_catkin_lib(network_io_nodelet)

# Mission control GUI.
add_executable(mission_control_gui_node src/mission_control_gui_node.cpp)
target_link_libraries(mission_control_gui_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(mission_control_gui_node)

add_library(mission_control_gui_nodelet src/mission_control_gui_nodelet.cpp)
target_link_libraries(mission_control_gui_nodelet mission_control_gui phantom_ros phantom_ros_node)
symlink_catkin_lib(mission_control_gui_nodelet)


# Note server node/nodelet.
add_executable(note_server_node src/note_server_node.cpp)
target_link_libraries(note_server_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(note_server_node)

add_library(note_server_nodelet src/note_server_nodelet.cpp)
target_link_libraries(note_server_nodelet core phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_lib(note_server_nodelet)


if(PHANTOM_AI_BUILD_PERCEPTION)
  # Point cloud motion compensation node/nodelet.
  add_executable(point_cloud_motion_compensation_node src/point_cloud_motion_compensation_node.cpp)
  target_link_libraries(point_cloud_motion_compensation_node phantom_ros_node ${catkin_LIBRARIES})
  symlink_catkin_bin(point_cloud_motion_compensation_node)

  add_library(point_cloud_motion_compensation_nodelet src/point_cloud_motion_compensation_nodelet.cpp)
  target_link_libraries(point_cloud_motion_compensation_nodelet point_cloud_motion_compensation phantom_ros phantom_ros_node)
  symlink_catkin_lib(point_cloud_motion_compensation_nodelet)
endif()

add_executable(phantom_lidar_viz_node src/phantom_lidar_viz_node.cpp)
target_link_libraries(phantom_lidar_viz_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(phantom_lidar_viz_node)

add_library(phantom_lidar_viz_nodelet src/phantom_lidar_viz_nodelet.cpp)
target_link_libraries(phantom_lidar_viz_nodelet core phantom_ros_node)
symlink_catkin_lib(phantom_lidar_viz_nodelet)

# Ford nodes
add_executable(ford_message_converter_node src/ford/ford_message_converter_node.cpp)
target_link_libraries(ford_message_converter_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(ford_message_converter_node)

add_library(ford_message_converter_nodelet src/ford/ford_message_converter_nodelet.cpp)
target_link_libraries(ford_message_converter_nodelet core ford phantom_ros_node)
symlink_catkin_lib(ford_message_converter_nodelet)

# PACCAR nodes
add_executable(paccar_message_converter_node src/paccar/paccar_message_converter_node.cpp)
target_link_libraries(paccar_message_converter_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(paccar_message_converter_node)

add_library(paccar_message_converter_nodelet src/paccar/paccar_message_converter_nodelet.cpp)
target_link_libraries(paccar_message_converter_nodelet core paccar phantom_ros_node)
symlink_catkin_lib(paccar_message_converter_nodelet)


# Parking nodes
add_executable(parking_message_converter_node src/parking/parking_message_converter_node.cpp)
target_link_libraries(parking_message_converter_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(parking_message_converter_node)

add_library(parking_message_converter_nodelet src/parking/parking_message_converter_nodelet.cpp)
target_link_libraries(parking_message_converter_nodelet core parking phantom_ros_node)
symlink_catkin_lib(parking_message_converter_nodelet)


# AB Dynamics nodes.
add_executable(ab_dynamics_rc_parser_node src/ab_dynamics_rc_parser_node.cpp)
target_link_libraries(ab_dynamics_rc_parser_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(ab_dynamics_rc_parser_node)

add_library(ab_dynamics_rc_parser_nodelet src/ab_dynamics_rc_parser_nodelet.cpp)
target_link_libraries(ab_dynamics_rc_parser_nodelet phantom_ros ab_dynamics_rc_parser phantom_ros_node)
symlink_catkin_lib(ab_dynamics_rc_parser_nodelet)

add_executable(ab_dynamics_rc_viz_node src/ab_dynamics_rc_viz_node.cpp)
target_link_libraries(ab_dynamics_rc_viz_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(ab_dynamics_rc_viz_node)

add_executable(ford_sensor_fusion_viz_node src/ford/ford_sensor_fusion_viz_node.cpp)
target_link_libraries(ford_sensor_fusion_viz_node phantom_ros_node rviz_helpers ${catkin_LIBRARIES})
symlink_catkin_bin(ford_sensor_fusion_viz_node)

add_library(ford_sensor_fusion_viz_nodelet src/ford/ford_sensor_fusion_viz_nodelet.cpp)
target_link_libraries(ford_sensor_fusion_viz_nodelet phantom_ros ford rviz_helpers phantom_ros_node)
symlink_catkin_lib(ford_sensor_fusion_viz_nodelet)

# Node handler node and nodelet.
add_executable(node_manager_node src/node_manager_node.cpp)
target_link_libraries(node_manager_node phantom_ros phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(node_manager_node)

add_library(node_manager_nodelet src/node_manager_nodelet.cpp)
target_link_libraries(node_manager_nodelet node_manager phantom_ros phantom_ros_node)
symlink_catkin_lib(node_manager_nodelet)

# GenesysADMA GPS to ROS nodes.
add_executable(genesys_adma_gps_parser_node src/genesys_adma_gps_parser_node.cpp)
target_link_libraries(genesys_adma_gps_parser_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(genesys_adma_gps_parser_node)

add_library(genesys_adma_gps_parser_nodelet src/genesys_adma_gps_parser_nodelet.cpp)
target_link_libraries(genesys_adma_gps_parser_nodelet phantom_ros genesys_adma_parser phantom_ros_node)
symlink_catkin_lib(genesys_adma_gps_parser_nodelet)

add_library(phantom_bag_player_nodelet src/phantom_bag_player_nodelet.cpp)
target_link_libraries(phantom_bag_player_nodelet phantom_bag_player phantom_ros phantom_ros_node ${Boost_SYSTEM_LIBRARY})
symlink_catkin_lib(phantom_bag_player_nodelet)

add_executable(phantom_bag_player_node src/phantom_bag_player_node.cpp)
target_link_libraries(phantom_bag_player_node phantom_ros_node ${catkin_LIBRARIES})
symlink_catkin_bin(phantom_bag_player_node)

# Camera sync node.
add_executable(camera_force_sync_node src/camera_force_sync_node.cpp)
target_link_libraries(camera_force_sync_node phantom_ros phantom_ros_node)
symlink_catkin_bin(camera_force_sync_node)

add_library(camera_force_sync_nodelet src/camera_force_sync_nodelet.cpp)
target_link_libraries(camera_force_sync_nodelet phantom_ros phantom_ros_node)
symlink_catkin_lib(camera_force_sync_nodelet)

# innovusion parser node.
add_executable(innovusion_parser_node src/innovusion_parser_node.cpp)
target_link_libraries(innovusion_parser_node phantom_ros phantom_ros_node)
symlink_catkin_bin(innovusion_parser_node)

add_library(innovusion_parser_nodelet src/innovusion_parser_nodelet.cpp)
target_link_libraries(innovusion_parser_nodelet phantom_ros phantom_ros_node)
symlink_catkin_lib(innovusion_parser_nodelet)

FILE(GLOB_RECURSE INCLUDE_FILES "*.h" "*.hpp" "*.msg" "*.launch" "*.sh" "*.bmr" "*.srv")
add_custom_target(${PROJECT_NAME}_headers SOURCES ${INCLUDE_FILES})

# Unfortunately there is no automagic CMake variable that tracks all the executables for a project.  We need a list
# like this for all nodes/nodelets affected by the PHANTOM_AI_BUILD_PERCEPTION flag
if (PHANTOM_AI_BUILD_PERCEPTION)
  set(PHANTOM_ROS_PERCEPTION_EXECUTABLES
    point_cloud_motion_compensation_node
    point_cloud_motion_compensation_nodelet
    vision_node2
    vision_nodelet2
    vision_viewer_node2
    vision_viewer_nodelet2
  )
endif()

# Unfortunately there is no automagic CMake variable that tracks all the executables for a project.  We need a list
# like this for the `test_launch_files`, so we will create it here and reuse it to specify the install targets.
set(PHANTOM_ROS_EXECUTABLES
  ab_dynamics_rc_parser_node
  ab_dynamics_rc_parser_nodelet
  ab_dynamics_rc_viz_node
  bandwidth_tester_node
  bandwidth_tester_nodelet  
  camera_force_sync_node
  camera_force_sync_nodelet
  camera_sync_logging_node
  camera_sync_logging_nodelet
  can_node
  can_nodelet
  clpe_image_convert
  clpe_image_convert_nodelet
  conti_ars4g0_parser_node
  conti_ars4g0_parser_nodelet
  conti_ars4g0_viz_node
  conti_mfc4t0_parser_node
  conti_mfc4t0_parser_nodelet
  conti_mfc4t0_viz_node
  conti_srr520_parser_node
  conti_srr520_parser_nodelet
  conti_srr520_viz_node
  conti_vehicle_state_parser_node
  conti_vehicle_state_parser_nodelet
  csi_camera_node
  csi_camera_nodelet
  csi_snapshot_node
  csi_snapshot_nodelet
  delphi_mrr_raw_parser_node
  delphi_mrr_raw_parser_nodelet
  delphi_mrr_raw_preprocessor_viz_node
  delphi_mrr_raw_viz_node
  delphi_mrr_track_parser_node
  delphi_mrr_track_parser_nodelet
  delphi_mrr_track_preprocessor_viz_node
  delphi_mrr_track_viz_node
  delphi_mrr_track_viz_nodelet
  delphi_srr_raw_parser_node
  delphi_srr_raw_parser_nodelet
  delphi_srr_raw_preprocessor_debug_viz_node
  delphi_srr_raw_viz_node
  delphi_srr_raw_viz_nodelet
  ford_message_converter_node
  ford_message_converter_nodelet
  ford_sensor_fusion_viz_node
  ford_sensor_fusion_viz_nodelet
  parking_message_converter_node
  parking_message_converter_nodelet
  health_message_handler
  hmi_viz_node
#  ibeo_node
#  ibeo_viz_node
  image_convert_node
  image_convert_nodelet
  innovusion_parser_node
  innovusion_parser_nodelet
  log_button_monitor_node
  log_server_node
  log_server_nodelet
  log_status_gui_node
  log_status_gui_nodelet
  main_hmi_node
  main_hmi_display_node
  mando_frm_track_parser_node
  mando_frm_track_parser_nodelet
  mando_frm_track_viz_node
  mando_frm_track_viz_nodelet
  mando_lrr_20_fcm_track_parser_node
  mando_lrr_20_fcm_track_parser_nodelet
  mando_lrr_20_fcm_track_viz_node
  mando_lrr_20_fcm_track_viz_nodelet
  mando_fusion_viz_node
  mando_vision_parser_node
  mando_vision_parser_nodelet
  map_server_node
  map_server_nodelet
  mission_control_gui_node
  mission_control_gui_nodelet
  mobileye_eyeq2_parser_node
  mobileye_eyeq2_parser_nodelet
  mobileye_eyeq2_viz_node
  mobileye_eyeq3_parser_node
  mobileye_eyeq3_parser_nodelet
  mobileye_eyeq3_viz_node
  mobis_mobileye_eyeq3_parser_node
  mobis_mobileye_eyeq3_parser_nodelet
  mobis_mobileye_eyeq3_viz_node
  sportage_sensor_parser_node
  sportage_sensor_parser_nodelet
  sportage_sensor_viz_node
  sportage_vehicle_parser_node
  sportage_vehicle_parser_nodelet
  network_io_node
  network_io_nodelet
  nmea_gps_node
  note_server_node
  note_server_nodelet
  nv_image_view_node
  nv_image_view_nodelet
  nvsipl_camera_node
  nvsipl_camera_nodelet
  oxts_node
  oxts_nodelet
  genesys_adma_gps_parser_node
  genesys_adma_gps_parser_nodelet
  phantom_bag_player_node
  phantom_bag_player_nodelet
  phantom_ros
  phantom_ros_node
  phantom_ros_nodelet_manager
  phantom_vision_viz_node
  phantom_vision_viz_nodelet
  phantom_lidar_viz_node
  phantom_lidar_viz_nodelet
  process_supervisor_node
  process_supervisor_nodelet
  rviz_helpers
  sensor_fusion_node
  sensor_fusion_nodelet
  sensor_fusion_viz_node
  sensor_fusion_viz_nodelet
  supervisor_node
  supervisor_nodelet
  embedded_capture_node
  embedded_capture_nodelet
  embedded_streamer_node
  embedded_streamer_nodelet
  phantom_rosbag_converter_node
  phantom_rosbag_converter_nodelet
  tf_node
  ti_video_capture_node
  ti_video_capture_nodelet
  ti_video_streamer_node
  ti_video_streamer_nodelet
  ti_viz_node
  ti_viz_nodelet
  time_sender_node
  time_sender_nodelet
  traffic_light_handler_node
  traffic_light_handler_nodelet
  ums_gateway_node
  vehicle_state_parser_node
  vehicle_state_parser_nodelet
  paccar_vehicle_state_parser_node
  paccar_vehicle_state_parser_nodelet
  ford_vehicle_state_parser_node
  ford_vehicle_state_parser_nodelet
  paccar_message_converter_node
  paccar_message_converter_nodelet
  paccar_viz_node
  paccar_viz_nodelet
  vehicle_state_provider_node
  vehicle_state_provider_nodelet
  velodyne_parser_node
  velodyne_parser_nodelet
  node_manager_node
  node_manager_nodelet
  exodus_front_sensor_parser_node
  exodus_front_sensor_parser_nodelet
  exodus_front_sensor_viz_node
  mando_lrr_25_frm_parser_node
  mando_lrr_25_frm_parser_nodelet
  mando_lrr_25_frm_viz_node
  ${PHANTOM_ROS_PERCEPTION_EXECUTABLES}
)

## Mark executables and/or libraries for installation.
install(
  TARGETS ${PHANTOM_ROS_EXECUTABLES}
  RUNTIME DESTINATION ${PHANTOM_OS_INSTALL_BIN_DIR}
  LIBRARY DESTINATION ${PHANTOM_OS_INSTALL_LIB_DIR}
)

## Copy launch files (both .launch and related .sh).
install(DIRECTORY launch/
  DESTINATION ${PHANTOM_OS_INSTALL_SHARE_DIR}/launch
)

# Copy Nodelet XML file.
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/nodelets.xml
  DESTINATION ${PHANTOM_OS_INSTALL_SHARE_DIR}
)

# Add the simulation stubs.
add_subdirectory(simulation)

# Add the test directory.
if(PHANTOM_OS_TEST)
  # We have to manually append the 3rd party ROS nodes to our list of executables.
  list(APPEND PHANTOM_ROS_EXECUTABLES "usb_cam_node" "image_view")
  add_subdirectory(test)
endif()

if(PHANTOM_AI_CONFIG_SOCKETCAN)
  # Force run SocketCAN config script if CAN code changes
  add_custom_command(TARGET can_nodelet POST_BUILD
      COMMAND /bin/sh ${CMAKE_SOURCE_DIR}/scripts/utilities/setcap_can_node.sh
  )
endif()
