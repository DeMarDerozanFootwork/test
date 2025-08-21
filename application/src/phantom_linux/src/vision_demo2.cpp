/*******************************************************************************
* @file    vision_demo2.cpp
* @date    03/30/2021
*
* @attention Copyright (c) 2021
* @attention Phantom AI, Inc.
* @attention All rights reserved.
*******************************************************************************/
#include <optional>
#include "phantom_ai/core/yaml.h"
#include "phantom_ai/core/log.h"
#include "phantom_ai/core/memory.h"
#include "phantom_ai/core/signal.h"
#include "phantom_ai/core/thread.h"
#include "phantom_ai/core/paths.h"

#include "phantom_ai/zf_vehicle_state_parser/zf_vehicle_state_parser.h"
#include "phantom_ai/vehicle_state_parser/vehicle_state_parser.h"

#include "tidl_inferencer.h"
#include "phantom_ai/vision_can_message_pack/can_message_pack.h"
#include "phantom_ai/wrappers/vision_wrapper2.h"
#include "phantom_ai/wrappers/camera_wrapper.h"
#include "phantom_ai/wrappers/can_wrapper.h"
#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include "phantom_linux_common.h"

#include "phantom_ai/functional_safety/functional_safety_task.h"

#include "phantom_ai/utils/utils.h"
#include "phantom_ai/core/timer.h"

#include "phantom_ai/wrappers/network_tx_wrapper.h"
#include <phantom_ai/test/test_log.hpp>
#include <phantom_ai/common/camera_info.h>

#include "system_mode.h"
#include "utils.h"

#include <condition_variable>

#if defined(AUTOBRAIN_VH_SPECIFIC_APP)
#include "phantom_ai/wrappers/protobuf_wrapper.h" // replace CAN with protobuf msgs
#include "phantom_ai/protobuf_msgs/ab_main_state_machine.h"
#include "ab_api.h"
#include <phantom_ai/core/sim_time_stamp.h>
#include <phantom_ai/core/time_stamp.h>
#endif

#if defined(ENABLE_ECU_KEY_MANAGER)
  #include "phantom_ai/wrappers/ecu_key_manager_wrapper.h"
#endif // ENABLE_ECU_KEY_MANAGER
#if (ENABLE_ZEROMQ)
  #include "phantom_ai/wrappers/zmq_wrapper.h"
#endif
#if defined(ENABLE_GSTREAMER)
  #include <glib.h>
  #include <gst/gst.h>
  #include "phantom_gst.h"
extern "C" {
  #include "gst_stream.h"
  #include "multi_cam_tivx.h"
}
#endif
extern "C" {
  #include "gst_stream.h"
  #include "multi_cam_tivx.h"
}

static constexpr int MAX_TIMEOUT = 100000; //timeout maximum in minutes

/*
Test code for dynamic crop contorl.
*** Please remove this test code once the actual implementation is complete. ***
JIRA: https://phantomai.atlassian.net/browse/MAS-1030
*/

//#define TEST_DYNAMIC_CROP_ARROW_KEY_CONTROL

#ifdef TEST_DYNAMIC_CROP_ARROW_KEY_CONTROL
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#endif

int max_diff_ms_ = DEVIATION_RANGE_IN_MILLISEC;
bool debug_mode_{};

namespace phantom_linux
{
  typedef struct
  {
    std::unordered_multimap<phantom_ai::CameraID, cv::Mat> img_obj;
    double timestamp;
    uint32_t frame_count;
  } NetworkImageObj;

  class VisionDemo2
  {
  public:
    VisionDemo2(int argc, char *argv[]) :
      main_loop_timeout_status_(true),
      system_tick_{},
      functional_safety_task_{nullptr},
      can_packer_{nullptr},
      vision_wrapper2_(nullptr),
      tidl_inferencer_(nullptr),
      camera_wrapper_(nullptr),
#if defined(ENABLE_PROTOBUF_MSG)
      pb_wrapper_(nullptr),
#elif (ENABLE_ZEROMQ)
      zmq_wrapper_(nullptr),
#endif
      can_bus0_wrapper_(nullptr),
      can_bus1_wrapper_(nullptr),
#if defined(ENABLE_ECU_KEY_MANAGER)
      ecu_key_manager_wrapper_(nullptr),
#endif // ENABLE_ECU_KEY_MANAGER
      vehicle_state_(nullptr),
      cnt_frame_vision_track_task_underrun_(0),
      t_image_last_(0.0),
      cameras_data_frame_index_(0),
      net_tx_wrapper_(nullptr),
#if defined(ENABLE_VISION_LOGGING)
      vision_data_{},
      dynm_calib_data_{},
      veh_data_{},
      can_data_{},
      fusa_data_{},
      camera_extra_data_{},
      hba_data_{},
      aeb_data_{},
#endif // ENABLE_VISION_LOGGING
      enable_vision_can_output_(true),
      is_ready_to_receive_switch_mode_command_(false)
    {
      options_ = ArgHandler(argc, argv);
      Initialize();
    }

    ~VisionDemo2()
    {
      if (true == system_mode_.check_precondition(SystemMode::Mode::EXIT))
      {
        system_mode_.set_current_mode(SystemMode::Mode::EXIT);
      }
      else
      {
        PHANTOM_ERROR("Could not exit system state machine");
      }

      camera_wrapper_->stopStreaming();

#if defined(ENABLE_VISION_LOGGING)
      quit_loop_ = true;

      //kick any waiting threads
      net_tx_loop_cv_.notify_all();

      if (net_tx_thread_.joinable())
      {
        net_tx_thread_.join();
      }
#endif
#if defined(ENABLE_GSTREAMER)
      if(encodeH264_send_by_gst_init_tried){
        encodeH264_send_by_gst_deinit();
      }
#endif
#ifdef TEST_DYNAMIC_CROP_ARROW_KEY_CONTROL
    test_dynamic_crop_quit_loop_ = true;
    if (test_dynamic_crop_thread_.joinable())
    {
        test_dynamic_crop_thread_.join();
    }
#endif

#if defined(USE_CAM_PARAM_INPUT_FOR_VISION_INIT)
      PHANTOM_LOG("saveDynamicCalibrationOutputToFile");
      saveDynamicCalibrationOutputToFile();
#endif  // USE_CAM_PARAM_INPUT_FOR_VISION_INIT

      // Explicitly destroy pointers, since it seems it doesn't happen automatically...
      // Do sources first
#if defined(ENABLE_PROTOBUF_MSG)
      delete pb_wrapper_;
      timestamp_jump_detect_stop_ = true;
      if ( timestamp_jump_detect_thread_.joinable())
        timestamp_jump_detect_thread_.join();
#endif
#if defined(ENABLE_ECU_KEY_MANAGER)
      delete ecu_key_manager_wrapper_;
#endif // ENABLE_ECU_KEY_MANAGER
#if (ENABLE_ZEROMQ)
      delete zmq_wrapper_;
#endif
      // everything else
      delete functional_safety_task_;
      delete tidl_inferencer_;
      delete vision_wrapper2_;
      delete vehicle_state_;
      if(net_tx_wrapper_)
      {
        delete net_tx_wrapper_;
      }
#if defined(ENABLE_GSTREAMER)
      if(gst_h264encoding_enabled_)
      {
        encodeH264_gst_deinit();
      }
#endif
      // delete TI hardware resource related component at last to execute "TI app deinitialization function" at last.
      delete camera_wrapper_;

#if !defined(ENABLE_PROTOBUF_MSG)
      delete can_packer_;
      delete can_bus0_wrapper_;
      delete can_bus1_wrapper_;
#endif
    }

#if defined(AUTOBRAIN_VH_SPECIFIC_APP)  && defined(ENABLE_PROTOBUF_MSG)
    phantom_ai::TimeStamp autobrainTimeStamp()
    {
      phantom_ai::TimeStamp ts(1e-6 * OsalTimer_getUs());
      return ts;
    }
#endif


#ifdef TEST_DYNAMIC_CROP_ARROW_KEY_CONTROL
void testDynamicCropArrowKeyLoop()
{
    PHANTOM_LOG("Starting Dynamic Crop Arrow Key Test");
    PHANTOM_LOG("Controls:");
    PHANTOM_LOG("  Arrow Keys: Move crop position");
    PHANTOM_LOG("  '+' or '=': Increase step size");
    PHANTOM_LOG("  '-': Decrease step size");
    PHANTOM_LOG("  'c': Switch cameraID");
    PHANTOM_LOG("  'f': Switch function (setPosition/moveRelative)");

    std::vector<phantom_ai::CameraID> test_cameras = {
        phantom_ai::CAM_FRONT_CENTER_CROP,
        phantom_ai::CAM_FRONT_CENTER
    };

    bool use_set_position = true; // true: setPosition, false: moveRelative
    int32_t move_step = 2;

    int cur_camid_idx = 0;
    phantom_ai::CameraID curr_camid = test_cameras[cur_camid_idx];
    PHANTOM_LOG("Dynamic crop enabled for camera: {}", static_cast<int>(curr_camid));

    // Print status function
    auto printStatus = [&]() {
        std::string function_name = use_set_position ? "setPosition" : "moveRelative";

        uint32_t current_x, current_y;
        if (camera_wrapper_->dynamicCropGetPosition(curr_camid, current_x, current_y))
        {
            bool enabled;
            phantom_ai::hal_camera_image cam_info;
            camera_wrapper_->dynamicCropGetInfo(curr_camid, enabled, cam_info);

            PHANTOM_LOG("=== Dynamic Crop Status ===");
            PHANTOM_LOG("Camera: {}", phantom_ai::camera_name(curr_camid));
            PHANTOM_LOG("Function: {}", function_name);
            PHANTOM_LOG("Step Size: {} pixels", move_step);
            PHANTOM_LOG("Current Position: left={}, top={}, right={}, bottom={}", current_x, current_y,  current_x+cam_info.roi.width, current_y+cam_info.roi.height);
            PHANTOM_LOG("========================");
        }
        else
        {
            PHANTOM_ERROR("Failed to get current position");
        }
    };

    printStatus();

    while (!test_dynamic_crop_quit_loop_)
    {
        char input = 0;
        // Init terminal keyinput settings
        struct termios old_termios, new_termios;
        tcgetattr(STDIN_FILENO, &old_termios);
        new_termios = old_termios;
        new_termios.c_lflag &= ~(ICANON | ECHO);
        new_termios.c_cc[VMIN] = 0;
        new_termios.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
        // Read a character
        ssize_t bytes_read = read(STDIN_FILENO, &input, 1);
        // Restore terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &old_termios);

        // Handle arrow keys
        if (input == 27)
        {
            char seq[3];
            if (read(STDIN_FILENO, &seq[0], 1) == 1 && seq[0] == '[')
            {
                if (read(STDIN_FILENO, &seq[1], 1) == 1)
                {
                    switch (seq[1])
                    {
                        case 'A': input = 'w'; break; // Up arrow
                        case 'B': input = 's'; break; // Down arrow
                        case 'C': input = 'd'; break; // Right arrow
                        case 'D': input = 'a'; break; // Left arrow
                    }
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(40));

        if (bytes_read > 0)
        {
            uint32_t current_x, current_y;
            bool position_changed = false;
            bool result ;
            switch (input)
            {
                case 'w': // Up arrow
                    if (use_set_position)
                    {
                        if (camera_wrapper_->dynamicCropGetPosition(curr_camid, current_x, current_y))
                        {
                            uint32_t new_y = (current_y >= move_step) ? current_y - move_step : 0;
                            result = camera_wrapper_->dynamicCropSetPosition(curr_camid, current_x, new_y);
                        }
                    }
                    else
                        result = camera_wrapper_->dynamicCropMoveRelative(curr_camid, 0, -move_step);

                    position_changed = true;
                    PHANTOM_LOG("[{}] Move UP (step={})",phantom_ai::camera_name(curr_camid), move_step);
                    break;

                case 's': // Down arrow
                    if (use_set_position)
                    {
                        if (camera_wrapper_->dynamicCropGetPosition(curr_camid, current_x, current_y))
                        {
                            uint32_t new_y = current_y + move_step;
                            result = camera_wrapper_->dynamicCropSetPosition(curr_camid, current_x, new_y);
                        }
                    }
                    else
                        result = camera_wrapper_->dynamicCropMoveRelative(curr_camid, 0, move_step);

                    position_changed = true;
                    PHANTOM_LOG("[{}] Move DOWN (step={})",phantom_ai::camera_name(curr_camid),move_step);
                    break;

                case 'a': // Left arrow
                    if (use_set_position)
                    {
                        if (camera_wrapper_->dynamicCropGetPosition(curr_camid, current_x, current_y))
                        {
                            uint32_t new_x = (current_x >= move_step) ? current_x - move_step : 0;
                            result = camera_wrapper_->dynamicCropSetPosition(curr_camid, new_x, current_y);
                        }
                    }
                    else
                        result = camera_wrapper_->dynamicCropMoveRelative(curr_camid, -move_step, 0);

                    position_changed = true;
                    PHANTOM_LOG("[{}] Move LEFT (step={})",phantom_ai::camera_name(curr_camid),move_step);
                    break;

                case 'd': // Right arrow
                    if (use_set_position)
                    {
                        if (camera_wrapper_->dynamicCropGetPosition(curr_camid, current_x, current_y))
                        {
                            uint32_t new_x = current_x + move_step;
                            result = camera_wrapper_->dynamicCropSetPosition(curr_camid, new_x, current_y);
                        }
                    }
                    else
                        result = camera_wrapper_->dynamicCropMoveRelative(curr_camid, move_step, 0);

                    position_changed = true;
                    PHANTOM_LOG("[{}] Move RIGHT (step={})",phantom_ai::camera_name(curr_camid),move_step);
                    break;

                case 'c': // Switch camera ID
                    cur_camid_idx = (cur_camid_idx + 1) % test_cameras.size();
                    curr_camid = test_cameras[cur_camid_idx];
                    PHANTOM_LOG("Switched to {}", phantom_ai::camera_name(curr_camid));

                    printStatus();
                    break;

                case 'f': // Switch function
                    use_set_position = !use_set_position;
                    PHANTOM_LOG("Switched to function: {}", use_set_position ? "setPosition" : "moveRelative");
                    printStatus();
                    break;

                case '+': // Increase step size
                case '=':
                    move_step = std::min(move_step * 2, 64);
                    PHANTOM_LOG("Step size increased to: {} pixels", move_step);
                    break;

                case '-': // Decrease step size
                    move_step = std::max(move_step / 2, 1);
                    PHANTOM_LOG("Step size decreased to: {} pixels", move_step);
                    break;

                default:
                    break;
            }

            if (position_changed)
            {
                if (result)
                {
                    // Wait for next frame to get updated position (67ms for 15fps)
                    std::this_thread::sleep_for(std::chrono::milliseconds(67));

                    if (camera_wrapper_->dynamicCropGetPosition(curr_camid, current_x, current_y))
                    {
                      bool enabled;
                      phantom_ai::hal_camera_image cam_info;
                      camera_wrapper_->dynamicCropGetInfo(curr_camid, enabled, cam_info);

                      PHANTOM_LOG("Current Position: left={}, top={}, right={}, bottom={}", current_x, current_y,  current_x+cam_info.roi.width, current_y+cam_info.roi.height);
                    }
                }
                else
                    PHANTOM_ERROR("Failed to move crop position");
            }
        }
    }
}
#endif
    void Initialize()
    {
#if defined(AUTOBRAIN_VH_SPECIFIC_APP) && defined(ENABLE_PROTOBUF_MSG)
      phantom_ai::SimTimeStamp::registerExternalTimestampSource(
          [this]() -> phantom_ai::TimeStamp {
            return this->autobrainTimeStamp();
          });
      Perform6MonthCheck();
#endif
      // @brief create functional_safety_task_
      functional_safety_task_ = new phantom_ai::FunctionalSafetyTask();

      //* Initialize VisionWrapper2
      vision_cfg_nodename_ = (options_.is_vision_param_set_) ? options_.vision_param_path_:
                                                              "perception/" + std::string(PHANTOM_VISION2_LIB_NAME) + "/tda4x";
      vision_cfg_filename_ = (options_.is_vision_param_set_) ? options_.vision_param_name_:
                                                                get_vision_config_filename(options_.image_capture_machine_name_);
      phantom_ai::YamlNode root_node = phantom_ai::load_yaml_file(vision_cfg_nodename_, vision_cfg_filename_);
      phantom_ai::read_vision_params(root_node, vision_params_);
      can_packer_ = new phantom_ai::CanMessagePacker();
      can_packer_->init([this](const phantom_ai::CanFrameList &frames) {
            this->canCallback(frames);},
           vision_params_.message_task.enable_face_bounding_angle,
           (vision_params_.message_task.verbosity & VERBOSITY_MESSAGE_CAN_PACKER) ? 3 : 2,
           vision_params_.message_task.can_max_object_counts);
      vision_wrapper2_ = new phantom_ai::VisionWrapper2();

#if defined(USE_CAM_PARAM_INPUT_FOR_VISION_INIT)
      dynamic_calibration_param_foldername_ = phantom_ai::phantom_paths_param_directory() + "/sensors/camera";
      dynamic_calibration_param_filename_ = get_dynamic_calibration_param_filename(dynamic_calibration_param_foldername_);
      if (dynamic_calibration_param_filename_.size() > 0)
      {
        phantom_ai::PhantomVisionDynamicCalibrationInput dynm_cal_input;
        readDynamicCalibrationInputFile(dynm_cal_input);
        vision_wrapper2_->onInit(vision_cfg_nodename_, vision_cfg_filename_, dynm_cal_input);
        PHANTOM_LOG("Dynamic calibration input file: {}", dynamic_calibration_param_filename_);
      }
      else
      {
        vision_wrapper2_->onInit(vision_cfg_nodename_, vision_cfg_filename_);
        setDummyExtrinsicCameraParameters();
        PHANTOM_LOG("No dynamic calibration input file exists. Use parameters from camera calibration file.");
      }
#else   // USE_CAM_PARAM_INPUT_FOR_VISION_INIT
      vision_wrapper2_->onInit(vision_cfg_nodename_, vision_cfg_filename_);
#endif  // USE_CAM_PARAM_INPUT_FOR_VISION_INIT
      vision_wrapper2_->registerCanPackCallback(
          [this](const phantom_ai::PhantomVisionMeasurement &vision_msmt) {
            this->can_packer_->receiveData(vision_msmt);
          });
      vision_wrapper2_->RegisterAebMsgCanPackCallback(
          [this](const phantom_ai::PhantomVisionAEB &aeb_data) {
            this->can_packer_->receiveAebData(aeb_data);
          });
      vision_wrapper2_->RegisterHbaMsgCanPackCallback(
          [this](const phantom_ai::PhantomVisionHBA &hba_data) {
            this->can_packer_->receiveHbaData(hba_data);
          });
      vision_wrapper2_->RegisterDynCalibMsgCanPackCallback(
          [this](const phantom_ai::PhantomVisionDynamicCalibrationOutputList &dynCalib_data) {
            this->can_packer_->receiveDynCalibData(dynCalib_data);
          });

#if defined(ENABLE_VISION_LOGGING) || defined(ENABLE_PROTOBUF_MSG)
      vision_wrapper2_->registerPhantomVisionMeasurementCallback(
          [this](const phantom_ai::PhantomVisionMeasurement &input) {
            this->publishPhantomVisionMeasurement(input);
          });

      vision_wrapper2_->registerHbaCallback(
          [this](const phantom_ai::PhantomVisionHBA &input) {
            this->publishHba(input);
          });
      vision_wrapper2_->registerAebCallback(
          [this](const phantom_ai::PhantomVisionAEB &input) {
            this->publishAeb(input);
          });
      vision_wrapper2_->registerFuSaCallback(
          [this](const phantom_ai::CameraMotionFuSaData& input) {
            functional_safety_task_->isCameraMotionDataInvalidCallback(input);
          });
      vision_wrapper2_->registerDynamicCalibrationOutputCallback(
          [this](const phantom_ai::PhantomVisionDynamicCalibrationOutputList &input) {
            this->publishDynamicCalibrationOutput(input);
          });
#endif

#if defined(ENABLE_PROTOBUF_MSG)
      //* Initialize ProtobufWrapper
      // this replaces vehicle_state_ and can_bus0_wrapper_
      pb_wrapper_ = new phantom_ai::ProtobufWrapper();
      pb_wrapper_->onInit();
      pb_wrapper_->registerWheelSpeedMeasurementsCallback(
          [this](const phantom_ai::WheelSpeedMeasurements& wsm) {
            this->wheelSpeedCallback(wsm);
          });
      pb_wrapper_->registerEspMeasurementsCallback(
          [this](const phantom_ai::EspMeasurements &esp) {
            this->espCallback(esp);
          });
      pb_wrapper_->registerSwitchModeCommandCallback(
          [this](const SwitchModeCommand &switch_mode_command) {
            this->switchModeCommandCallback(switch_mode_command);
          });
      pb_wrapper_->registerDynamicCalibrationInputCallback(
          [this](phantom_ai::PhantomVisionDynamicCalibrationInput &input) {
            this->dynamicCalibrationInputCallback(input);
          });
      pb_wrapper_->registerIPCErrorNotifyCallback(
          [this](int setFlag, int clearFlag) {
           this->ipcErrorNotify(setFlag, clearFlag);
	    (void)setFlag;
	    (void)clearFlag;
          });

      timestamp_jump_detect_thread_ = std::thread(&VisionDemo2::timeStampJumpDetect, this);
#else // ENABLE_PROTOBUF_MSG
      //* Initialize VehicleStateParser
  #ifdef USE_ZF_VEHICLE_STATES
      vehicle_state_ = new phantom_ai::ZFVehicleStateParser();
  #else
      vehicle_state_ = new phantom_ai::VehicleStateParser();
  #endif
      vehicle_state_->registerWheelSpeedMeasurementsParserCallback(
          [this](const phantom_ai::WheelSpeedMeasurements &wsm) {
            this->wheelSpeedCallback(wsm);
          });
      vehicle_state_->registerEspMeasurementsParserCallback(
          [this](const phantom_ai::EspMeasurements &esp) {
            this->espCallback(esp);
          });
      vehicle_state_->registerGearStateParserCallback(
          [this](const phantom_ai::GearState &gear_state) {
            this->gearStateCallback(gear_state);
          });
      vehicle_state_->registerSteeringAngleParserCallback(
          [this](const phantom_ai::SteeringAngle &steer_angle) {
            this->steerAngleCallback(steer_angle);
          });
      double epoch_offset = 0.0;
#if (ENABLE_ZEROMQ)
      zmq_wrapper_ = new phantom_ai::ZmqWrapper();
      zmq_wrapper_->onInit("zeromq_config.yaml");
      std::string vehicle_state_source = zmq_wrapper_->getVehicleStateSource();
      if (vehicle_state_source == "zmq"){
        zmq_wrapper_->registerWheelSpeedMeasurementsCallback(
          [this](const phantom_ai::WheelSpeedMeasurements& wsm) {
            this->wheelSpeedCallback(wsm);
          });
        zmq_wrapper_->registerEspMeasurementsCallback(
          [this](const phantom_ai::EspMeasurements &esp) {
            this->espCallback(esp);
          });
      } else{
        #if defined(SOC_J721S2) || defined(SOC_J721E)
        can_bus0_wrapper_ = new phantom_ai::CanWrapper("vehicle_can_j721s2_j721e.yaml");
        #else
        can_bus0_wrapper_ = new phantom_ai::CanWrapper("vehicle_can.yaml");
        #endif
        can_bus0_wrapper_->registerRxCallback(
          [this](const phantom_ai::CanFrame &frame) {
            this->rxParserCallback(frame);
          });
        std::vector<struct can_filter> can_filter_list = SetVehicleCANMessageFilter();
        can_bus0_wrapper_->setMessageFilter(&can_filter_list, true);
        can_bus0_wrapper_->setTimestampOffset(epoch_offset);
      }
#else
      //* Initialize CanWrapper
      #if defined(SOC_J721S2) || defined(SOC_J721E)
      can_bus0_wrapper_ = new phantom_ai::CanWrapper("vehicle_can_j721s2_j721e.yaml");
      #else
      can_bus0_wrapper_ = new phantom_ai::CanWrapper("vehicle_can.yaml");
      #endif
      can_bus0_wrapper_->registerRxCallback(
          [this](const phantom_ai::CanFrame &frame) {
            this->rxParserCallback(frame);
          });
      std::vector<struct can_filter> can_filter_list = SetVehicleCANMessageFilter();
      can_bus0_wrapper_->setMessageFilter(&can_filter_list, true);
      can_bus0_wrapper_->setTimestampOffset(epoch_offset);
#endif
      #if defined(SOC_J721S2) || defined(SOC_J721E)
      can_bus1_wrapper_ = new phantom_ai::CanWrapper("private_can_j721s2_j721e.yaml");
      #else
      can_bus1_wrapper_ = new phantom_ai::CanWrapper("private_can.yaml");
      #endif
      can_bus1_wrapper_->setTimestampOffset(epoch_offset);
#endif // ENABLE_PROTOBUF_MSG

#if defined(ENABLE_ECU_KEY_MANAGER)
      ecu_key_manager_wrapper_ = new phantom_ai::EcuKeyManagerWrapper();

      if(false == ecu_key_manager_wrapper_ -> checkEcuKey())
      {
        throw VisionException("ECU KEY WRONG");
      }
      else
      {
        PHANTOM_LOG("-------------------ECU KEY CORRECT-------------------");
      }
#endif // ENABLE_ECU_KEY_MANAGER

      auto camera_models = vision_wrapper2_->camera_models();
      cameras_crop_info_ = setCropInformationForCapturedImages(camera_models,
                                                               vision_wrapper2_->params().cameras_active,
                                                               vision_wrapper2_->params().camera_image_size);

      //* Initialize CameraWrapper
      std::string camera_cfg_filename = get_camera_config_filename();
      camera_wrapper_ = new phantom_ai::CameraWrapper();

#if defined(ENABLE_FULLRES_IMG_LOGGING)
      bool use_fullres_logging = true;
#else
      bool use_fullres_logging = false;
#endif
      camera_wrapper_->onInit(camera_cfg_filename, vision_wrapper2_->camera_models(), vision_wrapper2_->params().target_system, use_fullres_logging);
      camera_wrapper_->registerImageCallback(
          [this](std::vector<phantom_ai::CameraResults> &image_arr, const std::unordered_map<phantom_ai::CameraID, std::shared_ptr<phantom_ai::CameraExtraInfo>>& extra_info) {
            this->imageCallback(image_arr, extra_info);
          });

      //* Setup FuSa
      functional_safety_task_->onInit();
      functional_safety_task_->registerFuSaPublishViaCAN(
          [this](const phantom_ai::FunctionalSafetyOutput& functional_safety_output) {
            this->can_packer_->receiveFusaData(functional_safety_output);
          });
      functional_safety_task_->registerFuSaPublishViaNetwork(
          [this]([[maybe_unused]] const phantom_ai::FunctionalSafetyOutput& functional_safety_output)
          {
#if defined(ENABLE_VISION_LOGGING)
            this->publishFunctionalSafetyOutputToLoggingPC(functional_safety_output);
#endif
#if defined(ENABLE_PROTOBUF_MSG)
            this->publishFunctionalSafetyOutputToIPC(functional_safety_output);
#endif
            UNUSED(functional_safety_output);
          });
      functional_safety_task_->registerFuSaDebugVisualize(
          [this](const phantom_ai::FunctionalSafetyOutput& functional_safety_output) {
            this->vision_wrapper2_->VisualizeFuSaOutput(functional_safety_output);
          });

      if(options_.tx_debug_image_)
      {
        vision_wrapper2_->registerCallbackVisualizerImgPublish(
            [this](const cv::Mat &input, double) {
              this->processVisionViz(input);
            });
      }
#if defined(ENABLE_GSTREAMER)
      else if(options_.tx_gst_viz_)
      {
        vision_wrapper2_->registerCallbackVisualizerImgPublish(
            [this](const cv::Mat &input, double) {
              this->processGstViz(input);
            });
      }
#endif
#if defined(ENABLE_GSTREAMER)
    PHANTOM_ERROR(" ++++++++++  src/phantom_linux/src/vision_demo2.cpp  ENABLE_GSTREAMER ON ");//WOOSEOK
#else
    PHANTOM_ERROR(" ----------  src/phantom_linux/src/vision_demo2.cpp  ENABLE_GSTREAMER OFF ");//WOOSEOK
#endif
      //* Initialize TidlInferencer
    #if defined (SOC_AM62A) || defined(SOC_J722S) || defined(SOC_J721S2) || defined(SOC_J721E)
      tidl_inferencer_ = new phantom_ai::TidlInferencer(
                                                          camera_models.get(),
                                                          vision_wrapper2_->params().cameras_active,
                                                          options_.use_fullres_image_for_tsc_,
                                                          vision_wrapper2_->params().phantomnet_task.tsr_classifier_max_box_size_to_use_fullres_image
                                                        );
    #else
      tidl_inferencer_ = new phantom_ai::TidlInferencer(camera_models.get(), vision_wrapper2_->params().cameras_active);
    #endif
      if(tidl_inferencer_)
      {
        tidl_inferencer_->setClassifications(
          vision_wrapper2_->populateClassification(
            tidl_inferencer_->getClassificationNames()));
        tidl_inferencer_->onInit();
        tidl_inferencer_->registerTidlCallback(
            [this](std::shared_ptr<phantom_ai::TidlOutput> output) {
              this->tidlCallback(output);
            });
        // Register getFullResolutionImageROI callback function
        tidl_inferencer_->registerGetFullResolutionImageROI(
            [this](phantom_ai::CameraID cam_id, const cv::Rect &cam_crop_roi, uint32_t frame_number, const cv::Rect& resized_target_roi, phantom_ai::PixelFormat fmt) -> std::optional<phantom_ai::FrameData> {
                return this->camera_wrapper_->getFullResolutionImageROI(cam_id, cam_crop_roi, frame_number, resized_target_roi, fmt);
            });
        tidl_inferencer_->registerRunningFlagger(
            [this](bool running_flag) { this->setRunningFlag(running_flag); });

        auto critter = tidl_inferencer_->tidl_critical_section();
        vision_wrapper2_->set_critical_section(critter);
      }

      // vision_wrapper2_, camera_wrapper_, tidl_inferencer_ are initialized.

      quit_loop_ = false;

      // #PROC Intialize framerate arguments for debug visualizer
      viz_fixed_framerate_ = options_.viz_fixed_framerate_;

      if (viz_fixed_framerate_)
      {
        int viz_framerate_hz = options_.viz_framerate_;
        viz_framerate_ms_ = 1000.0f / viz_framerate_hz;
      }
      // #END
#if defined(ENABLE_VISION_LOGGING)
      net_tx_wrapper_ = new phantom_ai::NetTxWrapper();
#if defined(ENABLE_GSTREAMER)
//* Initialize Image encoding by Gst for logging images

    PHANTOM_ERROR(" ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ ON ");//WOOSEOK

      //camera_wrapper_->initGstEncoder(net_tx_wrapper_);
    phantom_ai::YamlNode gst_cfg = phantom_ai::load_yaml_file("sensors/embedded_network", "gst_config.yaml");
    gst_h264encoding_enabled_ = phantom_ai::get_yaml_key<bool>(gst_cfg, "encoding_image_by_h24", false);

    if(gst_h264encoding_enabled_)
    {

        uint32_t mosaic_width = 0, mosaic_height = 0;
        camera_wrapper_->getMosaicDimensions(mosaic_width, mosaic_height);

        std::string gst_pipeline_str = phantom_ai::get_yaml_key<std::string>(gst_cfg, "h264_pipeline_string");

        bool gst_init_status = encodeH264_gst_init(
                                gst_pipeline_str.c_str(),
                                gst_h264encoding_enabled_,
                                15, // framerate
                                mosaic_width,
                                mosaic_height);
#if 0
        if(gst_init_status) {
          PHANTOM_WARNING("NetTxWrapper: GStreamer H.264 encoder initialized successfully.");
          this->enableEncodeImgByGst(true);
          this->registerEncodingMosaicImg(&get_encoded_imgMosaicObj);
          mosaicImg_width_ = width;
          mosaicImg_height_ = height;
          return true;
        }
#endif
        if (gst_init_status) {
            PHANTOM_LOG("TI SDK GStreamer encoder initialized successfully.");
            net_tx_wrapper_->enableEncodeImgByGst(true);
            net_tx_wrapper_->registerEncodingMosaicImg(&get_encoded_imgMosaicObj);
            camera_wrapper_->registerEncoderCallback(&encodeH264_gst_feed_frame);
        } else {
            PHANTOM_ERROR("Failed to initialize TI SDK GStreamer encoder. Disabling feature.");
            gst_h264encoding_enabled_ = false;
        }
    }

    PHANTOM_ERROR("NetTxWrapper: Failed to initialize GStreamer encoder.");
    gst_h264encoding_enabled_ = false;
#endif
//* Initialize NetTxWrapper
      if (use_fullres_logging) net_tx_wrapper_->enableFullresLogging();
      net_tx_wrapper_->onInit("tda4x/stream_server.yaml", camera_cfg_filename);

      phantom_ai::CameraInfoList& cam_info_list = camera_wrapper_->getCameraInfoList();
      EmbeddedNet_CameraCalibration encc_info;
      convertCameraInfo(cam_info_list, encc_info);
      net_tx_wrapper_->queueCameraCalib(encc_info);

      net_tx_thread_ = std::thread(&VisionDemo2::networkLoop, this);
      auto handle = net_tx_thread_.native_handle();
      pthread_setname_np(handle, "NetTx");
#else
      if (options_.tx_debug_image_)
      {
        net_tx_wrapper_ = new phantom_ai::NetTxWrapper();
        net_tx_wrapper_->onInit("tda4x/stream_server.yaml", camera_cfg_filename);
      }
#endif

#ifdef TEST_DYNAMIC_CROP_ARROW_KEY_CONTROL
    test_dynamic_crop_thread_ = std::thread([this]() {this->testDynamicCropArrowKeyLoop();});
    auto thread_handle = test_dynamic_crop_thread_.native_handle();
    pthread_setname_np(thread_handle, "TestDynCropControl");
    test_dynamic_crop_quit_loop_ = false;
#endif

      // everything configured, start feeding frames
      camera_wrapper_->startStreaming();

      SystemMode::Mode initial_mode = SystemMode::Mode::NORMAL_DRIVING;
      if (true == system_mode_.check_precondition(initial_mode))
      {
        system_mode_.set_current_mode(initial_mode);
        switch_perception_mode(initial_mode);
        is_ready_to_receive_switch_mode_command_ = true;
      }
      else
      {
        PHANTOM_ERROR("System state machine can't be started");
      }
    }

    std::vector<struct can_filter> SetVehicleCANMessageFilter()
    {
      std::vector<struct can_filter> vehicle_can_filter_list;
      constexpr uint8_t kNumberOfCANRxBufferFromVehicleCAN = 4; // L_ESP12, L_SAS11, and L_WHL_SPD11
      vehicle_can_filter_list.reserve(kNumberOfCANRxBufferFromVehicleCAN);
      can_filter can_filter_temp;

      // use single frame filter for all Rx messages on vehicle CAN
      can_filter_temp.can_mask = CAN_SFF_MASK;

      // Check DBC to select what we need to receive.
#ifdef USE_ZF_VEHICLE_STATES
      can_filter_temp.can_id = can_Vehicle_Input_PrioB_02_mid;
      vehicle_can_filter_list.push_back(can_filter_temp);
      can_filter_temp.can_id = can_Vehicle_Input_PrioA_02_mid;
      vehicle_can_filter_list.push_back(can_filter_temp);
      can_filter_temp.can_id = can_Vehicle_Input_PrioA_01_mid;
      vehicle_can_filter_list.push_back(can_filter_temp);
      can_filter_temp.can_id = can_Vehicle_Input_PrioB_03_mid;
      vehicle_can_filter_list.push_back(can_filter_temp);
#else
      can_filter_temp.can_id = can_L_SAS11_mid;
      vehicle_can_filter_list.push_back(can_filter_temp);
      can_filter_temp.can_id = can_L_ESP12_mid;
      vehicle_can_filter_list.push_back(can_filter_temp);
      can_filter_temp.can_id = can_L_WHL_SPD11_mid;
      vehicle_can_filter_list.push_back(can_filter_temp);
      can_filter_temp.can_id = can_L_CGW_PC5_mid;
      vehicle_can_filter_list.push_back(can_filter_temp);
#endif
      return vehicle_can_filter_list;
    }

    void setRunningFlag(bool running_flag)
    {
      if (vision_wrapper2_)
      {
        // #PROC set tidl inferencer running flag
        vision_wrapper2_->set_running_inferencer(running_flag);
        // #END
      }
    }

    void canCallback(const phantom_ai::CanFrameList &frames)
    {
      static_cast<void>(frames);
      if (enable_vision_can_output_ && system_mode_.is_running())
      {
#if (ENABLE_ZEROMQ)
        zmq_wrapper_->sendZmqTx(frames);
#endif
        can_bus1_wrapper_->sendTxFrame(frames);
      }
      else {;}
#if defined(ENABLE_VISION_LOGGING)
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        uint32_t count = frames.can_frame_arr_.size();
        if ((count + can_data_.count) < MAX_CAN_MSG_COUNT)
        {
          convertCan(frames, &(can_data_.msgs[can_data_.count]));
          can_data_.count += count;
        }
        else
        {
          //PHANTOM_WARNING("Overflowed Vision CAN msgs, dropping");
        }
      }
#endif
    }

#if defined(ENABLE_VISION_LOGGING) || defined(ENABLE_PROTOBUF_MSG)
    void publishPhantomVisionMeasurement(const phantom_ai::PhantomVisionMeasurement& input)
    {
      TESTLOG(logPvOutputSeq("VisionDemo2::publishPhantomVisionMeasurement"));
#if defined(ENABLE_VISION_LOGGING)
      if (options_.enable_verbosity_)
      {
        PHANTOM_LOG("publishing PhantomVisionMeasurement through ethernet");
      }
      publishPhantomVisionMeasurementToLoggingPC(input);
#endif

#if defined(ENABLE_PROTOBUF_MSG)
      if (options_.enable_verbosity_)
      {
        PHANTOM_LOG("publishing PhantomVisionMeasurement through IPC");
      }
      publishPhantomVisionMeasurementThroughIPC(input);
#endif
    }
#endif

    void publishDynamicCalibrationOutput([[maybe_unused]] const phantom_ai::PhantomVisionDynamicCalibrationOutputList& input)
    {
      // #IGNORE Test logging
      TESTLOG(logPvOutputSeq("VisionDemo2::publishDynamicCalibrationOutput"));
      // #END
#if defined(ENABLE_VISION_LOGGING)
      if (options_.enable_verbosity_)
      {
        PHANTOM_LOG("publishing dynamic calibratinn output through ethernet");
      }
      publishDynamicCalibrationOutputToLoggingPC(input);
#endif

#if defined(ENABLE_PROTOBUF_MSG)
      if (options_.enable_verbosity_)
      {
        PHANTOM_LOG("publishing dynamic calibration output through IPC");
      }
      publishDynamicCalibrationOutputThroughIPC(input);
#endif

#if defined(USE_CAM_PARAM_INPUT_FOR_VISION_INIT)
      updateCAMExtParamWithDynamicCalibrationResult(input);
#endif  // USE_CAM_PARAM_INPUT_FOR_VISION_INIT
      UNUSED(input);
    }

#if defined(ENABLE_VISION_LOGGING)
    void publishDynamicCalibrationOutputToLoggingPC(const phantom_ai::PhantomVisionDynamicCalibrationOutputList& input)
    {
      // Send frame
      if (net_tx_wrapper_)
      {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (dynm_calib_data_.count >= MAX_VISION_RESULTS)
        {
          //PHANTOM_WARNING("Overflowed dynamic calibration output, dropping");
          return;
        }

        net_tx_wrapper_->convertDynamicCalibrationFrame(input, &dynm_calib_data_.item[dynm_calib_data_.count]);
        dynm_calib_data_.count++;
      }
    }
#endif

#if defined(ENABLE_PROTOBUF_MSG)
    void publishDynamicCalibrationOutputThroughIPC(const phantom_ai::PhantomVisionDynamicCalibrationOutputList& input)
    {
      if (pb_wrapper_)
      {
        pb_wrapper_->publishDynamicCalibrationOutput(input);
      }
    }
#endif

#if defined(USE_CAM_PARAM_INPUT_FOR_VISION_INIT)
    void updateCAMExtParamWithDynamicCalibrationResult(const phantom_ai::PhantomVisionDynamicCalibrationOutputList& input)
    {
      std::lock_guard<std::mutex> lock(mtx_dynm_cal_output_);
      for (const auto& dynm_cal_output : input.calibration_outputs)
      {
        uint8_t cam_idx = dynm_cal_output.camera_type;
        if (cam_idx < phantom_ai::NUM_MAX_PHYSICAL_CAM_IDS)
        {
          if (dynm_cal_output.roll_converging_progress == 100)  // 100 %
          {
            cam_ext_parameters_[cam_idx].data_valid = true;
            cam_ext_parameters_[cam_idx].roll_valid = true;
            cam_ext_parameters_[cam_idx].roll_deg = dynm_cal_output.roll_converged_degree;

            PHANTOM_LOG("[{}] Roll angle calibrated. roll = {:.2f}", physical_camera_name(static_cast<phantom_ai::PhysicalCameraID>(cam_idx)), cam_ext_parameters_[cam_idx].roll_deg);
          }

          if (dynm_cal_output.pitch_converging_progress == 100) // 100 %
          {
            cam_ext_parameters_[cam_idx].data_valid = true;
            cam_ext_parameters_[cam_idx].pitch_valid = true;
            cam_ext_parameters_[cam_idx].pitch_deg = dynm_cal_output.pitch_converged_degree;

            PHANTOM_LOG("[{}] Pitch angle calibrated. pitch = {:.2f}", physical_camera_name(static_cast<phantom_ai::PhysicalCameraID>(cam_idx)), cam_ext_parameters_[cam_idx].pitch_deg);
          }

          if (dynm_cal_output.yaw_converging_progress == 100)   // 100 %
          {
            cam_ext_parameters_[cam_idx].data_valid = true;
            cam_ext_parameters_[cam_idx].yaw_valid = true;
            cam_ext_parameters_[cam_idx].yaw_deg = dynm_cal_output.yaw_converged_degree;

            PHANTOM_LOG("[{}] Yaw angle calibrated. yaw = {:.2f}", physical_camera_name(static_cast<phantom_ai::PhysicalCameraID>(cam_idx)), cam_ext_parameters_[cam_idx].yaw_deg);
          }
        }
        else
        {
          PHANTOM_WARNING("Physical camera {} doesn't exist.", cam_idx);
        }
      }
    }
#endif  // USE_CAM_PARAM_INPUT_FOR_VISION_INIT

#if defined(ENABLE_VISION_LOGGING) || defined(ENABLE_PROTOBUF_MSG)
    void publishHba(const phantom_ai::PhantomVisionHBA& input)
    {
      TESTLOG(logPvOutputSeq("VisionDemo2::publishHba"));
#if defined(ENABLE_VISION_LOGGING)
      if (options_.enable_verbosity_)
      {
        PHANTOM_LOG("publishing HBA through ethernet");
      }
      publishHbaToLoggingPC(input);
#endif

#if defined(ENABLE_PROTOBUF_MSG)
      if (options_.enable_verbosity_)
      {
        PHANTOM_LOG("publishing HBA through IPC");
      }
      publishHbaThroughIPC(input);
#endif
    }
#endif

#if defined(ENABLE_VISION_LOGGING)
    void publishHbaToLoggingPC(const phantom_ai::PhantomVisionHBA& input)
    {
      // Send frame
      if (net_tx_wrapper_)
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (hba_data_.count >= MAX_HBA_RESULTS)
        {
          //PHANTOM_WARNING("Overflowed Hba output, dropping");
          return;
        }

        net_tx_wrapper_->convertHbaDataFrame(input, &hba_data_.item[hba_data_.count]);
        hba_data_.count++;
      }
    }
#endif

#if defined(ENABLE_PROTOBUF_MSG)
    void publishHbaThroughIPC(const phantom_ai::PhantomVisionHBA& input)
    {
      if (pb_wrapper_)
      {
        pb_wrapper_->updateHbaMessage(input);
      }
    }
#endif

#if defined(ENABLE_VISION_LOGGING) || defined(ENABLE_PROTOBUF_MSG)
    void publishAeb(const phantom_ai::PhantomVisionAEB& input)
    {
#if defined(ENABLE_VISION_LOGGING)
      if (options_.enable_verbosity_)
      {
        PHANTOM_LOG("publishing AEB through ethernet");
      }
      publishAebToLoggingPC(input);
#endif

#if defined(ENABLE_PROTOBUF_MSG)
      if (options_.enable_verbosity_)
      {
        PHANTOM_LOG("publishing AEB through IPC");
      }
      publishAebThroughIPC(input);
#endif
    }
#endif

#if defined(ENABLE_VISION_LOGGING)
    void publishAebToLoggingPC(const phantom_ai::PhantomVisionAEB& input)
    {
      // Send frame
      if (net_tx_wrapper_)
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (aeb_data_.count >= MAX_AEB_RESULTS)
        {
          //PHANTOM_WARNING("Overflowed Aeb output, dropping");
          return;
        }

        net_tx_wrapper_->convertAebDataFrame(input, &aeb_data_.item[aeb_data_.count]);
        aeb_data_.count++;
      }
    }
#endif

#if defined(ENABLE_PROTOBUF_MSG)
    void publishAebThroughIPC(const phantom_ai::PhantomVisionAEB& input)
    {
      if (pb_wrapper_)
      {
        pb_wrapper_->updateAebMessage(input);
      }
    }
#endif

    void processVisionViz(const cv::Mat& input)
    {
      if(net_tx_wrapper_ && options_.tx_debug_image_)
      {
        #if defined (ENABLE_VISION_LOGGING)
          std::lock_guard<std::mutex> lock(data_mutex_);
          net_tx_wrapper_->queueVisionVizData(input);
        #else
          if (viz_fixed_framerate_)
          {
            double t_now = phantom_ai::TimeStamp::Now().toSec();
            float time_diff = (t_now - last_timestamp_) * 1000.0f;
            if (time_diff >= viz_framerate_ms_)
            {
              net_tx_wrapper_->sendVisionViz(input, t_now);
              last_timestamp_ = t_now;
            }
          }
          else
          {
            net_tx_wrapper_->sendVisionViz(input, phantom_ai::TimeStamp::Now().toSec());
          }
        #endif
      }
    }

#if defined(ENABLE_GSTREAMER)
    void processGstViz(const cv::Mat& input)
    {
      RETURN_IF(!system_mode_.is_running());
      RETURN_IF(input.empty());

      static bool   gst_init_status_ = false;
      static int    gst_framerate_hz_= 0;
      static float  gst_framerate_ms_= 0.0f;

      if(!encodeH264_send_by_gst_init_tried && !gst_init_status_) {
        phantom_ai::YamlNode node = phantom_ai::load_yaml_file("tools/gstreamer", "gstreamer_config.yaml");
        std::string gst_pipeline = phantom_ai::get_yaml_key<std::string>(node, "pipeline_string");
        std::string host_ip_ = phantom_ai::get_yaml_key<std::string>(node, "host_ip_address", "192.168.5.101");
        int host_port_ = phantom_ai::get_yaml_key<int>(node, "host_port", 5500);
        int framerate_ = phantom_ai::get_yaml_key<int>(node, "framerate_hz", 10);
        int width = static_cast<int>(vision_params_.visualizer_window.total_size.width);
        int height = static_cast<int>(vision_params_.visualizer_window.total_size.height);
        gst_framerate_hz_ = viz_fixed_framerate_ ? options_.viz_framerate_ : framerate_;

        gchar *final_pipeline_str = g_strdup_printf(
          gst_pipeline.c_str(),
          width,
          height,
          gst_framerate_hz_,
          host_ip_.c_str(),
          host_port_);

        gst_init_status_ = encodeH264_send_by_gst_init(
                        final_pipeline_str,
                        width,
                        height,
                        gst_framerate_hz_);
        g_free(final_pipeline_str);

        PHANTOM_WARNING("Sending debug viz via GStreamer enabled! Init configuration by {}:{}, {}x{}, {}fps was {}", \
          host_ip_, host_port_, vision_params_.visualizer_window.total_size.width, vision_params_.visualizer_window.total_size.height, gst_framerate_hz_, \
          (gst_init_status_) ? "successful" : "failed");

        gst_framerate_ms_ = 1000.0f / gst_framerate_hz_;
        encodeH264_send_by_gst_init_tried = true;
      }

      double t_now = phantom_ai::TimeStamp::Now().toSec();
      float time_diff = (t_now - last_timestamp_) * 1000.0f;
      if (time_diff >= gst_framerate_ms_) {
        cv::Mat rgb_image;
        cv::cvtColor(input, rgb_image, cv::COLOR_BGR2RGB);
        encodeH264_send_by_gst(rgb_image.data, rgb_image.total() * rgb_image.elemSize());
        last_timestamp_ = t_now;
      }
    }
#endif

#if defined(ENABLE_VISION_LOGGING)
    void publishPhantomVisionMeasurementToLoggingPC(const phantom_ai::PhantomVisionMeasurement& input)
    {
      // Send frame
      if (net_tx_wrapper_)
      {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (vision_data_.count >= MAX_VISION_RESULTS)
        {
          //PHANTOM_WARNING("Overflowed vision measurement, dropping");
          return;
        }

        net_tx_wrapper_->convertVisionFrame(input, &vision_data_.item[vision_data_.count]);
        vision_data_.count++;
      }
    }
#endif

#if defined(ENABLE_PROTOBUF_MSG)
    void publishPhantomVisionMeasurementThroughIPC(const phantom_ai::PhantomVisionMeasurement& input)
    {
      // Send frame immediately
      if (pb_wrapper_)
      {
        pb_wrapper_->publishVisionMessage(input);
      }
    }

    void timeStampJumpDetect(void)
    {
      // Our timer use c++ steady_clock that is safe from autobrain's timestamp jump
      phantom_ai::Timer timer(1, true); // 1sec timer
      phantom_ai::TimeStamp prev=0;
      std::chrono::time_point<std::chrono::steady_clock> prev_steady_clock;

      while ( timestamp_jump_detect_stop_ == false )
      {
        timer.tic();
        phantom_ai::TimeStamp now = phantom_ai::TimeStamp::Now();
        std::chrono::time_point<std::chrono::steady_clock> now_steady_clock = std::chrono::steady_clock::now();
        if ( prev != 0 )
        {
          std::chrono::duration<double> diff_steady_clock = now_steady_clock - prev_steady_clock;
          phantom_ai::TimeStamp diff = now - prev;
          double diff_steady = diff_steady_clock.count();
          double gap =  fabs(diff_steady - diff.toSec() );
          //printf("----- diff %lf , steady diff %lf gap : %lf raio %lf --\n", diff.toSec(), diff_steady, gap, (gap/diff_steady));

          if ( (gap / diff_steady) > timestamp_diff_threshold_ )
            PHANTOM_ERROR("Autobrain timestamp may be wrong with timestamp {:f} for 1 sec , But steady clock timestamp count {:f} for 1 sec",
                           diff.toSec(), diff_steady );
        }

        prev = now;
        prev_steady_clock = now_steady_clock;

        timer.toc();
      }
    }
#endif

    void wheelSpeedCallback(const phantom_ai::WheelSpeedMeasurements& wsm)
    {
      RETURN_IF(!system_mode_.is_running());
      TESTLOG(logVehStateSeq("VisionDemo2::wheelSpeedCallback"));

      // NOTE: we have not decided on whether to set wheel speed
      // measurment when 1. task disabled, 2. param value is incorrect, 3.
      // status/range is invalid, 4. during the first 3 seconds of init. Until
      // agreement, we'll set wheel speed regardlessly
      functional_safety_task_->checkWheelSpeedMeasurements(wsm);

      vision_wrapper2_->setWheelSpeedMeasurement(wsm);

#if defined(ENABLE_VISION_LOGGING)
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (veh_data_.wheelspeed_count >= MAX_VEHICLE_STATE_NUM)
        {
          //PHANTOM_WARNING("Overflowed yaw rate, dropping");
          return;
        }

        convertWheelSpeed(wsm, &(veh_data_.wheelspeed[veh_data_.wheelspeed_count]));
        veh_data_.wheelspeed_count++;
      }
#endif
    }

    void espCallback(const phantom_ai::EspMeasurements& esp)
    {
      // #IGNORE interface test logging
      TESTLOG(logVehStateSeq("VisionDemo2::espCallback"));
      // #END
      RETURN_IF(!system_mode_.is_running());

      // NOTE: we have not decided on whether to set esp
      // measurment when 1. task disabled, 2. param value is incorrect, 3.
      // status/range is invalid, 4. during the first 3 seconds of init. Until
      // agreement, we'll set esp measurement regardlessly
      functional_safety_task_->checkEspMeasurements(esp);

      vision_wrapper2_->setEspMeasurements(esp);

#if defined(ENABLE_VISION_LOGGING)
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (veh_data_.esp_count >= MAX_VEHICLE_STATE_NUM)
        {
          //PHANTOM_WARNING("Overflowed yaw rate, dropping");
          return;
        }

        convertEsp(esp, &(veh_data_.esp[veh_data_.esp_count]));
        veh_data_.esp_count++;
      }
#endif
    }

    void gearStateCallback(const phantom_ai::GearState& gear_state)
    {
      // #IGNORE Test logging
      TESTLOG(logVehStateSeq("VisionDemo2::gearStateCallback"));
      // #END

      RETURN_IF(!system_mode_.is_running());

      // NOTE: we have not decided on whether to set gear_state
      // measurment when 1. task disabled, 2. param value is incorrect, 3.
      // status/range is invalid, 4. during the first 3 seconds of init. Until
      // agreement, we'll set gear_state measurement regardlessly

      // NOTE: disabling since Mobis official test cars does not send out gear
      //       shifter positions as of now (8/27/24)
      // functional_safety_task_->checkGearStateMeasurements(gear_state.gear_state_);

      vision_wrapper2_->setGearStateMeasurements(gear_state);

#if defined(ENABLE_VISION_LOGGING)
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (veh_data_.gearstate_count >= MAX_VEHICLE_STATE_NUM)
        {
          //PHANTOM_WARNING("Overflowed gear state, dropping");
          return;
        }

        convertGearState(gear_state, &(veh_data_.gearstate[veh_data_.gearstate_count]));
        veh_data_.gearstate_count++;
      }
#endif
    }

    void steerAngleCallback(const phantom_ai::SteeringAngle &steer_angle)
    {
      // #IGNORE Test logging
      TESTLOG(logVehStateSeq("VisionDemo2::steerAngleCallback"));
      // #END

      RETURN_IF(!system_mode_.is_running());

      // NOTE: we have not decided on whether to set steering angle
      // measurment when 1. task disabled, 2. param value is incorrect, 3.
      // status/range is invalid, 4. during the first 3 seconds of init. Until
      // agreement, we'll set steering angle measurement regardlessly
      functional_safety_task_->checkSteeringAngleMeasurements(steer_angle);

      vision_wrapper2_->setSteeringAngle(steer_angle);

      // TODO: currently not publishing steer angle via network
    }

    void switchProtobufVisionMeasurementPublication(const bool& enable_publication)
    {
#if defined(ENABLE_PROTOBUF_MSG)
      if (enable_publication)
      {
        pb_wrapper_->enableVisionMeasurementPublication();
      }
      else
      {
        pb_wrapper_->disableVisionMeasurementPublication();
      }
#else
      static_cast<void>(enable_publication);
#endif
    }

    void switchProtobufDynamicCalibrationOutputPublication(const bool& enable_publication)
    {
#if defined(ENABLE_PROTOBUF_MSG)
      if (enable_publication)
      {
        pb_wrapper_->enableDynamicCalibrationOutputPublication();
      }
      else
      {
        pb_wrapper_->disableDynamicCalibrationOutputPublication();
      }
#else
      static_cast<void>(enable_publication);
#endif
    }

#if defined(ENABLE_PROTOBUF_MSG)
    SystemMode::Mode convert_AB_mode_to_system_mode(uint8_t mode)
    {
      switch (mode)
      {
      case 1:
        return SystemMode::Mode::INIT;
      case 2:
        return SystemMode::Mode::INHIBITION;
      case 3:
        return SystemMode::Mode::NORMAL_DRIVING;
      case 4:
        return SystemMode::Mode::HIGH_DRIVING_CONTROL;
      case 5:
        return SystemMode::Mode::LOW_PARKING_NO_CONTROL;
      case 6:
        return SystemMode::Mode::LOW_PARKING_CONTROL;
      default:
        PHANTOM_ERROR("Invalid AB input for SystemMode: {}", mode);
        return SystemMode::Mode::INIT;
      }
    }

    uint8_t convert_system_mode_to_AB_mode(SystemMode::Mode mode)
    {
      switch (mode)
      {
      case SystemMode::Mode::INIT:
        return 1;
      case SystemMode::Mode::INHIBITION:
        return 2;
      case SystemMode::Mode::NORMAL_DRIVING:
        return 3;
      case SystemMode::Mode::HIGH_DRIVING_CONTROL:
        return 4;
      case SystemMode::Mode::LOW_PARKING_NO_CONTROL:
        return 5;
      case SystemMode::Mode::LOW_PARKING_CONTROL:
        return 6;
      case SystemMode::Mode::FAULT:       [[fallthrough]];
      case SystemMode::Mode::PAUSE:       [[fallthrough]];
      case SystemMode::Mode::DIAGNOSTIC:  [[fallthrough]];
      case SystemMode::Mode::INVALID_REQ: [[fallthrough]];
      case SystemMode::Mode::EXIT:        [[fallthrough]];
      default:
        return 0;
      }
    }

    void switchModeCommandCallback(const SwitchModeCommand& switch_mode_command)
    {
      // the system is still initializing itself.
      if(false == is_ready_to_receive_switch_mode_command_)
      {
        pb_wrapper_->publishSwitchModeAck(kSwitchReturnFail, convert_system_mode_to_AB_mode(system_mode_.get_current_mode()));
        PHANTOM_ERROR("The system is still initializing itself, send the request later");
      }

      static uint8_t switch_level_open_cnt = 0;

      SystemMode::Mode prev_mode_backup = system_mode_.get_previous_mode();   // backup
      // 10-30-2023 Autobrain requests us to ignore requests except for kSwitchLevelOpen or kSwitchLevelFinish
      if (kSwitchLevelOpen == switch_mode_command.switch_level)
      {
        if(SystemMode::Mode::PAUSE != system_mode_.get_current_mode())
        {
          SystemMode::Mode next_AB_mode = convert_AB_mode_to_system_mode(switch_mode_command.switch_state.next_state);

          if(system_mode_.get_current_mode() == next_AB_mode)
          {
            pb_wrapper_->publishSwitchModeAck(kSwitchReturnOk, convert_system_mode_to_AB_mode(system_mode_.get_current_mode()));

            return;
          }
          else if(SystemMode::Mode::INVALID_REQ == next_AB_mode)
          {
            pb_wrapper_->publishSwitchModeAck(kSwitchReturnFail, convert_system_mode_to_AB_mode(system_mode_.get_current_mode()));

            return;
          }
          else {;}

          system_mode_.set_previous_mode(system_mode_.get_current_mode());

          if (system_mode_.check_precondition(next_AB_mode))
          {
            // Assumption: set_current_mode() { current_mode_ = mode }; is atomic operation.
            system_mode_.set_current_mode(SystemMode::Mode::PAUSE); // 1. set the PAUSE before actual change.
            switch_perception_mode(next_AB_mode);                   // 2. change the perception actually.
            system_mode_.set_current_mode(next_AB_mode);            // 3. change the current_mode_.
            pb_wrapper_->publishSwitchModeAck(kSwitchReturnOk, convert_system_mode_to_AB_mode(system_mode_.get_current_mode()));
          }
          else
          {
            // final decision: ERR
            // let's rollback to the previous mode.
            switch_level_open_cnt = 0;
            // Assumption: set_current_mode() { current_mode_ = mode }; is atomic operation.
            system_mode_.set_current_mode(SystemMode::Mode::PAUSE);         // 1. set the PAUSE before actual change.
            switch_perception_mode(system_mode_.get_previous_mode());       // 2. change the perception actually.
            system_mode_.set_current_mode(system_mode_.get_previous_mode());// 3. change the current_mode_.
            system_mode_.set_previous_mode(prev_mode_backup);   //restore
            pb_wrapper_->publishSwitchModeAck(kSwitchReturnFail, convert_system_mode_to_AB_mode(system_mode_.get_current_mode()));
          }
        }
        else
        {
          // timeout error happens, return back to the previous system mode
          if(kSwitchLevelOpenCnt <= switch_level_open_cnt)
          {
            // final decision: ERR
            // let's rollback to the previous mode.
            switch_level_open_cnt = 0;
            // Assumption: set_current_mode() { current_mode_ = mode }; is atomic operation.
            system_mode_.set_current_mode(SystemMode::Mode::PAUSE);         // 1. set the PAUSE before actual change.
            switch_perception_mode(system_mode_.get_previous_mode());       // 2. change the perception actually.
            system_mode_.set_current_mode(system_mode_.get_previous_mode());// 3. change the current_mode_.
            system_mode_.set_previous_mode(prev_mode_backup);  //restore
            pb_wrapper_->publishSwitchModeAck(kSwitchReturnFail, convert_system_mode_to_AB_mode(system_mode_.get_current_mode()));
          }
          else
          {
            switch_level_open_cnt++;
            // There is no PAUSE mode in the Autobrain interface, so let's publish the previous mode instead.
            pb_wrapper_->publishSwitchModeAck(kSwitchReturnInProgress, convert_system_mode_to_AB_mode(system_mode_.get_previous_mode()));
          }
        }
      }
      else if (kSwitchLevelFinish == switch_mode_command.switch_level)
      {
        // clear variables, which are for "open" signal handling.
        switch_level_open_cnt = 0;
        if(SystemMode::Mode::PAUSE != system_mode_.get_current_mode())
        {
          SystemMode::Mode next_AB_mode = convert_AB_mode_to_system_mode(switch_mode_command.switch_state.next_state);
          if(system_mode_.get_current_mode() == next_AB_mode)
          {
            pb_wrapper_->publishSwitchModeAck(kSwitchReturnOk, convert_system_mode_to_AB_mode(system_mode_.get_current_mode()));

            return;
          }
          else if(SystemMode::Mode::INVALID_REQ == next_AB_mode)
          {
            pb_wrapper_->publishSwitchModeAck(kSwitchReturnFail, convert_system_mode_to_AB_mode(system_mode_.get_current_mode()));

            return;
          }
          else {;}

          // the next_state of the "finish" has the final target state.
          // it can be the same of the "open" or can be different.
          system_mode_.set_previous_mode(system_mode_.get_current_mode());

          if (system_mode_.check_precondition(next_AB_mode))
          {
            // Autobrain said that we don't need to check any timeout for "finish" signal.
            // Assumption: set_current_mode() { current_mode_ = mode }; is atomic operation.
            system_mode_.set_current_mode(SystemMode::Mode::PAUSE); // 1. set the PAUSE before actual change.
            switch_perception_mode(next_AB_mode);                   // 2. change the perception actually.
            system_mode_.set_current_mode(next_AB_mode);            // 3. change the current_mode_.
            pb_wrapper_->publishSwitchModeAck(kSwitchReturnOk, convert_system_mode_to_AB_mode(system_mode_.get_current_mode()));
          }
          else
          {
            // final decision: ERR
            // let's rollback to the previous mode.
            // Assumption: set_current_mode() { current_mode_ = mode }; is atomic operation.
            system_mode_.set_current_mode(SystemMode::Mode::PAUSE);          // 1. set the PAUSE before actual change.
            switch_perception_mode(system_mode_.get_previous_mode());        // 2. change the perception actually.
            system_mode_.set_current_mode(system_mode_.get_previous_mode()); // 3. change the current_mode_.
            system_mode_.set_previous_mode(prev_mode_backup);  //restore
            pb_wrapper_->publishSwitchModeAck(kSwitchReturnFail, convert_system_mode_to_AB_mode(system_mode_.get_current_mode()));
          }
        }
        else
        {
          // There is no PAUSE mode in the Autobrain interface, so let's publish the previous mode instead.
          pb_wrapper_->publishSwitchModeAck(kSwitchReturnInProgress, convert_system_mode_to_AB_mode(system_mode_.get_previous_mode()));
        }
      }
      else
      {
        // Just return ok
        pb_wrapper_->publishSwitchModeAck(kSwitchReturnOk, convert_system_mode_to_AB_mode(system_mode_.get_current_mode()));
      }
    }

    /// [[deprecated]]
    void dynamicCalibrationInputCallback(phantom_ai::PhantomVisionDynamicCalibrationInput& input)
    {
      if(phantom_ai::CalibrationMode::DC_SPC_MODE == input.calibration_mode)
      {
        // transit to the SPC submode
        system_mode_.set_current_submode(SystemMode::SubMode::SPC);
      }
      else
      {
        // transit to the NO_SPC submode
        system_mode_.set_current_submode(SystemMode::SubMode::NO_SPC);
      }

      functional_safety_task_->checkPhantomVisionDynamicCalibrationInput(input);
    }

  #endif // defined(ENABLE_PROTOBUF_MSG)

    void switch_perception_mode(SystemMode::Mode mode)
    {
#if defined(APP_MODE_6CAM)    // 8MP x 1ea + (2MP x 5ea)
      switch (mode)
      {
        case SystemMode::Mode::HIGH_DRIVING_CONTROL:
        {
          reset_c7x_and_vision_config(
            {true, true, true, true},
            vision_cfg_nodename_,
            "vision_param2_tda4x_6cam_whole_crop.yaml",
            tidl_inferencer_,
            vision_wrapper2_);
          break;
        }
        case SystemMode::Mode::NORMAL_DRIVING:
        {
          reset_c7x_and_vision_config(
            {false, true, true, true},
            vision_cfg_nodename_,
            "vision_param2_tda4x_4cam_front_and_rears_whole_crop.yaml",
            tidl_inferencer_,
            vision_wrapper2_);
          break;
        }
        case SystemMode::Mode::LOW_PARKING_NO_CONTROL:
        {
          reset_c7x_and_vision_config(
            {false, false, false, true},
            vision_cfg_nodename_,
            "vision_param2_tda4x_front_base.yaml",
            tidl_inferencer_,
            vision_wrapper2_);
          break;
        }
        case SystemMode::Mode::LOW_PARKING_CONTROL: [[fallthrough]];
        case SystemMode::Mode::FAULT:               [[fallthrough]];
        case SystemMode::Mode::PAUSE:               [[fallthrough]];
        case SystemMode::Mode::INIT:                [[fallthrough]];
        case SystemMode::Mode::INHIBITION:          [[fallthrough]];
        case SystemMode::Mode::DIAGNOSTIC:          [[fallthrough]];
        case SystemMode::Mode::INVALID_REQ:         [[fallthrough]];
        case SystemMode::Mode::EXIT:                [[fallthrough]];
        default:
          break; // Do nothing
      };
#else // #if defined(APP_MODE_6CAM)
      phantom_ai::UNUSED(mode); // prevent a compiler warning.
#endif // #if defined(APP_MODE_6CAM)
      vision_wrapper2_->startVisionTasks();
    }

    void rxParserCallback(const phantom_ai::CanFrame &frame)
    {
      // #IGNORE Test logging
      TESTLOG(logVehStateSeq("VisionDemo2::rxParserCallback"));
      // #END

      if (system_mode_.is_running())
      {
        int8_t parser_status = vehicle_state_->parseVehicleStateMessage(frame);
        if (parser_status != 0)
        {
          PHANTOM_WARNING("Vehicle state parser failed to parse CAN frame.");
        }
        else {;}
      }
      else {;}
    }

    void tidlCallback(std::shared_ptr<phantom_ai::TidlOutput> output)
    {
      // #IGNORE Test logging
      TESTLOG(logTIDLInputSeq("VisionDemo2::tidlCallback"));
      // #END

      RETURN_IF(!system_mode_.is_vision_function_running());

      vision_wrapper2_->decodeTidlOutput(output, options_.enable_verbosity_);
    }

#ifdef ENABLE_FETCH_CAMERA_EXTRA_DATA
    void copyCameraExtraData(uint8_t* result, phantom_ai::HbaCameraExtraDataInputS output)
    {
      CameraSuppInfo_t* pinfo = (CameraSuppInfo_t*)result;
      if(pinfo->is_valid_exp_front_)
      {
        output->SetExpTimeAnalogGain(pinfo->exp_front_, pinfo->a_gain_front_);
        output->SetHistogram(pinfo->histogram_front_, HISTOGRAM_NUM_BIN);
        output->SetH3a(pinfo->h3a_front_, H3A_BUF_SIZE);
      }
      if(pinfo->is_valid_raw_img_)
      {
        output->SetRawImg(pinfo->raw_front_center_crop_, 512, 256);
      }
    }
#endif
#ifdef ENABLE_FETCH_CAMERA_EXTRA_DATA_REV
    void copyCameraExtraData(const std::shared_ptr<phantom_ai::CameraExtraInfo>& cam_extra_info, phantom_ai::HbaCameraExtraDataInputS output)
    {
      HAL_Camera_Extra_Info* extra_info = &(cam_extra_info->hal);
      HAL_Cam_Settings* settings = &(extra_info->settings);
      HAL_Cam_H3A_Data* h3a = &(extra_info->h3a);
      HAL_Cam_Bayer_Data* bayer = &(extra_info->bayer[HAL_CAM_BAYER_CROP_FRONT_CENTER_CROP]);

      if(settings->valid_exp_again_mode_flag)
      {
        output->SetExpTimeAnalogGainMode(settings->exposure, settings->gain, settings->camera_mode);
      }

      if(h3a->valid_flag)
      {
        output->SetH3a(h3a->buffer, phantom_ai::SIZE_MAX_DATABUF_H3A);
      }

      if(bayer->valid_flag)
      {
        output->SetRawImg((uint16_t* )bayer->buffer, bayer->crop.region.width , bayer->crop.region.height);
      }
      //histogram is not implemented yet
    }
#endif

#ifdef ENABLE_FETCH_CAMERA_EXTRA_DATA_REV
#if defined (ENABLE_VISION_LOGGING) && defined (ENABLE_FULLRES_IMG_LOGGING) && defined (ENABLE_FULLRES_EXTRA_LOGGING)
    //to do more : get raw crop roi
    void publishCameraExtraInfoToLoggingPC(const std::shared_ptr<phantom_ai::CameraExtraInfo>& cam_extra_info, double ts_double)
    {
      phantom_ai::PhantomCameraExtraH3aConfig h3cfg; //use default values

      HAL_Camera_Extra_Info* extra_info = &(cam_extra_info->hal);
      HAL_Cam_Settings* settings = &(extra_info->settings);
      HAL_Cam_H3A_Data* h3a = &(extra_info->h3a);
      HAL_CAM_BAYER_CROP_ID bayer_index = HAL_CAM_BAYER_CROP_FRONT_CENTER_CROP; //hba mode uses crop roi

      #if defined (ENABLE_FULLRES_IMG_LOGGING)
      bayer_index = HAL_CAM_BAYER_CROP_FRONT_CENTER_FULL;
      #endif

      HAL_Cam_Bayer_Data* bayer = &(extra_info->bayer[bayer_index]);

      auto cameras_extra_data = std::make_shared<phantom_ai::PhantomCameraExtraInput>(phantom_ai::CameraID::CAM_FRONT_CENTER,
                                                                                      std::string("front_center"),
                                                                                      settings->exposure,
                                                                                      settings->gain,
                                                                                      settings->camera_mode,
                                                                                      #ifdef USE_EXTRA_INPUT_HISTOGRAM
                                                                                      NULL, //histogram is not available yet.
                                                                                      #endif
                                                                                      h3cfg,
                                                                                      (uint8_t*)h3a->buffer,
                                                                                      (uint8_t*)bayer->buffer,
                                                                                      bayer->crop.start_x,
                                                                                      bayer->crop.start_y,
                                                                                      bayer->crop.width ,
                                                                                      bayer->crop.height,
                                                                                      ts_double);
      phantom_ai::PhantomCameraExtarInputListS input_list;
      cameras_extra_data->header.set_time_stamp(T_NOW);

      input_list.push_back(cameras_extra_data);
      // Send frame
      if (net_tx_wrapper_)
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if(camera_extra_data_.count < MAX_SIZE_CAMERA_EXTRA_TIME_CNT)
        {
          net_tx_wrapper_->convertCameraExtraDataFrame(input_list, &(camera_extra_data_.extra_info_list[camera_extra_data_.count]));
          camera_extra_data_.count++;
        }
      }
    }

#endif//#if defined(ENABLE_VISION_LOGGING)
#endif //#ifdef ENABLE_FETCH_CAMERA_EXTRA_DATA_REV

#if defined (ENABLE_VISION_LOGGING) || defined (ENABLE_PROTOBUF_MSG)
    void publishFunctionalSafetyOutput(const phantom_ai::FunctionalSafetyOutput& functional_safety_output)
    {
  #if defined(ENABLE_VISION_LOGGING)
      publishFunctionalSafetyOutputToLoggingPC(functional_safety_output);
  #endif

  #if defined(ENABLE_PROTOBUF_MSG)
      publishFunctionalSafetyOutputToIPC(functional_safety_output);
  #endif
    }

  #if defined(ENABLE_VISION_LOGGING)
    void publishFunctionalSafetyOutputToLoggingPC(const phantom_ai::FunctionalSafetyOutput& functional_safety_output)
    {
      // Send frame
      if (net_tx_wrapper_)
      {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (fusa_data_.count >= MAX_FUSA_RESULTS)
        {
          // PHANTOM_WARNING("Overflowed fusa data, dropping");
          return;
        }

        net_tx_wrapper_->convertFusaDataFrame(functional_safety_output, &fusa_data_.item[fusa_data_.count]);
        fusa_data_.count++;
      }
    }
  #endif

  #if defined(ENABLE_PROTOBUF_MSG)
    void publishFunctionalSafetyOutputToIPC(const phantom_ai::FunctionalSafetyOutput& functional_safety_output)
    {
      pb_wrapper_->publishFunctionalSafetyOutput(functional_safety_output);
    }
  #endif
#endif

    // cv::Mat in callback are safe to modify and safe to share_ptr
    void imageCallback(std::vector<phantom_ai::CameraResults>& image_arr,
                       const std::unordered_map<phantom_ai::CameraID,
                       std::shared_ptr<phantom_ai::CameraExtraInfo>>& extra_info)
    {
      RETURN_IF(!system_mode_.is_vision_function_running());
      RETURN_IF(!vision_wrapper2_->isInitialized());
      RETURN_IF(functional_safety_task_->limit_throughput_flag());
      TESTLOG(logCameraInputSeq("VisionDemo2::imageCallback"));

      //* Check if there is no captured image, then return
      if (image_arr.empty())
      {
        // throw VisionException("No valid image sets");
        PHANTOM_ERROR("No valid image sets");
        return;
      }
      TESTLOG(logCameraInputFuSaDataMap<phantom_ai::CameraResults>(image_arr, true));

      functional_safety_task_->checkCameraData(image_arr);

      double timestamp = image_arr[0].timestamp;
      uint32_t frame_number = image_arr[0].frame_number;
      #if defined (SOC_AM62A)
      timestamp = monotonicToRealTime(timestamp);
      #endif
      TESTLOG(logCameraInputDataMapInput<phantom_ai::CameraResults>(image_arr, timestamp));
      TESTLOG(logCameraTIDLInput(timestamp, image_arr.size()));

      bool frame_rate_check = vision_wrapper2_->checkCameraFrameRate(timestamp);

#ifdef ENABLE_FETCH_CAMERA_EXTRA_DATA
      uint32_t num_of_images_original = image_arr.size() - NUM_CAMERA_EXTRA_DATA;
      uint32_t idx_imgarr_extra0 = num_of_images_original + CAMERA_EXTRA_DATA_INDEX_OFFSET;
      auto& extra0_on_imgarr = image_arr[idx_imgarr_extra0];

      auto cameras_extra_data = std::make_shared<phantom_ai::HbaCameraExtraDataInput>(T_NOW);

      if (extra0_on_imgarr.extra_data_mat.empty() == false)
      {
        copyCameraExtraData(extra0_on_imgarr.extra_data_mat.data, cameras_extra_data);

        cameras_extra_data->t_hw() = image_arr[0].timestamp;
        cameras_extra_data->t_src() = T_NOW;
        cameras_extra_data->t_end() = T_NOW;
        cameras_extra_data->frame() = cameras_data_frame_index_ + 1;

        /*post extra data*/
        if (frame_rate_check)
        {
          vision_wrapper2_->postCameraExtraData(cameras_extra_data);
        }
        /*resize to original size*/
        image_arr.resize(image_arr.size() - NUM_CAMERA_EXTRA_DATA);
      }
      else
      {
        PHANTOM_ERROR("Extra data is empty.");
      }
#endif
#ifdef ENABLE_FETCH_CAMERA_EXTRA_DATA_REV
      auto it = extra_info.find(phantom_ai::CAM_FRONT_CENTER); // front camera
      if (it != extra_info.end())
      {
        auto cam_extra_info = it->second;
        auto cameras_extra_data = std::make_shared<phantom_ai::HbaCameraExtraDataInput>(T_NOW);

        copyCameraExtraData(cam_extra_info, cameras_extra_data);

        cameras_extra_data->t_hw() = timestamp;
        cameras_extra_data->t_src() = T_NOW;
        cameras_extra_data->t_end() = T_NOW;
        cameras_extra_data->frame() = cameras_data_frame_index_ + 1;
#if defined (ENABLE_VISION_LOGGING) && defined (ENABLE_FULLRES_IMG_LOGGING) && defined (ENABLE_FULLRES_EXTRA_LOGGING)
        publishCameraExtraInfoToLoggingPC(cam_extra_info, timestamp);
#endif
        /*post extra data*/
        if (frame_rate_check)
        {
          vision_wrapper2_->postCameraExtraData(cameras_extra_data);
        }
      }
#else   // ENABLE_FETCH_CAMERA_EXTRA_DATA_REV
      UNUSED(extra_info);
#endif  // ENABLE_FETCH_CAMERA_EXTRA_DATA_REV

      std::unordered_multimap<phantom_ai::CameraID, cv::Mat> cam_imgs; // CameraID to image
      std::unordered_map<phantom_ai::CameraID, cv::Mat> vis_imgs; // CameraID to image
      for (auto& cam : image_arr)
      {
#if defined(APP_MODE_2CAM)
        // only consider images that are produced from front or rear center cameras
        if (cam.id <= phantom_ai::CAM_REAR_CENTER_CROP) {
#endif
          PHANTOM_LOG_IF(0, "Got a captured image from camera {}", cam.id)
          cam_imgs.insert({cam.id,cam.image});
          if (!is_annotation_image(cam.image.size().height)) {
            vis_imgs.insert({cam.id, cam.image});
          }

#if defined(ENABLE_PROTOBUF_MSG)
          // functional safety task only with main images of each cameras.
          if (phantom_ai::is_camera_type(cam.id, phantom_ai::CAM_TYPE_MAIN))
          {
            functional_safety_task_->updateCameraDataErrorValue(cam.id, cam.status);
          }
          else
          {;} //do nothing
#endif
#if defined(APP_MODE_2CAM)
        }
#endif
      }

      if (frame_rate_check)
      {
        //#COND Check if the feature task is underrunning
        if (vision_wrapper2_->isFeatureTrackTaskUnderrun())
        {
          cnt_frame_vision_track_task_underrun_++;
          if(cnt_frame_vision_track_task_underrun_ % 2 == 1)
          {
            //#PROC Skip a frame every 2 frames
            //#END
            return;
          }
        }
        else
        {
          cnt_frame_vision_track_task_underrun_ = 0;
        }

        cameras_data_frame_index_ = phantom_ai::convertToPhantomVisionImageFrameCount(cameras_data_frame_index_ + 1);
        t_image_last_ = timestamp;

        auto cameras_data = std::make_shared<phantom_ai::CamerasData>(T_NOW, phantom_ai::NUM_MAX_CAM_IDS);
        setCamerasData(timestamp, vis_imgs, cameras_data);
        // ------------[BEGIN] INSERT REVISED PRINT CODE HERE------------
        PHANTOM_LOG("===== Comparing Camera IDs for frame {} =====", cameras_data_frame_index_);

        // 1. Print IDs from the raw camera pipeline output
        PHANTOM_LOG("image_arr IDs (from physical camera pipeline):");
        for (const auto& cam_result : image_arr)
        {
            PHANTOM_LOG("  - Physical Camera ID: {}", static_cast<int>(cam_result.id));
        }

        // 2. Print IDs from the processed data object, ONLY if they have a valid image
        PHANTOM_LOG("cameras_data IDs (after processing with virtual child cameras):");
        const auto& all_possible_ids = cameras_data->cameras();
        bool found_processed_image = false;
        if (!all_possible_ids.empty())
        {
            for (const auto& cam_id : all_possible_ids)
            {
                // ADDED CHECK: Only print if a valid image exists for this ID
                if (cameras_data->Valid(cam_id))
                {
                    PHANTOM_LOG("  - Processed Camera ID: {}", static_cast<int>(cam_id));
                    found_processed_image = true;
                }
            }
        }

        if (!found_processed_image)
        {
            PHANTOM_LOG("  - (No valid images were inserted into cameras_data)");
        }
        PHANTOM_LOG("==================================================");
        // ------------[END] INSERT REVISED PRINT CODE HERE------------
        for (auto& cam : image_arr)
          cameras_data->set_crop_roi(cam.id, cam.crop_roi);

        //* Post CamerasData to Tidl Inference Task
        //* For the front only solution(SOP: end of Sept, 2023),
        //* we do not use HIGH_DRIVING_CONTROL, so we will not use 4 c7x cores
        //* we have dedicated c7x_3 and c7x_4, so let's use them for the all cases except for LOW_PARKING_CONTROL
        switch(system_mode_.get_current_mode())
        {
          case SystemMode::Mode::NORMAL_DRIVING:          [[fallthrough]];
          case SystemMode::Mode::LOW_PARKING_NO_CONTROL:  [[fallthrough]];
          // even though we do not use this mode, Autobrain can request this mode and expect the system to function.
          // so let's call postCamerasDataIntoTidlInferencer() for HIGH_DRIVING_CONTROL
          case SystemMode::Mode::HIGH_DRIVING_CONTROL:
            postCamerasDataIntoTidlInferencer(timestamp, frame_number, cameras_data);
            switchProtobufVisionMeasurementPublication(true);
            switchProtobufDynamicCalibrationOutputPublication(true);
            break;
          case SystemMode::Mode::INHIBITION:
            switchProtobufVisionMeasurementPublication(false);
            // If SPC requests even under INHIBITION mode, the system needs TIDL.
            if(SystemMode::SubMode::SPC == system_mode_.get_current_submode())
            {
              switchProtobufDynamicCalibrationOutputPublication(true);
              postCamerasDataIntoTidlInferencer(timestamp, frame_number, cameras_data);
            }
            else
            {
              switchProtobufDynamicCalibrationOutputPublication(false);
            }
            break;
          case SystemMode::Mode::LOW_PARKING_CONTROL: [[fallthrough]]; // Do not call postCamerasDataIntoTidlInferencer
          case SystemMode::Mode::FAULT:               [[fallthrough]];
          case SystemMode::Mode::INIT:                [[fallthrough]];
          case SystemMode::Mode::PAUSE:               [[fallthrough]];
          case SystemMode::Mode::DIAGNOSTIC:          [[fallthrough]];
          case SystemMode::Mode::INVALID_REQ:         [[fallthrough]];
          case SystemMode::Mode::EXIT:
          default:
          {
            switchProtobufVisionMeasurementPublication(false);
            switchProtobufDynamicCalibrationOutputPublication(false);
            break;
          }
        }

        //* Post CamerasData to Phantom Vision Tasks
        cameras_data->t_hw() = timestamp;
        cameras_data->t_src() = T_NOW;
        cameras_data->t_end() = T_NOW;
        cameras_data->frame() = cameras_data_frame_index_;
        vision_wrapper2_->postCamerasData(cameras_data);

#if defined(ENABLE_VISION_LOGGING)
        if (viz_fixed_framerate_)
        {
          float time_diff = (timestamp - last_timestamp_) * 1000.0f;
          if (time_diff >= viz_framerate_ms_)
          {
            postImageToNetwork(timestamp, cameras_data_frame_index_, cam_imgs);
            last_timestamp_ = timestamp;
          }
        }
        else
        {
          postImageToNetwork(timestamp, cameras_data_frame_index_, cam_imgs);
        }
#endif
      }
    }

    void setCamerasData(const double& timestamp, std::unordered_map<phantom_ai::CameraID, cv::Mat>& cam_imgs, phantom_ai::CamerasDataS& cameras_data)
    {
      RETURN_IF(!system_mode_.is_vision_function_running());

      // #COND for each captured camera image
      for (const auto& cam_img_it : cam_imgs)
      {
        // #IGNORE
        const auto& cam_id = cam_img_it.first;
        const auto& cam_img = cam_img_it.second;
        // #END

        // skip full resolution vision image
        if (phantom_ai::is_full_resolution_camera(cam_id)) {
          continue;
        }

        // #COND if crop information exist for this camera
        if (cameras_crop_info_.count(cam_id) && cameras_crop_info_[cam_id].is_initialized_)
        {
          // #IGNORE
          const auto& crop_info = cameras_crop_info_[cam_id];
          // #END

          // #IGNORE
          const auto& crop_roi = crop_info.crop_roi_;
          // #END

          // #PROC Copy the image using the crop ROI in the crop information
          cv::Mat copied;
          cam_img(crop_roi).copyTo(copied);
          cameras_data->Insert(cam_id, timestamp, false, copied);
          // #END

          // #COND If children cameras exist
          if (crop_info.has_children_)
          {
            // #COND for each child camera
            for (const auto& child_crop_info : crop_info.child_cams_crop_roi_)
            {
              // #IGNORE
              const auto& child_cam_id = child_crop_info.first;
              const auto& child_crop_roi = child_crop_info.second;
              // #END

              // #PROC Copy each child image using the crop ROI in the crop information
              cv::Mat child_copied;
              cam_img(child_crop_roi).copyTo(child_copied);
              cameras_data->Insert(child_cam_id, timestamp, false, child_copied);
              // #END
            }
          }
        }
        else
        {
          throw VisionException("Crop information for the camera {} doesn't exist.", phantom_ai::camera_name(cam_id));
        }
      }
    }

    void postCamerasDataIntoTidlInferencer(const double& timestamp, uint32_t frame_number, phantom_ai::CamerasDataS& cameras_data)
    {
#if defined(SKIP_3RD_FRAME)
      if (cameras_data_frame_index_ % 3 == 0)
        return;
#endif
      TESTLOG(logCameraInputSeq("VisionDemo2::postCamerasDataIntoTidlInferencer"));

      RETURN_IF(!system_mode_.is_vision_function_running());

      //* get camera image list to run tidl
      auto camera_list = vision_wrapper2_->params().cameras_active;
      cameras_data->BuildGray();
      //* post cnn input
      auto tidl_input = std::make_shared<phantom_ai::TidlInput>();
      tidl_input->frame = cameras_data_frame_index_;
      tidl_input->cameras.resize(camera_list.size());
      tidl_input->cam_crop_rois.resize(camera_list.size());
      tidl_input->images.resize(camera_list.size());
      tidl_input->grays.resize(camera_list.size());
      //* set camera and image for all PV's active cameras
      for (size_t i = 0; i < camera_list.size(); ++i)
      {
        auto cam = camera_list[i];
        cv::Mat gray3;
        cv::cvtColor(cameras_data->gray(cam), gray3, cv::COLOR_GRAY2BGR);
        tidl_input->cameras[i] = cam;
        tidl_input->cam_crop_rois[i] = cameras_data->get_crop_roi(cam);
        tidl_input->images[i] = cameras_data->image(cam);
        tidl_input->grays[i] = gray3;
      }

      tidl_input->source = cameras_data;
      tidl_input->frame_number = frame_number;
      tidl_input->hw_timestamp = timestamp;
      tidl_input->rcv_timestamp = T_NOW.toSec();
      if (system_mode_.is_vision_function_running())
      {
        tidl_inferencer_->sendTidlInput(tidl_input);
      }
    }

#if defined (ENABLE_VISION_LOGGING) || defined (ENABLE_PROTOBUF_MSG)
    void ipcErrorNotify(int setFlag, int clearFlag)
    {
      if ( setFlag & phantom_ai::VEHICLE_STATE_MESSAGE_ERROR_TIMEOUT ) {
        functional_safety_task_->updateVehicleMessageStateErrorCode( phantom_ai::VEHICLE_STATE_MESSAGE_ERROR_TIMEOUT,  true );
      }
      if ( setFlag & phantom_ai::VEHICLE_STATE_MESSAGE_ERROR_LOSS) {
        functional_safety_task_->updateVehicleMessageStateErrorCode( phantom_ai::VEHICLE_STATE_MESSAGE_ERROR_LOSS,  true );
      }

      if ( clearFlag & phantom_ai::VEHICLE_STATE_MESSAGE_ERROR_TIMEOUT) {
        functional_safety_task_->updateVehicleMessageStateErrorCode( phantom_ai::VEHICLE_STATE_MESSAGE_ERROR_TIMEOUT,  false );
      }

      if ( clearFlag & phantom_ai::VEHICLE_STATE_MESSAGE_ERROR_LOSS ) {
        functional_safety_task_->updateVehicleMessageStateErrorCode( phantom_ai::VEHICLE_STATE_MESSAGE_ERROR_LOSS,  false );
      }
    }
#endif

#if defined(ENABLE_VISION_LOGGING)
    void postImageToNetwork(const double& timestamp, const uint32_t& frame_count, const std::unordered_multimap<phantom_ai::CameraID, cv::Mat>& cam_imgs)
    {
      std::shared_ptr<NetworkImageObj> msg = std::make_shared<NetworkImageObj>();
      msg->img_obj = cam_imgs;
      msg->timestamp = timestamp;
      msg->frame_count = frame_count;

      size_t max_queue_depth = 2;
      {
        std::lock_guard<std::mutex> lock(net_tx_mutex_);

        if (net_tx_queue_.size() < max_queue_depth)
        {
          //Available space, add pionter
          net_tx_queue_.push(msg);
        }
        else if (net_tx_queue_.size() == max_queue_depth)
        {
          //An image frame is waiting, replace it with a fresher image
          net_tx_queue_.pop();
          net_tx_queue_.push(msg);
          PHANTOM_WARNING("NetTx overrun, dropping frame");
        }
        else
        {
          PHANTOM_LOG("WARNING: NetTxQ invalid state!");
        }
      }

      net_tx_loop_cv_.notify_all();
    }
#endif

#if defined(ENABLE_VISION_LOGGING)
void networkLoop()
{
  std::unique_lock<std::mutex> lock(net_tx_mutex_);

  while (quit_loop_ == false)
  {
    while (!net_tx_queue_.empty())
    {
      auto output_arr = net_tx_queue_.front();
      net_tx_queue_.pop();
      lock.unlock();

      std::vector<cv::Mat> img_arr;
      std::vector<phantom_ai::CameraID> loc_arr;

      for (auto it: output_arr->img_obj)
      {
        #if defined(ENABLE_DUAL_LOGGING)
        if ((current_frame_in_sequence_ % 3 != 0) && is_annotation_image(it.second.size().height)){
          continue; // skip annotation images
        }
        #endif

        #if defined(ENABLE_FULLRES_IMG_LOGGING)
        if (!phantom_ai::is_full_resolution_camera(it.first))
          continue;
        #endif
#if defined(ENABLE_GSTREAMER)
        if(!gst_h264encoding_enabled_) {
          img_arr.push_back(it.second);
        }
#else
        img_arr.push_back(it.second);
#endif
        loc_arr.push_back(it.first);
      }

      #if defined(ENABLE_DUAL_LOGGING)
      // reset the buffer size before sending every frame
      if (((current_frame_in_sequence_++) % 3) != 0){
        net_tx_wrapper_->resetFrameBuffer(true, false);
        if (current_frame_in_sequence_ == 3) {
          current_frame_in_sequence_ = 0;
        }
      } else {
        net_tx_wrapper_->resetFrameBuffer(false, false);
      }
      #endif
#if defined(ENABLE_GSTREAMER)
      net_tx_wrapper_->resetFrameBuffer(false, false);
#endif
      {
        std::lock_guard<std::mutex> data_lock(data_mutex_);
        net_tx_wrapper_->queueVehicleData(veh_data_);
        // reset data
        veh_data_.esp_count = 0;
        veh_data_.wheelspeed_count = 0;

        net_tx_wrapper_->queueCanTxData(can_data_);
        can_data_.count = 0;

        // send Vision data
        net_tx_wrapper_->queueVisionData(vision_data_);
        // reset data
        vision_data_.count = 0;

        // send Dynamic calibration data
        net_tx_wrapper_->queueDynamicCalibrationData(dynm_calib_data_);
        // reset data
        dynm_calib_data_.count = 0;

        // send Fusa data
        net_tx_wrapper_->queueFusaData(fusa_data_);
        // reset data
        fusa_data_.count = 0;

        #if defined (ENABLE_FULLRES_EXTRA_LOGGING)
        // send camera extra info data
        net_tx_wrapper_->queueCameraExtraData(camera_extra_data_);
        camera_extra_data_.count = 0;
        #endif

        // send Hba data
        net_tx_wrapper_->queueHbaData(hba_data_);
        // reset data
        hba_data_.count = 0;

        // send Aeb data
        net_tx_wrapper_->queueAebData(aeb_data_);
        // reset data
        aeb_data_.count = 0;
      }
#if defined(ENABLE_GSTREAMER)
      if(gst_h264encoding_enabled_) {
        net_tx_wrapper_->sendEncodedImageFrame(loc_arr, output_arr->timestamp, output_arr->frame_count);
      }
#else
        net_tx_wrapper_->sendImageFrame(img_arr, loc_arr, output_arr->timestamp, output_arr->frame_count);
#endif

      if (quit_loop_) {
        return;
      }

      lock.lock(); //lock so the while() predicate can access the queue
    }

    // wait for something to show up in queue
    net_tx_loop_cv_.wait(lock); //releases lock when called, reacquires it on return
  }
}
#endif //ENABLE_VISION_LOGGING

#if defined(AUTOBRAIN_VH_SPECIFIC_APP) && defined(ENABLE_PROTOBUF_MSG)
    void Perform6MonthCheck()
    {
      // In order to update the 6 month cutoff, edit this line
      phantom_ai::TimeStamp start_of_6_month_ts = phantom_ai::TimeStamp::FromUTCStringMilliSec("2024-01-01-00-00-00.000");

      static constexpr double kSecondsIn6Months{1.577e+7};
      if (phantom_ai::TimeStamp::Now().toSec() - start_of_6_month_ts.toSec() > kSecondsIn6Months)
      {
        PHANTOM_ERROR("Time has expired. Program exiting");
        exit(EXIT_FAILURE);
      }
    }
#endif

#if defined(USE_CAM_PARAM_INPUT_FOR_VISION_INIT)
    bool readDynamicCalibrationInputFile(phantom_ai::PhantomVisionDynamicCalibrationInput& dynm_cal_input)
    {
      bool ret;
      std::ifstream fin_dynm_cal_input(dynamic_calibration_param_filename_, std::ios::in);
      if (fin_dynm_cal_input.is_open())
      {
        while(fin_dynm_cal_input.good())
        {
          uint8_t camera_id;
          bool roll_valid, pitch_valid, yaw_valid;
          float roll_deg, pitch_deg, yaw_deg;

          std::string s;
          std::getline(fin_dynm_cal_input, s, ',');
          if (s.size() == 0)
            break;
          camera_id = std::stoi(s);

          std::getline(fin_dynm_cal_input, s, ',');
          roll_valid = std::stoi(s);

          std::getline(fin_dynm_cal_input, s, ',');
          roll_deg = std::stof(s);

          std::getline(fin_dynm_cal_input, s, ',');
          pitch_valid = std::stoi(s);

          std::getline(fin_dynm_cal_input, s, ',');
          pitch_deg = std::stof(s);

          std::getline(fin_dynm_cal_input, s, ',');
          yaw_valid = std::stoi(s);

          std::getline(fin_dynm_cal_input, s, '\n');
          yaw_deg = std::stof(s);

          phantom_ai::PhantomVisionDynamicCalibrationCameraInput one_camera_input;
          one_camera_input.camera_type = camera_id;
          one_camera_input.nominal_roll_deg_valid = roll_valid;
          one_camera_input.nominal_roll_deg = roll_deg;
          one_camera_input.nominal_pitch_deg_valid = pitch_valid;
          one_camera_input.nominal_pitch_deg = pitch_deg;
          one_camera_input.nominal_yaw_deg_valid = yaw_valid;
          one_camera_input.nominal_yaw_deg = yaw_deg;

          dynm_cal_input.camera_inputs.push_back(one_camera_input);
          dynm_cal_input.valid = true;

          cam_ext_parameters_[camera_id].data_valid = true;
          cam_ext_parameters_[camera_id].roll_valid = roll_valid;
          cam_ext_parameters_[camera_id].pitch_valid = pitch_valid;
          cam_ext_parameters_[camera_id].yaw_valid = yaw_valid;
          cam_ext_parameters_[camera_id].roll_deg = roll_deg;
          cam_ext_parameters_[camera_id].pitch_deg = pitch_deg;
          cam_ext_parameters_[camera_id].yaw_deg = yaw_deg;
        }

        if (dynm_cal_input.camera_inputs.size() > 0)
        {
          ret = true;
        }
        else
        {
          ret = false;
          dynm_cal_input.valid = false;
        }
      }
      else
      {
        PHANTOM_ERROR("Try to read camera parameters from file, but the file doesn't exist.");
        ret = false;
        dynm_cal_input.valid = false;
      }

      fin_dynm_cal_input.close();
      return ret;
    }

    void setDummyExtrinsicCameraParameters()
    {
      const auto& camera_models = vision_wrapper2_->camera_models();
      for (int i = 0; i < phantom_ai::NUM_MAX_CAM_IDS; ++i)
      {
        auto camera_model = camera_models->camera_model(i);
        if (camera_model)
        {
          const auto& physical_cam = get_physical_camera(static_cast<phantom_ai::CameraID>(i));
          if (cam_ext_parameters_[physical_cam].data_valid == false)
          {
            cam_ext_parameters_[physical_cam].data_valid = true;
            cam_ext_parameters_[physical_cam].roll_valid = false;
            cam_ext_parameters_[physical_cam].pitch_valid = false;
            cam_ext_parameters_[physical_cam].yaw_valid = false;
            cam_ext_parameters_[physical_cam].roll_deg = 0.0f;  // Dummy value. This will not be used because roll_valid == false
            cam_ext_parameters_[physical_cam].pitch_deg = 0.0f; // Dummy value. This will not be used because pitch_valid == false
            cam_ext_parameters_[physical_cam].yaw_deg = 0.0f;   // Dummy value. This will not be used because yaw_valid == false
          }
        }
      }
    }

    void saveDynamicCalibrationOutputToFile()
    {
      std::string target_dynamic_calibration_param_filename =
        dynamic_calibration_param_foldername_ + "/dynamic_cal_input.dcp";
      fout_dynm_cal_output_.open(target_dynamic_calibration_param_filename, std::ios::out);
      std::lock_guard<std::mutex> lock(mtx_dynm_cal_output_);
      for (int i = 0; i < phantom_ai::PhysicalCameraID::NUM_MAX_PHYSICAL_CAM_IDS; ++i)
      {
        const auto& cam_param = cam_ext_parameters_[i];
        if (cam_param.data_valid)
        {
          fout_dynm_cal_output_ << i << "," << cam_param.roll_valid << "," << cam_param.roll_deg << "," << cam_param.pitch_valid << "," << cam_param.pitch_deg
                               << "," << cam_param.yaw_valid << "," << cam_param.yaw_deg << std::endl;
        }
      }
      fout_dynm_cal_output_.close();
    }
#endif  // USE_CAM_PARAM_INPUT_FOR_VISION_INIT

    bool main_loop_timeout_status_;
    uint32_t system_tick_;

  private:
    std::string vision_cfg_nodename_ = "";
    std::string vision_cfg_filename_ = "";
    phantom_ai::VisionParams vision_params_;

    std::string dynamic_calibration_param_foldername_ = "";
    std::string dynamic_calibration_param_filename_ = "";

    phantom_ai::FunctionalSafetyTask *functional_safety_task_;
    phantom_ai::CanMessagePacker *can_packer_;
    phantom_ai::VisionWrapper2 *vision_wrapper2_;
    phantom_ai::TidlInferencer *tidl_inferencer_;
    phantom_ai::CameraWrapper *camera_wrapper_;
#if defined(ENABLE_PROTOBUF_MSG)
    phantom_ai::ProtobufWrapper *pb_wrapper_;

    // These are used to facilitate AB requesting our application to change states
    static constexpr uint8_t kSwitchLevelFree{0};
    static constexpr uint8_t kSwitchLevelCheck{1};
    static constexpr uint8_t kSwitchLevelCloseServer{2};
    static constexpr uint8_t kSwitchLevelCloseComponent{3};
    static constexpr uint8_t kSwitchLevelOpen{4};
    static constexpr uint8_t kSwitchLevelFinish{5};
    uint8_t current_switch_level_{kSwitchLevelFree}; // Start at FREE

    static constexpr uint8_t kSwitchLevelOpenCnt{60}; // 60 counts * 10 ms = 600 ms

    static constexpr uint8_t kSwitchReturnOk{0};
    static constexpr uint8_t kSwitchReturnFail{1};
    static constexpr uint8_t kSwitchReturnInProgress{2};

    std::thread timestamp_jump_detect_thread_ ;
    bool timestamp_jump_detect_stop_ = false;
    double timestamp_diff_threshold_ = 0.5;
#else
  #if (ENABLE_ZEROMQ)
    phantom_ai::ZmqWrapper *zmq_wrapper_;
  #endif
    phantom_ai::CanWrapper *can_bus0_wrapper_;
    phantom_ai::CanWrapper *can_bus1_wrapper_;
#endif
#if defined(ENABLE_ECU_KEY_MANAGER)
    phantom_ai::EcuKeyManagerWrapper *ecu_key_manager_wrapper_;
#endif // ENABLE_ECU_KEY_MANAGER

#ifdef USE_ZF_VEHICLE_STATES
    phantom_ai::ZFVehicleStateParser *vehicle_state_;
#else
    phantom_ai::VehicleStateParser *vehicle_state_;
#endif
    size_t cnt_frame_vision_track_task_underrun_;

    double t_image_last_;
    uint32_t cameras_data_frame_index_;

    phantom_ai::NetTxWrapper *net_tx_wrapper_;
#if defined(ENABLE_VISION_LOGGING)
    EmbeddedNet_PhantomVisionList vision_data_;
    EmbeddedNet_DynamicCalibrationResultList dynm_calib_data_;
    EmbeddedNet_VehicleInfo veh_data_;
    EmbeddedNet_CanFrames can_data_;
    EmbeddedNet_FunctionalSafetyOutputList fusa_data_;
    EmbeddedNet_CameraExtraInfoListTopLevel camera_extra_data_;
    EmbeddedNet_PhantomVisionHBAList hba_data_;
    EmbeddedNet_PhantomVisionAEBList aeb_data_;
#endif

#if defined(ENABLE_VISION_LOGGING)
    std::mutex data_mutex_;
    std::thread net_tx_thread_;
    std::queue<std::shared_ptr<NetworkImageObj>> net_tx_queue_;
    std::mutex net_tx_mutex_;
    std::condition_variable net_tx_loop_cv_;
    uint8_t current_frame_in_sequence_ = 0;
#endif

    bool quit_loop_{}; // initialized to false

#ifdef TEST_DYNAMIC_CROP_ARROW_KEY_CONTROL
    bool test_dynamic_crop_quit_loop_;
    std::thread test_dynamic_crop_thread_;
#endif

    bool enable_vision_can_output_;

    std::string vehicle_name_;

    ArgHandler options_;

    SystemMode system_mode_;

    bool is_ready_to_receive_switch_mode_command_;

    /// @brief Flag to indicate if the framerate of debug visualization is fixed.
    bool viz_fixed_framerate_ { false };

    /// @brief The speed of the debug visualization frame in milliseconds.
    /// The default is 15 hz -> 1000 ms / 15 = 66.67 ms.
    float viz_framerate_ms_ { 66.67f };

    /// @brief The last timestamp received from the camera.
    double last_timestamp_ { 0.0 };

    std::map<phantom_ai::CameraID, CapturedImageCropInfo> cameras_crop_info_;
#if defined(ENABLE_GSTREAMER)
    bool encodeH264_send_by_gst_init_tried{false};
    bool gst_h264encoding_enabled_ = false;
#endif
#if defined(USE_CAM_PARAM_INPUT_FOR_VISION_INIT)
    std::mutex mtx_dynm_cal_output_;
    std::array<CameraExtrinsicParameters, phantom_ai::PhysicalCameraID::NUM_MAX_PHYSICAL_CAM_IDS> cam_ext_parameters_;
    std::ofstream fout_dynm_cal_output_;
#endif  // USE_CAM_PARAM_INPUT_FOR_VISION_INIT
  }; // VisionDemo class

} // namespace phantom_linux

int main(int argc, char *argv[])
{
#if defined(AUTOBRAIN_VH_SPECIFIC_APP) && defined(ENABLE_PROTOBUF_MSG)
  // We use autobrain's osal timer. so we should call ab_init() firstly.
  ab_init(static_cast<int>(AutobrainCoreID::PERCEPTION));
#endif

  try {
    int timeout = 0;

    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "-t") {
        // Check if the next argument exists and is a valid integer
        if (i + 1 < argc && std::isdigit(argv[i + 1][0])) {
          max_diff_ms_ = std::stoi(argv[i + 1]);
          std::cout << "Max Diff MS: " << max_diff_ms_ << std::endl;
          // Remove the processed options and their values from argv
          for (int j = i; j < argc - 2; j++) {
            argv[j] = argv[j + 2];
          }
          argc -= 2; // Update the argument count
          i--;
        } else {
          std::cerr << "Error: -t option requires a valid integer value of millisecond." << std::endl;
          return 1;
        }
      } else if (std::string(argv[i]) == "-dt") {
        std::cout << "Debug Mode Enabled" << std::endl;
        debug_mode_ = true;
        // Remove the processed option from argv
        for (int j = i; j < argc - 1; j++) {
          argv[j] = argv[j + 1];
        }
        argc--; // Update the argument count
        i--;
      } else if (std::string(argv[i]) == "--timeout") {
        if (i + 1 < argc && std::isdigit(argv[i + 1][0])) {
          timeout = std::stoi(argv[i + 1]);
          if ((timeout < 1) || (timeout > MAX_TIMEOUT)) {
            std::cerr << "Error: Timeout must be in the range of 1 to %d minutes." << MAX_TIMEOUT << std::endl;
            return 1;
          } else {
            std::cout << "Application will stop after: " << timeout << " minutes" << std::endl;
            // Remove the processed options and their values from argv
            for (int j = i; j < argc - 2; j++) {
              argv[j] = argv[j + 2];
            }
            argc -= 2; // Update the argument count
            i--;
          }
        } else {
          std::cerr << "Error: --timeout option requires a valid integer value of minutes." << std::endl;
          return 1;
        }
      }
    }

    // Install the SIGINT (CTRL-C) handler.
    phantom_ai::signal_install_sigint_handler();

    // Lock all memory pages. This prevents page faults, which is a
    // potential performance issue in real time systems.
    // reference:
    // https://wiki.linuxfoundation.org/realtime/documentation/howto/applications/memory
    phantom_ai::memory_lock_all_current_and_future();

    // Set a a real-time FIFO scheduling policy for the main thread.
    // The scheduling policy is also inherited by child threads,
    // but can be overridden.
    phantom_ai::thread_set_fifo_schedule(phantom_ai::kMediumPriority);

    // Start the application.
    phantom_linux::VisionDemo2 vision_demo2(argc, argv);

    // Sleep until the user types CTRL-C.
    phantom_ai::signal_wait(timeout);

  } catch(std::exception &e) {
    PHANTOM_ERROR("Exiting with error: {}", e.what());
    return 1;
  }
  return 0;
}
