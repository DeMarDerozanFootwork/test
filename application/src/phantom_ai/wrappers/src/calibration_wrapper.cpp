/*********************************************************************
* @file    calibration_wrapper.cpp
* @date    03/02/2023
*
* @attention Copyright (c) 2023
* @attention Phantom AI, Inc.
* @attention All rights reserved.
*********************************************************************/

#include "phantom_ai/wrappers/calibration_wrapper.h"

#include "phantom_ai/utils/utils.h"
#include "phantom_ai/phantom_vision2/phantomnet_data.h"
#include "phantom_ai/core/profiler.h"

#define TIME_HW(t1, t2) (task_config_.enable_hardware_timestamp ? t1 : t2)
#ifndef CM_TO_M
#define CM_TO_M (0.01)
#endif

#define POST_CAMERA_CALIBRATION_MSG(msg_id, data)       \
  if (calibrator_main_) {                               \
    calibrator_main_->PostSharedData(                   \
      phantom_ai::VisionMessageID::msg_id,              \
      phantom_ai::VisionTaskID::TASK_ROS_NODE, data);   \
  }

#define CLEAR_CAMERA_CALIBRATION_MSG(msg_id)            \
  if (calibrator_main_) {                               \
    calibrator_main_->ClearMessage(                     \
      phantom_ai::VisionMessageID::msg_id);             \
  }

namespace phantom_ai
{

CalibrationWrapper::CalibrationWrapper() :
  calibrator_main_(nullptr),
  params_{},
  cameras_flag_(0),
  reset_counter_(0),
  tsec_image_last_(0.0)
{
  
}


CalibrationWrapper::~CalibrationWrapper()
{
  if (calibrator_main_)
  {
    calibrator_main_->Exit();
  }
}


void CalibrationWrapper::onInit(std::string &filename)
{
  phantom_ai::YamlNode root_node = phantom_ai::load_yaml_file("perception/" + std::string(PHANTOM_STATIC_CALIBRATION_LIB_NAME) + "/", filename); 
  phantom_ai::calib::read_params(root_node, params(), task_config_);

  calibrator_main_  = std::make_shared<phantom_ai::calib::CalibrationMain>(params());
  cameras_flag_     = phantom_ai::set_cameras_flag(params_.cameras_active);

  // register publishers which will publish messages as inputs to calibration module
  if (calibrator_main_) 
  {                  
    calibrator_main_->RegisterPublisher(phantom_ai::VisionMessageID::CAMERAS,
                                        phantom_ai::VisionTaskID::TASK_ROS_NODE);
    calibrator_main_->RegisterPublisher(phantom_ai::VisionMessageID::RESERVED,
                                        phantom_ai::VisionTaskID::TASK_ROS_NODE);
  }
}

void CalibrationWrapper::clearData()
{
  CLEAR_CAMERA_CALIBRATION_MSG(CAMERAS);
  CLEAR_CAMERA_CALIBRATION_MSG(RESERVED);
}

void CalibrationWrapper::reset()
{
  clearData();

  reset_counter_ = params_.cameras_active.size() * 2;
  tsec_image_last_ = 0.0;
  if (calibrator_main_)
  {
    calibrator_main_->SetReset(true);
  }
}


bool CalibrationWrapper::checkCameraFrameRate(double t)
{
  constexpr double tsec_margin = 0.010;
  double tsec_min_interval = 1 / task_config_.max_cameras_frame_rate - tsec_margin;
  double delta = t - tsec_image_last_;

  if (delta > tsec_min_interval || delta < 0.0)
  {
    tsec_image_last_ = t;
    return true;
  }

  return false;
}


bool CalibrationWrapper::checkImageSize(const cv::Size& input_size, cv::Rect& roi)
{
  cv::Size target_size = params().camera_image_size;

  // get the central crop if input size is larger than target size.
  roi.x = (input_size.width - target_size.width)/2;
  roi.y = (input_size.height - target_size.height)/2;
  roi.width = target_size.width;
  roi.height = target_size.height;

  if (roi.x >= 0 && roi.y >= 0)
  {
    return true;
  }
  else
  {
    PHANTOM_WARNING("Invalid input image size = {} x {}, target_size = {} x {}",
      input_size.width, input_size.height, target_size.width, target_size.height);
    return false;
  }
}

void CalibrationWrapper::setStaticCalibrationInputMsg(const StaticCalibrationDataInput &input)
{
  std::string log;
  if (input.sci.sci_routine_request != msg_data_.sci.sci_routine_request)
  {
    VLOG(log, "-------------------------------------- ");
    VLOG(log, "Receiving Static Calibration Input: ");
    VLOG(log, "  SubIPC Rx: timestamp                   : {}", phantom_ai::TimeStamp::Now().toSec());
    VLOG(log, "  SubIPC Rx: sci_routine_request         : {}", input.sci.sci_routine_request);
    VLOG(log, "  SubIPC Rx: sci_calibration_mode        : {}", input.sci.sci_calibration_mode);
    VLOG(log, "  SubIPC Rx: sci_calibration_target_type : {}", input.sci.sci_calibration_target_type);
    VLOG(log, "  SubIPC Rx: sci_vehicle_speed           : {}", input.sci.sci_vehicle_speed);
    VLOG(log, "  SubIPC Rx: sci_vehicle_door_status     : {}", input.sci.sci_vehicle_door_status);
    VLOG(log, "  SubIPC Rx: sci_gear_position_          : {}", input.sci.sci_gear_position);

    VLOG(log, "  SubIPC Rx: sci_whl_housing_height_fl_  : {}", input.sci.sci_whl_housing_height_fl);
    VLOG(log, "  SubIPC Rx: sci_whl_housing_height_fr_  : {}", input.sci.sci_whl_housing_height_fr);
    VLOG(log, "  SubIPC Rx: sci_whl_housing_height_rl_  : {}", input.sci.sci_whl_housing_height_rl);
    VLOG(log, "  SubIPC Rx: sci_whl_housing_height_rr_  : {}", input.sci.sci_whl_housing_height_rr);

    VLOG(log, "  SubIPC Rx: sci_wheel_base_             : {}", input.svpi.cvpi_wheelbase);
    VLOG(log, "  SubIPC Rx: sci_edr_flag_               : {}", input.sci.sci_eol_edr_flag);
    VLOG(log, "  SubIPC Rx: sci_number_of_cameras       : {}", input.sci.sci_number_of_cameras);
    
    for (uint8_t i = 0; i < input.sci.sci_number_of_cameras; ++i)
    {
      VLOG(log, "  camera[{}] intrinsic parameters:", i);
      VLOG(log, "    SubIPC Rx: sci_camera_type   : {}", input.sci.sci_camera[i].sci_camera_type);

      VLOG(log, "    SubIPC Rx: sci_pitch_min     : {}", input.sci.sci_camera[i].sci_pitch_min);
      VLOG(log, "    SubIPC Rx: sci_pitch_max     : {}", input.sci.sci_camera[i].sci_pitch_max);
      VLOG(log, "    SubIPC Rx: sci_yaw_min       : {}", input.sci.sci_camera[i].sci_yaw_min);
      VLOG(log, "    SubIPC Rx: sci_yaw_max       : {}", input.sci.sci_camera[i].sci_yaw_max);
      VLOG(log, "    SubIPC Rx: sci_roll_min      : {}", input.sci.sci_camera[i].sci_roll_min);
      VLOG(log, "    SubIPC Rx: sci_roll_max      : {}", input.sci.sci_camera[i].sci_roll_max);

      VLOG(log, "    SubIPC Rx: sci_length        : {}", input.sci.sci_camera[i].sci_camera_longitudinal_position);
      VLOG(log, "    SubIPC Rx: sci_width         : {}", input.sci.sci_camera[i].sci_camera_lateral_position);
      VLOG(log, "    SubIPC Rx: sci_camera_height : {}", input.sci.sci_camera[i].sci_camera_height);

      VLOG(log, "    SubIPC Rx: ici_fx            : {}", input.ici.ici_camera[i].ici_fx);
      VLOG(log, "    SubIPC Rx: ici_fy            : {}", input.ici.ici_camera[i].ici_fy);
      VLOG(log, "    SubIPC Rx: ici_cx            : {}", input.ici.ici_camera[i].ici_cx);
      VLOG(log, "    SubIPC Rx: ici_cy            : {}", input.ici.ici_camera[i].ici_cy);
      VLOG(log, "    SubIPC Rx: ici_distort:");

      int num_coeffs = sizeof(input.ici.ici_camera[i].ici_distort) / sizeof(input.ici.ici_camera[i].ici_distort[0]);
      for (int k = 0; k < num_coeffs; ++k)
      {
        VLOG(log, "      d[{}]:   {}", k, input.ici.ici_camera[i].ici_distort[k]);
      }
    }
  }
  VLOG(log, "");

  PHANTOM_LOG("{}", log);
  
  msg_data_ = input; 
}

void CalibrationWrapper::postCamerasData(std::shared_ptr<phantom_ai::CamerasData> cameras_data)
{
  cameras_data->t()     = T_NOW;
  cameras_data->t_hw()  = TIME_HW(cameras_data->t_hw(), cameras_data->t_src());

  PHANTOM_AI_PROFILE_FUNCTION;

  auto calib_data = std::make_shared<phantom_ai::calib::CalibrationInputMsgData>(cameras_data->t_hw());

  calib_data->frame() = cameras_data->frame();
  calib_data->t_hw()  = TIME_HW(calib_data->t_hw(), calib_data->t_src());

  for (uint8_t i = 0; i < msg_data_.sci.sci_number_of_cameras; ++i)
  {
    auto camera_id = phantom_ai::calib::ConvertABCameraIdToPhantomCameraId(msg_data_.sci.sci_camera[i].sci_camera_type).front();
    calib_data->cameras_.push_back(camera_id);

    // Extrinsics
    calib_data->length_.push_back(msg_data_.sci.sci_camera[i].sci_camera_longitudinal_position);
    calib_data->width_.push_back(msg_data_.sci.sci_camera[i].sci_camera_lateral_position);
    calib_data->height_.push_back(msg_data_.sci.sci_camera[i].sci_camera_height);

    calib_data->roll_min_.push_back(msg_data_.sci.sci_camera[i].sci_roll_min);
    calib_data->roll_max_.push_back(msg_data_.sci.sci_camera[i].sci_roll_max);

    calib_data->pitch_min_.push_back(msg_data_.sci.sci_camera[i].sci_pitch_min);
    calib_data->pitch_max_.push_back(msg_data_.sci.sci_camera[i].sci_pitch_max);

    calib_data->yaw_min_.push_back(msg_data_.sci.sci_camera[i].sci_yaw_min);
    calib_data->yaw_max_.push_back(msg_data_.sci.sci_camera[i].sci_yaw_max);

    // Intrinsics
    calib_data->fx_.push_back(msg_data_.ici.ici_camera[i].ici_fx);
    calib_data->fy_.push_back(msg_data_.ici.ici_camera[i].ici_fy);
    calib_data->cx_.push_back(msg_data_.ici.ici_camera[i].ici_cx);
    calib_data->cy_.push_back(msg_data_.ici.ici_camera[i].ici_cy);
    std::vector<double> distort_coeffs;
    distort_coeffs.assign(std::begin(msg_data_.ici.ici_camera[i].ici_distort), std::end(msg_data_.ici.ici_camera[i].ici_distort));
    calib_data->distortion_.push_back(distort_coeffs);
  }

  calib_data->environment_     = phantom_ai::calib::ConvertToPhantomEnvironment(msg_data_.sci.sci_calibration_target_type);
  calib_data->gear_position_   = phantom_ai::calib::ConvertToPhantomGearType(msg_data_.sci.sci_gear_position);
  calib_data->veh_door_status_ = phantom_ai::calib::ConvertToPhantomVehDoorStatus(msg_data_.sci.sci_vehicle_door_status);

  calib_data->request_         = msg_data_.sci.sci_routine_request;
  calib_data->edr_flag_        = msg_data_.sci.sci_eol_edr_flag;
  calib_data->wheel_base_m_    = msg_data_.svpi.cvpi_wheelbase * CM_TO_M;
  calib_data->mode_            = static_cast<phantom_ai::calib::PhantomCameraStaticCalibMode>(msg_data_.sci.sci_calibration_mode);

  calib_data->whl_housing_height_fl_ = msg_data_.sci.sci_whl_housing_height_fl;
  calib_data->whl_housing_height_fr_ = msg_data_.sci.sci_whl_housing_height_fr;
  calib_data->whl_housing_height_rl_ = msg_data_.sci.sci_whl_housing_height_rl;
  calib_data->whl_housing_height_rr_ = msg_data_.sci.sci_whl_housing_height_rr;

  calib_data->interface_version_     = 11;

  POST_CAMERA_CALIBRATION_MSG(RESERVED, calib_data);
  POST_CAMERA_CALIBRATION_MSG(CAMERAS, cameras_data);

  vprint(task_config_.verbosity, "{:19s} ==================== \n", "");
  vprint(task_config_.verbosity, "[{:.3f}:{:02d}] ==================== post camera\n",
         cameras_data->tsec_hw(), cameras_data->frame() % 100);
  vprint(task_config_.verbosity, "[{:02d}:{:02d}] ==================== post calibration\n",
         calib_data->request_, calib_data->frame() % 100);
}

void CalibrationWrapper::registerStaticCalibrationResultsCallback(
        std::function<void(const phantom_ai::calib::PhantomCameraStaticCalibOutputList&)> callback_func)
{
  calibrator_main_->RegisterCallbackCalibrationResults(callback_func);
}

} // namespace phantom_ai
