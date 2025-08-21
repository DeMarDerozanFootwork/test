/*******************************************************************************
 * @file    calibration_wrapper.h
 * @date    03/01/2023
 *
 * @attention Copyright (c) 2023
 * @attention Phantom AI, Inc.
 * @attention All rights reserved.
 *******************************************************************************/

#ifndef __PHANTOMCALIBRATION_CALIBRATION_WRAPPER_H__
#define __PHANTOMCALIBRATION_CALIBRATION_WRAPPER_H__

#include "phantom_ai/core/time_stamp.h"
#include "phantom_ai/core/timed_critical_section.h"
#include "phantom_ai/phantom_vision2/vision_constants.h"
#include "phantom_ai/phantom_vision2/vision_params.h"
#include "phantom_ai/phantom_vision2/cameras_data.h"

#include "phantom_ai/phantom_static_calibration/calibration_data.h"
#include "phantom_ai/phantom_static_calibration/calibration_main.h"

#include "phantom_ai/protobuf_msgs/calibration_msg.h"

namespace phantom_ai
{
  /// @brief This component provides interfaces of eol static calibration algorithm
  /// <p>
  /// The major functionalities of this class include
  /// <ul> public
  /// <li> initialize vision
  /// <li> reset vision
  /// <li> check camera frame rate
  /// <li> check image size
  /// <li> get yaw rate bias
  /// <li> post vehicle state data
  /// <li> post vision measurement data
  /// <li> post camera data
  /// <li> decode TIDL(TI Deep Learning) output
  /// <li> register vision measurement callback
  /// <li> register CAN packer callback
  /// <li> set classification for front model
  /// <li> set classification for front side model
  /// <li> access point for params_
  /// <li> access point for &vision_params
  /// <li> register publishers of vision messages
  /// <li> clear data
  /// <li> visualize CNN result
  /// </ul>
  class CalibrationWrapper
  {
  public:
    /// @brief Default constructor
    /// @param none
    CalibrationWrapper();
    /// @brief Default destructor
    /// @param none
    ~CalibrationWrapper();

    /// @brief initialize CalibrationWrapper instance
    /// @param filename – CalibrationWrapper configuration file name @dir in
    /// @return void
    void onInit(std::string &filename);

    /// @brief reset CalibrationWrapper instance
    /// @param none
    /// @return void
    void reset();

    /// @brief check camera frame rate
    /// @param timestamp @dir in @unit double
    /// @return bool - true: valid camera frame rate, false: invalid camera frame rate
    bool checkCameraFrameRate(double t);

    /// @brief check image size
    /// @param input_size @dir in @unit cv::Size&
    /// @param roi @dir in @unit cv::Rect&
    /// @return bool - true: valid input image size, false: invalid input image size
    bool checkImageSize(const cv::Size &input_size, cv::Rect &roi);

    /// @brief post camera data
    /// @param cameras_data @dir in phantom_ai::CamerasData&
    /// @return void
    void postCamerasData(std::shared_ptr<phantom_ai::CamerasData> cameras_data);

    /**
     * @brief register phantom static calibration results callback to publish results     * 
     * @param callback_func - application's callback function reference @dir in
     */
    void registerStaticCalibrationResultsCallback(
        std::function<void(const phantom_ai::calib::PhantomCameraStaticCalibOutputList &)> callback_func);

    /// @brief get function for params_ member variable's value
    /// const function stops this function to modify member variable inside.
    /// @param none
    /// @return phantom_ai::VisionParams - vision parameters
    phantom_ai::calib::CalibrationParams params() const { return params_; }

    /// @brief get function for params_ member variable's reference
    /// @param none
    /// @return phantom_ai::VisionParams& - vision parameters reference
    phantom_ai::calib::CalibrationParams &params() { return params_; }

    /// @brief get function for camera models
    /// @param none
    /// @return phantom_ai::CameraModelListS - camera models such as calibration information.
    phantom_ai::CameraModelListS camera_models() { return calibrator_main_->GetCameraModels(); }

    /// @brief a wrapper function to set critical section
    /// @param critical_section - critical section pointer where you want to use critical section management
    /// @return void
    void set_critical_section(phantom_ai::TimedCriticalSection *critical_section);

    /// @brief a wrapper function to wait for the other releasing critical section entry ticket
    /// @param none
    /// @return void
    void wait_critical_section();

    /// @brief Set the Static Calibration Input Message object
    /// @param scm  - measurement inputs @dir in
    void setStaticCalibrationInputMsg(const StaticCalibrationDataInput &input);

  protected:

    /// @brief reset publishing messages
    /// @param none
    /// @return void
    void clearData();

  private:
    /// @brief CalibrationMain instance
    std::shared_ptr<phantom_ai::calib::CalibrationMain> calibrator_main_;
    
    /// @brief static calibration parameters
    phantom_ai::calib::CalibrationParams params_;
 
    /// @brief a flag to indicate camera inactive or active
    /// 0: inactive, 1: active
    int cameras_flag_;

    /// @brief a counter to make vision stack publsh messsages and visualize phantomnet
    ///        after the end of processing according to the number of active cameras.
    int reset_counter_;

    /// @brief timestamp of the last captured image
    /// @unit second
    double tsec_image_last_;

    /// @brief task configuration as ROS node
    phantom_ai::calib::RosNodeParam task_config_;

    /// @brief static calibration input message
    StaticCalibrationDataInput msg_data_;
  };

}

#endif
