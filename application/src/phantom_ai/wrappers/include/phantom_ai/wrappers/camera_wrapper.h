/*********************************************************************
* @file    camera_wrapper.h
* @date    03/06/2021
*
* @attention Copyright (c) 2021
* @attention Phantom AI, Inc.
* @attention All rights reserved.
*********************************************************************/

#ifndef CAMERA_WRAPPER_H_
#define CAMERA_WRAPPER_H_

#include <optional>
#include "phantom_ai/utils/utils.h"
#include "phantom_ai/common/camera_info.h"
#if defined(PLATFORM_AM62A)
#include "hal_camera.h"
#else
#include "tda4x_camera.h"
#endif
#include "phantom_ai/phantom_vision2/camera_model.h"

//#if defined(ENABLE_GSTREAMER)
namespace phantom_ai {
    class NetTxWrapper;
}
//#endif
namespace phantom_ai
{
/// @brief This component is a wrapper of the tda4x_camera library.
///
/// <p>
/// The major functionalities of this class include
/// <ul>
/// <li> Parse camera calibration YAML file in onInit()
/// <li> Abstract services of Tda4xCamera APIs
/// <li> Sets time in seconds between uptime and Epoch time (Not implemented yet)
/// <li> Returns the CameraInfo params for camera. Check the YAML file for index.
/// </ul>
/// Input is from camera sensor; all output functionality has to be developed.
class CameraWrapper
{
public:
  CameraWrapper();
  ~CameraWrapper();

  /// @brief Load camera settings from YAML file.
  /// @param filename from which settings will be loaded @dir in
  /// @param camera_models from which image crop information will be loaded @dir in
  /// @param target_system from which target system string will be loaded @dir in
  /// @param enable_fullres_logging flag for whether the full res logging is enabled @dir in
  /// @param cam_app flag for whether the function is called from camera app @dir in
  /// @return void return
  void onInit(std::string filename, const CameraModelListS& camera_models = nullptr, std::string target_system = "tda4x", bool enable_fullres_logging = false, bool cam_app = false);

  /// @breif start video capture
  ///
  /// @return void return
  void startStreaming();

  /// @brief stop video capture
  ///
  /// @return void 
  void stopStreaming();

  /// @brief register callback function
  ///
  /// @param callback_func the function object which takes a set of camera results and a map of camera ID and camera extra information @dir in
  /// @return void
  void registerImageCallback(std::function<void(std::vector<phantom_ai::CameraResults>&, const std::unordered_map<CameraID, std::shared_ptr<CameraExtraInfo>>&)> callback_func);

  /// @brief  sets time in seconds between uptime and Epoch time.
  /// 
  /// @param offset @dir in @unit seconds @dir in
  /// @return void
  void setTimestampOffset(double offset);

  /// @brief Returns the CameraInfo params for camera.
  /// @return CameraInfoList
  CameraInfoList& getCameraInfoList();

  /// @brief return wrong connected camera list among cameras
  /// In tda4x system, always returns that all camera is connected properly
  /// 
  /// @param wrong_camera_connected list of wrong connected camera  @dir out
  /// @param num_of_cam number of cams @unit number of connected camera @scale 1 @range [0,255] @dir in
  /// @return void return
  void checkCameraLocation(uint8_t wrong_camera_connected[], uint8_t num_of_cam);

  /// @brief By using received camera frame, determine whether the camera is connected or not.
  /// 
  /// true - camera is connected properly
  /// false - camera is not connected properly
  /// 
  /// @param CameraResults Camera frame information @dir in
  /// @return boolean true:connected,false:not connected 
  bool isCameraConnected(phantom_ai::CameraResults cam_result);

  /// @brief Retrieves the full resolution frame ROI for a specific frame.
  ///
  /// @param cam_id - Camera ID
  /// @param cam_crop_roi - Camera crop region in original sensor coordinates @dir in @unit cv::Rect @scale N/A @range N/A
  /// @param frame_number - The frame number to retrieve ROI for. @dir in @unit count @scale N/A @range [0, N/A]
  /// @param resized_target_roi - Target ROI in resized image coordinate system @dir in @unit cv::Rect @scale N/A @range N/A
  /// @param fmt - Pixel format of the frames @dir in enumeration @unit PixelFormat @scale N/A @range N/A
  /// @return std::optional<FrameData> - Frame data for the ROI if successful, std::nullopt otherwise.
  std::optional<FrameData> getFullResolutionImageROI(CameraID cam_id, const cv::Rect &cam_crop_roi, uint32_t frame_number, const cv::Rect& resized_target_roi, PixelFormat fmt);

  /// @brief Retrieve multiple full-resolution images based on a base timestamp and the number of images
  ///
  /// @param base_timestamp The base timestamp to synchronize image retrieval @dir in @unit seconds @scale 1 @range [0, ∞)
  /// @param num_frames The number of images to retrieve @dir in @unit count @scale 1 @range [1, 255]
  /// @param fmt The pixel format of the retrieved images @dir in @unit enumeration @scale 1 @range [supported formats]
  /// @return std::vector<FrameData> A vector containing the retrieved full-resolution images. 
  ///         Note: As this is an asynchronous function, the number of returned images may occasionally be less than 
  ///         requested, depending on the availability of frames at the current timestamp.
  std::vector<FrameData> getFullResolutionImages(double base_timestamp, uint8_t num_frames, PixelFormat fmt);

  /// @brief Sets the absolute position of dynamic crop region.
  /// @param cam_id Camera ID
  /// @param x X coordinate of the crop region top-left corner
  /// @param y Y coordinate of the crop region top-left corner
  /// @return true if request is successfully queued, false otherwise
  /// @details The actual position change will be applied during the next camera data callback
  bool dynamicCropSetPosition(CameraID cam_id, uint32_t x, uint32_t y);

  /// @brief Gets current dynamic crop position for the specified camera.
  /// @param cam_id Camera ID
  /// @param x [out] Current x coordinate of the crop region
  /// @param y [out] Current y coordinate of the crop region
  /// @return true if position is successfully retrieved, false otherwise
  bool dynamicCropGetPosition(CameraID cam_id, uint32_t& x, uint32_t& y);

  /// @brief Moves dynamic crop position by relative offset.
  /// @param cam_id Camera ID
  /// @param dx Relative movement in x direction (positive: right, negative: left)
  /// @param dy Relative movement in y direction (positive: down, negative: up)
  /// @return true if request is successfully queued, false otherwise
  bool dynamicCropMoveRelative(CameraID cam_id, int32_t dx, int32_t dy);
#if defined(PLATFORM_AM62A)
  /// @brief Gets current dynamic crop information for the specified Camera ID.
  /// @param cam_id Camera ID
  /// @param enabled [out] Current enable/disable status of dynamic crop
  /// @param cam_img_info [out] Current camera image information including crop region and output size
  /// @return true if information is successfully retrieved, false otherwise
  bool dynamicCropGetInfo(CameraID cam_id, bool& enabled, hal_camera_image& cam_img_info);
#endif
  /// @brief Gets the valid range for dynamic crop position settings.
  /// @param cam_id Camera ID
  /// @param valid_range [out] Valid position range where x=0~width, y=0~height represent max coordinates
  /// @return true if valid range is successfully retrieved, false otherwise
  bool dynamicCropGetValidRange(CameraID cam_id, cv::Rect& valid_range);
//#if defined(ENABLE_GSTREAMER)
  void initGstEncoder(phantom_ai::NetTxWrapper* net_tx_wrapper);

  void getMosaicDimensions(uint32_t& width, uint32_t& height);

  void registerEncoderCallback(EncoderFeedFrameCallback callback);
//#endif
  typedef enum {
#if defined(PLATFORM_AM62A)
      WRAPPER_CAM_SUCCESS = HAL_CAM_SUCCESS,
      WRAPPER_CAM_DATA_LOSS = HAL_CAM_DATA_LOSS,
      WRAPPER_CAM_DATA_TIMEOUT = HAL_CAM_DATA_TIMEOUT,
      WRAPPER_CAM_LOW_FRAME_RATE = HAL_CAM_LOW_FRAME_RATE,
      WRAPPER_CAM_LATENCY = HAL_CAM_RESERVED1,
      WRAPPER_CAM_INIT_ERROR = HAL_CAM_INIT_ERROR,
      WRAPPER_CAM_PROCESS_ERROR = HAL_CAM_PROCESS_ERROR,
      WRAPPER_CAM_RESERVED1 = HAL_CAM_RESERVED2,
      WRAPPER_CAM_RESERVED2 = HAL_CAM_RESERVED3,
#else
      WRAPPER_CAM_SUCCESS = HIK_CAM_SUCCESS,
      WRAPPER_CAM_DATA_LOSS = HIK_CAM_DATALOSS_FAIL,
      WRAPPER_CAM_DATA_TIMEOUT = HIK_CAM_TIMEOUT_FAIL,
      WRAPPER_CAM_LOW_FRAME_RATE = HIK_CAM_FRAME_RATE_LOW_FAIL,
      WRAPPER_CAM_LATENCY = HIK_CAM_LATENCY_FAIL,
      WRAPPER_CAM_INIT_ERROR = HIK_CAM_INIT_FAIL,
      WRAPPER_CAM_PROCESS_ERROR = HIK_CAM_PROCESS_FAIL,
      WRAPPER_CAM_RESERVED1 = HIK_CAM_RELEASE_FAIL,
      WRAPPER_CAM_RESERVED2 = HIK_CAM_RELEASE_FAIL,
#endif
  } WRAPPER_CAM_STATUS;

private:
  /// @brief when new camera frame is received, this callback function is called.
  ///
  /// @param image_arr - received camera frame information array @dir in
  /// @param extra_info - mapped camera ID to camera extra information @dir in
  /// @return void return
  void imageCallback(std::vector<phantom_ai::CameraResults>& image_arr,  
                       const std::unordered_map<phantom_ai::CameraID, std::shared_ptr<phantom_ai::CameraExtraInfo>>& extra_info);

  /// @brief  callback function object
  ///
  /// This function object is called when new camera frame is received.
  std::function<void(std::vector<phantom_ai::CameraResults>&, const std::unordered_map<phantom_ai::CameraID, std::shared_ptr<phantom_ai::CameraExtraInfo>>&)> rx_frame_callback_;

  /// @brief CameraInfoList
  /// CameraInfo class contains camera information such as width,height,ROI...
  CameraInfoList cam_info_list_;

  /// @brief TI specific camera control related class pointer
  #if defined(PLATFORM_AM62A)
    phantom_ai::HalCamera *hw_cam_;
  #else
    phantom_ai::Tda4xCamera *hw_cam_;
  #endif
 
  /// @brief Timestamp value of last recevied camera frame
  phantom_ai::TimeStamp last_ts_;
  /// @brief  it seems that this variable is not used anymore. 
  phantom_ai::TimeStamp last_print_;

  /// @brief CameraModelList
  CameraModelListS camera_models_;
};

} // namespace phantom_ai


#endif //CAMERA_WRAPPER_H_
