/*******************************************************************************
* @file    tda4x_camera.h
* @date    03/03/2020
*
* @attention Copyright (c) 2021
* @attention Phantom AI, Inc.
* @attention All rights reserved.
*******************************************************************************/

#ifndef TDA4X_CAMERA_H_
#define TDA4X_CAMERA_H_

#include <opencv2/opencv.hpp>
#include "phantom_ai/common/phantom_vision_measurement.h"
#include "hal_base_hw.h"
#include "phantom_ai/common/camera_info.h"
#include "phantom_ai/phantom_vision2/camera_model.h"

#include <optional>
#include <unordered_map>

#ifdef AUTOBRAIN_VH_SPECIFIC_APP
// Declare all code as 'C'
extern "C" {
  #include "multi_cam_if.h" // From TDA4x repo
}
#else
// Declare all code as 'C'
extern "C" {
#if defined(SOC_J722S) || defined(SOC_J721S2) || defined(SOC_J721E)
  #include "hal_camera_iface.h"   // for high res image support
#endif
  #include "multi_cam_tivx.h" // From TDA4x repo
}
#endif

#define MAX_CAMERAS (8)
#define MAX_IMAGES (20)

namespace phantom_ai
{
  typedef struct
  {
    uint32_t dummy;
  } CameraExtraInfo;

  /// @brief Enumeration for camera format.
  /// <ul>
  /// <li>> format_yuv420sp - for YUV color space that Y(luma) component is followed by the U(chrominance) and V(chrominance)
  /// The format is composed of 4 of consecutive Y and then 1 of U or V as like YYYYCb, YYYYCr
  /// <li> format_bgr - format stands for Blue-Green-Red and its the reverse order of RGB
  /// <li> format_yuv422 - for YUV color space that composed of Y,U,V with 4:2:2 data ratio like YCbYCr,YCbYCr
  /// <li> format_rgb - format stands for Red-Green-Blue and additive color model in which red, green, blue light are added together in various ways to reproduce a broad array of colors
  /// </ul>
  enum class tda4x_camera_format : uint8_t
  {
    format_yuv420sp,
    format_bgr,
    format_yuv422,
    format_rgb
  };

  /// @brief convert tda4x camera format string to tda4x_camera_format
  /// @param str camera name string
  /// @return tda4x_camera_format
  tda4x_camera_format convert_tda4x_camera_format(const std::string& str);

  /// @brief Structure of image information for tda4x platform.
  /// <ul>
  /// <li> roi - for region of interest information which is adopted from cv::Rect type
  /// <li> output_width - for the width of output image
  /// <li> output_height - for the height of output image
  /// <li> location - physical mounted spot of the camera in the vehicle 
  /// </ul>
  typedef struct {
    cv::Rect roi;
    uint32_t output_width;
    uint32_t output_height;
    phantom_ai::CameraID location; // duplicate?
  } camera_subimage_tda4x;

  /// @brief Structure of image parameters for tda4x platform.
  /// <ul>
  /// <li> type - for device or model of the camera sensor
  /// <li> location - physical mounted spot of the camera in the vehicle
  /// <li> output_format - for output color space which is enumerated value
  /// <li> framerate - frame rate(fps) 
  /// <li> sensor_widtgh - for the width of the sensored image
  /// <li> sensor_height - for the height of the sensored image
  /// <li> subimage - arrayed image information
  /// </ul>
  typedef struct {
    std::string type;
    phantom_ai::CameraID location;
    tda4x_camera_format output_format;
    uint32_t framerate;         // logical framerate(desired fps)
    uint32_t capture_framerate; // physical framerate(capture fps)
    uint32_t sensor_width;
    uint32_t sensor_height;
    std::vector<camera_subimage_tda4x> subimage;
    bool dual_logging;
  } camera_params_tda4x;

  void readTda4xCamerasParameters(std::string camera_capture_config_filename,
                                  std::vector<camera_params_tda4x>& cam_params_arr,
                                  CameraInfoList& cam_info_list,
                                  std::vector<int>& annotation_image_widths,
                                  const bool& enable_fullres_logging,
                                  std::string target_system = "tda4x");

  /// @brief Structure of result for processed camera image
  /// <ul>
  /// <li> image - for gathered image, the type of this element is adopted by cv::Mat
  /// <li> id - physical mounted spot of the camera in the vehicle
  /// <li> timestamp - time stamp value of the image
  /// <li> status - status of the image
  /// </ul>
  typedef struct
  {
    cv::Mat image;
    phantom_ai::CameraID id;
    double timestamp;
    uint32_t frame_number;
    cv::Mat extra_data_mat;
    cv::Rect crop_roi;
    CAMERA_STATUS status;
  } CameraResults;

  /// @brief A functor/interface that trigger (tivx) camera driver with given camera parameters from the yaml file
  /// <p>
  /// The major functionalities of this class includes 
  /// <ul>
  /// <li> initiate sdk camera driver with using given parameters from the yaml file
  /// <li> trigger camera execution loop by creating a new thread
  /// <li> gather camera output and send it by callback 
  /// </ul> 
  class Tda4xCamera
  {
  public:
    /// @brief Tda4xCamera Constructor for utilizing the configured camera 
    /// The parameterized configurations of the camera to be delivered by yaml file 
    /// Initiation, triggering run time execution loop, gathering output of the camera to be done
    /// @param config_arr - configuration arrays for mounted cameras @dir in @unit NA @scale NA @range NA
    Tda4xCamera(std::vector<phantom_ai::camera_params_tda4x>& config_arr, bool use_full_res_logging = false);

    /// @brief Tda4xCamera Destructor for utilizing the configured camera 
    ~Tda4xCamera();

    /// @brief Update the privatge flag at the condition for starting video capture
    /// @return void
    void startStreaming();

    /// @brief Update the privatge flag at the condition for stopping video capture
    /// @return void
    void stopStreaming();

    /// @brief Register function pointer for rx callback to provide downscaled image, both main and crop 
    /// @param callback_func - Callback function @dir in @unit N/A @scale N/A @range N/A
    /// @return void
    void registerRxCallback(std::function<void(std::vector<CameraResults>&, const std::unordered_map<CameraID, std::shared_ptr<CameraExtraInfo>>&)> callback_func);

    /// @brief Return result of correctness for each location of the cameras
    /// @param wrong_camera_connected - result buffer array for wrong camera location, @dir in @unit N/A @scale N/A @range [0 ,1]
    /// @param num_of_cam - The number of the camera @dir in @unit N/A @scale N/A @range N/A
    /// @return void
    void checkCameraLocation(uint8_t wrong_camera_connected[], uint8_t num_of_cam);

    /// @brief Return status of the each camera connection
    /// @param cam_result - Received camera data @dir in @unit N/A @scale N/A @range N/A
    /// @return void
    bool isCameraConnected(phantom_ai::CameraResults cam_result);

#if defined(SOC_J722S) || defined(SOC_J721S2) || defined(SOC_J721E)
    /// @brief Converts resized ROI coordinates back to original resolution coordinates.
    /// @param cam_id - Camera ID
    /// @param roi_resized - Resized ROI coordinates
    /// @param cam_models - Camera Model List
    /// @return The ROI coordinates in original resolution.
    cv::Rect resizedROIToOriginalCoords(CameraID cam_id, const cv::Rect &roi_resized, CameraModelListS& cam_models);

    /// @brief Retrieves the full resolution frame ROI for a specific frame.
    /// @param cam_id - Camera ID
    /// @param frame_number - The frame number to retrieve ROI for. @dir in @unit count @scale N/A @range [0, N/A]
    /// @param roi - ROI region to extract at full resolution @dir in @unit cv::Rect @scale N/A @range N/A
    /// @param fmt - Pixel format of the frames @dir in enumeration @unit PixelFormat @scale N/A @range N/A
    /// @param cam_models - Camera Model List
    /// @return std::optional<FrameData> - Frame data for the ROI if successful, std::nullopt otherwise.
    std::optional<FrameData> getFullResolutionFrameROI(CameraID cam_id, uint32_t frame_number, const cv::Rect &roi, PixelFormat fmt, CameraModelListS& cam_models);
#endif

    /// @brief Return result of high latency failure
    /// @param timestamp - Received image timestamp from the sdk @dir in @unit msec @scale N/A @range N/A
    /// @return CAMERA_STATUS
    CAMERA_STATUS CheckHighLatency(uint64_t timestamp);
#if defined(ENABLE_GSTREAMER)
    void getMosaicDimensions(uint32_t& width, uint32_t& height);

    void registerEncoderCallback(EncoderFeedFrameCallback callback);
#endif
    private:

    /// @brief Callback function object
    /// This object to be called by the application whenever new camera image is received.
    std::function<void(std::vector<CameraResults>&, const std::unordered_map<CameraID, std::shared_ptr<CameraExtraInfo>>&)> image_callback_;

    /// @brief Function to store received image data into the result buffer
    /// The configuration of the camera information is previously done by the init in camera_wrapper. 
    /// tivx API to be hooked to grab the image data.
    /// Stamp the present time information for making fps afterward.
    /// Hook the callback function.
    void runCameraLoop();

    /// @brief Function to convert the type of the camera according to the given parameter 
    /// The type of the camera is declared in the sdk-multi_cam_tivx.h- 
    /// @param name - Given sensor type @dir in @unit string @scale N/A @range N/A, one of "imx390","ar0820","ar0233_gw5200","ox08b40","ox03c10" shall be given 
    /// @return TIVX_CAMERA_TYPES - camera types defined in the SDK. @unit enum @scale N/A @range [0, 4] 
    TIVX_CAMERA_TYPES convertSensorType(const std::string name);

#if defined(SOC_J722S) || defined(SOC_J721S2) || defined(SOC_J721E)
    /// @brief Maps a HAL_CAM_CROP_ID to a subimage index.
    /// This function associates a given crop_id with a subimage index, ensuring that the index is within valid bounds.
    /// @param crop_id - The crop identifier @unit enum @scale N/A @range [0, HAL_CAM_CROP_MAX-1]
    /// @param subimage_idx - The index of the subimage to associate with the crop ID @unit integer @scale N/A @range [0, HAL_CAM_CROP_MAX-1]
    void mapCropIdToSubImageIdx(HAL_CAM_CROP_ID crop_id, uint32_t subimage_idx);

    /// @brief Retrieves the subimage index associated with a given HAL_CAM_CROP_ID.
    /// This function returns the subimage index corresponding to the provided crop_id.
    /// @param crop_id - The crop identifier to look up the subimage index for @unit enum @scale N/A @range [0, HAL_CAM_CROP_MAX-1]
    /// @return uint32_t - The subimage index associated with the crop_id @unit integer @scale N/A @range [0, HAL_CAM_CROP_MAX-1]
    uint32_t halCropIdToSubImgIdx(HAL_CAM_CROP_ID crop_id);
#endif

    /// @brief Void pointer for init function in the sdk-multi_cam_tivx.c- with parameters of the camera 
    /// This can be utilized for the APIs of the sdk
    void *handle_;

    /// @brief Mutually shared object between classes
    /// This object to be used for initializing the ti hardware driver
    HalBaseHw *hw_ptr_;

    /// @brief Thread for running runCameraLoop function
    std::thread main_thread_;

    /// @brief Variable for forsaking the camera loop
    bool quit_loop_;

    /// @brief Variable for number of frame counter to calculate frame rate
    /// @unit number of times @scale 1 @range N/A
    uint32_t frame_counter_;

    /// @brief N number of frames where 1 frame is kept and rest are dropped
    /// @unit N/A @scale 1 @range N/A
    uint32_t frame_drop_idx_;

    /// @brief Variable for holding present camera format
    tda4x_camera_format output_format_;

    /// @brief Variable for previous time stamp value
    /// @unit usec @scale 1 @range N/A
    double ts_image_last_;

    /// @brief Variable for remaining time exceeding the previous period
    /// @unit usec @scale 1 @range N/A
    double ts_remainder_;

    /// @brief Variable for minimum time interval 
    /// @unit sec @scale 1 @range N/A
    double ts_min_interval_;

    /// @brief State of whether camera streaming is started or not
    bool output_enable_;

    /// @brief Number of the mounted camera
    /// @unit natural number of camera @scale 1 @range N/A
    uint32_t camera_count_;

    /// @brief Parameters of the camera that structure of the type is defined in the sdk-multi_cam_tivx.h-
    Camera_Params params_[MAX_CAMERAS];

#if defined(SOC_J722S) || defined(SOC_J721S2) || defined(SOC_J721E)
    /// @brief Array mapping HAL_CAM_CROP_ID to subimage indices.
    uint32_t cropid_to_subimage_map_[HAL_CAM_CROP_MAX];
#endif

    /// @brief Number of the images
    /// @unit natural number of images @scale 1 @range N/A
    uint32_t image_count_;

    /// @brief Array for the camera locations
    std::vector<std::vector<phantom_ai::CameraID>> locations_;

    /// @brief Indicate if the dual logging feature active or not.
    bool dual_logging_[MAX_CAMERAS];

    bool use_fullres_logging_;
  };

#ifdef ENABLE_GET_CAM_EXTRA_DATA
//ENABLE_FETCH_CAMERA_EXTRA_DATA will be used
#define ENABLE_FETCH_CAMERA_EXTRA_DATA
#define CAMERA_EXTRA_DATA_INDEX_OFFSET IDX_OFFSET_EXTRA_DATA0
#endif

}  // namespace phantom_ai

#endif //TDA4X_CAMERA_H_
