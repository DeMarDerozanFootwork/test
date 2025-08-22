/*******************************************************************************
* @file    network_tx_wrapper.h
* @date    11/26/2019
*
* @attention Copyright (c) 2019
* @attention Phantom AI, Inc.
* @attention All rights reserved.
*******************************************************************************/

#ifndef NETWORK_TX_WRAPPER_H_
#define NETWORK_TX_WRAPPER_H_

#include <opencv2/opencv.hpp>
#include "phantom_ai/core/time_stamp.h"
#include "phantom_ai/embedded_network/embedded_phantom_net.h"
#include "phantom_ai/common/phantom_vision_measurement.h"
#include "phantom_ai/phantom_vision2/phantomnet_data.h"
#include "phantom_ai/common/phantom_vision_dynamic_calibration_output.h"
#include "phantom_ai/common/functional_safety_output.h"
#include <phantom_ai/common/phantom_camera_extra_input.h>
#include <phantom_ai/tools/image_conversion_helper.h>

#include "multi_cam_tivx.h"

namespace phantom_ai
{
typedef enum
{
  tx_format_yuv420sp,
  tx_format_yuv422,
  tx_format_rgb,
  tx_format_bgr,
  tx_format_encoded
} embedded_streamer_format;

  /**
   * @brief This component provides Network Message Packet Tx Interface for embedded systems (ie non-ROS based) functionalities.
   * The major functionalities of this class include
   * - initialize embedded network Tx interface
   * - send message with single image from camera
   * - send message with main and crop image from camera
   * - send message with variable number of image from cameras
   * - add any vehicle state data to message
   * - add any vision measurement results to message
   * - add any camera calibration to message. All of these values are static, so only need to set once
   * - send message using raw byte format
   * - convert vision measurement result to embedded network output
   * Convience wrapper for Embedded Network module
   * @author Phantom AI
   * @version 1.0
   * @updated 14-Mar-2022 12:47:23 AM
   */
  class NetTxWrapper
  {
  public:
    NetTxWrapper();
    ~NetTxWrapper();

    void onInit(std::string stream_config_filename = "tda4x/stream_server.yaml",
                std::string camera_capture_config_filename = "",
                std::string target_system = "tda4x",
                bool cam_app = false);

    // Send message with single image from camera
    bool sendCameraFrame(cv::Mat &camera, double ts, uint32_t frame_count);

    // Send message with variable number of image from cameras
    bool sendImageFrame(std::vector<cv::Mat>& img_arr, std::vector<phantom_ai::CameraID>& cam_location, double ts, uint32_t frame_count);

    // Add any vehicle state data to message
    bool queueVehicleData(EmbeddedNet_VehicleInfo &data);

    // Add any vision measurement results to message
    bool queueVisionData(const EmbeddedNet_PhantomVisionList &data);

    // Add any dynamic calibration results to message
    bool queueDynamicCalibrationData(const EmbeddedNet_DynamicCalibrationResultList &data);

    // Add any camera calibration to message. All of these values are static, so only need to set once
    bool queueCameraCalib(const EmbeddedNet_CameraCalibration &data);

    // Add any CAN TX (output) data to message.
    bool queueCanTxData(const EmbeddedNet_CanFrames &data);

    // Add any Vision Viz data to message
    bool queueVisionVizData(const cv::Mat& data);

    // Add any Fusa data to message
    bool queueFusaData(const EmbeddedNet_FunctionalSafetyOutputList &data);
  
    //Add camera extra data to message
    bool queueCameraExtraData(const EmbeddedNet_CameraExtraInfoListTopLevel &data);

    // Add any Hba data to message
    bool queueHbaData(const EmbeddedNet_PhantomVisionHBAList &data);

    // Add any AEB data to message
    void queueAebData(const EmbeddedNet_PhantomVisionAEBList &data);

    // Send message using raw byte format
    bool sendRawFrame(Embedded_Frame& input);

    // helper functions
    bool convertVisionFrame(const phantom_ai::PhantomVisionMeasurement &input, EmbeddedNet_PhantomVisionResult *output);

    void convertDynamicCalibrationFrame(const phantom_ai::PhantomVisionDynamicCalibrationOutputList &input,
                                        EmbeddedNet_DynamicCalibrationResult *output);

    bool convertFusaDataFrame(const phantom_ai::FunctionalSafetyOutput &input, EmbeddedNet_FunctionalSafetyOutput *output);

    bool convertCameraExtraDataFrame(const PhantomCameraExtarInputListS &input, 
                                     EmbeddedNet_CamerasExtraInfoList *output);

    bool convertHbaDataFrame(const phantom_ai::PhantomVisionHBA &input, EmbeddedNet_PhantomVisionHBA *output);
    
    bool convertAebDataFrame(const phantom_ai::PhantomVisionAEB &input, EmbeddedNet_PhantomVisionAEB *output);

    void sendVisionViz(const cv::Mat& data, const double& ts);

    void resetFrameBuffer(const bool vision_only_dual_logging = false, const bool init = false, const std::vector<int>& annotation_image_widths = {});

    void enableFullresLogging() { use_fullres_logging_ = true; };
#if defined(ENABLE_GSTREAMER)
    // Send message with variable number of encoded image data
    bool sendEncodedImageFrame(std::vector<phantom_ai::CameraID>& cam_location, double ts);

    bool initGstEncoder(uint32_t width, uint32_t height);

    static EncoderFeedFrameCallback getFrameFeedCallback();

    void enableEncodeImgByGst(bool enable) { gst_h264encoding_enabled_ = enable; }

    void registerEncodingMosaicImg(std::function<int(uint8_t*, uint32_t, uint32_t*, uint32_t*, uint32_t*, uint32_t*)> callback_func);
#endif
    // Public variables, in case
    Embedded_Frame frame_;

    bool is_client_;

  private:

    void repackI420toNV12(uint8_t *src, uint8_t *dst, uint32_t width, uint32_t height);
    EmbeddedNet_PhantomFormat getImageFormat();
    char* getImageName(phantom_ai::CameraID loc);
    EmbeddedNet_PhantomFormat getCameraFormat();
    void updateStats(bool tx_status);
    uint32_t packBGRData(EmbeddedNet_PhantomHeader_Info *item, cv::Mat &input, uint8_t* dest, uint32_t offset);
    uint32_t packYuvData(EmbeddedNet_PhantomHeader_Info *item, cv::Mat &input, uint8_t* dest, uint32_t offset);
    uint32_t packYuv420Data(EmbeddedNet_PhantomHeader_Info *item, cv::Mat &input, uint8_t* dest, uint32_t offset);

    void appendVehicleData(uint32_t &offset);
    void appendVisionData(uint32_t &offset);
    void appendDynamicCalibrationData(uint32_t &offset);
    void appendCamCalibData(uint32_t &offset);
    void appendCanTxData(uint32_t &offset);
    void appendPhantomVisionViz(uint32_t &offset);
    void appendFusaData(uint32_t &offset);
    void appendCameraExtraData(uint32_t &offset);
    void appendHbaData(uint32_t &offset);
    void appendAebData(uint32_t &offset);

    void resizeDataBuffer(const uint32_t additional_len);

    bool vision_viz_message_recevied_ = false;
    
    void *ti_handle_;
    bool running_;

    // Used to resize data buffer. Shall be used only one time at most for initialization
    std::mutex data_buf_mutex_;
    std::mutex viz_data_mutex_;
    std::mutex send_frame_mutex_;

    cv::Size image_size_;
    embedded_streamer_format output_format_;

    EmbeddedNet_VehicleInfo veh_data_;
    EmbeddedNet_PhantomVisionList vision_data_;
    EmbeddedNet_DynamicCalibrationResultList dynm_calib_data_;
    EmbeddedNet_FunctionalSafetyOutputList fusa_data_;
    EmbeddedNet_CameraExtraInfoListTopLevel camera_extra_data_;
    EmbeddedNet_PhantomVisionHBAList hba_data_;
    EmbeddedNet_PhantomVisionAEBList aeb_data_;
    cv::Mat vision_viz_data_;
    bool vision_data_valid_;
    bool dynm_calib_data_valid_;
    bool vision_viz_data_valid_;
    bool fusa_data_valid_;
    std::atomic<bool> camera_extra_data_valid_;
    bool hba_data_valid_;
    bool aeb_data_valid_;
    EmbeddedNet_CameraCalibration cam_info_;
    EmbeddedNet_CanFrames can_tx_data_;    
#if defined(ENABLE_GSTREAMER)
    std::function<int(uint8_t*, uint32_t, uint32_t*, uint32_t*, uint32_t*, uint32_t*)> encoding_mosaicImg_gst_callback_func_;
#endif
    // Stats
    uint32_t tx_frame_cnt_;
    uint32_t rx_frame_cnt_;
    uint32_t error_cnt_;
    double stat_print_interval_sec_;
    phantom_ai::TimeStamp last_print_time_;

    std::string encoding_;
    CameraInfoList cam_info_list_;

    bool publish_only_valid_objects_;
    bool use_fullres_logging_;
#if defined(ENABLE_GSTREAMER)
    bool gst_h264encoding_enabled_ = false;
#endif
  };

}  // namespace phantom_ai

#endif //NETWORK_TX_WRAPPER_H_
