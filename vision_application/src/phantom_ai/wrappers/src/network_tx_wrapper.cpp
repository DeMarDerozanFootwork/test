/*********************************************************************
* @file    network_tx_wrapper.cpp
* @date    11/26/2019
*
* @attention Copyright (c) 2019
* @attention Phantom AI, Inc.
* @attention All rights reserved.
*********************************************************************/

#include "phantom_ai/core/log.h"
#include "phantom_ai/core/yaml.h"

#include "phantom_ai/wrappers/network_tx_wrapper.h"
#include "phantom_ai/utils/utils.h"

#if defined(PLATFORM_AM62A)
#include "hal_camera.h"
#else
#include "tda4x_camera.h"
#endif
#if defined(ENABLE_GSTREAMER)
#include "phantom_gst.h"
#endif

#ifndef SEC2USEC
#define SEC2USEC 1e6
#endif

namespace phantom_ai
{
  NetTxWrapper::NetTxWrapper() :
    ti_handle_(nullptr),
    running_(false),
    data_buf_mutex_{},
    viz_data_mutex_{},
    send_frame_mutex_{},
    image_size_{},
    output_format_(tx_format_yuv422),
    veh_data_{},
    vision_data_{},
    dynm_calib_data_{},
    fusa_data_{},
    camera_extra_data_{},
    hba_data_{},
    vision_viz_data_{},
    vision_data_valid_(false),
    dynm_calib_data_valid_(false),
    vision_viz_data_valid_(false),
    fusa_data_valid_(false),
    camera_extra_data_valid_(false),
    hba_data_valid_(false),
    cam_info_{},
    can_tx_data_{},
    tx_frame_cnt_(0),
    rx_frame_cnt_(0),
    error_cnt_(0),
    stat_print_interval_sec_(0.0),
    last_print_time_(),
    publish_only_valid_objects_(false),
#if defined(ENABLE_GSTREAMER)
    encoding_mosaicImg_gst_callback_func_(nullptr),
#endif
    use_fullres_logging_(false)
  {
#if defined(CUSTOMER_RELEASE_MODE)
    publish_only_valid_objects_ = true;
#endif
  }

  NetTxWrapper::~NetTxWrapper()
  {
    if (running_)
    {
      PHANTOM_LOG("Stopping transmitter");
      if (is_client_)
      {
        embeddedNetStop(ti_handle_);
      }
      else
      {
        embeddedServerNetStop(ti_handle_);
        embeddedServerNetDeinit(ti_handle_);
      }
      ti_handle_ = nullptr;
      running_ = false;
    }
    if (frame_.data != nullptr)
    {
      delete [] frame_.data;
    }
  }

  bool NetTxWrapper::sendCameraFrame(cv::Mat &camera, double ts, uint32_t frame_count)
  {
    // check input frame
    if ((camera.channels() == 3) && (camera.type() == CV_8UC3))
    {
      uint32_t offset = 0;
      memset(&frame_.header, 0, sizeof(EmbeddedNet_PhantomHeader));
      frame_.header.timestamp = static_cast<uint64_t>(ts * SEC2USEC);
      frame_.header.img_frame_count = frame_count;

      cv::Mat resized;
      cv::Mat cconverted;

      // scale if necessary
      if (image_size_ != camera.size())
      {
        cv::resize(camera, resized, image_size_);
      }
      else
      {
        resized = camera;
      }

      EmbeddedNet_PhantomHeader_Info *item = &frame_.header.item[frame_.header.item_count];
      item->width = image_size_.width;
      item->height = image_size_.height;

      uint8_t *dest = (&frame_.data[0] + offset);
      uint32_t imgsize = 0;

      item->format = getCameraFormat();

      imgsize = packBGRData(item, resized, dest, offset);

      frame_.header.item_count++;
      offset += imgsize;

      appendVehicleData(offset);
      appendVisionData(offset);
      appendDynamicCalibrationData(offset);
      appendCamCalibData(offset);
      appendCanTxData(offset);
      appendAebData(offset);

      return sendRawFrame(frame_);
    }
    else if ((camera.channels() == 2) && (camera.type() == CV_8UC2))
    {
      uint32_t offset = 0;
      memset(&frame_.header, 0, sizeof(EmbeddedNet_PhantomHeader));
      frame_.header.timestamp = static_cast<uint64_t>(ts * SEC2USEC);

      cv::Mat resized;
      cv::Mat cconverted;

      // scale if necessary
      if (image_size_ != camera.size())
      {
        PHANTOM_ERROR("Unsupported resize of YUV image, skipping all scaling!");
        resized = camera;
        image_size_ = camera.size();
      }
      else
      {
        resized = camera;
      }

      EmbeddedNet_PhantomHeader_Info *item = &frame_.header.item[frame_.header.item_count];
      item->width = image_size_.width;
      item->height = image_size_.height;

      uint8_t *dest = (&frame_.data[0] + offset);
      uint32_t imgsize = 0;

      item->format = getCameraFormat();

      imgsize = packYuvData(item, resized, dest, offset);

      frame_.header.item_count++;
      offset += imgsize;

      appendVehicleData(offset);
      appendVisionData(offset);
      appendDynamicCalibrationData(offset);
      appendCamCalibData(offset);
      appendCanTxData(offset);
      appendAebData(offset);

      return sendRawFrame(frame_);
    }
    else
    {
      PHANTOM_ERROR("Unsupported camera cv::Mat, type {}, chan {}", camera.type(), camera.channels());
    }
    return false;
  }

  //Assumes vector is ordered Main, Crop, Side Left, Side Right
  bool NetTxWrapper::sendImageFrame(std::vector<cv::Mat>& img_arr, std::vector<phantom_ai::CameraID>& cam_location, double ts, uint32_t frame_count)
  {
    bool input_bgr = false;
    bool input_yuv422 = false;
    bool input_yuv420 = false;

    if (img_arr.size() != 0)
    {
      // sanity check
      if (img_arr.size() != cam_location.size())
      {
        PHANTOM_ERROR("Mismatched vector sizes, skipping netTx");
        return false;
      }

      // check input frame
      if ((img_arr[0].channels() == 3) && (img_arr[0].type() == CV_8UC3))
      {
        input_bgr = true;
      }
      else if ((img_arr[0].channels() == 2) && (img_arr[0].type() == CV_8UC2))
      {
        input_yuv422 = true;
      }
      else if ((img_arr[0].channels() == 1) && (img_arr[0].type() == CV_8UC1))
      {
        input_yuv420 = true;
      }
      else
      {
        PHANTOM_ERROR("NetTx: Unsupported Mat type {} {}", img_arr[0].channels(), img_arr[0].type());
        return false;
      }
    }

    uint32_t offset = 0;
    memset(&frame_.header, 0, sizeof(EmbeddedNet_PhantomHeader));
    frame_.header.timestamp = static_cast<uint64_t>(ts * SEC2USEC);
    frame_.header.img_frame_count = frame_count;

    data_buf_mutex_.lock();
    for (size_t i = 0; i < img_arr.size(); i++)
    {
      cv::Mat input;
      cv::Mat resized;
      cv::Mat cconverted;

      input = img_arr[i];

      // disable all scaling for image_size_
      if (input_bgr)
      {
        image_size_ = input.size(); // no resize
        resized = input;
      }
      else if (input_yuv422)
      {
        image_size_ = input.size(); // no resize
        resized = input;
      }
      else if (input_yuv420)
      {
        // scale if necessary
        cv::Size adj_size = input.size();
        adj_size.height = adj_size.height * 2 / 3; // undo opencv weird size for YUV420
        image_size_ = adj_size; // no resize
        resized = input;
      }

      EmbeddedNet_PhantomHeader_Info *item = &frame_.header.item[frame_.header.item_count];
      item->format = getImageFormat();
      item->id = convertCameraID(cam_location[i]);
      item->width = image_size_.width;
      item->height = image_size_.height;

      uint8_t *dest = (&frame_.data[0] + offset);
      uint32_t imgsize = 0;

      if (input_bgr)
      {
        imgsize = packBGRData(item, resized, dest, offset);
      }
      else if (input_yuv422)
      {
        imgsize = packYuvData(item, resized, dest, offset);
      }
      else if (input_yuv420)
      {
        imgsize = packYuv420Data(item, resized, dest, offset);
      }

      frame_.header.item_count++;
      offset += imgsize;
    }

    appendVehicleData(offset);
    appendVisionData(offset);
    appendDynamicCalibrationData(offset);
    appendCamCalibData(offset);
    appendCanTxData(offset);
    appendPhantomVisionViz(offset);
    appendFusaData(offset);
    appendCameraExtraData(offset);
    appendHbaData(offset);
    appendAebData(offset);

    bool sucess = sendRawFrame(frame_);

    data_buf_mutex_.unlock();

    return sucess;
  }
//#if defined(ENABLE_GSTREAMER)
  void NetTxWrapper::registerEncodingMosaicImg(
    std::function<int(uint32_t, uint8_t*, uint32_t, uint32_t*, uint32_t*, uint32_t*)> callback_func)
  {
    encoding_mosaicImg_gst_callback_func_ = callback_func;
  }

  bool NetTxWrapper::sendEncodedImageFrame(std::vector<phantom_ai::CameraID>& cam_location, double ts, uint32_t frame_count)
  {
    if (!encoding_mosaicImg_gst_callback_func_) {
        PHANTOM_ERROR("Encoded image callback not registered!");
        return false;
    }

    std::vector<uint8_t> encoded_data_buffer(2 * 1024 * 1024);
    uint32_t encoded_size = 0;
    uint32_t mosaic_w = 0, mosaic_h = 0;

    int result = encoding_mosaicImg_gst_callback_func_(frame_count, encoded_data_buffer.data(), encoded_data_buffer.size(), &encoded_size, &mosaic_w, &mosaic_h);
    if (result != 0) {
        PHANTOM_LOG("Encoded frame %u not found or buffer too small (status: %d)", frame_count, result);
        return false;
    }

    encoded_data_buffer.resize(encoded_size);

    uint32_t offset = 0;
    memset(&frame_.header, 0, sizeof(EmbeddedNet_PhantomHeader));
    frame_.header.timestamp = static_cast<uint64_t>(ts * SEC2USEC);
    frame_.header.img_frame_count = frame_count;

    data_buf_mutex_.lock();

    EmbeddedNet_PhantomHeader_Info *item = &frame_.header.item[frame_.header.item_count];
    item->format = CAMERA_IMG_H264;

    for (size_t i = 0; i < cam_location.size(); i++) {
      item->id_arry[i] = convertCameraID(cam_location[i]);
    }
    item->width = mosaic_w;
    item->height = mosaic_h;

    uint8_t *dest = (&frame_.data[0]);
    if (encoded_data_buffer.size() > frame_.data_size) {
        PHANTOM_ERROR("Encoded data size (%zu) is larger than the network buffer (%u)!", encoded_data_buffer.size(), frame_.data_size);
        data_buf_mutex_.unlock();
        return false;
    }

    memcpy(dest, encoded_data_buffer.data(), encoded_data_buffer.size());

    frame_.header.item_count++;
    offset += encoded_data_buffer.size();

    appendVehicleData(offset);
    appendVisionData(offset);
    appendDynamicCalibrationData(offset);
    appendCamCalibData(offset);
    appendCanTxData(offset);
    appendPhantomVisionViz(offset);
    appendFusaData(offset);
    appendCameraExtraData(offset);
    appendHbaData(offset);
    appendAebData(offset);

    bool sucess = sendRawFrame(frame_);
    data_buf_mutex_.unlock();

    return sucess;
  }
//#endif
  bool NetTxWrapper::sendRawFrame(Embedded_Frame& input)
  {
    bool status = false;

    if (!running_)
    {
      return false;
    }

    // send frame to transmitter
    std::lock_guard<std::mutex> lock(send_frame_mutex_);
    if (is_client_)
    {
      if (embeddedNetSendFrame(ti_handle_, &input) < 0)
      {
        PHANTOM_ERROR("Failed to transmit frame");
        error_cnt_++;
      }
      else
      {
        error_cnt_ = 0;
        status = true;
      }
    }
    else
    {
      if (embeddedServerNetStart(ti_handle_) >= 0)
      {
        if (embeddedServerNetSendFrame(ti_handle_, &input) < 0)
        {
          PHANTOM_ERROR("Failed to transmit frame");
          error_cnt_++;
        }
        else
        {
          error_cnt_ = 0;
          status = true;
        }
      }
    }
    return status;
  }

  bool NetTxWrapper::queueVehicleData(EmbeddedNet_VehicleInfo &data)
  {
    veh_data_ = data;
    return true;
  }

  bool NetTxWrapper::queueVisionData(const EmbeddedNet_PhantomVisionList &data)
  {
    vision_data_ = data;
    vision_data_valid_ = true;
    return true;
  }

  bool NetTxWrapper::queueDynamicCalibrationData(const EmbeddedNet_DynamicCalibrationResultList &data)
  {
    dynm_calib_data_ = data;
    dynm_calib_data_valid_ = true;
    return true;
  }

  bool NetTxWrapper::queueVisionVizData(const cv::Mat& data)
  {
    if(!vision_viz_message_recevied_)
    {
      // Based on the vision viz message, resize the data_size of the frame to be sent.
      uint32_t additional_len = data.size().width * data.size().height * 3; // * 3 for RGB
      resizeDataBuffer(additional_len);

      vision_viz_message_recevied_ = true;  // to prevent multiple resizeDataBuffer
    }

    std::lock_guard<std::mutex> lock(viz_data_mutex_);
    vision_viz_data_ = data;
    vision_viz_data_valid_ = true;
    return true;
  }

  void NetTxWrapper::resizeDataBuffer(const uint32_t additional_len)
  {
    data_buf_mutex_.lock();

    frame_.data_size += additional_len;

    delete[] frame_.data;
    frame_.data = new uint8_t[frame_.data_size];

    if (frame_.data == nullptr)
    {
      throw VisionException("Dynmic frame data resizing failed {} b", frame_.data_size);
    }

    data_buf_mutex_.unlock();
  }

  bool NetTxWrapper::queueCameraCalib(const EmbeddedNet_CameraCalibration &data)
  {
      cam_info_ = data;
      return true;
  }

  bool NetTxWrapper::queueCanTxData(const EmbeddedNet_CanFrames &data)
  {
    can_tx_data_ = data;
    return true;
  }

  bool NetTxWrapper::queueFusaData(const EmbeddedNet_FunctionalSafetyOutputList &data)
  {
    fusa_data_ = data;
    fusa_data_valid_ = true;
    return true;
  }

  bool NetTxWrapper::queueCameraExtraData(const EmbeddedNet_CameraExtraInfoListTopLevel &data)
  {
    camera_extra_data_ = data;
    camera_extra_data_valid_ = true;
    return true;
  }

  bool NetTxWrapper::queueHbaData(const EmbeddedNet_PhantomVisionHBAList &data)
  {
    hba_data_ = data;
    hba_data_valid_ = true;
    return true;
  }

  void NetTxWrapper::queueAebData(const EmbeddedNet_PhantomVisionAEBList &data)
  {
    aeb_data_ = data;
    aeb_data_valid_ = true;
  }


  void NetTxWrapper::resetFrameBuffer(const bool vision_only_dual_logging, const bool init, const std::vector<int>& annotation_image_widths)
  {
    frame_.data_size = 0;
#if !defined(ENABLE_GSTREAMER)
      for (auto cam_infos: cam_info_list_) {
        for (const auto& it: cam_infos) {
          const auto& cam_info = it.second;
          // skip front_center_crop_bayer camera
          if (cam_info.location_ == "front_center_crop_bayer")
          {
            continue;
          }
          int32_t image_width = cam_info.output_resolution_.width;
          int32_t image_height = cam_info.output_resolution_.height;

          if ((use_fullres_logging_ && !is_full_resolution_camera(camera_name(it.first)))
              || (!use_fullres_logging_ && is_full_resolution_camera(camera_name(it.first))))
          {
            continue;
          }

          auto process_encoding = [&]() {
            if (encoding_ == "yuv420") {
              output_format_ = tx_format_yuv420sp;
              frame_.data_size += image_width * image_height * 3 / 2;
            } else if (encoding_ == "yuv422") {
              output_format_ = tx_format_yuv422;
              frame_.data_size += image_width * image_height * 2;
            } else if (encoding_ == "rgb") {
              output_format_ = tx_format_rgb;
              frame_.data_size += image_width * image_height * 3;
            } else if (encoding_ == "bgr") {
              output_format_ = tx_format_bgr;
              frame_.data_size += image_width * image_height * 3;
            } else {
              throw std::runtime_error("Invalid format " + encoding_);
            }
          };

          // only add vision images out of both loaded vision and annotation image info
          // if the current frame only include vision images
          auto anno_it = std::find(annotation_image_widths.begin(), annotation_image_widths.end(), image_height);
          if ((vision_only_dual_logging && (anno_it == annotation_image_widths.end())) || (!vision_only_dual_logging)) {
            process_encoding();
        }
      }
    }
#else
      output_format_ = tx_format_encoded;
      frame_.data_size += (mosaicImg_width_ * mosaicImg_height_ * 3 ) / 2;
#endif
    frame_.data_size += sizeof(EmbeddedNet_VehicleInfo); // extra data for vehicle states
    frame_.data_size += sizeof(EmbeddedNet_PhantomVisionList); // extra data for phantom vision measurements
    frame_.data_size += sizeof(EmbeddedNet_DynamicCalibrationResultList); // extra data for dynamic calibration outputs
    frame_.data_size += sizeof(EmbeddedNet_CameraCalibration); // extra data for camera information
    frame_.data_size += sizeof(EmbeddedNet_CanFrames); // extra data for can frames
    frame_.data_size += sizeof(EmbeddedNet_FunctionalSafetyOutputList); // extra data for fusa outputs
    frame_.data_size += sizeof(EmbeddedNet_CameraExtraInfoListTopLevel); // extra data for 'camera extra data'
    frame_.data_size += sizeof(EmbeddedNet_PhantomVisionHBAList); // extra data for hba outputs
    frame_.data_size += sizeof(EmbeddedNet_PhantomVisionAEBList); // extra data for aeb outputs

    if (!init){
      if (frame_.data != nullptr) {
        delete[] frame_.data;
      }
    }

    // Allocate new memory
    frame_.data = new uint8_t[frame_.data_size];

    if (frame_.data == nullptr) {
        throw VisionException("alloc failed {} b", frame_.data_size);
    }
  }

//#if defined(ENABLE_GSTREAMER)
  EncoderFeedFrameCallback NetTxWrapper::getFrameFeedCallback()
  {
      return &encodeH264_gst_feed_frame;
  }
#if 0
  bool NetTxWrapper::initGstEncoder(uint32_t width, uint32_t height)
  {
#if defined(ENABLE_GSTREAMER)
    PHANTOM_ERROR(" ++++++++++  src/phantom_ai/wrappers/src/network_tx_wrapper.cpp  ENABLE_GSTREAMER ON ");//WOOSEOK
#else
    PHANTOM_ERROR(" ----------  src/phantom_ai/wrappers/src/network_tx_wrapper.cpp  ENABLE_GSTREAMER OFF ");//WOOSEOK
#endif
    phantom_ai::YamlNode gst_cfg = phantom_ai::load_yaml_file("sensors/embedded_network", "gst_config.yaml");
    gst_h264encoding_enabled_ = phantom_ai::get_yaml_key<bool>(gst_cfg, "encoding_image_by_h24", false);

    if(gst_h264encoding_enabled_)
    {
      if (width > 0 && height > 0)
      {
        std::string gst_pipeline_str = phantom_ai::get_yaml_key<std::string>(gst_cfg, "h264_pipeline_string");

        bool gst_init_status = encodeH264_gst_init(
                                gst_pipeline_str.c_str(),
                                gst_h264encoding_enabled_,
                                15, // framerate
                                width,
                                height);
        if(gst_init_status) {
          PHANTOM_WARNING("NetTxWrapper: GStreamer H.264 encoder initialized successfully.");
          this->enableEncodeImgByGst(true);
          this->registerEncodingMosaicImg(&get_encoded_imgMosaicObj);
          mosaicImg_width_ = width;
          mosaicImg_height_ = height;
          return true;
        }
      }
      else {
        PHANTOM_ERROR("NetTxWrapper: Invalid dimensions (%u x %u) for GStreamer encoder.", width, height);
      }
    }

    PHANTOM_ERROR("NetTxWrapper: Failed to initialize GStreamer encoder.");
    gst_h264encoding_enabled_ = false;
    return false;
  }
#endif
//#endif
  void NetTxWrapper::onInit(std::string stream_config_filename, std::string camera_capture_config_filename, std::string target_system, bool cam_app)
  {
    // Load the stream config.
    phantom_ai::YamlNode stream_cfg = phantom_ai::load_yaml_file("sensors/embedded_network", stream_config_filename);

    std::string server_ip = phantom_ai::get_yaml_value(stream_cfg, "server_ip").as<std::string>();
    std::string server_binding_ip = phantom_ai::get_yaml_value(stream_cfg, "binding_ip").as<std::string>();
    std::string mode = phantom_ai::get_yaml_value(stream_cfg, "mode").as<std::string>();
    uint32_t port = phantom_ai::get_yaml_value(stream_cfg, "port").as<uint32_t>();
    uint32_t connect_timeout = phantom_ai::get_yaml_value(stream_cfg, "connect_timeout").as<uint32_t>();
    uint32_t read_timeout = phantom_ai::get_yaml_value(stream_cfg, "read_timeout").as<uint32_t>();
    encoding_ = phantom_ai::get_yaml_value(stream_cfg, "output_format").as<std::string>();

    // Load the camera capture configs and allocate the memory space for the images.
    #if defined(PLATFORM_AM62A)
    std::vector<camera_params_hal> cam_params_arr;
    std::vector<int> annotation_image_widths;
    readHalCamerasParameters(camera_capture_config_filename, cam_params_arr, cam_info_list_, annotation_image_widths, use_fullres_logging_, false, target_system, cam_app);
    #else
    std::vector<camera_params_tda4x> cam_params_arr;
    std::vector<int> annotation_image_widths;
    UNUSED(cam_app);
    readTda4xCamerasParameters(camera_capture_config_filename, cam_params_arr, cam_info_list_, annotation_image_widths, use_fullres_logging_, target_system);
    #endif

    resetFrameBuffer(false, true, annotation_image_widths);

    // clear out any old values
    memset(&frame_.header, 0, sizeof(EmbeddedNet_PhantomHeader));

    if (mode == "client")
    {
      PHANTOM_LOG("Starting transmitter");
      PHANTOM_LOG("Server IP {}, port {}", server_ip, port);
      ti_handle_ = embeddedNetStart(server_ip.c_str(), server_binding_ip.c_str(), port, NET_TCP, connect_timeout, read_timeout);
      is_client_ = true;
    }
    else
    {
      PHANTOM_LOG("Starting transmitter in host mode");
      PHANTOM_LOG("Server IP {}, port {}", server_binding_ip, port);
      ti_handle_ = embeddedServerNetInit(server_binding_ip.c_str(), port, NET_TCP, read_timeout);
      is_client_ = false;
    }

    if (ti_handle_ == nullptr)
    {
      throw VisionException("Transmitter failed to initialize");
    }

    stat_print_interval_sec_ = 5.0;
    last_print_time_ = phantom_ai::TimeStamp::Now();

    running_ = true;
  }

  bool NetTxWrapper::convertVisionFrame(const phantom_ai::PhantomVisionMeasurement &input, EmbeddedNet_PhantomVisionResult *output)
  {
    // zero out buffer
    memset(output, 0, sizeof(EmbeddedNet_PhantomVisionResult));

    // Copy Lane data
    output->lane_count = input.lanes_.size();

    if (output->lane_count > MAX_LANE_ARR_NUM)
    {
      PHANTOM_WARNING("SIM: Exceeded lane count size, truncating");
      output->lane_count = MAX_LANE_ARR_NUM;
    }

    for (size_t i = 0; i <  output->lane_count; i++)
    {
      auto& out = output->lanes[i];

      out.id = input.lanes_[i].id_;
      out.valid = input.lanes_[i].valid_;
      out.measured = input.lanes_[i].measured_;
      out.mark_type = static_cast<uint8_t>(input.lanes_[i].mark_type_);
      out.color = static_cast<uint8_t>(input.lanes_[i].color_);
      out.quality = static_cast<uint8_t>(input.lanes_[i].quality_);
      out.location = static_cast<uint8_t>(input.lanes_[i].location_);

      out.c0 = input.lanes_[i].c0_;
      out.c1 = input.lanes_[i].c1_;
      out.c2 = input.lanes_[i].c2_;
      out.c3 = input.lanes_[i].c3_;

      out.width_m = input.lanes_[i].ego_lane_width_m_;
      out.view_range_start_m = input.lanes_[i].view_range_start_m_;
      out.view_range_end_m = input.lanes_[i].view_range_end_m_;
      out.age_secs = input.lanes_[i].age_secs_;
      out.pitch_rads = input.lanes_[i].pitch_rads_;
      strncpy(out.camera_name, input.lanes_[i].camera_name_.c_str(), MAX_CAM_NAME_LEN);
      out.camera_id = input.lanes_[i].camera_id_;

      if (input.lanes_[i].x_px_.size() != input.lanes_[i].y_px_.size())
      {
        PHANTOM_WARNING("SIM: Lane points mismatch");
        out.camera_px_count = std::min(input.lanes_[i].x_px_.size(), input.lanes_[i].y_px_.size());
      }
      else
      {
        out.camera_px_count = input.lanes_[i].x_px_.size();
      }

      if (out.camera_px_count > MAX_LANE_CAM_POINTS)
      {
        PHANTOM_WARNING("SIM: Lane points truncated {}", out.camera_px_count);
        out.camera_px_count = MAX_LANE_CAM_POINTS;
      }

      for (size_t j = 0; j < out.camera_px_count; j++)
      {
        out.x_px[j] = input.lanes_[i].x_px_[j];
        out.y_px[j] = input.lanes_[i].y_px_[j];
      }
    }

    // Copy object data
    if (publish_only_valid_objects_)
    {
      output->object_count = 0;
      for (const auto& obj : input.objects_)
      {
        if (obj.valid_)
          output->object_count++;
      }
    }
    else
    {
      output->object_count = input.objects_.size();
    }

    if (output->object_count > MAX_OBJ_ARR_NUM)
    {
      PHANTOM_WARNING("SIM: Exceeded object count size {}, truncating", MAX_OBJ_ARR_NUM);
      output->object_count = MAX_OBJ_ARR_NUM;
    }

    for (size_t i = 0; i <  output->object_count; i++)
    {
      auto& out = output->objects[i];

      if (publish_only_valid_objects_ && !input.objects_[i].valid_)
        continue;

      out.id = input.objects_[i].id_;
      out.valid = input.objects_[i].valid_;
      out.measured = input.objects_[i].measured_;
      out.classification = static_cast<uint8_t>(input.objects_[i].classification_);
      out.view_face = static_cast<uint8_t>(input.objects_[i].view_face_);
      out.lane_assignment = static_cast<uint8_t>(input.objects_[i].lane_assignment_);
      out.brake_light = static_cast<uint8_t>(input.objects_[i].brake_light_);
      out.flash_light = static_cast<uint8_t>(input.objects_[i].flash_light_);
      out.turn_signal = static_cast<uint8_t>(input.objects_[i].turn_signal_);
      out.confidence = static_cast<uint8_t>(input.objects_[i].confidence_);

      if (input.objects_[i].classification_candidates_.size() != input.objects_[i].classification_confidences_.size())
      {
        PHANTOM_WARNING("SIM: Mismatched size for classfication canidates");
        out.classification_count = std::min(
          input.objects_[i].classification_candidates_.size(),
          input.objects_[i].classification_confidences_.size());
      }
      else
      {
        out.classification_count = input.objects_[i].classification_candidates_.size();
      }

      if (out.classification_count > MAX_OBJ_CLASSIFICATION_CNT)
      {
        PHANTOM_WARNING("SIM: Truncating classifications");
        out.classification_count = MAX_OBJ_CLASSIFICATION_CNT;
      }

      for (size_t j = 0; j < out.classification_count; j++)
      {
        out.classification_candidates[j] = static_cast<uint8_t>(input.objects_[i].classification_candidates_[j]);
        out.classification_confidences[j] = input.objects_[i].classification_confidences_[j];
      }

      out.position_y_m = input.objects_[i].position_y_m_;
      out.position_x_m = input.objects_[i].position_x_m_;
      out.position_z_m = input.objects_[i].position_z_m_;
      out.rel_velocity_x_mps = input.objects_[i].rel_velocity_x_mps_;
      out.rel_velocity_y_mps = input.objects_[i].rel_velocity_y_mps_;
      out.rel_accel_x_mps2 = input.objects_[i].rel_accel_x_mps2_;
      out.rel_accel_y_mps2 = input.objects_[i].rel_accel_x_mps2_;
      out.range_velocity_mps = input.objects_[i].range_velocity_mps_;
      out.range_m = input.objects_[i].range_m_;
      out.angle_rads = input.objects_[i].angle_rads_;
      out.width_m = input.objects_[i].width_m_;
      out.length_m = input.objects_[i].length_m_;
      out.height_m = input.objects_[i].height_m_;
      out.orientation_rads = input.objects_[i].orientation_rads_;
      out.total_left_bounding_angle_valid = input.objects_[i].total_left_bounding_angle_valid_;
      out.total_right_bounding_angle_valid = input.objects_[i].total_right_bounding_angle_valid_;
      out.total_left_bounding_angle_rads = input.objects_[i].total_left_bounding_angle_rads_;
      out.total_right_bounding_angle_rads = input.objects_[i].total_right_bounding_angle_rads_;
      out.face_left_bounding_angle_valid = input.objects_[i].face_left_bounding_angle_valid_;
      out.face_right_bounding_angle_valid = input.objects_[i].face_right_bounding_angle_valid_;
      out.face_left_bounding_angle_rads = input.objects_[i].face_left_bounding_angle_rads_;
      out.face_right_bounding_angle_rads = input.objects_[i].face_right_bounding_angle_rads_;
      out.age_secs = input.objects_[i].age_secs_;
      out.time_to_collision_secs = input.objects_[i].time_to_collision_secs_;
      out.cipv = input.objects_[i].cipv_;
      out.oncoming_traffic = input.objects_[i].oncoming_traffic_;
      out.cross_traffic = input.objects_[i].cross_traffic_;
      out.extra_values_count = std::min((int)input.objects_[i].extra_values_.size(), MAX_EXTRA_ARR_NUM);
      for (size_t j = 0; j < out.extra_values_count; j++)
      {
        out.extra_values[j] = input.objects_[i].extra_values_[j];
      }
      out.extra_infos_count = std::min((int)input.objects_[i].extra_infos_.size(), MAX_EXTRA_ARR_NUM);
      for (size_t j = 0; j < out.extra_infos_count; j++)
      {
        strncpy(out.extra_infos[j], input.objects_[i].extra_infos_[j].c_str(), MAX_CAM_NAME_LEN);
      }
      strncpy(out.camera_name, input.objects_[i].camera_name_.c_str(), MAX_CAM_NAME_LEN);
      out.camera_id = input.objects_[i].camera_id_;
      out.x_px = input.objects_[i].x_px_;
      out.y_px = input.objects_[i].y_px_;
      out.width_px = input.objects_[i].width_px_;
      out.height_px = input.objects_[i].height_px_;
    }

    // Copy traffic data
    if (publish_only_valid_objects_)
    {
      output->traffic_count = 0;
      for (const auto& obj : input.traffic_signs_)
      {
        if (obj.valid_ || obj.is_this_for_highway_)
          output->traffic_count++;
      }
    }
    else
    {
      output->traffic_count = input.traffic_signs_.size();
    }

    if (output->traffic_count > MAX_TFC_ARR_NUM)
    {
      PHANTOM_WARNING("SIM: Exceeded traffic count size {}, truncating", MAX_TFC_ARR_NUM);
      output->traffic_count = MAX_TFC_ARR_NUM;
    }

    for (size_t i = 0; i <  output->traffic_count; i++)
    {
      auto& out = output->traffic_signs[i];

      if (publish_only_valid_objects_ && !input.traffic_signs_[i].valid_ && !input.traffic_signs_[i].is_this_for_highway_)
        continue;

      out.id = input.traffic_signs_[i].id_;
      out.valid = input.traffic_signs_[i].valid_;
      out.measured = input.traffic_signs_[i].measured_;
      out.is_this_for_highway = input.traffic_signs_[i].is_this_for_highway_;
      out.highway_mode = input.traffic_signs_[i].highway_mode_;
      out.classification = static_cast<uint8_t>(input.traffic_signs_[i].classification_);
      out.filter_type = static_cast<uint8_t>(input.traffic_signs_[i].filter_type_);
      out.lane_assignment = static_cast<uint8_t>(input.traffic_signs_[i].lane_assignment_);
      out.confidence = static_cast<uint8_t>(input.traffic_signs_[i].confidence_);

      out.position_y_m = input.traffic_signs_[i].position_y_m_;
      out.position_x_m = input.traffic_signs_[i].position_x_m_;
      out.position_z_m = input.traffic_signs_[i].position_z_m_;
      out.width_m = input.traffic_signs_[i].width_m_;
      out.length_m = input.traffic_signs_[i].length_m_;
      out.height_m = input.traffic_signs_[i].height_m_;

      out.age_secs = input.traffic_signs_[i].age_secs_;
      out.extra_values_count = std::min((int)input.traffic_signs_[i].extra_values_.size(), MAX_EXTRA_ARR_NUM);
      for (size_t j = 0; j < out.extra_values_count; j++)
      {
        out.extra_values[j] = input.traffic_signs_[i].extra_values_[j];
      }
      out.extra_infos_count = std::min((int)input.traffic_signs_[i].extra_infos_.size(), MAX_EXTRA_ARR_NUM);
      for (size_t j = 0; j < out.extra_infos_count; j++)
      {
        strncpy(out.extra_infos[j], input.traffic_signs_[i].extra_infos_[j].c_str(), MAX_CAM_NAME_LEN);
      }
      strncpy(out.camera_name, input.traffic_signs_[i].camera_name_.c_str(), MAX_CAM_NAME_LEN);
      out.camera_id = input.traffic_signs_[i].camera_id_;
      out.x_px = input.traffic_signs_[i].x_px_;
      out.y_px = input.traffic_signs_[i].y_px_;
      out.width_px = input.traffic_signs_[i].width_px_;
      out.height_px = input.traffic_signs_[i].height_px_;
    }

    // Copy space data
    output->space_count = input.free_spaces_.size();

    if (output->space_count > MAX_SPACES_ARR_NUM)
    {
      PHANTOM_WARNING("SIM: Exceeded free space size, truncating");
      output->space_count = MAX_SPACES_ARR_NUM;
    }

    for (size_t i = 0; i <  output->space_count; i++)
    {
      auto& out = output->free_spaces[i];

      out.valid = input.free_spaces_[i].valid_;
      out.count = input.free_spaces_[i].points_.size();
      if (out.count > MAX_SPACE_POINT_NUM)
      {
        PHANTOM_WARNING("SIM: Exceeded point array size, truncating");
        out.count = MAX_SPACE_POINT_NUM;
      }

      for (size_t j = 0; j < out.count; j++)
      {
        out.points[j].x = input.free_spaces_[i].points_[j].x;
        out.points[j].y = input.free_spaces_[i].points_[j].y;
      }
    }

    // Copy status data
    output->status_count = input.status_.size();

    if (output->status_count > MAX_STATUS_ARR_NUM)
    {
      PHANTOM_WARNING("SIM: Exceeded status count size, truncating");
      output->status_count = MAX_STATUS_ARR_NUM;
    }

    for (size_t i = 0; i <  output->status_count; i++)
    {
      auto& out = output->status[i];

      out.valid = input.status_[i].valid_;
      strncpy(out.camera_name, input.status_[i].camera_name_.c_str(), MAX_CAM_NAME_LEN);
      out.blockage = input.status_[i].blockage_;
      out.blurness = input.status_[i].blurness_;
    }

    // set top level vars
    output->x_m = input.x_m_;
    output->y_m = input.y_m_;
    output->heading_rads = input.heading_rads_;
    output->frame_count = input.frame_count_;
    output->stamp = static_cast<uint64_t>(input.hw_timestamp_ * SEC2USEC); // in us

    return true;
  }

  void NetTxWrapper::convertDynamicCalibrationFrame(const phantom_ai::PhantomVisionDynamicCalibrationOutputList &input,
                                                    EmbeddedNet_DynamicCalibrationResult *output)
  {
    // zero out buffer
    memset(output, 0, sizeof(EmbeddedNet_DynamicCalibrationResult));

    // Copy Lane data
    output->count = input.calibration_outputs.size();

    if (output->count > MAX_DYNAMIC_CALIBRATION_CAMERA)
    {
      PHANTOM_WARNING("SIM: Exceeded maximum number of dynamic calibration camera, truncating");
      output->count = MAX_DYNAMIC_CALIBRATION_CAMERA;
    }

    for (size_t i = 0; i < output->count; ++i)
    {
      auto& output_per_cam = output->item[i];
      const phantom_ai::PhantomVisionDynamicCalibrationOutput& input_per_cam = input.calibration_outputs[i];

      output_per_cam.version = input_per_cam.version;
      output_per_cam.sync_id = input_per_cam.sync_id;
      output_per_cam.camera_type = input_per_cam.camera_type;
      output_per_cam.calibration_mode = input_per_cam.calibration_mode;
      output_per_cam.routine_request = input_per_cam.routine_request;
      output_per_cam.calibration_running_status = input_per_cam.calibration_running_status;
      output_per_cam.precondition_error = input_per_cam.precondition_error;

      output_per_cam.roll_pause_status = input_per_cam.roll_pause_status;
      output_per_cam.pitch_pause_status = input_per_cam.pitch_pause_status;
      output_per_cam.yaw_pause_status = input_per_cam.yaw_pause_status;
      output_per_cam.height_pause_status = input_per_cam.height_pause_status;

      output_per_cam.roll_converging_progress = input_per_cam.roll_converging_progress;
      output_per_cam.pitch_converging_progress = input_per_cam.pitch_converging_progress;
      output_per_cam.yaw_converging_progress = input_per_cam.yaw_converging_progress;
      output_per_cam.height_converging_progress = input_per_cam.height_converging_progress;

      output_per_cam.roll_converging_status = misc::to_underlying(input_per_cam.roll_converging_status);
      output_per_cam.pitch_converging_status = misc::to_underlying(input_per_cam.pitch_converging_status);
      output_per_cam.yaw_converging_status = misc::to_underlying(input_per_cam.yaw_converging_status);
      output_per_cam.height_converging_status = misc::to_underlying(input_per_cam.height_converging_status);

      output_per_cam.roll_converging_error = misc::to_underlying(input_per_cam.roll_converging_error);
      output_per_cam.pitch_converging_error = misc::to_underlying(input_per_cam.pitch_converging_error);
      output_per_cam.yaw_converging_error = misc::to_underlying(input_per_cam.yaw_converging_error);
      output_per_cam.height_converging_error = misc::to_underlying(input_per_cam.height_converging_error);

      output_per_cam.roll_converged_time_sec = input_per_cam.roll_converged_time_sec;
      output_per_cam.pitch_converged_time_sec = input_per_cam.pitch_converged_time_sec;
      output_per_cam.yaw_converged_time_sec = input_per_cam.yaw_converged_time_sec;
      output_per_cam.height_converged_time_sec = input_per_cam.height_converged_time_sec;

      output_per_cam.roll_converged_distance_meter = input_per_cam.roll_converged_distance_meter;
      output_per_cam.pitch_converged_distance_meter = input_per_cam.pitch_converged_distance_meter;
      output_per_cam.yaw_converged_distance_meter = input_per_cam.yaw_converged_distance_meter;
      output_per_cam.height_converged_distance_meter = input_per_cam.height_converged_distance_meter;

      output_per_cam.roll_converged_degree = input_per_cam.roll_converged_degree;
      output_per_cam.pitch_converged_degree = input_per_cam.pitch_converged_degree;
      output_per_cam.yaw_converged_degree = input_per_cam.yaw_converged_degree;
      output_per_cam.height_converged_meter = input_per_cam.height_converged_meter;

      output_per_cam.camera_lateral_position = input_per_cam.camera_lateral_position;
      output_per_cam.camera_longitudinal_position = input_per_cam.camera_longitudinal_position;
      output_per_cam.camera_nominal_roll_deg = input_per_cam.camera_nominal_roll_deg;
      output_per_cam.camera_nominal_pitch_deg = input_per_cam.camera_nominal_pitch_deg;
      output_per_cam.camera_nominal_yaw_deg = input_per_cam.camera_nominal_yaw_deg;

      output_per_cam.roll_cost_m = input_per_cam.roll_cost_m;
      output_per_cam.pitch_delta_deg = input_per_cam.pitch_delta_deg;
      output_per_cam.yaw_delta_deg = input_per_cam.yaw_delta_deg;

      output_per_cam.image_health = input_per_cam.image_health;
    }

    output->stamp = static_cast<uint64_t>(input.header_.time_stamp.toSec() * SEC2USEC); // in us
  }

  bool NetTxWrapper::convertFusaDataFrame(const phantom_ai::FunctionalSafetyOutput &input, EmbeddedNet_FunctionalSafetyOutput *output)
  {
    // zero out buffer
    memset(output, 0, sizeof(EmbeddedNet_FunctionalSafetyOutput));

    // camera related fail safe
    uint32_t count = 0;
    for (const auto& pair : input.cameras_fail_safe_info_)
    {
      auto& out = output->cameras_fail_safe_info[count];
      const CameraFailSafeInformation& fusa_info = pair.second;
      out.camera_id = fusa_info.camera_id_;
      out.dynamic_calibration_input_error = fusa_info.dynamic_calibration_input_error_;
      out.intrinsic_calibration_input_error = fusa_info.intrinsic_calibration_input_error_;
      out.dynamic_calibration_output_error = fusa_info.dynamic_calibration_output_error_;
      out.camera_status_error = static_cast<uint8_t>(fusa_info.camera_status_error_);
      count++;
      if (count >= MAX_FUSA_INFO_CAMERA_COUNT)
      {
        break;
      }
    }

    // vehicle related fail safe
    output->vehicle_state_fail_safe_info.yaw_rate_input_error =
        static_cast<uint8_t>(input.vehicle_state_fail_safe_info_.yaw_rate_input_error_);
    output->vehicle_state_fail_safe_info.long_accel_input_error =
        static_cast<uint8_t>(input.vehicle_state_fail_safe_info_.long_accel_input_error_);
    output->vehicle_state_fail_safe_info.lat_accel_input_error =
        static_cast<uint8_t>(input.vehicle_state_fail_safe_info_.lat_accel_input_error_);
    output->vehicle_state_fail_safe_info.steer_angle_input_error =
        static_cast<uint8_t>(input.vehicle_state_fail_safe_info_.steer_angle_input_error_);
    output->vehicle_state_fail_safe_info.wheel_speed_input_error =
        static_cast<uint8_t>(input.vehicle_state_fail_safe_info_.wheel_speed_input_error_);

    output->fail_safe_count = count;
    output->stamp = static_cast<uint64_t>(input.header_.time_stamp.toSec() * SEC2USEC); // in us

    return true;
  }

  bool NetTxWrapper::convertCameraExtraDataFrame(const PhantomCameraExtarInputListS &input, 
                                                 EmbeddedNet_CamerasExtraInfoList *output)
  {
    // zero out buffer
    memset(output, 0, sizeof(EmbeddedNet_CamerasExtraInfoList));  

    output->count_camera = 0;

    //currently camera extra data is only available for front camera.
    for(const auto &cam : input)
    {
      if(output->count_camera >= MAX_NUM_CAMERAS_EXTRA_EMBEDDED_NET)
      {
        break;
      }
      auto& per_cam_extra_data = (output->cams_list[output->count_camera]);

      per_cam_extra_data.camera_id = cam->camera_id_;
      snprintf(per_cam_extra_data.camera_name, MAX_CAM_NAME_LEN, "%.*s", MAX_CAM_NAME_LEN - 1, cam->camera_name_.c_str());
      per_cam_extra_data.hw_timestamp = static_cast<uint64_t>(cam->hw_timestamp_ * SEC2USEC);
      per_cam_extra_data.exposure_time = cam->exposure_time_;
      per_cam_extra_data.analog_gain = cam->analog_gain_;
      per_cam_extra_data.camera_mode = cam->camera_mode_;

      per_cam_extra_data.roi_raw_crop[0] = cam->raw_img_crop_start_x_;
      per_cam_extra_data.roi_raw_crop[1] = cam->raw_img_crop_start_y_;
      per_cam_extra_data.roi_raw_crop[2] = cam->raw_img_width_;
      per_cam_extra_data.roi_raw_crop[3] = cam->raw_img_height_;

      memcpy(per_cam_extra_data.raw16bit_crop, cam->raw_, sizeof(uint8_t)*cam->raw_img_width_*cam->raw_img_height_*2);
#ifdef USE_EXTRA_INPUT_HISTOGRAM
      /*histogram is not implemented yet*/
      memcpy(per_cam_extra_data.histogram, cam->histogram, sizeof(uint32_t)*MAX_SIZE_CAMERA_EXTRA_HISTO);
#endif

      const auto& h3a_cfg_in = cam->h3a_cfg_;
      per_cam_extra_data.h3a_config.v_start  = h3a_cfg_in.v_start_; 
      per_cam_extra_data.h3a_config.h_start  = h3a_cfg_in.h_start_; 
      per_cam_extra_data.h3a_config.v_size   = h3a_cfg_in.v_size_; 
      per_cam_extra_data.h3a_config.h_size   = h3a_cfg_in.h_size_; 
      per_cam_extra_data.h3a_config.v_count  = h3a_cfg_in.v_count_; 
      per_cam_extra_data.h3a_config.h_count  = h3a_cfg_in.h_count_; 
      per_cam_extra_data.h3a_config.v_skip   = h3a_cfg_in.v_skip_; 
      per_cam_extra_data.h3a_config.h_skip   = h3a_cfg_in.h_skip_; 
      per_cam_extra_data.h3a_config.data_array_size = h3a_cfg_in.data_array_size_;

      memcpy(per_cam_extra_data.h3a, cam->h3a_, sizeof(uint8_t)*MAX_SIZE_CAMERA_EXTRA_H3A);

      per_cam_extra_data.hba_input_version = 1;

      output->count_camera++;
      output->hw_timestamp = per_cam_extra_data.hw_timestamp; //use last cam's time stamp
    }
    return true;
  }

  bool NetTxWrapper::convertHbaDataFrame(const phantom_ai::PhantomVisionHBA &input, EmbeddedNet_PhantomVisionHBA *output)
  {
    // zero out buffer
    memset(output, 0, sizeof(EmbeddedNet_PhantomVisionHBA));  

    output->illumination = input.illumination_;
    output->backlight_status = input.backlight_status_;
    output->camera_is_in_tunnel = input.camera_is_in_tunnel_;
    output->vehicle_light_info = input.vehicle_light_info_;
    output->beam_result = input.beam_result_;
    output->beam_result_availability = input.beam_result_availability_;
    output->street_light_status = static_cast<uint8_t>(input.street_light_status_);
    output->street_light_count_scene = input.street_light_count_scene_;
    output->environmental_status_scene = input.environmental_status_scene_;
    output->camera_mode = input.camera_mode_;
    output->exposure_time = input.exposure_time_;
    output->analog_gain = input.analog_gain_;
    output->ambient_light_lux = input.ambient_light_lux_;
    output->horizon_fullres = input.horizon_fullres_;
    output->horizon_h3a = input.horizon_h3a_;
    output->curvature_radius = input.curvature_radius_;
    output->hba_deactive_speed = static_cast<uint8_t>(input.hba_deactive_speed_);
    output->hba_deactive_curv = static_cast<uint8_t>(input.hba_deactive_curv_);
    output->hba_deactive_min_l2h_time_flag = static_cast<uint8_t>(input.hba_deactive_min_l2h_time_flag_);
    output->hba_deactive_min_l2h_time_value = input.hba_deactive_min_l2h_time_value_;
    output->hw_timestamp = static_cast<uint64_t>(input.hw_timestamp_ * SEC2USEC);

    return true;
  }

  bool NetTxWrapper::convertAebDataFrame(const phantom_ai::PhantomVisionAEB &input, EmbeddedNet_PhantomVisionAEB *output)
  {
    // zero out buffer
    memset(output, 0, sizeof(EmbeddedNet_PhantomVisionAEB));  

    // Copy object data
    if (publish_only_valid_objects_)
    {
      output->object_count = 0;
      for (const auto& obj : input.objects_)
      {
        if (obj.valid_)
          output->object_count++;
      }
    }
    else
    {
      output->object_count = input.objects_.size();
    }

    if (output->object_count > MAX_PHANTOM_AEB_OBJECTS_NUM)
    {
      PHANTOM_WARNING("SIM: Exceeded object count size {}, truncating", MAX_OBJ_ARR_NUM);
      output->object_count = MAX_PHANTOM_AEB_OBJECTS_NUM;
    }

    for (size_t i = 0; i <  output->object_count; i++)
    {
      auto& out = output->objects[i];

      if (publish_only_valid_objects_ && !input.objects_[i].valid_)
        continue;

      out.id       = input.objects_[i].id_;
      out.valid    = input.objects_[i].valid_;
      out.measured = input.objects_[i].measured_;

      out.classification  = static_cast<uint8_t>(input.objects_[i].classification_);
      out.view_face       = static_cast<uint8_t>(input.objects_[i].view_face_);
      out.lane_assignment = static_cast<uint8_t>(input.objects_[i].lane_assignment_);
      out.brake_light     = static_cast<uint8_t>(input.objects_[i].brake_light_);
      out.flash_light     = static_cast<uint8_t>(input.objects_[i].flash_light_);
      out.turn_signal     = static_cast<uint8_t>(input.objects_[i].turn_signal_);
      out.confidence      = static_cast<uint8_t>(input.objects_[i].confidence_);

      if (input.objects_[i].classification_candidates_.size() != input.objects_[i].classification_confidences_.size())
      {
        PHANTOM_WARNING("SIM: Mismatched size for classfication canidates");
        out.classification_count = std::min(
          input.objects_[i].classification_candidates_.size(),
          input.objects_[i].classification_confidences_.size());
      }
      else
      {
        out.classification_count = input.objects_[i].classification_candidates_.size();
      }

      if (out.classification_count > MAX_OBJ_CLASSIFICATION_CNT)
      {
        PHANTOM_WARNING("SIM: Truncating classifications");
        out.classification_count = MAX_OBJ_CLASSIFICATION_CNT;
      }

      for (size_t j = 0; j < out.classification_count; j++)
      {
        out.classification_candidates[j]  = static_cast<uint8_t>(input.objects_[i].classification_candidates_[j]);
        out.classification_confidences[j] = input.objects_[i].classification_confidences_[j];
      }

      out.position_x_m       = input.objects_[i].position_x_m_;
      out.position_y_m       = input.objects_[i].position_y_m_;
      out.position_z_m       = input.objects_[i].position_z_m_;
      out.rel_velocity_x_mps = input.objects_[i].rel_velocity_x_mps_;
      out.rel_velocity_y_mps = input.objects_[i].rel_velocity_y_mps_;
      out.rel_accel_x_mps2   = input.objects_[i].rel_accel_x_mps2_;
      out.rel_accel_y_mps2   = input.objects_[i].rel_accel_x_mps2_;
      out.range_velocity_mps = input.objects_[i].range_velocity_mps_;
      out.range_m            = input.objects_[i].range_m_;

      out.angle_rads       = input.objects_[i].angle_rads_;
      out.orientation_rads = input.objects_[i].orientation_rads_;

      out.width_m  = input.objects_[i].width_m_;
      out.length_m = input.objects_[i].length_m_;
      out.height_m = input.objects_[i].height_m_;

      out.total_left_bounding_angle_valid  = input.objects_[i].total_left_bounding_angle_valid_;
      out.total_right_bounding_angle_valid = input.objects_[i].total_right_bounding_angle_valid_;
      out.total_left_bounding_angle_rads   = input.objects_[i].total_left_bounding_angle_rads_;
      out.total_right_bounding_angle_rads  = input.objects_[i].total_right_bounding_angle_rads_;
      out.face_left_bounding_angle_valid   = input.objects_[i].face_left_bounding_angle_valid_;
      out.face_right_bounding_angle_valid  = input.objects_[i].face_right_bounding_angle_valid_;
      out.face_left_bounding_angle_rads    = input.objects_[i].face_left_bounding_angle_rads_;
      out.face_right_bounding_angle_rads   = input.objects_[i].face_right_bounding_angle_rads_;

      out.age_secs = input.objects_[i].age_secs_;
      out.cipv     = input.objects_[i].cipv_;

      out.time_to_collision_secs = input.objects_[i].time_to_collision_secs_;      
      out.oncoming_traffic       = input.objects_[i].oncoming_traffic_;
      out.cross_traffic          = input.objects_[i].cross_traffic_;
      out.extra_values_count     = std::min((int)input.objects_[i].extra_values_.size(), MAX_EXTRA_ARR_NUM);
      for (size_t j = 0; j < out.extra_values_count; j++)
      {
        out.extra_values[j] = input.objects_[i].extra_values_[j];
      }
      out.extra_infos_count = std::min((int)input.objects_[i].extra_infos_.size(), MAX_EXTRA_ARR_NUM);
      for (size_t j = 0; j < out.extra_infos_count; j++)
      {
        strncpy(out.extra_infos[j], input.objects_[i].extra_infos_[j].c_str(), MAX_CAM_NAME_LEN);
      }
      strncpy(out.camera_name, input.objects_[i].camera_name_.c_str(), MAX_CAM_NAME_LEN);
      out.camera_id = input.objects_[i].camera_id_;
      out.x_px      = input.objects_[i].x_px_;
      out.y_px      = input.objects_[i].y_px_;
      out.width_px  = input.objects_[i].width_px_;
      out.height_px = input.objects_[i].height_px_;
    }

    // Copy space data
    output->flag_count = input.vehicle_flags_.size();

    if (output->flag_count > MAX_PHANTOM_AEB_FLAG_NUM)
    {
      PHANTOM_WARNING("SIM: Exceeded free space size, truncating");
      output->flag_count = MAX_PHANTOM_AEB_FLAG_NUM;
    }

    for (size_t i = 0; i < output->flag_count; i++)
    {
      auto& veh_flag = output->veh_flags[i];

      veh_flag.type          = input.vehicle_flags_[i].type_;
      veh_flag.activated     = input.vehicle_flags_[i].activated_;
      veh_flag.object_id     = input.vehicle_flags_[i].object_id_;
      veh_flag.object_id_can = input.vehicle_flags_[i].object_id_can_;

      strncpy(veh_flag.name, input.vehicle_flags_[i].name_.c_str(), MAX_AEB_FLAG_NAME - 1);
      veh_flag.name[MAX_AEB_FLAG_NAME - 1] = '\0';

      auto& vru_flag = output->vru_flags[i];

      vru_flag.type          = input.vru_flags_[i].type_;
      vru_flag.activated     = input.vru_flags_[i].activated_;
      vru_flag.object_id     = input.vru_flags_[i].object_id_;
      vru_flag.object_id_can = input.vru_flags_[i].object_id_can_;

      strncpy(vru_flag.name, input.vru_flags_[i].name_.c_str(), MAX_AEB_FLAG_NAME - 1);
      vru_flag.name[MAX_AEB_FLAG_NAME - 1] = '\0';
    }

    output->stamp = static_cast<uint64_t>(input.hw_timestamp_ * SEC2USEC); // in us
    return true;
  }

  // private

  void NetTxWrapper::repackI420toNV12(uint8_t *src, uint8_t *dst, uint32_t width, uint32_t height)
  {
    uint32_t len_luma = width * height;
    uint32_t len_chroma = width / 2 * height / 2;

    // input format is I420, 3 planar buffers with Y, U, V. The U,V is 2x2 subsampled
    uint8_t *src_y = src;
    uint8_t *src_u = src_y + len_luma;
    uint8_t *src_v = src_u + len_chroma;

    uint8_t *dst_y = dst;
    uint8_t *dst_uv = dst_y + len_luma;

    // copy Y as is
    memcpy(dst_y, src_y, len_luma);

    // copy separate U,V to interleaved UV
    for (uint32_t i = 0; i < len_chroma; i++)
    {
      *dst_uv++ = *src_u++;
      *dst_uv++ = *src_v++;
    }
  }

  EmbeddedNet_PhantomFormat NetTxWrapper::getImageFormat()
  {
    EmbeddedNet_PhantomFormat val = GENERIC_BUFFER;

    if (output_format_ == tx_format_yuv420sp)
    {
      val = CAMERA_IMG_YUV_420SP;
    }
    else if (output_format_ == tx_format_yuv422)
    {
      val = CAMERA_IMG_YUV_422;
    }
    else if (output_format_ == tx_format_rgb)
    {
      val = CAMERA_IMG_RGB;
    }
    else if (output_format_ == tx_format_bgr)
    {
      val = CAMERA_IMG_BGR;
    }

    if (val == GENERIC_BUFFER)
    {
      throw VisionException("Invalid combo format {}", output_format_);
    }

    return val;
  }

  EmbeddedNet_PhantomFormat NetTxWrapper::getCameraFormat()
  {
    EmbeddedNet_PhantomFormat val = GENERIC_BUFFER;

    if (output_format_ == tx_format_yuv420sp)
    {
      val = CAMERA_IMG_YUV_420SP;
    }
    else if (output_format_ == tx_format_yuv422)
    {
      val = CAMERA_IMG_YUV_422;
    }
    else if (output_format_ == tx_format_rgb)
    {
      val = CAMERA_IMG_RGB;
    }
    else if (output_format_ == tx_format_bgr)
    {
      val = CAMERA_IMG_BGR;
    }
    if (val == GENERIC_BUFFER)
    {
      throw VisionException("Invalid combo format {}", output_format_);
    }

    return val;
  }

  void NetTxWrapper::updateStats(bool tx_status)
  {
    if (tx_status)
    {
      tx_frame_cnt_++;
    }

    phantom_ai::TimeStamp now = phantom_ai::TimeStamp::Now();

    double delta_time = now.toSec() - last_print_time_.toSec();

    if (delta_time >= stat_print_interval_sec_)
    {
      double fps = (double)tx_frame_cnt_ / delta_time;
      double in_fps = (double)rx_frame_cnt_ / delta_time / 2;
      PHANTOM_LOG("ROS fps: {:.2f}, Tx fps: {:.2f}, frames: {}, delta: {:.3f} s", in_fps, fps, tx_frame_cnt_, delta_time);

      // reset counters
      last_print_time_ = now;
      tx_frame_cnt_ = 0;
      rx_frame_cnt_ = 0;
    }
  }

  uint32_t NetTxWrapper::packBGRData(EmbeddedNet_PhantomHeader_Info *item, cv::Mat &input, uint8_t* dest, uint32_t offset)
  {
    cv::Mat cconverted;
    uint32_t imgsize = 0;

    // convert BGR to final color format
    if (output_format_ == tx_format_yuv420sp)
    {
      cv::cvtColor(input, cconverted, cv::COLOR_BGR2YUV_I420);
      imgsize = image_size_.width * image_size_.height * 3 / 2;
      // Check if the addition of new data will exceed the buffer size.
      if ((offset + imgsize) > frame_.data_size)
      {
        throw VisionException("Overflow buffer {} > {}", (offset + imgsize), frame_.data_size);
      }

      repackI420toNV12(&cconverted.data[0], dest, image_size_.width, image_size_.height);

      // preserve separate Y and UV buffers
      item->pitch[0] = image_size_.width;
      item->pitch[1] = image_size_.width;
      item->buf_size[0] = image_size_.width * image_size_.height;
      item->buf_size[1] = image_size_.width * image_size_.height / 2;
      item->offset[0] = offset;
      item->offset[1] = offset + item->buf_size[0];
    }
    else if (output_format_ == tx_format_yuv422)
    {
      cv::cvtColor(input, cconverted, cv::COLOR_BGR2YUV); // YUV444

      imgsize = image_size_.width * image_size_.height * 2;
      // Check if the addition of new data will exceed the buffer size.
      if ((offset + imgsize) > frame_.data_size)
      {
        throw VisionException("Overflow buffer {} > {}", (offset + imgsize), frame_.data_size);
      }

      // No OpenCV BGR->422 available!
      convertYUV444to422(&cconverted.data[0], dest, image_size_.width, image_size_.height);

      item->pitch[0] = image_size_.width * 2;
      item->buf_size[0] = imgsize;
      item->offset[0] = offset;
    }
    else if (output_format_ == tx_format_rgb)
    {
      cv::cvtColor(input, cconverted, cv::COLOR_BGR2RGB);

      imgsize = image_size_.width * image_size_.height * 3;
      // Check if the addition of new data will exceed the buffer size.
      if ((offset + imgsize) > frame_.data_size)
      {
        throw VisionException("Overflow buffer {} > {}", (offset + imgsize), frame_.data_size);
      }

      // straight up copy
      memcpy(dest, &cconverted.data[0], imgsize);

      item->pitch[0] = image_size_.width * 3;
      item->buf_size[0] = imgsize;
      item->offset[0] = offset;
    }
    else if (output_format_ == tx_format_bgr)
    {
      imgsize = image_size_.width * image_size_.height * 3;
      // Check if the addition of new data will exceed the buffer size.
      if ((offset + imgsize) > frame_.data_size)
      {
        throw VisionException("Overflow buffer {} > {}", (offset + imgsize), frame_.data_size);
      }

      // straight up copy
      memcpy(dest, &input.data[0], imgsize);

      item->pitch[0] = image_size_.width * 3;
      item->buf_size[0] = imgsize;
      item->offset[0] = offset;
    }
    else
    {
      throw VisionException("Invalid format {}", output_format_);
    }
    return imgsize;
  }


  uint32_t NetTxWrapper::packYuvData(EmbeddedNet_PhantomHeader_Info *item, cv::Mat &input, uint8_t* dest, uint32_t offset)
  {
    cv::Mat cconverted;
    uint32_t imgsize = 0;

    if (output_format_ == tx_format_yuv422)
    {
      imgsize = image_size_.width * image_size_.height * 2;
      // Check if the addition of new data will exceed the buffer size.
      if ((offset + imgsize) > frame_.data_size)
      {
        throw VisionException("Overflow buffer {} > {}", (offset + imgsize), frame_.data_size);
      }

      // straight up copy
      memcpy(dest, input.data, imgsize);

      item->pitch[0] = image_size_.width * 2;
      item->buf_size[0] = imgsize;
      item->offset[0] = offset;
    }
    else
    {
      throw VisionException("Invalid format {}", output_format_);
    }
    return imgsize;
  }

  uint32_t NetTxWrapper::packYuv420Data(EmbeddedNet_PhantomHeader_Info *item, cv::Mat &input, uint8_t* dest, uint32_t offset)
  {
    cv::Mat cconverted;
    uint32_t imgsize = 0;

    if (output_format_ == tx_format_yuv420sp)
    {
      imgsize = image_size_.width * image_size_.height * 3 / 2;
      // Check if the addition of new data will exceed the buffer size.
      if ((offset + imgsize) > frame_.data_size)
      {
        throw VisionException("Overflow buffer {} > {}", (offset + imgsize), frame_.data_size);
      }

      // straight up copy
      memcpy(dest, input.data, imgsize);

      item->pitch[0] = image_size_.width;
      item->buf_size[0] = imgsize;
      item->offset[0] = offset;
    }
    else
    {
      throw VisionException("Invalid format {}", output_format_);
    }
    return imgsize;
  }

  void NetTxWrapper::appendVehicleData(uint32_t &offset)
  {
    uint32_t len = sizeof(EmbeddedNet_VehicleInfo);

    // Check if the addition of new data will exceed the buffer size.
    if ((offset + len) > frame_.data_size)
    {
      throw VisionException("Overflow buffer {} > {}. Verify the resolution specified in the vision modes"
        " in the camera calibration settings is large enough to hold the output images.",
        (offset + len), frame_.data_size);
    }

    EmbeddedNet_PhantomHeader_Info *item = &frame_.header.item[frame_.header.item_count];
    item->format = VEHICLE_STATE;
    item->buf_size[0] = len;
    item->offset[0] = offset;
    item->offset[1] = 0;

    uint8_t *dest = (&frame_.data[0] + offset);

    memcpy((void*)dest, (void*)&veh_data_, len);

    frame_.header.item_count++;
    offset += len;
  }

  void NetTxWrapper::appendVisionData(uint32_t &offset)
  {
    if (vision_data_valid_ == false)
    {
      // nothing pending skip it
      return;
    }

    uint32_t result_size = sizeof(EmbeddedNet_PhantomVisionList);

    // Check if the addition of new data will exceed the buffer size.
    if ((offset + result_size) > frame_.data_size)
    {
      throw VisionException("Overflow buffer {} > {}. Verify the resolution specified in the vision modes"
        " in the camera calibration settings is large enough to hold the output images.",
        (offset + result_size), frame_.data_size);
    }

    // update header
    EmbeddedNet_PhantomHeader_Info *item = &frame_.header.item[frame_.header.item_count];
    item->format = VISION_RESULT;
    item->buf_size[0] = result_size;
    item->offset[0] = offset;

    uint8_t *dest = (&frame_.data[0] + offset);

    memcpy((void*)dest, (void*)&vision_data_, result_size);

    frame_.header.item_count++;
    offset += result_size;
    vision_data_valid_ = false;
  }

  void NetTxWrapper::appendDynamicCalibrationData(uint32_t &offset)
  {
    if (dynm_calib_data_valid_ == false)
    {
      // nothing pending skip it
      return;
    }

    uint32_t result_size = sizeof(EmbeddedNet_DynamicCalibrationResultList);

    // Check if the addition of new data will exceed the buffer size.
    if ((offset + result_size) > frame_.data_size)
    {
      throw VisionException("Overflow buffer {} > {}. Verify the resolution specified in the vision modes"
        " in the camera calibration settings is large enough to hold the output images.",
        (offset + result_size), frame_.data_size);
    }

    // update header
    EmbeddedNet_PhantomHeader_Info *item = &frame_.header.item[frame_.header.item_count];
    item->format = DYNM_CALIB_RESULT;
    item->buf_size[0] = result_size;
    item->offset[0] = offset;

    uint8_t *dest = (&frame_.data[0] + offset);

    memcpy((void*)dest, (void*)&dynm_calib_data_, result_size);

    frame_.header.item_count++;
    offset += result_size;
    dynm_calib_data_valid_ = false;
  }

  void NetTxWrapper::appendCamCalibData(uint32_t &offset)
  {
    uint32_t result_size = sizeof(EmbeddedNet_CameraCalibration);

    // Check if the addition of new data will exceed the buffer size.
    if ((offset + result_size) > frame_.data_size)
    {
      throw VisionException("Overflow buffer {} > {}. Verify the resolution specified in the vision modes"
        " in the camera calibration settings is large enough to hold the output images.",
        (offset + result_size), frame_.data_size);
    }

    // update header
    EmbeddedNet_PhantomHeader_Info *item = &frame_.header.item[frame_.header.item_count];
    item->format = CAMERA_CALIB;
    item->buf_size[0] = result_size;
    item->offset[0] = offset;

    uint8_t *dest = (&frame_.data[0] + offset);

    memcpy((void*)dest, (void*)&cam_info_, result_size);

    frame_.header.item_count++;
    offset += result_size;
  }

  void NetTxWrapper::appendCanTxData(uint32_t &offset)
  {
    uint32_t result_size = sizeof(EmbeddedNet_CanFrames);

    // Check if the addition of new data will exceed the buffer size.
    if ((offset + result_size) > frame_.data_size)
    {
      throw VisionException("Overflow buffer {} > {}. Verify the resolution specified in the vision modes"
        " in the camera calibration settings is large enough to hold the output images.",
        (offset + result_size), frame_.data_size);
    }

    // update header>name
    EmbeddedNet_PhantomHeader_Info *item = &frame_.header.item[frame_.header.item_count];
    item->format = CAN_TX;
    item->id = 0; // "CH0"
    item->buf_size[0] = result_size;
    item->offset[0] = offset;

    uint8_t *dest = (&frame_.data[0] + offset);

    memcpy((void*)dest, (void*)&can_tx_data_, result_size);

    frame_.header.item_count++;
    offset += result_size;
  }

  void NetTxWrapper::appendPhantomVisionViz(uint32_t &offset)
  {
    if(!vision_viz_data_valid_)
    {
      return;
    }
    EmbeddedNet_PhantomHeader_Info *item = &frame_.header.item[frame_.header.item_count];
    frame_.header.item_count++;

    uint32_t imgsize = vision_viz_data_.size().width * vision_viz_data_.size().height * 3;

    // Check if the addition of new data will exceed the buffer size.
    if ((offset + imgsize) > frame_.data_size)
    {
      throw VisionException("Overflow buffer {} > {}", (offset + imgsize), frame_.data_size);
    }

    item->format = VISION_VIZ_IMG;
    item->width = vision_viz_data_.size().width;
    item->height = vision_viz_data_.size().height;
    item->pitch[0] = vision_viz_data_.size().width * 3;
    item->buf_size[0] = imgsize;
    item->offset[0] = offset;

    uint8_t *dest = (&frame_.data[0] + offset);

    viz_data_mutex_.lock();
    memcpy(dest, &vision_viz_data_.data[0], imgsize);
    viz_data_mutex_.unlock();
    offset += imgsize;

    vision_viz_data_valid_ = false;
  }

  void NetTxWrapper::appendFusaData(uint32_t &offset)
  {
    if (fusa_data_valid_ == false)
    {
      // nothing pending skip it
      return;
    }

    uint32_t result_size = sizeof(EmbeddedNet_FunctionalSafetyOutputList);

    // Check if the addition of new data will exceed the buffer size.
    if ((offset + result_size) > frame_.data_size)
    {
      throw VisionException("Overflow buffer {} > {}. Verify the resolution specified in the vision modes"
        " in the camera calibration settings is large enough to hold the output images.",
        (offset + result_size), frame_.data_size);
    }

    // update header
    EmbeddedNet_PhantomHeader_Info *item = &frame_.header.item[frame_.header.item_count];
    item->format = FUSA_OUTPUT;
    item->buf_size[0] = result_size;
    item->offset[0] = offset;

    uint8_t *dest = (&frame_.data[0] + offset);

    memcpy((void*)dest, (void*)&fusa_data_, result_size);

    frame_.header.item_count++;
    offset += result_size;
    fusa_data_valid_ = false;
  }

  void NetTxWrapper::appendCameraExtraData(uint32_t &offset)
  {
    if (camera_extra_data_valid_ == false)
    {
      // nothing pending skip it
      return;
    }

    uint32_t result_size = sizeof(EmbeddedNet_CameraExtraInfoListTopLevel);

    // Check if the addition of new data will exceed the buffer size.
    if ((offset + result_size) > frame_.data_size)
    {
      throw phantom_ai::PhantomAIException("Overflow buffer {} > {}", (offset + result_size), frame_.data_size);
    }

    // update header
    EmbeddedNet_PhantomHeader_Info *item = &frame_.header.item[frame_.header.item_count];
    item->format = CAMERA_EXTRA_INFO;
    item->buf_size[0] = result_size;
    item->offset[0] = offset;

    uint8_t *dest = (&frame_.data[0] + offset);

    memcpy((void*)dest, (void*)&camera_extra_data_, result_size);

    frame_.header.item_count++;
    offset += result_size;
    camera_extra_data_valid_ = false;
  }

 
  void NetTxWrapper::appendHbaData(uint32_t &offset)
  {
    if (hba_data_valid_ == false)
    {
      // nothing pending skip it
      return;
    }

    uint32_t result_size = sizeof(EmbeddedNet_PhantomVisionHBAList);

    // Check if the addition of new data will exceed the buffer size.
    if ((offset + result_size) > frame_.data_size)
    {
      throw VisionException("Overflow buffer {} > {}. Verify the resolution specified in the vision modes"
        " in the camera calibration settings is large enough to hold the output images.",
        (offset + result_size), frame_.data_size);
    }

    // update header
    EmbeddedNet_PhantomHeader_Info *item = &frame_.header.item[frame_.header.item_count];
    item->format = HBA_OUTPUT;
    item->buf_size[0] = result_size;
    item->offset[0] = offset;

    uint8_t *dest = (&frame_.data[0] + offset);

    memcpy((void*)dest, (void*)&hba_data_, result_size);

    frame_.header.item_count++;
    offset += result_size;
    hba_data_valid_ = false;
  }

  void NetTxWrapper::appendAebData(uint32_t &offset)
  {
    if (aeb_data_valid_ == false)
    {
      // nothing pending skip it
      return;
    }

    uint32_t result_size = sizeof(EmbeddedNet_PhantomVisionAEBList);

    // Check if the addition of new data will exceed the buffer size.
    if ((offset + result_size) > frame_.data_size)
    {
      throw VisionException("Overflow buffer {} > {}. Verify the resolution specified in the vision modes"
        " in the camera calibration settings is large enough to hold the output images.",
        (offset + result_size), frame_.data_size);
    }

    // update header
    EmbeddedNet_PhantomHeader_Info *item = &frame_.header.item[frame_.header.item_count];
    item->format = AEB_OUTPUT;
    item->buf_size[0] = result_size;
    item->offset[0] = offset;

    uint8_t *dest = (&frame_.data[0] + offset);

    memcpy((void*)dest, (void*)&aeb_data_, result_size);

    frame_.header.item_count++;
    offset += result_size;
    aeb_data_valid_ = false;
  }

  void NetTxWrapper::sendVisionViz(const cv::Mat& vision_viz, const double& ts)
  {
    uint32_t offset = 0;
    uint32_t size = vision_viz.size().width * vision_viz.size().height * 3; // * 3 for RGB
    std::vector<uint8_t> buffer(size);

    Embedded_Frame frame;
    frame.data = buffer.data();
    frame.data_size = size;

    memset(&frame.header, 0, sizeof(EmbeddedNet_PhantomHeader));
    frame.header.timestamp = static_cast<uint64_t>(ts * 1.0e6); // in us
    frame.header.item_count = 1;

    EmbeddedNet_PhantomHeader_Info *item = &frame.header.item[0];
    item->format = VISION_VIZ_IMG;
    item->width = vision_viz.size().width;
    item->height = vision_viz.size().height;
    item->pitch[0] = vision_viz.size().width * 3;
    item->buf_size[0] = size;
    item->offset[0] = offset;

    memcpy(&frame.data[0], &vision_viz.data[0], size);

    if(!sendRawFrame(frame))
    {
      PHANTOM_WARNING("Fail to send vision viz image size:  {}", frame.data_size);
    }

  }

} //namespace
