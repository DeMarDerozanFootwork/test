/*********************************************************************
* @file    tda4x_camera.cpp
* @date    03/04/2020
*
* @attention Copyright (c) 2020
* @attention Phantom AI, Inc.
* @attention All rights reserved.
*********************************************************************/

#include "phantom_ai/core/time_stamp.h"
#include "phantom_ai/core/log.h"
#include "phantom_ai/core/exception.h"
#include "phantom_ai/core/profiler.h"
#include "phantom_ai/core/yaml.h"
#include "phantom_ai/core/paths.h"
#include "phantom_ai/geometry/angle.h"
#include "phantom_ai/phantom_vision2/camera_model.h"

#include <phantom_ai/test/test_log.hpp>

#include "hal_hw.h"
#include "tda4x_camera.h"
#include "multi_cam_tivx.h"

#include <optional>

#define ENABLE_FPS_PRINTS (0)
#define FPS_ADJUST_BY_TIME (0)
#define ENABLE_CONNECTION_CHECKER (0)

// // When set to 1, maintain the target framerate defined in the camera settings
// // yaml file as much as possible. Otherwise the code would begin counting the
// // period for the next frame only starting from when the last frame was output,
// // which can potentially result in a lower framerate.
// #define MAINTAIN_TARGET_FRAMERATE (1)

//#define ENABLE_LATENCY_PRINT
static constexpr uint64_t kCameraFrameHighLatencyFaultThreshold {133};

// this should be 33ms ideally, but 60~70 ms takes time to get the data from a queue inside of HIK SDK.
// + 10 ~ 20 margin = 80 ms
// to-do: we need to figure out the exact reason with HIK
// https://phantomai.atlassian.net/browse/SD3A-2454
static constexpr uint64_t kCameraFrameLatencyWarningThreshold {80};
#if defined(AUTOBRAIN_VH_SPECIFIC_APP) && defined(USE_HIK_CAM_API)
static constexpr double kTimeStampScaleFactor {1e3};
#else
static constexpr double kTimeStampScaleFactor {1e6};
#endif

static constexpr double kUpperBoundTimestamp {8019803577}; // 2224 year
static constexpr double kLowerBoundTimestamp {4389177};    // 1970 year

namespace phantom_ai
{
  tda4x_camera_format convert_tda4x_camera_format(const std::string& str)
  {
    if (str == "bgr24")
    {
      return tda4x_camera_format::format_bgr;
    }
    else if (str == "yuv420")
    {
      return tda4x_camera_format::format_yuv420sp;
    }
    else if (str == "yuv422")
    {
      return tda4x_camera_format::format_yuv422;
    }
    else
    {
      throw VisionException("Unsupported output_format {}", str);
    }
  }

#if defined(SOC_J722S) || defined(SOC_J721S2) || defined(SOC_J721E)
  HAL_CAM_CROP_ID cameraID_to_halCropId(CameraID cam_id)
  {
    switch (cam_id)
    {
      case CAM_FRONT_CENTER:
        return HAL_CAM_CROP_FRONT_CENTER;
      case CAM_FRONT_CENTER_CROP:
        return HAL_CAM_CROP_FRONT_CENTER_CROP;
      case CAM_FRONT_CENTER_WHOLE:
        return HAL_CAM_CROP_FRONT_CENTER_WHOLE;
      case CAM_FRONT_LEFT:
        return HAL_CAM_CROP_FRONT_LEFT;
      case CAM_FRONT_RIGHT:
        return HAL_CAM_CROP_FRONT_RIGHT;
      case CAM_FRONT_CENTER_CROP_BAYER:
      case CAM_FRONT_CENTER_MIDDLE:
      case CAM_FRONT_LEFT_CROP:
      case CAM_FRONT_RIGHT_CROP:
      case CAM_FRONT_CENTER_FULL:
      case CAM_FRONT_CENTER_NARROW:
      case CAM_FRONT_CENTER_NARROW_CROP:
      case CAM_FRONT_CENTER_NARROW_FULL:
      case CAM_FRONT_CENTER_SVM:
      case CAM_FRONT_CENTER_SVM_CROP:
      case CAM_FRONT_CENTER_SVM_FULL:
      case CAM_REAR_CENTER:
      case CAM_REAR_CENTER_CROP:
      case CAM_REAR_CENTER_FULL:
      case CAM_REAR_SIDE_LEFT:
      case CAM_REAR_SIDE_LEFT_CROP:
      case CAM_REAR_SIDE_LEFT_FULL:
      case CAM_REAR_SIDE_RIGHT:
      case CAM_REAR_SIDE_RIGHT_CROP:
      case CAM_REAR_SIDE_RIGHT_FULL:
      case CAM_REAR_CENTER_SVM:
      case CAM_REAR_CENTER_SVM_CROP:
      case CAM_REAR_CENTER_SVM_FULL:
      case CAM_SIDE_LEFT:
      case CAM_SIDE_LEFT_FULL:
      case CAM_SIDE_RIGHT:
      case CAM_SIDE_RIGHT_FULL:
      case CAM_SIDE_FRONT_LEFT:
      case CAM_SIDE_FRONT_LEFT_FULL:
      case CAM_SIDE_FRONT_RIGHT:
      case CAM_SIDE_FRONT_RIGHT_FULL:
      case CAM_SIDE_REAR_LEFT:
      case CAM_SIDE_REAR_LEFT_FULL:
      case CAM_SIDE_REAR_RIGHT:
      case CAM_SIDE_REAR_RIGHT_FULL:
      case CAM_GROUND:
      case CAM_RESERVED:
      case NUM_MAX_CAM_IDS:
      default:
        throw VisionException("CameraID(={}) does not match any defined HAL_CAM_CROP_ID.", phantom_ai::camera_name(cam_id));
    }
  }
#endif

  void checkFrameRateValidity(uint32_t framerate)
  {
    // an example, framerate 4.8
    if ((1 != framerate) &&  // 1 != 4.8: true
        (5 != framerate) &&  // 5 != 4.8: true
       (10 != framerate) && // 10 != 4.8: true
       (15 != framerate) && // 15 != 4.8: true
       (30 != framerate))   // 30 != 4.8: true -> all true -> exception
    {
      throw VisionException("Illegal frame rate {}, valid: 1, 5, 10, 15, 30", framerate);
    }
  }

  void readTda4xCamerasParameters(std::string camera_capture_config_filename,
                                  std::vector<camera_params_tda4x>& cam_params_arr,
                                  CameraInfoList& cam_info_list,
                                  std::vector<int>& annotation_image_widths,
                                  const bool& enable_fullres_logging,
                                  std::string target_system)
  {
    // Load the config.
    YamlNode cfg = load_yaml_file("sensors/camera/tda4x", camera_capture_config_filename);

    std::string mode = get_yaml_value(cfg, "mode").as<std::string>();

    std::string output_format_str = get_yaml_value(cfg, "output_format").as<std::string>();
    tda4x_camera_format output_format = convert_tda4x_camera_format(output_format_str);

    int32_t framerate = get_yaml_value(cfg, "framerate").as<int32_t>();
    checkFrameRateValidity(framerate);

    bool enable_full_res_output = get_yaml_value(cfg, "enable_full_res_output").as<bool>();

    bool dual_logging = get_yaml_value(cfg, "dual_logging").as<bool>();

    std::vector<std::string> active_cams = get_yaml_value(cfg, "active_camera").as<std::vector<std::string>>();
    cam_params_arr.resize(active_cams.size());

    int port_check = 0;
    if (cam_info_list.isEmpty())
    {
      // vehicle config
      std::string vehicle_name = phantom_paths_vehicle_name();
      std::string settings_file = vehicle_name + "/" + get_yaml_value(cfg, "settings_file").as<std::string>();

      // get camera settings
      YamlNode cam_settings = load_yaml_file("sensors/camera", settings_file);
      PHANTOM_LOG("Camera using {}, vehicle {}", settings_file, vehicle_name);

      YamlNode veh_node = get_yaml_key<YamlNode>(cam_settings, vehicle_name, YamlNode());
      if (veh_node.IsNull())
      {
        throw VisionException("Failed to find the vehicle \"{}\" from {}", vehicle_name, settings_file);
      }

      for (size_t i = 0; i < active_cams.size(); i++)
      {
        std::string active_cam_str = active_cams[i];
        CameraID cam_id = camera_name(active_cam_str);

        // Delete '_tda4x' keyword from the camera name
        if (target_system == "tda4x")
        {
          std::string keyword = "_tda4x";
          // #COND if active_cam_str has keyword string "_tda4x"
          if (active_cam_str.find(keyword) == std::string::npos)
          {
            active_cam_str += "_tda4x";
          }
        }

        YamlNode subnode = get_yaml_key<YamlNode>(veh_node, active_cam_str, YamlNode());
        if (subnode.IsNull())
        {
          throw VisionException("Failed to find camera \"{}\" settings!", active_cam_str);
        }

        // check for sequential port numbers. The order is explict in active_camera list. WA rather
        // than try to sort active_cameras by port number
        std::string port = get_yaml_value(subnode, "port").as<std::string>();
        if (port.empty())
        {
          throw VisionException("Wrong port number {} for camera {}", port, active_cam_str);
        }

        //* check if the port string is integer or not
        //* if it's not integer, assume that the port string is {a..z} + {integer string}
        std::string n_port = port.substr((port.find_first_not_of("0123456789") == std::string::npos)? 0 : 1);
        int i_port = std::stoi(n_port);
        if (port_check != i_port)
        {
        //   throw VisionException("active_camera list port order wrong, expect {}, got {} for cam {}", port_check, port, active_cam_str);
        }
        port_check++;

        // save camera params
        CameraInfo cam_info;
        cam_info.serial_number_ = get_yaml_value(subnode, "serial_number").as<std::string>();
        cam_info.sensor_type_ = get_yaml_value(subnode, "sensor_type").as<std::string>();
        cam_info.calib_model_ = get_yaml_value(subnode, "calib_model").as<std::string>();
        cam_info.lens_fov_ = get_yaml_value(subnode, "lens_fov").as<float>();

        auto m = get_yaml_value(subnode, "camera_matrix").as<std::vector<float>>();
        auto d = get_yaml_value(subnode, "distortion").as<std::vector<float>>();
        auto s = get_yaml_value(subnode, "skew").as<std::vector<float>>();
        auto r = get_yaml_value(subnode, "resolution").as<std::vector<int>>();

        cam_info.camera_matrix_ = cv::Mat(3, 3, CV_32F, m.data()).clone();
        cam_info.distortion_ = cv::Mat(1, d.size(), CV_32F, d.data()).clone();
        cam_info.skew_ = cv::Mat(2, 2, CV_32F, s.data()).clone();
        cam_info.resolution_ = cv::Size(r[0], r[1]);

        cam_info.location_ = get_yaml_value(subnode, "location").as<std::string>();
        cam_info.vehicle_ = get_yaml_value(subnode, "vehicle").as<std::string>();
        cam_info.comment_ = get_yaml_value(subnode, "comment").as<std::string>();
        cam_info.position_meter_ = get_yaml_value(subnode, "position_meter").as<std::array<float, 3>>();
        cam_info.rotation_rad_ = get_yaml_value(subnode, "rotation_deg").as<std::array<float, 3>>();
        std::for_each(cam_info.rotation_rad_.begin(), cam_info.rotation_rad_.end(), [](float &x){ x = degree_to_radian(x); });
        cam_info.horizontal_flip_ = get_yaml_value(subnode, "horizontal_flip").as<bool>();
        cam_info.vertical_flip_ = false;

        auto process_roi_node = [=](const std::string& process_mode) {
            YamlNode roi_nodes = get_yaml_key<YamlNode>(subnode, process_mode, YamlNode());
            if (roi_nodes.IsNull()) {
              throw VisionException("Failed to find mode \"{}\" settings!", process_mode);
            }

            std::vector<CameraInfo> processed_crop_infos; // Store processed crop_infos
            std::vector<int> processed_annotation_image_widths; // Store processed annotation image widths
            for (auto roi_node : roi_nodes) {
              CameraInfo crop_info = cam_info;
              std::string subimg_name = roi_node.first.as<std::string>();

              auto mode_cfg = roi_node.second.as<std::vector<std::vector<int>>>();
              auto c = mode_cfg[0];
              cv::Rect img_rect = cv::Rect(c[0], c[1], c[2], c[3]);

              auto output = mode_cfg[1];
              int32_t scalar_image_width = output[0];
              int32_t scalar_image_height = output[1];

              crop_info.name_ = subimg_name;
              crop_info.location_ = subimg_name;
              crop_info.crop_roi_ = img_rect;
              crop_info.src_roi_ = crop_info.crop_roi_;
              crop_info.output_resolution_ = cv::Size(scalar_image_width, scalar_image_height);

              processed_crop_infos.push_back(std::move(crop_info)); // Move crop_info into the vector

              if (process_mode == "annotation_mode"){
                processed_annotation_image_widths.push_back(img_rect.width);
              }
            }

            if (enable_fullres_logging)
            {
              CameraInfo crop_info = cam_info;
              std::string subimg_name = cam_info.location_ + "_full";
              crop_info.name_ = subimg_name;
              crop_info.location_ = subimg_name;
              crop_info.crop_roi_ = cv::Rect(0, 0, cam_info.resolution_.width, cam_info.resolution_.height);
              crop_info.src_roi_ = crop_info.crop_roi_;
              crop_info.output_resolution_ = cam_info.resolution_;
              processed_crop_infos.push_back(std::move(crop_info));
            }

            return std::make_pair(processed_crop_infos, processed_annotation_image_widths);
        };

        if (dual_logging) {
          // Process vision mode
          auto result_vision = process_roi_node("vision_mode");
          for (const auto& crop_info : result_vision.first) {
            cam_info_list.insert(cam_id, crop_info);
          }

          // Process annotation mode
          auto result_annotation = process_roi_node("annotation_mode");
          for (const auto& crop_info : result_annotation.first) {
            cam_info_list.insert(cam_id, crop_info);
          }
          for (const auto& width : result_annotation.second) {
            annotation_image_widths.push_back(width);
          }
        } else {
          auto result_mode = process_roi_node(mode);
          for (const auto& crop_info : result_mode.first) {
            cam_info_list.insert(cam_id, crop_info);
          }
          if (mode == "annotation_mode"){
            for (const auto& width : result_mode.second) {
              annotation_image_widths.push_back(width);
            }
          }
        }
      }
    }

    for (size_t i = 0; i < active_cams.size(); i++)
    {
      camera_params_tda4x& cam_params = cam_params_arr[i];

      const std::string& active_cam_str = active_cams[i];
      cam_params.location = camera_name(active_cam_str);
      cam_params.output_format = output_format;
      cam_params.framerate = framerate;

      auto crops = cam_info_list.getCropsInfo(cam_params.location);

      if (crops.empty())
      {
        throw VisionException("No crop information");
      }

      cam_params.type = (crops.begin())->second.sensor_type_;
      cam_params.sensor_width = (crops.begin())->second.resolution_.width;
      cam_params.sensor_height = (crops.begin())->second.resolution_.height;

      for (auto it : crops)
      {
        CameraInfo& crop_info = it.second;
        camera_subimage_tda4x sub_img;

        if (enable_full_res_output)
        {
          if ((crop_info.name_ == "front_center") || (crop_info.name_ == "rear_center") ||
              (crop_info.name_ == "side_front_left") || (crop_info.name_ == "side_front_right") ||
              (crop_info.name_ == "side_rear_left") || (crop_info.name_ == "side_rear_right"))
          {
            sub_img.roi = (cv::Rect(0, 0, cam_params.sensor_width, cam_params.sensor_height));
            sub_img.output_width  = (cam_params.sensor_width);
            sub_img.output_height = (cam_params.sensor_height);
            sub_img.location = camera_name(crop_info.name_, true);

            PHANTOM_WARNING_IF(0, "{}", it.first)
            PHANTOM_WARNING_IF(0, "  ROI {}, {}, {}, {}", sub_img.roi.x, sub_img.roi.y, sub_img.roi.width, sub_img.roi.height)
            PHANTOM_WARNING_IF(0, "  OUT {}, {}", sub_img.output_width, sub_img.output_height)
            PHANTOM_WARNING_IF(0, "  LOC {}", crop_info.name_)
            cam_params.subimage.push_back(sub_img);
          }
          else
          {
            ; // skip non-main crops.
          }
        }
        else
        {
          sub_img.roi = (cv::Rect(crop_info.src_roi_.x, crop_info.src_roi_.y, crop_info.src_roi_.width, crop_info.src_roi_.height));
          sub_img.output_width  = (crop_info.output_resolution_.width);
          sub_img.output_height = (crop_info.output_resolution_.height);
          sub_img.location = camera_name(crop_info.name_, true);

          PHANTOM_WARNING_IF(0, "{}", it.first)
          PHANTOM_WARNING_IF(0, "  ROI {}, {}, {}, {}", sub_img.roi.x, sub_img.roi.y, sub_img.roi.width, sub_img.roi.height)
          PHANTOM_WARNING_IF(0, "  OUT {}, {}", sub_img.output_width, sub_img.output_height)
          PHANTOM_WARNING_IF(0, "  LOC {}", crop_info.name_)
          cam_params.subimage.push_back(sub_img);
        }
      }

      cam_params.dual_logging = dual_logging;
    }
  }

  Tda4xCamera::Tda4xCamera(std::vector<phantom_ai::camera_params_tda4x>& config_arr, bool use_full_res_logging)
    : quit_loop_(false),
    frame_counter_(0),
    ts_image_last_(0.0),
    ts_remainder_(0.0),
    ts_min_interval_(0.0),
    output_enable_(false),
    params_{},
    image_count_(0),
    dual_logging_{},
    use_fullres_logging_(use_full_res_logging)
  {
    // #PROC Initiate ti hardware driver
    hw_ptr_ = HalBaseHw::getInstance();
    // #END

    locations_.resize(config_arr.size());
    camera_count_ = config_arr.size();

    // #COND For converting the given configuration array to camera driver struct
    for (size_t i = 0; i < camera_count_; i++)
    {
      params_[i].camera_type = convertSensorType(config_arr[i].type);
      // TDA4x driver outputs YUV420, do conversion locally
      params_[i].output_format = 0;
      params_[i].framerate = config_arr[i].framerate;
#if defined(SOC_J722S) || defined(SOC_J721S2)
      // Tda4AEN, TDA4VMEco provides this API.
#if defined(TARGET_BOARD_EVM_AL_86)
      frame_drop_idx_ = 1;
#else
      params_[i].capture_framerate = tivx_get_camera_capture_fps();
#endif
#else
      // The other SoCs assume that the camera capture rate is 30 fps.
      params_[i].capture_framerate = 30;
#endif // defined(SOC_J722S)
#ifndef TARGET_BOARD_EVM_AL_86
      checkFrameRateValidity(params_[i].capture_framerate);
#endif
      params_[i].sensor_width = config_arr[i].sensor_width;
      params_[i].sensor_height = config_arr[i].sensor_height;
      params_[i].num_subimages = 0;
      // #COND if the size of sub image is defined number from the sdk
      if (config_arr[i].subimage.size() > TIVX_MAX_CROPS)
      {
        throw VisionException("Exceeded max number of subimages");
      }
      // #COND For storing sub image data into local buffer
      //       Increase counter for the number of sub images to check the value afterward
      for (auto &subimg : config_arr[i].subimage)
      {
        auto &cfg = params_[i].subimage[params_[i].num_subimages];
#if defined(SOC_J722S) || defined(SOC_J721S2) || defined(SOC_J721E)
        HAL_CAM_CROP_ID hal_crop_id = cameraID_to_halCropId(subimg.location);
        mapCropIdToSubImageIdx(hal_crop_id, params_[i].num_subimages);
#endif
        cfg.start_x = subimg.roi.x;
        cfg.start_y = subimg.roi.y;
        cfg.width = subimg.roi.width;
        cfg.height = subimg.roi.height;
        cfg.output_width = subimg.output_width;
        cfg.output_height = subimg.output_height;

        image_count_++;
        params_[i].num_subimages++;
        locations_[i].push_back(subimg.location);
      }

      dual_logging_[i] = config_arr[i].dual_logging;
    }
    // #COND Check if the counted sub images exceeds the maximum value(20) and throwing the corresponding log
    if (image_count_ > MAX_IMAGES)
    {
      throw VisionException("Exceeded max number of images");
    }

    // #PROC Store given output format
    output_format_ = config_arr[0].output_format; //assume all configs have same setting
    // #END

    // #PROC Make minimum time interval value to calculate FPS
#if FPS_ADJUST_BY_TIME
    constexpr double ts_margin = 0.005;
    ts_min_interval_ = (1.0 / static_cast<double>(params_[0].framerate)) - ts_margin;
#else
#ifndef TARGET_BOARD_EVM_AL_86
    // frame_drop_idx_ is n number of frames where 1 frame is kept and rest are dropped.
    if(params_[0].capture_framerate < params_[0].framerate)
    {
      // The capture framerate is slower than the perception framerate.
      // for example capture fps 15, but the perception tries to read the frame faster like 30 fps.
      frame_drop_idx_ = 1; // set the default value, no-drop, 1
      PHANTOM_ERROR("capture framerate != perception framerate");
    }
    else
    {
      // prevent divide-by-zero just in case even though we check the validity with checkFrameRateValidity().
      frame_drop_idx_ = (params_[0].framerate == 0) ? 1 : params_[0].capture_framerate / params_[0].framerate;
    }
#endif
#endif
    // #END

    // #PROC Initiate sdk driver with the given camera parameters
    //       and trigger the runtim camera loop
#if defined(AUTOBRAIN_VH_SPECIFIC_APP) && defined(USE_HIK_CAM_API)
    handle_ = hik_multi_cam_init(params_, camera_count_);
#else
    handle_ = tivx_multi_cam_init(params_, camera_count_);
#endif
    if(nullptr != handle_)
    {
      quit_loop_ = false;
      main_thread_ = std::thread(&Tda4xCamera::runCameraLoop, this);
      auto handle = main_thread_.native_handle();
      pthread_setname_np(handle, "CameraMain");
    }
    else
    {
      PHANTOM_ERROR("A nullptr handle error occurred duing camera init");
    }

    // #IGNORE
    PHANTOM_LOG("[Tda4xCamera] has initialized.");
    // #END
  }

  Tda4xCamera::~Tda4xCamera()
  {
    PHANTOM_LOG("Destructing Tda4xCamera");

    // #PROC Change the state of the loop to spin down the loops
    quit_loop_ = true;
    // #END

    // #COND Wait thread for runCameraLoop is finish its execution
    if (main_thread_.joinable())
    {
      main_thread_.join();
    }

    // #PROC Delete the sdk driver
#if defined(AUTOBRAIN_VH_SPECIFIC_APP) && defined(USE_HIK_CAM_API)
    hik_multi_cam_delete(&handle_);
#else
    tivx_multi_cam_delete(&handle_);
#endif
    // #END

    // #PROC Release the instance
    if (hw_ptr_ != nullptr) hw_ptr_->reset();
    // #END
  }

#if defined(SOC_J722S) || defined(SOC_J721S2) || defined(SOC_J721E)
  cv::Rect Tda4xCamera::resizedROIToOriginalCoords(CameraID cam_id, const cv::Rect &roi_resized, CameraModelListS& cam_models)
  {
    CameraModelS cam_model = cam_models->camera_model(cam_id);
    CameraInfo cam_model_info;
    Roi_Window parent_subimg_config;
    if (cam_model)
    {
       cam_model_info = cam_model->cameraInfo();
       parent_subimg_config = params_[HAL_CAM_FRONT_ID].subimage[halCropIdToSubImgIdx(cameraID_to_halCropId(camera_name(cam_model_info.parent_)))];
    }
    else
    {
       PHANTOM_ERROR("cam_model is unexpectedly null in Tda4xCamera::resizedROIToOriginalCoords");
       return cv::Rect(-1,-1,-1,-1);
    }

    // Calculate the original ROI coordinates
    float scale_x, scale_y;
    int x, y, width, height;
    if (cam_model_info.is_virtual_)
    {
      // When crop is virtual, use subimg_config from parent
      scale_x = static_cast<float>(parent_subimg_config.width) / parent_subimg_config.output_width;
      scale_y = static_cast<float>(parent_subimg_config.height) / parent_subimg_config.output_height;
      x = cam_model_info.crop_roi_.x + static_cast<int>(std::round(roi_resized.x * scale_x));
      y = cam_model_info.crop_roi_.y + static_cast<int>(std::round(roi_resized.y * scale_y));
      width = static_cast<int>(std::round(roi_resized.width * scale_x));
      height = static_cast<int>(std::round(roi_resized.height * scale_y));
    }
    else
    {
      // Only non-virtual crops have a subimg_config from which info can be obtained
      auto subimg_config = params_[HAL_CAM_FRONT_ID].subimage[halCropIdToSubImgIdx(cameraID_to_halCropId(cam_id))];
      scale_x = static_cast<float>(subimg_config.width) / subimg_config.output_width;
      scale_y = static_cast<float>(subimg_config.height) / subimg_config.output_height;
      x = subimg_config.start_x + static_cast<int>(std::round(roi_resized.x * scale_x));
      y = subimg_config.start_y + static_cast<int>(std::round(roi_resized.y * scale_y));
      width = static_cast<int>(std::round(roi_resized.width * scale_x));
      height = static_cast<int>(std::round(roi_resized.height * scale_y));
    }

    // Align x and y to be multiples of 2
    int aligned_x = x & ~1;
    int aligned_y = y & ~1;

    // Adjust width and height
    int x_offset = x - aligned_x;
    int y_offset = y - aligned_y;

    int aligned_width = width + x_offset;
    int aligned_height = height + y_offset;

    // Adjust width and height to be multiples of 2
    aligned_width = (aligned_width + 1) & ~1;
    aligned_height = (aligned_height + 1) & ~1;

    return cv::Rect(aligned_x, aligned_y, aligned_width, aligned_height);
  }

  void Tda4xCamera::mapCropIdToSubImageIdx(HAL_CAM_CROP_ID crop_id, uint32_t subimage_idx)
  {
    if (subimage_idx >= HAL_CAM_CROP_MAX)
    {
        throw VisionException("Subimage index {} exceeds max allowed {}", subimage_idx, HAL_CAM_CROP_MAX);
    }

    cropid_to_subimage_map_[crop_id] = subimage_idx;
  }

  uint32_t Tda4xCamera::halCropIdToSubImgIdx(HAL_CAM_CROP_ID crop_id)
  {
    return cropid_to_subimage_map_[crop_id];
  }
#endif

  // Start video capture
  void Tda4xCamera::startStreaming()
  {
    // #PROC Turn on the state of streaming function
    output_enable_ = true;
    // #END

    PHANTOM_LOG("Start Streaming.");
  }

  // Stop video capture
  void Tda4xCamera::stopStreaming()
  {
    // #PROC Turn off the state of streaming function
    output_enable_ = false;
    // #END

    PHANTOM_LOG("Stop Streaming.");
  }

  // Register call back function
  void Tda4xCamera::registerRxCallback(std::function<void(std::vector<CameraResults>&, const std::unordered_map<phantom_ai::CameraID, std::shared_ptr<phantom_ai::CameraExtraInfo>>&)> callback_func)
  {
    image_callback_ = callback_func;
  }

  // Check if each camera mounted in wrong location
  void Tda4xCamera::checkCameraLocation(uint8_t wrong_camera_connected[], uint8_t num_of_cam)
  {
    // To-do: tivx_check_camera_location(wrong_camera_connected, num_of_cam);
    // Currently all fine.
    // #COND Check each camera locations
    for(uint8_t i = 0; i < num_of_cam; i++)
    {
      // #PROC Store the fault status of the location
      // no fault for 0
      wrong_camera_connected[i] = 0;
      // #END
    }
  }

  // Check if each camera is properly connected
  bool Tda4xCamera::isCameraConnected(phantom_ai::CameraResults)
  {
#if ENABLE_CONNECTION_CHECKER
    // check output_format_ per camera later with cam_result.id
    // currently assume all configs have same setting

    cv::Mat img_final;

    // #COND Check if the output format is bgr
    if(format_bgr == output_format_)
    {
      // #PROC already converted to BGR in runCameraLoop
      img_final = cam_result.image;
      // #END
    }
    // #COND Check if the output format is yuv420
    else if(format_yuv420sp == output_format_)
    {
      // #PROC need to convert to GRAY image to check black image
      cv::cvtColor(cam_result.image, img_final, cv::COLOR_YUV2GRAY_420);
      // #END
    }
    else
    {
      ; // do nothing, should have checked at init
    }

    // #PROC Check if the gathered image is all black with using max value
    bool is_camera_connected = false;
    double minVal{}, maxVal{};
    cv::minMaxLoc(img_final, &minVal, &maxVal);
    // #END

    // #COND Check if the maximum value of the image is 0
    // 0: black -> maxVal 0 means all black
    if(0 == maxVal)
    {
      // #PROC Notify wrong camera connection
      is_camera_connected = false;
      PHANTOM_ERROR("black image!!!");
      // #END
    }
    // #COND in case of none-zero maximum value
    else
    {
      // #PROC Update the state of the variable for camera connection
      is_camera_connected = true;
      // #END
    }

    // #PROC Return the state of the camera connection
    return is_camera_connected;
    // #END
#else
    return true;
#endif
  }

CAMERA_STATUS Tda4xCamera::CheckHighLatency(uint64_t timestamp)
{
  uint64_t ts_present = phantom_ai::TimeStamp::Now().toUSec()/ 1e3;
  if ((ts_present - timestamp) > kCameraFrameHighLatencyFaultThreshold)
  {
#if defined(ENABLE_LATENCY_PRINT)
    PHANTOM_ERROR("HIK CAM DELAY ERROR: arrival timestamp({}) - image capture timestamp({}) = {}",
                        ts_present,
                        timestamp,
                        ts_present - timestamp);
#endif // #if defined(ENABLE_LATENCY_PRINT)
    return HIK_CAM_LATENCY_FAIL;
  }
  else if ((ts_present - timestamp) > kCameraFrameLatencyWarningThreshold)
  {
#if defined(ENABLE_LATENCY_PRINT)
    PHANTOM_WARNING("HIK CAM DELAY WARNING: arrival timestamp({}) - image capture timestamp({}) = {}",
                        ts_present,
                        timestamp,
                        ts_present - timestamp);
#endif // #if defined(ENABLE_LATENCY_PRINT)
    return HIK_CAM_SUCCESS;
  }
  else
  {
    return HIK_CAM_SUCCESS;
  }
}


  // private

  // Run time camera loop functioin
  void Tda4xCamera::runCameraLoop()
  {
#if ENABLE_FPS_PRINTS
    double loop_time = 0.0;
    double ts_print_ = 0.0;
    uint32_t counter = 0;
#endif

#ifdef ENABLE_FETCH_CAMERA_EXTRA_DATA
    char *image_arr[image_count_ + NUM_CAMERA_EXTRA_DATA] = {};
    uint32_t idx_cam_extra_data0 = image_count_ +  CAMERA_EXTRA_DATA_INDEX_OFFSET;
#else
    char *image_arr[image_count_] = {};
#endif

	// #COND Do unless the instance is activated
    while (quit_loop_ == false)
    {
      PHANTOM_AI_PROFILE_SCOPE("tda4x_camera_runloop");
      TESTLOG(logCameraInputSeq("Tda4xCamera::onCameraDataReceived"));

#ifdef ENABLE_FETCH_CAMERA_EXTRA_DATA
      std::vector<CameraResults> results(image_count_ + NUM_CAMERA_EXTRA_DATA);
#else
      std::vector<CameraResults> results(image_count_);
#endif
      // in usec
      uint64_t image_time[image_count_] = {};

      uint32_t idx = 0;
      // #COND For camera count
      for (uint32_t i = 0; i < camera_count_; i++)
      {
        // #COND For each numbers of sub images
		for (uint32_t j = 0; j < params_[i].num_subimages; j++)
        {
          // #PROC store output buffers for all images
#if !defined(AUTOBRAIN_VH_SPECIFIC_APP) || !defined(USE_HIK_CAM_API)
          auto &subimg = params_[i].subimage[j];
          cv::Size buf_size(subimg.output_width, subimg.output_height * 3 / 2);
          results[idx].image = cv::Mat(buf_size, CV_8UC1);
          if (results[idx].image.data != nullptr) image_arr[idx] = reinterpret_cast<char*>(results[idx].image.data);
#endif
          results[idx].id = locations_[i][j];
          results[idx].status = HIK_CAM_SUCCESS;
          idx++;
          // #END
        }
      }

#ifdef ENABLE_FETCH_CAMERA_EXTRA_DATA
      if(idx == idx_cam_extra_data0)
      {
        cv::Size size_cv_buf_cam = cv::Size(1,sizeof(CameraSuppInfo_t));
        results[idx_cam_extra_data0].extra_data_mat = cv::Mat::zeros(size_cv_buf_cam, CV_8UC1);
        results[idx_cam_extra_data0].id = CAM_FRONT_CENTER;
        results[idx_cam_extra_data0].status = HIK_CAM_SUCCESS;
        // Note that this extra data buffer is passed in as input to hik_multi_cam_process, whereas
        // the rest of the pointers in image_arr are set by hik_multi_cam_process as output parameters.
        image_arr[idx_cam_extra_data0] = (char*)results[idx_cam_extra_data0].extra_data_mat.data;
      }
      else
      {
        PHANTOM_ERROR("'idx_cam_extra_data0' does not continue from 'image_count_'\n");
      }
#endif

      {
		// #PROC Invoke sdk driver API to process each image inputs
#if defined(AUTOBRAIN_VH_SPECIFIC_APP) && defined(USE_HIK_CAM_API)
        PHANTOM_AI_PROFILE_SCOPE("hik_multi_cam_process");

#ifdef ENABLE_FETCH_CAMERA_EXTRA_DATA
        hik_multi_cam_process(handle_, image_arr, image_time, idx_cam_extra_data0);
#else
        hik_multi_cam_process(handle_, image_arr, image_time);
#endif
        // #END
        CAMERA_STATUS camera_status[TOTAL_CAMERA_GROUPS] = {HIK_CAM_SUCCESS,HIK_CAM_SUCCESS,HIK_CAM_SUCCESS};

		    // #PROC Invoke sdk driver API to get each sensor status
        hik_multi_cam_get_status(handle_, camera_status);
        // #END

        CAMERA_STATUS high_latency_status{CheckHighLatency(image_time[FRONT_CAMERA_GROUP_ID])};

        uint32_t idx = 0;
        for (uint32_t i = 0; i < camera_count_; i++)
        {
          for (uint32_t j = 0; j < params_[i].num_subimages; j++)
          {
            // Wrap buffers returned by HIK driver into Mat
            auto &subimg = params_[i].subimage[j];
            cv::Size buf_size(subimg.output_width, subimg.output_height * 3 / 2);
            results[idx].image = cv::Mat(buf_size, CV_8UC1, image_arr[idx]);

            switch(i)
            {
              case FRONT_CAMERA_GROUP_ID:
                results[idx].status = static_cast<CAMERA_STATUS>(camera_status[FRONT_CAMERA_GROUP_ID] | high_latency_status);
                break;
              case REAR_CAMERA_GROUP_ID:
                results[idx].status = camera_status[REAR_CAMERA_GROUP_ID];
                break;
              case SIDE_CAMERA_GROUP_ID:       [[fallthrough]];
              default:
                results[idx].status = camera_status[SIDE_CAMERA_GROUP_ID];
                break;
            }
            results[idx].timestamp = (double)image_time[idx] / kTimeStampScaleFactor;
            idx++;
          }
        }

        for (uint32_t i = 0; i < TOTAL_CAMERA_GROUPS; i++)
        {
          if(HIK_CAM_SUCCESS != camera_status[i])
          {
            PHANTOM_ERROR("hik_multi_cam_process failed for camera{}: status=0x{:x}",i, camera_status[i]);
          }
        }

#ifdef ENABLE_FETCH_CAMERA_EXTRA_DATA
        if(image_count_ > 0)
        {
          CameraSuppInfo_t* extra_data = (CameraSuppInfo_t*)results[idx_cam_extra_data0].extra_data_mat.data;
          results[idx_cam_extra_data0].status = ( extra_data->is_valid_exp_front_ && extra_data->is_valid_raw_img_) ? HIK_CAM_SUCCESS : HIK_CAM_TIMEOUT_FAIL;
        }
#endif //end of #ifdef ENABLE_FETCH_CAMERA_EXTRA_DATA

#else
		// #COND Check if sdk API returns NG
        PHANTOM_AI_PROFILE_SCOPE("tivx_multi_cam_process");
#if defined(SOC_J722S) || defined(SOC_J721S2) || defined(SOC_J721E)
        uint32_t frame_number;
        if (tivx_multi_cam_process(handle_, image_arr, &frame_number, &image_time[0]) != 0)
#else
        if (tivx_multi_cam_process(handle_, image_arr, &image_time[0]) != 0)
#endif
        {
          throw VisionException("Failed camera process");
        }
        else
        {
          // #COND Update all timestamps and frame numbers
          for (uint32_t i = 0; i < image_count_; i++)
          {
#if defined(SOC_J722S) || defined(SOC_J721S2) || defined(SOC_J721E)
            results[i].frame_number = frame_number;
#endif
            results[i].timestamp = image_time[0] / kTimeStampScaleFactor;
            if (results[i].timestamp < kLowerBoundTimestamp || results[i].timestamp > kUpperBoundTimestamp){
              PHANTOM_ERROR("Camera Timestamp is Out of Range here");
            }
          }
        }
#endif // #ifdef AUTOBRAIN_VH_SPECIFIC_APP
      }

	  // #PROC Swapping UV
	  // default off
#if defined(ENABLE_CAMERA_SWAP_UV)
      {
      PHANTOM_AI_PROFILE_SCOPE("camera_swap_up");
      // swap UV
      idx = 0;
      for (uint32_t i = 0; i < image_count_; i++)
      {
        for (uint32_t j = 0; j < params_[i].num_subimages; j++)
        {
          uint32_t y_size = params_[i].subimage[j].output_width * params_[i].subimage[j].output_height;
          uint32_t uv_size = params_[i].subimage[j].output_width * params_[i].subimage[j].output_height / 2;
          uint32_t* addr = (uint32_t*)(results[idx++].image.data + y_size);
          for (uint32_t cnt = 0; cnt < uv_size; cnt+=4)
          {
            uint32_t val = *addr;
            val = ((val >> 8) & 0x00FF00FF) | ((val << 8) & 0xFF00FF00);
            *addr = val;
            addr++;
          }
        }
      }
      }
#endif
      // #END

      // #PROC FPS adjustment by time
#if FPS_ADJUST_BY_TIME
      auto ts_start = phantom_ai::TimeStamp::Now().toSec();
      double delta = ts_start - ts_image_last_ + ts_remainder_;
      // #END

      // #COND Check the delta time is usable
      if (delta > ts_min_interval_ || delta < 0.0)
      {
        // #PROC Store present time
        ts_image_last_ = ts_start;

        // currently index 0 of dual_logging_ is the representative.

        if(true == dual_logging_[0])

        {

          // maintain the target framerate defined in the camera settings

          // yaml file as much as possible. Otherwise the code would begin counting the

          // period for the next frame only starting from when the last frame was output,

          // which can potentially result in a lower framerate.

          ts_remainder_ = std::fmod(delta, ts_min_interval_);

        } else {;}
// #if MAINTAIN_TARGET_FRAMERATE
//         ts_remainder_ = std::fmod(delta, ts_min_interval_);
// #endif // MAINTAIN_TARGET_FRAMERATE
        // #END
      }
      else
      {
        // #PROC Skip frame
        //PHANTOM_LOG("Skip frame {:.06f}", delta);
#if defined(AUTOBRAIN_VH_SPECIFIC_APP) && defined(USE_HIK_CAM_API)
        hik_multi_cam_release(handle_);
#endif
        continue;
        // #END
      }
#else
      frame_counter_++;

      // #COND Do frame rate adjustment here
      if ((frame_counter_ % frame_drop_idx_) != 0)
      {
        // skip frame
        //PHANTOM_LOG("Skip frame {}", frame_counter_);
#if defined(AUTOBRAIN_VH_SPECIFIC_APP) && defined(USE_HIK_CAM_API)
        hik_multi_cam_release(handle_);
#endif
        continue;
      }
#endif

      // #COND Check if streaming is started
      if (output_enable_)
      {
        // Converting output format according to given configuration
        // #COND YUV420 to BGR
        if (output_format_ == tda4x_camera_format::format_bgr)
        {
          PHANTOM_AI_PROFILE_SCOPE("yuv2bgr_nv12");
          for (uint32_t i = 0; i < image_count_; i++)
          {
            if (use_fullres_logging_ && is_full_resolution_camera(results[i].id))
              continue;

            // #PROC convert to BGR output
            cv::Mat img_final;
            cv::cvtColor(results[i].image, img_final, cv::COLOR_YUV2BGR_NV12);
            results[i].image = img_final;
			// #END
          }
        }
        // #COND Keep YUV420 format
        else if (output_format_ == tda4x_camera_format::format_yuv420sp)
        {
          // nothing to do, driver already output yuv420
        }
        // #COND Neither BGR nor YUV420 format
        else
        {
          // nothing, should have checked at init
        }
        // #END

        // Send images through the callback
        // #COND Check the callback is not null pointer
        if (image_callback_ != nullptr)
        {
          PHANTOM_AI_PROFILE_SCOPE("image_callback");
          for (uint32_t i = 0; i < image_count_; i++)
          {
            isCameraConnected(results[i]);
          }
          // #PROC Invoke the callback
          std::unordered_map<CameraID, std::shared_ptr<CameraExtraInfo>> dummy;
          image_callback_(results, dummy);
          // #END
        }
      }

#if defined(AUTOBRAIN_VH_SPECIFIC_APP) && defined(USE_HIK_CAM_API)
      hik_multi_cam_release(handle_);
#endif

      // ENABLE_FPS_PRINTS
#if ENABLE_FPS_PRINTS
      auto ts_end = phantom_ai::TimeStamp::Now().toSec();
      counter++;

      loop_time += ts_end - ts_start;

      // # IGNORE debugging
      if (ts_end - ts_print_ > 4.0)
      {
        // note this is for one image, so 2x
        PHANTOM_LOG("Camera fps {:.02f}, avg {} ms",
          counter/(ts_end - ts_print_), (loop_time / counter * 1000));
        ts_print_ = ts_end;
        counter = 0;
        loop_time = 0.0;
      }
	  // #END

#endif

      // #COND Check if stop of streaming is reuqested
      if (quit_loop_ == true)
      {
        // #PROC Quit the run loop sdfimmediately
        PHANTOM_LOG("Quitting Camera Loop");
        return;
        // #END
      }
    }
  }

#if defined(SOC_J722S) || defined(SOC_J721S2) || defined(SOC_J721E)
  std::optional<FrameData> Tda4xCamera::getFullResolutionFrameROI(CameraID cam_id, uint32_t frame_number, const cv::Rect& roi, PixelFormat fmt,
      CameraModelListS& cam_models)
  {
    if(handle_ == nullptr)
    {
      PHANTOM_ERROR("Handle is null.\n");
      return std::nullopt;
    }

    FrameData roi_frame_out;
    HAL_Frame_Info hal_frames_out;
    // Transform the ROI from resized image to original image coordinates
    cv::Rect roi_original = resizedROIToOriginalCoords(cam_id, roi, cam_models);
    if (roi_original.width == -1) {
      return std::nullopt;
    }

    // Prepare frames_out and hal_frames_out with NV12 format
    cv::Size buf_size(roi_original.width, roi_original.height * 3 / 2);
    roi_frame_out.image = cv::Mat(buf_size, CV_8UC1);
    roi_frame_out.image_format = PixelFormat::NV12;
    roi_frame_out.crop_rect = roi_original;
    hal_frames_out.data = reinterpret_cast<uint8_t*>(roi_frame_out.image.data);
    HAL_ROI hal_roi = {(uint32_t)roi_original.x, (uint32_t)roi_original.y, (uint32_t)roi_original.width, (uint32_t)roi_original.height};

    int retrieved_count = tivx_camera_fetch_full_resolution_frame_roi(handle_, frame_number, &hal_roi, &hal_frames_out);
    if (retrieved_count <= 0)
    {
      if (retrieved_count < 0)
      {
        PHANTOM_ERROR("Failed to fetch full-resolution frame roi. Status code: {}\n", retrieved_count);
      }
      return std::nullopt;
    }

    // Populate timestamp and frame_number
    roi_frame_out.timestamp = static_cast<double>(hal_frames_out.timestamp_us) / kTimeStampScaleFactor;
    roi_frame_out.frame_number = hal_frames_out.frame_count;

    // If requested format is BGR, convert each frame from NV12 to BGR
    if (fmt == PixelFormat::BGR)
    {
      cv::Mat bgr_image;
      try
      {
        cv::cvtColor(roi_frame_out.image, bgr_image, cv::COLOR_YUV2BGR_NV12);
      }
      catch (const cv::Exception& e)
      {
        PHANTOM_ERROR("cvtColor failed : {}\n", e.what());
        roi_frame_out.image = cv::Mat(); // empty image
      }

      roi_frame_out.image = bgr_image;
      roi_frame_out.image_format = PixelFormat::BGR;
    }

    return roi_frame_out;
  }
#endif

  // Function for converting sensor type
  TIVX_CAMERA_TYPES Tda4xCamera::convertSensorType(const std::string name)
  {
    // #PROC Set CAM_IMX390 by default
    TIVX_CAMERA_TYPES retval = CAM_IMX390;
    // #END

    // #COND In case of imx390
    if (name == "imx390")
    {
      // #PROC Set imx390 for return value
      retval = CAM_IMX390;
      // #END
    }
    // #COND In case of ar0820
    else if (name == "ar0820")
    {
      // #PROC Set imx390 for return value
      retval = CAM_AR0820;
      // #END
    }
    // #COND In case of ox08b40
    else if (name == "ox08b40")
    {
      // #PROC Set ox08b40 for return value
      retval = CAM_OX08B40;
      // #END
    }
    // #COND In case of ox03c10
    else if (name == "ox03c10")
    {
      // #PROC Set ox03c10 for return value
      retval = CAM_OX03C10;
      // #END
    }
#if defined(SOC_J722S)
    // #COND In case of imx728
    else if (name == "imx728")
    {
      // #PROC Set imx728 for return value
      retval = CAM_IMX728;
      // #END
    }
#endif
    // #COND Handling for unsupported type sensor
    else
    {
      throw VisionException("Unsupport sensor type {}", name);
    }

    // #PROC Return the camera type
    return retval;
    // #END
  }
#if defined(ENABLE_GSTREAMER)
  void Tda4xCamera::getMosaicDimensions(uint32_t& width, uint32_t& height)
  {
    tivx_multi_cam_get_mosaic_dimensions(&width, &height);
  }

  void Tda4xCamera::registerEncoderCallback(EncoderFeedFrameCallback callback)
  {
    tivx_multi_cam_register_encoder_callback(callback);
  }
#endif
} //namespace
