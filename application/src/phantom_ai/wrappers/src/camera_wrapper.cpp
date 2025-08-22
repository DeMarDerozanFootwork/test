/*********************************************************************
* @file    camera_wrapper.cpp
* @date    03/05/2021
*
* @attention Copyright (c) 2021
* @attention Phantom AI, Inc.
* @attention All rights reserved.
*********************************************************************/

#include <unistd.h>

#include <opencv2/opencv.hpp>

#include "phantom_ai/core/log.h"
#include "phantom_ai/core/yaml.h"
#include "phantom_ai/core/time_stamp.h"
#include "phantom_ai/core/exception.h"
#include "phantom_ai/core/paths.h"
#include "phantom_ai/geometry/angle.h"
#if defined(PLATFORM_AM62A)
#include "hal_camera.h"
#else
#include "tda4x_camera.h"
#endif
#include "phantom_ai/wrappers/camera_wrapper.h"

#include <phantom_ai/test/test_log.hpp>

namespace phantom_ai
{
  CameraWrapper::CameraWrapper()
  :
    rx_frame_callback_(nullptr),
    cam_info_list_(),
    hw_cam_(nullptr),
    last_ts_(0.0),
    last_print_(0.0)
  {
  }

  void CameraWrapper::onInit(std::string filename, const CameraModelListS& camera_models, std::string target_system, bool enable_fullres_logging, bool cam_app)
  {
    PHANTOM_LOG("[CameraWrapper] has initialized.");

    camera_models_ = camera_models;
    // #PROC if the given CameraModelList is valid, read the image crop information from CameraModelList
    if (camera_models)
    {
      // #COND Read the image crop information from CameraModelList
      for (int cam = 0; cam < NUM_MAX_CAM_IDS; ++cam)
      {
        CameraModelS cam_model = camera_models->camera_model(cam);
        if (cam_model)
        {
          // WA: currently, only the front center, front left, and front right are virtual images (wide image)
          // I wanted to keep only front center, so used camera name like this.

          // #COND Check if camera name matches
          if (camera_name(cam).find(camera_name(cam_model->Parent())) == 0)
          {
            auto cam_info = cam_model->cameraInfo();
            // #COND If full resolution logging and camera application are enabled
            if (enable_fullres_logging && cam_app) {
              cam_info.resolution_.height = IMAGE_HEIGHT_2MP_FULL_RES;
            }
            cam_info_list_.insert(cam_model->Parent(),
                                  cam_info);
          }
        }
      }
      // #COND Fix the camera name strings with postfix, "_virtual", "_tda4x"
      for (auto& crops: cam_info_list_)
      {
        for (auto& it: crops)
        {
          auto& crop_info = it.second;
          std::string location = crop_info.name_;
          // #COND If crop info is virtual
          if (crop_info.is_virtual_)
          {
            std::string virtual_keyword = "_virtual";
            location = location.substr(0, location.size() - virtual_keyword.size());
          }
          std::string target_system_keyword = "_tda4x";
          // #COND If camera name contains target system keyword
          if (crop_info.name_.find(target_system_keyword) != std::string::npos)
          {
            location = location.substr(0, location.size() - target_system_keyword.size());
          }
          crop_info.name_ = location;
        }
      }
    }
  #if defined(PLATFORM_AM62A)
    std::vector<camera_params_hal> cam_params_arr;
    std::vector<int> annotation_image_widths;
    readHalCamerasParameters(filename, cam_params_arr, cam_info_list_, annotation_image_widths, enable_fullres_logging, true, target_system, cam_app);
    hw_cam_ = new phantom_ai::HalCamera(cam_params_arr, enable_fullres_logging, cam_app);
  #else
    std::vector<camera_params_tda4x> cam_params_arr;
    std::vector<int> annotation_image_widths;
    // #PROC Read TDA4X camera parameters
    readTda4xCamerasParameters(filename, cam_params_arr, cam_info_list_, annotation_image_widths, enable_fullres_logging, target_system);
    // #END
    hw_cam_ = new phantom_ai::Tda4xCamera(cam_params_arr, enable_fullres_logging);
  #endif

    // #COND If hardware camera is initialized
    if (hw_cam_ != nullptr)
    {
      hw_cam_->registerRxCallback(
        [this](std::vector<phantom_ai::CameraResults> &image_arr, const std::unordered_map<phantom_ai::CameraID, std::shared_ptr<phantom_ai::CameraExtraInfo>>& extra_info) {
          this->imageCallback(image_arr, extra_info);
        });
    }

    // #PROC Get current timestamp
    last_ts_ = phantom_ai::TimeStamp::Now();
    // #END

    // #PROC Initialize last print timestamp
    last_print_ = last_ts_;
    // #END
  }


  CameraWrapper::~CameraWrapper()
  {
    PHANTOM_LOG("[CameraWrapper] has terminated.");

    // #COND If hardware camera is initialized
    if (hw_cam_ != nullptr)
    {
      // #PROC Stop streaming
      hw_cam_->stopStreaming(); 
      // #END

      // #PROC Clean up camera object
      delete hw_cam_;
      // #END

      // #PROC Set pointer to nullptr
      hw_cam_ = nullptr;
      // #END
    }
  }

  void CameraWrapper::startStreaming()
  {
    // #COND If hardware camera is initialized
    if (hw_cam_ != nullptr) hw_cam_->startStreaming();
  }

  void CameraWrapper::stopStreaming()
  {
    // #COND If hardware camera is initialized
    if (hw_cam_ != nullptr) hw_cam_->stopStreaming();
  }

  void CameraWrapper::registerImageCallback(std::function<void(std::vector<phantom_ai::CameraResults>&, const std::unordered_map<phantom_ai::CameraID, std::shared_ptr<phantom_ai::CameraExtraInfo>>&)> callback_func)
  {
    // #PROC Register image callback function
    rx_frame_callback_ = callback_func;
    // #END
  }

  void CameraWrapper::checkCameraLocation(uint8_t wrong_camera_connected[], uint8_t num_of_cam)
  {
    // #COND If hardware camera is initialized
    if (hw_cam_ != nullptr) hw_cam_->checkCameraLocation(wrong_camera_connected, num_of_cam);
  }

  bool CameraWrapper::isCameraConnected(phantom_ai::CameraResults cam_result)
  {
    // #COND If hardware camera is initialized
    if (hw_cam_ != nullptr) return hw_cam_->isCameraConnected(cam_result);

    // #PROC Return false if not initialized
    return false;
    // #END
  }

  void CameraWrapper::imageCallback(std::vector<phantom_ai::CameraResults>& image_arr,
                       const std::unordered_map<phantom_ai::CameraID, std::shared_ptr<phantom_ai::CameraExtraInfo>>& extra_info)
  {
    // #IGNORE Test logging
    TESTLOG(logCameraInputSeq("CameraWrapper::imageCallback"));
    // #END

    // #COND If callback function is registered
    if (rx_frame_callback_ != nullptr)
    {
      // #PROC Invoke callback with received images
      rx_frame_callback_(image_arr, extra_info);
      // #END
    }
  }

#if defined(ENABLE_GSTREAMER)
  void CameraWrapper::getMosaicDimensions(uint32_t& width, uint32_t& height)
  {
    if (hw_cam_)
    {
      hw_cam_->getMosaicDimensions(width, height);
    }
  }

  void CameraWrapper::registerEncoderCallback(EncoderFeedFrameCallback callback)
  {
    if (hw_cam_)
    {
      hw_cam_->registerEncoderCallback(callback);
    }
  }
#endif

  void CameraWrapper::setTimestampOffset(double)
  {
    // #IGNORE Currently not implemented
    // #END
  }

  CameraInfoList& CameraWrapper::getCameraInfoList()
  {
    // #PROC Return camera info list
    return cam_info_list_;
    // #END
  }
  std::optional<FrameData> CameraWrapper::getFullResolutionImageROI(CameraID cam_id, const cv::Rect &cam_crop_roi, uint32_t frame_number, const cv::Rect& resized_target_roi, PixelFormat fmt)
  {
#if defined(PLATFORM_AM62A)
    if (hw_cam_) {
        return hw_cam_->getFullResolutionFrameROI(cam_id, cam_crop_roi, frame_number, resized_target_roi, fmt);
    } else {
        PHANTOM_ERROR("hw_cam_ is not initialized.");
        return std::nullopt;
    }
#elif defined(SOC_J722S) || defined(SOC_J721S2) || defined(SOC_J721E)
    UNUSED(cam_crop_roi);
    if (hw_cam_) {
        return hw_cam_->getFullResolutionFrameROI(cam_id, frame_number, resized_target_roi, fmt, camera_models_);
    } else {
        PHANTOM_ERROR("hw_cam_ is not initialized.");
        return std::nullopt;
    }
#else
    PHANTOM_WARNING("getFullResolutionFrameROI() is not supported on this platform.");
    UNUSED(cam_id);
    UNUSED(cam_crop_roi);
    UNUSED(frame_number);
    UNUSED(resized_target_roi);
    UNUSED(fmt);
    return std::nullopt;
#endif
  }
  
  std::vector<FrameData> CameraWrapper::getFullResolutionImages(double base_timestamp, uint8_t num_frames, PixelFormat fmt)
  {
#if defined(PLATFORM_AM62A)
    return hw_cam_->getFullResolutionFrames(base_timestamp, num_frames, fmt);
#else
    PHANTOM_WARNING("getFullResolutionImages() is not supported on this platform.");
    UNUSED(base_timestamp);
    UNUSED(num_frames);
    UNUSED(fmt);
    return std::vector<FrameData>();
#endif
  }

  bool CameraWrapper::dynamicCropSetPosition(CameraID cam_id, uint32_t x, uint32_t y)
  {
  #if defined(PLATFORM_AM62A)
      if (hw_cam_) {
          return hw_cam_->dynamicCropSetPosition(cam_id, x, y);
      } else {
          PHANTOM_ERROR("hw_cam_ is not initialized.");
          return false;
      }
  #else
      PHANTOM_WARNING("dynamicCropSetPosition() is not supported on this platform.");
      UNUSED(cam_id);
      UNUSED(x);
      UNUSED(y);
      return false;
  #endif
  }

  bool CameraWrapper::dynamicCropGetPosition(CameraID cam_id, uint32_t& x, uint32_t& y)
  {
  #if defined(PLATFORM_AM62A)
      if (hw_cam_) {
          return hw_cam_->dynamicCropGetPosition(cam_id, x, y);
      } else {
          PHANTOM_ERROR("hw_cam_ is not initialized.");
          return false;
      }
  #else
      PHANTOM_WARNING("dynamicCropGetPosition() is not supported on this platform.");
      UNUSED(cam_id);
      UNUSED(x);
      UNUSED(y);
      return false;
  #endif
  }

  bool CameraWrapper::dynamicCropMoveRelative(CameraID cam_id, int32_t dx, int32_t dy)
  {
  #if defined(PLATFORM_AM62A)
      if (hw_cam_) {
          return hw_cam_->dynamicCropMoveRelative(cam_id, dx, dy);
      } else {
          PHANTOM_ERROR("hw_cam_ is not initialized.");
          return false;
      }
  #else
      PHANTOM_WARNING("dynamicCropMoveRelative() is not supported on this platform.");
      UNUSED(cam_id);
      UNUSED(dx);
      UNUSED(dy);
      return false;
  #endif
  }

#if defined(PLATFORM_AM62A)
  bool CameraWrapper::dynamicCropGetInfo(CameraID cam_id, bool& enabled, hal_camera_image& cam_img_info)
  {
  #if defined(PLATFORM_AM62A)
      if (hw_cam_) {
          return hw_cam_->dynamicCropGetInfo(cam_id, enabled, cam_img_info);
      } else {
          PHANTOM_ERROR("hw_cam_ is not initialized.");
          return false;
      }
  #else
      PHANTOM_WARNING("dynamicCropGetInfo() is not supported on this platform.");
      UNUSED(cam_id);
      UNUSED(enabled);
      UNUSED(cam_img_info);
      return false;
  #endif
  }
#endif

  bool CameraWrapper::dynamicCropGetValidRange(CameraID cam_id, cv::Rect& valid_range)
  {
  #if defined(PLATFORM_AM62A)
      if (hw_cam_) {
          return hw_cam_->dynamicCropGetValidRange(cam_id, valid_range);
      } else {
          PHANTOM_ERROR("hw_cam_ is not initialized.");
          return false;
      }
  #else
      PHANTOM_WARNING("dynamicCropGetValidRange() is not supported on this platform.");
      UNUSED(cam_id);
      UNUSED(valid_range);
      return false;
  #endif
  }
}