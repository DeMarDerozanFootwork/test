/*******************************************************************************
* @file    embedded_capture_nodelet.cpp
* @date    10/30/2019
*
* @attention Copyright (c) 2019
* @attention Phantom AI, Inc.
* @attention All rights reserved.
*******************************************************************************/

/* ROS Libraries */
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include "phantom_ros/phantom_ros.h"
#include <phantom_ros/phantom_ros_nodelet.h>

#include "phantom_ai/core/log.h"
#include "phantom_ai/core/time_stamp.h"
#include "phantom_ai/core/yaml.h"
#include "phantom_ai/core/process_health_monitor.h"

#include "phantom_ai/embedded_network/embedded_phantom_net.h"
#include "phantom_ai/embedded_network/embedded_network_if.h"

#include <functional>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>

namespace phantom_ros
{

static constexpr uint32_t max_can_chans = 8;
static constexpr float kVizWindowScaleFactorMax = 1.0f;
static constexpr int BOARD_TIME_PORT = 9998;
static constexpr int ANNOTATION_LARGE_HEIGHT = 1080;
static constexpr int ANNOTATION_SMALL_HEIGHT = 540;

  typedef struct
  {
      cv::Mat image;
      ros::Publisher vision_pub;
      ros::Publisher annotation_pub;
      ros::Publisher vision_cam_info_pub;
      ros::Publisher annotation_cam_info_pub;
  } EmbeddedImageObj;

  typedef enum
  {
    format_yuv420sp,
    format_yuv422,
    format_rgb,
    format_bgr
  } embedded_capture_format;

  class embeddedCaptureNodelet : public PhantomRosNodelet
  {
  public:
    ~embeddedCaptureNodelet() {
      is_running_ = false;
      if (data_cap_thread_.joinable())
      {
        data_cap_thread_.join();
      }

      if (frame_.data != nullptr)
      {
        delete [] frame_.data;
      }

      free(const_cast<char*>(board_ip));
    }

  private:

    cv::ColorConversionCodes openCvConversion(embedded_capture_format input, std::string output)
    {
      // So many formats: https://www.fourcc.org/yuv.php
      if (input == format_yuv420sp)
      {
        if (output == sensor_msgs::image_encodings::BGR8)
        {
          // For some reason raw video is NV12 not NV21
          return cv::COLOR_YUV2BGR_NV12;
        }
        else if (output == sensor_msgs::image_encodings::RGB8)
        {
          // For some reason raw video is NV12 not NV21
          return cv::COLOR_YUV2RGB_NV12;
        }
      }
      else if (input == format_yuv422)
      {
        if (output == sensor_msgs::image_encodings::BGR8)
        {
          return cv::COLOR_YUV2BGR_YUYV; //YUYV
        }
        else if (output == sensor_msgs::image_encodings::RGB8)
        {
          return cv::COLOR_YUV2RGB_YUYV; //YUYV
        }
      }
      else if (input == format_rgb)
      {
        if (output == sensor_msgs::image_encodings::BGR8)
        {
          return cv::COLOR_RGB2BGR;
        }
        else if (output == sensor_msgs::image_encodings::RGB8)
        {
          return cv::COLOR_COLORCVT_MAX; // special case, no conversion!
        }
      }
      else if (input == format_bgr)
      {
        if (output == sensor_msgs::image_encodings::BGR8)
        {
          return cv::COLOR_COLORCVT_MAX; // special case, no conversion!
        }
        else if (output == sensor_msgs::image_encodings::RGB8)
        {
          return cv::COLOR_RGB2BGR; // operation is symetic, BGR->RGB or RGB->BGR
        }
      }
      throw phantom_ai::PhantomAIException("Unsupported input format {} with output format {}", input, output);
    }

    void cropAndResize(cv::Mat &input, cv::Mat &output)
    {
      cv::Size input_size = input.size();
      cv::Mat cropped;
      cv::Size output_size = output_cam_image_size_;

      if (enable_cropping_)
      {
        // adjust height based on format difference
        cropped = input(crop_win_);
        input_size = cv::Size(crop_win_.width, crop_win_.height);
      }
      else
      {
        cropped = input;
      }

      if (output_cam_image_size_.width == -1)
      {
        // no width scaling
        output_size.width = input_size.width;
      }
      if (output_cam_image_size_.height == -1)
      {
        // no height scaling
        output_size.height = input_size.height;
      }

      // scale image to match output
      if (input_size != output_size)
      {
        // scale image to match output
        cv::resize(cropped, output, output_size);
      }
      else
      {
        // just copy ptrs
        output = cropped;
      }
    }

    void updateStats()
    {
      frame_cnt_++;
      phantom_ai::TimeStamp now = phantom_ai::TimeStamp::Now();

      double delta_time = now.toSec() - last_time_.toSec();

      if (delta_time >= stat_print_interval_sec_)
      {
        double fps = (double)frame_cnt_ / delta_time;
        PHANTOM_LOG("Rx fps: {:.2f}, frames: {}, delta: {:.3f} s", fps, frame_cnt_, delta_time);
        // reset counters
        last_time_ = now;
        frame_cnt_ = 0;
      }
    }

    std::shared_ptr<EmbeddedImageObj> getImageObj(const uint32_t cam)
    {
      if (cam < phantom_ai::CameraID::CAM_GROUND)
      {
        return getImageObj(static_cast<phantom_ai::CameraID>(cam));
      }
      throw phantom_ai::PhantomAIException("Invalid CameraID {} is given", cam);
      return nullptr;
    }

    std::shared_ptr<EmbeddedImageObj> getImageObj(const phantom_ai::CameraID& cam)
    {
      if (embedded_img_objs_[cam] == nullptr)
      {
        embedded_img_objs_[cam] = std::make_shared<EmbeddedImageObj>();
        std::string topic_name = topic_name_prefix_ + "/" + phantom_ai::camera_name(cam);
        embedded_img_objs_[cam]->vision_pub = node_.advertise<sensor_msgs::Image>(topic_name + "/image_raw", 1);
        if (cam != phantom_ai::CAM_FRONT_CENTER_WHOLE) {
          embedded_img_objs_[cam]->annotation_pub = node_.advertise<sensor_msgs::Image>("/annot" + topic_name + "/image_raw", 1);
          embedded_img_objs_[cam]->annotation_cam_info_pub = node_.advertise<phantom_ros::PhantomCameraInfo>("/annot" + topic_name + "/camera_info", 1);
        }
        embedded_img_objs_[cam]->vision_cam_info_pub = node_.advertise<phantom_ros::PhantomCameraInfo>(topic_name + "/camera_info", 1);
      }
      return embedded_img_objs_[cam];
    }

#if USE_EXTRA_INPUT_HISTOGRAM
    std::shared_ptr<EmbeddedImageObj> getHistogramImageObj(const phantom_ai::CameraID& cam)
    {
      if (camera_extra_info_histogram_objs_[cam] == nullptr)
      {
        camera_extra_info_histogram_objs_[cam] = std::make_shared<EmbeddedImageObj>();
        std::string topic_name = topic_name_prefix_ + "/" + phantom_ai::camera_name(cam);
        camera_extra_info_histogram_objs_[cam]->vision_pub = node_.advertise<sensor_msgs::Image>(topic_name + "/histogram", 1);
      }
      return camera_extra_info_histogram_objs_[cam];
    }
#endif

    std::shared_ptr<EmbeddedImageObj> getH3aImageObj(const phantom_ai::CameraID& cam)
    {
      if (camera_extra_info_h3a_objs_[cam] == nullptr)
      {
        camera_extra_info_h3a_objs_[cam] = std::make_shared<EmbeddedImageObj>();
        std::string topic_name = topic_name_prefix_ + "/" + phantom_ai::camera_name(cam);
        camera_extra_info_h3a_objs_[cam]->vision_pub = node_.advertise<sensor_msgs::Image>(topic_name + "/H3a", 1);
      }
      return camera_extra_info_h3a_objs_[cam];
    }

    std::shared_ptr<EmbeddedImageObj> getRawImageObj(const phantom_ai::CameraID& cam)
    {
      if (camera_extra_info_raw_objs_[cam] == nullptr)
      {
        camera_extra_info_raw_objs_[cam] = std::make_shared<EmbeddedImageObj>();
        std::string topic_name = topic_name_prefix_ + "/" + phantom_ai::camera_name(cam);
        camera_extra_info_raw_objs_[cam]->vision_pub = node_.advertise<sensor_msgs::Image>(topic_name + "/raw16bit", 1);
      }
      return camera_extra_info_raw_objs_[cam];
    }

    int32_t getCanIndex(EmbeddedNet_PhantomFormat type, uint32_t chan)
    {
      // map type {TX,RX} and channel number to index for can_pub_[].
      // can_pub_ is ordered
      // [0] /can/can0/rx
      // [1] /can/can0/tx
      // [2] /can/can1/rx
      // [3] /can/can1/tx
      // ...
      // [7] /can/can3/tx
      const int CHAN_INTERVAL = 2;
      int32_t idx = chan * CHAN_INTERVAL;

      if (type == CAN_TX)
      {
        idx++;// increment for TX
      }

      return idx;
    }

    void sendImage(std::shared_ptr<EmbeddedImageObj> imageObj, std::string &format, ros::Time &ts, uint32_t frame_count = 0)
    {
      boost::shared_ptr<sensor_msgs::Image> msg(new sensor_msgs::Image);

      std_msgs::Header header;
      header.stamp = ts;
      header.frame_id = std::to_string(frame_count);

      cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, format, imageObj->image);

      img_bridge.toImageMsg(*msg);

      if(is_annotated_) {
        imageObj->annotation_pub.publish(msg);
      } else {
        imageObj->vision_pub.publish(msg);
      }
    }

    void generateImage(cv::Mat &input, std::string &format, ros::Time &ts, const uint32_t& frame_count, const uint32_t cam_id){
      if(generate_images_){
        std::string vehicle_name = phantom_ai::phantom_paths_vehicle_name();
        std::string config_file = vehicle_name + "/camera_calibration.yaml";

        phantom_ai::YamlNode cam_settings = phantom_ai::load_yaml_file("sensors/camera", config_file);
        phantom_ai::YamlNode camera_list = cam_settings[vehicle_name];

        std::string cam_fullname = "front_center_tda4x"; // hard code here
        phantom_ai::YamlNode cam_cfg = camera_list[cam_fullname];
        if (cam_cfg)
        {
          phantom_ai::YamlNode vroi_configs = phantom_ai::get_yaml_key<phantom_ai::YamlNode>(cam_cfg, "vision_mode", phantom_ai::YamlNode());
          cv::Mat cropped;
          for(auto vroi_config : vroi_configs){
            std::string crop_name = vroi_config.first.as<std::string>();
            auto v_crop_roi = vroi_configs[crop_name].as<std::vector<std::vector<int>>>();

            cv::Rect vision_roi  = cv::Rect(v_crop_roi[0][0], v_crop_roi[0][1], v_crop_roi[0][2], v_crop_roi[0][3]);
            cv::Size vision_size = cv::Size(v_crop_roi[1][0], v_crop_roi[1][1]);

            std::shared_ptr<EmbeddedImageObj> cam = getImageObj(phantom_ai::camera_name(crop_name));
            cropped = input(vision_roi);
            cv::resize(cropped, cam->image, vision_size);
            sendImage(cam, format, ts, frame_count);
          }
        }
      }
      else{
        std::shared_ptr<EmbeddedImageObj> cam = getImageObj(cam_id);
        cropAndResize(input, cam->image);
        // publish frame
        sendImage(cam, format, ts, frame_count);
      }
    }

    void parseLaneResult(const EmbeddedNet_PhantomVisionResult *input, boost::shared_ptr<phantom_ros::PhantomVisionMeasurement> output)
    {
      phantom_ai::PhantomVisionLane lane;

      for (uint32_t i = 0; i < input->lane_count; i++)
      {
        lane.id_ = input->lanes[i].id;
        lane.valid_ = input->lanes[i].valid;
        lane.measured_ = input->lanes[i].measured;
        lane.mark_type_ = static_cast<phantom_ai::PhantomVisionLaneMarkType>(input->lanes[i].mark_type);
        lane.color_ = static_cast<phantom_ai::PhantomVisionLaneColor>(input->lanes[i].color);
        lane.quality_ = static_cast<phantom_ai::PhantomVisionLaneMarkQuality>(input->lanes[i].quality);
        lane.location_ = static_cast<phantom_ai::PhantomVisionLaneLocation>(input->lanes[i].location);
        lane.c0_ = input->lanes[i].c0;
        lane.c1_ = input->lanes[i].c1;
        lane.c2_ = input->lanes[i].c2;
        lane.c3_ = input->lanes[i].c3;
        lane.ego_lane_width_m_ = input->lanes[i].width_m;
        lane.view_range_start_m_ = input->lanes[i].view_range_start_m;
        lane.view_range_end_m_ = input->lanes[i].view_range_end_m;
        lane.age_secs_ = input->lanes[i].age_secs;
        lane.pitch_rads_ = input->lanes[i].pitch_rads;
        lane.camera_name_ = std::string(input->lanes[i].camera_name);
        lane.camera_id_ = input->lanes[i].camera_id;

        lane.x_px_.resize(input->lanes[i].camera_px_count);
        lane.y_px_.resize(input->lanes[i].camera_px_count);
        for (uint32_t j = 0; j < input->lanes[i].camera_px_count; j++)
        {
          lane.x_px_[j] = input->lanes[i].x_px[j];
          lane.y_px_[j] = input->lanes[i].y_px[j];
        }

        output->lanes.push_back(convert_phantomvision_lane_msg_to_ros(lane));
      }
    }

    void parseObjectResult(const EmbeddedNet_PhantomVisionResult *input, boost::shared_ptr<phantom_ros::PhantomVisionMeasurement> output)
    {
      phantom_ai::PhantomVisionObject obj;

      for (uint32_t i = 0; i < input->object_count; i++)
      {
        obj.id_ = input->objects[i].id;
        obj.valid_ = input->objects[i].valid;
        obj.measured_ = input->objects[i].measured;
        obj.classification_ = static_cast<phantom_ai::PhantomVisionObjectClassification>(input->objects[i].classification);
        obj.view_face_ = static_cast<phantom_ai::PhantomVisionObjectViewFace>(input->objects[i].view_face);
        obj.lane_assignment_ = static_cast<phantom_ai::PhantomVisionObjectLaneAssignment>(input->objects[i].lane_assignment);

        obj.brake_light_ = static_cast<phantom_ai::PhantomVisionObjectBrakeLight>(input->objects[i].brake_light);
        obj.flash_light_ = static_cast<phantom_ai::PhantomVisionObjectFlashLight>(input->objects[i].flash_light);
        obj.turn_signal_ = static_cast<phantom_ai::PhantomVisionObjectTurnSignal>(input->objects[i].turn_signal);
        obj.confidence_ = static_cast<phantom_ai::PhantomVisionObjectConfidence>(input->objects[i].confidence);

        obj.classification_candidates_.resize(input->objects[i].classification_count);
        obj.classification_confidences_.resize(input->objects[i].classification_count);
        for (uint32_t j = 0; j < input->objects[i].classification_count; j++)
        {
          obj.classification_candidates_[j] = static_cast<phantom_ai::PhantomVisionObjectClassification>(input->objects[i].classification_candidates[j]);
          obj.classification_confidences_[j] = input->objects[i].classification_confidences[j];
        }
        obj.position_x_m_ = input->objects[i].position_x_m;
        obj.position_y_m_ = input->objects[i].position_y_m;
        obj.position_z_m_ = input->objects[i].position_z_m;
        obj.rel_velocity_x_mps_ = input->objects[i].rel_velocity_x_mps;
        obj.rel_velocity_y_mps_ = input->objects[i].rel_velocity_y_mps;
        obj.rel_accel_x_mps2_ = input->objects[i].rel_accel_x_mps2;
        obj.rel_accel_y_mps2_ = input->objects[i].rel_accel_y_mps2;
        obj.range_velocity_mps_ = input->objects[i].range_velocity_mps;
        obj.range_m_ = input->objects[i].range_m;
        obj.angle_rads_ = input->objects[i].angle_rads;
        obj.width_m_ = input->objects[i].width_m;
        obj.length_m_ = input->objects[i].length_m;
        obj.height_m_ = input->objects[i].height_m;
        obj.orientation_rads_ = input->objects[i].orientation_rads;
        obj.total_left_bounding_angle_valid_ = input->objects[i].total_left_bounding_angle_valid;
        obj.total_right_bounding_angle_valid_ = input->objects[i].total_right_bounding_angle_valid;
        obj.total_left_bounding_angle_rads_ = input->objects[i].total_left_bounding_angle_rads;
        obj.total_right_bounding_angle_rads_ = input->objects[i].total_right_bounding_angle_rads;
        obj.face_left_bounding_angle_valid_ = input->objects[i].face_left_bounding_angle_valid;
        obj.face_right_bounding_angle_valid_ = input->objects[i].face_right_bounding_angle_valid;
        obj.face_left_bounding_angle_rads_ = input->objects[i].face_left_bounding_angle_rads;
        obj.face_right_bounding_angle_rads_ = input->objects[i].face_right_bounding_angle_rads;
        obj.age_secs_ = input->objects[i].age_secs;
        obj.time_to_collision_secs_ = input->objects[i].time_to_collision_secs;
        obj.cipv_ = input->objects[i].cipv;
        obj.oncoming_traffic_ = input->objects[i].oncoming_traffic;
        obj.cross_traffic_ = input->objects[i].cross_traffic;
        obj.extra_values_.resize(input->objects[i].extra_values_count);
        for (uint8_t j = 0; j < input->objects[i].extra_values_count; j++)
        {
          obj.extra_values_[j] = input->objects[i].extra_values[j];
        }
        obj.extra_infos_.resize(input->objects[i].extra_infos_count);
        for (uint8_t j = 0; j < input->objects[i].extra_infos_count; j++)
        {
          obj.extra_infos_[j] = std::string(input->objects[i].extra_infos[j]);
        }
        obj.camera_name_ = std::string(input->objects[i].camera_name);
        obj.camera_id_ = input->objects[i].camera_id;
        obj.x_px_ = input->objects[i].x_px;
        obj.y_px_ = input->objects[i].y_px;
        obj.width_px_ = input->objects[i].width_px;
        obj.height_px_ = input->objects[i].height_px;

        output->objects.push_back(convert_phantomvision_object_msg_to_ros(obj));
      }
    }

    void parseTrafficSignResult(const EmbeddedNet_PhantomVisionResult *input, boost::shared_ptr<phantom_ros::PhantomVisionMeasurement> output)
    {
      phantom_ai::PhantomVisionTrafficSign sign;

      for (uint32_t i = 0; i < input->traffic_count; i++)
      {
        sign.id_ = input->traffic_signs[i].id;
        sign.valid_ = input->traffic_signs[i].valid;
        sign.measured_ = input->traffic_signs[i].measured;
        sign.is_this_for_highway_ = input->traffic_signs[i].is_this_for_highway;
        sign.highway_mode_ = input->traffic_signs[i].highway_mode;
        sign.classification_ = static_cast<phantom_ai::PhantomVisionObjectClassification>(input->traffic_signs[i].classification);
        sign.lane_assignment_ = static_cast<phantom_ai::PhantomVisionObjectLaneAssignment>(input->traffic_signs[i].lane_assignment);

        sign.confidence_ = static_cast<phantom_ai::PhantomVisionObjectConfidence>(input->traffic_signs[i].confidence);
        sign.confidence_value_ = input->traffic_signs[i].confidence_value;
        sign.filter_type_ = static_cast<phantom_ai::PhantomVisionTrafficSignFilterType>(input->traffic_signs[i].filter_type);


        sign.position_x_m_ = input->traffic_signs[i].position_x_m;
        sign.position_y_m_ = input->traffic_signs[i].position_y_m;
        sign.position_z_m_ = input->traffic_signs[i].position_z_m;

        sign.width_m_ = input->traffic_signs[i].width_m;
        sign.length_m_ = input->traffic_signs[i].length_m;
        sign.height_m_ = input->traffic_signs[i].height_m;

        sign.age_secs_ = input->traffic_signs[i].age_secs;

        for (uint8_t j = 0; j < input->traffic_signs[i].extra_values_count; j++)
        {
          sign.extra_values_[j] = input->traffic_signs[i].extra_values[j];
        }
        sign.extra_infos_.resize(input->traffic_signs[i].extra_infos_count);
        for (uint8_t j = 0; j < input->traffic_signs[i].extra_infos_count; j++)
        {
          sign.extra_infos_[j] = std::string(input->traffic_signs[i].extra_infos[j]);
        }
        sign.camera_name_ = std::string(input->traffic_signs[i].camera_name);
        sign.camera_id_ = input->traffic_signs[i].camera_id;
        sign.x_px_ = input->traffic_signs[i].x_px;
        sign.y_px_ = input->traffic_signs[i].y_px;
        sign.width_px_ = input->traffic_signs[i].width_px;
        sign.height_px_ = input->traffic_signs[i].height_px;

        output->traffic_signs.push_back(convert_phantomvision_traffic_msg_to_ros(sign));
      }
    }

    void parseAEBResult(const EmbeddedNet_PhantomVisionAEB *input, boost::shared_ptr<phantom_ros::PhantomVisionAEB> output)
    {
      phantom_ai::PhantomVisionObject obj;

      for (uint32_t i = 0; i < input->object_count; i++)
      {
        obj.id_ = input->objects[i].id;
        obj.valid_ = input->objects[i].valid;
        obj.measured_ = input->objects[i].measured;
        obj.classification_ = static_cast<phantom_ai::PhantomVisionObjectClassification>(input->objects[i].classification);
        obj.view_face_ = static_cast<phantom_ai::PhantomVisionObjectViewFace>(input->objects[i].view_face);
        obj.lane_assignment_ = static_cast<phantom_ai::PhantomVisionObjectLaneAssignment>(input->objects[i].lane_assignment);

        obj.brake_light_ = static_cast<phantom_ai::PhantomVisionObjectBrakeLight>(input->objects[i].brake_light);
        obj.flash_light_ = static_cast<phantom_ai::PhantomVisionObjectFlashLight>(input->objects[i].flash_light);
        obj.turn_signal_ = static_cast<phantom_ai::PhantomVisionObjectTurnSignal>(input->objects[i].turn_signal);
        obj.confidence_  = static_cast<phantom_ai::PhantomVisionObjectConfidence>(input->objects[i].confidence);

        obj.classification_candidates_.resize(input->objects[i].classification_count);
        obj.classification_confidences_.resize(input->objects[i].classification_count);
        for (uint32_t j = 0; j < input->objects[i].classification_count; j++)
        {
          obj.classification_candidates_[j] = static_cast<phantom_ai::PhantomVisionObjectClassification>(input->objects[i].classification_candidates[j]);
          obj.classification_confidences_[j] = input->objects[i].classification_confidences[j];
        }
        obj.position_x_m_       = input->objects[i].position_x_m;
        obj.position_y_m_       = input->objects[i].position_y_m;
        obj.position_z_m_       = input->objects[i].position_z_m;
        obj.rel_velocity_x_mps_ = input->objects[i].rel_velocity_x_mps;
        obj.rel_velocity_y_mps_ = input->objects[i].rel_velocity_y_mps;
        obj.rel_accel_x_mps2_   = input->objects[i].rel_accel_x_mps2;
        obj.rel_accel_y_mps2_   = input->objects[i].rel_accel_y_mps2;
        obj.range_velocity_mps_ = input->objects[i].range_velocity_mps;

        obj.range_m_          = input->objects[i].range_m;
        obj.angle_rads_       = input->objects[i].angle_rads;
        obj.width_m_          = input->objects[i].width_m;
        obj.length_m_         = input->objects[i].length_m;
        obj.height_m_         = input->objects[i].height_m;
        obj.orientation_rads_ = input->objects[i].orientation_rads;

        obj.total_left_bounding_angle_valid_  = input->objects[i].total_left_bounding_angle_valid;
        obj.total_right_bounding_angle_valid_ = input->objects[i].total_right_bounding_angle_valid;
        obj.total_left_bounding_angle_rads_   = input->objects[i].total_left_bounding_angle_rads;
        obj.total_right_bounding_angle_rads_  = input->objects[i].total_right_bounding_angle_rads;
        obj.face_left_bounding_angle_valid_   = input->objects[i].face_left_bounding_angle_valid;
        obj.face_right_bounding_angle_valid_  = input->objects[i].face_right_bounding_angle_valid;
        obj.face_left_bounding_angle_rads_    = input->objects[i].face_left_bounding_angle_rads;
        obj.face_right_bounding_angle_rads_   = input->objects[i].face_right_bounding_angle_rads;

        obj.age_secs_               = input->objects[i].age_secs;
        obj.time_to_collision_secs_ = input->objects[i].time_to_collision_secs;
        obj.cipv_                   = input->objects[i].cipv;
        obj.oncoming_traffic_       = input->objects[i].oncoming_traffic;
        obj.cross_traffic_          = input->objects[i].cross_traffic;

        obj.extra_values_.resize(input->objects[i].extra_values_count);
        for (uint8_t j = 0; j < input->objects[i].extra_values_count; j++)
        {
          obj.extra_values_[j] = input->objects[i].extra_values[j];
        }
        
        obj.extra_infos_.resize(input->objects[i].extra_infos_count);
        for (uint8_t j = 0; j < input->objects[i].extra_infos_count; j++)
        {
          obj.extra_infos_[j] = std::string(input->objects[i].extra_infos[j]);
        }
        obj.camera_name_ = std::string(input->objects[i].camera_name);
        obj.camera_id_   = input->objects[i].camera_id;
        obj.x_px_        = input->objects[i].x_px;
        obj.y_px_        = input->objects[i].y_px;
        obj.width_px_    = input->objects[i].width_px;
        obj.height_px_   = input->objects[i].height_px;

        output->objects.push_back(convert_phantomvision_object_msg_to_ros(obj));
      }

      phantom_ai::PhantomVisionAEBFlag flag;

      for (uint32_t i = 0; i < input->flag_count; i++)
      {
        output->vehicle_flags[i].activated = input->veh_flags[i].activated;
        output->vehicle_flags[i].name      = std::string(input->veh_flags[i].name);
        output->vehicle_flags[i].type      = input->veh_flags[i].type;
        output->vehicle_flags[i].object_id = input->veh_flags[i].object_id;

        output->vru_flags[i].activated = input->vru_flags[i].activated;
        output->vru_flags[i].name      = std::string(input->vru_flags[i].name);
        output->vru_flags[i].type      = input->vru_flags[i].type;
        output->vru_flags[i].object_id = input->vru_flags[i].object_id;
      }
    }

    void parseFreeSpaceResult(const EmbeddedNet_PhantomVisionResult *input, boost::shared_ptr<phantom_ros::PhantomVisionMeasurement> output)
    {
      output->free_spaces.resize(input->space_count);

      for (uint32_t i = 0; i < input->space_count; i++)
      {
        output->free_spaces[i].valid = input->free_spaces[i].valid;
        output->free_spaces[i].points.resize(input->free_spaces[i].count);
        for (uint32_t j = 0; j < input->free_spaces[i].count; j++)
        {
          output->free_spaces[i].points[j].x = input->free_spaces[i].points[j].x;
          output->free_spaces[i].points[j].y = input->free_spaces[i].points[j].y;
          output->free_spaces[i].points[j].z = 0;
        }
      }
    }

    void parseStatusResult(const EmbeddedNet_PhantomVisionResult *input, boost::shared_ptr<phantom_ros::PhantomVisionMeasurement> output)
    {
      output->status.resize(input->status_count);

      for (uint32_t i = 0; i < input->status_count; i++)
      {
        output->status[i].valid = input->status[i].valid;
        output->status[i].camera_name = std::string(input->status[i].camera_name);
        output->status[i].blockage = input->status[i].blockage;
        output->status[i].blurness = input->status[i].blurness;
      }
    }

    void parseDynamicCalibrationOutput(const EmbeddedNet_DynamicCalibrationResult *input,
                                       boost::shared_ptr<phantom_ros::PhantomVisionDynamicCalibrationOutputList> output)
    {
      output->calibration_outputs.resize(input->count);

      for (size_t i = 0; i < input->count; ++i)
      {
        auto& input_per_cam = input->item[i];
        auto& output_per_cam = output->calibration_outputs[i];

        output_per_cam.version = input_per_cam.version;
        output_per_cam.sync_id = input_per_cam.sync_id;
        output_per_cam.camera_type = input_per_cam.camera_type;
        output_per_cam.calibration_mode = input_per_cam.calibration_mode;
        output_per_cam.routine_request = input_per_cam.routine_request;
        output_per_cam.calibration_running_status = input_per_cam.calibration_running_status;
        output_per_cam.precondition_error = input_per_cam.precondition_error;
        /// @brief pause reason
        output_per_cam.roll_pause_status = input_per_cam.roll_pause_status;
        output_per_cam.pitch_pause_status = input_per_cam.pitch_pause_status;
        output_per_cam.yaw_pause_status = input_per_cam.yaw_pause_status;
        output_per_cam.height_pause_status = input_per_cam.height_pause_status;
        /// @brief converging progress of each estimation
        output_per_cam.roll_converging_progress = input_per_cam.roll_converging_progress;
        output_per_cam.pitch_converging_progress = input_per_cam.pitch_converging_progress;
        output_per_cam.yaw_converging_progress = input_per_cam.yaw_converging_progress;
        output_per_cam.height_converging_progress = input_per_cam.height_converging_progress;
        /// @brief converging status of each estimation
        output_per_cam.roll_converging_status = input_per_cam.roll_converging_status;
        output_per_cam.pitch_converging_status = input_per_cam.pitch_converging_status;
        output_per_cam.yaw_converging_status = input_per_cam.yaw_converging_status;
        output_per_cam.height_converging_status = input_per_cam.height_converging_status;
        /// @brief converging error of each estimation
        output_per_cam.roll_converging_error = input_per_cam.roll_converging_error;
        output_per_cam.pitch_converging_error = input_per_cam.pitch_converging_error;
        output_per_cam.yaw_converging_error = input_per_cam.yaw_converging_error;
        output_per_cam.height_converging_error = input_per_cam.height_converging_error;
        /// @brief converged time of each estimation
        output_per_cam.roll_converged_time_sec = input_per_cam.roll_converged_time_sec;
        output_per_cam.pitch_converged_time_sec = input_per_cam.pitch_converged_time_sec;
        output_per_cam.yaw_converged_time_sec = input_per_cam.yaw_converged_time_sec;
        output_per_cam.height_converged_time_sec = input_per_cam.height_converged_time_sec;
        /// @brief converged distance of each estimation
        output_per_cam.roll_converged_distance_meter = input_per_cam.roll_converged_distance_meter;
        output_per_cam.pitch_converged_distance_meter = input_per_cam.pitch_converged_distance_meter;
        output_per_cam.yaw_converged_distance_meter = input_per_cam.yaw_converged_distance_meter;
        output_per_cam.height_converged_distance_meter = input_per_cam.height_converged_distance_meter;
        /// @brief converged value of each estimation
        output_per_cam.roll_converged_degree = input_per_cam.roll_converged_degree;
        output_per_cam.pitch_converged_degree = input_per_cam.pitch_converged_degree;
        output_per_cam.yaw_converged_degree = input_per_cam.yaw_converged_degree;
        output_per_cam.height_converged_meter = input_per_cam.height_converged_meter;

        output_per_cam.camera_lateral_position = input_per_cam.camera_lateral_position;
        output_per_cam.camera_longitudinal_position = input_per_cam.camera_longitudinal_position;
        output_per_cam.camera_nominal_roll_deg = input_per_cam.camera_nominal_roll_deg;
        output_per_cam.camera_nominal_pitch_deg = input_per_cam.camera_nominal_roll_deg;
        output_per_cam.camera_nominal_yaw_deg = input_per_cam.camera_nominal_roll_deg;

        output_per_cam.image_health = input_per_cam.image_health;
      }
    }

    void parseCameraFailSafeInfo(const EmbeddedNet_FunctionalSafetyOutput *input, boost::shared_ptr<phantom_ros::FunctionalSafetyOutput> output)
    {
      phantom_ros::CameraFailSafeInformation fail_safe_info;

      for (uint32_t i = 0; i < input->fail_safe_count; i++)
      {
        fail_safe_info.camera_id = input->cameras_fail_safe_info[i].camera_id;
        // NOTE: commenting out since these are not being used for now (8/8/24)
        // fail_safe_info.dynamic_calibration_input_error = input->cameras_fail_safe_info[i].dynamic_calibration_input_error;
        // fail_safe_info.intrinsic_calibration_input_error = input->cameras_fail_safe_info[i].intrinsic_calibration_input_error;
        // fail_safe_info.dynamic_calibration_output_error = input->cameras_fail_safe_info[i].dynamic_calibration_output_error;
        fail_safe_info.camera_status_error = input->cameras_fail_safe_info[i].camera_status_error;

        output->camera_fail_safe_info_list.push_back(std::move(fail_safe_info));
      }
    }

    void parseVehicleStateFailSafeInfo(const EmbeddedNet_FunctionalSafetyOutput *input, const boost::shared_ptr<phantom_ros::FunctionalSafetyOutput>& output)
    {
      output->vehicle_state_fail_safe_info.yaw_rate_input_error    = input->vehicle_state_fail_safe_info.yaw_rate_input_error;
      output->vehicle_state_fail_safe_info.long_accel_input_error  = input->vehicle_state_fail_safe_info.long_accel_input_error;
      output->vehicle_state_fail_safe_info.lat_accel_input_error   = input->vehicle_state_fail_safe_info.lat_accel_input_error;
      output->vehicle_state_fail_safe_info.steer_angle_input_error = input->vehicle_state_fail_safe_info.steer_angle_input_error;
      output->vehicle_state_fail_safe_info.wheel_speed_input_error = input->vehicle_state_fail_safe_info.wheel_speed_input_error;
    }

    bool checkFullResolution(const uint16_t height){
      // check for 8mp or 2mp full resolution image
      return (height == 2160 || height == 1096);
    }

    bool isAnnotated(const uint16_t width){
      // check for annotated images
      return (width == 1920 || width == 960);
    }

    void parsePublishCameraExtraInfoFrame(const EmbeddedNet_CamerasExtraInfoList *input, boost::shared_ptr<phantom_ros::PhantomCameraExtraInfoFrame> output)
    {

      phantom_ros::PhantomCameraExtraInfo cam_extra_info;

      uint32_t sec_frame = input->hw_timestamp / 1000000;
      uint32_t nsec_frame = (input->hw_timestamp % 1000000) * 1000;
      output->header.stamp = ros::Time(sec_frame, nsec_frame);

      phantom_ai::TimeStamp now = phantom_ai::TimeStamp::Now();
      double ts_now = now.toSec();
      uint32_t sec_now = ((uint32_t)ts_now);
      uint32_t nsec_now = (ts_now - sec_now) * 1000000000;
      output->header2.stamp = ros::Time(sec_now, nsec_now);

      for (uint32_t i = 0; i < input->count_camera; i++)
      {
        double ts_cam = ((double)(input->cams_list[i].hw_timestamp))/1000000.0;
        PhantomCameraExtraInfoH3aConfig &h3a_cfg = cam_extra_info.h3a_cfg;
        h3a_cfg.v_start                 = input->cams_list[i].h3a_config.v_start        ;
        h3a_cfg.h_start                 = input->cams_list[i].h3a_config.h_start        ;
        h3a_cfg.v_size                  = input->cams_list[i].h3a_config.v_size         ;
        h3a_cfg.h_size                  = input->cams_list[i].h3a_config.h_size         ;
        h3a_cfg.v_count                 = input->cams_list[i].h3a_config.v_count        ;
        h3a_cfg.h_count                 = input->cams_list[i].h3a_config.h_count        ;
        h3a_cfg.v_skip                  = input->cams_list[i].h3a_config.v_skip         ;
        h3a_cfg.h_skip                  = input->cams_list[i].h3a_config.h_skip         ;
        h3a_cfg.data_array_size         = input->cams_list[i].h3a_config.data_array_size;
        cam_extra_info.camera_id        = input->cams_list[i].camera_id                 ;
        cam_extra_info.camera_name      = input->cams_list[i].camera_name               ;
        cam_extra_info.hw_timestamp     = ts_cam                                        ;
        cam_extra_info.hw_timestamp2    = ts_now                                        ;
        cam_extra_info.exposure_time    = input->cams_list[i].exposure_time             ;
        cam_extra_info.analog_gain      = input->cams_list[i].analog_gain               ;
        cam_extra_info.camera_mode      = input->cams_list[i].camera_mode               ;
        cam_extra_info.raw_crop_roi[0]  = input->cams_list[i].roi_raw_crop[0]           ;
        cam_extra_info.raw_crop_roi[1]  = input->cams_list[i].roi_raw_crop[1]           ;
        cam_extra_info.raw_crop_roi[2]  = input->cams_list[i].roi_raw_crop[2]           ;
        cam_extra_info.raw_crop_roi[3]  = input->cams_list[i].roi_raw_crop[3]           ;
        cam_extra_info.msg_version      = input->cams_list[i].hba_input_version         ;

        output->cams_extra.push_back(cam_extra_info);

        //big chunk data will be posted by image message overriding.
#ifdef USE_EXTRA_INPUT_HISTOGRAM
        //histogram is not implemented yet
        uint32_t *data_histogram        = const_cast<uint32_t *>(input->cams_list[i].histogram);
        uint8_t  *data_histogram_8bit   = (uint8_t *)data_histogram;
#endif
        uint8_t  *data_raw_crop         = const_cast<uint8_t *>(input->cams_list[i].raw16bit_crop);
        uint8_t  *data_h3a              = const_cast<uint8_t *>(input->cams_list[i].h3a);
        cv::Mat img_raw_crop            = cv::Mat(cam_extra_info.raw_crop_roi[3],
                                                  cam_extra_info.raw_crop_roi[2]*2,
                                                  CV_8UC1,
                                                  data_raw_crop);
        cv::Mat img_h3a                 = cv::Mat(1,
                                                  h3a_cfg.data_array_size,
                                                  CV_8UC1,
                                                  data_h3a);
#ifdef USE_EXTRA_INPUT_HISTOGRAM
        //histogram is not implemented yet
        cv::Mat img_histogram           = cv::Mat(1,
                                                  MAX_SIZE_CAMERA_EXTRA_HISTO*sizeof(uint32_t),
                                                  CV_8UC1,
                                                  data_histogram_8bit);
#endif
        uint32_t sec = input->cams_list[i].hw_timestamp / 1000000;
        uint32_t nsec = (input->cams_list[i].hw_timestamp % 1000000) * 1000;
        auto rostimestamp = ros::Time(sec, nsec);

        phantom_ai::CameraID cam_id = static_cast<phantom_ai::CameraID>(cam_extra_info.camera_id);
        std::shared_ptr<EmbeddedImageObj> cam_raw  = getRawImageObj(cam_id);
        std::shared_ptr<EmbeddedImageObj> cam_h3a  = getH3aImageObj(cam_id);
#ifdef USE_EXTRA_INPUT_HISTOGRAM
        std::shared_ptr<EmbeddedImageObj> cam_hist = getHistogramImageObj(cam_id);
        cam_hist->image = img_histogram;
#endif
        cam_raw->image  = img_raw_crop;
        cam_h3a->image  = img_h3a;

        std::string format_string = sensor_msgs::image_encodings::MONO8;
        sendImage(cam_raw,  format_string, rostimestamp);
        sendImage(cam_h3a,  format_string, rostimestamp);
#ifdef USE_EXTRA_INPUT_HISTOGRAM
        sendImage(cam_hist, format_string, rostimestamp);
#endif
      }
      camera_extra_info_frame_pub_.publish(output);
    }

    bool processFrame()
    {
      if (is_client_)
      {
        if (embeddedNetGetFrame(ti_handle_, &frame_) < 0)
        {
          PHANTOM_ERROR("Error receiving frame");
          std::string error = ros::this_node::getName().substr(1); // remove slash
          error += " Error receiving frame";
          health_monitor_.setHealthStatus(phantom_ai::ProcessHealthState::FAILURE, error);

          return false;
        }
      }
      else
      {
        if (embeddedServerNetStart(ti_handle_) >= 0)
        {
          if (embeddedServerNetGetFrame(ti_handle_, &frame_) < 0)
          {
            PHANTOM_ERROR("Error receiving frame");
            std::string error = ros::this_node::getName().substr(1); // remove slash
            error += " Error receiving frame";
            health_monitor_.setHealthStatus(phantom_ai::ProcessHealthState::FAILURE, error);

            return true; // wait for another client
          }
        }
        else
        {
          // sleep a bit so don't busy spin
          ros::Duration(0.010).sleep();

          // Connnection isn't ready, so just return
          return true;
        }
      }

      // Check for missed frames
      if (frame_.header.frame_counter != counter_)
      {
        PHANTOM_WARNING("Mismatched frame count {} != {}", frame_.header.frame_counter, counter_);
        counter_ = frame_.header.frame_counter;
      }

      counter_++;

      cv::Mat cconverted;

      // Multiple images are aggregated into buffer, split them up.
      EmbeddedNet_PhantomHeader *hdr = &frame_.header;

      // convert to sec, nsec. TI time is only us resolution
      uint32_t sec = hdr->timestamp / 1000000;
      uint32_t nsec = (hdr->timestamp % 1000000) * 1000;
      ros::Time hw_timestamp = ros::Time(sec, nsec);
      uint32_t frame_count = hdr->img_frame_count;

      // Commented out as it logs too frequently
      // PHANTOM_LOG("sec: {} , nsec: {}", sec, nsec);

      if (hdr->item_count > MAX_NETWORK_PHANTOMHEADER_ITEMS)
      {
        throw phantom_ai::PhantomAIException("Illegal count of items {}", hdr->item_count);
      }

      // cycle through elements
      for (uint32_t i = 0; i < hdr->item_count; i++)
      {
        EmbeddedNet_PhantomHeader_Info *item = &(hdr->item[i]);

        if (item->offset[0] > frame_.data_size)
        {
          throw phantom_ai::PhantomAIException("Illegal offset {}", item->offset[0]);
        }
        void* data = &(frame_.data[item->offset[0]]);

        switch (item->format)
        {
          // all YUV422 cases
          case CAMERA_IMG_YUV_422:
          {
            is_annotated_ = isAnnotated(item->width);
            generate_images_ &= checkFullResolution(item->height);

            // Setup cv::Mat for YUV422
            cv::Mat yuv(item->height, item->width, CV_8UC2, data);

            cv::ColorConversionCodes ccc = openCvConversion(format_yuv422, output_format_);

            // convert frame to correct color format
            // do conversion first since it's simpler for cropping
            cv::cvtColor(yuv, cconverted, ccc);

            generateImage(cconverted, output_format_, hw_timestamp, frame_count, item->id);

            break;
          }

          // all YUV 420 SP (NV12) cases
          case CAMERA_IMG_YUV_420SP: // fallthrough
          {
            is_annotated_ = isAnnotated(item->width);

            generate_images_ &= checkFullResolution(item->height);

            // Setup cv::Mat for YUV420sp (actually NV12!)
            cv::Mat yuv(item->height * 3 / 2, item->pitch[0], CV_8UC1, data);

            cv::ColorConversionCodes ccc = openCvConversion(format_yuv420sp, output_format_);

            // convert frame to correct color format
            // do conversion first since it's simpler for cropping
            cv::cvtColor(yuv, cconverted, ccc);

            generateImage(cconverted, output_format_, hw_timestamp, frame_count, item->id);
            break;
          }

          // all RGB 8 bit cases
          case CAMERA_IMG_RGB: // fallthrough
          {
            is_annotated_ = isAnnotated(item->width);
            generate_images_ &= checkFullResolution(item->height);
            cv::Mat rgb(item->height, item->width, CV_8UC3, data);
            cv::ColorConversionCodes ccc = openCvConversion(format_rgb, output_format_);

            if (ccc != cv::COLOR_COLORCVT_MAX)
            {
              // convert frame to correct color format
              cv::cvtColor(rgb, cconverted, ccc);
            }
            else
            {
              // copy pointers
              cconverted = rgb;
            }

            generateImage(cconverted, output_format_, hw_timestamp, frame_count, item->id);
            break;
          }

          // all BGR 8 bit cases
          case CAMERA_IMG_BGR:
          {
            is_annotated_ = isAnnotated(item->width);
            generate_images_ &= checkFullResolution(item->height);

            cv::Mat bgr(item->height, item->width, CV_8UC3, data);

            cv::ColorConversionCodes ccc = openCvConversion(format_bgr, output_format_);

            if (ccc != cv::COLOR_COLORCVT_MAX)
            {
              // convert frame to correct color format
              cv::cvtColor(bgr, cconverted, ccc);
            }
            else
            {
              // copy pointers
              cconverted = bgr;
            }

            generateImage(cconverted, output_format_, hw_timestamp, frame_count, item->id);
            break;
          }

          case SEGMAP:
          {
#if 0
            EmbeddedNet_PhantomCamera cid = static_cast<EmbeddedNet_PhantomCamera>(item->id);
            if (cid == EmbeddedNet_PhantomCamera::FRONT_CENTER)
            {
              cv::Mat mono(item->height, item->width, CV_8UC1, data);
              std::string format = "mono8";

              // no conversion for segmap since it's mono, always
              cropAndResize(mono, main_seg_image_.image);

              // publish frame
              sendImage(&main_seg_image_, format, hw_timestamp);
            }
            else if (cid == EmbeddedNet_PhantomCamera::FRONT_CROP)
            {
              cv::Mat mono(item->height, item->width, CV_8UC1, data);
              std::string format = "mono8";

              // no conversion for segmap since it's mono, always
              cropAndResize(mono, crop_seg_image_.image);

              // publish frame
              sendImage(&crop_seg_image_, format, hw_timestamp);
            }
            else
            {
              PHANTOM_WARNING("Unsupported SEGMAP {}", item->id);
            }
#endif
            break;
          }

          case VEHICLE_STATE:
          {
            uint32_t i;
            EmbeddedNet_VehicleInfo *info = (EmbeddedNet_VehicleInfo*)data;

            for (i = 0; i < info->esp_count && i < MAX_VEHICLE_STATE_NUM; i++)
            {
              boost::shared_ptr<phantom_ros::EspMeasurements> msg(new phantom_ros::EspMeasurements);

              uint32_t sec = info->esp[i].stamp / 1000000;
              uint32_t nsec = (info->esp[i].stamp % 1000000) * 1000;
              msg->header.stamp = ros::Time(sec, nsec);
              msg->yaw_rate = info->esp[i].yaw_rate;
              msg->long_accel = info->esp[i].long_accel;
              msg->hw_timestamp = msg->header.stamp.toSec();
              esp_measurements_pub_.publish(msg);
            }

            for (i = 0; i < info->wheelspeed_count && i < MAX_VEHICLE_STATE_NUM; i++)
            {
              boost::shared_ptr<phantom_ros::WheelSpeedMeasurements> msg(new phantom_ros::WheelSpeedMeasurements);

              uint32_t sec = info->wheelspeed[i].stamp / 1000000;
              uint32_t nsec = (info->wheelspeed[i].stamp % 1000000) * 1000;
              msg->header.stamp = ros::Time(sec, nsec);

              msg->front_left = info->wheelspeed[i].front_left;
              msg->front_right = info->wheelspeed[i].front_right;
              msg->rear_left = info->wheelspeed[i].rear_left;
              msg->rear_right = info->wheelspeed[i].rear_right;
              msg->hw_timestamp = msg->header.stamp.toSec();

              wheel_speed_pub_.publish(msg);
            }

            for (i = 0; i < info->gearstate_count && i < MAX_VEHICLE_STATE_NUM; i++)
            {
              boost::shared_ptr<phantom_ros::GearState> msg(new phantom_ros::GearState);

              uint32_t sec = info->gearstate[i].stamp / 1000000;
              uint32_t nsec = (info->gearstate[i].stamp % 1000000) * 1000;
              msg->header.stamp = ros::Time(sec, nsec);
              msg->gear_state = info->gearstate[i].gear_state;
              msg->hw_timestamp = msg->header.stamp.toSec();
              gear_state_pub_.publish(msg);
            }

            break;
          }

          case BBOX_INFO:
          {
#if 0
            // Only for debugging, just print out data;
            EmbeddedNet_PhantomBBOXInfo *obj = (EmbeddedNet_PhantomBBOXInfo*)data;
            EmbeddedNet_PhantomCamera cid = static_cast<EmbeddedNet_PhantomCamera>(item->id);
            if (cid == EmbeddedNet_PhantomCamera::FRONT)
            {
              PHANTOM_LOG("Main BBox info, size {}", obj->count);
            }
            else if (cid == EmbeddedNet_PhantomCamera::FRONT_CROP)
            {
              PHANTOM_LOG("Crop BBox info, size {}", obj->count);
            }
            else
            {
              PHANTOM_ERROR("Unknown bbox info type {}", item->id);
            }

            for (uint32_t i = 0; i < obj->count && i < MAX_BBOX_INFO_NUM; i++)
            {
              PHANTOM_LOG("  BOX[{}]: {}x{}, {}x{}", i, obj->bbox[i].x_px, obj->bbox[i].y_px, obj->bbox[i].width_px, obj->bbox[i].height_px);
              PHANTOM_LOG("    Class {}, id {}, score {}", obj->bbox[i].classification, obj->bbox[i].id, obj->bbox[i].score);
            }
#endif
            break;
          }

          case VISION_RESULT:
          {
            EmbeddedNet_PhantomVisionList *results = (EmbeddedNet_PhantomVisionList*)data;

            // Sanity check
            if (item->buf_size[0] != sizeof(EmbeddedNet_PhantomVisionList))
            {
              throw phantom_ai::PhantomAIException("EmbeddedNet_PhantomVisionList size mismatch");
            }

            for (uint32_t i = 0; i < results->count; i++)
            {
              EmbeddedNet_PhantomVisionResult *input = &results->item[i];

              boost::shared_ptr<phantom_ros::PhantomVisionMeasurement> vision_result(new phantom_ros::PhantomVisionMeasurement);

              parseLaneResult(input, vision_result);
              parseObjectResult(input, vision_result);
              parseTrafficSignResult(input, vision_result);
              parseFreeSpaceResult(input, vision_result);
              parseStatusResult(input, vision_result);

              // top level
              vision_result->x_m = input->x_m;
              vision_result->y_m = input->y_m;
              vision_result->heading_rads = input->heading_rads;
              vision_result->frame_count = input->frame_count;

              uint32_t sec = input->stamp / 1000000;
              uint32_t nsec = (input->stamp % 1000000) * 1000;
              vision_result->header.stamp = ros::Time(sec, nsec);

              vision_result->hw_timestamp = vision_result->header.stamp.toSec();

              vision_measurement_pub_.publish(vision_result);
            }

            break;
          }

          // Vision debug viz image
          case VISION_VIZ_IMG:
          {
            if(!viz_show_inited_)
            {
              if(show_vision_debug_viz_image_)
              {
                viz_show_thread_ = std::thread(&embeddedCaptureNodelet::vizShowLoop, this);
                pthread_setname_np(viz_show_thread_.native_handle(), "VisionVizShow");
              }

              if(publish_vision_viz_topic_)
              {
                vision_debug_viz_obj_ = std::make_shared<EmbeddedImageObj>();
                vision_debug_viz_obj_->vision_pub = node_.advertise<sensor_msgs::Image>(vision_viz_topic_name_, 1);
              }

              viz_show_inited_ = true;
            }

            cv::Mat rgb(item->height, item->width, CV_8UC3, data);

            cv::ColorConversionCodes ccc = openCvConversion(format_rgb, sensor_msgs::image_encodings::RGB8);

            if (ccc != cv::COLOR_COLORCVT_MAX)
            {
              // convert frame to correct color format
              cv::cvtColor(rgb, cconverted, ccc);
            }
            else
            {
              // copy pointers
              cconverted = rgb;
            }
            viz_data_mutex_.lock();
            viz_data_ = cconverted;
            viz_data_mutex_.unlock();

            if(publish_vision_viz_topic_ && vision_debug_viz_obj_)
            {
              vision_debug_viz_obj_->image = cconverted;
              std::string format = sensor_msgs::image_encodings::BGR8;
              sendImage(vision_debug_viz_obj_, format, hw_timestamp);
            }

            break;
          }

          case CAMERA_CALIB:
          {
            EmbeddedNet_CameraCalibration *results = (EmbeddedNet_CameraCalibration*)data;

            // Sanity check
            if (item->buf_size[0] != sizeof(EmbeddedNet_CameraCalibration))
            {
              throw phantom_ai::PhantomAIException("EmbeddedNet_CameraCalibration size mismatch");
            }

            for (int i = 0; i < MAX_CAM_INFO_COUNT; i++)
            {
              if (results->info[i].valid == 1)
              {
                boost::shared_ptr<phantom_ros::PhantomCameraInfo> msg(new phantom_ros::PhantomCameraInfo);

                if (results->info[i].stamp != 0)
                {
                  uint32_t sec = results->info[i].stamp / 1000000;
                  uint32_t nsec = (results->info[i].stamp % 1000000) * 1000;
                  msg->header.stamp = ros::Time(sec, nsec);
                }
                else
                {
                  // no timestamp, use current time
                  msg->header.stamp = ros::Time::now();
                }

                msg->width = results->info[i].width;
                msg->height = results->info[i].height;
                msg->distortion_model = std::string(results->info[i].distortion_model);

                msg->D.resize(MAX_CAM_CALIB_D_LEN);
                for (int j = 0; j < MAX_CAM_CALIB_D_LEN; j++)
                {
                  msg->D[j] = results->info[i].D[j];
                }

                if (msg->R.size() >= (MAX_CAM_CALIB_SKEW_LEN+2))
                {
                  for (int j = 0; j < MAX_CAM_CALIB_SKEW_LEN; j++)
                  {
                    msg->R[j] = results->info[i].skew[j];
                  }
                  msg->R[MAX_CAM_CALIB_SKEW_LEN] = results->info[i].output_width;
                  msg->R[MAX_CAM_CALIB_SKEW_LEN+1] = results->info[i].output_height;
                }
                else
                {
                  PHANTOM_WARNING("Cam calib R param array mismatch");
                }

                if (msg->P.size() >= MAX_CAM_CALIB_P_LEN)
                {
                  for (int j = 0; j < MAX_CAM_CALIB_P_LEN; j++)
                  {
                    msg->P[j] = results->info[i].P[j];
                  }
                }
                else
                {
                  PHANTOM_WARNING("Cam calib P param array mismatch");
                }

                msg->binning_x = results->info[i].binning_x;
                msg->binning_y = results->info[i].binning_y;
                msg->roi.x_offset = results->info[i].roi_x_offset;
                msg->roi.y_offset = results->info[i].roi_y_offset;
                msg->roi.height = results->info[i].roi_height;
                msg->roi.width = results->info[i].roi_width;
                msg->name = std::string(results->info[i].name);
                msg->serial_number = std::string(results->info[i].serial_number);
                msg->sensor_type = std::string(results->info[i].sensor_type);
                msg->lens_fov = results->info[i].lens_fov;

                is_annotated_ = (msg->roi.height == ANNOTATION_LARGE_HEIGHT || msg->roi.height == ANNOTATION_SMALL_HEIGHT);

                if (msg->position.size() >= MAX_CAM_CALIB_POSE_LEN)
                {
                  for (int j = 0; j < MAX_CAM_CALIB_POSE_LEN; j++)
                  {
                    msg->position[j] = results->info[i].position[j];
                  }
                }
                else
                {
                  PHANTOM_WARNING("Cam calib position array mismatch");
                }

                if (msg->rotation.size() >= MAX_CAM_CALIB_POSE_LEN)
                {
                  for (int j = 0; j < MAX_CAM_CALIB_POSE_LEN; j++)
                  {
                    msg->rotation[j] = results->info[i].rotation[j];
                  }
                }
                else
                {
                  PHANTOM_WARNING("Cam calib rotation array mismatch");
                }

                msg->horizontal_flip = results->info[i].horizontal_flip;
                msg->vertical_flip = results->info[i].vertical_flip;
                msg->location = std::string(results->info[i].location);
                msg->vehicle = std::string(results->info[i].vehicle);
                msg->user_comment = std::string(results->info[i].user_comment);

                phantom_ai::CameraID cid = phantom_ai::camera_name(msg->name, true);
                std::shared_ptr<EmbeddedImageObj> cam = getImageObj(cid);
                if (cam != nullptr)
                {
                  if (is_annotated_) {
                    if (counter_%3 == 1) {
                      cam->annotation_cam_info_pub.publish(msg);
                    }
                  } else {
                    cam->vision_cam_info_pub.publish(msg);
                  }
                }
              }
            }

            break;
          }

          case CAN_TX: // fallthrough
          case CAN_RX: // fallthrough
          {
            boost::shared_ptr<phantom_ros::Can> msg(new phantom_ros::Can);

            EmbeddedNet_CanFrames *results = (EmbeddedNet_CanFrames*)data;

            int32_t idx = getCanIndex(item->format, item->id);

            if (idx >= 0)
            {
              uint32_t max_count = std::min(results->count, (uint32_t)MAX_CAN_MSG_COUNT);

              msg->can_list.resize(max_count);

              for (uint32_t i = 0; i < max_count; i++)
              {
                EmbeddedNet_CanMessage *frame = (EmbeddedNet_CanMessage*)&(results->msgs[i]);
                msg->can_list[i].id = frame->id;
                msg->can_list[i].dlc = frame->dlc;
                uint32_t len = std::min(frame->dlc, (uint8_t)MAX_CAN_DATA_LEN);
                msg->can_list[i].payload.resize(len);
                memcpy((void*)msg->can_list[i].payload.data(), (void*)frame->payload, len);

                // convert to sec, nsec. TI time is only us resolution
                uint32_t sec = frame->timestamp / 1000000;
                uint32_t nsec = (frame->timestamp % 1000000) * 1000;
                msg->can_list[i].header.stamp = ros::Time(sec, nsec);
                msg->can_list[i].timestamp = msg->can_list[i].header.stamp.toSec();
                msg->can_list[i].header.seq = can_seq_num_[idx]++;
              }

              if (max_count > 0)
              {
                // use first message stamp
                msg->header.stamp = msg->can_list[0].header.stamp;
              }
              else
              {
                // use message stamp
                msg->header.stamp = hw_timestamp;
              }

              can_pub_[idx].publish(msg);
            }

            break;
          }

          case DYNM_CALIB_RESULT:
          {
            EmbeddedNet_DynamicCalibrationResultList *results = (EmbeddedNet_DynamicCalibrationResultList*)data;

            // Sanity check
            if (item->buf_size[0] != sizeof(EmbeddedNet_DynamicCalibrationResultList))
            {
              throw phantom_ai::PhantomAIException("EmbeddedNet_DynamicCalibrationResultList size mismatch");
            }

            for (size_t i = 0; i < results->count; ++i)
            {
              EmbeddedNet_DynamicCalibrationResult *input = &results->item[i];

              boost::shared_ptr<phantom_ros::PhantomVisionDynamicCalibrationOutputList> dynm_cal_output_list(
                new phantom_ros::PhantomVisionDynamicCalibrationOutputList);

              parseDynamicCalibrationOutput(input, dynm_cal_output_list);

              // timestamp from board is RTOS time since boot up, which won't match the ROS bag timestamps
              // that use logging PC local host time. So, we'll use the ROS time now.
              dynm_cal_output_list->header.stamp = ros::Time::now();

              dynm_calibration_output_pub_.publish(dynm_cal_output_list);
            }

            break;
          }

          case FUSA_OUTPUT:
          {
            EmbeddedNet_FunctionalSafetyOutputList *results = (EmbeddedNet_FunctionalSafetyOutputList*)data;

            // Sanity check
            if (item->buf_size[0] != sizeof(EmbeddedNet_FunctionalSafetyOutputList))
            {
              throw phantom_ai::PhantomAIException("EmbeddedNet_FunctionalSafetyOutputList size mismatch");
            }

            for (uint32_t i = 0; i < results->count; i++)
            {
              EmbeddedNet_FunctionalSafetyOutput *input = &results->item[i];

              boost::shared_ptr<phantom_ros::FunctionalSafetyOutput> fusa_data(new phantom_ros::FunctionalSafetyOutput);

              parseCameraFailSafeInfo(input, fusa_data);
              parseVehicleStateFailSafeInfo(input, fusa_data);

              uint32_t sec = input->stamp / 1000000;
              uint32_t nsec = (input->stamp % 1000000) * 1000;
              fusa_data->header.stamp = ros::Time(sec, nsec);

              fusa_output_pub_.publish(fusa_data);
            }

            break;
          }

          case CAMERA_EXTRA_INFO:
          {
            EmbeddedNet_CameraExtraInfoListTopLevel *results = (EmbeddedNet_CameraExtraInfoListTopLevel*)data;
            // Sanity check
            if (item->buf_size[0] != sizeof(EmbeddedNet_CameraExtraInfoListTopLevel))
            {
              throw phantom_ai::PhantomAIException("EmbeddedNet_CameraExtraInfoListTopLevel size mismatch");
            }
            else
            {
              for (uint32_t i = 0; i < results->count; i++)
              {
                EmbeddedNet_CamerasExtraInfoList *inputlist = &results->extra_info_list[i]; //containing many cams
                boost::shared_ptr<phantom_ros::PhantomCameraExtraInfoFrame> extra_data_frame(new phantom_ros::PhantomCameraExtraInfoFrame);
                parsePublishCameraExtraInfoFrame(inputlist, extra_data_frame);
              }
            }
            break;
          }

          case HBA_OUTPUT:
          {
            EmbeddedNet_PhantomVisionHBAList *results = (EmbeddedNet_PhantomVisionHBAList*)data;

            // Sanity check
            if (item->buf_size[0] != sizeof(EmbeddedNet_PhantomVisionHBAList))
            {
              throw phantom_ai::PhantomAIException("EmbeddedNet_PhantomVisionHBAList size mismatch");
            }

            for (uint32_t i = 0; i < results->count; i++)
            {
              EmbeddedNet_PhantomVisionHBA *input = &results->item[i];

              boost::shared_ptr<phantom_ros::PhantomVisionHBA> hba_data(new phantom_ros::PhantomVisionHBA);

              hba_data->illumination = input->illumination;
              hba_data->backlight_status = input->backlight_status;
              hba_data->camera_is_in_tunnel = input->camera_is_in_tunnel;
              hba_data->vehicle_light_info = input->vehicle_light_info;
              hba_data->beam_result_availability = input->beam_result_availability;
              hba_data->beam_result = input->beam_result;
              hba_data->street_light_status = input->street_light_status;
              hba_data->street_light_count_scene = input->street_light_count_scene;
              hba_data->environmental_status_scene = input->environmental_status_scene;
              hba_data->camera_mode = input->camera_mode;
              hba_data->exposure_time = input->exposure_time;
              hba_data->analog_gain = input->analog_gain;
              hba_data->ambient_light_lux = input->ambient_light_lux;
              hba_data->horizon_fullres = input->horizon_fullres;
              hba_data->horizon_h3a = input->horizon_h3a;
              hba_data->curvature_radius = input->curvature_radius;
              hba_data->hba_deactive_speed = input->hba_deactive_speed;
              hba_data->hba_deactive_curv = input->hba_deactive_curv;
              hba_data->hba_deactive_min_l2h_time_flag = input->hba_deactive_min_l2h_time_flag;
              hba_data->hba_deactive_min_l2h_time_value = input->hba_deactive_min_l2h_time_value;

              //original scale for double type time stamp
              uint32_t sec = input->hw_timestamp / 1000000;
              uint32_t nsec = (input->hw_timestamp % 1000000) * 1000;
              hba_data->header.stamp = ros::Time(sec, nsec);
              // header.stamp and hba_data->hw_timestamp are duplicated.
              //But for the debug purpose, let's keep both.
              hba_data->hw_timestamp = ((double)(input->hw_timestamp))/1000000.0;

              //for the debug purpose
              //let's keep logging PC's time stamp also
              phantom_ai::TimeStamp now = phantom_ai::TimeStamp::Now();
              double ts_now = now.toSec();
              hba_data->hw_timestamp2 = ts_now;
              uint32_t sec_now = ((uint32_t)ts_now);
              uint32_t nsec_now = (ts_now - sec_now) * 1000000000;
              hba_data->header2.stamp = ros::Time(sec_now, nsec_now);

              hba_output_pub_.publish(hba_data);
            }

            break;
          }

          case AEB_OUTPUT:
          {
            EmbeddedNet_PhantomVisionAEBList *results = (EmbeddedNet_PhantomVisionAEBList*)data;

            // Sanity check
            if (item->buf_size[0] != sizeof(EmbeddedNet_PhantomVisionAEBList))
            {
              throw phantom_ai::PhantomAIException("EmbeddedNet_PhantomVisionList size mismatch");
            }

            for (uint32_t i = 0; i < results->count; i++)
            {
              EmbeddedNet_PhantomVisionAEB *input = &results->item[i];

              boost::shared_ptr<phantom_ros::PhantomVisionAEB> aeb_result(new phantom_ros::PhantomVisionAEB);

              parseAEBResult(input, aeb_result);

              uint32_t sec  = input->stamp / 1000000;
              uint32_t nsec = (input->stamp % 1000000) * 1000;

              // in case the header timestamp of received AEB data is not set
              if(sec == 0) {
                phantom_ai::TimeStamp now = phantom_ai::TimeStamp::Now();
                double ts_now = now.toSec();
                sec = ((uint32_t)ts_now);
                nsec = (ts_now - sec) * 1000000000;
              }

              aeb_result->header.stamp = ros::Time(sec, nsec);
              aeb_result->hw_timestamp = aeb_result->header.stamp.toSec();

              aeb_output_pub_.publish(aeb_result);
            }

            break;
          }

          case CAMERA_IMG_H264:
          {
            PHANTOM_WARNING("CAMERA_IMG_H264 format received!");

            // Step1. Decode the H.264 encoded data via GST
                // Encoding pipeline was -->  appsrc name=h264_source is-live=true format=time block=true caps=video/x-raw,format=NV12,width=%d,height=%d,framerate=15/1,interlace-mode=progressive,colorimetry=bt709 ! queue ! v4l2h264enc extra-controls="controls,video_bitrate=(int)4000000,video_gop_size=(int)15" ! h264parse config-interval=-1 ! appsink name=h264_sink emit-signals=true max-buffers=1 drop=true
                // The decoding pipeline needs to be implemented in vision_apps/apps/dl_demos/phantom_common/gst_stream/gst_stream.c, h 
                // The decoding pipeline string should be stored in the src/phantom_ai/params/sensors/embedded_network/gst_config.yaml
                // After decoding, "decoded_buffer" holds a NV12 tiled image. (it would be cv::Mat ?)

            // Step2. Split the received tiled mosaic image using the corresponding camera calibration
                // Include the ROI for each image in the network message payload during sending (in sendEncodedImageFrame?)
                // On the SDK side, the logic lives in tivx_multi_cam_get_mosaic_dimensions() within multi_cam_tivx.c
                // In the set_img_mosaic_params in the multi_cam_tivx.c, the code below contains each ROI:
                    //for (ch = 0; ch < numCh; ch++) {
                    //  for (j = 0; j < param_array[ch].num_subimages; j++) {
                    //    imgMosaicObj->params.windows[idx].width   = param_array[ch].subimage[j].output_width;
                    //    imgMosaicObj->params.windows[idx].height  = param_array[ch].subimage[j].output_height;
                // we need to split a single NV12 image into its sub-images --> How? 
                // After this step, we should have three of cv::Mat images, with each mapped to a camera location.
                // The locations are stored in the data packet as an array during networkLoop() in the vision_demo2.cpp like below
                    //std::vector<phantom_ai::CameraID> loc_arr;
                    //for (auto it: output_arr->img_obj) {
                    //  if(!gst_h264encoding_enabled_) img_arr.push_back(it.second);
                    //  loc_arr.push_back(it.first);

            // Step3. Setup cv::Mat for NV12
                // Determine how many sub-images are tiled, then iterate:
                    //cv::Mat yuv(item->height * 3 / 2, item->pitch[0], CV_8UC1, data);
                    //cv::ColorConversionCodes ccc = openCvConversion(format_yuv420sp, output_format_);

            // Step4. Generate image
                // Determine how many sub-images are tiled, then iterate:
                    //generateImage(cconverted, output_format_, hw_timestamp, frame_count, item->id);
            break;
          }

          // Unsupported items
          case STATS_INFO: // fallthrough
          case GENERIC_BUFFER: // fallthrough
          default:
            PHANTOM_WARNING("Unknown format {}, skipping", static_cast<int>(item->format));
            break;
        }
      }

      updateStats();

      health_monitor_.setHealthStatus(phantom_ai::ProcessHealthState::RUNNING);

      return true;
    }

    void captureLoop()
    {
      while (is_running_)
      {
        if (processFrame() == false)
        {
          break;
        }
      }

      PHANTOM_LOG("Stopping receiver");
      if (is_client_)
      {
        if (embeddedNetStop(ti_handle_) < 0)
        {
          PHANTOM_ERROR("Shutdown of receiver failed");
        }
      }
      else
      {
        if (embeddedServerNetStop(ti_handle_) < 0)
        {
          PHANTOM_ERROR("Closing connection failed");
        }
        if (embeddedServerNetDeinit(ti_handle_) < 0)
        {
          PHANTOM_ERROR("Deinit failed");
        }
      }

      if (is_running_)
      {
        // loop exit wasn't normal, try to die
        ros::shutdown();
      }

    }

    void startTimer(const char* board_ip, const uint32_t board_port)
    {
      // only need to adjust board time when actual logging
      if (is_client_) {
        embeddedNetworkTimerRun(board_ip, board_port, false);
      }
    }

    void vizShowLoop()
    {
      uint32_t kLoopTimerHz{10};
      phantom_ai::Timer timer{kLoopTimerHz};
      std::string window_name{"PhantomVision"};
      cv::startWindowThread();

      while(is_running_)
      {
        timer.tic();
        cv::Mat img;
        viz_data_mutex_.lock();
        if (vision_viz_window_scale_factor_ == 1.0f)
        {
          img = viz_data_;
        }
        else
        {
          cv::Size size = cv::Size2f(viz_data_.size()) * vision_viz_window_scale_factor_;
          cv::resize(viz_data_, img, size, 0, 0, cv::INTER_LINEAR);
        }
        viz_data_mutex_.unlock();

        if(!img.empty())
        {
          cv::namedWindow(window_name, cv::WND_PROP_AUTOSIZE);
          cv::imshow(window_name, img);
        }

        timer.toc();
      }
    }

    virtual void onInit()
    {
      /* Set up node */
      node_ = getNodeHandle();

      // Load the config.
      phantom_ai::YamlNode cfg = getConfig("sensors/embedded_network", "capture.yaml");

      std::string server_ip = phantom_ai::get_yaml_value(cfg, "server_ip").as<std::string>();
      std::string server_binding_ip = phantom_ai::get_yaml_value(cfg, "binding_ip").as<std::string>();

      std::string mode = phantom_ai::get_yaml_value(cfg, "mode").as<std::string>();
      uint32_t port = phantom_ai::get_yaml_value(cfg, "port").as<uint32_t>();
      uint32_t connect_timeout = phantom_ai::get_yaml_value(cfg, "connect_timeout").as<uint32_t>();
      uint32_t read_timeout = phantom_ai::get_yaml_value(cfg, "read_timeout").as<uint32_t>();

      enable_cropping_ = phantom_ai::get_yaml_value(cfg, "crop_enable").as<bool>();
      generate_images_ = phantom_ai::get_yaml_value(cfg, "generate_image").as<bool>();

      if (enable_cropping_)
      {
        crop_win_ = cv::Rect(phantom_ai::get_yaml_value(cfg, "crop_start_x").as<int32_t>(),
                             phantom_ai::get_yaml_value(cfg, "crop_start_y").as<int32_t>(),
                             phantom_ai::get_yaml_value(cfg, "crop_width").as<int32_t>(),
                             phantom_ai::get_yaml_value(cfg, "crop_height").as<int32_t>());
      }

      int output_width = phantom_ai::get_yaml_value(cfg, "output_width").as<int32_t>();
      int output_height = phantom_ai::get_yaml_value(cfg, "output_height").as<int32_t>();
      std::string encoding = phantom_ai::get_yaml_value(cfg, "output_format").as<std::string>();

      uint32_t buffer_size = phantom_ai::get_yaml_value(cfg, "capture_buffer_size").as<uint32_t>();

      topic_name_prefix_ = phantom_ai::get_yaml_value(cfg, "topic_prefix").as<std::string>();
      // allow ros params to override YAML config
      ros::NodeHandle private_node_handle = getPrivateNodeHandle();
      std::string camera_basename;
      std::string info_basename;
      private_node_handle.param("topic_prefix", camera_basename, topic_name_prefix_);
      private_node_handle.param("info_prefix", info_basename, topic_name_prefix_);

      std::string esp_measurements_topic_name           = info_basename + phantom_ai::get_yaml_value(cfg, "esp_measurements_topic_name").as<std::string>();
      std::string wheel_speed_topic_name        = info_basename + phantom_ai::get_yaml_value(cfg, "wheel_speed_topic_name").as<std::string>();
      std::string gear_state_topic_name        = info_basename + phantom_ai::get_yaml_value(cfg, "gear_state_topic_name").as<std::string>();
      std::string vision_measurement_topic_name = info_basename + phantom_ai::get_yaml_value(cfg, "vision_measurement_topic_name").as<std::string>();
      std::string can_topic_prefix_name         = info_basename + phantom_ai::get_yaml_value(cfg, "can_topic_prefix_name").as<std::string>();
      std::string dynm_cal_output_topic_name    = info_basename + phantom_ai::get_yaml_value(cfg, "dynm_cal_output_topic_name").as<std::string>();
      std::string fusa_output_topic_name        = info_basename + phantom_ai::get_yaml_value(cfg, "fusa_output_topic_name").as<std::string>();
      std::string cam_ext_frame_topic_name      = info_basename + phantom_ai::get_yaml_value(cfg, "camera_extra_info_frame_topic_name").as<std::string>();
      std::string hba_output_topic_name         = info_basename + phantom_ai::get_yaml_value(cfg, "hba_output_topic_name").as<std::string>();
      std::string aeb_output_topic_name         = info_basename + phantom_ai::get_yaml_value(cfg, "aeb_output_topic_name").as<std::string>();


      vision_viz_window_scale_factor_ = phantom_ai::get_yaml_key<float>(cfg, "vision_viz_window_scale_factor", kVizWindowScaleFactorMax);

      if(vision_viz_window_scale_factor_ <= 0.0 || vision_viz_window_scale_factor_ > kVizWindowScaleFactorMax)
      {
        throw phantom_ai::PhantomAIException("Invalid vision_viz_window_scale_factor value {} that should be between 0.0 ~ {}",
                                             vision_viz_window_scale_factor_, kVizWindowScaleFactorMax);
      }

      publish_vision_viz_topic_ = phantom_ai::get_yaml_key<bool>(cfg, "publish_vision_viz_topic", false);
      show_vision_debug_viz_image_ = phantom_ai::get_yaml_key<bool>(cfg, "show_vision_debug_viz_image", false);
      vision_viz_topic_name_ = info_basename + phantom_ai::get_yaml_value(cfg, "vision_viz_topic_name").as<std::string>();

      if (encoding == "rgb")
      {
        output_format_ = sensor_msgs::image_encodings::RGB8;
      }
      else if (encoding == "bgr")
      {
        output_format_ = sensor_msgs::image_encodings::BGR8;
      }
      else
      {
        throw phantom_ai::PhantomAIException("Invalid output format {}", encoding);
      }

      output_cam_image_size_.width = output_width;
      output_cam_image_size_.height = output_height;
      counter_ = 1;

      // alloc data buffer
      frame_.data_size = buffer_size;
      frame_.data = new uint8_t[frame_.data_size];

      if (frame_.data == nullptr)
      {
        throw phantom_ai::PhantomAIException("alloc failed %d b", frame_.data_size);
      }

      embedded_img_objs_.clear();
      embedded_img_objs_.resize(phantom_ai::CameraID::NUM_MAX_CAM_IDS, nullptr);
#ifdef USE_EXTRA_INPUT_HISTOGRAM
      camera_extra_info_histogram_objs_.clear();
      camera_extra_info_histogram_objs_.resize(phantom_ai::CameraID::NUM_MAX_CAM_IDS, nullptr);
#endif
      camera_extra_info_h3a_objs_.clear();
      camera_extra_info_h3a_objs_.resize(phantom_ai::CameraID::NUM_MAX_CAM_IDS, nullptr);
      camera_extra_info_raw_objs_.clear();
      camera_extra_info_raw_objs_.resize(phantom_ai::CameraID::NUM_MAX_CAM_IDS, nullptr);

      esp_measurements_pub_ = node_.advertise<phantom_ros::EspMeasurements>(esp_measurements_topic_name, 20);
      wheel_speed_pub_ = node_.advertise<phantom_ros::WheelSpeedMeasurements>(wheel_speed_topic_name, 10);
      gear_state_pub_ = node_.advertise<phantom_ros::GearState>(gear_state_topic_name, 10);
      vision_measurement_pub_ = node_.advertise<phantom_ros::PhantomVisionMeasurement>(vision_measurement_topic_name, 10);
      dynm_calibration_output_pub_ = node_.advertise<phantom_ros::PhantomVisionDynamicCalibrationOutputList>(dynm_cal_output_topic_name, 10);
      fusa_output_pub_ = node_.advertise<phantom_ros::FunctionalSafetyOutput>(fusa_output_topic_name, 10);
      camera_extra_info_frame_pub_ = node_.advertise<phantom_ros::PhantomCameraExtraInfoFrame>(cam_ext_frame_topic_name, 10);
      hba_output_pub_ = node_.advertise<phantom_ros::PhantomVisionHBA>(hba_output_topic_name, 10);
      aeb_output_pub_ = node_.advertise<phantom_ros::PhantomVisionAEB>(aeb_output_topic_name, 10);

      for (uint32_t i = 0; i < max_can_chans; i++)
      {
        // ordered by channel number then rx,tx
        std::string can_topic_name = can_topic_prefix_name + "/can" + std::to_string(i / 2) + "/" + (i % 2 ? "tx" : "rx");
        can_pub_[i] = node_.advertise<phantom_ros::Can>(can_topic_name, 5);
        can_seq_num_[i] = 0;
      }

      if (mode == "client")
      {
        PHANTOM_LOG("Starting receiver");
        PHANTOM_LOG("Server IP {}, port {}", server_ip, port);
        ti_handle_ = embeddedNetStart(server_ip.c_str(), server_binding_ip.c_str(), port, NET_TCP, connect_timeout, read_timeout);
        is_client_ = true;
      }
      else
      {
        PHANTOM_LOG("Starting receiver in host mode");
        PHANTOM_LOG("Server IP {}, port {}", server_binding_ip, port);
        ti_handle_ = embeddedServerNetInit(server_binding_ip.c_str(), port, NET_TCP, read_timeout);
        is_client_ = false;
      }

      if (ti_handle_ == nullptr)
      {
        throw phantom_ai::PhantomAIException("Receiver failed to initialize");
      }

      // Register the nodelet's set function for the health monitor status with the camera's status setter.
      health_monitor_.registerNodeletHealthStatusMutator(std::bind(&PhantomRosNodelet::setHealthStatus,
        this, std::placeholders::_1, std::placeholders::_2));

      frame_cnt_ = 0;
      error_cnt_ = 0;
      stat_print_interval_sec_ = 5.0;
      last_time_ = phantom_ai::TimeStamp::Now();

      is_running_ = true;

      data_cap_thread_ = std::thread(&embeddedCaptureNodelet::captureLoop, this);
      board_ip = getBoardIPFromPC();
      time_server_thread_ = std::thread(&embeddedCaptureNodelet::startTimer, this, board_ip, BOARD_TIME_PORT);
      time_server_thread_.detach();

      auto handle = data_cap_thread_.native_handle();
      pthread_setname_np(handle,"EmbeddedCap");
    }

    ros::NodeHandle node_;
    void *ti_handle_;
    Embedded_Frame frame_;

    std::string topic_name_prefix_;
    std::vector<std::shared_ptr<EmbeddedImageObj>> embedded_img_objs_;

    ros::Publisher esp_measurements_pub_;
    ros::Publisher wheel_speed_pub_;
    ros::Publisher gear_state_pub_;
    ros::Publisher vision_measurement_pub_;
    ros::Publisher dynm_calibration_output_pub_;
    ros::Publisher fusa_output_pub_;
    ros::Publisher camera_extra_info_frame_pub_;
    ros::Publisher hba_output_pub_;
    ros::Publisher aeb_output_pub_;

    ros::Publisher can_pub_[max_can_chans];
    uint32_t can_seq_num_[max_can_chans];

    // Cropping vars
    bool enable_cropping_;
    cv::Rect crop_win_;
    cv::Size output_cam_image_size_;

    bool is_client_;
    std::string output_format_;
    std::thread data_cap_thread_;
    std::thread time_server_thread_;
    bool is_running_;
    uint32_t counter_;
    bool is_annotated_;

    // Stats
    uint32_t frame_cnt_;
    uint32_t error_cnt_;
    double stat_print_interval_sec_;
    phantom_ai::TimeStamp last_time_;

    // Object used for reporting health status to nodelet for publishing.
    phantom_ai::ProcessHealthMonitor health_monitor_;

    // To show vision visualization images
    std::mutex viz_data_mutex_;
    std::thread viz_show_thread_;
    cv::Mat viz_data_;
    bool viz_show_inited_ = false;
    bool publish_vision_viz_topic_ = false;
    bool show_vision_debug_viz_image_ = true;
    std::string vision_viz_topic_name_;
    float vision_viz_window_scale_factor_;
    std::shared_ptr<EmbeddedImageObj> vision_debug_viz_obj_;
#ifdef USE_EXTRA_INPUT_HISTOGRAM
    //histogram is not implemented yet
    std::vector<std::shared_ptr<EmbeddedImageObj>> camera_extra_info_histogram_objs_;
#endif
    std::vector<std::shared_ptr<EmbeddedImageObj>> camera_extra_info_h3a_objs_;
    std::vector<std::shared_ptr<EmbeddedImageObj>> camera_extra_info_raw_objs_;
    const char* board_ip;

    // enable generate sub images default as false
    bool generate_images_{false};

  };
} //namespace

PHANTOM_ROS_NODELET_EXPORT_CLASS(phantom_ros::embeddedCaptureNodelet)
