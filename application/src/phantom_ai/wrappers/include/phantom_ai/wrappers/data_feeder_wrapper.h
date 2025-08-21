/*******************************************************************************
* @file    data_feeder_wrapper.h
* @date    02/14/2023
*
* @attention Copyright (c) 2023
* @attention Phantom AI, Inc.
* @attention All rights reserved.
*******************************************************************************/

#ifndef DATA_FEEDER_WRAPPER_H_
#define DATA_FEEDER_WRAPPER_H_

#include <opencv2/opencv.hpp>
#include "phantom_ai/core/time_stamp.h"
#include "phantom_ai/embedded_network/embedded_phantom_net.h"

namespace phantom_ai
{
  class DataFeeder
  {
  public:
    DataFeeder();
    ~DataFeeder();

    void onInit (pid_t mainpid, std::string& prosbag_data_dir, std::string filename = "data_feeder.yaml");
    void registerReadCallback(std::function<void(const Embedded_Frame&)> callback_func);

    Embedded_Frame frame_;

  private:
    void getPhantomRosbagFilelist(const std::string& base_dir, std::vector<std::string>* file_list_ptr);
    int32_t ReadHeader(FILE* fp, EmbeddedNet_PhantomHeader* pHeader);
    int32_t VerifyHeader(EmbeddedNet_PhantomHeader* pHeader);
    int32_t ReadData(FILE* fp, Embedded_Frame* frame);
    int32_t ReadFrame(FILE* fp, Embedded_Frame* frame);
    
    void readLoop();
    void updateStats();

    bool cfg_is_read_repeat_;
    uint32_t cfg_frame_rate_;
    std::string cfg_prosbag_data_dir_;

    pid_t mainpid_;
    bool is_running_;
    std::thread data_feeder_thread_;

    uint32_t frame_cnt_;
    std::function<void(const Embedded_Frame&)> read_frame_callback_;
    double stat_print_interval_sec_;
    phantom_ai::TimeStamp last_time_;
  }; // class DataFeeder

}  // namespace phantom_ai

#endif //DATA_FEEDER_WRAPPER_H_
