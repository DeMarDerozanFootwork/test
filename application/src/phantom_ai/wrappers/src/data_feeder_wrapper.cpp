/*********************************************************************
* @file    data_feeder_wrapper.cpp
* @date    02/15/2023
*
* @attention Copyright (c) 2023
* @attention Phantom AI, Inc.
* @attention All rights reserved.
*********************************************************************/

#include "phantom_ai/core/log.h"
#include "phantom_ai/core/yaml.h"
#include "phantom_ai/core/thread.h"
#include "phantom_ai/core/profiler.h"
#include "phantom_ai/wrappers/data_feeder_wrapper.h"
#include "phantom_ai/core/timer.h"
#include <dirent.h>
#include <signal.h>

namespace
{
// The thread scheduling priority.
// We consider 7 to be relatively low priority. This ensures that
// the data_feeder_wrapper does not block processing tasks that are
// on the critical path.
constexpr int kDataFeederPriority = 7;
// The range of the expected read frame rate, outside of which we show an error message.
constexpr double kMinReadFrameRate = 14.0;
constexpr double kMaxReadFrameRate = 16.0;
} // namespace

namespace phantom_ai
{
  DataFeeder::DataFeeder() :
    frame_{},
    is_running_(false),
    frame_cnt_(0),
    read_frame_callback_(nullptr),
    stat_print_interval_sec_(0.0),
    last_time_()
  {}

  DataFeeder::~DataFeeder() {
    is_running_ = false;
    if (data_feeder_thread_.joinable())
    {
      data_feeder_thread_.join();
    }

    if (frame_.data != nullptr)
    {
      delete [] frame_.data;
    }
  }

  void DataFeeder::registerReadCallback(std::function<void(const Embedded_Frame&)> callback_func)
  {
    read_frame_callback_ = callback_func;
  }

  void DataFeeder::onInit(pid_t mainpic, std::string& prosbag_data_dir, std::string filename)
  {
    mainpid_ = mainpic;
    cfg_prosbag_data_dir_ = prosbag_data_dir;

    // Load the config.
    phantom_ai::YamlNode cfg = phantom_ai::load_yaml_file("tools/data_feeder", filename);
    cfg_is_read_repeat_ = phantom_ai::get_yaml_value(cfg, "read_repeat").as<bool>();
    frame_.data_size = phantom_ai::get_yaml_value(cfg, "read_buffer_size").as<uint32_t>();
    cfg_frame_rate_ = phantom_ai::get_yaml_value(cfg, "frame_rate").as<uint32_t>();
    if((cfg_frame_rate_ < 1) || (cfg_frame_rate_ > 30))
    {
      PHANTOM_WARNING("Yaml config 'frame_rate_={}', It should be 0 < frame_rate_ <= 30, It is set as default={} ", cfg_frame_rate_, 10);
      cfg_frame_rate_ = 10;
    }

    // alloc data buffer
    frame_.data = new uint8_t[frame_.data_size];

    if (frame_.data == nullptr)
    {
      throw phantom_ai::PhantomAIException("alloc failed %d b", frame_.data_size);
    }

    stat_print_interval_sec_ = 5.0;
    last_time_ = phantom_ai::TimeStamp::Now();

    is_running_ = true;

    data_feeder_thread_ = std::thread(&DataFeeder::readLoop, this);
    phantom_ai::thread_set_name(data_feeder_thread_, "DataFeeder");
  }

// private
  void DataFeeder::updateStats()
  {
    frame_cnt_++;
    phantom_ai::TimeStamp now = phantom_ai::TimeStamp::Now();

    double delta_time = now.toSec() - last_time_.toSec();

    if (delta_time >= stat_print_interval_sec_)
    {
      double fps = (double)frame_cnt_ / delta_time;
      if ((fps < kMinReadFrameRate) || (fps > kMaxReadFrameRate))
      {
        PHANTOM_ERROR("Read fps: {:.2f}, frames: {}, delta: {:.3f} s. The frame rate is out of range. "
          "Please do not show this in a demo.", fps, frame_cnt_, delta_time);
      }
      else
      {
        PHANTOM_LOG("Read fps: {:.2f}, frames: {}, delta: {:.3f} s", fps, frame_cnt_, delta_time);
      }

      // reset counters
      last_time_ = now;
      frame_cnt_ = 0;
    }
  }

  inline int isPhantomPosbagFile(const struct dirent *dp)
  {
    if(strstr(dp->d_name, ".pbag") != nullptr)
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }

  void DataFeeder::getPhantomRosbagFilelist(const std::string& base_dir, std::vector<std::string>* file_list_ptr)
  {
    struct  dirent **namelist;

    int count = scandir(base_dir.c_str(), &namelist, isPhantomPosbagFile, alphasort);

    if(count == -1 || count == 0)
    {
      throw phantom_ai::PhantomAIException("Faild to scan '*.pbag' in \"{}\": {}", base_dir.c_str(), strerror(errno));
    }

    for(int i = 0; i < count; i++)
    {
        std::string file = base_dir + "/" + namelist[i]->d_name;
        file_list_ptr->push_back(file);
        //PHANTOM_LOG("{}",file.c_str() );
        free(namelist[i]);
    }

    free(namelist);
  }

  int32_t DataFeeder::VerifyHeader(EmbeddedNet_PhantomHeader* pHeader)
  {
    /* check if everything looks ok */
    if (pHeader->flag != EMBEDDED_HEADER_FLAG)
    {
      PHANTOM_ERROR("{}: Flag mismatch, 0x{:x} != 0x{:x}", __func__, pHeader->flag, EMBEDDED_HEADER_FLAG);
      return -1;
    }

    if (pHeader->version != EMBEDDED_NETWORK_VERSION)
    {
      PHANTOM_ERROR("{}: Version mismatch, 0x{:x} != 0x{:x}", __func__, pHeader->version, EMBEDDED_NETWORK_VERSION);
      return -1;
    }

    return 0;
  }

  int32_t DataFeeder::ReadHeader(FILE* fp, EmbeddedNet_PhantomHeader* pHeader)
  {
    int32_t status = 0;
    uint32_t dataSize = sizeof(EmbeddedNet_PhantomHeader);
    size_t bytesRead = fread((uint8_t*)pHeader, 1, dataSize, fp);
    
    if (bytesRead != dataSize)
    {
        PHANTOM_ERROR("{}: Failed to read header: Could only read {} bytes of {} bytes.",__func__, bytesRead, dataSize);
        return -1;
    }

    status = VerifyHeader(pHeader);
    if (status < 0)
    {
      PHANTOM_ERROR("{}: bad header", __func__);
      return status;
    }

    return status;
  }

  int32_t DataFeeder::ReadData(FILE* fp, Embedded_Frame* frame)
  {
    int32_t status = 0;
    uint32_t read_bytes = 0;
    uint32_t i, j;

    for (i = 0; i < frame->header.item_count; i++)
    {
      for (j = 0; j < MAX_IMG_PLANES; j++)
      {
        read_bytes += frame->header.item[i].buf_size[j];
        //PHANTOM_LOG("width:{} , height:{}", frame->header.item[i].width, frame->header.item[i].height);
      }
    }
    
    if (read_bytes > frame->data_size)
    {
      PHANTOM_ERROR("{} : User buffer too small for read, {}  < {} , aborting", __func__, frame->data_size, read_bytes);
      return -1;
    }

    size_t bytesRead = fread(frame->data, 1, read_bytes, fp);
    if (bytesRead != read_bytes)
    {
      PHANTOM_ERROR("{} : failed read: Could only read {}  bytes of {}  bytes", __func__,  bytesRead, read_bytes);
      return -1;
    }

    return status;
  }

  int32_t DataFeeder::ReadFrame(FILE* fp, Embedded_Frame* frame)
  {
    int32_t status = 0;

    status = ReadHeader(fp, &frame->header);

    if (status < 0)
    {
      PHANTOM_ERROR("{}: Failed to read header", __func__);
      fclose(fp);
      return status;
    }

    status = ReadData(fp, frame);

    if (status < 0)
    {
      PHANTOM_ERROR("{}: Failed to read data", __func__);
      fclose(fp);
      return status;
    }

    return status;
  }

  void DataFeeder::readLoop()
  {
    // polling loop....
    phantom_ai::Timer timer(cfg_frame_rate_, false);

    // Set a a real-time FIFO scheduling policy for the main thread.
    // The scheduling policy is also inherited by child threads,
    // but can be overridden.
    phantom_ai::thread_set_fifo_schedule(kDataFeederPriority);

    // Get Phantom Rosbag file list
    std::vector<std::string> prosbag_file_list;
    getPhantomRosbagFilelist(cfg_prosbag_data_dir_, &prosbag_file_list);

    // Selelect the 1st file. 
    std::vector<std::string>::iterator prosbag_file_iter = prosbag_file_list.begin();

    // Open the 1st file. 
    FILE *fp_prosbag = fopen((*prosbag_file_iter).c_str(), "rb");
    if (fp_prosbag == nullptr)
    {
      throw phantom_ai::PhantomAIException("Failed to open the file ({})",(*prosbag_file_iter).c_str());
    }

    while (is_running_)
    {
      timer.tic();
        
      // check if request to exit has occured
      if (!is_running_)
      {
        continue;
      }

      if (read_frame_callback_ != nullptr)
      {
          if (ReadFrame(fp_prosbag, &frame_) < 0)
          {
            throw phantom_ai::PhantomAIException("Error reading frame ({})",(*prosbag_file_iter).c_str());
          }

          read_frame_callback_(frame_);

          if (fgetc(fp_prosbag) == EOF)
          {
            fclose(fp_prosbag);

            //Select next the file.
            prosbag_file_iter++;
            if(prosbag_file_iter == prosbag_file_list.end())
            {
              if(cfg_is_read_repeat_ == true)
              {
                prosbag_file_iter = prosbag_file_list.begin(); // repeat
              }
              else
              {
                fp_prosbag = nullptr;
                break;
              }
            }

            //Opne the file.
            fp_prosbag = fopen((*prosbag_file_iter).c_str(), "rb");
            if (fp_prosbag == nullptr)
            {
              throw phantom_ai::PhantomAIException("Failed to open the file ({})",(*prosbag_file_iter).c_str());
            }
          }
          else
          {
            fseek(fp_prosbag, -1, SEEK_CUR); // move back 1;
          }
      }

      updateStats();

      timer.toc();
    }

    PHANTOM_LOG("Stopping datafeeder");

    if(fp_prosbag != nullptr)
    {
      fclose(fp_prosbag);
    }

    if (is_running_)
    {
      kill(mainpid_,SIGINT);
    }
  }
} // namespace phantom_ai
