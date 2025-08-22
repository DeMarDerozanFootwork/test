/*
 *
 * Copyright (c) 2020 Texas Instruments Incorporated
 *
 * All rights reserved not granted herein.
 *
 * Limited License.
 *
 * Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive
 * license under copyrights and patents it now or hereafter owns or controls to make,
 * have made, use, import, offer to sell and sell ("Utilize") this software subject to the
 * terms herein.  With respect to the foregoing patent license, such license is granted
 * solely to the extent that any such patent is necessary to Utilize the software alone.
 * The patent license shall not apply to any combinations which include this software,
 * other than combinations with devices manufactured by or for TI ("TI Devices").
 * No hardware patent is licensed hereunder.
 *
 * Redistributions must preserve existing copyright notices and reproduce this license
 * (including the above copyright notice and the disclaimer and (if applicable) source
 * code license limitations below) in the documentation and/or other materials provided
 * with the distribution
 *
 * Redistribution and use in binary form, without modification, are permitted provided
 * that the following conditions are met:
 *
 * *       No reverse engineering, decompilation, or disassembly of this software is
 * permitted with respect to any software provided in binary form.
 *
 * *       any redistribution and use are licensed by TI for use only with TI Devices.
 *
 * *       Nothing shall obligate TI to provide you with source code for the software
 * licensed and provided to you in object code.
 *
 * If software source code is provided to you, modification and redistribution of the
 * source code are permitted provided that the following conditions are met:
 *
 * *       any redistribution and use of the source code, including any resulting derivative
 * works, are licensed by TI for use only with TI Devices.
 *
 * *       any redistribution and use of any object code compiled from the source code
 * and any resulting derivative works, are licensed by TI for use only with TI Devices.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of its suppliers
 *
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * DISCLAIMER.
 *
 * THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <utils/perf_stats/include/app_perf_stats.h>
#include <VX/vx_khr_pipelining.h>
#include <semaphore.h>
#include <sys/time.h>
#include <arm_neon.h>

#include "app_common.h"
#include <utils/timer/include/app_timer.h>
#include "app_sensor_module.h"
#include "app_capture_module.h"
#include "app_viss_module.h"
#include "app_aewb_module.h"
#include "app_ldc_module.h"
#include "app_img_mosaic_module.h"
#include "hal_camera_iface.h"
#include "mcam_app.h"
#include "multi_cam_tivx.h"

#include <TI/video_io_kernels.h>

#define APP_PIPELINE_DEPTH   (7)

#define TIOVX_MODULES_MAX_REF_HANDLES     (16u)

// Only for debugging, need to add app_display_module.c
//#define ENABLE_DISPLAY

#ifdef ENABLE_DISPLAY
#include <utils/draw2d/include/draw2d.h>
#include "app_display_module.h"
#include <utils/grpx/include/app_grpx.h>
#endif

#define SENSOR_OUT_FORMAT_RAW 0
#define SENSOR_OUT_FORMAT_YUV 1

// Enable TI CPU and memory usage statiatics dump to console
//#define ENABLE_DEBUG_STATISTICS

/* Order is depedent on results on camera enumeration query, sample output with default SDK drivers:
   269.194335 s: ISS: Enumerating sensors ... found 0 : IMX390-UB953_D3
   269.194356 s: ISS: Enumerating sensors ... found 1 : IMX390-UB953-DISCOVERY-NARROW
   269.194365 s: ISS: Enumerating sensors ... found 2 : IMX390-UB953-DISCOVERY-MID
   269.194373 s: ISS: Enumerating sensors ... found 3 : IMX390-UB953-DISCOVERY-WIDE
   269.194380 s: ISS: Enumerating sensors ... found 4 : AR0233-UB953_MARS
   269.194387 s: ISS: Enumerating sensors ... found 5 : AR0820-UB953_LI
   269.194394 s: ISS: Enumerating sensors ... found 6 : UB9xxx_RAW12_TESTPATTERN
   269.194401 s: ISS: Enumerating sensors ... found 7 : UB96x_UYVY_TESTPATTERN
   269.194408 s: ISS: Enumerating sensors ... found 8 : GW_AR0233_UYVY

*/
#define TI_CAM_IMX390_UB953_D3 0
#define TI_CAM_IMX390_UB953_DISCOVERY_NARROW 1
#define TI_CAM_IMX390_UB953_DISCOVERY_MID 2
#define TI_CAM_IMX390_UB953_DISCOVERY_WIDE 3
#define TI_CAM_AR0233_UB953_MARS 4
#define TI_CAM_AR0820_UB953_LI 5
#define TI_CAM_UB9XXX_RAW12_TESTPATTERN 6
#define TI_CAM_UB96X_UYVY_TESTPATTERN 7
#define TI_CAM_GW_AR0233_UYVY 8

#define kUpperBoundTimestampInUsec (8019803577000000ULL) // 2224 year
#define kLowerBoundTimestampInUsec (4389177000000ULL)    // 1970 year
#define STARTUP_TIME  (CAPTURE_FPS * 3)   // FPS * 3 sec

static EncoderFeedFrameCallback g_encoder_callback = NULL;
static uint32_t g_mosaic_width = 0;
static uint32_t g_mosaic_height = 0;

static uint64_t delta_timestamp_;
static vx_bool fullres_supported;

static vx_status app_init(McamAppObj *obj);
static void app_deinit(McamAppObj *obj);
static vx_status app_create_graph(McamAppObj *obj);
static vx_status app_verify_graph(McamAppObj *obj);
static void app_delete_graph(McamAppObj *obj);
static void app_default_param_set(McamAppObj *obj);
static void app_update_param_set(McamAppObj *obj, Camera_Params* param_array, int count);
static void app_pipeline_params_defaults(McamAppObj *obj);
static void add_graph_parameter_by_node_index(vx_graph graph, vx_node node, vx_uint32 node_parameter_index);
static void set_img_mosaic_params(ImgMosaicObj *imgMosaicObj, Camera_Params* param_array, vx_int32 numCh);
static vx_status app_run_graph_for_one_frame_pipeline(McamAppObj *obj, vx_int32 frame_id, char **output_array, uint64_t *timestamp);

static void app_set_cfg_default(McamAppObj *obj)
{
  snprintf(obj->captureObj.output_file_path,APP_MAX_FILE_PATH, ".");
  snprintf(obj->vissObj.output_file_path,APP_MAX_FILE_PATH, ".");
  snprintf(obj->ldcObj.output_file_path,APP_MAX_FILE_PATH, ".");

  obj->captureObj.en_out_capture_write = 0;
  obj->vissObj.en_out_viss_write = 0;
  obj->ldcObj.en_out_ldc_write = 0;

  obj->num_frames_to_write = 0;
  obj->num_frames_to_skip = 0;
}

static void app_parse_cfg_file(McamAppObj *obj)
{
  // hard code from app_multi_cam.cfg
  obj->sensorObj.sensor_index = TI_CAM_IMX390_UB953_D3;
  obj->num_frames_to_run = 1000000;
  obj->captureObj.enable_error_detection = 1;
  obj->sensorObj.enable_ldc = 0;
  obj->captureObj.en_out_capture_write = 0;
  obj->vissObj.en_out_viss_write = 0;
  obj->ldcObj.en_out_ldc_write = 0;
  strcpy(obj->captureObj.output_file_path, "/opt/vision_apps/app_cam_out");
  strcpy(obj->vissObj.output_file_path, "/opt/vision_apps/app_cam_out");
  strcpy(obj->ldcObj.output_file_path, "/opt/vision_apps/app_cam_out");
  strcpy(obj->output_file_path, "/opt/vision_apps/app_cam_out");
#ifdef ENABLE_DISPLAY
  obj->displayObj.display_option = 1;
#endif
  obj->is_interactive = 1;
  obj->sensorObj.is_interactive = obj->is_interactive;
  obj->sensorObj.num_cameras_enabled = 1;
  //obj->sensorObj.usecase_option = 0;
  obj->num_frames_to_write = 1;
  obj->num_frames_to_skip = 0;
  obj->captureObj.test_mode = 0;
}

vx_status setup_cross_graph_buffer_sharing(vx_reference* src_ref,  vx_reference* dest_ref)
{
    vx_status status = VX_SUCCESS;

    if (!src_ref || !dest_ref)
    {
        printf("Cannot setup buffer sharing - buffer refs not initialized\n");
        return VX_FAILURE;
    }

    void *src_buf_addr[TIOVX_MODULES_MAX_REF_HANDLES] = {NULL};
    vx_uint32 src_buf_size[TIOVX_MODULES_MAX_REF_HANDLES], num_src_buf_entries;

    status = tivxReferenceExportHandle((vx_reference)*src_ref,
                                   src_buf_addr, src_buf_size,
                                   TIOVX_MODULES_MAX_REF_HANDLES,
                                   &num_src_buf_entries);
    if(status != VX_SUCCESS)
    {
        printf("Error exporting src_ref output handles\n");
        return status;
    }

    // Check metadata compatibility between src_buf output and sink_buf input
    if (vx_false_e == tivxIsReferenceMetaFormatEqual(
            (vx_reference)*src_ref,
            (vx_reference)*dest_ref))
    {
        printf("Metadata format mismatch between src_ref and dest_ref\n");
        return VX_FAILURE;
    }

    // Import src_buf handles into sink_buf input buffer
    status = tivxReferenceImportHandle(*dest_ref,
                                   (const void **)src_buf_addr,
                                   src_buf_size,
                                   num_src_buf_entries);
    if(status != VX_SUCCESS)
    {
        printf("Error importing handles to sink_buf input\n");
        return status;
    }

    return VX_SUCCESS;
}

static vx_status cleanup_cross_graph_buffer_sharing(vx_reference* ref)
{
    vx_status status = VX_SUCCESS;
    void *temp_addr[TIOVX_MODULES_MAX_REF_HANDLES] = {NULL};
    vx_uint32 temp_size[TIOVX_MODULES_MAX_REF_HANDLES] = {0};
    vx_uint32 num_entries = 0;

    if (!ref)
    {
        printf("reference not initialized\n");
        return VX_FAILURE;
    }

    status = tivxReferenceExportHandle(*ref, temp_addr, temp_size, TIOVX_MODULES_MAX_REF_HANDLES, &num_entries);
    if(status != VX_SUCCESS || num_entries == 0)
    {
        printf("Error exporting handle count from reference or no handles found\n");
        return status;
    }

    // Fill each entry with NULL
    for(vx_uint32 i = 0; i < num_entries; i++)
    {
        temp_addr[i] = NULL;
        temp_size[i] = 0;
    }

    status = tivxReferenceImportHandle(*ref, (const void **)temp_addr, temp_size, num_entries);
    if(status != VX_SUCCESS)
    {
        printf("Error cleaning up handles from reference\n");
        return status;
    }

    printf("Cross-graph buffer sharing cleaned up successfully\n");

    return VX_SUCCESS;
}

uint64_t getLocalTimeInUsec()
{
  uint64_t timeInUsecs = 0;
  struct timeval tv;

  if (gettimeofday(&tv, NULL) < 0)
  {
    timeInUsecs = 0;
  }
  else
  {
    timeInUsecs = tv.tv_sec * 1000000ull + tv.tv_usec;
  }

  return (timeInUsecs);
}

void initialize_delta_timestamp()
{
  // the first chance
  uint64_t local_time = getLocalTimeInUsec(); // Linux timestamp

  if (local_time < (uint64_t)kLowerBoundTimestampInUsec || local_time > (uint64_t)kUpperBoundTimestampInUsec)
  {
    appLogWaitMsecs(200);
    // the second chance
    local_time = getLocalTimeInUsec();        // Linux timestamp
  }

  if (local_time < (uint64_t)kLowerBoundTimestampInUsec || local_time > (uint64_t)kUpperBoundTimestampInUsec)
  {
    printf("local_time: %llu\n", (unsigned long long)local_time);
    printf("local_time in initialize_delta_timestamp is out of range.\n");
    // sorry, no third chance
    exit(EXIT_FAILURE);
  }

  uint64_t global_time = appLogGetGlobalTimeInUsec(); // GTC based timestamp
  delta_timestamp_ = local_time - global_time;

  APP_PRINTF("start initialize_delta_timestamp\n");
  APP_PRINTF("local_time: %llu\n", (unsigned long long)local_time);
  APP_PRINTF("global_time: %llu\n", (unsigned long long)global_time);
  APP_PRINTF("end initialize_delta_timestamp\n");
}

uint64_t query_capture_timestamp(vx_reference* ref)
{
  /* We can now release the entry for VISS to be reused, now that output_arr_info has been filled */
  uint64_t prev_timestamp = 0;
  uint64_t timestamp = 0;
  static uint8_t wait_cnt = 0;
  if(STARTUP_TIME < wait_cnt)
  {
    wait_cnt = STARTUP_TIME;
  }
  else
  {
    wait_cnt++;
  }

  vx_status status = vxQueryReference((vx_reference)*ref, TIVX_REFERENCE_TIMESTAMP, &timestamp, sizeof(uint64_t));
  if (status != VX_SUCCESS)
  {
    printf("Error getting capture timestamp. Timestamps may be inaccurate!\n");
  }

  // local_timestamp = global_timestamp + delta_timestamp_
  // local_timestamp = global_timestamp + (local_timestap - global_timestamp)
  // local_timestamp = local_timestamp
  timestamp = timestamp + delta_timestamp_;

  // rollover happens after 3 seconds
  if((prev_timestamp > timestamp || 0 == timestamp) && (STARTUP_TIME <= wait_cnt))
  {
    printf("prev_timestamp: %llu\n", (unsigned long long)prev_timestamp);
    printf("timestamp: %llu\n", (unsigned long long)timestamp);
    APP_PRINTF("###########################\n");
    APP_PRINTF("###########################\n");
    APP_PRINTF("timestamp rollover happened\n");
    APP_PRINTF("The app will not stop.\n");
    APP_PRINTF("###########################\n");
    APP_PRINTF("###########################\n");
  }

  return timestamp;
}

void* tivx_multi_cam_init(Camera_Params* param_array, int count)
{
  vx_status status = VX_SUCCESS;

  McamAppObj *obj = (McamAppObj*)tivxMemAlloc(sizeof(McamAppObj), TIVX_MEM_EXTERNAL);

  /*Optional parameter setting*/
  app_default_param_set(obj);

  /*Config parameter reading*/
  app_set_cfg_default(obj);
  app_parse_cfg_file(obj);

      // override
  obj->sensorObj.num_cameras_enabled = count;
  fullres_supported = 0;
  if(param_array[0].camera_type == CAM_AR0820)
  {
    obj->sensorObj.sensor_index = TI_CAM_AR0820_UB953_LI;
    fullres_supported = 1;
  }
  else if (param_array[0].camera_type == CAM_IMX390)
  {
    obj->sensorObj.sensor_index = TI_CAM_IMX390_UB953_D3;
    fullres_supported = 1;
  }
  else if (param_array[0].camera_type == CAM_AR0233_GW5200)
  {
    obj->sensorObj.sensor_index = TI_CAM_GW_AR0233_UYVY;
  }
  else
  {
    printf("Unknown camera type %d\n", param_array[0].camera_type);
    assert(0); // game over
  }

  /* Querry sensor parameters */
  status = app_querry_sensor(&obj->sensorObj);
  if(SENSOR_OUT_FORMAT_YUV == obj->sensorObj.sensor_out_format)
  {
    printf("YUV Input selected. VISS, AEWB nodes will be bypassed. \n");
    obj->enable_viss = 0;
    obj->enable_aewb = 0;

    // Must use mosaic for generating downscale and crop images, however
    // MSC cannot work on YUV422, so enable(!) LDC to do the YUV422 to NV12 conversion
    printf("Forcing mosaic/LDC on for UYVY->NV12 conversion\n");
    obj->enable_mosaic = 1;
    obj->sensorObj.enable_ldc = 1;
  }
  else
  {
    obj->enable_viss = 1;
    obj->enable_aewb = 1;
    obj->enable_mosaic = 1;
  }

  /*Update of parameters are config file read*/
  app_update_param_set(obj, param_array, count);

  if (status == VX_SUCCESS)
  {
    status = app_init(obj);
  }
  if(status == VX_SUCCESS)
  {
    APP_PRINTF("App Init Done! \n");

    status = app_create_graph(obj);

    if(status == VX_SUCCESS)
    {
      APP_PRINTF("App Create Graph Done! \n");

      status = app_verify_graph(obj);

      if(status == VX_SUCCESS)
      {
        APP_PRINTF("App Verify Graph Done! \n");

        SensorObj *sensorObj = &obj->sensorObj;

        app_pipeline_params_defaults(obj);
        APP_PRINTF("app_pipeline_params_defaults returned\n");

        if(NULL == sensorObj->sensor_name)
        {
          printf("sensor name is NULL \n");
          return NULL;
        }

        status = appStartImageSensor(sensorObj->sensor_name, ((1 << sensorObj->num_cameras_enabled) - 1));
        APP_PRINTF("appStartImageSensor returned with status: %d\n", status);
      }
    }
  }

  initialize_delta_timestamp();

  return (void*)obj;
}

vx_int32 tivx_multi_cam_process(void *handle, char **buffer_array, uint32_t *frame_number, uint64_t *timestamp)
{
  if (handle == NULL)
  {
    printf("Invalid handle\n");
    return -1;
  }
  McamAppObj* obj = (McamAppObj*)handle;

  vx_status status;

  *frame_number = obj->frame_cnt;

  status = app_run_graph_for_one_frame_pipeline(obj, obj->frame_cnt++, buffer_array, timestamp);

  return (status == VX_SUCCESS) ? 0 : -1;
}

vx_int32 tivx_multi_cam_delete(void **handle_ptr)
{
  vx_status status;
  vx_reference ref;

  if (handle_ptr == NULL || *handle_ptr == NULL)
  {
    // already destroyed?
    return 0;
  }
  McamAppObj* obj = (McamAppObj*)*handle_ptr;

  SensorObj *sensorObj = &obj->sensorObj;

  obj->stop_task = 1;

  if (g_encoder_callback != NULL)
  {
    tivx_multi_cam_register_encoder_callback(NULL);
  }

  printf("Calling app_delete_graph\n");
  app_delete_graph(obj);

  printf("App Delete Graph Done! \n");

  status = appStopImageSensor(obj->sensorObj.sensor_name, ((1 << sensorObj->num_cameras_enabled) - 1));

  ref = (vx_reference)vxGetObjectArrayItem(obj->vissObj.raw_image_arr[0], 0);
  cleanup_cross_graph_buffer_sharing((vx_reference *)&ref);
  vxReleaseReference(&ref);

  ref = (vx_reference)vxGetObjectArrayItem(obj->imgMosaicObj.input_arr[0], 0);
  cleanup_cross_graph_buffer_sharing((vx_reference *)&ref);
  vxReleaseReference(&ref);

  app_deinit(obj);

  APP_PRINTF("App De-init Done! \n");

  tivxMemFree((void*)obj, sizeof(McamAppObj), TIVX_MEM_EXTERNAL);

  return status;
}

uint8_t tivx_get_camera_capture_fps()
{
#if CAPTURE_FPS == 30
  return 30;
#elif CAPTURE_FPS == 15
  return 15;
#else
  #error "Bad frame rate"
#endif
}

static vx_status app_init(McamAppObj *obj)
{
  vx_status status = VX_SUCCESS;

  /* Create OpenVx Context for Capture */
  obj->capture_context = vxCreateContext();
  status = vxGetStatus((vx_reference)obj->capture_context);
  APP_PRINTF("Creating capture context done!\n");
  if (status == VX_SUCCESS)
  {
    tivxHwaLoadKernels(obj->capture_context);
    tivxVideoIOLoadKernels(obj->capture_context);
    tivxImagingLoadKernels(obj->capture_context);
    APP_PRINTF("Kernel loading done!\n");
  }

  /* Create OpenVx Context for VISS */
  obj->viss_context = vxCreateContext();
  status = vxGetStatus((vx_reference)obj->viss_context);
  APP_PRINTF("Creating viss context done!\n");
  if (status == VX_SUCCESS)
  {
    tivxHwaLoadKernels(obj->viss_context);
    tivxVideoIOLoadKernels(obj->viss_context);
    tivxImagingLoadKernels(obj->viss_context);
    APP_PRINTF("Kernel loading done!\n");
  }

  /* Create OpenVx Context for mosaic */
  obj->mosaic_context = vxCreateContext();
  status = vxGetStatus((vx_reference)obj->mosaic_context);
  APP_PRINTF("Creating mosaic context done!\n");
  if (status == VX_SUCCESS)
  {
    tivxHwaLoadKernels(obj->mosaic_context);
    tivxVideoIOLoadKernels(obj->mosaic_context);
    tivxImagingLoadKernels(obj->mosaic_context);
    APP_PRINTF("Kernel loading done!\n");
  }

  /* Initialize modules */
  if (status == VX_SUCCESS)
  {
    status = app_init_sensor(&obj->sensorObj, "sensor_obj");
  }

  if (status == VX_SUCCESS)
  {
    APP_PRINTF("Sensor init done!\n");
    status = app_init_capture(obj->capture_context, &obj->captureObj, &obj->sensorObj, "capture_obj", APP_BUFFER_Q_DEPTH);
  }

  if((1 == obj->enable_viss) && (status == VX_SUCCESS))
  {
    status = app_init_viss(obj->viss_context, &obj->vissObj, &obj->sensorObj, "viss_obj", APP_BUFFER_Q_DEPTH);
    APP_PRINTF("VISS init done!\n");
  }

  if((1 == obj->enable_aewb) && (status == VX_SUCCESS))
  {
    status = app_init_aewb(obj->viss_context, &obj->aewbObj, &obj->sensorObj, "aewb_obj");
    APP_PRINTF("AEWB init done!\n");
  }

  if((obj->sensorObj.enable_ldc == 1) && (status == VX_SUCCESS))
  {
    status = app_init_ldc(obj->mosaic_context, &obj->ldcObj, &obj->sensorObj, "ldc_obj");
    APP_PRINTF("LDC init done!\n");
  }


  if((obj->enable_mosaic == 1) && (status == VX_SUCCESS))
  {
    status = app_init_img_mosaic(obj->mosaic_context, &obj->imgMosaicObj, &obj->sensorObj, "img_mosaic_obj", APP_BUFFER_Q_DEPTH);
    APP_PRINTF("Img Mosaic init done!\n");
  }

  if (status == VX_SUCCESS)
  {
#ifdef ENABLE_DISPLAY
    status = app_init_display(obj->context, &obj->displayObj, "display_obj");
    APP_PRINTF("Display init done!\n");

    app_grpx_init_prms_t grpx_prms;
    appGrpxInitParamsInit(&grpx_prms, obj->context);
    appGrpxInit(&grpx_prms);
#endif
  }

  appPerfPointSetName(&obj->total_perf , "TOTAL");
  appPerfPointSetName(&obj->fileio_perf, "FILEIO");
  return status;
}

static void app_deinit(McamAppObj *obj)
{
  app_deinit_sensor(&obj->sensorObj);
  APP_PRINTF("Sensor deinit done!\n");

  app_deinit_capture(&obj->captureObj, APP_BUFFER_Q_DEPTH);
  APP_PRINTF("Capture deinit done!\n");

  if(1 == obj->enable_viss)
  {
    app_deinit_viss(&obj->vissObj, &obj->sensorObj, APP_BUFFER_Q_DEPTH);
    APP_PRINTF("VISS deinit done!\n");
  }

  if(1 == obj->enable_aewb)
  {
    app_deinit_aewb(&obj->aewbObj);
    APP_PRINTF("AEWB deinit done!\n");
  }

  if(obj->sensorObj.enable_ldc == 1)
  {
    app_deinit_ldc(&obj->ldcObj);
    APP_PRINTF("LDC deinit done!\n");
  }

  if(obj->enable_mosaic == 1)
  {
    app_deinit_img_mosaic(&obj->imgMosaicObj, APP_BUFFER_Q_DEPTH);
    APP_PRINTF("Img Mosaic deinit done!\n");
  }

#ifdef ENABLE_DISPLAY
  app_deinit_display(&obj->displayObj);
  APP_PRINTF("Display deinit done!\n");
#endif

  tivxVideoIOUnLoadKernels(obj->capture_context);
  tivxHwaUnLoadKernels(obj->capture_context);
  tivxImagingUnLoadKernels(obj->capture_context);

  tivxVideoIOUnLoadKernels(obj->viss_context);
  tivxHwaUnLoadKernels(obj->viss_context);
  tivxImagingUnLoadKernels(obj->viss_context);

  tivxVideoIOUnLoadKernels(obj->mosaic_context);
  tivxHwaUnLoadKernels(obj->mosaic_context);
  tivxImagingUnLoadKernels(obj->mosaic_context);

  APP_PRINTF("Kernels unload done!\n");

  vxReleaseContext(&obj->capture_context);
  vxReleaseContext(&obj->viss_context);
  vxReleaseContext(&obj->mosaic_context);
  APP_PRINTF("Release context done!\n");
}

static void app_delete_graph(McamAppObj *obj)
{
  app_stop_capture(&obj->captureObj);
  printf("Capture stop done!\n");

  app_stop_viss(&obj->vissObj);
  printf("VISS stop done!\n");

  app_stop_img_mosaic(&obj->imgMosaicObj);
  printf("Img Mosaic stop done!\n");

  // enqueue a random reference to unblock the capture thread if necessary
  vx_object_array* raw_image_arr = (vx_object_array*)RingBuffer_getStorage(obj->captureObj.raw_image_arr);
  vxGraphParameterEnqueueReadyRef(obj->capture_graph, obj->captureObj.graph_parameter_index,
    (vx_reference*)&raw_image_arr[0], 1);

  app_join_capture(&obj->captureObj);
  printf("Capture join done!\n");

  app_join_viss(&obj->vissObj);
  printf("VISS join done!\n");

  app_join_img_mosaic(&obj->imgMosaicObj);
  printf("Img Mosaic join done!\n");

  vxWaitGraph(obj->capture_graph);
  vxWaitGraph(obj->viss_graph);
  vxWaitGraph(obj->mosaic_graph);

  app_delete_capture(&obj->captureObj);
  APP_PRINTF("Capture delete done!\n");

  app_delete_viss(&obj->vissObj);
  APP_PRINTF("VISS delete done!\n");

  app_delete_aewb(&obj->aewbObj);
  APP_PRINTF("AEWB delete done!\n");

  if(obj->sensorObj.enable_ldc == 1)
  {
    app_delete_ldc(&obj->ldcObj);
    APP_PRINTF("LDC delete done!\n");
  }

  app_delete_img_mosaic(&obj->imgMosaicObj);
  APP_PRINTF("Img Mosaic delete done!\n");

#ifdef ENABLE_DISPLAY
  app_delete_display(&obj->displayObj);
  APP_PRINTF("Display delete done!\n");
#endif

  vxReleaseGraph(&obj->capture_graph);
  vxReleaseGraph(&obj->viss_graph);
  vxReleaseGraph(&obj->mosaic_graph);
  APP_PRINTF("Graph delete done!\n");
}

static vx_status app_create_graph(McamAppObj *obj)
{
  vx_status status = VX_SUCCESS;
  vx_graph_parameter_queue_params_t capture_graph_parameters_queue_params_list[1];
  vx_graph_parameter_queue_params_t viss_graph_parameters_queue_params_list[2];
  vx_graph_parameter_queue_params_t mosaic_graph_parameters_queue_params_list[2];
  vx_int32 capture_graph_parameter_index, viss_graph_parameter_index, mosaic_graph_parameter_index;

  obj->capture_graph = vxCreateGraph(obj->capture_context);
  status = vxGetStatus((vx_reference)obj->capture_graph);
  if (status == VX_SUCCESS)
  {
    status = vxSetReferenceName((vx_reference)obj->capture_graph, "app_multi_cam_capture_graph");
    APP_PRINTF("Capture Graph create done!\n");
  }

  if(status == VX_SUCCESS)
  {
    status = app_create_graph_capture(obj->capture_graph, &obj->captureObj);
    APP_PRINTF("Capture graph done!\n");
  }

  obj->viss_graph = vxCreateGraph(obj->viss_context);
  status = vxGetStatus((vx_reference)obj->viss_graph);
  if (status == VX_SUCCESS)
  {
    status = vxSetReferenceName((vx_reference)obj->viss_graph, "app_multi_cam_viss_graph");
    APP_PRINTF("VISS Graph create done!\n");
  }

  if(1 == obj->enable_viss)
  {
    if(status == VX_SUCCESS)
    {
      status = app_create_graph_viss(obj->viss_graph, &obj->vissObj, obj, obj->vissObj.raw_image_arr[0]);
      APP_PRINTF("VISS graph done!\n");
    }
  }

  if(1 == obj->enable_aewb)
  {
    if(status == VX_SUCCESS)
    {
      status = app_create_graph_aewb(obj->viss_graph, &obj->aewbObj, obj->vissObj.h3a_stats_arr);

      APP_PRINTF("AEWB graph done!\n");
    }
  }

  if (status == VX_SUCCESS)
  {
    obj->mosaic_graph = vxCreateGraph(obj->mosaic_context);
    status = vxGetStatus((vx_reference)obj->mosaic_graph);
    if (status == VX_SUCCESS)
    {
      status = vxSetReferenceName((vx_reference)obj->mosaic_graph, "app_multi_cam_mosaic_graph");
      APP_PRINTF("Mosaic Graph create done!\n");
    }
  }

  vx_int32 idx = 0;
  if(obj->sensorObj.enable_ldc == 1)
  {
    vx_object_array ldc_in_arr;
    if(1 == obj->enable_viss)
    {
      vx_object_array* output_arr = (vx_object_array*)RingBuffer_getStorage(obj->vissObj.output_arr);
      ldc_in_arr = output_arr[0];
    }
    else
    {
      vx_object_array* raw_image_arr = (vx_object_array*)RingBuffer_getStorage(obj->captureObj.raw_image_arr);
      ldc_in_arr = raw_image_arr[0];
    }
    if (status == VX_SUCCESS)
    {
      status = app_create_graph_ldc(obj->mosaic_graph, &obj->ldcObj, ldc_in_arr);
      APP_PRINTF("LDC graph done!\n");
    }
    obj->imgMosaicObj.input_arr[idx++] = obj->ldcObj.output_arr;
  }
  else
  {
    idx++;
  }

  vx_image display_in_image;
  if(obj->enable_mosaic == 1)
  {
    obj->imgMosaicObj.num_inputs = idx;

    if(status == VX_SUCCESS)
    {
      status = app_create_graph_img_mosaic(obj->mosaic_graph, &obj->imgMosaicObj, obj);
      APP_PRINTF("Img Mosaic graph done!\n");
    }
    vx_image* output_image = (vx_image*)RingBuffer_getStorage(obj->imgMosaicObj.output_image);
    display_in_image = output_image[0];
  }
  else
  {
    vx_object_array* raw_image_arr = (vx_object_array*)RingBuffer_getStorage(obj->captureObj.raw_image_arr);
    display_in_image = (vx_image)vxGetObjectArrayItem(raw_image_arr[0], 0);
  }

#ifdef ENABLE_DISPLAY
  if(status == VX_SUCCESS)
  {
    status = app_create_graph_display(obj->mosaic_graph, &obj->displayObj, display_in_image);
    APP_PRINTF("Display graph done!\n");
  }
#else
  (void)display_in_image;
#endif

  if(status == VX_SUCCESS)
  {
    capture_graph_parameter_index = 0;
    add_graph_parameter_by_node_index(obj->capture_graph, obj->captureObj.node, 1);
    obj->captureObj.graph_parameter_index = capture_graph_parameter_index;
    capture_graph_parameters_queue_params_list[capture_graph_parameter_index].graph_parameter_index = capture_graph_parameter_index;
    capture_graph_parameters_queue_params_list[capture_graph_parameter_index].refs_list_size = APP_BUFFER_Q_DEPTH;
    vx_object_array* raw_image_arr = (vx_object_array*)RingBuffer_getStorage(obj->captureObj.raw_image_arr);
    capture_graph_parameters_queue_params_list[capture_graph_parameter_index].refs_list = (vx_reference*)&raw_image_arr[0];
    capture_graph_parameter_index++;

    viss_graph_parameter_index = 0;
    add_graph_parameter_by_node_index(obj->viss_graph, obj->vissObj.node, 3);   //raw
    obj->vissObj.graph_parameter_index[viss_graph_parameter_index] = viss_graph_parameter_index;
    viss_graph_parameters_queue_params_list[viss_graph_parameter_index].graph_parameter_index = viss_graph_parameter_index;
    viss_graph_parameters_queue_params_list[viss_graph_parameter_index].refs_list_size = 1;
    viss_graph_parameters_queue_params_list[viss_graph_parameter_index].refs_list = (vx_reference*)&obj->vissObj.raw_image_arr[0];
    viss_graph_parameter_index++;

    add_graph_parameter_by_node_index(obj->viss_graph, obj->vissObj.node, 6);   //output[2]
    obj->vissObj.graph_parameter_index[viss_graph_parameter_index] = viss_graph_parameter_index;
    viss_graph_parameters_queue_params_list[viss_graph_parameter_index].graph_parameter_index = viss_graph_parameter_index;
    viss_graph_parameters_queue_params_list[viss_graph_parameter_index].refs_list_size = HAL_CAM_FULLRES_RECENT_IMAGE_POOL_SIZE;
    vx_object_array* output_arr = (vx_object_array*)RingBuffer_getStorage(obj->vissObj.output_arr);
    viss_graph_parameters_queue_params_list[viss_graph_parameter_index].refs_list = (vx_reference*)&output_arr[0];
    viss_graph_parameter_index++;

    mosaic_graph_parameter_index = 0;
    if (1)
    {
      add_graph_parameter_by_node_index(obj->mosaic_graph, obj->imgMosaicObj.node, 3);
      obj->imgMosaicObj.graph_parameter_index[mosaic_graph_parameter_index] = mosaic_graph_parameter_index;
      mosaic_graph_parameters_queue_params_list[mosaic_graph_parameter_index].graph_parameter_index = mosaic_graph_parameter_index;
      mosaic_graph_parameters_queue_params_list[mosaic_graph_parameter_index].refs_list_size = 1;
      mosaic_graph_parameters_queue_params_list[mosaic_graph_parameter_index].refs_list = (vx_reference*)&obj->imgMosaicObj.input_arr[0];
      mosaic_graph_parameter_index++;

      add_graph_parameter_by_node_index(obj->mosaic_graph, obj->imgMosaicObj.node, 1);
      obj->imgMosaicObj.graph_parameter_index[mosaic_graph_parameter_index] = mosaic_graph_parameter_index;
      mosaic_graph_parameters_queue_params_list[mosaic_graph_parameter_index].graph_parameter_index = mosaic_graph_parameter_index;
      mosaic_graph_parameters_queue_params_list[mosaic_graph_parameter_index].refs_list_size = APP_BUFFER_Q_DEPTH;
      vx_image* output_image = (vx_image*)RingBuffer_getStorage(obj->imgMosaicObj.output_image);
      mosaic_graph_parameters_queue_params_list[mosaic_graph_parameter_index].refs_list = (vx_reference*)&output_image[0];
      mosaic_graph_parameter_index++;
    }

    status = vxSetGraphScheduleConfig(obj->capture_graph,
      VX_GRAPH_SCHEDULE_MODE_QUEUE_AUTO,
      capture_graph_parameter_index,
      capture_graph_parameters_queue_params_list);
      printf("done capture vxSetGraphScheduleConfig\n");
    status = vxSetGraphScheduleConfig(obj->viss_graph,
      VX_GRAPH_SCHEDULE_MODE_QUEUE_AUTO,
      viss_graph_parameter_index,
      viss_graph_parameters_queue_params_list);
      printf("done viss vxSetGraphScheduleConfig\n");
    status = vxSetGraphScheduleConfig(obj->mosaic_graph,
      VX_GRAPH_SCHEDULE_MODE_QUEUE_AUTO,
      mosaic_graph_parameter_index,
      mosaic_graph_parameters_queue_params_list);
      printf("done mosaic vxSetGraphScheduleConfig\n");

    if (status == VX_SUCCESS)
    {
      status = tivxSetGraphPipelineDepth(obj->capture_graph, APP_PIPELINE_DEPTH);
    }
    if (status == VX_SUCCESS)
    {
      status = tivxSetGraphPipelineDepth(obj->viss_graph, 1);
    }
    if (status == VX_SUCCESS)
    {
      status = tivxSetGraphPipelineDepth(obj->mosaic_graph, 1);
    }
    if((obj->enable_viss == 1) && (status == VX_SUCCESS))
    {
      if (status == VX_SUCCESS)
      {
        status = tivxSetNodeParameterNumBufByIndex(obj->vissObj.node, 9, APP_BUFFER_Q_DEPTH);  // h3a_stats
      }
      if ((status == VX_SUCCESS) && (1 == obj->enable_aewb))
      {
        status = tivxSetNodeParameterNumBufByIndex(obj->aewbObj.node, 4, APP_BUFFER_Q_DEPTH);  // aewb_output
      }
    }
    if((obj->sensorObj.enable_ldc == 1) && (status == VX_SUCCESS))
    {
      status = tivxSetNodeParameterNumBufByIndex(obj->ldcObj.node, 7, APP_BUFFER_Q_DEPTH);    // output_img
    }
    if(status == VX_SUCCESS)
    {
      APP_PRINTF("Pipeline params setup done!\n");
    }

    APP_PRINTF("app_create_graph_done!\n");
  }

  return status;
}

static vx_status app_verify_graph(McamAppObj *obj)
{
  vx_status status = VX_SUCCESS;

  status = vxVerifyGraph(obj->capture_graph);
  if(status == VX_SUCCESS)
  {
    status = vxVerifyGraph(obj->viss_graph);
  }

  if(status == VX_SUCCESS)
  {
    status = vxVerifyGraph(obj->mosaic_graph);
  }

  if(status == VX_SUCCESS)
  {
    APP_PRINTF("Graph verify done!\n");
  }

#if 0
  if(VX_SUCCESS == status)
  {
    status = tivxExportGraphToDot(obj->graph,".", "vx_app_multi_cam");
  }
#endif

  if ((obj->captureObj.enable_error_detection) && (status == VX_SUCCESS))
  {
    status = app_send_error_frame(&obj->captureObj);
  }

/* wait a while for prints to flush */
  tivxTaskWaitMsecs(100);

  return status;
}

static void copyMosaicImage(ImgMosaicObj *imgMosaicObj, vx_image src_img, char **output_array)
{
  vx_status status;
  vx_imagepatch_addressing_t image_addr;
  vx_map_id map_id;
  void * data_ptr = NULL;
  vx_uint32 mosaic_width = 0;
  vx_uint32 mosaic_height = 0;
  vx_df_image image_format;
  int idx;

  status = vxQueryImage(src_img, VX_IMAGE_WIDTH, &mosaic_width, sizeof(vx_uint32));
  if (status != VX_SUCCESS)
  {
    printf("invalid vx_image...\n");
    return;
  }
  vxQueryImage(src_img, VX_IMAGE_HEIGHT, &mosaic_height, sizeof(vx_uint32));
  vxQueryImage(src_img, VX_IMAGE_FORMAT, &image_format, sizeof(vx_df_image));

  if (mosaic_width == 0 || image_format != VX_DF_IMAGE_NV12)
  {
    printf("Invalid vx_image..\n");
    return;
  }
  int y_size;
  int uv_size;

  vx_rectangle_t rect;
  rect.start_x = 0;
  rect.start_y = 0;
  rect.end_x = mosaic_width;
  rect.end_y = mosaic_height;

  status = vxMapImagePatch(src_img,
    &rect,
    0,
    &map_id,
    &image_addr,
    &data_ptr,
    VX_READ_AND_WRITE,
    VX_MEMORY_TYPE_HOST,
    VX_NOGAP_X);

  if (status != VX_SUCCESS || data_ptr == NULL)
  {
    printf("Failed patch\n");
  }

  // break down each image and copy to appropriate buffer
  for (idx = 0; idx < imgMosaicObj->params.num_windows; idx++)
  {
    char *src;
    char *dst = output_array[idx]; // Y Buf

    if (dst == NULL)
    {
      printf("Buf %d is NULL!\n", idx);
      continue;
    }

    // get location inside mosaic
    uint32_t start_x = imgMosaicObj->params.windows[idx].startX;
    uint32_t start_y = imgMosaicObj->params.windows[idx].startY;
    uint32_t width = imgMosaicObj->params.windows[idx].width;
    uint32_t height = imgMosaicObj->params.windows[idx].height;

    src = (char*)data_ptr + (start_y * image_addr.stride_y) + start_x;

    if (image_addr.stride_y == mosaic_width && image_addr.stride_y == width)
    {
      y_size = width * height; // 8 bit Y
      memcpy(dst, src, y_size); //copy Y
    }
    else
    {
      int i;
      // must copy line by line
      for (i = 0; i < height; i++)
      {
        memcpy(dst, src, width);
        dst += width;
        src += image_addr.stride_y;
      }
    }
  }

  vxUnmapImagePatch(src_img, map_id);

  rect.start_x = 0;
  rect.start_y = 0;
  rect.end_x = mosaic_width;
  rect.end_y = mosaic_height / 2;

  status = vxMapImagePatch(src_img,
    &rect,
    1,
    &map_id,
    &image_addr,
    &data_ptr,
    VX_READ_AND_WRITE,
    VX_MEMORY_TYPE_HOST,
    VX_NOGAP_X);

  if (status != VX_SUCCESS || data_ptr == NULL)
  {
    printf("Failed patch2\n");
  }

  // break down each image and copy to appropriate buffer
  for (idx = 0; idx < imgMosaicObj->params.num_windows; idx++)
  {
    char *dst = output_array[idx]; // Y Buf
    char *src;
    if (dst == NULL)
    {
      printf("Buf %d is NULL!\n", idx);
      continue;
    }

    // get location inside mosaic
    uint32_t start_x = imgMosaicObj->params.windows[idx].startX;
    uint32_t start_y = imgMosaicObj->params.windows[idx].startY;
    uint32_t width = imgMosaicObj->params.windows[idx].width;
    uint32_t height = imgMosaicObj->params.windows[idx].height;

    src = (char*)data_ptr + (start_y * image_addr.stride_y / 2) + start_x;
    y_size = width * height; // 8 bit Y
    dst += y_size;

    if (image_addr.stride_y == mosaic_width && image_addr.stride_y == width)
    {
      uv_size = width * height / 2;
      memcpy(dst, src, uv_size); //copy UV
    }
    else
    {
      int i;
      // must copy line by line
      for (i = 0; i < height / 2; i++)
      {
        memcpy(dst, src, width);
        dst += width;
        src += image_addr.stride_y;
      }
    }
  }

  vxUnmapImagePatch(src_img, map_id);
}

static vx_status app_run_graph_for_one_frame_pipeline(McamAppObj *obj, vx_int32 frame_id, char **output_array, uint64_t *timestamp)
{
  vx_status status = VX_SUCCESS;

  appPerfPointBegin(&obj->total_perf);

  ImgMosaicObj *imgMosaicObj = &obj->imgMosaicObj;
  VISSObj *vissObj = &obj->vissObj;
  CaptureObj *captureObj = &obj->captureObj;

  vx_image* mosaic_output_image = NULL;
  uint64_t capture_timestamp;
  static int32_t pipeline = -APP_BUFFER_Q_DEPTH + 1;
  static int32_t enqueueCnt = 0;
  static int32_t full_res_image_idx = 0;

  // Signal capture thread
  if (sem_post(&captureObj->start_sem) != 0) {
    printf("app_run_graph_for_one_frame_pipeline: sem_post failed");
    return VX_FAILURE;
  }

  if (pipeline >= 0)
  {
    if (status == VX_SUCCESS)
    {
      if (RingBuffer_dequeue(imgMosaicObj->output_image, (void**)&mosaic_output_image) != 0)
      {
          printf("RingBuffer_dequeue failed on output image\n");
          status = VX_FAILURE;
      }
    }

    if (status == VX_SUCCESS)
    {
      if (g_encoder_callback != NULL)
        {
          bool enc_result = g_encoder_callback(*mosaic_output_image, frame_id);
          if (!enc_result)
          {
            printf("Registered encoder callback failed\n");
          }
        }
      // copy mosaic output to user buffer
      copyMosaicImage(imgMosaicObj, *mosaic_output_image, output_array);
    }

    if (status == VX_SUCCESS)
    {
      if (RingBuffer_releaseOldest(imgMosaicObj->output_image) != 0)
      {
          printf("RingBuffer_releaseOldest failed on output image\n");
          status = VX_FAILURE;
      }
    }

    capture_timestamp = captureObj->capture_timestamps[enqueueCnt];

    enqueueCnt++;
    enqueueCnt = (enqueueCnt  >= APP_BUFFER_Q_DEPTH)? 0 : enqueueCnt;
  
    vissObj->output_arr_info[full_res_image_idx].frame_id = frame_id;
    vissObj->output_arr_info[full_res_image_idx].timestamp_us = capture_timestamp;

    if (timestamp != NULL)
    {
      *timestamp = vissObj->output_arr_info[full_res_image_idx].timestamp_us; 
    }

    // Synchronize the enqueue of capture graph references to the application
    if (status == VX_SUCCESS)
    {
      status = vxGraphParameterEnqueueReadyRef(obj->capture_graph, captureObj->graph_parameter_index,
          (vx_reference*)&(vissObj->output_arr_info[full_res_image_idx].capture_input_arr), 1);
    }

    full_res_image_idx = (++full_res_image_idx >= HAL_CAM_FULLRES_RECENT_IMAGE_POOL_SIZE)? 0 : full_res_image_idx;

    if (status == VX_SUCCESS)
    {
      if (RingBuffer_releaseOldest(vissObj->output_arr) != 0)
      {
          printf("RingBuffer_releaseOldest failed on viss output arr\n");
          status = VX_FAILURE;
      }
    }
  }
  else
  {
    if (timestamp != NULL)
    {
      *timestamp = getLocalTimeInUsec();
    }
    pipeline++;
  }

#ifdef ENABLE_DEBUG_STATISTICS
  {
    vx_int32 interval_secs = 5; // every 5 sec
    vx_int32 fps = 30; // assume 30hz
    if (frame_id % (fps*interval_secs) == 0)
    {
      printf("[%llu] frame %d\n", (unsigned long long)*timestamp, frame_id); // assume *timestamp exists
      appPerfStatsPrintAll();
      status = tivx_utils_graph_perf_print(obj->capture_graph);
      status = tivx_utils_graph_perf_print(obj->viss_graph);
      status = tivx_utils_graph_perf_print(obj->mosaic_graph);
      appPerfPointPrint(&obj->total_perf);
      appPerfPointReset(&obj->total_perf);
      printf("\n");
    }
  }
#endif

  appPerfPointEnd(&obj->total_perf);
  return status;
}

#ifdef ENABLE_DISPLAY
static void set_display_defaults(DisplayObj *displayObj)
{
  displayObj->display_option = 1;
}
#endif

static void app_pipeline_params_defaults(McamAppObj *obj)
{
  obj->pipeline       = -APP_BUFFER_Q_DEPTH + 1;
  obj->enqueueCnt     = 0;
  obj->dequeueCnt     = 0;
}

static void set_sensor_defaults(SensorObj *sensorObj)
{
  strcpy(sensorObj->sensor_name, SENSOR_SONY_IMX390_UB953_D3);

  sensorObj->num_sensors_found = 0;
  sensorObj->sensor_features_enabled = 0;
  sensorObj->sensor_features_supported = 0;
  sensorObj->sensor_dcc_enabled = 0;
  sensorObj->sensor_wdr_enabled = 0;
  sensorObj->sensor_exp_control_enabled = 0;
  sensorObj->sensor_gain_control_enabled = 0;
}

static void app_default_param_set(McamAppObj *obj)
{
  set_sensor_defaults(&obj->sensorObj);

#ifdef ENABLE_DISPLAY
  set_display_defaults(&obj->displayObj);
#endif

  app_pipeline_params_defaults(obj);

  obj->is_interactive = 1;
  obj->write_file = 0;

  obj->sensorObj.enable_ldc = 0;
  obj->sensorObj.num_cameras_enabled = 1;
//  obj->sensorObj.usecase_option = APP_SENSOR_FEATURE_CFG_UC1; // 60fps WDR mode
  obj->sensorObj.usecase_option = APP_SENSOR_FEATURE_CFG_UC0; // 30fps WDR mode
}

void tivx_multi_cam_register_encoder_callback(EncoderFeedFrameCallback callback)
{
    g_encoder_callback = callback;
}

void tivx_multi_cam_get_mosaic_dimensions(uint32_t* width, uint32_t* height)
{
    if (width) *width = g_mosaic_width;
    if (height) *height = g_mosaic_height;
}

static void set_img_mosaic_params(ImgMosaicObj *imgMosaicObj, Camera_Params* param_array, vx_int32 numCh)
{
  vx_int32 idx, ch, j;
  vx_int32 max_width = 0;
  vx_int32 max_height = 0;

  // find max width and heights
  for (ch = 0; ch < numCh; ch++)
  {
    for (j = 0; j < param_array[ch].num_subimages; j++)
    {
      if (param_array[ch].subimage[j].output_width > max_width)
      {
        max_width = param_array[ch].subimage[j].output_width;
      }
      if (param_array[ch].subimage[j].output_height > max_height)
      {
        max_height = param_array[ch].subimage[j].output_height;
      }
    }
  }
  printf("Max output size %dx%d\n", max_width, max_height);

  idx = 0;

  tivxImgMosaicParamsSetDefaults(&imgMosaicObj->params);

  // need way to keep track of which index maps to which areas of mosaic
  // each MSC output is mapped to vertical arragement of images, e.g Front Main, Front Crop, Side Left, Side Right.
  for (ch = 0; ch < numCh; ch++)
  {
    for (j = 0; j < param_array[ch].num_subimages; j++)
    {
      imgMosaicObj->params.windows[idx].startX  = 0;
      imgMosaicObj->params.windows[idx].startY  = max_height*idx;
      imgMosaicObj->params.windows[idx].width   = param_array[ch].subimage[j].output_width;
      imgMosaicObj->params.windows[idx].height  = param_array[ch].subimage[j].output_height;
      imgMosaicObj->params.windows[idx].enable_roi = 1;
      imgMosaicObj->params.windows[idx].roiStartX = param_array[ch].subimage[j].start_x;
      imgMosaicObj->params.windows[idx].roiStartY = param_array[ch].subimage[j].start_y;
      imgMosaicObj->params.windows[idx].roiWidth  = param_array[ch].subimage[j].width;
      imgMosaicObj->params.windows[idx].roiHeight = param_array[ch].subimage[j].height;
      imgMosaicObj->params.windows[idx].input_select   = 0;
      imgMosaicObj->params.windows[idx].channel_select = ch;
      printf("[%d][%d] ROI %dx%d, %dx%d\n", ch, j, imgMosaicObj->params.windows[idx].roiStartX, imgMosaicObj->params.windows[idx].roiStartY, imgMosaicObj->params.windows[idx].roiWidth, imgMosaicObj->params.windows[idx].roiHeight);
      printf("[%d][%d] Output location %dx%d, %dx%d\n",ch, j, imgMosaicObj->params.windows[idx].startX, imgMosaicObj->params.windows[idx].startY, imgMosaicObj->params.windows[idx].width, imgMosaicObj->params.windows[idx].height);
      idx++;
    }
  }

  // make a 1xN mosaic
  imgMosaicObj->out_width    = max_width;
  imgMosaicObj->out_height   = max_height * idx;
  printf("Mosaic output %dx%d, cams %d, numWin %d\n", imgMosaicObj->out_width, imgMosaicObj->out_height, numCh, idx);
  imgMosaicObj->num_inputs   = 1;

  imgMosaicObj->params.num_windows  = idx;

  /* Number of time to clear the output buffer before it gets reused */
  imgMosaicObj->params.clear_count  = APP_BUFFER_Q_DEPTH;
  //imgMosaicObj->params.enable_overlay = 0;

  g_mosaic_width  = (uint32_t)imgMosaicObj->out_width;
  g_mosaic_height = (uint32_t)imgMosaicObj->out_height;
}

static void app_update_param_set(McamAppObj *obj, Camera_Params* param_array, int count)
{
  //vx_uint16 resized_width, resized_height;
  //appIssGetResizeParams(obj->sensorObj.image_width, obj->sensorObj.image_height, DISPLAY_WIDTH, DISPLAY_HEIGHT, &resized_width, &resized_height);

  if (count < obj->sensorObj.num_cameras_enabled)
  {
    // must have at least equal number of configs
    printf("Not enough Camera_Params, %d != %d\n", count, obj->sensorObj.num_cameras_enabled);
    assert(0);
  }

  set_img_mosaic_params(&obj->imgMosaicObj, param_array, obj->sensorObj.num_cameras_enabled);
}

/*
* Utility API used to add a graph parameter from a node, node parameter index
*/
static void add_graph_parameter_by_node_index(vx_graph graph, vx_node node, vx_uint32 node_parameter_index)
{
  vx_parameter parameter = vxGetParameterByIndex(node, node_parameter_index);

  vxAddParameterToGraph(graph, parameter);
  vxReleaseParameter(&parameter);
}

/* 
    validate_fullres_nv12_image
*/
static vx_status validate_fullres_nv12image(vx_image fullres_img, McamAppObj* handle)
{
    vx_status status = VX_SUCCESS;
    vx_df_image format = 0;
    SensorObj* sensorObj = &handle->sensorObj;

    // Query the image format
    status = vxQueryImage(fullres_img, VX_IMAGE_FORMAT, &format, sizeof(format));
    if (status != VX_SUCCESS)
    {
        printf("Failed to query image format: status %d\n", status);
        return status;
    }

    // Check if the image format is VX_DF_IMAGE_NV12
    if (format != VX_DF_IMAGE_NV12)
    {
        printf("Unsupported image format: %d, expected VX_DF_IMAGE_NV12.\n", format);
        return VX_ERROR_INVALID_FORMAT;
    }

    //Query the image width and height
    vx_uint32 img_width = 0, img_height = 0;
    status = vxQueryImage(fullres_img, VX_IMAGE_WIDTH, &img_width, sizeof(img_width));
    if (status != VX_SUCCESS)
    {
        printf("Failed to query image width: status %d\n", status);
        return status;
    }
    status = vxQueryImage(fullres_img, VX_IMAGE_HEIGHT, &img_height, sizeof(img_height));
    if (status != VX_SUCCESS)
    {
        printf("Failed to query image height: status %d\n", status);
        return status;
    }

    // Check if the image size is equal to the full resolution image size
    if (img_width != sensorObj->image_width || img_height != sensorObj->image_height)
    {
        printf("Invalid image dimensions: width %u, height %u. Expected width %u, height %u.\n",
                   img_width, img_height, sensorObj->image_width, sensorObj->image_height);
        return VX_ERROR_INVALID_FORMAT;
    }

    return status;
}

/* 
    extract_roi_from_fullres_nv12image
*/
static vx_status extract_roi_from_fullres_nv12image(vx_image fullres_img, vx_rectangle_t *rect, uint8_t *roi_img_out, McamAppObj* handle)
{
    vx_status status = VX_SUCCESS;
    SensorObj* sensorObj = &handle->sensorObj;

    // Validate the full resolution image format only once
    static bool is_fullres_image_validated = false;
    if (!is_fullres_image_validated)
    {
        status = validate_fullres_nv12image(fullres_img, handle);
        if (status != VX_SUCCESS)
        {
            printf("Image validation failed: status=%d\n", status);
            return status;
        }
        is_fullres_image_validated = true;
    }

    // Check if the ROI start and end coordinates are valid
    if (rect->start_x >= rect->end_x || rect->start_y >= rect->end_y)
    {
        printf("ROI coordinates are reversed or invalid: start_x=%d, end_x=%d, start_y=%d, end_y=%d",
                            rect->start_x, rect->end_x, rect->start_y, rect->end_y);
        return VX_ERROR_INVALID_PARAMETERS;
    }

    // Check if the requested ROI lies within the fullres image bounds
    if (rect->end_x > sensorObj->image_width || rect->end_y > sensorObj->image_height)
    {
        printf("Requested ROI is out of fullres image bounds: rect->end_x=%d, rect->end_y=%d, fullres_width=%d, fullres_height=%d",
                            rect->end_x, rect->end_y, sensorObj->image_width, sensorObj->image_height);
        return VX_ERROR_INVALID_PARAMETERS;
    }

    // Check if the x and y coordinates are even
    if ((rect->start_x & 0x1) != 0x0 || (rect->start_y & 0x1) != 0x0 || (rect->end_x & 0x1) != 0x0 || (rect->end_y & 0x1) != 0x0)
    {
        printf("start_x=%d, start_y=%d, end_x=%d, end_y=%d must be even.\n", rect->start_x, rect->start_y, rect->end_x, rect->end_y);
        return VX_ERROR_INVALID_PARAMETERS;
    }

    vx_uint32 roi_img_width = rect->end_x - rect->start_x;
    vx_uint32 roi_img_height = rect->end_y - rect->start_y;

    vx_imagepatch_addressing_t image_addr;
    void *data_ptr = NULL;
    vx_map_id map_id = 0;

    // Map the Y plane (plane index 0)
    status = vxMapImagePatch(fullres_img, rect, 0, &map_id, &image_addr, &data_ptr,
                             VX_READ_ONLY, VX_MEMORY_TYPE_HOST, VX_NOGAP_X);
    if (status != VX_SUCCESS || data_ptr == NULL)
    {
        printf("Failed to map Y plane, status: %d\n", status);
        return status;
    }

    // Copy Y data
    uint8_t *dstY = roi_img_out;
    for (vx_uint32 y = 0; y < roi_img_height; y++)
    {
        uint8_t *src_line_y = (uint8_t *)data_ptr + y * image_addr.stride_y;
        uint8_t *dst_line_y = dstY + y * roi_img_width;

        // Use NEON for bulk copy
        vx_uint32 x = 0;
        for (; x + 15 < roi_img_width; x += 16)
        {
            uint8x16_t data_y = vld1q_u8(src_line_y + x);
            vst1q_u8(dst_line_y + x, data_y);
        }

        // Handle any remaining bytes
        for (; x < roi_img_width; x++)
        {
            dst_line_y[x] = src_line_y[x];
        }
    }

    // Unmap the Y plane
    status = vxUnmapImagePatch(fullres_img, map_id);
    if (status != VX_SUCCESS)
    {
        printf("Failed to unmap Y plane, status: %d\n", status);
        return status;
    }

    // Map the UV plane (plane index 1)
    data_ptr = NULL;
    map_id = 0;

    status = vxMapImagePatch(fullres_img, rect, 1, &map_id, &image_addr, &data_ptr,
                             VX_READ_ONLY, VX_MEMORY_TYPE_HOST, VX_NOGAP_X);
    if (status != VX_SUCCESS || data_ptr == NULL)
    {
        printf("Failed to map UV plane, status: %d\n", status);
        return status;
    }

    // Copy UV data
    uint8_t *dstUV = roi_img_out + (roi_img_width * roi_img_height);
    vx_uint32 uv_line_bytes = roi_img_width;

    for (vx_uint32 y = 0; y < roi_img_height / 2; y++)
    {
        uint8_t *src_line_uv = (uint8_t *)data_ptr + y * image_addr.stride_y;
        uint8_t *dst_line_uv = dstUV + y * uv_line_bytes;

        // Use NEON for bulk copy
        vx_uint32 x = 0;
        for (; x + 15 < uv_line_bytes; x += 16)
        {
            uint8x16_t data_uv = vld1q_u8(src_line_uv + x);
            vst1q_u8(dst_line_uv + x, data_uv);
        }

        // Handle any remaining bytes
        for (; x < uv_line_bytes; x++)
        {
            dst_line_uv[x] = src_line_uv[x];
        }
    }

    // Unmap the UV plane
    status = vxUnmapImagePatch(fullres_img, map_id);
    if (status != VX_SUCCESS)
    {
        printf("Failed to unmap UV plane, status: %d\n", status);
        return status;
    }

    return VX_SUCCESS;
}

/* 
    tivx_camera_fetch_full_resolution_frame_roi
*/
int tivx_camera_fetch_full_resolution_frame_roi(void *handle, uint32_t frame_number, HAL_ROI *roi, HAL_Frame_Info *roi_frame_out)
{
  if ((handle == NULL) || (roi_frame_out == NULL))
  {
    printf("Invalid input pointers provided\n");
    return -1;
  }

  // Initialize context pointers
  McamAppObj *obj = handle;
  VISSObj *vissObj = &obj->vissObj;
  vx_int32 retrieved_count = 0;

  if (!fullres_supported) {
    printf("Not supported camera type for full resolution frame acqusition. \n");
    return -2;
  }

  pthread_mutex_lock(&vissObj->lock);
  int retrieved_index = -1;

  for (int i = 0; i < HAL_CAM_FULLRES_RECENT_IMAGE_POOL_SIZE; i++)
  {
    if (i != vissObj->current_output_idx)
    {
      if (vissObj->output_arr_info[i].frame_id == frame_number) {
        retrieved_index = i;
        break;
      }
    }
  }

  if (retrieved_index != -1)
  {
    retrieved_count = 1;
    vx_object_array* output_arr = (vx_object_array*)RingBuffer_getStorage(vissObj->output_arr);
    vx_reference img_handle = vxGetObjectArrayItem(output_arr[retrieved_index], 0);   //per tiovx_modules_allocate_bufpool
    vx_rectangle_t rect = {
      .start_x = roi->x,
      .start_y = roi->y,
      .end_x = roi->x + roi->width,
      .end_y = roi->y + roi->height
    };

    vx_status img_fetch_status = extract_roi_from_fullres_nv12image((vx_image)img_handle, &rect, roi_frame_out->data, handle);
    if (img_fetch_status != VX_SUCCESS)
    {
      retrieved_count = -1;
      printf("Failed(code=%d) to extract image.\n", img_fetch_status);
    }
    else
    {
      roi_frame_out->region = *roi;
      roi_frame_out->frame_count = vissObj->output_arr_info[retrieved_index].frame_id;
      roi_frame_out->timestamp_us = vissObj->output_arr_info[retrieved_index].timestamp_us;
    }

    vxReleaseReference(&img_handle);
  }
  else
  {
    memset(roi_frame_out, 0, sizeof(HAL_Frame_Info));
    APP_PRINTF("Full-res frame id %d not found\n", frame_number);
  }

  pthread_mutex_unlock(&vissObj->lock);

  return retrieved_count;
}
