/*
 *
 * Copyright (c) 2017 Texas Instruments Incorporated
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

#include "app_img_mosaic_module.h"
#include "mcam_app.h"

static void* img_mosaic_thread(void* arg);
extern vx_status setup_cross_graph_buffer_sharing(vx_reference* src_ref,  vx_reference* dest_ref);

vx_status app_init_img_mosaic(vx_context context, ImgMosaicObj *imgMosaicObj, SensorObj *sensorObj, char *objName, vx_int32 bufq_depth)
{
    vx_status status = VX_SUCCESS;
    vx_int32 q;

    imgMosaicObj->config = vxCreateUserDataObject(context, "ImgMosaicConfig", sizeof(tivxImgMosaicParams), NULL);
    status = vxGetStatus((vx_reference)imgMosaicObj->config);

    if(status == VX_SUCCESS)
    {
        status = vxCopyUserDataObject(imgMosaicObj->config, 0, sizeof(tivxImgMosaicParams),\
                    &imgMosaicObj->params, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);
    }

    imgMosaicObj->output_image = RingBuffer_create(APP_MAX_BUFQ_DEPTH, sizeof(vx_image));
    if (imgMosaicObj->output_image == NULL)
    {
        printf("failed to create RingBuffer\n");
        return VX_FAILURE;
    }

    vx_image* output_image = (vx_image*)RingBuffer_getStorage(imgMosaicObj->output_image);
    for(q = 0; q < bufq_depth; q++)
    {
        if(status == VX_SUCCESS)
        {
            output_image[q] = vxCreateImage(context, imgMosaicObj->out_width, imgMosaicObj->out_height, VX_DF_IMAGE_NV12);
            status = vxGetStatus((vx_reference)output_image[q]);
        }
    }

    if(status == VX_SUCCESS)
    {
        imgMosaicObj->kernel = tivxAddKernelImgMosaic(context, imgMosaicObj->num_inputs);
        status = vxGetStatus((vx_reference)imgMosaicObj->kernel);
    }

    vx_uint32 img_height, img_width;
    img_width  = sensorObj->sensorParams.sensorInfo.raw_params.width;
    img_height = sensorObj->sensorParams.sensorInfo.raw_params.height;
    vx_image input_img = vxCreateImage(context, img_width, img_height, VX_DF_IMAGE_NV12);
    for(q = 0; q < TIVX_IMG_MOSAIC_MAX_INPUTS; q++)
    {
      imgMosaicObj->input_arr[q] = vxCreateObjectArray(context, (vx_reference)input_img, sensorObj->num_cameras_enabled);
    }
    vxReleaseImage(&input_img);

    if (status == VX_SUCCESS) {
      imgMosaicObj->terminated = false;
    }

    return status;
}

void app_deinit_img_mosaic(ImgMosaicObj *imgMosaicObj, vx_int32 bufq_depth)
{
  vx_int32 q;

  vxReleaseUserDataObject(&imgMosaicObj->config);

  for(q = 0; q < TIVX_IMG_MOSAIC_MAX_INPUTS; q++)
  {
    vxReleaseObjectArray(&imgMosaicObj->input_arr[q]);
  }

  vx_image* output_image = (vx_image*)RingBuffer_getStorage(imgMosaicObj->output_image);
  for(q = 0; q < bufq_depth; q++)
  {
    vxReleaseImage(&output_image[q]);
  }

  if (imgMosaicObj->output_image) {
    RingBuffer_destroy(imgMosaicObj->output_image);
  }

  return;
}

void app_stop_img_mosaic(ImgMosaicObj *imgMosaicObj)
{
  // Terminate img mosaic thread
  imgMosaicObj->terminated = true;
  RingBuffer_stop(imgMosaicObj->output_image);
}

void app_delete_img_mosaic(ImgMosaicObj *imgMosaicObj)
{
  if(imgMosaicObj->node != NULL)
  {
    vxReleaseNode(&imgMosaicObj->node);
  }
  if(imgMosaicObj->kernel != NULL)
  {
    vxRemoveKernel(imgMosaicObj->kernel);
  }
  return;
}

void app_join_img_mosaic(ImgMosaicObj *imgMosaicObj)
{
  pthread_join(imgMosaicObj->thread, NULL);
}

vx_status app_create_graph_img_mosaic(vx_graph graph, ImgMosaicObj *imgMosaicObj, void* mcamAppObj)
{

  vx_status status = VX_SUCCESS;

  vx_image* output_image = (vx_image*)RingBuffer_getStorage(imgMosaicObj->output_image);
  imgMosaicObj->node = tivxImgMosaicNode(graph,
                                         imgMosaicObj->kernel,
                                         imgMosaicObj->config,
                                         output_image[0],
                                         NULL,
                                         imgMosaicObj->input_arr,
                                         imgMosaicObj->num_inputs);

  APP_ASSERT_VALID_REF(imgMosaicObj->node);
  #ifdef x86_64
  vxSetNodeTarget(imgMosaicObj->node, VX_TARGET_STRING, TIVX_TARGET_DSP1);
  #else
  vxSetNodeTarget(imgMosaicObj->node, VX_TARGET_STRING, TIVX_TARGET_VPAC_MSC1);
  #endif
  vxSetReferenceName((vx_reference)imgMosaicObj->node, "mosaic_node");
  status = vxGetStatus((vx_reference)imgMosaicObj->node);

  if (pthread_create(&imgMosaicObj->thread, NULL, img_mosaic_thread, mcamAppObj) != 0)
  {
    printf("Failed to create img mosaic thread\n");
    status = VX_FAILURE;
  }

  return (status);

}

vx_status writeMosaicOutput(char* file_name, vx_image out_img)
{
  vx_status status;

  status = vxGetStatus((vx_reference)out_img);

  if(status == VX_SUCCESS)
  {
    FILE * fp = fopen(file_name,"wb");

    if(fp == NULL)
    {
      printf("Unable to open file %s \n", file_name);
      return (VX_FAILURE);
    }

    {
      vx_rectangle_t rect;
      vx_imagepatch_addressing_t image_addr;
      vx_map_id map_id;
      void * data_ptr;
      vx_uint32  img_width;
      vx_uint32  img_height;
      vx_uint32  num_bytes;

      vxQueryImage(out_img, VX_IMAGE_WIDTH, &img_width, sizeof(vx_uint32));
      vxQueryImage(out_img, VX_IMAGE_HEIGHT, &img_height, sizeof(vx_uint32));

      rect.start_x = 0;
      rect.start_y = 0;
      rect.end_x = img_width;
      rect.end_y = img_height;
      status = vxMapImagePatch(out_img,
                               &rect,
                               0,
                               &map_id,
                               &image_addr,
                               &data_ptr,
                               VX_READ_ONLY,
                               VX_MEMORY_TYPE_HOST,
                               VX_NOGAP_X);

      //Copy Luma
      num_bytes = fwrite(data_ptr,1,img_width*img_height, fp);

      if(num_bytes != (img_width*img_height))
        printf("Luma bytes written = %d, expected = %d", num_bytes, img_width*img_height);

      vxUnmapImagePatch(out_img, map_id);

      status = vxMapImagePatch(out_img,
                               &rect,
                               1,
                               &map_id,
                               &image_addr,
                               &data_ptr,
                               VX_READ_ONLY,
                               VX_MEMORY_TYPE_HOST,
                               VX_NOGAP_X);


      //Copy CbCr
      num_bytes = fwrite(data_ptr,1,img_width*img_height/2, fp);

      if(num_bytes != (img_width*img_height/2))
        printf("CbCr bytes written = %d, expected = %d", num_bytes, img_width*img_height/2);

      vxUnmapImagePatch(out_img, map_id);

    }

    fclose(fp);
  }

  return(status);
}


static void* img_mosaic_thread(void* arg) {
  vx_status status = VX_SUCCESS;
  McamAppObj* obj = (McamAppObj*)arg;
  ImgMosaicObj *imgMosaicObj = &obj->imgMosaicObj;
  VISSObj *vissObj = &obj->vissObj;
  vx_object_array* viss_output_arr = NULL;
  vx_object_array ref = NULL;
  vx_image mosaic_output_image;
  uint32_t num_refs;
  int32_t enqueueCnt     = 0;
  int ret = 0;

  vx_image* output_image = (vx_image*)RingBuffer_getStorage(obj->imgMosaicObj.output_image);

  while (1) {
      if ((status == VX_FAILURE) || (imgMosaicObj->terminated))
      {
          pthread_exit(NULL);
      }

      if (status == VX_SUCCESS)
      {
          ret = RingBuffer_dequeue(vissObj->output_arr, (void**)&viss_output_arr);
          if (ret < 0)
          {
              printf("Img Mosaic thread: RingBuffer_dequeue failed\n");
              status = VX_FAILURE;
          }
          else if (ret == 1)
          {
              printf("Img Mosaic thread: RingBuffer_dequeue stopped\n");
              status = VX_FAILURE;
          }
      }

      if (status == VX_SUCCESS)
      {
          // Wait for output_arr not full
          ret = RingBuffer_waitForNotFull(imgMosaicObj->output_image);
          if (ret < 0)
          {
              printf("Img Mosaic thread: RingBuffer_waitForNotFull failed\n");
              status = VX_FAILURE;
          }
          else if (ret == 1)
          {
              printf("Img Mosaic thread: RingBuffer_waitForNotFull stopped\n");
              status = VX_FAILURE;
          }
      }

      // Execute Mosaic graph
      if (status == VX_SUCCESS)
      {
        status = vxGraphParameterEnqueueReadyRef(obj->mosaic_graph, imgMosaicObj->graph_parameter_index[OUTPUT_PARAMETER], (vx_reference*)&output_image[enqueueCnt], 1);
      }

      vx_reference src_ref = (vx_reference)vxGetObjectArrayItem(*viss_output_arr, 0);
      vx_reference dst_ref = (vx_reference)vxGetObjectArrayItem(imgMosaicObj->input_arr[0], 0);

      setup_cross_graph_buffer_sharing((vx_reference*)&src_ref, (vx_reference*)&dst_ref);
      vxReleaseReference(&src_ref);
      vxReleaseReference(&dst_ref);

      if (status == VX_SUCCESS)
      {
        status = vxGraphParameterEnqueueReadyRef(obj->mosaic_graph, imgMosaicObj->graph_parameter_index[INPUT_PARAMETER], (vx_reference*)&imgMosaicObj->input_arr[0], 1);
      }
      if (status == VX_SUCCESS)
      {
        status = vxGraphParameterDequeueDoneRef(obj->mosaic_graph,  imgMosaicObj->graph_parameter_index[INPUT_PARAMETER], (vx_reference*)&ref, 1, &num_refs);
      }
      /* Dequeue mosaic output */
      if (status == VX_SUCCESS)
      {
        status = vxGraphParameterDequeueDoneRef(obj->mosaic_graph, imgMosaicObj->graph_parameter_index[OUTPUT_PARAMETER], (vx_reference*)&mosaic_output_image, 1, &num_refs);
      }

      if (status == VX_SUCCESS)
      {
          /* enqueue into circular buffer */
          ret = RingBuffer_enqueue(imgMosaicObj->output_image);
          if (ret < 0)
          {
              printf("Img Mosaic thread: RingBuffer_enqueue failed\n");
              status = VX_FAILURE;
          }
          else if (ret == 1)
          {
              printf("Img Mosaic thread: RingBuffer_enqueue stopped\n");
              status = VX_FAILURE;
          }
      }

      enqueueCnt++;
      enqueueCnt = (enqueueCnt  >= APP_BUFFER_Q_DEPTH)? 0 : enqueueCnt;
  }
}
