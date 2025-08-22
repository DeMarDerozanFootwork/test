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

#ifndef MULTI_CAM_TIVX_H_
#define MULTI_CAM_TIVX_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <VX/vx.h>

typedef enum {
  CAM_IMX390,
  CAM_AR0820,
  CAM_AR0233_GW5200,
  CAM_OX08B40,
  CAM_OX03C10
} TIVX_CAMERA_TYPES;

/* CAMERA_STATUS represents error codes using bit masks. */
typedef enum {
  HIK_CAM_SUCCESS = 0x00u,       // No errors, operation successful
  HIK_CAM_DATALOSS_FAIL = 0x01u, // frameCnt does not increment (bit 0)
  HIK_CAM_TIMEOUT_FAIL = 0x02u,  // imageData is not available within time (bit 1)
  HIK_CAM_FRAME_RATE_LOW_FAIL = 0x04u,  // slow frame rate (bit 2)
  HIK_CAM_LATENCY_FAIL = 0x08u,  // camera pipeline(from capture to pass the image data to phantom-os-2) has latency (bit 4)
  HIK_CAM_INIT_FAIL = 0x10u,    // Initialization of the camera failed (bit 5)
  HIK_CAM_PROCESS_FAIL = 0x20u, // Processing or operation on the camera failed (bit 6)
  HIK_CAM_RELEASE_FAIL = 0x40u, // Releasing or closing the camera failed (bit 7)
  // Additional errors would continue in the format: HIK_CAM_ANOTHER_FAIL = 0xXu (bit X)
} CAMERA_STATUS;

typedef struct {
  uint32_t start_x; // based on original resolution
  uint32_t start_y; // based on original resolution
  uint32_t width; // based on original resolution
  uint32_t height; // based on original resolution
  uint32_t output_width; // scaled output width
  uint32_t output_height; // scaled output height
} Roi_Window;

#define TIVX_MAX_CROPS 6

typedef struct {
  TIVX_CAMERA_TYPES camera_type;
  uint32_t output_format;
  uint32_t framerate;
  uint32_t capture_framerate;
  uint32_t sensor_width;
  uint32_t sensor_height;
  uint32_t num_subimages;
  Roi_Window subimage[TIVX_MAX_CROPS];
} Camera_Params;

typedef bool (*EncoderFeedFrameCallback)(vx_image mosaic_image, uint32_t frame_number);


void* tivx_multi_cam_init(Camera_Params* param_array, int count);
int tivx_multi_cam_process(void* handle, char **buffer_array, uint32_t *frame_number, uint64_t *timestamp);
int tivx_multi_cam_delete(void** handle_ptr);
uint8_t tivx_get_camera_capture_fps();

void tivx_multi_cam_register_encoder_callback(EncoderFeedFrameCallback callback);
void tivx_multi_cam_get_mosaic_dimensions(uint32_t* width, uint32_t* height);

#endif //MULTI_CAM_TIVX_H_
