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

#ifndef TIDL_TIVX_H_
#define TIDL_TIVX_H_

#include <stdio.h>
#include <stdint.h>

#define MAXPATHLENGTH 256

#define TIDL_MAX_OUTPUT_TENSORS (18)

typedef struct {
    const char* test_name;
    const char* config_name;
    const char* network_name;
} Tidl_Arg;

typedef struct CT_Rect_ {
    uint32_t x;
    uint32_t y;
    uint32_t width;
    uint32_t height;
} CT_Rect;

typedef struct CT_ImageHdr {
    uint32_t   width;  // in pixels
    uint32_t   height;
    uint32_t   stride; // in pixels
    uint32_t   format;
    CT_Rect    roi;    // stores top left corner offset and full size of parent image

    union
    {
      uint8_t*  y;
      uint16_t* u16;
      int16_t*  s16;
      uint32_t* u32;
      int32_t*  s32;
      struct { uint8_t r,g,b; }*     rgb;
      struct { uint8_t y,u,v; }*     yuv;
      struct { uint8_t r,g,b,x; }*   rgbx;
      struct { uint8_t y0,u,y1,v; }* yuyv;
      struct { uint8_t u,y0,v,y1; }* uyvy;
    } data;

    // private area
    void* data_begin_;
    uint32_t* refcount_;
} *CT_Image;

// Initialize TIDL / TIVX 
void* tivx_tidl_init(Tidl_Arg *args);

// Get TIDL config, required for PreProc
void* tivx_tidl_config_get(void *handle);

// Query Tensor sizes based on model info
int tivx_tidl_query_input_dim(void* handle, int *dim_arr, int *arr_len);
int tivx_tidl_query_output_dim(void* handle, int *dim_arr, int *arr_len);

// Run one frame of TIDL
int tivx_tidl_process(void *handle, unsigned char *image, void *data_buf[]);

// Cleanup
int tivx_tidl_delete(void **handle);

#endif //TIDL_TIVX_H_
