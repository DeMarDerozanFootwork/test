/*
*
* Copyright (c) 2018 Texas Instruments Incorporated
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


#include <float.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>

#include <TI/tivx.h>
#include <TI/j7_tidl.h>
#include <TI/tivx_mem.h>
#include <TI/dl_kernels.h>

#include "itidl_ti.h"
#include "tivx_utils_tidl_trace.h"

#include "tidl_tivx.h"

//#define DEBUG_TEST_TIDL_FLOW
//#define DEBUG_TEST_TIDL_CONFIG

// In seg+det model, it causes buffer overflow in tensor output buffers
// not sure why, but yeet it with HACK WORKAROUND
#define USE_HACK_WA

/*
 * This is the size of trace buffer allocated in host memory and
 * shared with target.
 */
#define TIVX_TIDL_TRACE_DATA_SIZE  (256 * 1024 * 1024)

#define TEST_TIDL_MAX_TENSOR_DIMS   (4u)

typedef struct TidlContext_ {
    vx_context vx_context;

    vx_graph graph;
    vx_node node;
    vx_kernel kernel;
    vx_user_data_object network;
    vx_user_data_object createParams;
    vx_user_data_object config;
    vx_user_data_object inArgs;
    vx_user_data_object outArgs;
    vx_tensor input_tensor[1];
    vx_tensor output_tensor[TIDL_MAX_OUTPUT_TENSORS];

    vx_uint32 output_tensor_count;

} TidlContext;

static TidlContext* tidl_create_vx_context()
{
    TidlContext* dcontext = (TidlContext*)tivxMemAlloc(sizeof(TidlContext), TIVX_MEM_EXTERNAL);

    assert(dcontext != NULL);

    dcontext->vx_context = vxCreateContext();

    dcontext->graph = 0;
    dcontext->node = 0;
    dcontext->kernel = 0;

    return dcontext;
}

static void tidl_teardown_vx_context(TidlContext *dcontext)
{
    vxReleaseContext(&dcontext->vx_context);
    tivxMemFree(dcontext, sizeof(TidlContext), TIVX_MEM_EXTERNAL);
}

static vx_user_data_object readConfig(vx_context context, const char *config_file, uint32_t *num_input_tensors, uint32_t *num_output_tensors)
{
    vx_status status = VX_SUCCESS;
    tivxTIDLJ7Params  *tidlParams;
    sTIDL_IOBufDesc_t *ioBufDesc = NULL;

    vx_user_data_object   config = NULL;
    vx_uint32 capacity;
    vx_map_id map_id;
    vx_size read_count;

    FILE *fp_config;

    #ifdef DEBUG_TEST_TIDL_FLOW
    printf("Reading IO config file %s ...\n", config_file);
    #endif

    fp_config = fopen(config_file, "rb");

    if(fp_config == NULL)
    {
        printf("ERROR: Unable to open IO config file %s \n", config_file);

        return NULL;
    }

    fseek(fp_config, 0, SEEK_END);
    capacity = ftell(fp_config);
    fseek(fp_config, 0, SEEK_SET);

    if( capacity != sizeof(sTIDL_IOBufDesc_t) )
    {
        printf("ERROR: Config file size (%d bytes) does not match size of sTIDL_IOBufDesc_t (%d bytes)\n", capacity, (vx_uint32)sizeof(sTIDL_IOBufDesc_t));
        fclose(fp_config);
        return NULL;
    }

    /* Create a user struct type for handling config data*/
    config = vxCreateUserDataObject(context, "tivxTIDLJ7Params", sizeof(tivxTIDLJ7Params), NULL );

    status = vxGetStatus((vx_reference)config);

    if (VX_SUCCESS == status)
    {
        status = vxMapUserDataObject(config, 0, sizeof(tivxTIDLJ7Params), &map_id,
                            (void **)&tidlParams, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST, 0);

        if (VX_SUCCESS == status)
        {
            if(tidlParams == NULL)
            {
                printf("ERROR: Map of config object failed\n");
                fclose(fp_config);
                return NULL;
            }

            tivx_tidl_j7_params_init(tidlParams);

            ioBufDesc = (sTIDL_IOBufDesc_t *)&tidlParams->ioBufDesc;

            read_count = fread(ioBufDesc, capacity, 1, fp_config);
            if(read_count != 1)
            {
              printf("ERROR: Unable to read config file\n");
            }
            fclose(fp_config);

            *num_input_tensors  = ioBufDesc->numInputBuf;
            *num_output_tensors = ioBufDesc->numOutputBuf;

            vxUnmapUserDataObject(config, map_id);

            #ifdef DEBUG_TEST_TIDL_CONFIG
            printf("Finished reading IO config file of %d bytes, num_input_tensors = %d, num_output_tensors = %d\n", capacity, *num_input_tensors, *num_output_tensors);
            #endif

            if (*num_output_tensors > TIDL_MAX_OUTPUT_TENSORS)
            {
              printf("Unexpected mismatch of output tensors, %d > %d\n", *num_output_tensors, TIDL_MAX_OUTPUT_TENSORS);
            }
        }
        else
        {
            fclose(fp_config);
        }
    }
    else
    {
        fclose(fp_config);
    }

    return config;
}

static vx_user_data_object readNetwork(vx_context context, const char *network_file)
{
    vx_status status;

    vx_user_data_object  network;
    vx_map_id  map_id;
    vx_uint32  capacity;
    void      *network_buffer = NULL;
    vx_size read_count;

    FILE *fp_network;

    #ifdef DEBUG_TEST_TIDL_FLOW
    printf("Reading network file %s ...\n", network_file);
    #endif

    fp_network = fopen(network_file, "rb");

    if(fp_network == NULL)
    {
        printf("ERROR: Unable to open network file %s \n", network_file);

        return NULL;
    }

    fseek(fp_network, 0, SEEK_END);
    capacity = ftell(fp_network);
    fseek(fp_network, 0, SEEK_SET);

    network = vxCreateUserDataObject(context, "TIDL_network", capacity, NULL );

    status = vxGetStatus((vx_reference)network);

    if (VX_SUCCESS == status)
    {
        status = vxMapUserDataObject(network, 0, capacity, &map_id,
                            (void **)&network_buffer, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST, 0);

        if (VX_SUCCESS == status)
        {
            if(network_buffer) {
                read_count = fread(network_buffer, capacity, 1, fp_network);
                if(read_count != 1)
                {
                    printf("ERROR: Unable to read network file\n");
                }
            } else {
                printf("ERROR: Unable to allocate memory for reading network file of %d bytes\n", capacity);
            }

            vxUnmapUserDataObject(network, map_id);

            #ifdef DEBUG_TEST_TIDL_FLOW
            printf("Finished reading network file of %d bytes\n", capacity);
            #endif
        }
    }

    fclose(fp_network);

    return network;
}

static vx_user_data_object setCreateParams(vx_context context)
{
    vx_status status;

    vx_user_data_object  createParams;
    vx_map_id  map_id;
    vx_uint32  capacity;
    void *createParams_buffer = NULL;

    capacity = sizeof(TIDL_CreateParams);
    createParams = vxCreateUserDataObject(context, "TIDL_CreateParams", capacity, NULL );

    status = vxGetStatus((vx_reference)createParams);

    if (VX_SUCCESS == status)
    {
        status = vxMapUserDataObject(createParams, 0, capacity, &map_id,
                        (void **)&createParams_buffer, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST, 0);

        if (VX_SUCCESS == status)
        {
            if(createParams_buffer)
            {
              TIDL_CreateParams *prms = createParams_buffer;
              //write create params here
              TIDL_createParamsInit(prms);

              prms->isInbufsPaded                 = 1;

              prms->quantRangeExpansionFactor     = 1.0;
              prms->quantRangeUpdateFactor        = 0.0;

              prms->traceLogLevel                 = 0;
              prms->traceWriteLevel               = 0;
            }
            else
            {
                printf("Unable to allocate memory for create time params! %d bytes\n", capacity);
            }

            vxUnmapUserDataObject(createParams, map_id);
        }
    }

    return createParams;
}

static vx_user_data_object setInArgs(vx_context context)
{
    vx_status status;

    vx_user_data_object  inArgs;
    vx_map_id  map_id;
    vx_uint32  capacity;
    void *inArgs_buffer = NULL;

    capacity = sizeof(TIDL_InArgs);
    inArgs = vxCreateUserDataObject(context, "TIDL_InArgs", capacity, NULL );

    status = vxGetStatus((vx_reference)inArgs);

    if (VX_SUCCESS == status)
    {
        status = vxMapUserDataObject(inArgs, 0, capacity, &map_id,
                        (void **)&inArgs_buffer, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST, 0);

        if (VX_SUCCESS == status)
        {
            if(inArgs_buffer)
            {
              TIDL_InArgs *prms = inArgs_buffer;
              prms->iVisionInArgs.size         = sizeof(TIDL_InArgs);
              prms->iVisionInArgs.subFrameInfo = 0;
            }
            else
            {
                printf("Unable to allocate memory for inArgs! %d bytes\n", capacity);
            }

            vxUnmapUserDataObject(inArgs, map_id);
        }
    }

    return inArgs;
}

static vx_user_data_object setOutArgs(vx_context context)
{
    vx_status status;

    vx_user_data_object  outArgs;
    vx_map_id  map_id;
    vx_uint32  capacity;
    void *outArgs_buffer = NULL;

    capacity = sizeof(TIDL_outArgs);
    outArgs = vxCreateUserDataObject(context, "TIDL_outArgs", capacity, NULL );

    status = vxGetStatus((vx_reference)outArgs);

    if (VX_SUCCESS == status)
    {
        status = vxMapUserDataObject(outArgs, 0, capacity, &map_id,
                        (void **)&outArgs_buffer, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST, 0);

        if (VX_SUCCESS == status)
        {
            if(outArgs_buffer)
            {
              TIDL_outArgs *prms = outArgs_buffer;
              prms->iVisionOutArgs.size         = sizeof(TIDL_outArgs);
            }
            else
            {
                printf("Unable to allocate memory for outArgs! %d bytes\n", capacity);
            }

            vxUnmapUserDataObject(outArgs, map_id);
        }
    }

    return outArgs;
}

static vx_enum get_vx_tensor_datatype(int32_t tidl_datatype)
{
    vx_enum tiovx_datatype = VX_TYPE_INVALID;

    if(tidl_datatype == TIDL_UnsignedChar)
    {
        tiovx_datatype = VX_TYPE_UINT8;
    }
    else if(tidl_datatype == TIDL_SignedChar)
    {
        tiovx_datatype = VX_TYPE_INT8;
    }
    else if(tidl_datatype == TIDL_UnsignedShort)
    {
        tiovx_datatype = VX_TYPE_UINT16;
    }
    else if(tidl_datatype == TIDL_SignedShort)
    {
        tiovx_datatype = VX_TYPE_INT16;
    }
    else if(tidl_datatype == TIDL_UnsignedWord)
    {
        tiovx_datatype = VX_TYPE_UINT32;
    }
    else if(tidl_datatype == TIDL_SignedWord)
    {
        tiovx_datatype = VX_TYPE_INT32;
    }
    else if(tidl_datatype == TIDL_SinglePrecFloat)
    {
        tiovx_datatype = VX_TYPE_FLOAT32;
    }
    else
    {
      printf("get_vx_tensor_datatype unknown type %d\n", tidl_datatype);
      assert(0);
    }

    return (tiovx_datatype);
}

static int get_vx_tensor_byte_count(int32_t tidl_datatype)
{
    switch (tidl_datatype) 
    {
        case TIDL_UnsignedChar:
        case TIDL_SignedChar:
          return 1;

        case TIDL_UnsignedShort:
        case TIDL_SignedShort:
          return 2;

        case TIDL_UnsignedWord:
        case TIDL_SignedWord:
        case TIDL_SinglePrecFloat:
          return 4;

        default:
          printf("get_vx_tensor_byte_count: Unknown type %d\n", tidl_datatype);
          return 1;
    }
    return 1;
}

static int get_vx_tensor_signed(int32_t tidl_datatype)
{
    switch (tidl_datatype) 
    {
        case TIDL_SignedChar:
        case TIDL_SignedShort:
        case TIDL_SignedWord:
        case TIDL_SinglePrecFloat:
          return 1;

        case TIDL_UnsignedChar:
        case TIDL_UnsignedShort:
        case TIDL_UnsignedWord:
          return 0;

        default:
          printf("get_vx_tensor_signed: Unknown type %d\n", tidl_datatype);
          return 1;
    }
    return 1;
}

static vx_tensor createInputTensor(vx_context context, vx_user_data_object config)
{
    vx_size   input_sizes[TEST_TIDL_MAX_TENSOR_DIMS];
    vx_map_id map_id_config;
    sTIDL_IOBufDesc_t *ioBufDesc;
    tivxTIDLJ7Params *tidlParams;

    vxMapUserDataObject(config, 0, sizeof(tivxTIDLJ7Params), &map_id_config,
                      (void **)&tidlParams, VX_READ_ONLY, VX_MEMORY_TYPE_HOST, 0);

    ioBufDesc = (sTIDL_IOBufDesc_t *)&tidlParams->ioBufDesc;
    input_sizes[0] = ioBufDesc->inWidth[0]  + ioBufDesc->inPadL[0] + ioBufDesc->inPadR[0];
    input_sizes[1] = ioBufDesc->inHeight[0] + ioBufDesc->inPadT[0] + ioBufDesc->inPadB[0];
    input_sizes[2] = ioBufDesc->inNumChannels[0]; //3

    #ifdef DEBUG_TEST_TIDL_CONFIG
    printf(" input_sizes[0] = %d, w = %d padL = %d padR = %d\n", (uint32_t)input_sizes[0], ioBufDesc->inWidth[0], ioBufDesc->inPadL[0], ioBufDesc->inPadR[0]);
    printf(" input_sizes[1] = %d, h = %d padT = %d padB = %d\n", (uint32_t)input_sizes[1], ioBufDesc->inHeight[0], ioBufDesc->inPadT[0], ioBufDesc->inPadB[0]);
    printf(" input_sizes[2] = %d, dim = %d \n", (uint32_t)input_sizes[2], ioBufDesc->inNumChannels[0]);
    #endif

    vxUnmapUserDataObject(config, map_id_config);

    return vxCreateTensor(context, 3, input_sizes, VX_TYPE_UINT8, 0);
}

static void createOutputTensor(vx_context context, vx_user_data_object config, vx_tensor output_tensors[])
{
    vx_size output_sizes[TEST_TIDL_MAX_TENSOR_DIMS];
    vx_map_id map_id_config;
    sTIDL_IOBufDesc_t *ioBufDesc;
    tivxTIDLJ7Params *tidlParams;
    int id;

    vxMapUserDataObject(config, 0, sizeof(tivxTIDLJ7Params), &map_id_config,
                      (void **)&tidlParams, VX_READ_ONLY, VX_MEMORY_TYPE_HOST, 0);

    ioBufDesc = (sTIDL_IOBufDesc_t *)&tidlParams->ioBufDesc;

    for (id = 0; id < ioBufDesc->numOutputBuf; id++)
    {
      output_sizes[0] = ioBufDesc->outWidth[id]  + ioBufDesc->outPadL[id] + ioBufDesc->outPadR[id];
      output_sizes[1] = ioBufDesc->outHeight[id] + ioBufDesc->outPadT[id] + ioBufDesc->outPadB[id];
      output_sizes[2] = ioBufDesc->outNumChannels[id];
      vx_enum data_type = get_vx_tensor_datatype(ioBufDesc->outElementType[id]);

      #ifdef DEBUG_TEST_TIDL_CONFIG
      printf(" output_sizes[%d][0] = %d, w = %d padL = %d padR = %d\n", id, (uint32_t)output_sizes[0], ioBufDesc->outWidth[id], ioBufDesc->outPadL[id], ioBufDesc->outPadR[id]);
      printf(" output_sizes[%d][1] = %d, h = %d padT = %d padB = %d\n", id, (uint32_t)output_sizes[1], ioBufDesc->outHeight[id], ioBufDesc->outPadT[id], ioBufDesc->outPadB[id]);
      printf(" output_sizes[%d][2] = %d, dim = %d type = %X\n", id, (uint32_t)output_sizes[2], ioBufDesc->outNumChannels[id], (int)data_type);
      #endif

#ifdef USE_HACK_WA
      output_tensors[id] = vxCreateTensor(context, 3, output_sizes, VX_TYPE_FLOAT32, 0); //WA for buffer overflow issue with det+seg model
      (void)data_type; // kill compiler warning
#else
      output_tensors[id] = vxCreateTensor(context, 3, output_sizes, data_type, 0); // works OK for det only or seg only
#endif
    }

    vxUnmapUserDataObject(config, map_id_config);

    return;
}

static vx_status prepInputNoPadding( vx_user_data_object config, vx_tensor *input_tensors, unsigned char *image)
{
    vx_status status = VX_SUCCESS;

    void      *input_buffer = NULL;
    int32_t    capacity;
    uint32_t   id;

    vx_map_id map_id_config;
    vx_map_id map_id_input;

    vx_size    start[TEST_TIDL_MAX_TENSOR_DIMS];
    vx_size    input_strides[TEST_TIDL_MAX_TENSOR_DIMS];
    vx_size    input_sizes[TEST_TIDL_MAX_TENSOR_DIMS];

    sTIDL_IOBufDesc_t *ioBufDesc;
    tivxTIDLJ7Params  *tidlParams;

    vxMapUserDataObject(config, 0, sizeof(tivxTIDLJ7Params), &map_id_config,
                      (void **)&tidlParams, VX_READ_ONLY, VX_MEMORY_TYPE_HOST, 0);

    ioBufDesc = (sTIDL_IOBufDesc_t *)&tidlParams->ioBufDesc;

    for(id = 0; id < ioBufDesc->numInputBuf; id++) // 1
    {
        input_sizes[0] = ioBufDesc->inWidth[id]  + ioBufDesc->inPadL[id] + ioBufDesc->inPadR[id];
        input_sizes[1] = ioBufDesc->inHeight[id] + ioBufDesc->inPadT[id] + ioBufDesc->inPadB[id];
        input_sizes[2] = ioBufDesc->inNumChannels[id];

        capacity = input_sizes[0] * input_sizes[1] * input_sizes[2];

        start[0] = start[1] = start[2] = 0;

        input_strides[0] = 1;
        input_strides[1] = input_sizes[0];
        input_strides[2] = input_sizes[1] * input_sizes[0];

        status = tivxMapTensorPatch(input_tensors[id], 3, start, input_sizes, &map_id_input, input_strides, &input_buffer, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);

        if (VX_SUCCESS == status)
        {
            vx_uint8 *data_ptr = NULL;

            data_ptr = (vx_uint8 *)image;

            memcpy(input_buffer, data_ptr, capacity);

            tivxUnmapTensorPatch(input_tensors[id], map_id_input);
        }
    }

    vxUnmapUserDataObject(config, map_id_config);

    return status;
}

#if 0
// TODO REMOVE
static void dumpbyte(uint8_t *buf, int len)
{
  int i;
  for (i = 0; i < len; i++)
  {
    if (i % 16 == 0) printf("\n");
    printf("%02x ", buf[i]);
  }
  printf("\n");
}

static void dumpfloat(float *buf, int len)
{
  int i;
  for (i = 0; i < len; i++)
  {
    if (i % 16 == 0) printf("\n");
    printf("%6.04f ", buf[i]);
  }
  printf("\n");
}
#endif

static vx_status copyOutput(vx_user_data_object config, vx_user_data_object outArgs, vx_tensor output_tensors[], void *output_buf[])
{
    vx_status status = VX_SUCCESS;

    vx_size output_sizes[TEST_TIDL_MAX_TENSOR_DIMS];

    vx_map_id map_id_config;
    vx_map_id map_id_out_args;

    int32_t id;

    tivxTIDLJ7Params *tidlParams = NULL;
    sTIDL_IOBufDesc_t *ioBufDesc = NULL;
    TIDL_outArgs *output_args = NULL;

    vxMapUserDataObject(config, 0, sizeof(tivxTIDLJ7Params), &map_id_config,
                      (void **)&tidlParams, VX_READ_ONLY, VX_MEMORY_TYPE_HOST, 0);

    ioBufDesc = (sTIDL_IOBufDesc_t *)&tidlParams->ioBufDesc;

    vxMapUserDataObject(outArgs, 0, sizeof(TIDL_outArgs), &map_id_out_args,
                      (void **)&output_args, VX_READ_ONLY, VX_MEMORY_TYPE_HOST, 0);

    for(id = 0; id < ioBufDesc->numOutputBuf; id++)
    {
        if (output_buf[id] == NULL)
        {
            printf("copyOutput out of buffers %d\n", id);
            status = VX_FAILURE;
            break;
        }

        status = vxGetStatus((vx_reference)output_tensors[id]);

        if (VX_SUCCESS == status)
        {
            void *output_buffer;

            vx_map_id map_id_output;

            vx_size output_strides[TEST_TIDL_MAX_TENSOR_DIMS];
            vx_size start[TEST_TIDL_MAX_TENSOR_DIMS];
            vx_size num_dims;
            uint32_t byte_factor = get_vx_tensor_byte_count(ioBufDesc->outElementType[id]);


            output_sizes[0] = ioBufDesc->outWidth[id]  + ioBufDesc->outPadL[id] + ioBufDesc->outPadR[id];
            output_sizes[1] = ioBufDesc->outHeight[id] + ioBufDesc->outPadT[id] + ioBufDesc->outPadB[id];
            output_sizes[2] = ioBufDesc->outNumChannels[id];

            #ifdef DEBUG_TEST_TIDL_CONFIG
            printf("[%d] %dx%d\n", id, ioBufDesc->outWidth[id], ioBufDesc->outHeight[id]);
            printf("[%d] PadL %d PadR %d\n", id, ioBufDesc->outPadL[id], ioBufDesc->outPadR[id]);
            printf("[%d] PadT %d PadB %d\n", id, ioBufDesc->outPadT[id], ioBufDesc->outPadB[id]);
            printf("[%d] outNumChannels %d\n", id, ioBufDesc->outNumChannels[id]);
            printf("[%d] scale %0.1f\n", id, output_args->scale[id]);
            #endif

            start[0] = start[1] = start[2] = start[3] = 0;

            output_strides[0] = 1;
            output_strides[1] = output_sizes[0];
            output_strides[2] = output_sizes[1] * output_sizes[0];

            #ifdef DEBUG_TEST_TIDL_CONFIG
            printf("output_sizes %d %d %d\n", (int)output_sizes[0], (int)output_sizes[1], (int)output_sizes[2]);
            printf("output_strides %d %d %d\n", (int)output_strides[0], (int)output_strides[1], (int)output_strides[2]);
            #endif

            vxQueryTensor(output_tensors[id], VX_TENSOR_NUMBER_OF_DIMS, &num_dims, sizeof(vx_size));

            tivxMapTensorPatch(output_tensors[id], num_dims, start, output_sizes, &map_id_output, output_strides, &output_buffer, VX_READ_ONLY, VX_MEMORY_TYPE_HOST);
#if 0
            snprintf(filename, MAXPATHLENGTH, "test_%d.raw", file_cnt++);
            printf("saving file %s\n", filename);
            FILE *fp_config = fopen(filename, "wb");
            fwrite(output_buffer, output_sizes[1] * output_sizes[0], 1, fp_config);
            fclose(fp_config);
#endif
            // check if no padding and no scale
            if ((output_sizes[0] == ioBufDesc->outWidth[id]) &&
                (output_sizes[1] == ioBufDesc->outHeight[id]) &&
                (output_args->scale[id] == 1.0f))
            {
                // no padding, straight copy
                memcpy(output_buf[id], output_buffer, output_sizes[0] * output_sizes[1] * output_sizes[2] * byte_factor);
            }
            // check if padding and no scale
            else if (output_args->scale[id] == 1.0f)
            {
                // Undo padding
                uint32_t ch, y;
                int8_t *dst_addr = (int8_t*)output_buf[id]; //recast to byte
                for (ch = 0; ch < ioBufDesc->outNumChannels[id]; ch++)
                {
                    // adjust top 
                    int8_t *src_addr = (int8_t*)output_buffer +
                      ((output_strides[2] * ch) +
                       (output_strides[1] * ioBufDesc->outPadT[id]) +
                       ioBufDesc->outPadL[id]) * byte_factor;
 
                    for (y = 0; y < ioBufDesc->outHeight[id]; y++)
                    {
                        // copy line
                        memcpy(dst_addr, src_addr, ioBufDesc->outWidth[id] * byte_factor);
                        // increment src
                        src_addr += (output_strides[1] * byte_factor); 
                        // increment dst
                        dst_addr += (ioBufDesc->outWidth[id] * byte_factor);
                    }
                }
            }
            else
            {
                // Undo padding, perform descale/dequantization
                uint32_t ch, x, y;
                float *dst_addr = (float*)output_buf[id]; //recast to float
                uint32_t signed_val = get_vx_tensor_signed(ioBufDesc->outElementType[id]);
                if (byte_factor == 1)
                {
#ifdef USE_HACK_WA
                    // Due to float size, all the output_strides were multiplied by 4, undo it
                    output_strides[0] /= 4;
                    output_strides[1] /= 4;
                    output_strides[2] /= 4;
#endif
                    if (signed_val)
                    {
                        // signed
                        for (ch = 0; ch < ioBufDesc->outNumChannels[id]; ch++)
                        {
                            // adjust top 
                            int8_t *src_addr = (int8_t*)output_buffer +
                              (output_strides[2] * ch) +
                              (output_strides[1] * ioBufDesc->outPadT[id]) +
                              ioBufDesc->outPadL[id];
             
                            for (y = 0; y < ioBufDesc->outHeight[id]; y++)
                            {
                                for (x = 0; x < ioBufDesc->outWidth[id]; x++)
                                {
                                    int32_t tmp = (int32_t)src_addr[x]; // should sign extend
                                    float value = (float)tmp / output_args->scale[id];
                                    dst_addr[x] = value;
                                }
                                // step src by one line
                                src_addr += output_strides[1]; 
                                // step dst by one line
                                dst_addr += ioBufDesc->outWidth[id]; 
                            }
                        }
                    }
                    else
                    {         
                        // unsigned, duplicate loop to avoid inloop branching
                        for (ch = 0; ch < ioBufDesc->outNumChannels[id]; ch++)
                        {
                            // adjust top 
                            uint8_t *src_addr = (uint8_t*)output_buffer +
                              (output_strides[2] * ch) +
                              (output_strides[1] * ioBufDesc->outPadT[id]) +
                              ioBufDesc->outPadL[id];

                            for (y = 0; y < ioBufDesc->outHeight[id]; y++)
                            {
                                for (x = 0; x < ioBufDesc->outWidth[id]; x++)
                                {
                                    uint32_t tmp = (uint32_t)src_addr[x]; // no sign extend
                                    float value = (float)tmp / output_args->scale[id];
                                    dst_addr[x] = value;
                                }
                                // step src by one line
                                src_addr += output_strides[1]; 
                                // step dst by one line
                                dst_addr += ioBufDesc->outWidth[id]; 
                            }
                        }
                    }
#if 0
if (id == 4)
{
dumpbyte(output_buffer, 256);
printf("scale %0.1f\n", output_args->scale[id]);
dumpfloat((float*)output_buf[id], 256);
}
#endif
                }
                else
                {
                    printf("Unsupported dequantization with size %d, skip %d output\n", byte_factor, id);
                }
            }

            tivxUnmapTensorPatch(output_tensors[id], map_id_output);
        }
        else
        {
            printf("copyOutput failed vxGetStatus %d\n", status);
        }
    }

    vxUnmapUserDataObject(config, map_id_config);
    vxUnmapUserDataObject(outArgs, map_id_out_args);

    return status;
}

static int tivx_tidl_query_input(vx_user_data_object config, int *dim_array)
{
  vx_map_id map_id_config;
  tivxTIDLJ7Params *tidlParams = NULL;
  sTIDL_IOBufDesc_t *ioBufDesc = NULL;
  int count = 0;
  int id;

  vxMapUserDataObject(config, 0, sizeof(tivxTIDLJ7Params), &map_id_config,
                    (void **)&tidlParams, VX_READ_ONLY, VX_MEMORY_TYPE_HOST, 0);

  ioBufDesc = (sTIDL_IOBufDesc_t *)&tidlParams->ioBufDesc;

  count = ioBufDesc->numInputBuf;

  for(id = 0; id < count; id++)
  {
    // calc len, include padding since it's needed by preproc process
    int w = ioBufDesc->inWidth[id]  + ioBufDesc->inPadL[id] + ioBufDesc->inPadR[id];
    int h = ioBufDesc->inHeight[id] + ioBufDesc->inPadT[id] + ioBufDesc->inPadB[id];
    int factor = get_vx_tensor_byte_count(ioBufDesc->inElementType[id]);
    dim_array[id] = w * factor * h * ioBufDesc->inNumChannels[id];
  }

  vxUnmapUserDataObject(config, map_id_config);

  return count;
}

static int tivx_tidl_query_output(vx_user_data_object config, int *dim_array)
{
  vx_map_id map_id_config;
  tivxTIDLJ7Params *tidlParams = NULL;
  sTIDL_IOBufDesc_t *ioBufDesc = NULL;
  int count = 0;
  int id;

  vxMapUserDataObject(config, 0, sizeof(tivxTIDLJ7Params), &map_id_config,
                    (void **)&tidlParams, VX_READ_ONLY, VX_MEMORY_TYPE_HOST, 0);

  ioBufDesc = (sTIDL_IOBufDesc_t *)&tidlParams->ioBufDesc;

  count = ioBufDesc->numOutputBuf;

  for(id = 0; id < count; id++)
  {
    // calc len, exclude padding
    int factor = get_vx_tensor_byte_count(ioBufDesc->outElementType[id]);
    dim_array[id] = ioBufDesc->outWidth[id] * factor * ioBufDesc->outHeight[id] * ioBufDesc->outNumChannels[id];
  }

  vxUnmapUserDataObject(config, map_id_config);

  // sanity check
  assert(ioBufDesc->numOutputBuf <= TIDL_MAX_OUTPUT_TENSORS);

  return count;
}

#define PARAMETERS \
    CT_GENERATE_PARAMETERS("mobilenetv1", ARG, "tidl_io_mobilenet_v1_1.bin", "tidl_net_mobilenet_v1.bin", 0, 0), \
    CT_GENERATE_PARAMETERS("mobilenetv1", ARG, "tidl_io_mobilenet_v1_1.bin", "tidl_net_mobilenet_v1.bin", 0, 1)


void* tivx_tidl_init(Tidl_Arg *args)
{
  if (vx_true_e != tivxIsTargetEnabled(TIVX_TARGET_DSP_C7_1))
  {
    printf("Error C7 DSP not enabled\n");
    return NULL;
  }

  TidlContext *handle = tidl_create_vx_context();

  vx_context context = handle->vx_context;

  vx_user_data_object traceData;

  uint32_t num_input_tensors  = 0;
  uint32_t num_output_tensors = 0;

  tivxTIDLLoadKernels(context);

  handle->config = readConfig(context, args->config_name, &num_input_tensors, &num_output_tensors);
  handle->output_tensor_count = num_output_tensors;

  handle->kernel = tivxAddKernelTIDL(context, num_input_tensors, num_output_tensors);

  handle->graph = vxCreateGraph(context);

  handle->network = readNetwork(context, args->network_name);

  handle->createParams = setCreateParams(context);
  handle->inArgs = setInArgs(context);
  handle->outArgs = setOutArgs(context);

  handle->input_tensor[0] = createInputTensor(context, handle->config);

  createOutputTensor(context, handle->config, handle->output_tensor);

  traceData = NULL;

  vx_reference params[] = {
          (vx_reference)handle->config,
          (vx_reference)handle->network,
          (vx_reference)handle->createParams,
          (vx_reference)handle->inArgs,
          (vx_reference)handle->outArgs,
          (vx_reference)traceData
  };

  handle->node = tivxTIDLNode(handle->graph, handle->kernel, params, handle->input_tensor, handle->output_tensor);

  #ifdef DEBUG_TEST_TIDL_FLOW
  printf("Verifying graph ...\n");
  #endif

  vx_status status = vxVerifyGraph(handle->graph);

  if (status != VX_SUCCESS)
  {
    printf("Failed vxVerifyGraph\n");
    return NULL;
  }

  return (void*)handle;
}

int tivx_tidl_load_image(void *handle, unsigned char *image)
{
  if (handle == NULL)
  {
    return -1;
  }

  TidlContext *context = (TidlContext*)handle;

  prepInputNoPadding(context->config, &context->input_tensor[0], image);

  return 0;
}

void* tivx_tidl_config_get(void *handle)
{
  if (handle == NULL)
  {
    return NULL;
  }

  TidlContext *context = (TidlContext*)handle;
  return (void*)context->config;
}

int tivx_tidl_query_input_dim(void* handle, int *dim_arr, int *arr_len)
{
  int len = 0;

  if (handle == NULL)
  {
    return -1;
  }

  TidlContext *context = (TidlContext*)handle;

  len = tivx_tidl_query_input(context->config, dim_arr);
  if (arr_len != NULL)
  {
    *arr_len = len;
  }

  return len;
}

int tivx_tidl_query_output_dim(void* handle, int *dim_arr, int *arr_len)
{
  int len = 0;

  if (handle == NULL)
  {
    return -1;
  }

  TidlContext *context = (TidlContext*)handle;

  len = tivx_tidl_query_output(context->config, dim_arr);
  if (arr_len != NULL)
  {
    *arr_len = len;
  }

  return len;
}

// takes padded image
int tivx_tidl_process(void *handle, unsigned char *image, void *data_buf[])
{
  if (handle == NULL)
  {
    return -1;
  }

  TidlContext *context = (TidlContext*)handle;

  tivx_tidl_load_image(handle, image);

  #ifdef DEBUG_TEST_TIDL_FLOW
  printf("Running graph ...\n");
  #endif
  vxProcessGraph(context->graph);
  #ifdef DEBUG_TEST_TIDL_FLOW
  printf("Showing output ...\n");
  #endif

  copyOutput(context->config, context->outArgs, context->output_tensor, data_buf);

  return 0;
}

int tivx_tidl_delete(void **handle)
{
  if (handle != NULL && *handle != NULL)
  {
    TidlContext *context = (TidlContext*)*handle;

    vxReleaseNode(&context->node);

    for (int id = 0; id < context->output_tensor_count; id++)
    {
      vxReleaseTensor(&context->output_tensor[id]);
    }

    vxReleaseTensor(&context->input_tensor[0]);

    vxReleaseUserDataObject(&context->outArgs);

    vxReleaseUserDataObject(&context->inArgs);

    vxReleaseUserDataObject(&context->createParams);

    vxReleaseUserDataObject(&context->network);

    vxReleaseGraph(&context->graph);

    vxRemoveKernel(context->kernel);

    vxReleaseUserDataObject(&context->config);

    tivxTIDLUnLoadKernels(context->vx_context);

    tidl_teardown_vx_context(context);

    *handle = NULL;
  }

  return 0;
}

