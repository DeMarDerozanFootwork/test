#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <arm_neon.h>
#include <pthread.h>
#include <string.h>
#include <time.h>

#include "multi_cam_tivx.h"
#include "app_common.h"
#include "gst_stream.h"

static vx_context   context_            = NULL;
static vx_graph     color_conversion_graph_  = NULL;
static vx_image     input_rgb_image_    = NULL;
static vx_image     output_nv12_image_  = NULL;
static GstElement   *gst_pipeline       = NULL;
static GstElement   *appsrc             = NULL;
static uint64_t     frame_counter       = 0;
static int frame_width_ = 0;
static int frame_height_= 0;
static int target_fps_  = 30;

static GstElement* g_mosaic_pipeline= NULL;
static GstElement* g_mosaic_appsrc  = NULL;
static GstElement* g_mosaic_appsink = NULL;
static uint64_t    g_mosaic_frame_counter = 0;
static int         g_mosaic_fps = 15;
static pthread_cond_t g_encoded_frame_cond = PTHREAD_COND_INITIALIZER;

#define MAX_ENCODED_BUF_SIZE (2 * 1024 * 1024) // 2MB
#define ENCODED_FRAME_QUEUE_SIZE 3
#define GET_ENCODED_FRAME_TIMEOUT_MS 20
#define DEBUG_MODE (1)

typedef struct {
    uint32_t frame_number;
    uint32_t size;
    uint8_t  buffer[MAX_ENCODED_BUF_SIZE];
} EncodedFrame;

static EncodedFrame g_encoded_frame_queue[ENCODED_FRAME_QUEUE_SIZE];
static volatile uint32_t g_encoded_frame_write_idx = 0;
static pthread_mutex_t g_encoded_frame_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_once_t g_gst_is_initialized = PTHREAD_ONCE_INIT;
static int g_internal_mosaic_width = 0;
static int g_internal_mosaic_height = 0;
static bool g_is_mosaic_encoding_initialized = false;


static void initialize_gstreamer_once(void) {
    gst_init(NULL, NULL);
}

static inline void neon_memcpy(void *dst, const void *src, size_t n) {
    uint8_t *d = (uint8_t *)dst;
    const uint8_t *s = (const uint8_t *)src;
    vx_uint32 i = 0;

    // Use NEON for bulky copy
    for (; i + 15 < n; i += 16) {
        vst1q_u8(d + i, vld1q_u8(s + i));
    }

    // Handle any remaining bytes
    for (; i < n; i++) {
        d[i] = s[i];
    }
}

/**
 * Initializes the OpenVX and GStreamer components for H264 encoding and streaming via UDP.
 * The OpenVX part handles RGB to NV12 color space conversion.
 * The GStreamer part handles H264 encoding and network streaming.
 */
bool encodeH264_send_by_gst_init(const char *pipeline_str, int width, int height, int fps)
{
    vx_node   color_convert_node = NULL;
    GError *gst_err = NULL;

    context_ = vxCreateContext();
    if (vxGetStatus((vx_reference)context_) != VX_SUCCESS) {
        APP_PRINTF("Failed to create OpenVX context in encodeH264_send_by_gst_init.\n");
        return false;
    }

    color_conversion_graph_ = vxCreateGraph(context_);
    if (vxGetStatus((vx_reference)color_conversion_graph_) != VX_SUCCESS) {
        APP_PRINTF("Failed to create OpenVX graph in encodeH264_send_by_gst_init.\n");
        goto error_handling;
    }

    input_rgb_image_ = vxCreateImage(context_, width, height, VX_DF_IMAGE_RGB);
    output_nv12_image_ = vxCreateImage(context_, width, height, VX_DF_IMAGE_NV12);
    if (vxGetStatus((vx_reference)input_rgb_image_) != VX_SUCCESS ||
        vxGetStatus((vx_reference)output_nv12_image_) != VX_SUCCESS) {
        APP_PRINTF("Failed to create OpenVX image in encodeH264_send_by_gst_init.\n");
        goto error_handling;
    }

    color_convert_node = vxColorConvertNode(color_conversion_graph_, input_rgb_image_, output_nv12_image_);
    if (vxGetStatus((vx_reference)color_convert_node) != VX_SUCCESS) {
        APP_PRINTF("Failed to create OpenVX color convert node in encodeH264_send_by_gst_init.\n");
        goto error_handling;
    }

    if (vxVerifyGraph(color_conversion_graph_) != VX_SUCCESS) {
        APP_PRINTF("Failed to verify OpenVX graph in encodeH264_send_by_gst_init.\n");
        vxReleaseNode(&color_convert_node);
        goto error_handling;
    }

    vxReleaseNode(&color_convert_node);

    //APP_PRINTF("Using GStreamer pipeline: %s\n", pipeline_str);

    // Initialize the GStreamer library.
    gst_init(NULL, NULL);
    gst_pipeline = gst_parse_launch(pipeline_str, &gst_err);

    if (!gst_pipeline) {
        APP_PRINTF("Failed to create GStreamer pipeline: %s\n", gst_err ? gst_err->message : "Unknown error");
        if (gst_err) g_error_free(gst_err);
        goto error_handling;
    }

    appsrc = gst_bin_get_by_name(GST_BIN(gst_pipeline), "raw_video_source");
    if (!appsrc) {
        APP_PRINTF("Failed to get appsrc element from pipeline.\n");
        gst_object_unref(gst_pipeline);
        gst_pipeline = NULL;
        goto error_handling;
    }

    gst_element_set_state(gst_pipeline, GST_STATE_PLAYING);

    frame_width_ = width;
    frame_height_ = height;
    target_fps_ = fps;
    frame_counter = 0;

    return true;

error_handling:
    if (color_conversion_graph_) vxReleaseGraph(&color_conversion_graph_);
    if (input_rgb_image_) vxReleaseImage(&input_rgb_image_);
    if (output_nv12_image_) vxReleaseImage(&output_nv12_image_);
    if (context_) vxReleaseContext(&context_);
    return false;
}

/**
 * Processes a single frame: converts it from RGB to NV12 using OpenVX,
 * and pushes it into the GStreamer pipeline for encoding and streaming.
 */
bool encodeH264_send_by_gst(uint8_t *data, size_t size)
{
    if (!context_ || !appsrc || !data || size != (frame_width_ * frame_height_ * 3)) {
        APP_PRINTF("Invalid parameters for encodeH264_send_by_gst. \n");
        return false;
    }

    vx_rectangle_t rect = {0, 0, (vx_uint32)frame_width_, (vx_uint32)frame_height_};
    vx_map_id map_id;
    vx_imagepatch_addressing_t addr = {0};
    void *ptr = NULL;

    // Map the OpenVX input RGB image to host memory for writing
    vxMapImagePatch(input_rgb_image_, &rect, 0, &map_id, &addr, &ptr, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST, 0);
    // Check if the image stride matches RGB
    if (addr.stride_y == frame_width_ * 3) {
        neon_memcpy(ptr, data, size);
    } else {
        for (vx_uint32 y = 0; y < frame_height_; y++) {
            neon_memcpy((uint8_t*)ptr + y * addr.stride_y, data + y * frame_width_ * 3, frame_width_ * 3);
        }
    }
    // Unmap the image patch, applying the changes
    vxUnmapImagePatch(input_rgb_image_, map_id);

    // Process the OpenVX graph to convert the color from RGB to NV12
    if (vxProcessGraph(color_conversion_graph_) != VX_SUCCESS) {
        APP_PRINTF("Failed to process OpenVX graph in encodeH264_send_by_gst. \n");
        return false;
    }

    const size_t nv12_size = frame_width_ * frame_height_ * 3 / 2;
    GstBuffer *buffer = gst_buffer_new_allocate(NULL, nv12_size, NULL);
    GstMapInfo gst_map;
    gst_buffer_map(buffer, &gst_map, GST_MAP_WRITE);

    // --- Copy Y plane ---
    vx_map_id y_map_id;
    vx_imagepatch_addressing_t y_addr = {0};
    void *y_ptr = NULL;
    // Map the Y-plane of NV12 output image
    vxMapImagePatch(output_nv12_image_, &rect, 0, &y_map_id, &y_addr, &y_ptr, VX_READ_ONLY, VX_MEMORY_TYPE_HOST, 0);
    // Copy Y data
    for (vx_uint32 y = 0; y < frame_height_; y++) {
        neon_memcpy(gst_map.data + y * frame_width_, (uint8_t*)y_ptr + y * y_addr.stride_y, frame_width_);
    }
    vxUnmapImagePatch(output_nv12_image_, y_map_id);

    // --- Copy UV plane ---
    vx_map_id uv_map_id;
    vx_imagepatch_addressing_t uv_addr = {0};
    void *uv_ptr = NULL;
    rect.end_y /= 2;    // height of UV plane = half Y plane
    // Map the UV-plane
    vxMapImagePatch(output_nv12_image_, &rect, 1, &uv_map_id, &uv_addr, &uv_ptr, VX_READ_ONLY, VX_MEMORY_TYPE_HOST, 0);
    uint8_t* uv_dest = gst_map.data + (frame_width_ * frame_height_);
    // Copy UV data
    for (vx_uint32 y = 0; y < frame_height_ / 2; y++) {
        neon_memcpy(uv_dest + y * frame_width_, (uint8_t*)uv_ptr + y * uv_addr.stride_y, frame_width_);
    }
    vxUnmapImagePatch(output_nv12_image_, uv_map_id);
  
    // Unmap the GStreamer buffer, making the data available to the pipeline
    gst_buffer_unmap(buffer, &gst_map);

    // Set the Presentation Timestamp, Decoding Timestamp of the buffer
    GstClockTime timestamp = gst_util_uint64_scale(frame_counter, GST_SECOND, target_fps_);
    GST_BUFFER_PTS(buffer) = timestamp;
    GST_BUFFER_DTS(buffer) = timestamp;
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale(1, GST_SECOND, target_fps_);
    frame_counter++;

    GstFlowReturn ret;
    // Emit the push-buffer signal to send the buffer to the appsrc element
    g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
    // Unreference the GStreamer buffer to release its memory
    gst_buffer_unref(buffer);

    if (ret != GST_FLOW_OK) {
      APP_PRINTF("GStreamer push-buffer failed with error: %d\n", ret);
      return false;
    }

    return true;
}

void encodeH264_send_by_gst_deinit()
{
    if (gst_pipeline) {
        gst_element_set_state(gst_pipeline, GST_STATE_NULL);
        gst_object_unref(gst_pipeline);
        gst_pipeline = NULL;
        appsrc = NULL;
    }
    if (color_conversion_graph_) vxReleaseGraph(&color_conversion_graph_);
    if (input_rgb_image_) vxReleaseImage(&input_rgb_image_);
    if (output_nv12_image_) vxReleaseImage(&output_nv12_image_);
    if (context_) vxReleaseContext(&context_);

    context_ = NULL;
    color_conversion_graph_ = NULL;
    input_rgb_image_ = NULL;
    output_nv12_image_ = NULL;
}

static GstFlowReturn on_new_sample_from_sink(GstAppSink *sink, gpointer user_data)
{
    GstSample *sample = gst_app_sink_pull_sample(sink);
    if (sample)
    {
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        GstMapInfo map;
        if (gst_buffer_map(buffer, &map, GST_MAP_READ))
        {
            pthread_mutex_lock(&g_encoded_frame_mutex);

            uint32_t write_idx = g_encoded_frame_write_idx;
            EncodedFrame *frame = &g_encoded_frame_queue[write_idx];

            // The frame number was stored in the PTS field when the buffer was pushed.
            frame->frame_number = (uint32_t)GST_BUFFER_PTS(buffer);

            if (map.size <= MAX_ENCODED_BUF_SIZE)
            {
                frame->size = map.size;
                memcpy(frame->buffer, map.data, map.size);
            }
            else
            {
                APP_PRINTF("ERROR: Encoded frame size (%zu) exceeds max buffer size (%d)\n", map.size, MAX_ENCODED_BUF_SIZE);
                frame->size = 0;
            }

            g_encoded_frame_write_idx = (write_idx + 1) % ENCODED_FRAME_QUEUE_SIZE;

            pthread_cond_signal(&g_encoded_frame_cond);

            pthread_mutex_unlock(&g_encoded_frame_mutex);
            gst_buffer_unmap(buffer, &map);
        }
        gst_sample_unref(sample);
    }
    return GST_FLOW_OK;
}



bool encodeH264_gst_init(const char *pipeline_str, bool enabled, int fps, int mosaic_width, int mosaic_height)
{
    if (!enabled) {
        return false;
    }

    if (mosaic_width <= 0 || mosaic_height <= 0) {
        APP_PRINTF("GST_STREAM: ERROR: Mosaic dimensions not set or invalid (%dx%d).\n", mosaic_width, mosaic_height);
        return false;
    }

    g_internal_mosaic_width = mosaic_width;
    g_internal_mosaic_height = mosaic_height;

    GError *gst_err = NULL;
    pthread_once(&g_gst_is_initialized, initialize_gstreamer_once);

    char final_pipeline[1024];
    snprintf(final_pipeline, sizeof(final_pipeline), pipeline_str, mosaic_width, mosaic_height);
    APP_PRINTF("GST_STREAM: Using GStreamer pipeline for mosaic: %s\n", final_pipeline);

    g_mosaic_pipeline = gst_parse_launch(final_pipeline, &gst_err);

    if (!g_mosaic_pipeline) {
        APP_PRINTF("GST_STREAM: Failed to create GStreamer pipeline: %s\n", gst_err ? gst_err->message : "Unknown error");
        if (gst_err) g_error_free(gst_err);
        return false;
    }

    g_mosaic_appsrc = gst_bin_get_by_name(GST_BIN(g_mosaic_pipeline), "h264_source");
    if (!g_mosaic_appsrc) {
        APP_PRINTF("GST_STREAM: Failed to get appsrc (h264_source) from pipeline.\n");
        goto error_handling;
    }

    g_mosaic_appsink = gst_bin_get_by_name(GST_BIN(g_mosaic_pipeline), "h264_sink");
    if (!g_mosaic_appsink) {
        APP_PRINTF("GST_STREAM: Failed to get appsink (h264_sink) from pipeline.\n");
        goto error_handling;
    }

    // Configure appsink to emit signals
    g_object_set(g_mosaic_appsink, "emit-signals", TRUE, NULL);
    g_signal_connect(g_mosaic_appsink, "new-sample", G_CALLBACK(on_new_sample_from_sink), NULL);

    gst_element_set_state(g_mosaic_pipeline, GST_STATE_PLAYING);

    g_mosaic_fps = fps;
    g_mosaic_frame_counter = 0;
    g_is_mosaic_encoding_initialized = true;

    // Initialize the frame queue
    memset(g_encoded_frame_queue, 0, sizeof(g_encoded_frame_queue));

    APP_PRINTF("GST_STREAM: Mosaic encoding pipeline initialized successfully.\n");
    return true;

error_handling:
    if (g_mosaic_appsrc) gst_object_unref(g_mosaic_appsrc);
    if (g_mosaic_appsink) gst_object_unref(g_mosaic_appsink);
    if (g_mosaic_pipeline) gst_object_unref(g_mosaic_pipeline);
    g_mosaic_pipeline = NULL;
    g_mosaic_appsrc = NULL;
    g_mosaic_appsink = NULL;
    return false;
}

void encodeH264_gst_deinit()
{
    if (g_mosaic_pipeline) {
        gst_element_set_state(g_mosaic_pipeline, GST_STATE_NULL);
        gst_object_unref(g_mosaic_pipeline);
        g_mosaic_pipeline = NULL;
        g_mosaic_appsrc = NULL;
        g_mosaic_appsink = NULL;
    }
    g_is_mosaic_encoding_initialized = false;
    APP_PRINTF("GST_STREAM: Mosaic encoding pipeline de-initialized.\n");
}

bool encodeH264_gst_feed_frame(vx_image mosaic_image, uint32_t frame_number)
{
    if (!g_is_mosaic_encoding_initialized || !g_mosaic_appsrc || !mosaic_image) {
        return false;
    }

    vx_status status = VX_SUCCESS;
    const size_t nv12_size = g_internal_mosaic_width * g_internal_mosaic_height * 3 / 2;
    GstBuffer *buffer = gst_buffer_new_allocate(NULL, nv12_size, NULL);
    if (!buffer) {
        APP_PRINTF("GST_STREAM: Failed to allocate GstBuffer\n");
        return false;
    }

    GstMapInfo gst_map;
    gst_buffer_map(buffer, &gst_map, GST_MAP_WRITE);

    // --- Copy Y plane ---
    vx_rectangle_t rect_y = {0, 0, (vx_uint32)g_internal_mosaic_width, (vx_uint32)g_internal_mosaic_height};
    vx_map_id y_map_id;
    vx_imagepatch_addressing_t y_addr = {0};
    void *y_ptr = NULL;
    status = vxMapImagePatch(mosaic_image, &rect_y, 0, &y_map_id, &y_addr, &y_ptr, VX_READ_ONLY, VX_MEMORY_TYPE_HOST, 0);
    if (status == VX_SUCCESS) {
        if (y_addr.stride_y == g_internal_mosaic_width) {
            neon_memcpy(gst_map.data, y_ptr, g_internal_mosaic_width * g_internal_mosaic_height);
        } else {
            for (vx_uint32 y = 0; y < g_internal_mosaic_height; y++) {
                neon_memcpy(gst_map.data + y * g_internal_mosaic_width, (uint8_t*)y_ptr + y * y_addr.stride_y, g_internal_mosaic_width);
            }
        }
        vxUnmapImagePatch(mosaic_image, y_map_id);
    } else {
        APP_PRINTF("GST_STREAM: vxMapImagePatch for Y-plane failed.\n");
        goto copy_error;
    }

    // --- Copy UV plane ---
    vx_rectangle_t rect_uv = {0, 0, (vx_uint32)g_internal_mosaic_width, (vx_uint32)g_internal_mosaic_height / 2};
    vx_map_id uv_map_id;
    vx_imagepatch_addressing_t uv_addr = {0};
    void *uv_ptr = NULL;
    status = vxMapImagePatch(mosaic_image, &rect_uv, 1, &uv_map_id, &uv_addr, &uv_ptr, VX_READ_ONLY, VX_MEMORY_TYPE_HOST, 0);
    if (status == VX_SUCCESS) {
        uint8_t* uv_dest = gst_map.data + (g_internal_mosaic_width * g_internal_mosaic_height);
        if (uv_addr.stride_y == g_internal_mosaic_width) {
            neon_memcpy(uv_dest, uv_ptr, g_internal_mosaic_width * g_internal_mosaic_height / 2);
        } else {
            for (vx_uint32 y = 0; y < g_internal_mosaic_height / 2; y++) {
                neon_memcpy(uv_dest + y * g_internal_mosaic_width, (uint8_t*)uv_ptr + y * uv_addr.stride_y, g_internal_mosaic_width);
            }
        }
        vxUnmapImagePatch(mosaic_image, uv_map_id);
    } else {
        APP_PRINTF("GST_STREAM: vxMapImagePatch for UV-plane failed.\n");
        goto copy_error;
    }

    gst_buffer_unmap(buffer, &gst_map);

    GST_BUFFER_PTS(buffer) = frame_number;
    GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale(1, GST_SECOND, g_mosaic_fps);

    g_mosaic_frame_counter++;

    GstFlowReturn ret;
    g_signal_emit_by_name(g_mosaic_appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);

    if (ret != GST_FLOW_OK) {
      APP_PRINTF("GST_STREAM: GStreamer push-buffer for mosaic failed with error: %d\n", ret);
      return false;
    }
    if(DEBUG_MODE) printf("encodeH264_gst_feed_frame was done successfully!\n");
    return true;

copy_error:
    gst_buffer_unmap(buffer, &gst_map);
    gst_buffer_unref(buffer);
    return false;
}

int get_latest_encoded_imgMosaicObj(uint8_t* buffer, uint32_t buffer_size, uint32_t* encoded_size, uint32_t* frame_number, uint32_t* mosiacImg_width, uint32_t* mosiacImg_height)
{
    int status = VX_FAILURE;
    struct timespec timeout;
    pthread_mutex_lock(&g_encoded_frame_mutex);

    int latest_idx = -1;
    uint32_t max_frame_num = 0;
    for (int i = 0; i < ENCODED_FRAME_QUEUE_SIZE; i++) {
        if (g_encoded_frame_queue[i].size > 0 && g_encoded_frame_queue[i].frame_number > max_frame_num) {
            max_frame_num = g_encoded_frame_queue[i].frame_number;
            latest_idx = i;
        }
    }

    if (latest_idx != -1) {
        if (g_encoded_frame_queue[latest_idx].size <= buffer_size) {
            if(DEBUG_MODE) printf("get_latest_encoded_imgMosaicObj found the requested image with frame #%u\n",max_frame_num);
            *encoded_size = g_encoded_frame_queue[latest_idx].size;
            *frame_number = g_encoded_frame_queue[latest_idx].frame_number;
            memcpy(buffer, g_encoded_frame_queue[latest_idx].buffer, *encoded_size);
            *mosiacImg_width  = g_internal_mosaic_width;
            *mosiacImg_height = g_internal_mosaic_height;
            status = VX_SUCCESS;
        } else {
            if(DEBUG_MODE) printf("get_latest_encoded_imgMosaicObj buffer size issue occurred! Actual size is %u while assigned %u \n",g_encoded_frame_queue[latest_idx].size, buffer_size);
            status = VX_FAILURE;
        }
    } else {
        if(DEBUG_MODE) printf("get_latest_encoded_imgMosaicObj could NOT find the requested image!\n");
        clock_gettime(CLOCK_REALTIME, &timeout);
        timeout.tv_nsec += (GET_ENCODED_FRAME_TIMEOUT_MS * 1000000L);
        if (timeout.tv_nsec >= 1000000000L) {
            timeout.tv_sec++;
            timeout.tv_nsec -= 1000000000L;
        }
        pthread_cond_timedwait(&g_encoded_frame_cond, &g_encoded_frame_mutex, &timeout);
    }
    pthread_mutex_unlock(&g_encoded_frame_mutex);
    return (int)status;
}