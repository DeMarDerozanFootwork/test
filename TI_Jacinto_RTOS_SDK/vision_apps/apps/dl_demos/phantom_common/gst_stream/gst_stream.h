#ifndef _GST_STREAM_H_
#define _GST_STREAM_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <VX/vx.h>
#include <TI/tivx.h>

#ifdef __cplusplus
extern "C" {
#endif

bool encodeH264_send_by_gst_init(const char *pipeline_str, int width, int height, int fps);
bool encodeH264_send_by_gst(uint8_t *data, size_t size);
void encodeH264_send_by_gst_deinit();

bool encodeH264_gst_init(const char *pipeline_str, bool enabled, int fps, int mosaic_width, int mosaic_height);
void encodeH264_gst_deinit();
bool encodeH264_gst_feed_frame(vx_image mosaic_image, uint32_t frame_number);
int get_latest_encoded_imgMosaicObj(uint8_t* buffer, uint32_t buffer_size, uint32_t* encoded_size, uint32_t* frame_number, uint32_t* mosiacImg_width, uint32_t* mosiacImg_height);
#ifdef __cplusplus
}
#endif

#endif //_GST_STREAM_H_

