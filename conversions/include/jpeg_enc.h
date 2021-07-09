#include "stdbool.h"
#include "stdint.h"

#ifndef __JPEG_ENC_H__
#define __JPEG_ENC_H__

/* JPEG chroma subsampling factors */
typedef enum {
    JPEG_SUB_SAMPLE_Y = 0,    /*!< Grayscale */
    JPEG_SUB_SAMPLE_YUV = 1,  /*!< YCbCr 1x1x1   3 pixel per MCU  Y444*/
    JPEG_SUB_SAMPLE_YUYV = 2, /*!< YCbYCr 2x1x1  2 pixel per MCU  Y422*/
    JPEG_SUB_SAMPLE_YYUV = 3  /*!< YYCbCr 4x1x1  1.25 pixel per MCU  Y420)*/
} jpeg_enc_subsampling_t;

/* input stream type */
typedef enum {
    JPEG_ENC_SRC_TYPE_GRAY = 0,   /*!< Grayscale */
    JPEG_ENC_SRC_TYPE_RGB = 1,    /*!< RGB*/
    JPEG_ENC_SRC_TYPE_RGBA = 2,   /*!< RGBA*/
    JPEG_ENC_SRC_TYPE_YCbYCr = 3, /*!< Y422*/
} jpeg_enc_src_type_t;

/* JPEG configure information*/
typedef struct jpeg_info {
    int width;                          /*!< Image wdith */
    int height;                         /*!< Image height */
    jpeg_enc_src_type_t src_type;       /*!< Input image type */
    jpeg_enc_subsampling_t subsampling; /*!< JPEG chroma subsampling factors.*/
    uint8_t quality;                      /*!< Quality: 1-100, higher is better. Typical values are around 40 - 100. */
    uint8_t hfm_task_priority;
    uint8_t hfm_task_core;
} jpeg_enc_info_t;

/**
 * @brief      Create an JPEG handle to encode
 *
 * @param      info  The configuration information
 *
 * @return     other values: The JPEG encoder handle
 *             NULL: failed
 */
void *jpeg_enc_open(jpeg_enc_info_t *info);

/**
 * @brief      Encode one image
 *
 * @param      handle      The JPEG encoder handle. It gained from `jpeg_enc_open`
 * @param      in_buf      The input buffer, It needs a completed image.
 * @param      inbuf_size  The size of `in_buf`. The value must be size of a completed image.
 * @param      out_buf     The output buffer, It saves a completed JPEG image. the size must be gather than 700 bytes.
 * @param      outbuf_size The size of output buffer
 * @param      out_size    The size of JPEG image
 *
 * @return     0     It has finished to encode.
 */
int jpeg_enc_process(const void *handle, const uint8_t *in_buf, int inbuf_size, uint8_t *out_buf, int outbuf_size, int *out_size);

/**
 * @brief      Get block size. Block size is minimum process unit.
 *
 * @param      handle      The JPEG encoder handle. It gained from `jpeg_enc_open`
 *
 * @return     positive value     block size
 */
int jpeg_enc_get_block_size(const void *handle);

/**
 * @brief      Encode block size image. Get block size from  `jpeg_enc_get_block_size`
 *
 * @param      handle      The JPEG encoder handle. It gained from `jpeg_enc_open`
 * @param      in_buf      The input buffer, It needs a completed image.
 * @param      inbuf_size  The size of `in_buf`. Get block size from  `jpeg_enc_get_block_size`
 * @param      out_buf     The output buffer, It saves a completed JPEG image. the size must be gather than 700 bytes.
 * @param      outbuf_size The size of output buffer
 * @param      out_size    The size of JPEG image
 *
 * @return     0     It has finished to encode one image.
 *             block size  It has finished to encode current block size image.
 *             -1    Encoder failed
 */
int jpeg_enc_process_with_block(const void *handle, const uint8_t *in_buf, int inbuf_size, uint8_t *out_buf, int outbuf_size, int *out_size);

/**
 * @brief      Deinit JPEG handle
 *
 * @param      handle      The JPEG encoder handle. It gained from `jpeg_enc_open`
 *
 * @return     0     It has finished to deinit.
 */
int jpeg_enc_close(void *handle);

#endif
