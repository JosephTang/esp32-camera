/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * OV7725 driver.
 *
 */
#include "gc0328.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gc0328_settings.h"
#include "sccb.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char *TAG = "gc0328";
#endif

static int get_reg(sensor_t *sensor, int reg, int mask)
{
    int ret = SCCB_Read(sensor->slv_addr, reg & 0xFF);
    if (ret > 0) {
        ret &= mask;
    }
    return ret;
}

static int set_reg(sensor_t *sensor, int reg, int mask, int value)
{
    int ret = 0;
    ret = SCCB_Read(sensor->slv_addr, reg & 0xFF);
    if (ret < 0) {
        return ret;
    }
    value = (ret & ~mask) | (value & mask);
    ret = SCCB_Write(sensor->slv_addr, reg & 0xFF, value);
    return ret;
}

static int set_reg_bits(sensor_t *sensor, uint8_t reg, uint8_t offset, uint8_t length, uint8_t value)
{
    int ret = 0;
    ret = SCCB_Read(sensor->slv_addr, reg);
    if (ret < 0) {
        return ret;
    }
    uint8_t mask = ((1 << length) - 1) << offset;
    value = (ret & ~mask) | ((value << offset) & mask);
    ret = SCCB_Write(sensor->slv_addr, reg & 0xFF, value);
    return ret;
}

static int get_reg_bits(sensor_t *sensor, uint8_t reg, uint8_t offset, uint8_t length)
{
    int ret = 0;
    ret = SCCB_Read(sensor->slv_addr, reg);
    if (ret < 0) {
        return ret;
    }
    uint8_t mask = ((1 << length) - 1) << offset;
    return (ret & mask) >> offset;
}

static int reset(sensor_t *sensor)
{
    for (int i = 0; i < sizeof(sensor_default_regs) / 2; i++) {
        SCCB_Write(sensor->slv_addr, sensor_default_regs[i][0], sensor_default_regs[i][1]);
    }

    return 0;
}

static int set_pixformat(sensor_t *sensor, pixformat_t pixformat)
{
    int ret = 0;
    sensor->pixformat = pixformat;

    SCCB_Write(sensor->slv_addr, 0xFE, 0x00); // REGF select

    uint8_t reg = SCCB_Read(sensor->slv_addr, 0x44);

    switch (pixformat) {
        case PIXFORMAT_RGB565:
            reg = (reg & 0xE0) | 0x06;
            break;
        case PIXFORMAT_YUV422:
            reg = (reg & 0xE0) | 0x06;
            break;
        default:
            return -1;
    }

    // Write back register COM7
    ret = SCCB_Write(sensor->slv_addr, 0x44, reg);

    // Delay
    vTaskDelay(30 / portTICK_PERIOD_MS);

    return ret;
}

static int set_framesize(sensor_t *sensor, framesize_t framesize)
{
    int ret = 0;

    switch (framesize) {
        case FRAMESIZE_QVGA:
            for (int i = 0; i < sizeof(sensor_framesize_QVGA_regs) / 2; i++) {
                SCCB_Write(sensor->slv_addr, sensor_framesize_QVGA_regs[i][0], sensor_framesize_QVGA_regs[i][1]);
            }
            break;
        case FRAMESIZE_VGA:
            for (int i = 0; i < sizeof(sensor_framesize_QVGA_regs) / 2; i++) {
                SCCB_Write(sensor->slv_addr, sensor_framesize_QVGA_regs[i][0], sensor_framesize_QVGA_regs[i][1]);
            }
            break;
        default:
            ESP_LOGE(TAG, "framesize unsupported!");
            ret = -1;
    }

    // Delay
    vTaskDelay(30 / portTICK_PERIOD_MS);

    return ret;
}

static int set_colorbar(sensor_t *sensor, int enable)
{
    int ret = 0;

    return ret;
}

static int set_whitebal(sensor_t *sensor, int enable)
{
    SCCB_Write(sensor->slv_addr, 0xFE, 0x00);
    if (set_reg_bits(sensor, 0x42, 1, 1, enable) >= 0) {
        sensor->status.awb = !!enable;
    }
    return sensor->status.awb;
}

static int set_gain_ctrl(sensor_t *sensor, int enable)
{
    return sensor->status.agc;
}

static int set_exposure_ctrl(sensor_t *sensor, int enable)
{
    return sensor->status.aec;
}

static int set_hmirror(sensor_t *sensor, int enable)
{
    SCCB_Write(sensor->slv_addr, 0xFE, 0x00);
    if (set_reg_bits(sensor, 0x17, 0, 1, enable) >= 0) {
        sensor->status.hmirror = !!enable;
    }
    return sensor->status.hmirror;
}

static int set_vflip(sensor_t *sensor, int enable)
{
    SCCB_Write(sensor->slv_addr, 0xFE, 0x00);
    if (set_reg_bits(sensor, 0x17, 1, 1, enable) >= 0) {
        sensor->status.vflip = !!enable;
    }
    return sensor->status.vflip;
}

static int set_dcw_dsp(sensor_t *sensor, int enable)
{
    int ret = 0;
    return ret;
}

static int set_aec2(sensor_t *sensor, int enable)
{
    int ret = 0;
    return ret;
}

static int set_bpc_dsp(sensor_t *sensor, int enable)
{
    int ret = 0;
    return ret;
}

static int set_wpc_dsp(sensor_t *sensor, int enable)
{
    int ret = 0;
    return ret;
}

static int set_raw_gma_dsp(sensor_t *sensor, int enable)
{
    int ret = 0;
    return ret;
}

static int set_lenc_dsp(sensor_t *sensor, int enable)
{
    int ret = 0;
    return ret;
}

//real gain
static int set_agc_gain(sensor_t *sensor, int gain)
{
    int ret = 0;
    return ret;
}

static int set_aec_value(sensor_t *sensor, int value)
{
    int ret = 0;
    return ret;
}

static int set_awb_gain_dsp(sensor_t *sensor, int enable)
{
    int ret = 0;
    return ret;
}

static int set_brightness(sensor_t *sensor, int level)
{
    int ret = 0;
    return ret;
}

static int set_contrast(sensor_t *sensor, int level)
{
    int ret = 0;
    return ret;
}

static int init_status(sensor_t *sensor)
{
    reset(sensor);
    return 0;
}

static int set_dummy(sensor_t *sensor, int val) { return -1; }
static int set_gainceiling_dummy(sensor_t *sensor, gainceiling_t val) { return -1; }
static int set_res_raw(sensor_t *sensor, int startX, int startY, int endX, int endY, int offsetX, int offsetY, int totalX, int totalY, int outputX, int outputY, bool scale, bool binning) { return -1; }
static int _set_pll(sensor_t *sensor, int bypass, int multiplier, int sys_div, int root_2x, int pre_div, int seld5, int pclk_manual, int pclk_div) { return -1; }

esp_err_t xclk_timer_conf(int ledc_timer, int xclk_freq_hz);
static int set_xclk(sensor_t *sensor, int timer, int xclk)
{
    int ret = 0;
    sensor->xclk_freq_hz = xclk * 1000000U;
    ret = xclk_timer_conf(timer, sensor->xclk_freq_hz);
    return ret;
}

int gc0328_init(sensor_t *sensor)
{
    // Set function pointers
    sensor->reset = reset;
    sensor->init_status = init_status;
    sensor->set_pixformat = set_pixformat;
    sensor->set_framesize = set_framesize;
    sensor->set_colorbar = set_colorbar;
    sensor->set_whitebal = set_whitebal;
    sensor->set_gain_ctrl = set_gain_ctrl;
    sensor->set_exposure_ctrl = set_exposure_ctrl;
    sensor->set_hmirror = set_hmirror;
    sensor->set_vflip = set_vflip;

    sensor->set_brightness = set_brightness;
    sensor->set_contrast = set_contrast;
    sensor->set_aec2 = set_aec2;
    sensor->set_aec_value = set_aec_value;
    sensor->set_awb_gain = set_awb_gain_dsp;
    sensor->set_agc_gain = set_agc_gain;
    sensor->set_dcw = set_dcw_dsp;
    sensor->set_bpc = set_bpc_dsp;
    sensor->set_wpc = set_wpc_dsp;
    sensor->set_raw_gma = set_raw_gma_dsp;
    sensor->set_lenc = set_lenc_dsp;

    //not supported
    sensor->set_saturation = set_dummy;
    sensor->set_sharpness = set_dummy;
    sensor->set_denoise = set_dummy;
    sensor->set_quality = set_dummy;
    sensor->set_special_effect = set_dummy;
    sensor->set_wb_mode = set_dummy;
    sensor->set_ae_level = set_dummy;
    sensor->set_gainceiling = set_gainceiling_dummy;

    sensor->get_reg = get_reg;
    sensor->set_reg = set_reg;
    sensor->set_res_raw = set_res_raw;
    sensor->set_pll = _set_pll;
    sensor->set_xclk = set_xclk;

    // Retrieve sensor's signature
    sensor->id.PID = SCCB_Read(sensor->slv_addr, 0xf0);

    ESP_LOGI(TAG, "gc0328 Attached");

    return 0;
}
