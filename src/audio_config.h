/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#pragma once

#include "sdkconfig.h"

/* Example configurations */
#define AUDIO_RECV_BUF_SIZE   (2400)
#define AUDIO_SAMPLE_RATE     (16000)
#define AUDIO_MCLK_MULTIPLE   (384) // If not using 24-bit data width, 256 should be enough
#define AUDIO_MCLK_FREQ_HZ    (AUDIO_SAMPLE_RATE * AUDIO_MCLK_MULTIPLE)
#define AUDIO_VOICE_VOLUME    CONFIG_AUDIO_VOICE_VOLUME


/* I2C port and GPIOs */
#define I2C_NUM         (0)
#define I2C_SCL_IO      (GPIO_NUM_14)
#define I2C_SDA_IO      (GPIO_NUM_15)
#define GPIO_OUTPUT_PA  (GPIO_NUM_46)

/* I2S port and GPIOs */
#define I2S_NUM         (0)
#define I2S_MCK_IO      (GPIO_NUM_16)
#define I2S_BCK_IO      (GPIO_NUM_9)
#define I2S_WS_IO       (GPIO_NUM_45)
#define I2S_DO_IO       (GPIO_NUM_8)
#define I2S_DI_IO       (GPIO_NUM_10)
