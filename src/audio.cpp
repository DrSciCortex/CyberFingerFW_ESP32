/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "esp_system.h"
#include "esp_check.h"
#include "esp_log.h"
#include <stdarg.h>
#include "es8311.h"
#include "audio_config.h"
#include "driver/gpio.h"
//#include "driver/i2c.h"
#include <Arduino.h>
#include <Wire.h>
#include "audio.h"


#include "HWCDC.h"
extern HWCDC USBSerial;

static const char *TAG = "i2s_es8311";
static const char err_reason[][30] = {"input param is invalid",
                                      "operation timeout"
                                     };
static i2s_chan_handle_t tx_handle = NULL;
static i2s_chan_handle_t rx_handle = NULL;

/* Import sound files as buffer */

//extern const uint8_t music_pcm_start[] asm("_binary_src_assets_canon_pcm_start");
//extern const uint8_t music_pcm_end[]   asm("_binary_src_assets_canon_pcm_end");

extern const uint8_t boot_pcm_start[] asm("_binary_src_assets_cyberboot_pcm_start");
extern const uint8_t boot_pcm_end[]   asm("_binary_src_assets_cyberboot_pcm_end");
extern const uint8_t shutdown_pcm_start[] asm("_binary_src_assets_cybershutdown_pcm_start");
extern const uint8_t shutdown_pcm_end[]   asm("_binary_src_assets_cybershutdown_pcm_end");



struct PcmClip {
    const uint8_t* start;
    const uint8_t* end;
};

// Table of all sounds, indexable by SoundId
const PcmClip kSoundClips[SOUND_COUNT] = {
    { boot_pcm_start,     boot_pcm_end     }, // SOUND_BOOT
    { shutdown_pcm_start, shutdown_pcm_end }, // SOUND_SHUTDOWN
};

volatile bool soundBusy = false;

void pa_on() {
    pinMode(GPIO_OUTPUT_PA, OUTPUT);   // define it as output
    digitalWrite(GPIO_OUTPUT_PA, HIGH); // turn PA on
}

void pa_off() {
    //pinMode(GPIO_OUTPUT_PA, OUTPUT);   // define it as output
    digitalWrite(GPIO_OUTPUT_PA, LOW); // turn PA on
}

esp_err_t i2s_driver_init(void)
{

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG((i2s_port_t)I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle));
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_MCK_IO,
            .bclk = I2S_BCK_IO,
            .ws = I2S_WS_IO,
            .dout = I2S_DO_IO,
            .din = I2S_DI_IO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    std_cfg.clk_cfg.mclk_multiple = (i2s_mclk_multiple_t)AUDIO_MCLK_MULTIPLE;

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

    return ESP_OK;
}

static void i2s_play_simple(const uint8_t *start, const uint8_t *end) 
{

    esp_err_t ret = ESP_OK;
    size_t bytes_write = 0;
    //uint8_t *data_ptr = (uint8_t *)start;

    /* Enable the TX channel */
    //ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
    /* Write music to earphone */
    ret = i2s_channel_write(tx_handle, start, end - start, &bytes_write, portMAX_DELAY);
    if (ret != ESP_OK) {
        /* Since we set timeout to 'portMAX_DELAY' in 'i2s_channel_write'
            so you won't reach here unless you set other timeout value,
            if timeout detected, it means write operation failed. */
        ESP_LOGE(TAG, "[audio] i2s write failed, %s", err_reason[ret == ESP_ERR_TIMEOUT]);
        abort();
    }
    if (bytes_write > 0) {
        ESP_LOGI(TAG, "[audio] i2s sound played, %d bytes are written.", bytes_write);
    } else {
        ESP_LOGE(TAG, "[audio] i2s sound play failed.");
        abort();
    }
 
}

void i2s_play_clip(const PcmClip& clip) {
    i2s_play_simple(clip.start, clip.end);  
}

struct PlaySoundArgs {
    SoundId id;
    uint32_t delay_ms;
};

static void play_sound_task(void* arg)
{
    PlaySoundArgs* args = static_cast<PlaySoundArgs*>(arg);

    // Optional delay before playback
    if (args->delay_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(args->delay_ms));
    }

    SoundId id = args->id;

    if (id >= 0 && id < SOUND_COUNT) {
        const PcmClip& clip = kSoundClips[id];
        i2s_play_clip(clip);
    } else {
        ESP_LOGE("SOUND", "Invalid sound id: %d", (int)id);
    }

    soundBusy = false;
    delete args;              // free heap
    vTaskDelete(NULL);
}


// Public API: call from anywhere
void play_sound(SoundId id, uint32_t delay_ms = 0)
{
    if (soundBusy) {
        ESP_LOGI("SOUND", "Sound busy, skipping id=%d", (int)id);
        return;
    }

    soundBusy = true;

    auto* args = new PlaySoundArgs{
        .id = id,
        .delay_ms = delay_ms
    };

    xTaskCreate(
        play_sound_task,
        "play_sound",
        4096,
        args,
        5,
        nullptr
    );
}

esp_err_t es8311_codec_init(void)
{
    // DO NOT configure IDF I2C here; we already did Wire.begin(...)
    // Just create the codec handle and run the Espressif init sequence.

    es8311_handle_t es_handle = es8311_create(0 /*port (unused now)*/, ES8311_ADDRRES_0);
    if (!es_handle) {
        ESP_LOGE(TAG, "es8311 create failed");
        return ESP_FAIL;
    }

    const es8311_clock_config_t es_clk = {
        .mclk_inverted     = false,
        .sclk_inverted     = false,
        .mclk_from_mclk_pin = true,
        .mclk_frequency    = AUDIO_MCLK_FREQ_HZ,   // e.g. 12288000 or whatever your I2S MCLK is
        .sample_frequency  = AUDIO_SAMPLE_RATE     // e.g. 16000 / 48000
    };

    esp_err_t err;

    err = es8311_init(es_handle, &es_clk,
                      ES8311_RESOLUTION_16, ES8311_RESOLUTION_16);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "es8311_init failed: %s", esp_err_to_name(err));
        return err;
    }

    // Optional but matches example: explicitly configure sample frequency again
    err = es8311_sample_frequency_config(es_handle,
                                         AUDIO_MCLK_FREQ_HZ,
                                         AUDIO_SAMPLE_RATE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "es8311_sample_frequency_config failed: %s", esp_err_to_name(err));
        return err;
    }

    // Set output volume in 0–100% (example constant)
    int actual_vol = 80;
    err = es8311_voice_volume_set(es_handle, AUDIO_VOICE_VOLUME, &actual_vol);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "es8311_voice_volume_set failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "ES8311 volume set to %d%%", actual_vol);

    ESP_LOGI(TAG, "es8311 codec init success");
    return ESP_OK;
}

int usbserial_log_vprintf(const char *fmt, va_list args) {
    // Small buffer is usually enough; bump if you log long lines
    char buf[256];
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    if (len > 0) {
        USBSerial.print(buf);      // or Serial.print(buf);
    }
    return len;
}

/*

void setup(void)
{

    USBSerial.begin(115200);
    while (!USBSerial && millis() < 2000) { delay(10); }

    // Send all ESP_LOGx output to USBSerial
    esp_log_set_vprintf(usbserial_log_vprintf);

    // Optionally set log level (INFO, DEBUG, VERBOSE, etc.)
    //esp_log_level_set("*", ESP_LOG_VERBOSE);   // or ESP_LOGD / ESP_LOGV
    //set in platformio.ini
    // build_flags =-DCORE_DEBUG_LEVEL=XYZ

    delay(2000);

    ESP_LOGE(TAG, "i2s es8311 codec example start");

    // Make sure Wire is running on the correct pins.
    // (If you already called Wire.begin(IIC_SDA, IIC_SCL) elsewhere,
    //  this will just reconfigure it consistently.)
    Wire.begin(I2C_SDA_IO, I2C_SCL_IO);
    Wire.setClock(400000);  // ES8311 is fine up to 400 kHz

    pa_init();

    ESP_LOGI(TAG, "Wire and gpio init suceeded");

    if (i2s_driver_init() != ESP_OK) {
        ESP_LOGE(TAG, "i2s driver init failed");
        abort();
    }
    else {
        ESP_LOGI(TAG, "i2s driver init suceeded");

    }

    if (es8311_codec_init() != ESP_OK) {
        ESP_LOGE(TAG, "es8311 codec init failed");
        abort();
    }
        else {
        ESP_LOGI(TAG, "es8311 codec init suceeded");

    }

    //xTaskCreate(i2s_music, "i2s_music", 4096, NULL, 5, NULL);
    play_sound(SOUND_BOOT);

    delay(3000);

    play_sound(SOUND_SHUTDOWN);
    
    

}

void loop() {
    
    // On boot complete:
    play_sound(SOUND_BOOT);

    delay(3000);

    // On shut-down event (or “power off” button):
    play_sound(SOUND_SHUTDOWN);

    delay(3000);
}

*/
