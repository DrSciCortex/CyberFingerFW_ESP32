/*
 * SPDX-FileCopyrightText: 2026 DrSciCortex
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#pragma once

enum SoundId : int {
    SOUND_BOOT = 0,
    SOUND_SHUTDOWN,
    SOUND_COUNT
};


void pa_on();
void pa_off();

esp_err_t i2s_driver_init(void);

void play_sound(SoundId id, uint32_t delay_ms);

esp_err_t es8311_codec_init(void);
