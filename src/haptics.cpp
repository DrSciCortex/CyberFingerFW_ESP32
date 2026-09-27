/*
 * SPDX-FileCopyrightText: 2026 DrSciCortex
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

// haptics.cpp — see haptics.h

#include "haptics.h"
#include <Arduino.h>
#include <Adafruit_DRV2605.h>
#include "HWCDC.h"

extern HWCDC USBSerial;

static Adafruit_DRV2605* s_drv     = nullptr;
static bool              s_present = false;

// The request, shared between the BLE task (writer) and the main loop (reader).
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t  s_amplitude = 0;
static uint16_t s_frequency = 0;
static uint32_t s_startMs   = 0;
static uint32_t s_endMs     = 0;
static bool     s_pending   = false;   // a request came in since the main loop last looked

// The main loop's side.
static bool     s_realtime = false;    // the chip is in real-time playback (a request is running)
static uint8_t  s_drive    = 0;        // the real-time value on the chip
static bool     s_logged   = false;

void hapticsBegin(Adafruit_DRV2605* drv, bool present) {
    s_drv = drv;
    s_present = present && drv != nullptr;
}

void hapticsRequest(uint8_t amplitude, uint16_t durationMs, uint16_t frequencyHz) {
    const uint32_t now = millis();
    uint32_t on = durationMs;
    if (on < HAPTIC_MIN_PULSE_MS) on = HAPTIC_MIN_PULSE_MS;
    if (on > HAPTIC_MAX_PULSE_MS) on = HAPTIC_MAX_PULSE_MS;
    portENTER_CRITICAL(&s_mux);
    if (amplitude == 0) {
        s_endMs = now;                                        // stop
    } else {
        const bool running = s_amplitude != 0 && (int32_t)(s_endMs - now) > 0;
        if (!running) s_startMs = now;
        if (!running || (int32_t)(now + on - s_endMs) > 0) s_endMs = now + on;   // extend, never shorten
        s_frequency = frequencyHz;
    }
    s_amplitude = amplitude;
    s_pending = true;
    portEXIT_CRITICAL(&s_mux);
}

static uint8_t DriveFor(uint8_t amplitude) {
    if (amplitude == 0) return 0;
    return (uint8_t)(HAPTIC_MIN_DRIVE + ((uint32_t)(HAPTIC_MAX_DRIVE - HAPTIC_MIN_DRIVE) * amplitude + 127) / 255);
}

void hapticsService(uint32_t nowMs) {
    if (!s_present) return;
    uint8_t amplitude;
    uint16_t frequency;
    uint32_t startMs, endMs;
    bool pending;
    portENTER_CRITICAL(&s_mux);
    amplitude = s_amplitude;
    frequency = s_frequency;
    startMs = s_startMs;
    endMs = s_endMs;
    pending = s_pending;
    s_pending = false;
    portEXIT_CRITICAL(&s_mux);
    const bool active = amplitude != 0 && (int32_t)(endMs - nowMs) > 0;
    if (pending && active && !s_logged) {
        s_logged = true;
        USBSerial.printf("[HAPTICS] first request from the bridge: amplitude %u, %u ms, %u Hz\n",
                         amplitude, (unsigned)(endMs - nowMs), frequency);
    }

    uint8_t drive = 0;
    if (active) {
        drive = DriveFor(amplitude);
        // Low frequencies are felt as a pulse train: on for the first half of each period.
        if (frequency > 0 && frequency < HAPTIC_PULSE_BELOW_HZ &&
            (((nowMs - startMs) * 2u * frequency / 1000u) & 1u))
            drive = 0;
        if (!s_realtime) {                                    // starting: from effect playback to real time
            s_drv->setMode(DRV2605_MODE_REALTIME);
            s_realtime = true;
            s_drive = 0;
        }
    }
    if (drive != s_drive) {
        s_drv->setRealtimeValue(drive);
        s_drive = drive;
    }
    if (!active && s_realtime) {                              // done: back to effect playback (playEffect)
        s_drv->setMode(DRV2605_MODE_INTTRIG);
        s_realtime = false;
    }
}
