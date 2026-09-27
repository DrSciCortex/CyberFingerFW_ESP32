/*
 * SPDX-FileCopyrightText: 2026 DrSciCortex
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

// haptics.h — vibration requests from the bridge (SteamVR's haptic events) on
// the DRV2605L + ERM coin motor (CFV1BP boards)
//
// The bridge writes VR_CMD_HAPTIC to the control characteristic (vr_gatt.h).
// That write arrives on the NimBLE host task, which must neither block nor touch
// I2C: the DRV2605L shares the bus with the IMUs the main loop reads. So a request
// is only recorded there (hapticsRequest), and the main loop drives the motor
// (hapticsService), in the DRV2605L's real-time playback mode.
//
// An ERM motor needs a few tens of ms to spin up and a minimum drive to start at
// all: every pulse lasts at least HAPTIC_MIN_PULSE_MS, and a non-zero amplitude
// maps onto [HAPTIC_MIN_DRIVE, HAPTIC_MAX_DRIVE]. SteamVR apps send short pulses
// back to back; each request extends the running vibration.

#pragma once
#include <stdint.h>

class Adafruit_DRV2605;

#define HAPTIC_MIN_PULSE_MS   35     // shorter requests are stretched to this (ERM spin-up)
#define HAPTIC_MAX_PULSE_MS   2000   // one request never runs longer (a lost stop can't buzz forever)
#define HAPTIC_MIN_DRIVE      40     // real-time value where the motor starts to be felt (signed format, 0..127)
#define HAPTIC_MAX_DRIVE      127    // full scale
#define HAPTIC_PULSE_BELOW_HZ 30     // below this frequency the drive is pulsed at it; above, continuous

// After the DRV2605L is set up (or found absent). The motor stays off without it.
void hapticsBegin(Adafruit_DRV2605* drv, bool present);

// A vibration request. amplitude 0..255 (0 stops at once), durationMs, frequencyHz (0: unspecified).
// Any task, never blocks.
void hapticsRequest(uint8_t amplitude, uint16_t durationMs, uint16_t frequencyHz);

// Drive the motor: call every main-loop iteration (the task that owns the I2C bus).
void hapticsService(uint32_t nowMs);
