/*
 * SPDX-FileCopyrightText: 2026 DrSciCortex
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#pragma once

#include <Arduino.h>

// The board can carry two ICM-45686, strapped to different addresses via
// AP_AD0. Each gets its own fusion state, so they are addressed by instance
// rather than the driver guessing which one it found.
//
// Address-to-role mapping is a BOARD WIRING FACT, not something the driver can
// detect: both parts are identical and report the same WHO_AM_I. If a unit is
// built with the straps the other way round, swap these two addresses.
enum IcmInstance : uint8_t {
    ICM_BODY  = 0,   // primary body sensor
    ICM_JOINT = 1,   // optional joint sensor
    ICM_COUNT = 2,
};

#define ICM_ADDR_BODY   0x69
#define ICM_ADDR_JOINT  0x68

/**
 * @brief Initialize one ICM-45686.
 * @param instance Which slot to bind (ICM_BODY / ICM_JOINT).
 * @param addr     I2C address for that slot.
 * @return true if the part answered and was configured.
 */
bool icm45686_init(uint8_t instance, uint8_t addr);

/**
 * @brief Update one instance's fusion filter. Call frequently from the loop.
 *        Safe (and free) to call on an instance that failed to initialize.
 */
void icm45686_update(uint8_t instance);

/**
 * @brief Get one instance's orientation as a quaternion [w, x, y, z].
 */
void icm45686_get_quat(uint8_t instance, float q[4]);

/**
 * @brief WHO_AM_I read during init, for diagnostics. 0 if the read failed.
 */
uint8_t icm45686_whoami(uint8_t instance);
