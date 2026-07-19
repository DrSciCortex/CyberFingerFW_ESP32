/*
 * SPDX-FileCopyrightText: 2026 DrSciCortex
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#pragma once

#include <Arduino.h>

// Onboard QMI8658 6-axis IMU (I2C 0x6B). This is the secondary body sensor,
// redundant with the ICM-45686 at 0x69. Mirrors the icm45686_handler API so
// the two can be driven identically from the main loop.

/**
 * @brief Initialize the QMI8658. Safe to call when the part is absent.
 * @return true if the sensor was found and configured.
 */
bool qmi8658_init();

/**
 * @brief Update the internal fusion filter. Call frequently from the loop.
 *        No-op until the sensor signals a fresh sample, so it is cheap to
 *        call faster than the sensor's ODR.
 */
void qmi8658_update();

/**
 * @brief Get the current orientation as a quaternion.
 * @param q Array to store [w, x, y, z].
 */
void qmi8658_get_quat(float q[4]);
