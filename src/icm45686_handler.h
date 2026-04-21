/*
 * SPDX-FileCopyrightText: 2026 DrSciCortex
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#pragma once

#include <Arduino.h>

/**
 * @brief Initialize the ICM-45686 sensor.
 * @return true if success, false otherwise.
 */
bool icm45686_init();

/**
 * @brief Update the internal fusion filter. Should be called frequently in the loop.
 */
void icm45686_update();

/**
 * @brief Get the current orientation as a quaternion.
 * @param q Array to store [w, x, y, z].
 */
void icm45686_get_quat(float q[4]);
