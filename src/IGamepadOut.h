/*
 * SPDX-FileCopyrightText: 2026 DrSciCortex
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#pragma once
#include <stdint.h>

// Generic gamepad interface that GamepadMerged needs.
struct IGamepadOut {
  virtual ~IGamepadOut() = default;

  virtual void press(uint16_t hidBit) = 0;
  virtual void release(uint16_t hidBit) = 0;

  virtual void setLeftThumb(int16_t x, int16_t y) = 0;
  virtual void setRightThumb(int16_t x, int16_t y) = 0;

  virtual void setTriggers(uint8_t lt, uint8_t rt) = 0;

  // Optional: if later dpad/hats
  // virtual void setDpad(uint8_t dpad) = 0;

  virtual void sendGamepadReport() = 0;
};
