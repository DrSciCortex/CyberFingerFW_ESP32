/*
 * SPDX-FileCopyrightText: 2026 DrSciCortex
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#pragma once
#include "IGamepadOut.h"
#include <GamepadDevice.h>
#include "gamepad_merged.h"

class GenericOut final : public IGamepadOut {
public:
  explicit GenericOut(GamepadDevice* gp = nullptr) : gp_(gp) {}
  void set(GamepadDevice* gp) { gp_ = gp; }

  void press(uint16_t hidBit) override {
    if (!gp_) return;
    int btn = mapButton_(hidBit);
    if (btn > 0) gp_->press((uint8_t)btn);
  }

  void release(uint16_t hidBit) override {
    if (!gp_) return;
    int btn = mapButton_(hidBit);
    if (btn > 0) gp_->release((uint8_t)btn);
  }

  void setLeftThumb(int16_t x, int16_t y) override {
    if (!gp_) return;
    // --- You may need to adjust these calls to match GamepadDevice API ---
    // Common patterns in these libraries:
    // 1) gp_->setLeftThumb(x, y);
    // 2) gp_->setAxes(x, y, rx, ry, lt, rt);
    // 3) gp_->setX(x); gp_->setY(y);
    setLeft_(x, y);
  }

  void setRightThumb(int16_t x, int16_t y) override {
    if (!gp_) return;
    setRight_(x, y);
  }

  void setTriggers(uint8_t lt, uint8_t rt) override {
    if (!gp_) return;
    setTriggers_(lt, rt);
  }

  void sendGamepadReport() override {
    if (!gp_) return;
    // Some variants call sendReport(), some sendGamepadReport().
    gp_->sendGamepadReport();
  }

private:
  GamepadDevice* gp_ = nullptr;

  // Map your semantic buttons to BUTTON_1.. etc.
  // Choose any stable mapping; keep it consistent with host-side bindings.
  static int mapButton_(uint16_t hidBit) {
    if      (hidBit == H_A)      return BUTTON_1;
    else if (hidBit == H_B)      return BUTTON_2;
    else if (hidBit == H_X)      return BUTTON_3;
    else if (hidBit == H_Y)      return BUTTON_4;
    else if (hidBit == H_LB)     return BUTTON_5;
    else if (hidBit == H_RB)     return BUTTON_6;
    else if (hidBit == H_SELECT) return BUTTON_7;
    else if (hidBit == H_START)  return BUTTON_8;
    else if (hidBit == H_LS)     return BUTTON_9;
    else if (hidBit == H_RS)     return BUTTON_10;
    return -1;
  }

  // ---- These 3 helpers are where API drift is isolated ----
  // Replace bodies to match Mystfit/GamepadDevice.h exactly.
  void setLeft_(int16_t x, int16_t y)  { gp_->setLeftThumb(x, y); }
  void setRight_(int16_t x, int16_t y) { gp_->setRightThumb(x, y); }
  void setTriggers_(uint8_t lt, uint8_t rt) {
    gp_->setLeftTrigger(lt);
    gp_->setRightTrigger(rt);
  }
};
