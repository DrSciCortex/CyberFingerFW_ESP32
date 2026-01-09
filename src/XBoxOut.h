/*
 * SPDX-FileCopyrightText: 2026 DrSciCortex
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#pragma once
#include "IGamepadOut.h"
#include <XboxGamepadDevice.h>

// internal bits (H_A, H_B, ...) live in gamepad_merged.h.
// Include it here if needed for H_* constants, or forward-declare mapping.
#include "gamepad_merged.h"

class XboxOut final : public IGamepadOut {
public:
  explicit XboxOut(XboxGamepadDevice* gp = nullptr) : gp_(gp) {}
  void set(XboxGamepadDevice* gp) { gp_ = gp; }

  void press(uint16_t hidBit) override {
    if (!gp_) return;
    if      (hidBit == H_START)  gp_->press(XBOX_BUTTON_START);
    else if (hidBit == H_SELECT) gp_->press(XBOX_BUTTON_SELECT);
    else if (hidBit == H_A)      gp_->press(XBOX_BUTTON_A);
    else if (hidBit == H_B)      gp_->press(XBOX_BUTTON_B);
    else if (hidBit == H_X)      gp_->press(XBOX_BUTTON_X);
    else if (hidBit == H_Y)      gp_->press(XBOX_BUTTON_Y);
    else if (hidBit == H_RS)     gp_->press(XBOX_BUTTON_RS);
    else if (hidBit == H_RB)     gp_->press(XBOX_BUTTON_RB);
    else if (hidBit == H_LS)     gp_->press(XBOX_BUTTON_LS);
    else if (hidBit == H_LB)     gp_->press(XBOX_BUTTON_LB);
  }

  void release(uint16_t hidBit) override {
    if (!gp_) return;
    if      (hidBit == H_START)  gp_->release(XBOX_BUTTON_START);
    else if (hidBit == H_SELECT) gp_->release(XBOX_BUTTON_SELECT);
    else if (hidBit == H_A)      gp_->release(XBOX_BUTTON_A);
    else if (hidBit == H_B)      gp_->release(XBOX_BUTTON_B);
    else if (hidBit == H_X)      gp_->release(XBOX_BUTTON_X);
    else if (hidBit == H_Y)      gp_->release(XBOX_BUTTON_Y);
    else if (hidBit == H_RS)     gp_->release(XBOX_BUTTON_RS);
    else if (hidBit == H_RB)     gp_->release(XBOX_BUTTON_RB);
    else if (hidBit == H_LS)     gp_->release(XBOX_BUTTON_LS);
    else if (hidBit == H_LB)     gp_->release(XBOX_BUTTON_LB);
  }

  void setLeftThumb(int16_t x, int16_t y) override  { if (gp_) gp_->setLeftThumb(x, y); }
  void setRightThumb(int16_t x, int16_t y) override { if (gp_) gp_->setRightThumb(x, y); }

  void setTriggers(uint8_t lt, uint8_t rt) override {
    if (!gp_) return;
    gp_->setLeftTrigger(lt);
    gp_->setRightTrigger(rt);
  }

  void sendGamepadReport() override { if (gp_) gp_->sendGamepadReport(); }

private:
  XboxGamepadDevice* gp_ = nullptr;
};
