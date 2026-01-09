/*
 * SPDX-FileCopyrightText: 2026 DrSciCortex
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#pragma once
#include <Arduino.h>
#include <stdint.h>

// Button mask
#ifndef TM_MOUSE_BTN_LEFT
#define TM_MOUSE_BTN_LEFT 0x01
#endif

// User-supplied callbacks
typedef void (*TM_MoveFn)(int dx, int dy, void* user);
typedef void (*TM_PressFn)(uint8_t buttons, void* user);
typedef void (*TM_ReleaseFn)(uint8_t buttons, void* user);
typedef void (*TM_ScrollFn)(int v, int h, void* user);

struct TM_Callbacks {
  TM_MoveFn    move    = nullptr;
  TM_PressFn   press   = nullptr;
  TM_ReleaseFn release = nullptr;
  TM_ScrollFn  scroll  = nullptr;
  void*        user    = nullptr;
};

class TouchMouse {
public:
  TouchMouse() = default;

  void begin(const TM_Callbacks& cb);

  // Call this with latest touch sample (primary point only).
  void handleSample(bool hasPoint, uint16_t x, uint16_t y);

    // Multi-touch: pass touches=0,1,2 (>=2 treated as 2)
  // x2/y2 ignored unless touches >= 2
  void handleSampleMulti(uint8_t touches,
                         uint16_t x1, uint16_t y1,
                         uint16_t x2 = 0, uint16_t y2 = 0);

  void reset();

  // axis inversion flags you can set directly
  bool invertX = false;
  bool invertY = false;

    // Scroll direction inversions (like OS settings: "natural" scroll)
  bool invertScrollV = false;
  bool invertScrollH = false;

private:
  TM_Callbacks _cb;

  // --- cursor/gesture state (one-finger path) ---
  bool     _touchDown   = false;   // finger currently down?
  bool     _dragging    = false;   // left button held?
  bool     _armedClick  = false;   // still eligible to be a tap click
  bool     _secondTap   = false;   // in second tap of double-tap-hold
  uint16_t _prevX = 0, _prevY = 0;
  uint16_t _downX = 0, _downY = 0;
  uint32_t _downAtMs = 0;
  uint32_t _secondDownAtMs = 0;
  uint32_t _lastUpAtMs = 0;
  uint32_t _lastSendAtUS = 0;

  // --- new: lift-hold-to-drag priming (after a move) ---
  bool     _primeLiftHold = false;
  uint32_t _primeExpireMs = 0;
  uint16_t _primeX = 0, _primeY = 0;

  // --- scrolling state (two-finger path) ---
  bool     _scrolling    = false;
  uint16_t _prev2X = 0, _prev2Y = 0;   // previous 2-finger *average* pos
  int      _accumV = 0;                // accumulated px for vertical scroll
  int      _accumH = 0;                // accumulated px for horizontal scroll

  void sendMove(int dx, int dy);
  void pressLeft();
  void releaseLeft();

    // Core 1-finger handler used by handleSampleMulti when touches==1
  void handleOneFinger(bool hasPoint, uint16_t x, uint16_t y);

  // Two-finger scroll handling
  void handleTwoFingers(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
};
