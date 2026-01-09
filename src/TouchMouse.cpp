/*
 * SPDX-FileCopyrightText: 2026 DrSciCortex
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include "TouchMouse.h"

// ---- Tunables ----
static const uint16_t TAP_MOVE_PX         = 10;   // max move to still count as a tap
static const uint16_t TAP_MAX_MS          = 180;  // max press length for a tap
static const uint16_t DOUBLE_TAP_GAP_MS   = 250;  // max gap between two taps
static const uint16_t DT_HOLD_TO_DRAG_MS  = 200;  // second-tap hold before drag
// Convert pixel motion to scroll notches: emit 1 notch per N pixels
static const uint16_t SCROLL_PX_PER_NOTCH = 24;   // adjust to taste

// ---- NEW: “lift near here, hold to drag” tuning ----
static const uint16_t LIFT_HOLD_GAP_MS    = 400; // time after lift to accept hold as drag
static const uint16_t LIFT_HOLD_RADIUS_PX = 14;  // how close the new touch must be to last point

// Don't send moves more frequently than this
constexpr uint32_t MIN_INTERVAL_US = 4000;

void TouchMouse::begin(const TM_Callbacks& cb) {
  _cb = cb;
  reset();
}

void TouchMouse::reset() {
  _touchDown = false;
  _dragging  = false;
  _armedClick = false;
  _secondTap  = false;
  _prevX = _prevY = 0;
  _downX = _downY = 0;
  _downAtMs = 0;
  _secondDownAtMs = 0;
  // keep _lastUpAtMs to preserve double-tap gap if desired

  _primeLiftHold = false;
  _primeExpireMs = 0;
  _primeX = _primeY = 0;

  _scrolling = false;
  _prev2X = _prev2Y = 0;
  _accumV = _accumH = 0;
  _lastSendAtUS = micros();
}

void TouchMouse::sendMove(int dx, int dy) {
  if (invertX) dx = -dx;
  if (invertY) dy = -dy;
  if (_cb.move) _cb.move(dx, dy, _cb.user);
}

void TouchMouse::pressLeft() { 
  if (_cb.press)   _cb.press(TM_MOUSE_BTN_LEFT, _cb.user); 
}
void TouchMouse::releaseLeft() { 
  if (_cb.release) _cb.release(TM_MOUSE_BTN_LEFT, _cb.user); 
}

void TouchMouse::handleSample(bool hasPoint, uint16_t x, uint16_t y) {
  handleSampleMulti(hasPoint ? 1 : 0, x, y, 0, 0);
}

void TouchMouse::handleSampleMulti(uint8_t touches,
                                   uint16_t x1, uint16_t y1,
                                   uint16_t x2, uint16_t y2) {
  if (touches >= 2) {
    handleTwoFingers(x1, y1, x2, y2);
    return;
  }
  // If we were scrolling and fingers reduced to 1 or 0, stop scrolling state
  if (_scrolling) {
    _scrolling = false;
    _accumV = _accumH = 0;
  }
  handleOneFinger(touches == 1, x1, y1);
}

void TouchMouse::handleOneFinger(bool hasPoint, uint16_t x, uint16_t y) {
  uint32_t now = millis();

  // ---- DOWN
  if (hasPoint && !_touchDown) {
    _touchDown = true;
    _downX = _prevX = x;
    _downY = _prevY = y;
    _downAtMs = now;
    _armedClick = true;

    // double-tap detection as before
    if ((now - _lastUpAtMs) <= DOUBLE_TAP_GAP_MS) {
      _secondTap = true;
      _secondDownAtMs = now;
    } else {
      _secondTap = false;
    }

    // NEW: if we recently lifted after moving, and this down is near that spot,
    // treat it like the start of a “lift-hold-to-drag” pseudo second tap.
    if (_primeLiftHold && now <= _primeExpireMs) {
      int ndx = abs((int)x - (int)_primeX);
      int ndy = abs((int)y - (int)_primeY);
      if (ndx <= LIFT_HOLD_RADIUS_PX && ndy <= LIFT_HOLD_RADIUS_PX) {
        _secondTap = true;              // reuse the existing second-tap path
        _secondDownAtMs = now;          // holding will enter drag after DT_HOLD_TO_DRAG_MS
      }
    }
    // consume the prime regardless on any new down
    _primeLiftHold = false;

    return;
  }

  // ---- HOLD
  if (hasPoint && _touchDown) {
    int dx  = (int)x - (int)_prevX;
    int dy  = (int)y - (int)_prevY;
    int mdx = abs((int)x - (int)_downX);
    int mdy = abs((int)y - (int)_downY);

    if (_armedClick && (mdx > TAP_MOVE_PX || mdy > TAP_MOVE_PX)) {
      _armedClick = false;
    }

    // Only double-tap-hold (or primed lift-hold) starts drag
    if (_secondTap && !_dragging && (now - _secondDownAtMs) >= DT_HOLD_TO_DRAG_MS) {
      _dragging = true;
      _armedClick = false;
      pressLeft();
    }

    if (micros() - _lastSendAtUS > MIN_INTERVAL_US && (dx != 0 || dy != 0)) {
      sendMove(dx, dy);
      _lastSendAtUS = micros();
      _prevX = x;
      _prevY = y;
    }
    return;
  }

  // ---- UP
  if (!hasPoint && _touchDown) {
    _touchDown = false;
    uint32_t pressDur = now - _downAtMs;

    if (_dragging) {
      releaseLeft();
      _dragging = false;
      _armedClick = false;
      _secondTap = false;
      _lastUpAtMs = now;
      return;
    }

    int dx  = (int)x - (int)_prevX;
    int dy  = (int)y - (int)_prevY;
    if (dx != 0 || dy != 0) {
      sendMove(dx, dy);
      _prevX = x;
      _prevY = y;
    }

    // Single tap click
    if (_armedClick && pressDur <= TAP_MAX_MS) {
      pressLeft();
      releaseLeft();
    }

    // NEW: prime a “lift-hold-to-drag” window at the last pointer position.
    // Use the last stable position (_prevX/_prevY).
    _primeLiftHold = true;
    _primeExpireMs = now + LIFT_HOLD_GAP_MS;
    _primeX = _prevX;
    _primeY = _prevY;

    _armedClick = false;
    _secondTap  = false;
    _lastUpAtMs = now;
  }
}

// ---------------- Two-finger scrolling ----------------
void TouchMouse::handleTwoFingers(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
  // Ignore any click/drag state while two fingers are down
  // (lift both to end scroll; next one-finger cycle resumes normal behavior)
  if (!_scrolling) {
    _scrolling = true;
    _accumV = _accumH = 0;
    // start from average position to avoid a jump
    _prev2X = (uint16_t)(((uint32_t)x1 + x2) / 2);
    _prev2Y = (uint16_t)(((uint32_t)y1 + y2) / 2);
    return;
  }

  // current average
  uint16_t ax = (uint16_t)(((uint32_t)x1 + x2) / 2);
  uint16_t ay = (uint16_t)(((uint32_t)y1 + y2) / 2);

  int dx = (int)ax - (int)_prev2X;
  int dy = (int)ay - (int)_prev2Y;

  _prev2X = ax;
  _prev2Y = ay;

  // accumulate pixel deltas into scroll notches
  _accumV += dy;
  _accumH += dx;

  int notchesV = _accumV / (int)SCROLL_PX_PER_NOTCH;
  int notchesH = _accumH / (int)SCROLL_PX_PER_NOTCH;

  if (notchesV != 0 || notchesH != 0) {
    _accumV -= notchesV * (int)SCROLL_PX_PER_NOTCH;
    _accumH -= notchesH * (int)SCROLL_PX_PER_NOTCH;

    // invert per user preference (common to invert vertical for "natural" scroll)
    if (invertScrollV) notchesV = -notchesV;
    if (invertScrollH) notchesH = -notchesH;

    if (_cb.scroll) _cb.scroll(notchesV, notchesH, _cb.user);
  }
}
