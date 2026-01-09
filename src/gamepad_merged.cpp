/*
 * SPDX-FileCopyrightText: 2026 DrSciCortex
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include "gamepad_merged.h"
#include <Arduino.h>
#include <XboxGamepadDevice.h> 
#include "HWCDC.h"
#include "cybernow_proto.h"
#include <esp_now.h>

extern HWCDC USBSerial;

static inline int iabs(int v) { return v < 0 ? -v : v; }
static inline int uabs8(int a, int b) { int d = a - b; return d < 0 ? -d : d; }

static inline bool leftMeaningfullyChanged(const HalfPacket &a, const HalfPacket &b,
                                           int stickDeltaThreshold, int triggerDeltaThreshold) {
  if (a.btnMask != b.btnMask) return true;
  if (a.dpad != b.dpad) return true;

  if (iabs((int)a.jx - (int)b.jx) > stickDeltaThreshold) return true;
  if (iabs((int)a.jy - (int)b.jy) > stickDeltaThreshold) return true;

  if (iabs((int)a.trigger - (int)b.trigger) > triggerDeltaThreshold) return true;

  return false;
}

GamepadMerged::GamepadMerged(IGamepadOut* gamepad, const Config& cfg)
: gamepad_(gamepad), cfg_(cfg) {}

GamepadMerged::GamepadMerged(IGamepadOut* gamepadPtr)
: GamepadMerged(gamepadPtr, Config{}) {}

void GamepadMerged::setGamepad(IGamepadOut* gamepadPtr) {
    gamepad_ = gamepadPtr;
}

void GamepadMerged::setConfig(const Config& cfg) {
  cfg_ = cfg;
}

void GamepadMerged::setRightLocal(const HalfPacket& right) {
  right_ = right;
  rightValid_ = true;
}

void GamepadMerged::setLeftRemote(const HalfPacket& left) {
  left_ = left;
  leftValid_ = true;
}

void GamepadMerged::setLeftLocal(const HalfPacket &left) {
  left_ = left;
  leftValid_ = true;
}

void GamepadMerged::merge_() {
  // Keep last-known values if one side hasn't updated.
  MergedState m = cur_;

  // LEFT contribution (typical: dpad + LT + left stick + X/Y/LB/LS/Select)
  if (leftValid_) {
    uint16_t b = m.buttons;
    setBit(b, H_X,      (left_.btnMask & PKT_AX));
    setBit(b, H_Y,      (left_.btnMask & PKT_BY));
    setBit(b, H_LS,     (left_.btnMask & PKT_ST));
    setBit(b, H_LB,     (left_.btnMask & PKT_BP));
    setBit(b, H_SELECT, (left_.btnMask & PKT_STARTSELECT));
    m.buttons = b;

    m.lx = left_.jx; m.ly = left_.jy;

    // convention: left packet carries LT and dpad
    m.lt = left_.trigger;
    m.dpad = left_.dpad;
  }

  // RIGHT contribution (typical: RT + right stick + A/B/RB/RS/Start)
  if (rightValid_) {
    uint16_t b = m.buttons;
    setBit(b, H_A,     (right_.btnMask & PKT_AX));
    setBit(b, H_B,     (right_.btnMask & PKT_BY));
    setBit(b, H_RS,    (right_.btnMask & PKT_ST));
    setBit(b, H_RB,    (right_.btnMask & PKT_BP));
    setBit(b, H_START, (right_.btnMask & PKT_STARTSELECT));
    m.buttons = b;

    m.rx = right_.jx; m.ry = right_.jy;

    // convention: right packet carries RT
    m.rt = right_.trigger;
  }

  cur_ = m;
}

void GamepadMerged::press_(uint16_t hidBit)   { if (gamepad_) gamepad_->press(hidBit); }
void GamepadMerged::release_(uint16_t hidBit) { if (gamepad_) gamepad_->release(hidBit); }

// ---- Adapter points for gamepad API ----

void GamepadMerged::setLeftThumb_(int16_t x, int16_t y) {
  if (gamepad_) gamepad_->setLeftThumb(x, y);
}

void GamepadMerged::setRightThumb_(int16_t x, int16_t y) {
  if (gamepad_) gamepad_->setRightThumb(x, y);
}

void GamepadMerged::setTriggers_(uint8_t lt, uint8_t rt) {
  if (gamepad_) gamepad_->setTriggers(lt, rt);
}

void GamepadMerged::sendReport_() {
  if (gamepad_) gamepad_->sendGamepadReport();
}

// ---- Diff emitters ----

bool GamepadMerged::emitButtonEvents_() {
  bool changed = false;
  const uint16_t all = H_START|H_SELECT|H_A|H_B|H_X|H_Y|H_RS|H_RB|H_LS|H_LB;

  uint16_t diff = (cur_.buttons ^ prev_.buttons) & all;
  while (diff) {
    uint16_t bit = diff & (uint16_t)(-(int16_t)diff); // lowbit
    diff ^= bit;

    const bool nowPressed = (cur_.buttons & bit) != 0;
    if (nowPressed) press_(bit);
    else            release_(bit);
    changed = true;
  }
  return changed;
}

bool GamepadMerged::emitStickEvents_() {
  bool changed = false;

  const bool leftMoved =
    (iabs((int)cur_.lx - (int)prev_.lx) > cfg_.stickDeltaThreshold) ||
    (iabs((int)cur_.ly - (int)prev_.ly) > cfg_.stickDeltaThreshold);

  const bool rightMoved =
    (iabs((int)cur_.rx - (int)prev_.rx) > cfg_.stickDeltaThreshold) ||
    (iabs((int)cur_.ry - (int)prev_.ry) > cfg_.stickDeltaThreshold);

  if (leftMoved)  { setLeftThumb_(cur_.lx, cur_.ly); changed = true; }
  if (rightMoved) { setRightThumb_(cur_.rx, cur_.ry); changed = true; }

  return changed;
}

bool GamepadMerged::emitTriggerEvents_() {
  // Triggers are 0..255; use a small delta threshold to avoid spam.
  if (uabs8(cur_.lt, prev_.lt) > cfg_.triggerDeltaThreshold ||
      uabs8(cur_.rt, prev_.rt) > cfg_.triggerDeltaThreshold) {
    setTriggers_(cur_.lt, cur_.rt);
    return true;
  }
  return false;
}

/*
bool GamepadMerged::emitDpadEvents_() {
  if (cur_.dpad != prev_.dpad) {
    setDpad_(cur_.dpad);
    return true;
  }
  return false;
}*/

void GamepadMerged::updateAndSendIfChanged() {
  merge_();

  bool changed = false;
  changed |= emitButtonEvents_();
  changed |= emitStickEvents_();
  changed |= emitTriggerEvents_();
  //changed |= emitDpadEvents_();

  if (changed) {
    sendReport_();
    prev_ = cur_;
  }
}

bool GamepadMerged::buildLeftPacketIfChanged(HalfPacket &outPkt) {
  if (!leftValid_) return false; // nothing to send

  // Compare logical state, ignoring seq/ms/ver/srcRole
  if (leftHasPrevSent_) {
    if (!leftMeaningfullyChanged(left_, leftSentPrev_, cfg_.stickDeltaThreshold, cfg_.triggerDeltaThreshold)) {
      return false;
    }
  }

  // Build outgoing packet from current left_ snapshot
  outPkt = left_;
  outPkt.seq = ++leftSeq_;
  outPkt.ms  = millis();
  outPkt.srcRole = CYBER_ROLE_LEFT;
  outPkt.ver = CYBER_PROTO_HALFGAMEPAD;

  // Save what we sent (store the logical fields too, including dpad/trigger/btnMask/jx/jy)
  leftSentPrev_ = left_;
  leftHasPrevSent_ = true;

  return true;
}

bool GamepadMerged::sendLeftPacketToRightIfChanged(const uint8_t peerMac[6]) {
  CyberPkt16 pkt{};
  if (!buildLeftPacketIfChanged(pkt.halfgamepad)) return false;

  esp_err_t err = esp_now_send(peerMac, pkt.raw, sizeof(pkt.raw));
  if (err != ESP_OK) {
    if (cfg_.debug) USBSerial.printf("[ESP-NOW] HalfPacket send err=%d\n", err);
    return false;
  }
  return true;
}

bool GamepadMerged::sendLeftPacketToRight(const uint8_t peerMac[6]) {
  CyberPkt16 pkt{};
  HalfPacket& outPkt = pkt.halfgamepad;

  outPkt = left_;
  outPkt.seq = ++leftSeq_;
  outPkt.ms  = millis();
  outPkt.srcRole = CYBER_ROLE_LEFT;
  outPkt.ver = CYBER_PROTO_HALFGAMEPAD;

  leftSentPrev_ = left_;
  leftHasPrevSent_ = true;

  esp_err_t err = esp_now_send(peerMac, pkt.raw, sizeof(pkt.raw));
  if (err != ESP_OK) {
    if (cfg_.debug) USBSerial.printf("[ESP-NOW] HalfPacket send err=%d\n", err);
    return false;
  }
  return true;
}