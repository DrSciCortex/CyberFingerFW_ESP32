/*
 * SPDX-FileCopyrightText: 2026 DrSciCortex
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

// cybernow_proto.h
#pragma once
#include <stdint.h>
#include "gamepad_merged.h"

#define CYBER_ROLE_LEFT  0
#define CYBER_ROLE_RIGHT 1

enum CYBER_PROTO_VER : uint8_t { CYBER_PROTO_MOUSE = 1, CYBER_PROTO_HALFGAMEPAD = 2 };

// Choose a fixed 2.4GHz channel for ESP-NOW (both must match); 1, 6 or 11 are common
#define CYBER_WIFI_CHANNEL 6

// Max time we'll trust the last left packet before we consider it "stale" (ms)
#define CYBER_LEFT_STALE_MS 120

enum : uint16_t {
    BTN_MOUSE1_PRESS = 1u << 0, // is a press event being transmitted 
    BTN_MOUSE1_RELEASE = 1u << 1 // is a release event being transmitted (false is nop)
};

// Keep payload small (ESP-NOW max 250B). Align/pack to avoid padding.
typedef struct __attribute__((packed)) {
    uint8_t  ver;        // CYBER_PROTO_VER
    uint8_t  srcRole;    // 0=left, 1=right
    uint16_t seq;        // rolling sequence number
    uint32_t ms;         // millis() at sender
    int16_t  mx; //mouse x
    int16_t  my; //mouse y
    uint16_t buttons;    // BTN_* bits (not Dpad)
    uint16_t reserved; // pad to 16 bytes (or use for wheel/flags later)
   } CyberNowPkt;


union CyberPkt16 {
  uint8_t     raw[16];
  HalfPacket  halfgamepad;
  CyberNowPkt mouse;
};

static_assert(sizeof(HalfPacket)  == sizeof(CyberNowPkt), "HalfPacket must be same size as CyberNowPkt");
static_assert(sizeof(HalfPacket)  == 16, "HalfPacket must be size = 16");

static inline bool parseCyberPkt(CyberPkt16 &u, const uint8_t* data, int len) {
  if (!data || len != 16) return false;
  memcpy(u.raw, data, 16);          // safe load (alignment-proof)
  return true;
}

