/*
 * SPDX-FileCopyrightText: 2026 DrSciCortex
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

// vr_gatt.h — BLE GATT service for VR Direct Mode
// Each ESP32 advertises this service and sends button/joystick notifications
// directly to a PC bridge app, bypassing Xbox gamepad HID entirely.

#pragma once
#include <stdint.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include "gamepad_merged.h"  // for HalfPacket, PKT_* constants

// ── Service & Characteristic UUIDs ─────────────────────────────────────────
// Short UUIDs under the Bluetooth base UUID
#define VR_GATT_SERVICE_UUID        "0000CF00-0000-1000-8000-00805F9B34FB"
#define VR_GATT_INPUT_CHAR_UUID     "0000CF01-0000-1000-8000-00805F9B34FB"
#define VR_GATT_CONTROL_CHAR_UUID   "0000CF02-0000-1000-8000-00805F9B34FB"

// ── Wire format: notification payload ──────────────────────────────────────
// 28 bytes per notification, sent at ~100Hz when in VR mode
typedef struct __attribute__((packed)) {
    uint8_t  hand;           // 0=left, 1=right
    uint8_t  buttons;        // bit0=AX(trigger), bit1=BY(grip), bit2=CZ,
                             // bit3=DD, bit4=EE, bit5=BP(menu),
                             // bit6=ST(joy click), bit7=STARTSELECT
    int16_t  joy_x;          // -32767..32767 (little-endian)
    int16_t  joy_y;          // -32767..32767
    uint8_t  trigger_analog; // 0-255 (future use, currently 0 or 255)
    uint8_t  battery_pct;    // 0-100
    uint32_t seq;            // rolling sequence number
    float    q[4];           // Quaternion (w, x, y, z) - 16 bytes
} VrGattInputReport;

static_assert(sizeof(VrGattInputReport) == 28, "VrGattInputReport must be 28 bytes");

// ── Control commands (written by bridge to 0xCF02) ─────────────────────────
enum VrGattCommand : uint8_t {
    VR_CMD_ENTER_VR   = 0x01,
    VR_CMD_EXIT_VR    = 0x02,
    VR_CMD_QUERY_MODE = 0x03,
};

// ── API ────────────────────────────────────────────────────────────────────

// Call once during setup(), after compositeHID->begin() has started the BLE stack.
// Pass the BLEServer pointer from your BLE stack.
// Returns true on success.
bool vrGattInit(NimBLEServer* pServer, bool isRight);

// Builds a VrGattInputReport from the HalfPacket and sends a BLE notification.
// Returns true if notification was sent (client subscribed).
bool vrGattSendInput(const HalfPacket& local, uint8_t batteryPct, const float q[4] = nullptr);

// Check if a VR GATT client is connected and subscribed to notifications.
bool vrGattClientConnected();

// Get the current sequence number (for debugging)
uint32_t vrGattGetSeq();
