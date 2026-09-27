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
#include <stddef.h>          // offsetof, used by the wire-format static_assert
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include "gamepad_merged.h"  // for HalfPacket, PKT_* constants

// ── Service & Characteristic UUIDs ─────────────────────────────────────────
// Short UUIDs under the Bluetooth base UUID
#define VR_GATT_SERVICE_UUID        "0000CF00-0000-1000-8000-00805F9B34FB"
#define VR_GATT_INPUT_CHAR_UUID     "0000CF01-0000-1000-8000-00805F9B34FB"
#define VR_GATT_CONTROL_CHAR_UUID   "0000CF02-0000-1000-8000-00805F9B34FB"

// ── IMU presence bits (VrGattInputReport.imu_present) ──────────────────────
// This board can carry up to three IMUs: the onboard QMI8658 and up to two
// ICM-45686 strapped to 0x68/0x69. The two body sensors are redundant (either
// may be missing); the joint sensor is optional.
enum VrImuBit : uint8_t {
    VR_IMU_BODY_PRIMARY   = 0x01,  // q       is valid
    VR_IMU_BODY_SECONDARY = 0x02,  // q_body2 is valid
    VR_IMU_JOINT          = 0x04,  // q_joint is valid
};

// ── Wire format: notification payload ──────────────────────────────────────
// VARIABLE length, sent at ~100Hz when in VR mode. The struct below is the
// MAXIMUM layout (all IMU slots present, 79 bytes). The actual notification
// carries the frozen header plus ONLY the slots flagged in imu_present, packed
// contiguously - absent/dropped slots are omitted from the wire entirely.
// Omitting them shrinks airtime, which is what lets one host schedule two
// peripherals at a tight connection interval without starving either.
//
// What is actually sent:
//   [0 .. VR_GATT_HEADER_LEN-1]  always: frozen prefix + imu_present
//   then appended in THIS ORDER, each block only if its bit is set:
//     VR_IMU_BODY_PRIMARY    a_body1[3]                (quat is in header q)
//     VR_IMU_BODY_SECONDARY  q_body2[4] + a_body2[3]
//     VR_IMU_JOINT           q_joint[4] + a_joint[3]
//
// COMPATIBILITY: bytes 0..27 are frozen - byte-for-byte identical to the
// original 28-byte report, with q carrying the PRIMARY body orientation (from
// whichever body sensor is active; the two body sensors are redundant). A
// reader that takes the first 28 bytes at fixed offsets and ignores the rest
// stays correct. Do not reorder or resize any field above imu_present.
//
// A variable-length reader MUST switch on imu_present, not on total length -
// different slot combinations can yield the same byte count.
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
    float    q[4];           // Quaternion (w, x, y, z) - PRIMARY body IMU
    // ── appended below; older readers stop here ──
    uint8_t  imu_present;    // bitmask of VrImuBit
    // Optional slots, in WIRE ORDER: each present slot contributes its block,
    // grouped per slot (quaternion then accel). The primary body quaternion is
    // NOT repeated here - it lives in q above; only its accel appears. In a
    // real notification, blocks whose bit is clear in imu_present are OMITTED,
    // so every offset past imu_present shifts. This struct is only the layout
    // for the all-present case; parse the wire by imu_present, not by offset.
    //
    // Accel units: LSB at VR_ACCEL_LSB_PER_G (+/-16g on every sensor), raw,
    // NOT gravity-corrected, SENSOR frame. Remove gravity with the slot's own
    // quaternion (same packet, same sample):
    //   gx = 2*(q1*q3 - q0*q2)
    //   gy = 2*(q0*q1 + q2*q3)
    //   gz = q0*q0 - q1*q1 - q2*q2 + q3*q3
    //   linear[i] = accel[i]/VR_ACCEL_LSB_PER_G*9.80665 - g[i]*9.80665   [m/s^2]
    // Same derivation SlimeVR uses: feed linear accel to PACKET_ACCEL (4) and
    // the quaternion to PACKET_ROTATION_DATA (17).
    int16_t  a_body1[3];     // PRIMARY body accel      (present iff bit0)
    float    q_body2[4];     // SECONDARY body quat     (present iff bit1)
    int16_t  a_body2[3];     // SECONDARY body accel    (present iff bit1)
    float    q_joint[4];     // JOINT quat              (present iff bit2)
    int16_t  a_joint[3];     // JOINT accel             (present iff bit2)
} VrGattInputReport;

// Divide a_* by this to get g.
#define VR_ACCEL_LSB_PER_G 2048.0f

// Always-present part of every notification: the frozen 28-byte prefix plus the
// one-byte imu_present that follows it. The variable tail begins right after.
#define VR_GATT_HEADER_LEN  (offsetof(VrGattInputReport, imu_present) + 1)  // 29
#define VR_GATT_MAX_REPORT  (sizeof(VrGattInputReport))                     // 79

static_assert(sizeof(VrGattInputReport) == 79, "VrGattInputReport max must be 79 bytes");
static_assert(offsetof(VrGattInputReport, imu_present) == 28,
              "legacy 28-byte prefix must stay frozen for older mod builds");
static_assert(offsetof(VrGattInputReport, a_body1) == VR_GATT_HEADER_LEN,
              "variable tail must start immediately after imu_present");
static_assert(VR_GATT_HEADER_LEN == 29, "header is frozen prefix + imu_present");

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

// Orientation set gathered from whichever IMUs this unit actually has.
// Slots flagged absent in `present` are ignored and sent as identity.
typedef struct {
    uint8_t present;      // bitmask of VrImuBit
    float   body1[4];     // primary body IMU (w, x, y, z)
    float   body2[4];     // secondary body IMU
    float   joint[4];     // joint IMU
    int16_t a_body1[3];   // raw body-frame accel, VR_ACCEL_LSB_PER_G
    int16_t a_body2[3];
    int16_t a_joint[3];
} VrImuSet;

// Builds a VrGattInputReport from the HalfPacket and sends a BLE notification.
// Passing nullptr for `imus` sends identity for all three slots with
// present = 0, which is what a unit with no working IMU reports.
// Returns true if notification was sent (client subscribed).
bool vrGattSendInput(const HalfPacket& local, uint8_t batteryPct, const VrImuSet* imus = nullptr);

// Check if a VR GATT client is connected and subscribed to notifications.
bool vrGattClientConnected();

// Get the current sequence number (for debugging)
uint32_t vrGattGetSeq();
