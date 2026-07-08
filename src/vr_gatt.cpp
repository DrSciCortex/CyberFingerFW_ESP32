/*
 * SPDX-FileCopyrightText: 2026 DrSciCortex
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

// vr_gatt.cpp — BLE GATT service for VR Direct Mode (NimBLE)

#include "vr_gatt.h"
#include <Arduino.h>
#include "HWCDC.h"

extern HWCDC USBSerial;

// ── Module state ───────────────────────────────────────────────────────────
static NimBLECharacteristic* s_inputChar  = nullptr;
static NimBLECharacteristic* s_ctrlChar   = nullptr;
static NimBLEService*        s_service    = nullptr;
static bool                  s_isRight    = false;
static uint32_t              s_seq        = 0;
static bool                  s_subscribed = false;

// Extern: the main loop checks this to know if bridge requested mode change
extern volatile bool vrDirectMode;

// ── Callbacks ──────────────────────────────────────────────────────────────

// Called when bridge writes to the control characteristic (0xCF02)
class VrControlCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) {
        std::string val = pChar->getValue();
        if (val.empty()) return;

        uint8_t cmd = (uint8_t)val[0];
        switch (cmd) {
            case VR_CMD_ENTER_VR:
                USBSerial.println("[VR_GATT] Bridge requested ENTER VR mode");
                vrDirectMode = true;
                break;
            case VR_CMD_EXIT_VR:
                USBSerial.println("[VR_GATT] Bridge requested EXIT VR mode");
                vrDirectMode = false;
                break;
            case VR_CMD_QUERY_MODE: {
                USBSerial.printf("[VR_GATT] Bridge queried mode: %s\n",
                                 vrDirectMode ? "VR" : "GAMEPAD");
                if (s_inputChar) {
                    VrGattInputReport rpt{};
                    rpt.hand = s_isRight ? 1 : 0;
                    rpt.buttons = vrDirectMode ? 0xFF : 0x00;
                    rpt.seq = 0;
                    s_inputChar->setValue((uint8_t*)&rpt, sizeof(rpt));
                    s_inputChar->notify();
                }
                break;
            }
            default:
                USBSerial.printf("[VR_GATT] Unknown command: 0x%02X\n", cmd);
                break;
        }
    }
};

// Track notification subscription state
class VrInputCallbacks : public NimBLECharacteristicCallbacks {
    void onSubscribe(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo, uint16_t subValue) {
        s_subscribed = (subValue & 0x0001) != 0;
        USBSerial.printf("[VR_GATT] Notifications %s\n",
                         s_subscribed ? "ENABLED" : "DISABLED");
    }
};

static VrControlCallbacks s_ctrlCb;
static VrInputCallbacks   s_inputCb;

// ── Init ───────────────────────────────────────────────────────────────────

bool vrGattInit(NimBLEServer* pServer, bool isRight) {
    s_isRight = isRight;

    if (!pServer) {
        USBSerial.println("[VR_GATT] ERROR: null NimBLE server pointer");
        return false;
    }

    USBSerial.println("[VR_GATT] Creating service...");

    // Create the VR service on the existing NimBLE server
    s_service = pServer->createService(VR_GATT_SERVICE_UUID);
    if (!s_service) {
        USBSerial.println("[VR_GATT] ERROR: Failed to create service");
        return false;
    }

    // Input characteristic: Notify (ESP32 → Bridge)
    // NimBLE automatically adds CCCD (0x2902) when NOTIFY property is set
    s_inputChar = s_service->createCharacteristic(
        VR_GATT_INPUT_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    s_inputChar->setCallbacks(&s_inputCb);

    // Set initial value
    VrGattInputReport initRpt{};
    initRpt.hand = isRight ? 1 : 0;
    s_inputChar->setValue((uint8_t*)&initRpt, sizeof(initRpt));

    // Control characteristic: Write (Bridge → ESP32)
    s_ctrlChar = s_service->createCharacteristic(
        VR_GATT_CONTROL_CHAR_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    s_ctrlChar->setCallbacks(&s_ctrlCb);

    // Start the service
    s_service->start();

    // Update advertising to include the new service UUID
    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->addServiceUUID(VR_GATT_SERVICE_UUID);

    USBSerial.printf("[VR_GATT] Service started (hand=%s)\n",
                     isRight ? "RIGHT" : "LEFT");
    return true;
}

// ── Send Input ─────────────────────────────────────────────────────────────

bool vrGattSendInput(const HalfPacket& local, uint8_t batteryPct, const float q[4]) {
    if (!s_inputChar || !s_subscribed) return false;

    VrGattInputReport rpt{};
    rpt.hand = s_isRight ? 1 : 0;

    // Map HalfPacket button mask to VR GATT button bits
    rpt.buttons = 0;
    if (local.btnMask & PKT_AX)          rpt.buttons |= 0x01; // trigger
    if (local.btnMask & PKT_BY)          rpt.buttons |= 0x02; // grip
    if (local.btnMask & PKT_CZ)          rpt.buttons |= 0x04; // C/Z
    if (local.btnMask & PKT_DD)          rpt.buttons |= 0x08; // D/D
    if (local.btnMask & PKT_EE)          rpt.buttons |= 0x10; // E/E
    if (local.btnMask & PKT_BP)          rpt.buttons |= 0x20; // bumper/menu
    if (local.btnMask & PKT_ST)          rpt.buttons |= 0x40; // stick click
    if (local.btnMask & PKT_STARTSELECT) rpt.buttons |= 0x80; // start/select

    rpt.joy_x = local.jx;
    rpt.joy_y = local.jy;
    rpt.trigger_analog = local.trigger;
    rpt.battery_pct = batteryPct;
    rpt.seq = ++s_seq;

    // Add quaternion data
    if (q) {
        rpt.q[0] = q[0];
        rpt.q[1] = q[1];
        rpt.q[2] = q[2];
        rpt.q[3] = q[3];
    } else {
        rpt.q[0] = 1.0f; // Identity
        rpt.q[1] = 0.0f;
        rpt.q[2] = 0.0f;
        rpt.q[3] = 0.0f;
    }

    s_inputChar->setValue((uint8_t*)&rpt, sizeof(rpt));
    s_inputChar->notify();

    return true;
}

bool vrGattClientConnected() {
    return s_subscribed;
}

uint32_t vrGattGetSeq() {
    return s_seq;
}
