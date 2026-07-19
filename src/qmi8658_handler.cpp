/*
 * SPDX-FileCopyrightText: 2026 DrSciCortex
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include "qmi8658_handler.h"
#include "vqf.h"
#include <Wire.h>
#include <math.h>
#include "SensorQMI8658.hpp"
#include "pin_config.h"

// Note the SensorLib address constants are named counter-intuitively:
// QMI8658_L_SLAVE_ADDRESS is 0x6B and _H_ is 0x6A. The bus scan finds this
// part at 0x6B, which is also the library default.
#define QMI_ADDR QMI8658_L_SLAVE_ADDRESS

static SensorQMI8658 qmi;
static VQF vqf(0.01f);              // 100 Hz nominal; real dt passed per sample
static uint32_t last_update = 0;
static bool     initialised = false;

bool qmi8658_init() {
    // Wire is already up by the time this runs; SensorLib re-calls Wire.begin()
    // internally, which the ESP32 core turns into a no-op that preserves the
    // existing pin assignment, so passing the pins here is harmless.
    if (!qmi.init(Wire, IIC_SDA, IIC_SCL, QMI_ADDR)) {
        return false;
    }

    // Ranges chosen to match the ICM-45686's role as the redundant body
    // sensor. 1024 dps is this part's maximum - the ICM runs 2000 dps, so a
    // fast enough wrist flick can saturate this one first.
    qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G,
                            SensorQMI8658::ACC_ODR_125Hz);
    qmi.configGyroscope(SensorQMI8658::GYR_RANGE_1024DPS,
                        SensorQMI8658::GYR_ODR_112_1Hz);

    qmi.enableAccelerometer();
    qmi.enableGyroscope();

    last_update = micros();
    vqf.resetState();
    initialised = true;
    return true;
}

void qmi8658_update() {
    if (!initialised) return;

    // Unlike the ICM path, this part exposes a data-ready flag, so we consume
    // each sample exactly once instead of re-reading or skipping when the loop
    // and the sensor ODR drift against each other.
    if (!qmi.getDataReady()) return;

    float ax, ay, az, gx, gy, gz;
    if (!qmi.getAccelerometer(ax, ay, az)) return;   // g
    if (!qmi.getGyroscope(gx, gy, gz)) return;       // dps

    float acc[3] = {ax, ay, az};                     // VQF normalises accel

    const float kDegToRad = (float)(M_PI / 180.0);
    float gyr[3] = {gx * kDegToRad, gy * kDegToRad, gz * kDegToRad};

    // Feed the real elapsed time, for the same reason as the ICM handler: the
    // main loop's cadence jitters and a fixed dt accumulates orientation error.
    uint32_t now = micros();
    float dt = (now - last_update) * 1e-6f;          // unsigned math wraps fine
    last_update = now;
    if (dt <= 0.0f || dt > 0.5f) {
        dt = -1.0f;                                   // fall back to nominal
    }

    vqf.updateGyr(gyr, dt);
    vqf.updateAcc(acc);
}

void qmi8658_get_quat(float quat[4]) {
    vqf.getQuat6D(quat);
}
