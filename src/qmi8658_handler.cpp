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

// Sample rate. LPF_MODE_3 is a fixed 13.37% of ODR, so the rate sets the filter: at 112 Hz it sat at ~15 Hz,
// and its group delay made this sensor trail the ICM joint by 31-38 ms (measured on the host, 2026-09-26). At
// 448 Hz it sits near 60 Hz. Every sample comes through the FIFO into VQF, so although the loop runs at ~100 Hz
// nothing is dropped or aliased. 224.2 Hz (GYR_ODR_224_2Hz / ACC_ODR_250Hz) halves the I2C traffic if needed.
static constexpr float kQmiRate    = 448.4f;
static constexpr uint16_t kFifoMax = 64;                 // FIFO_SAMPLES_64: 143 ms of samples at 448 Hz
static IMUdata s_accFifo[kFifoMax], s_gyrFifo[kFifoMax];

static VQF vqf(1.0f / kQmiRate, 1.0f / kQmiRate);
static float    s_period = 1.0f / kQmiRate;              // measured sample period: the part's clock runs off nominal
static uint32_t last_update = 0;
static bool     initialised = false;

// Last raw accelerometer sample for the wire format, LSB at 2048 LSB/g.
static int16_t  s_accel[3] = {0, 0, 0};

bool qmi8658_init() {
    // Wire is already up by the time this runs; SensorLib re-calls Wire.begin()
    // internally, which the ESP32 core turns into a no-op that preserves the
    // existing pin assignment, so passing the pins here is harmless.
    if (!qmi.init(Wire, IIC_SDA, IIC_SCL, QMI_ADDR)) {
        return false;
    }

    // Ranges chosen to match the ICM-45686's role as the redundant body
    // sensor. 16g yields 2048 LSB/g, identical to the ICM at its 16g range, so
    // every IMU slot on the wire shares one scale factor. 1024 dps is this
    // part's maximum - the ICM runs 2000 dps, so a fast enough wrist flick can
    // saturate this one first.
    // LPF choice matters more than it looks: SensorLib's default is LPF_MODE_0,
    // only 2.66% of ODR, and its group delay showed up as this sensor visibly
    // lagging the ICM. LPF_MODE_3 is the widest mode, 13.37% of ODR; since the
    // cutoff scales with ODR, the ODR is what's left to raise (kQmiRate above).
    // In 6DOF mode the accelerometer runs at the gyro's rate.
    //
    // Not LPF_OFF: band-limiting still matters, the FIFO just removes the
    // dropped-sample aliasing that came from reading at ~100Hz.
    qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_16G,
                            SensorQMI8658::ACC_ODR_500Hz,
                            SensorQMI8658::LPF_MODE_3);
    qmi.configGyroscope(SensorQMI8658::GYR_RANGE_1024DPS,
                        SensorQMI8658::GYR_ODR_448_4Hz,
                        SensorQMI8658::LPF_MODE_3);
    // Stream mode: when the loop stalls past the FIFO's depth the oldest samples
    // go, not the newest.
    qmi.configFIFO(SensorQMI8658::FIFO_MODE_STREAM, SensorQMI8658::FIFO_SAMPLES_64);

    qmi.enableGyroscope();
    qmi.enableAccelerometer();

    s_period = 1.0f / kQmiRate;
    last_update = micros();
    vqf.resetState();
    initialised = true;
    return true;
}

void qmi8658_update() {
    if (!initialised) return;

    // Everything the part sampled since the last call (4-5 samples at ~100Hz),
    // each consumed exactly once, in order.
    const uint16_t n = qmi.readFromFifo(s_accFifo, kFifoMax, s_gyrFifo, kFifoMax);
    if (n == 0) return;

    // The FIFO samples are evenly spaced on the part's own clock, which runs a
    // few percent off nominal; integrating at the nominal rate would scale every
    // rotation by that error. Track the real period: elapsed time over samples
    // read, smoothed (a single batch is jittery). Outliers (a stall that
    // overflowed the FIFO) are skipped.
    uint32_t now = micros();
    float elapsed = (now - last_update) * 1e-6f;     // unsigned math wraps fine
    last_update = now;
    if (elapsed > 0.0f && elapsed < 0.5f) {
        const float p = elapsed / n;
        if (p > 0.5f / kQmiRate && p < 2.0f / kQmiRate) {
            s_period += 0.02f * (p - s_period);
        }
    }

    const float kDegToRad = (float)(M_PI / 180.0);
    for (uint16_t i = 0; i < n; i++) {
        float gyr[3] = {s_gyrFifo[i].x * kDegToRad, s_gyrFifo[i].y * kDegToRad, s_gyrFifo[i].z * kDegToRad};
        float acc[3] = {s_accFifo[i].x, s_accFifo[i].y, s_accFifo[i].z};   // g; VQF normalises accel
        vqf.updateGyr(gyr, s_period);
        vqf.updateAcc(acc);
    }

    // Raw counts of the newest sample for the wire format, in the same 2048
    // LSB/g units the ICMs report so the host applies one scale factor to every
    // slot (16g range: exactly 2048 LSB/g).
    const IMUdata& a = s_accFifo[n - 1];
    const float v[3] = {a.x, a.y, a.z};
    for (int k = 0; k < 3; k++) {
        float c = roundf(v[k] * 2048.0f);
        s_accel[k] = (int16_t)(c > 32767.0f ? 32767.0f : (c < -32768.0f ? -32768.0f : c));
    }
}

void qmi8658_get_quat(float quat[4]) {
    vqf.getQuat6D(quat);
}

void qmi8658_get_accel(int16_t out[3]) {
    out[0] = s_accel[0];
    out[1] = s_accel[1];
    out[2] = s_accel[2];
}
