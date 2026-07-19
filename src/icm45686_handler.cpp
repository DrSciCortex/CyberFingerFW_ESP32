/*
 * SPDX-FileCopyrightText: 2026 DrSciCortex
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include "icm45686_handler.h"
#include "vqf.h"
#include <Wire.h>
#include <math.h>

// ICM-456xx register map. These differ from the ICM-426xx map this driver
// originally used - every address below was previously wrong, which is why
// configuration silently went nowhere and the data reads returned garbage.
//
// WHO_AM_I is at 0x72 (value 0xE9); 0x75 is the ICM-426xx/206xx location and
// reads back 0x00 on this part - confirmed by bus dump.
#define REG_WHO_AM_I      0x72
#define ICM45686_WHOAMI   0xE9
#define REG_DEVICE_CONFIG 0x7F
#define DEVICE_CONFIG_SW_RESET 0x03
#define REG_ACCEL_DATA_X1 0x00   // was 0x1F
#define REG_GYRO_DATA_X1  0x06   // was 0x25
#define REG_PWR_MGMT0     0x10   // was 0x4E
#define REG_ACCEL_CONFIG0 0x1B   // was 0x50
#define REG_GYRO_CONFIG0  0x1C   // was 0x4F

// PWR_MGMT0 = GYRO_MODE << 2 | ACCEL_MODE; 0x03 is low-noise for both.
#define PWR_MGMT0_ACCEL_GYRO_LN  ((0x03 << 2) | 0x03)   // 0x0F

// {ACCEL,GYRO}_CONFIG0 = FS_SEL << 4 | ODR.
// 4g and 2000dps match the scaling applied in icm45686_update();
// 100 Hz matches the nominal VQF sample rate.
#define ACCEL_FS_4G       0x03
#define GYRO_FS_2000DPS   0x01
#define ODR_100HZ         0x09

// Scaling constants, kept next to the FS selects above so the two cannot drift
// apart: 32768 / 4g = 8192 LSB/g, and 2000 dps over a full-scale int16.
#define ACCEL_LSB_PER_G   8192.0f
#define GYRO_DPS_PER_LSB  (2000.0f / 32768.0f)

// Per-instance state. Both parts are identical and share a register map, so
// only the address and the fusion state differ.
static VQF      s_vqf[ICM_COUNT]    = { VQF(0.01f), VQF(0.01f) };
static uint8_t  s_addr[ICM_COUNT]   = { 0, 0 };
static uint32_t s_last[ICM_COUNT]   = { 0, 0 };
static uint8_t  s_whoami[ICM_COUNT] = { 0, 0 };
static bool     s_ok[ICM_COUNT]     = { false, false };

// Helper to write a register
static void write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

// Helper to read multiple bytes
static bool read_bytes(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;

    uint8_t read = Wire.requestFrom((int)addr, (int)len);
    if (read != len) return false;

    for (uint8_t i = 0; i < len; i++) {
        buf[i] = Wire.read();
    }
    return true;
}

bool icm45686_init(uint8_t instance, uint8_t addr) {
    if (instance >= ICM_COUNT) return false;

    s_ok[instance]     = false;
    s_addr[instance]   = addr;
    s_whoami[instance] = 0;

    uint8_t whoami = 0;
    if (!read_bytes(addr, REG_WHO_AM_I, &whoami, 1)) return false;
    s_whoami[instance] = whoami;

    if (whoami == 0x00 || whoami == 0xFF) return false;

    // Software reset before touching anything else. The part boots with its
    // APEX feature engines active and in an otherwise unknown state, and this
    // driver has no other way to clear that. 35ms settle matches SlimeVR.
    write_reg(addr, REG_DEVICE_CONFIG, DEVICE_CONFIG_SW_RESET);
    delay(35);

    // Configure BEFORE powering the sensors on: changing FS/ODR on a running
    // sensor glitches its output. (SlimeVR writes PWR_MGMT0 last for this
    // reason; the original order here powered up first, then reconfigured.)
    write_reg(addr, REG_GYRO_CONFIG0,  (GYRO_FS_2000DPS << 4) | ODR_100HZ);
    write_reg(addr, REG_ACCEL_CONFIG0, (ACCEL_FS_4G     << 4) | ODR_100HZ);
    write_reg(addr, REG_PWR_MGMT0, PWR_MGMT0_ACCEL_GYRO_LN);
    delay(10);

    s_last[instance] = micros();
    s_vqf[instance].resetState();
    s_ok[instance] = true;
    return true;
}

// VQF Fusion Integration
void icm45686_update(uint8_t instance) {
    if (instance >= ICM_COUNT || !s_ok[instance]) return;

    const uint8_t addr = s_addr[instance];
    uint8_t buf[6];

    // Data is LITTLE-endian: the byte at the lower address is the LOW byte, so
    // the block really starts at ACCEL_DATA_X0_UI. Confirmed on hardware - held
    // flat, this parse gives (-146, 528, 8179) i.e. 1g on Z at 4g FS, while the
    // big-endian parse gives an incoherent (28415, 4098, -3297).
    //
    // Note SlimeVR's nRF driver parses these registers big-endian; its ESP
    // driver parses the FIFO little-endian. This part's register path is
    // little-endian, per the measurement above.

    // Read accelerometer
    if (!read_bytes(addr, REG_ACCEL_DATA_X1, buf, 6)) return;
    int16_t ax_raw = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t ay_raw = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t az_raw = (int16_t)((buf[5] << 8) | buf[4]);

    // Read gyroscope
    if (!read_bytes(addr, REG_GYRO_DATA_X1, buf, 6)) return;
    int16_t gx_raw = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t gy_raw = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t gz_raw = (int16_t)((buf[5] << 8) | buf[4]);

    // Convert to float
    float acc[3];
    acc[0] = (float)ax_raw / ACCEL_LSB_PER_G;
    acc[1] = (float)ay_raw / ACCEL_LSB_PER_G;
    acc[2] = (float)az_raw / ACCEL_LSB_PER_G;

    const float kDegToRad = (float)(M_PI / 180.0);
    float gyr[3];
    gyr[0] = (float)gx_raw * GYRO_DPS_PER_LSB * kDegToRad;
    gyr[1] = (float)gy_raw * GYRO_DPS_PER_LSB * kDegToRad;
    gyr[2] = (float)gz_raw * GYRO_DPS_PER_LSB * kDegToRad;

    // Update VQF filter with the ACTUAL interval since the previous sample.
    // The main loop targets 100 Hz but jitters: its delay truncates to whole
    // milliseconds and is skipped entirely whenever an iteration overruns, so
    // integrating every sample as a fixed 10 ms accumulates orientation error.
    uint32_t now = micros();
    float dt = (now - s_last[instance]) * 1e-6f;   // unsigned math wraps correctly
    s_last[instance] = now;

    // Guard against the first sample after init and any pathological stall.
    if (dt <= 0.0f || dt > 0.5f) {
        dt = -1.0f;                           // tells VQF to use its nominal rate
    }

    s_vqf[instance].updateGyr(gyr, dt);
    s_vqf[instance].updateAcc(acc);
}

void icm45686_get_quat(uint8_t instance, float quat[4]) {
    if (instance >= ICM_COUNT) return;
    s_vqf[instance].getQuat6D(quat);
}

uint8_t icm45686_whoami(uint8_t instance) {
    return (instance < ICM_COUNT) ? s_whoami[instance] : 0;
}
