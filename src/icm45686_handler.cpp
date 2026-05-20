/*
 * SPDX-FileCopyrightText: 2026 DrSciCortex
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include "icm45686_handler.h"
#include "vqf.h"
#include <Wire.h>
#include <math.h>

// Registers (minimal subset for ICM-45xxx)
#define ICM_ADDR        0x68
#define REG_WHO_AM_I    0x75
#define REG_PWR_MGMT0   0x4E
#define REG_GYRO_CONFIG0 0x4F
#define REG_ACCEL_CONFIG0 0x50
#define REG_ACCEL_DATA_X1 0x1F
#define REG_GYRO_DATA_X1  0x25

static VQF vqf(0.01f); // 100 Hz sampling rate
static uint32_t last_update = 0;

// Helper to write a register
static void write_reg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(ICM_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

// Helper to read multiple bytes
static bool read_bytes(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(ICM_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    
    uint8_t read = Wire.requestFrom(ICM_ADDR, (int)len);
    if (read != len) return false;
    
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = Wire.read();
    }
    return true;
}

bool icm45686_init() {
    uint8_t whoami = 0;
    if (!read_bytes(REG_WHO_AM_I, &whoami, 1)) return false;
    
    if (whoami == 0x00 || whoami == 0xFF) return false;

    // Reset procedure (simplistic)
    write_reg(REG_PWR_MGMT0, 0x0F); // Accel LN, Gyro LN
    delay(10);
    
    // 2000 dps, 100 Hz
    write_reg(REG_GYRO_CONFIG0, 0x08 | 0x06); 
    // 4g, 100 Hz
    write_reg(REG_ACCEL_CONFIG0, 0x02 | 0x06);
    
    last_update = micros();
    vqf.resetState();
    return true;
}

// VQF Fusion Integration
void icm45686_update() {
    uint8_t buf[6];
    
    // Read accelerometer
    if (!read_bytes(REG_ACCEL_DATA_X1, buf, 6)) return;
    int16_t ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay_raw = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az_raw = (int16_t)((buf[4] << 8) | buf[5]);

    // Read gyroscope
    if (!read_bytes(REG_GYRO_DATA_X1, buf, 6)) return;
    int16_t gx_raw = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t gy_raw = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t gz_raw = (int16_t)((buf[4] << 8) | buf[5]);

    // Convert to float
    float acc[3];
    acc[0] = (float)ax_raw / 8192.0f; // 4g scale
    acc[1] = (float)ay_raw / 8192.0f;
    acc[2] = (float)az_raw / 8192.0f;
    
    float gyr[3];
    gyr[0] = (float)gx_raw * (2000.0f / 32768.0f) * (M_PI / 180.0f);
    gyr[1] = (float)gy_raw * (2000.0f / 32768.0f) * (M_PI / 180.0f);
    gyr[2] = (float)gz_raw * (2000.0f / 32768.0f) * (M_PI / 180.0f);

    // Update VQF filter
    vqf.updateGyr(gyr);
    vqf.updateAcc(acc);
}

void icm45686_get_quat(float quat[4]) {
    vqf.getQuat6D(quat);
}
