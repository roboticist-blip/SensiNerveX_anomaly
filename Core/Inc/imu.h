/**
 * @file    imu.h
 * @brief   MPU-6050 driver for STM32F405 (I2C1, HAL)
 *
 * Reads accelerometer + gyroscope, converts to float SI units.
 * Accelerometer: ±8g range → 4096 LSB/g
 * Gyroscope    : ±500 dps  → 65.5  LSB/(deg/s)
 *
 * Sampling driven by timer-based polling at SNX_SAMPLE_RATE_HZ (200 Hz).
 * No DMA required — I2C burst read takes ~250 µs at 400 kHz.
 */

#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "features.h"  /* IMU_Sample */

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_ADDR        (0x68u << 1)   
#define MPU6050_WHO_AM_I    0x75u
#define MPU6050_PWR_MGMT_1  0x6Bu
#define MPU6050_SMPLRT_DIV  0x19u
#define MPU6050_CONFIG      0x1Au
#define MPU6050_GYRO_CFG    0x1Bu
#define MPU6050_ACCEL_CFG   0x1Cu
#define MPU6050_ACCEL_XOUT  0x3Bu

#define MPU6050_ACCEL_SCALE (1.0f / 4096.0f)   
#define MPU6050_GYRO_SCALE  (1.0f / 65.5f)    

/**
 * @brief Initialise MPU-6050: wake, configure ODR and ranges.
 * @return 0 on success, non-zero on I2C error.
 */
uint8_t IMU_Init(void);

/**
 * @brief Read one sample (blocking I2C burst).
 * @param out Populated with float accel (g) + gyro (deg/s) + HAL_GetTick()
 * @return 0 on success.
 */
uint8_t IMU_Read(IMU_Sample *out);

/**
 * @brief Self-test: reads WHO_AM_I register.
 * @return 1 if device responded correctly, 0 otherwise.
 */
uint8_t IMU_SelfTest(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H */
