/**
 * @file    imu.c
 * @brief   MPU-6050 I2C driver (STM32F405, hi2c1, HAL)
 */

#include "imu.h"
#include "debug_uart.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;   /* Defined by CubeMX in i2c.c */

static HAL_StatusTypeDef WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, buf, 2, 10);
}

static HAL_StatusTypeDef ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef r;
    r = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &reg, 1, 10);
    if (r != HAL_OK) return r;
    return HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, buf, len, 20);
}

uint8_t IMU_Init(void)
{
    HAL_Delay(100);   

    if (WriteReg(MPU6050_PWR_MGMT_1, 0x00) != HAL_OK) {
        DBG_Printf("[IMU] PWR_MGMT_1 write failed\r\n");
        return 1;
    }
    HAL_Delay(10);

    /* Sample rate divider: ODR = 1000 / (1 + SMPLRT_DIV)
       For 200 Hz: SMPLRT_DIV = 4 (DLPF active → gyro ODR = 1 kHz)   */
    if (WriteReg(MPU6050_SMPLRT_DIV, 4) != HAL_OK) return 1;

    if (WriteReg(MPU6050_CONFIG, 0x03) != HAL_OK) return 1;

    if (WriteReg(MPU6050_GYRO_CFG, 0x08) != HAL_OK) return 1;

    if (WriteReg(MPU6050_ACCEL_CFG, 0x10) != HAL_OK) return 1;

    DBG_Printf("[IMU] MPU-6050 init OK (±8g, ±500dps, 200Hz)\r\n");
    return 0;
}

uint8_t IMU_SelfTest(void)
{
    uint8_t who = 0;
    if (ReadRegs(MPU6050_WHO_AM_I, &who, 1) != HAL_OK) return 0;
    return (who == 0x68u) ? 1 : 0;
}

uint8_t IMU_Read(IMU_Sample *out)
{
    uint8_t raw[14];  

    if (ReadRegs(MPU6050_ACCEL_XOUT, raw, 14) != HAL_OK) {
        return 1;
    }

    int16_t ax_raw = (int16_t)((raw[0]  << 8) | raw[1]);
    int16_t ay_raw = (int16_t)((raw[2]  << 8) | raw[3]);
    int16_t az_raw = (int16_t)((raw[4]  << 8) | raw[5]);
    int16_t gx_raw = (int16_t)((raw[8]  << 8) | raw[9]);
    int16_t gy_raw = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz_raw = (int16_t)((raw[12] << 8) | raw[13]);

    out->ax = (float)ax_raw * MPU6050_ACCEL_SCALE;
    out->ay = (float)ay_raw * MPU6050_ACCEL_SCALE;
    out->az = (float)az_raw * MPU6050_ACCEL_SCALE;
    out->gx = (float)gx_raw * MPU6050_GYRO_SCALE;
    out->gy = (float)gy_raw * MPU6050_GYRO_SCALE;
    out->gz = (float)gz_raw * MPU6050_GYRO_SCALE;
    out->timestamp = HAL_GetTick();

    return 0;
}
