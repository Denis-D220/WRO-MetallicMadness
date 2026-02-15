#include "sensor_imu.h"
#include <stdio.h>

/* Helper function to write a register */
static uint8_t sensor_imu_write_register(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(hi2c, LSM6DS3_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}

/* Helper function to read a register */
static uint8_t sensor_imu_read_register(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *value) {
    return HAL_I2C_Mem_Read(hi2c, LSM6DS3_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, value, 1, HAL_MAX_DELAY);
}

/* Initialize the LSM6DS3TR-C IMU */
uint8_t sensor_imu_init(I2C_HandleTypeDef *hi2c) {
    uint8_t who_am_i = 0;

//    /* Check WHO_AM_I register */
//    if (sensor_imu_read_register(hi2c, LSM6DS3_WHO_AM_I, &who_am_i) != HAL_OK || who_am_i != 0x69) {
//        printf("LSM6DS3TR-C not detected! WHO_AM_I = 0x%02X\n", who_am_i);
//        return HAL_ERROR;
//    }
    /* Check WHO_AM_I register */
    if (sensor_imu_read_register(hi2c, LSM6DS3_WHO_AM_I, &who_am_i) != HAL_OK || who_am_i != 0x6A) {
        printf("LSM6DS3TR-C not detected! WHO_AM_I = 0x%02X\n", who_am_i);
        return HAL_ERROR;
    }

    printf("LSM6DS3TR-C detected! WHO_AM_I = 0x%02X\n", who_am_i);

    /* Configure accelerometer (CTRL1_XL) - 2g, 104Hz */
    if (sensor_imu_write_register(hi2c, LSM6DS3_CTRL1_XL, 0x50) != HAL_OK) {
        printf("Failed to configure accelerometer\n");
        return HAL_ERROR;
    }

    /* Configure gyroscope (CTRL2_G) - 250dps, 104Hz */
    if (sensor_imu_write_register(hi2c, LSM6DS3_CTRL2_G, 0x50) != HAL_OK) {
        printf("Failed to configure gyroscope\n");
        return HAL_ERROR;
    }

    printf("LSM6DS3TR-C initialized successfully!\n");
    return HAL_OK;
}

/* Read accelerometer values */
uint8_t sensor_imu_read_accel(I2C_HandleTypeDef *hi2c, int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t data[6];

    if (HAL_I2C_Mem_Read(hi2c, LSM6DS3_I2C_ADDR, LSM6DS3_OUTX_L_XL, I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    *ax = (int16_t)((data[1] << 8) | data[0]);
    *ay = (int16_t)((data[3] << 8) | data[2]);
    *az = (int16_t)((data[5] << 8) | data[4]);

    return HAL_OK;
}

/* Read gyroscope values */
uint8_t sensor_imu_read_gyro(I2C_HandleTypeDef *hi2c, int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t data[6];

    if (HAL_I2C_Mem_Read(hi2c, LSM6DS3_I2C_ADDR, LSM6DS3_OUTX_L_G, I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    *gx = (int16_t)((data[1] << 8) | data[0]);
    *gy = (int16_t)((data[3] << 8) | data[2]);
    *gz = (int16_t)((data[5] << 8) | data[4]);

    return HAL_OK;
}
