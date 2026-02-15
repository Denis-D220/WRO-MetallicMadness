#include "platform2.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

extern I2C_HandleTypeDef hi2c2;

uint8_t VL53L4CD_RdByte(Dev_t dev, uint16_t RegisterAdress, uint8_t *value)
{
    uint8_t reg[2];
    reg[0] = (RegisterAdress >> 8) & 0xFF;
    reg[1] = RegisterAdress & 0xFF;
    if (HAL_I2C_Master_Transmit(&hi2c2, dev, reg, 2, 100) != HAL_OK)
        return 1;
    return HAL_I2C_Master_Receive(&hi2c2, dev, value, 1, 100);
}

uint8_t VL53L4CD_RdWord(Dev_t dev, uint16_t RegisterAdress, uint16_t *value)
{
    uint8_t buffer[2];
    uint8_t status = VL53L4CD_RdMulti(dev, RegisterAdress, buffer, 2);
    *value = (buffer[0] << 8) | buffer[1];
    return status;
}

uint8_t VL53L4CD_RdDWord(Dev_t dev, uint16_t RegisterAdress, uint32_t *value)
{
    uint8_t buffer[4];
    uint8_t status = VL53L4CD_RdMulti(dev, RegisterAdress, buffer, 4);
    *value = ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16) |
             ((uint32_t)buffer[2] << 8) | buffer[3];
    return status;
}

uint8_t VL53L4CD_WrByte(Dev_t dev, uint16_t RegisterAdress, uint8_t value)
{
    uint8_t data[3];
    data[0] = (RegisterAdress >> 8) & 0xFF;
    data[1] = RegisterAdress & 0xFF;
    data[2] = value;
    return HAL_I2C_Master_Transmit(&hi2c2, dev, data, 3, 100);
}

uint8_t VL53L4CD_WrWord(Dev_t dev, uint16_t RegisterAdress, uint16_t value)
{
    uint8_t data[4];
    data[0] = (RegisterAdress >> 8) & 0xFF;
    data[1] = RegisterAdress & 0xFF;
    data[2] = (value >> 8) & 0xFF;
    data[3] = value & 0xFF;
    return HAL_I2C_Master_Transmit(&hi2c2, dev, data, 4, 100);
}

uint8_t VL53L4CD_WrDWord(Dev_t dev, uint16_t RegisterAdress, uint32_t value)
{
    uint8_t data[6];
    data[0] = (RegisterAdress >> 8) & 0xFF;
    data[1] = RegisterAdress & 0xFF;
    data[2] = (value >> 24) & 0xFF;
    data[3] = (value >> 16) & 0xFF;
    data[4] = (value >> 8) & 0xFF;
    data[5] = value & 0xFF;
    return HAL_I2C_Master_Transmit(&hi2c2, dev, data, 6, 100);
}

uint8_t VL53L4CD_RdMulti(Dev_t dev, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size)
{
    uint8_t reg[2];
    reg[0] = (RegisterAdress >> 8) & 0xFF;
    reg[1] = RegisterAdress & 0xFF;
    if (HAL_I2C_Master_Transmit(&hi2c2, dev, reg, 2, 100) != HAL_OK)
        return 1;
    return HAL_I2C_Master_Receive(&hi2c2, dev, p_values, size, 100);
}

uint8_t VL53L4CD_WaitMs(Dev_t dev, uint32_t TimeMs)
{
    HAL_Delay(TimeMs);
    return 0;
}
