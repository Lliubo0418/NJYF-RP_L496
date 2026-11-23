#include "stlm75m2f.h"

HAL_StatusTypeDef STLM75_Init(void)
{
    return BSP_I2C_IsDeviceReady(STLM75M2F_DEV_ADDR, 10, 100);
}

HAL_StatusTypeDef STLM75_ReadTemp(float *temperature)
{
    uint8_t rawData[2] = {0};
    HAL_StatusTypeDef status;
    int16_t tempRaw;

    if (temperature == NULL) {
        return HAL_ERROR;
    }

    status = BSP_I2C_Read(STLM75M2F_DEV_ADDR, STLM75_REG_TEMP, I2C_MEMADD_SIZE_8BIT, rawData, 2, 100);
    if (status != HAL_OK) {
        return status;
    }

    /* 数据手册：16 位数据中高 9 位有效、低 7 位为 0（LSB = 0.5℃）*/
    tempRaw = (int16_t)((rawData[0] << 8) | rawData[1]);
    *temperature = (float)(tempRaw) / 256.0f * 0.5f;

    return HAL_OK;
}