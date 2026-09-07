#include "stlm75m2f.h"

HAL_StatusTypeDef STLM75_Init(void)
{
    // 可以在这里进行一些初始化配置，比如读取设备ID或配置寄存器
    // 简单起见，只检查设备是否在线
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

    // 从温度寄存器读取2字节数据，地址宽度为8位
    status = BSP_I2C_Read(STLM75M2F_DEV_ADDR, STLM75_REG_TEMP, I2C_MEMADD_SIZE_8BIT, rawData, 2, 100);
    if (status != HAL_OK) {
        return status;
    }

    // 将两个字节组合成16位有符号数
    // 根据数据手册，温度数据为高9位有效，低7位为0
    tempRaw = (int16_t)((rawData[0] << 8) | rawData[1]);
    // 右移7位得到9位有符号温度值，再乘以0.5得到实际温度
    // 或者直接计算: tempRaw / 256.0f * 0.5f? 
    // 标准计算方法：tempRaw >> 7 得到整数部分，再 * 0.5
    // 但更严谨的浮点计算如下：
    *temperature = (float)(tempRaw) / 256.0f * 0.5f; // 实际温度 = (tempRaw / 256) * 0.5

    return HAL_OK;
}