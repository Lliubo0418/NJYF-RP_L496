#include "bsp_i2c.h"
#include <stdio.h>

HAL_StatusTypeDef BSP_I2C_Write(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddSize, 
                                uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    // 使用HAL库的内存写入函数，适用于大多数I2C存储器
    return HAL_I2C_Mem_Write(&hi2c1, DevAddr, MemAddr, MemAddSize, pData, Size, Timeout);
}

HAL_StatusTypeDef BSP_I2C_Read(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddSize, 
                               uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return HAL_I2C_Mem_Read(&hi2c1, DevAddr, MemAddr, MemAddSize, pData, Size, Timeout);
}

/**
  * @brief  检查I2C从机设备是否就绪（能响应通信）
  * @param  DevAddr: 从机设备地址（7位地址，左移一位）
  * @param  Trials: 尝试次数
  * @param  Timeout: 超时时间（毫秒）
  * @retval HAL 状态
  */
// HAL_StatusTypeDef BSP_I2C_IsDeviceReady(uint16_t DevAddr, uint32_t Trials, uint32_t Timeout)
// {
//     return HAL_I2C_IsDeviceReady(&hi2c1, DevAddr, Trials, Timeout);
// }
HAL_StatusTypeDef BSP_I2C_IsDeviceReady(uint16_t DevAddr, uint32_t Trials, uint32_t Timeout)
{
    /* 仅返回状态，不打印。
       该函数也用于 24LC256 写完成 ACK 轮询（器件写周期内 NACK 属正常），
       打印会造成刷屏；是否报错由调用方决定。*/
    return HAL_I2C_IsDeviceReady(&hi2c1, DevAddr, Trials, Timeout);
}