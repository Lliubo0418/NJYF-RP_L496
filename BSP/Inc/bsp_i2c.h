#ifndef __BSP_I2C_H
#define __BSP_I2C_H

#include "stm32l4xx_hal.h"

// 根据你CubeMX中I2C句柄的名称进行修改，例如是 hi2c1 或 hi2c2
extern I2C_HandleTypeDef hi2c1; 

// 函数声明：向从机写入数据
/**
  * @brief  通过I2C向从机的指定内存地址写入数据
  * @param  DevAddr: 从机设备地址（7位地址，左移一位）
  * @param  MemAddr: 从机内部存储器地址
  * @param  MemAddSize: 内部存储器地址宽度（I2C_MEMADD_SIZE_8BIT 或 I2C_MEMADD_SIZE_16BIT）
  * @param  pData: 待写入数据的指针
  * @param  Size: 待写入数据的字节数
  * @param  Timeout: 超时时间（毫秒）
  * @retval HAL 状态
  */

HAL_StatusTypeDef BSP_I2C_Write(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddSize, 
                                uint8_t *pData, uint16_t Size, uint32_t Timeout);

// 函数声明：从从机读取数据
/**
  * @brief  通过I2C从从机的指定内存地址读取数据
  * @param  DevAddr: 从机设备地址（7位地址，左移一位）
  * @param  MemAddr: 从机内部存储器地址
  * @param  MemAddSize: 内部存储器地址宽度（I2C_MEMADD_SIZE_8BIT 或 I2C_MEMADD_SIZE_16BIT）
  * @param  pData: 用于存储读取数据的指针
  * @param  Size: 要读取的字节数
  * @param  Timeout: 超时时间（毫秒）
  * @retval HAL 状态
  */

HAL_StatusTypeDef BSP_I2C_Read(uint16_t DevAddr, uint16_t MemAddr, uint16_t MemAddSize, 
                               uint8_t *pData, uint16_t Size, uint32_t Timeout);

// 函数声明：检查从机是否就绪（常用于等待EEPROM内部写操作完成）
HAL_StatusTypeDef BSP_I2C_IsDeviceReady(uint16_t DevAddr, uint32_t Trials, uint32_t Timeout);

#endif