#ifndef __24LC256_H
#define __24LC256_H

#include "bsp_i2c.h"
#include <stdint.h>

// 24LC256 的 I2C 7位设备地址（A0/A1/A2 接地）
#define EEPROM_24LC256_ADDR     (0x50 << 1) // 0x50

// 24LC256 的页大小
#define EEPROM_PAGE_SIZE        (64)

// 函数声明
/**
  * @brief  向24LC256写入一个字节
  * @param  addr: 要写入的地址 (0 ~ 32767)
  * @param  data: 要写入的数据
  * @retval HAL 状态
  */

HAL_StatusTypeDef EEPROM_WriteByte(uint16_t addr, uint8_t data);
/**
  * @brief  从24LC256读取一个字节
  * @param  addr: 要读取的地址 (0 ~ 32767)
  * @param  data: 存储读取数据的指针
  * @retval HAL 状态
  */

HAL_StatusTypeDef EEPROM_ReadByte(uint16_t addr, uint8_t *data);
/**
  * @brief  向24LC256写入一页数据（最多64字节）[reference:6]
  * @param  addr: 要写入的起始地址（必须与页边界对齐，即 addr % 64 == 0）
  * @param  pData: 待写入数据的指针
  * @param  len:  待写入数据的长度 (1 ~ 64)
  * @retval HAL 状态
  */

HAL_StatusTypeDef EEPROM_WritePage(uint16_t addr, uint8_t *pData, uint16_t len);
/**
  * @brief  从24LC256读取任意长度的数据
  * @param  addr: 要读取的起始地址
  * @param  pData: 用于存储读取数据的指针
  * @param  len:  要读取的数据长度
  * @retval HAL 状态
  */

HAL_StatusTypeDef EEPROM_ReadBytes(uint16_t addr, uint8_t *pData, uint16_t len);
/**
  * @brief  向24LC256写入任意长度的数据（自动处理跨页拆分）
  * @param  addr: 起始地址 (0~32767)
  * @param  pData: 数据指针
  * @param  len:  数据长度（字节）
  * @retval HAL 状态
  */

HAL_StatusTypeDef EEPROM_WriteBytes(uint16_t addr, uint8_t *pData, uint16_t len);

#endif