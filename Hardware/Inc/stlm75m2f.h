#ifndef __STLM75M2F_H
#define __STLM75M2F_H

#include "bsp_i2c.h"
#include <stdint.h>

// STLM75M2F 的 I2C 7 位设备地址 0x49（A0/A1/A2 接地）；HAL 需传左移 1 位后的 8 位格式
#define STLM75M2F_DEV_ADDR      (0x49 << 1)

#define STLM75_REG_TEMP         (0x00) // 温度寄存器
#define STLM75_REG_CONFIG       (0x01) // 配置寄存器
#define STLM75_REG_THYST        (0x02) // 滞后温度寄存器
#define STLM75_REG_TOS          (0x03) // 过温阈值寄存器

/**
  * @brief  初始化 STLM75M2F 温度传感器
  * @retval HAL 状态
  */

HAL_StatusTypeDef STLM75_Init(void);
/**
  * @brief  读取 STLM75M2F 的温度值
  * @param  temperature: 存储读取到的温度值（单位：摄氏度）
  * @retval HAL 状态
  */

HAL_StatusTypeDef STLM75_ReadTemp(float *temperature);

#endif