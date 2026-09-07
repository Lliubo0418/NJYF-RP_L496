#ifndef __AD5421_H
#define __AD5421_H

#include "bsp_spi.h"
#include <stdint.h>

// 根据硬件连接定义同步（SYNC）引脚（相当于片选）
#define AD5421_SYNC_PORT        GPIOB
#define AD5421_SYNC_PIN         GPIO_PIN_12  // 举例：假设SYNC接在PB12
#define AD5421_SYNC_LOW()       HAL_GPIO_WritePin(AD5421_SYNC_PORT, AD5421_SYNC_PIN, GPIO_PIN_RESET)
#define AD5421_SYNC_HIGH()      HAL_GPIO_WritePin(AD5421_SYNC_PORT, AD5421_SYNC_PIN, GPIO_PIN_SET)

// 常用命令/地址字节定义[reference:9]
#define AD5421_CMD_WRITE_DAC        (0x01)  // 写入 DAC 寄存器
#define AD5421_CMD_WRITE_CONTROL    (0x02)  // 写入控制寄存器
#define AD5421_CMD_WRITE_OFFSET     (0x03)  // 写入偏移调整寄存器
#define AD5421_CMD_WRITE_GAIN       (0x04)  // 写入增益调整寄存器
#define AD5421_CMD_LOAD_DAC         (0x05)  // 加载 DAC (数据位无关)
#define AD5421_CMD_FORCE_ALARM      (0x06)  // 强制报警电流 (数据位无关)
#define AD5421_CMD_RESET            (0x07)  // 复位 (数据位无关)
#define AD5421_CMD_MEASURE          (0x08)  // 启动 VLOOP/温度测量 (数据位无关)
#define AD5421_CMD_NOP              (0x09)  // 空操作 (数据位无关)
#define AD5421_CMD_READ_DAC         (0x81)  // 读 DAC 寄存器
#define AD5421_CMD_READ_CONTROL     (0x82)  // 读控制寄存器
#define AD5421_CMD_READ_OFFSET      (0x83)  // 读偏移调整寄存器
#define AD5421_CMD_READ_GAIN        (0x84)  // 读增益调整寄存器
#define AD5421_CMD_READ_FAULT       (0x85)  // 读故障寄存器

// 函数声明
/**
  * @brief  初始化 AD5421
  * @retval HAL 状态
  */

HAL_StatusTypeDef AD5421_Init(void);
/**
  * @brief  向 AD5421 写入一个 24 位帧
  * @param  cmd:  8 位命令/地址字节
  * @param  data: 16 位数据
  * @retval HAL 状态
  */

HAL_StatusTypeDef AD5421_WriteRegister(uint8_t cmd, uint16_t data);
/**
  * @brief  从 AD5421 读取一个寄存器（回读模式）
  * @note   AD5421 的回读机制特殊：发送读命令后，数据会在下一次写操作时从 SDO 输出[reference:12]
  *         此函数简化实现，发送读命令后执行一次 NOP 以获取数据。
  * @param  cmd:  8 位读命令（0x81~0x85）
  * @param  pData: 存储读取的 16 位数据
  * @retval HAL 状态
  */

HAL_StatusTypeDef AD5421_ReadRegister(uint8_t cmd, uint16_t *pData);
/**
  * @brief  设置 DAC 输出电流对应的码值
  * @param  dacCode: 16 位 DAC 码值 (0 ~ 65535)
  * @retval HAL 状态
  */

HAL_StatusTypeDef AD5421_SetDACOutput(uint16_t dacCode);
/**
  * @brief  读取故障寄存器
  * @param  pFaultData: 存储故障寄存器值 (低 8 位为 ADC 转换结果)
  * @retval HAL 状态
  */

HAL_StatusTypeDef AD5421_ReadFaultRegister(uint16_t *pFaultData);

#endif