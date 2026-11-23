#ifndef __AD5421_H
#define __AD5421_H

#include "bsp_spi.h"
#include <stdint.h>

// AD5421 SYNC 引脚（相当于片选）
#define AD5421_SYNC_PORT        GPIOB
#define AD5421_SYNC_PIN         GPIO_PIN_12
#define AD5421_SYNC_LOW()       HAL_GPIO_WritePin(AD5421_SYNC_PORT, AD5421_SYNC_PIN, GPIO_PIN_RESET)
#define AD5421_SYNC_HIGH()      HAL_GPIO_WritePin(AD5421_SYNC_PORT, AD5421_SYNC_PIN, GPIO_PIN_SET)

// 常用命令/地址字节定义
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
  * @note   AD5421 回读机制特殊：发送读命令后数据在下一次写操作时从 SDO 输出；
  *         本函数发读命令后执行一次 NOP 获取数据。
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

/* ============ 4-20mA 标定常量（DAC 码值 ↔ 环路电流）============
 * AD5421 16 位 DAC；量程由 RANGE0/RANGE1 硬件跳线选择（软件不可改）：
 *   COM/COM → 4~20mA；COM/DVDD → 3.8~21mA；DVDD/COM → 3.2~24mA。
 * ⚠ 本工程 RANGE 引脚实际接法未在代码中体现，标定前必须实测确认，否则 mA↔码值换算全错。
 * ⚠ 报警电流勿用 DAC 码值硬凑：应用硬件命令 AD5421_CMD_FORCE_ALARM(0x06)，否则随量程/温度算错。
 * ⚠ 3.8mA（显示板 MENU_CURRENT_MIN 档 1）无法用 4~20mA 标定的 DAC 码值表达。实测确认前
 *   AD5421_DAC_3P8MA_CALIBRATED 保持【不定义】（app_radar.c 的 3.8mA 地板分支不编译，档 1 与档 0 一致 = 4mA）。 */
/* #define AD5421_DAC_3P8MA_CALIBRATED */   /* ← 实测 3.8mA 码值后取消注释 */
#define AD5421_DAC_3P8MA            0U      /* ← 占位：待实测替换为真实码值 */
HAL_StatusTypeDef AD5421_ForceAlarm(void);

#endif