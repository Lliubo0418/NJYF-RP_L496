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

/* ============================================================
 *  4-20mA 电流标定常量（DAC 码值 ↔ 环路电流）
 *
 *  AD5421 是 16 位 DAC，环路电流 = F(RANGE 引脚跳线, DAC 码值)。
 *  量程由芯片 RANGE0/RANGE1 引脚（硬件跳线）选择，软件不可改：
 *    RANGE1/RANGE0 = COM/COM   → 4 mA ~ 20 mA
 *    RANGE1/RANGE0 = COM/DVDD  → 3.8 mA ~ 21 mA
 *    RANGE1/RANGE0 = DVDD/COM  → 3.2 mA ~ 24 mA
 *  ⚠ 本工程原理图上 RANGE 引脚的实际接法未在代码中体现，
 *    标定前必须实测确认，否则 mA↔码值 换算全错。
 *
 *  ⚠ 报警电流（20.5mA / 22.0mA）不要用 DAC 码值硬凑：
 *    AD5421 提供硬件级报警命令 AD5421_CMD_FORCE_ALARM(0x06)，
 *    芯片会按 ALARM_CURRENT_DIRECTION 引脚输出自身的低/高报警电流。
 *    用 DAC 码值逼近会因为量程不同而算错，且随温度漂移。
 *
 *  ⚠ 最小电流 3.8mA（显示板 MENU_CURRENT_MIN 档位 1）无法用当前 4~20mA
 *    标定的 0..65535 码值表达 —— DAC 0 已经是 4mA 端点，要输出更小的
 *    3.8mA 需要"小于 0 的码值"，物理上不存在。两条可行路径：
 *      ① 把 RANGE0/RANGE1 跳线改到 "3.8~21mA" 档（则 3.8mA ↔ DAC 0，
 *         代价是 20mA 上界跟着变 21mA，需重标整套换算）；
 *      ② 保持 4~20mA 档，实测确认无法真正到 3.8mA。
 *    两条路都必须【实测】，不能线性外推。在实测出结果前，
 *    AD5421_DAC_3P8MA_CALIBRATED 保持【不定义】，app_radar.c 中的
 *    3.8mA 地板分支不参与编译（currMin 档 1 与档 0 行为一致 = 4mA），
 *    避免静默输出一个未经验证的错误电流。
 *    实测完成后：定义 AD5421_DAC_3P8MA_CALIBRATED 并把
 *    AD5421_DAC_3P8MA 改成实测码值。
 * ============================================================ */
/* #define AD5421_DAC_3P8MA_CALIBRATED */   /* ← 实测 3.8mA 码值后取消注释 */
#define AD5421_DAC_3P8MA            0U      /* ← 占位：待实测替换为真实码值 */
HAL_StatusTypeDef AD5421_ForceAlarm(void);

#endif