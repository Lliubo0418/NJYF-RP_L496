#ifndef __BSP_GPIO_H__
#define __BSP_GPIO_H__

#include "main.h"       /* CubeMX 生成的引脚宏（引脚真值来源） */
#include <stdbool.h>

/* ============================================================
 *  BSP GPIO 控制
 *  基于 Core/Src/gpio.c 与 Core/Inc/main.h 的引脚定义。
 *  所有板级 GPIO 操作集中于此，供各驱动调用，避免引脚定义散落。
 * ============================================================ */

/* ---------- SPI 片选 ----------
 *  SPI1_CS(PA4) / ADC_CS_X(PC4)：板上 CS 路径经反相器 → MCU SET=芯片 CS LOW(选中)
 *  SPI2_CS(PB12) (AD5421 SYNC)：直连，标准低有效 → MCU RESET=选中               */

/* ============================================================
 *  实现约定：
 *  - 所有引脚宏一律来自 main.h (_Pin/_GPIO_Port)
 *  - EXTI 输入引脚只读（IDR 仍然可读），不做 WritePin
 * ============================================================ */

void BSP_GPIO_SPI1_CS_Select(void);      /* PA4  SPI1_CS                */
void BSP_GPIO_SPI1_CS_Deselect(void);
void BSP_GPIO_SPI2_CS_Select(void);      /* PB12 SPI2_CS(AD5421 SYNC)   */
void BSP_GPIO_SPI2_CS_Deselect(void);
void BSP_GPIO_ADC_CS_X_Select(void);       /* PC4  ADC_CS_X - 反相输出    */
void BSP_GPIO_ADC_CS_X_Deselect(void);

/* ---- RS485 方向控制（USART3，单 DE 引脚）----
 * 收发器 DE 与 /RE 共接 PC12：HIGH=发送, LOW=接收              */
void BSP_GPIO_RS485_TxMode(void);        /* PC12 = HIGH */
void BSP_GPIO_RS485_RxMode(void);        /* PC12 = LOW  */

/* ---------- HART 调制解调器（USART2, AD5700）----------
 *  AD5700 RTS: 0=发送模式, 1=接收模式
 *  AD5700 /RST: 低有效复位
 */

void BSP_GPIO_HART_SetTransmit(void);    /* RTS=LOW  -> 发送 HART       */
void BSP_GPIO_HART_SetReceive(void);     /* RTS=HIGH -> 接收 HART       */
void BSP_GPIO_HART_Reset(bool assert);   /* RST=LOW 复位(低有效)        */
bool BSP_GPIO_HART_CarrierDetect(void);  /* CD 电平(注意:CubeMX当前配为输出,建议改输入) */

/* ---------- 24LC256 EEPROM 写保护 ----------
 *  WP=HIGH 受保护, WP=LOW 可写
 */

void BSP_GPIO_EEPROM_WriteProtect(bool enable); /* PD2: HIGH=保护 LOW=可写 */

/* ---------- STLM75 温度告警 ----------
 *  STLM75 INT 开漏低有效；返回 true=引脚低=告警触发
 */

bool BSP_GPIO_TEMP_IntActive(void);      /* PC12 TEMP_INT 引脚: true=低=告警触发 */

/* ---------- AD5421/AD5241 DAC 故障 ----------
 *  FAULT 开漏低有效；返回 true=引脚低=故障
 */

bool BSP_GPIO_DAC_FaultActive(void);     /* PB2 FAULT: true=低=有故障(开漏低有效) */
/* ---------- 5V 使能 ----------
 *  HIGH=使能；LOW=关闭（原理图极性请在硬件上二次核实）
 */

void BSP_GPIO_S5_Enable(bool on);        /* PC5: HIGH=使能(默认假设,依原理图核实) */

/* ---------- 通用输出 IO ----------
 *  IO1=PC0, IO2=PC1, IO3=PC2, IO2B5=PB5
 */

void BSP_GPIO_IO1_Set(bool on);          /* PC0 */
void BSP_GPIO_IO2_Set(bool on);          /* PC1 */
void BSP_GPIO_IO3_Set(bool on);          /* PC2 */
void BSP_GPIO_IO2B5_Set(bool on);        /* PB5 */

#endif /* __BSP_GPIO_H__ */
