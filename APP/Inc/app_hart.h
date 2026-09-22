#ifndef __APP_HART_H
#define __APP_HART_H

#include <stdint.h>

/* 最小 HART 从机（命令 0/1/2/3/6），物理层经 AD5700 FSK 调制解调器。
 * PV（主变量）= 物位/距离（米），由 App_HART_SetPV 更新。
 * USART2：1200bps, 8O1，由 App_HART_Init 重新配置。 */

/* 初始化：把 USART2 配置为 HART 参数（1200 8O1），置 AD5700 为接收模式，
 * 并按 gRadarConfig.servHART / servHARTAddr 设定轮询地址（飞卓 p14 §4.6）：
 *   标准模式 → 地址强制 0；多点模式 → 取 servHARTAddr 并夹取到 1~15。
 * ★须在 MX_USART2_UART_Init() 与 App_Config_LoadFromEEPROM() 【之后】调用，
 *   否则读到的是配置默认值，菜单里设的 HART 模式/地址会被开机流程覆盖。 */
void App_HART_Init(void);

/* 主循环调用：解析主机发来的 HART 帧并自动响应。 */
void App_HART_Task(void);

/* 更新主变量（PV）值，供命令 1/2/3 响应使用。单位：米。 */
void App_HART_SetPV(float level_m);

/* 设置轮询地址。仅改运行时变量，不写 EEPROM。
 * 开机由 App_HART_Init 依配置写入；运行中命令 6 亦可改写（HART 标准行为）。 */
void App_HART_SetPollAddr(uint8_t addr);

#endif /* __APP_HART_H */
