#ifndef __BSP_DEBUG_H__
#define __BSP_DEBUG_H__

/* ===== 串口调试打印总开关（全工程唯一一处）=====
 * 1 = printf 输出到 USART4；0 = 全部 printf(...) 编译期消除（零运行时间/零 Flash）。
 * ⚠ 仅控制 USART4 调试口，关闭不影响显示板协议帧（USART1 走独立 BSP_USART_Transmit）。
 * ⚠ 本头必须在 stdio.h 之后重定义 printf，否则破坏 stdio.h 的原型声明。 */
#include <stdio.h>

#define DEBUG_PRINT_ENABLE   1

#if !DEBUG_PRINT_ENABLE
  #define printf(...)   ((void)0)
#endif

#endif /* __BSP_DEBUG_H__ */
