#ifndef __BSP_DEBUG_H__
#define __BSP_DEBUG_H__

/* ============================================================
 *  串口调试打印总开关（全工程唯一一处）
 *
 *  DEBUG_PRINT_ENABLE = 1 : 调试 printf 输出到 USART4（开发/上板验证阶段）
 *  DEBUG_PRINT_ENABLE = 0 : 全部 printf(...) 编译期消除
 *                           —— 零运行时间、零 Flash（格式串一并被优化掉）、
 *                              无需逐处注释代码。
 *
 *  注意：
 *   - 仅控制调试口 USART4 的 printf；与显示板的协议帧（USART1 的
 *     Disp_DownSendMeas/Disp_DownSendDiag 等，走独立 BSP_USART_Transmit）无关，
 *     关闭本开关不影响双板通信。
 *   - fputc 重定向函数（bsp_usart.c）保留不删，重新打开开关即可恢复打印。
 *   - 本头必须在 stdio.h 之后（或先包含 stdio.h）再重定义 printf，
 *     否则会破坏 stdio.h 中 printf 的原型声明。
 * ============================================================ */
#include <stdio.h>

#define DEBUG_PRINT_ENABLE   1

#if !DEBUG_PRINT_ENABLE
  #define printf(...)   ((void)0)
#endif

#endif /* __BSP_DEBUG_H__ */
