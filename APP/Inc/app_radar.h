#ifndef __APP_RADAR_H
#define __APP_RADAR_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  雷达业务初始化
  * @note   仅做算法侧标定初始化。zero_offset 为测距起点到 adc_buf[0] 的距离偏移（米），需现场标定；
  *         此处置 0：tx_sample_valid 有效时走飞行时间法、不依赖该值。
  * @retval 无
  * @complexity  时间 O(1)，空间 O(1)
  */

void App_Radar_Init(void);

/**
  * @brief  雷达业务周期运行（主循环调用）
  * @note   职责：① 采样消费去重（每完成一次采样只跑一次算法，s_sample_consumed）；
  *         ② S6 周期闭环（被控量 = 实测 S6 周期，目标 T* = 44.567ms）：周期 < 下界 → S9 充电，
  *            周期 > 上界 → S7 放电，带内不动作（窗口定义见 bsp_timer.h）。
  * @retval 无（副作用：算法处理 / 脉冲发送 / 串口输出 / 下行帧）
  * @complexity  时间 O(N*W)（算法分支）+ O(1)（校正分支），空间 O(N) 栈
  */

void App_Radar_Run(void);

/* ===== 调试标定通道开关（取消下行注释即启用）=====
 * 打开后主循环调用 App_Debug_Task()，解析 UART4 的 "CAL <dps> <offset>"。
 * ⚠ 量产固件必须保持关闭：UART4 无鉴权可改测量标定，且会把 sscanf 拖进镜像。 */
/* #define DEBUG_CAL_CHANNEL  1 */

/* 调试标定通道：解析 UART4 的 "CAL <dps> <offset>"，调用 Algo_SetCalibration，仅产线/调试用。
 * ⚠ 仅 DEBUG_CAL_CHANNEL 定义时需链接；与双板串口协议独立。 */
void App_Debug_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_RADAR_H */
