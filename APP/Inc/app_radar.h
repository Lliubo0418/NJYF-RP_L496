#ifndef __APP_RADAR_H
#define __APP_RADAR_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  雷达业务初始化
  * @note   仅做算法侧标定初始化；外设/存储/通信的接入留待后续。
  *         zero_offset 为发射时刻到 adc_buf[0] 的距离偏移（米），需现场标定，
  *         此处先置 0：若 tx_sample_valid 有效则走飞行时间法、不依赖该值。
  * @retval 无
  * @complexity  时间 O(1)，空间 O(1)
  */

void App_Radar_Init(void);

/**
  * @brief  雷达业务周期运行（主循环调用）
  * @note   两个职责：
  *         ① 采样消费管理：每完成一次采样只跑一次算法（s_sample_consumed 去重）
  *         ② S6 脉宽校正控制：22-23ms 目标区间，偏低→S9充电，偏高→S7放电
  * @retval 无（副作用：算法处理 / 脉冲发送 / 串口输出 / 下行帧）
  * @complexity  时间 O(N*W)（算法分支）+ O(1)（校正分支），空间 O(N) 栈
  */

void App_Radar_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_RADAR_H */
