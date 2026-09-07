#ifndef __APP_DISP_H
#define __APP_DISP_H

#include <stdint.h>
#include "bsp_usart.h"
#include "dispproto.h"
#include "app_algorithm.h"   /* Algo_RadarResult_t */

/* 主板侧协议接口。
 * 发送：下行命令（主板 -> 显示板）由雷达业务在测距完成后调用。
 * 接收：上行命令（显示板 -> 主板）为预留，注册逐字节回调并解析，
 *       具体动作在 Disp_OnUplink() 中实现。
 */

/* 协议初始化：注册上行接收回调并启动 USART1 单字节接收。无参数。
 * 注意：须在 BSP_USART_Init() 之后调用。 */
/* 协议初始化：注册上行接收回调并启动 USART1 单字节接收。
 * 必须在 BSP_USART_Init() 之后调用（main.c 中已挂接）。
 * 参数：无 */

void Disp_Init(void);

/* 下行发送（主板 -> 显示板） */
/* 下行：发送一帧 MEAS 测量结果（命令 0x01）。
 * 把 res 中的 distance/position/delta_amp/peak_count 与 mode 打包成 14 字节负载。
 * 参数：
 *   res  - 雷达测量结果指针（Algo_RadarResult_t）
 *   mode - 测量模式标识（1 字节，业务自定义含义） */

void Disp_SendMeas(const Algo_RadarResult_t *res, uint8_t mode);
/* 下行：发送一帧 ECHO 回波包络（命令 0x02）。
 * 参数：
 *   echo128 - 已归一化到 0~255 的 128 点回波数组指针（为 NULL 时直接返回） */

void Disp_SendEcho(const uint8_t *echo128);
/* 发送 DIAG 诊断：reliability=可靠性(1B) status=状态字(1B)
 *                  peakMinEmpty/peakMaxEmpty=空高极值(f32) temperature=传感器温度(f32) */
/* 下行：发送一帧 DIAG 诊断信息（命令 0x03）。
 * 参数：
 *   reliability  - 测量可靠性（0~100，1 字节）
 *   status       - 设备状态字（1 字节，业务自定义）
 *   peakMinEmpty - 空高最小峰值（float，4 字节）
 *   peakMaxEmpty - 空高最大峰值（float，4 字节） */

void Disp_SendDiag(uint8_t reliability, uint8_t status, float peakMinEmpty, float peakMaxEmpty, float temperature);

/* 下行：发送一帧 INFO 传感器信息（命令 0x04）。
 * 参数：无（内容取自 App_Config 与固定固件版本号） */

void Disp_SendInfo(void);

/* 回波包络构建（预留）：把 adc 降采样归一化为 128 点。
 * 默认实现为简单抽点 + 线性归一化，用户可替换为更优算法。
 *   adc=ADC原始包络 adc_len=长度 out128=输出缓冲(≥128B) */
/* 回波包络构建（预留）：把长度为 adc_len 的 ADC 原始包络降采样归一化为 128 点。
 * 默认实现为“简单抽点 + 线性归一化”，用户可替换为更优算法（均值/最大抽取、对数压缩等）。
 * 参数：
 *   adc      - ADC 原始包络数组指针（uint16_t）
 *   adc_len  - ADC 数组长度
 *   out128   - 输出缓冲，至少 128 字节，存放归一化后的回波 */

void Disp_BuildEcho(const uint16_t *adc, uint16_t adc_len, uint8_t *out128);

/* 上行命令处理回调（预留，用户实现具体动作）
 *   cmd=命令字 payload=负载 len=负载长度 */
/* 上行命令钩子（默认空实现）：用户在此实现显示板发来命令的具体动作。
 * 参数：
 *   cmd     - 上行命令字（DISP_CMD_REQ_ECHO / REQ_MEAS / KEY / SET_PARAM）
 *   payload - 负载数据指针
 *   len     - 负载字节数 */

void Disp_OnUplink(uint8_t cmd, const uint8_t *payload, uint8_t len);

#endif /* __APP_DISP_H */
