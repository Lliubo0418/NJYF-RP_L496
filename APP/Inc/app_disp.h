#ifndef __APP_DISP_H
#define __APP_DISP_H

#include <stdint.h>
#include "bsp_usart.h"
#include "dispproto.h"
#include "app_algorithm.h"   /* Algo_RadarResult_t */

/* 主板侧协议接口。
 * 发送（下行，主板 -> 显示板）：雷达业务在测距完成后调用 Disp_DownSendXxx()。
 * 接收（上行，显示板 -> 主板）：USART1 ISR 仅入环形缓冲，主循环 Disp_Poll()
 *       出队解析，完整帧交给 Disp_OnUplink() 执行具体业务动作。
 */

/* 协议初始化：注册 USART1 逐字节接收回调并启动接收。
 * 必须在 BSP_USART_Init() 之后调用（main.c 中已挂接）。 */
void Disp_Init(void);

/* ---------------- 下行发送（主板 -> 显示板） ---------------- */

/* 下行 0x01 MEAS：把 res 的 distance/position/delta_amp/peak_count 与 mode
 * 打包成 14 字节负载发送。mode 为测量模式标识（1 字节，业务自定义）。 */
void Disp_DownSendMeas(const Algo_RadarResult_t *res, uint8_t mode);

/* 下行 0x02 ECHO：发送 128 点归一化(0~255)回波包络。
 * echo128 为 NULL 时直接返回；同时缓存最近一帧供 REQ_ECHO 立即回送。 */
void Disp_DownSendEcho(const uint8_t *echo128);

/* 下行 0x03 DIAG：reliability=可靠性(0~100,1B) status=状态字(1B)
 *   peakMinEmpty/peakMaxEmpty=空高极值(f32) temperature=传感器温度(f32) */
void Disp_DownSendDiag(uint8_t reliability, uint8_t status, float peakMinEmpty, float peakMaxEmpty, float temperature);

/* 下行 0x04 INFO：发送传感器类型与固件版本（内容取自固定版本号）。 */
void Disp_DownSendInfo(void);

/* 下行 0x05 PARAM_DUMP：把 gRadarConfig 按 dispproto.h 的 81 字节布局序列化发送。 */
void Disp_DownSendParamDump(void);

/* 下行 0x06 ECHO_TYPED：发送带曲线类型标识的 129 字节帧
 * （curveType(u8) + 128 点归一化数据）。type 取 DISP_CURVE_* 。 */
void Disp_DownSendEchoTyped(uint8_t type, const uint8_t *data128);

/* ---------------- 上行接收（显示板 -> 主板） ---------------- */

/* 主循环轮询：从 USART1 环形缓冲逐字节出队、解析上行帧并执行业务。
 * ISR（Disp_OnRxByte）仅入队，帧解析与下行响应发送均在此处主循环上下文完成，
 * 保证采样定时器（TIM7，优先级更高）不被串口处理阻塞。 */
void Disp_Poll(void);

/* 上行帧业务处理：REQ_ECHO/REQ_MEAS/KEY/SET_PARAM/SET_STR/REQ_INFO/REQ_PARAM_DUMP。 */
void Disp_OnUplink(uint8_t cmd, const uint8_t *payload, uint8_t len);

/* 查询并清除测量请求标志（REQ_MEAS / KEY=0x01 置位）。
 * app_radar.c 每轮测量下发 MEAS 后调用，标志返回 1 表示该帧同时是对补发请求的响应。 */
uint8_t Disp_ConsumeMeasReq(void);

/* 回波包络构建：把长度 adc_len 的 ADC 原始包络(uint16)抽点归一化为 128 点。
 * 默认实现为简单抽点 + 线性归一化，可替换为均值/最大抽取、对数压缩等算法。 */
void Disp_BuildEcho(const uint16_t *adc, uint16_t adc_len, uint8_t *out128);

/* 曲线包络构建（带外部归一化基准）：
 * 与 Disp_BuildEcho 同构（等距抽点 + 归一化到 0~255），但除数由调用方给定。
 *
 * 【为什么要传入 norm_peak 而不是各自归一化】
 *   假回波曲线（空罐学习基线）是"空罐背景"，幅度远小于真实回波。若按自身峰值
 *   归一化，会把微弱背景放大到满屏，看起来像有条强回波 —— 是误导性的显示。
 *   传入回波曲线的同一峰值做基准，两条曲线幅度才可比：
 *   基线上某点的值越低，说明该处越"干净"，这正是诊断时想看的信息。
 *
 * 参数：
 *   src      - 源数据（一般为 s_baseline，长度 sample_count）
 *   src_len  - 源数据点数
 *   norm_peak- 归一化除数（传回波峰值即 algo 的 maxv；<=0 时退化为自身峰值）
 *   out128   - 输出 128 点归一化结果
 * 返回：实际用于归一化的除数（便于调用方复用/诊断打印） */
uint16_t Disp_BuildCurveNorm(const uint16_t *src, uint16_t src_len,
                             uint16_t norm_peak, uint8_t *out128);

/* 同 Disp_BuildEcho，但额外返回本次使用的归一化除数（自身峰值）。
 * 供调用方拿到回波峰值后，再用同一基准去构建虚假回波曲线，使两条曲线
 * 幅度可比（详见 Disp_BuildCurveNorm 注释）。 */
uint16_t Disp_BuildEchoEx(const uint16_t *adc, uint16_t adc_len, uint8_t *out128);

#endif /* __APP_DISP_H */
