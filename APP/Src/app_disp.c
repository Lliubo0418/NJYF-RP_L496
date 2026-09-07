/* 主板侧：自定义协议发送（下行）+ 上行命令接收解析（预留）
 *
 * 下行：雷达业务在测距完成后调用 Disp_SendMeas() 等把数据发往显示板。
 * 上行：显示板发来的命令（0x81~0x84）经逐字节状态机解析后交给 Disp_OnUplink()。
 *
 * 注意：L496 的 BSP 层（bsp_usart.c）不在 HAL_UART_RxCpltCallback 内自动重武装，
 *       因此本文件在 Disp_OnRxByte 末尾手动调用 BSP_USART_Receive_IT 重新启动接收。
 *
 * 设计原则：本文件只做“传输 + 帧解析 + 字段打包”，具体的触发频率、
 *       回波降采样算法、上行命令动作均为预留项，用户按需细化。
 */
#include "app_disp.h"
#include "app_algorithm.h"
#include "app_config.h"
#include "bsp_timer.h"    /* adc_buf, ADC_SAMPLE_COUNT */
#include "bsp_usart.h"
#include "dispproto.h"
#include <string.h>

/* ---------------- 下行发送 ---------------- */

/* 下行帧组装并发送：把 cmd + payload 按 AA 55 帧格式封装后发往显示板。
 * 参数：
 *   cmd     - 下行命令字（DISP_CMD_MEAS / ECHO / DIAG）
 *   payload - 负载数据指针
 *   len     - 负载字节数 */
static void Disp_SendFrame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    uint8_t frame[DISP_FRAME_MAX];
    uint8_t idx = 0;
    frame[idx++] = DISP_SYNC1;
    frame[idx++] = DISP_SYNC2;
    frame[idx++] = cmd;
    frame[idx++] = len;
    for (uint8_t i = 0; i < len; i++) frame[idx++] = payload[i];
    frame[idx++] = Disp_CRC(cmd, payload, len);
    BSP_USART_Send(BSP_USART_INSTANCE_1, frame, idx, 100);
}

void Disp_SendMeas(const Algo_RadarResult_t *res, uint8_t mode)
{
    uint8_t p[DISP_MEAS_LEN];
    memcpy(&p[0], &res->distance,  4);
    memcpy(&p[4], &res->position,  4);
    memcpy(&p[8], &res->delta_amp, 4);
    p[12] = (uint8_t)res->peak_count;
    p[13] = mode;
    Disp_SendFrame(DISP_CMD_MEAS, p, DISP_MEAS_LEN);
}

void Disp_SendEcho(const uint8_t *echo128)
{
    if (echo128 == 0) return;
    Disp_SendFrame(DISP_CMD_ECHO, echo128, DISP_ECHO_LEN);
}

void Disp_SendDiag(uint8_t reliability, uint8_t status, float peakMinEmpty, float peakMaxEmpty, float temperature)
{
    uint8_t p[DISP_DIAG_LEN];
    p[0] = reliability;
    p[1] = status;
    memcpy(&p[2], &peakMinEmpty, 4);
    memcpy(&p[6], &peakMaxEmpty, 4);
    memcpy(&p[10], &temperature, 4);
    Disp_SendFrame(DISP_CMD_DIAG, p, DISP_DIAG_LEN);
}

void Disp_SendInfo(void)
{
    uint8_t p[DISP_INFO_LEN];
    p[0] = 1;   /* sensorType：1 = 雷达物位计（占位，待产品定义后扩展） */
    p[1] = 1;   /* 固件主版本 */
    p[2] = 0;   /* 固件次版本 */
    Disp_SendFrame(DISP_CMD_INFO, p, DISP_INFO_LEN);
}

void Disp_BuildEcho(const uint16_t *adc, uint16_t adc_len, uint8_t *out128)
{
    if (adc == 0 || out128 == 0 || adc_len == 0u) return;

    uint16_t maxv = 0;
    for (uint16_t i = 0; i < adc_len; i++)
    {
        if (adc[i] > maxv) maxv = adc[i];
    }
    if (maxv == 0u) maxv = 1u;

    for (uint8_t j = 0; j < DISP_ECHO_LEN; j++)
    {
        uint32_t idx = (uint32_t)j * adc_len / DISP_ECHO_LEN;
        if (idx >= adc_len) idx = adc_len - 1u;
        uint32_t v = (uint32_t)adc[idx] * 255u / maxv;
        out128[j] = (uint8_t)(v > 255u ? 255u : v);
    }
}

/* ---------------- 上行命令接收（预留） ---------------- */
typedef enum { S_SYNC1=0, S_SYNC2, S_CMD, S_LEN, S_PAYLOAD, S_CRC } DispRxState_t;

static DispRxState_t s_state = S_SYNC1;
static uint8_t s_cmd   = 0;
static uint8_t s_len   = 0;
static uint8_t s_idx   = 0;
static uint8_t s_crc   = 0;
static uint8_t s_payload[DISP_ECHO_LEN];
static uint8_t s_rxbyte;   /* L496 BSP 不自动重武装，这里持有接收缓冲 */

/* 上行命令处理：解析出完整且 CRC 正确的上行帧后，把命令与负载交给 Disp_OnUplink() 钩子。
 * 具体动作由用户在 Disp_OnUplink 中按需实现。
 * 参数：
 *   cmd     - 上行命令字（DISP_CMD_REQ_ECHO / REQ_MEAS / KEY / SET_PARAM）
 *   payload - 负载数据指针
 *   len     - 负载字节数 */
static void Disp_HandleUplink(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    /* TODO(用户实现)：处理显示板发来的命令
     *   0x81 REQ_ECHO  -> 触发一次 Disp_SendEcho(...)
     *   0x82 REQ_MEAS  -> 触发一次 Disp_SendMeas(...)
     *   0x83 KEY       -> payload[0]=键码，可转发给雷达业务
     *   0x84 SET_PARAM -> payload[0]=id, payload[1..4]=float value
     */

    Disp_OnUplink(cmd, payload, len);   /* 预留钩子 */
}

/* 逐字节接收回调：HAL 在每次收到 1 字节后调用，驱动状态机解析整帧。
 * 注意：L496 的 BSP 层不在回调内自动重武装接收，因此本函数在末尾手动调用
 *   BSP_USART_Receive_IT() 重新启动下一次单字节接收。
 * 参数：
 *   instance - 触发回调的 USART 实例（恒为 USART1，用于重武装接收）
 *   b        - 本次收到的 1 字节数据 */
void Disp_OnRxByte(BSP_USART_Instance_t instance, uint8_t b)
{
    switch (s_state)
    {
        case S_SYNC1:
            if (b == DISP_SYNC1) s_state = S_SYNC2;
            break;
        case S_SYNC2:
            if (b == DISP_SYNC2)      s_state = S_CMD;
            else if (b == DISP_SYNC1) s_state = S_SYNC2;
            else                      s_state = S_SYNC1;
            break;
        case S_CMD:
            s_cmd = b; s_crc = b; s_state = S_LEN;
            break;
        case S_LEN:
            s_len = b; s_crc ^= b; s_idx = 0;
            if (s_len > sizeof(s_payload)) { s_state = S_SYNC1; break; }
            s_state = (b > 0) ? S_PAYLOAD : S_CRC;
            break;
        case S_PAYLOAD:
            s_payload[s_idx++] = b; s_crc ^= b;
            if (s_idx >= s_len) s_state = S_CRC;
            break;
        case S_CRC:
            if (b == s_crc) Disp_HandleUplink(s_cmd, s_payload, s_len);
            s_state = S_SYNC1;
            break;
        default:
            s_state = S_SYNC1;
            break;
    }
    /* L496 BSP 层不在回调里重武装，这里手动重新启动接收 */
    BSP_USART_Receive_IT(instance, &s_rxbyte, 1);
}

void Disp_OnUplink(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    switch (cmd)
    {
        case DISP_CMD_SET_PARAM:
            /* 负载：param_id(u8) + value(f32) = 5B */
            if (len >= 5U && payload != 0)
            {
                uint8_t id = payload[0];
                float   v;
                memcpy(&v, &payload[1], 4);
                App_Config_SetParam((DISP_PARAM_ID)id, v);
            }
            break;

        case DISP_CMD_SET_STR:
            /* 负载：param_id(u8) + len(u8) + data(len) */
            if (len >= 2U && payload != 0)
            {
                uint8_t id   = payload[0];
                uint8_t slen = payload[1];
                if ((uint16_t)slen + 2U <= (uint16_t)len)
                {
                    char buf[17];
                    uint8_t n = (slen > 16U) ? 16U : slen;
                    memcpy(buf, &payload[2], n);
                    buf[n] = '\0';
                    App_Config_SetStr((DISP_PARAM_ID)id, buf);
                }
            }
            break;

        case DISP_CMD_REQ_INFO:
            Disp_SendInfo();
            break;

        default:
            break;
    }
}

void Disp_Init(void)
{
    BSP_USART_RegisterRxCallback(BSP_USART_INSTANCE_1, Disp_OnRxByte);
    BSP_USART_Receive_IT(BSP_USART_INSTANCE_1, &s_rxbyte, 1);
}
