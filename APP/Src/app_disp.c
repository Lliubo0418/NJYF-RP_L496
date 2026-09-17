/* 主板侧：自定义协议发送（下行）+ 上行命令接收解析（预留）
 *
 * 下行：雷达业务在测距完成后调用 Disp_DownSendMeas() 等把数据发往显示板。
 * 上行：显示板发来的命令（0x81~0x84）经逐字节状态机解析后交给 Disp_OnUp()。
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
#include "bsp_timer.h"    /* adc_buf, ADC_SAMPLE_COUNT_MAX, BSP_Range_GetSampleCount */
#include "bsp_usart.h"
#include "dispproto.h"
#include <string.h>

/* 最近一次回波缓冲，供 REQ_ECHO 立即回送 */
static uint8_t s_last_echo[DISP_ECHO_LEN];
static uint8_t s_last_echo_valid = 0U;
/* REQ_MEAS 请求标志：置 1 后下一次测量完成补发 MEAS */
static volatile uint8_t s_meas_req_pending = 0U;

/* ---------------- 下行发送 ---------------- */

/* 下行帧组装并发送：把 cmd + payload 按 AA 55 帧格式封装后发往显示板。
 * 参数：
 *   cmd     - 下行命令字（DISP_CMD_MEAS / ECHO / DIAG）
 *   payload - 负载数据指针
 *   len     - 负载字节数 */
static void Disp_DownSendFrame(uint8_t cmd, const uint8_t *payload, uint8_t len)
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

void Disp_DownSendMeas(const Algo_RadarResult_t *res, uint8_t mode)
{
    uint8_t p[DISP_MEAS_LEN];
    memcpy(&p[0], &res->distance,  4);
    memcpy(&p[4], &res->position,  4);
    memcpy(&p[8], &res->delta_amp, 4);
    p[12] = (uint8_t)res->peak_count;
    p[13] = mode;
    Disp_DownSendFrame(DISP_CMD_MEAS, p, DISP_MEAS_LEN);
}

void Disp_DownSendEcho(const uint8_t *echo128)
{
    if (echo128 == 0) return;
    memcpy(s_last_echo, echo128, DISP_ECHO_LEN);   /* 缓存最近回波供 REQ_ECHO 回送 */
    s_last_echo_valid = 1U;
    Disp_DownSendFrame(DISP_CMD_ECHO, echo128, DISP_ECHO_LEN);
}

void Disp_DownSendDiag(uint8_t reliability, uint8_t status, float peakMinEmpty, float peakMaxEmpty, float temperature)
{
    uint8_t p[DISP_DIAG_LEN];
    p[0] = reliability;
    p[1] = status;
    memcpy(&p[2], &peakMinEmpty, 4);
    memcpy(&p[6], &peakMaxEmpty, 4);
    memcpy(&p[10], &temperature, 4);
    Disp_DownSendFrame(DISP_CMD_DIAG, p, DISP_DIAG_LEN);
}

void Disp_DownSendInfo(void)
{
    uint8_t p[DISP_INFO_LEN];
    p[0] = 1;   /* sensorType：1 = 雷达物位计（占位，待产品定义后扩展） */
    p[1] = 1;   /* 固件主版本 */
    p[2] = 0;   /* 固件次版本 */
    Disp_DownSendFrame(DISP_CMD_INFO, p, DISP_INFO_LEN);
}

/* 把 gRadarConfig 按 dispproto.h 定义的 81 字节布局序列化并发送。
 * 字段顺序必须与显示板 UI_UpdateParamDump 严格一致。 */
void Disp_DownSendParamDump(void)
{
    uint8_t buf[81];
    App_Config_Serialize(&gRadarConfig, buf);
    Disp_DownSendFrame(DISP_CMD_PARAM_DUMP, buf, 81);
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

/* 环形缓冲：USART ISR 只入队，主循环 Disp_Poll 出队解析，避免 ISR 内阻塞发送影响采样 */
#define DISP_RING_SIZE  256u
static volatile uint8_t s_ring[DISP_RING_SIZE];
static volatile uint16_t s_ring_head = 0u;   /* ISR 写入位置 */
static volatile uint16_t s_ring_tail = 0u;   /* Disp_Poll 读取位置 */

static void Disp_RingPut(uint8_t b)
{
    uint16_t next = (s_ring_head + 1u) % DISP_RING_SIZE;
    if (next != s_ring_tail)   /* 满则丢弃（极端情况，避免阻塞 ISR） */
    {
        s_ring[s_ring_head] = b;
        s_ring_head = next;
    }
}

static uint8_t Disp_RingGet(uint8_t *b)
{
    if (s_ring_head == s_ring_tail) return 0u;
    *b = s_ring[s_ring_tail];
    s_ring_tail = (s_ring_tail + 1u) % DISP_RING_SIZE;
    return 1u;
}

/* 上行命令处理：解析出完整且 CRC 正确的上行帧后，把命令与负载交给 Disp_OnUplink() 钩子。
 * 具体动作由用户在 Disp_OnUplink 中按需实现。
 * 参数：
 *   cmd     - 上行命令字（DISP_CMD_REQ_ECHO / REQ_MEAS / KEY / SET_PARAM）
 *   payload - 负载数据指针
 *   len     - 负载字节数 */
static void Disp_HandleUplink(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    /* TODO(用户实现)：处理显示板发来的命令
     *   0x81 REQ_ECHO  -> 触发一次 Disp_DownSendEcho(...)
     *   0x82 REQ_MEAS  -> 触发一次 Disp_DownSendMeas(...)
     *   0x83 KEY       -> payload[0]=键码，可转发给雷达业务
     *   0x84 SET_PARAM -> payload[0]=id, payload[1..4]=float value
     */

    Disp_OnUplink(cmd, payload, len);   /* 预留钩子 */
}

/* 逐字节接收回调（USART1 ISR 上下文）：仅入环形缓冲，不做帧解析/发送，
 * 保证采样定时器（TIM7，优先级更高）不被串口阻塞。完整帧解析在 Disp_Poll 中完成。 */
void Disp_OnRxByte(BSP_USART_Instance_t instance, uint8_t b)
{
    Disp_RingPut(b);
    /* L496 BSP 层不在回调里重武装，这里手动重新启动接收 */
    BSP_USART_Receive_IT(instance, &s_rxbyte, 1);
}

/* 单字节状态机推进：由 Disp_Poll 在主循环调用。 */
static void Disp_RxProcessByte(uint8_t b)
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
}

void Disp_Poll(void)
{
    uint8_t b;
    /* 一次性消费环形缓冲中所有已到达字节；上行帧解析 + 下行响应发送均在主循环 */
    while (Disp_RingGet(&b))
    {
        Disp_RxProcessByte(b);
    }
}

void Disp_OnUplink(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    switch (cmd)
    {
        case DISP_CMD_REQ_ECHO:
            /* 下行最近一次回波；若尚无缓存则用当前 ADC 包络构建一帧 */
            if (s_last_echo_valid)
            {
                Disp_DownSendFrame(DISP_CMD_ECHO, s_last_echo, DISP_ECHO_LEN);
            }
            else
            {
                uint16_t n = BSP_Range_GetSampleCount();
                if (n == 0U || n > ADC_SAMPLE_COUNT_MAX) n = ADC_SAMPLE_COUNT_MAX;
                uint8_t tmp[DISP_ECHO_LEN];
                Disp_BuildEcho((const uint16_t *)(void *)adc_buf, n, tmp);
                Disp_DownSendEcho(tmp);
            }
            break;

        case DISP_CMD_REQ_MEAS:
            /* 不打断当前采样周期，下一次测量完成后补发一帧 MEAS */
            s_meas_req_pending = 1U;
            break;

        case DISP_CMD_KEY:
            /* 按键转发：payload[0] = 按键编码。
             * 约定按键 0x01（K1）触发一次测量请求；其余按键保留扩展。 */
            if (len >= 1U && payload != 0)
            {
                uint8_t key = payload[0];
                if (key == 0x01U)
                {
                    s_meas_req_pending = 1U;
                }
                /* TODO: 其余按键可在此处映射到具体业务动作 */
            }
            break;

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
            Disp_DownSendInfo();
            break;

        case DISP_CMD_REQ_PARAM_DUMP:
            Disp_DownSendParamDump();
            break;

        default:
            break;
    }
}

/* 供 app_radar.c 查询/清除测量请求标志。 */
uint8_t Disp_ConsumeMeasReq(void)
{
    uint8_t r = s_meas_req_pending;
    s_meas_req_pending = 0U;
    return r;
}

void Disp_Init(void)
{
    BSP_USART_RegisterRxCallback(BSP_USART_INSTANCE_1, Disp_OnRxByte);
    BSP_USART_Receive_IT(BSP_USART_INSTANCE_1, &s_rxbyte, 1);
}
