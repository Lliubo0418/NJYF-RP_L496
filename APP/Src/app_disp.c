/* 主板侧：自定义协议下行发送 + 上行命令接收解析。
 *
 * 下行：雷达业务在测距完成后调用 Disp_DownSendMeas() 等把数据发往显示板。
 * 上行：显示板发来的命令（0x81~0x87）经环形缓冲由 Disp_Poll 出队解析，
 *       完整帧交给 Disp_OnUplink() 执行具体业务。
 *
 * 接收方式：USART1 使用 DMA + 空闲中断接收（根治 ORE）。
 *           DMA 硬件自动搬字节到缓冲区，不受 TIM7 优先级抢占影响；
 *           空闲事件回调批量投递到环形缓冲并自动重启 DMA，本文件无需手动重武装。
 *
 * 设计原则：本文件只做“传输 + 帧解析 + 字段打包”，测距触发频率在 bsp_timer/app_radar，
 *       回波降采样算法见 Disp_BuildEcho，参数业务动作在 app_config。
 */
#include "app_disp.h"
#include "app_algorithm.h"
#include "app_config.h"
#include "bsp_timer.h"    /* adc_buf, ADC_SAMPLE_COUNT_MAX, BSP_Range_GetSampleCount */
#include "bsp_usart.h"
#include "dispproto.h"
#include <stdio.h>
#include <string.h>

/* 最近一次回波缓冲，供 REQ_ECHO 立即回送 */
static uint8_t s_last_echo[DISP_ECHO_LEN];
static uint8_t s_last_echo_valid = 0U;
/* REQ_MEAS 请求标志：置 1 后下一次测量完成补发 MEAS */
static volatile uint8_t s_meas_req_pending = 0U;

/* 接收诊断：DMA 累计投递字节数（ISR 写），用于判定 RX 硬件链路是否畅通 */
volatile uint32_t s_diag_rx_bytes = 0U;

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

void Disp_DownSendEchoTyped(uint8_t type, const uint8_t *data128)
{
    uint8_t p[DISP_ECHO_TYPED_LEN];
    if (data128 == 0) return;
    p[0] = type;
    memcpy(&p[1], data128, DISP_ECHO_LEN);
    Disp_DownSendFrame(DISP_CMD_ECHO_TYPED, p, DISP_ECHO_TYPED_LEN);
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

/* 曲线归一化。
 * ★哨兵剔除（§3.10 #C3）：源数组可能含 bsp_timer.c:585 写入的 0xFFFF 失败标记。
 *   若不剔除，会造成两个可见故障：
 *     ① 自归一化时 maxv 被抬到 65535 → 所有点 ×255/65535 都被压到 0 附近
 *        → 整条曲线在屏上变成【一条贴底平线】（用户表现为"曲线没了/一片空白"）；
 *     ② 该点自身也被当成长度 65535 纳秒级最大峰，曲线形状失真。
 *   这里在取值处把哨兵按 0 处理：既不影响正常点，也不让失败点主导标度。*/
uint16_t Disp_BuildCurveNorm(const uint16_t *src, uint16_t src_len,
                             uint16_t norm_peak, uint8_t *out128)
{
    if (src == 0 || out128 == 0 || src_len == 0u) return 0u;

    /* 归一化除数：调用方给了就用它（曲线间幅度可比）；没给则退回自身峰值 */
    uint16_t maxv = norm_peak;
    if (maxv == 0u)
    {
        for (uint16_t i = 0; i < src_len; i++)
        {
            uint16_t v = src[i];
            if (v == 0xFFFFu) continue;   /* 跳过失败哨兵，不让它决定标度 */
            if (v > maxv) maxv = v;
        }
    }
    if (maxv == 0u) maxv = 1u;

    for (uint8_t j = 0; j < DISP_ECHO_LEN; j++)
    {
        uint32_t idx = (uint32_t)j * src_len / DISP_ECHO_LEN;
        if (idx >= src_len) idx = src_len - 1u;
        uint16_t s = src[idx];
        if (s == 0xFFFFu) s = 0u;        /* 失败点按 0 画，避免假尖峰 */
        uint32_t v = (uint32_t)s * 255u / maxv;
        out128[j] = (uint8_t)(v > 255u ? 255u : v);
    }
    return maxv;
}

uint16_t Disp_BuildEchoEx(const uint16_t *adc, uint16_t adc_len, uint8_t *out128)
{
    /* 传 norm_peak=0 → 用自身峰值归一化，行为与原 Disp_BuildEcho 逐字节一致 */
    return Disp_BuildCurveNorm(adc, adc_len, 0u, out128);
}

void Disp_BuildEcho(const uint16_t *adc, uint16_t adc_len, uint8_t *out128)
{
    (void)Disp_BuildEchoEx(adc, adc_len, out128);
}

/* ---------------- 上行命令接收（显示板 -> 主板） ---------------- */
typedef enum { S_SYNC1=0, S_SYNC2, S_CMD, S_LEN, S_PAYLOAD, S_CRC } DispRxState_t;

static DispRxState_t s_state = S_SYNC1;
static uint8_t s_cmd   = 0;
static uint8_t s_len   = 0;
static uint8_t s_idx   = 0;
static uint8_t s_crc   = 0;
static uint8_t s_payload[DISP_ECHO_LEN];

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

/* 上行命令分发：解析出完整且 CRC 正确的上行帧后，交给 Disp_OnUplink() 执行业务。
 * 参数：
 *   cmd     - 上行命令字（0x81~0x87，见 dispproto.h）
 *   payload - 负载数据指针
 *   len     - 负载字节数 */
static void Disp_HandleUplink(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    Disp_OnUplink(cmd, payload, len);
}

/* 批量接收回调（DMA 空闲事件 ISR 上下文）：把 DMA 缓冲中的字节逐个入环形缓冲。
 * DMA 自动重启在 BSP 层 HAL_UARTEx_RxEventCallback 完成，本函数无需手动重武装。
 * 保证采样定时器（TIM7，优先级更高）不被串口阻塞。完整帧解析在 Disp_Poll 中完成。 */
void Disp_OnRxByte(BSP_USART_Instance_t instance, uint8_t b)
{
    s_diag_rx_bytes++;
    Disp_RingPut(b);
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

    /* 诊断：每 1s 上报 DMA 累计接收字节数，判定 RX 硬件链路：
     *   数字增长 → DMA 在收；一直为 0 → DMA/CSELR/接线问题 */
    {
        static uint32_t s_last_diag_ms = 0U;
        uint32_t now = HAL_GetTick();
        if ((now - s_last_diag_ms) >= 1000U)
        {
            s_last_diag_ms = now;
            printf("[DISP] rx_bytes=%lu\r\n", (unsigned long)s_diag_rx_bytes);
        }
    }
}

void Disp_OnUplink(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    /* 本函数运行在主循环上下文（Disp_Poll 同步调用），printf 安全 */
    printf("[DISP] uplink cmd=0x%02X len=%u\r\n", cmd, (unsigned)len);
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
    /* DMA + 空闲中断接收：硬件自动搬字节，不受 TIM7 优先级抢占影响，根治 ORE */
    BSP_USART_ReceiveToIdle_DMA(BSP_USART_INSTANCE_1);
}
