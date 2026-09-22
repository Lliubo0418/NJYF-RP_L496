#include "app_hart.h"
#include "app_config.h"   /* gRadarConfig.servHART / servHARTAddr（轮询地址来源） */
#include "bsp_gpio.h"     /* BSP_GPIO_HART_SetTransmit / SetReceive */
#include "stm32l4xx_hal.h"
#include <string.h>

extern UART_HandleTypeDef huart2;

/* HART 协议常量 */
#define HART_PREAMBLE       0xFFu
#define HART_DELIM_SLAVE_S  0x06u   /* 短帧从机响应 */
#define HART_DELIM_SLAVE_L  0x86u   /* 长帧从机响应 */
#define HART_UNIT_METER     0x09u   /* HART 单位码：米 */

/* 运行时状态 */
static float   s_pv          = 0.0f;   /* 主变量：物位（米） */
static uint8_t s_poll_addr   = 0U;     /* 轮询地址（命令 6 可改） */
static uint8_t s_tx_armed    = 0U;

/* 接收缓冲 */
#define HART_RX_MAX  64u
static uint8_t s_rxbuf[HART_RX_MAX];
static uint8_t s_rxlen = 0U;

/* 计算 HART 校验和：从 Delimiter 到最后一个数据字节逐字节异或 */
static uint8_t HART_Checksum(const uint8_t *data, uint8_t len)
{
    uint8_t cs = 0U;
    for (uint8_t i = 0; i < len; i++) cs ^= data[i];
    return cs;
}

/* 发送一帧 HART 响应：preamble + delimiter + addr + cmd + bc + data + checksum */
static void HART_SendResponse(uint8_t delimiter, uint8_t addr, uint8_t cmd,
                              const uint8_t *data, uint8_t dlen)
{
    uint8_t frame[48u];
    uint8_t idx = 0U;
    uint8_t i;

    /* 前导码：5 字节 */
    for (i = 0; i < 5U; i++) frame[idx++] = HART_PREAMBLE;

    /* 帧头（参与校验） */
    uint8_t hdr_start = idx;
    frame[idx++] = delimiter;
    frame[idx++] = addr;
    frame[idx++] = cmd;
    frame[idx++] = dlen;
    if (data != 0 && dlen > 0U)
    {
        memcpy(&frame[idx], data, dlen);
        idx += dlen;
    }
    /* 校验和：delimiter..data（先算长度再写入，避免 idx 未排序访问） */
    uint8_t cs_len = (uint8_t)(idx - hdr_start);
    frame[idx] = HART_Checksum(&frame[hdr_start], cs_len);
    idx++;

    BSP_GPIO_HART_SetTransmit();
    HAL_UART_Transmit(&huart2, frame, idx, 100);
    HAL_Delay(2);   /* 等待最后一字节发送完成 */
    BSP_GPIO_HART_SetReceive();
}

/* 解析并响应单条 HART 请求。buf 指向 delimiter（不含前导码）。 */
static void HART_HandleRequest(const uint8_t *buf, uint8_t len)
{
    if (len < 5U) return;   /* delimiter + addr + cmd + bc + checksum */

    uint8_t delim = buf[0];
    uint8_t addr  = buf[1];
    uint8_t cmd   = buf[2];
    uint8_t bc    = buf[3];
    if ((uint16_t)bc + 5U > len) return;   /* 长度不足 */

    /* 校验和校验 */
    uint8_t cs_calc = HART_Checksum(buf, (uint8_t)(4U + bc));
    if (cs_calc != buf[4U + bc]) return;

    /* 地址匹配（短帧：高 6 位为轮询地址） */
    uint8_t recv_poll = (addr >> 2) & 0x3Fu;
    if (recv_poll != s_poll_addr) return;   /* 非本机地址，忽略 */

    uint8_t rsp_addr = (uint8_t)((s_poll_addr << 2) | 0x01U);   /* bit0=1 表示从机响应 */
    uint8_t rsp[32];
    uint8_t rsp_len = 0U;

    switch (cmd)
    {
        case 0:   /* Read Unique Identifier：最小响应 */
            rsp[0] = 0x00;          /* 响应码：OK */
            rsp[1] = 0x00;          /* 设备状态 */
            rsp[2] = 0x01;          /* 制造商 ID */
            rsp[3] = 0x01;          /* 设备类型 */
            rsp[4] = 0x01;          /* 必备请求前导码数 */
            rsp[5] = 0x00;          /* 通用命令版本 */
            rsp[6] = 0x01;          /* 设备规范版本 */
            rsp[7] = 0x01;          /* 软件修正版本 */
            rsp[8] = 0x00;          /* 硬件修正版本 */
            rsp[9] = 0x00;          /* 物理层代码 */
            rsp[10]= 0x00;          /* 选项 */
            rsp_len = 11U;
            break;

        case 1:   /* Read Primary Variable */
            rsp[0] = 0x00;          /* 响应码 OK */
            rsp[1] = 0x00;          /* 设备状态 */
            rsp[2] = HART_UNIT_METER;
            memcpy(&rsp[3], &s_pv, 4);
            rsp_len = 7U;
            break;

        case 2:   /* Read Loop Current and Percentage of Range */
        {
            rsp[0] = 0x00;
            rsp[1] = 0x00;
            /* 回路电流：4~20mA 对应 PV（简化：4 + (PV/量程)*16，量程取 10m） */
            float span = 10.0f;
            float pct  = (s_pv / span) * 100.0f;
            if (pct < 0.0f) pct = 0.0f;
            if (pct > 100.0f) pct = 100.0f;
            float curr = 4.0f + pct * 0.16f;
            memcpy(&rsp[2], &curr, 4);
            memcpy(&rsp[6], &pct, 4);
            rsp_len = 10U;
            break;
        }

        case 3:   /* Read Dynamic Variables and Loop Current */
        {
            rsp[0] = 0x00;
            rsp[1] = 0x00;
            rsp[2] = HART_UNIT_METER;   /* PV 单位 */
            memcpy(&rsp[3], &s_pv, 4);  /* PV */
            rsp[7] = 0x7F;              /* SV 单位：未使用 */
            uint32_t nan = 0x7FC00000U; /* SV=NaN */
            memcpy(&rsp[8], &nan, 4);
            rsp[12]= 0x7F;              /* TV 单位 */
            memcpy(&rsp[13], &nan, 4);  /* TV=NaN */
            rsp[17]= 0x7F;              /* FV 单位 */
            memcpy(&rsp[18], &nan, 4);  /* FV=NaN */
            float curr = 4.0f + (s_pv / 10.0f) * 16.0f;
            if (curr < 4.0f) curr = 4.0f;
            if (curr > 20.0f) curr = 20.0f;
            memcpy(&rsp[22], &curr, 4); /* 回路电流 */
            rsp_len = 26U;
            break;
        }

        case 6:   /* Write Polling Address */
            if (bc >= 1U)
            {
                s_poll_addr = buf[4] & 0x3Fu;
                rsp[0] = 0x00;
                rsp[1] = 0x00;
                rsp[2] = s_poll_addr;
                rsp_len = 3U;
            }
            break;

        default:
            /* 不支持的命令：返回响应码 64（命令未实现） */
            rsp[0] = 64U;
            rsp[1] = 0x00;
            rsp_len = 2U;
            break;
    }

    uint8_t rsp_delim = ((delim & 0x80U) != 0U) ? HART_DELIM_SLAVE_L : HART_DELIM_SLAVE_S;
    HART_SendResponse(rsp_delim, rsp_addr, cmd, rsp, rsp_len);
}

void App_HART_Init(void)
{
    /* 重新配置 USART2 为 HART 参数：1200bps, 8 数据位, 奇校验, 1 停止位 */
    HAL_UART_DeInit(&huart2);
    huart2.Init.BaudRate     = 1200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_ODD;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);

    /* ---- HART 轮询地址：对齐飞卓 p14 §4.6 原文 ----
     * 飞卓原文：
     *   "用  键选择标准或多点工作模式。选择标准工作模式时，本机地址被指定为0；
     *    若选择多点工作模式，按  键，进入地址设置菜单，地址设置为1—15。"
     *
     * 显示板 dict_hart = {"标准","多点"}，索引 0 = 标准、1 = 多点，
     * 与 MENU_HART_MODE 的 min=0/max=1 一致；
     * servHARTAddr 由 Page_HARTAddr 编辑（min=0/max=15），经 SET_PARAM 上报。
     *
     * ★调用时机约束：本函数必须在 App_Config_LoadFromEEPROM() 【之后】执行，
     *   否则读到的是 App_Config_Init() 的默认值（servHART=0 → 恒为标准模式，
     *   用户在菜单里选的多点/地址会被开机流程覆盖掉）。
     *   main.c 中的实际顺序：:131 LoadFromEEPROM → :135 App_HART_Init，
     *   已满足该约束。若日后调整 init 顺序，此处必须同步复核。
     *
     * ★标准模式下【强制 0】：这是协议语义，不是"取用户值再夹取"。
     *   多点模式下才取 servHARTAddr，并夹取到合法区间 1~15
     *   （菜单 min 虽为 0，但多点模式地址 0 非法，会把本机从总线上"藏起来"，
     *     故这里夹到 1；上限 15 同理，防止 EEPROM 残留越界值）。*/
    if (gRadarConfig.servHART == 1U)   /* 1 = 多点模式 */
    {
        uint8_t a = gRadarConfig.servHARTAddr;
        if (a < 1U)  a = 1U;    /* 多点模式地址下限 1（0 非法） */
        if (a > 15U) a = 15U;   /* 上限 15（HART 短帧轮询地址范围） */
        App_HART_SetPollAddr(a);
    }
    else                               /* 0 = 标准模式（含未初始化） */
    {
        App_HART_SetPollAddr(0U);      /* 标准模式：地址强制为 0 */
    }

    BSP_GPIO_HART_SetReceive();   /* 默认接收模式 */
    s_rxlen = 0U;
}

void App_HART_Task(void)
{
    uint8_t b;
    /* 逐字节读取 USART2（非阻塞，单次最多读取一字节） */
    if (HAL_UART_Receive(&huart2, &b, 1, 0) == HAL_OK)
    {
        if (b == HART_PREAMBLE)
        {
            /* 前导码：重置接收状态，但不存入帧缓冲 */
            s_rxlen = 0U;
            return;
        }
        if (s_rxlen < HART_RX_MAX)
        {
            s_rxbuf[s_rxlen++] = b;
        }
    }

    /* 尝试解析：delimiter + addr + cmd + bc + 数据 + checksum */
    if (s_rxlen >= 5U)
    {
        uint8_t bc = s_rxbuf[3];
        uint8_t total = (uint8_t)(4U + bc + 1U);
        if (s_rxlen >= total)
        {
            HART_HandleRequest(s_rxbuf, total);
            s_rxlen = 0U;
        }
    }
}

void App_HART_SetPV(float level_m)
{
    s_pv = level_m;
}

void App_HART_SetPollAddr(uint8_t addr)
{
    s_poll_addr = addr & 0x3Fu;
}
