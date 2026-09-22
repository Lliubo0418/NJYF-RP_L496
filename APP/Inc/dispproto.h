#ifndef __DISP_PROTO_H
#define __DISP_PROTO_H

#include <stdint.h>

/* ============================================================
 * 双板自定义通信协议（雷达主板 STM32L496  <->  显示板 STM32F1）
 * 物理链路：USART1，115200 / 8N1
 *
 * 帧格式（小端字节序，STM32 原生）：
 *   [SYNC1][SYNC2][CMD][LEN][  PAYLOAD (LEN 字节)  ][CRC]
 *    0xAA   0x55  1B   1B          LEN 字节         1B
 *   CRC = CMD ^ LEN ^ (PAYLOAD 逐字节异或)
 *
 * 命令字：
 *   下行（主板 -> 显示板）
 *     0x01 MEAS  测量结果：distance(f32) position(f32) delta_amp(f32) peak_count(u8) mode(u8) = 14B
 *     0x02 ECHO  回波包络：128×u8（已归一化 0~255）                                        = 128B
 *     0x03 DIAG  诊断：reliability(u8) status(u8) peakMinEmpty(f32) peakMaxEmpty(f32) temperature(f32) = 14B
 *     0x04 INFO  传感器信息：sensorType(u8) verMajor(u8) verMinor(u8)                        = 3B
 *     0x05 PARAM_DUMP 全量配置：gRadarConfig 序列化（见 DISP_PARAM_DUMP_LEN）              = 变长
 *     0x06 ECHO_TYPED 带类型标识的曲线帧：curveType(u8) + 128×u8                              = 129B
 *   上行（显示板 -> 主板）
 *     0x81 REQ_ECHO   请求回波帧（无 payload）
 *     0x82 REQ_MEAS   请求测量帧（无 payload）
 *     0x83 KEY        按键转发：key(u8)                                                      = 1B
 *     0x84 SET_PARAM  设置参数：param_id(u8) value(f32)                                      = 5B
 *     0x85 SET_STR    设置字符串参数：param_id(u8) len(u8) data(len)                         = 2B+len
 *     0x86 REQ_INFO   请求传感器信息（无 payload）
 *     0x87 REQ_PARAM_DUMP 请求全量配置（无 payload）
 *
 * 说明：字段映射 / 回波降采样算法等“业务细节”为预留项，由用户在对应
 *       APP 文件中按需填充；本文件只规定帧结构与校验。
 * ============================================================ */

#define DISP_SYNC1         0xAA
#define DISP_SYNC2         0x55

/* 下行命令 */
#define DISP_CMD_MEAS      0x01
#define DISP_CMD_ECHO      0x02
#define DISP_CMD_DIAG      0x03
#define DISP_CMD_INFO      0x04   /* 传感器信息（主板 -> 显示板） */
#define DISP_CMD_PARAM_DUMP 0x05  /* 全量配置（主板 -> 显示板） */
#define DISP_CMD_ECHO_TYPED  0x06   /* 带类型标识的曲线帧（主板 -> 显示板）*/

/* 
 * 曲线类型（DISP_CMD_ECHO_TYPED 的 payload[0]）
 *
 * 【为什么要新开 0x06，而不给 0x02 ECHO 加类型字节】
 *   0x02 的载荷是 128 字节纯数据、无类型字段，两板各自只有一块
 *   radar_echo[128] 缓冲。三种曲线（回波/虚假回波/输出走势）如果
 *   共用同一个命令字，接收端无法分辨这 128 字节属于哪一条，
 *   会互相覆盖 —— 这正是"曲线标题与内容不符"的根因。
 *
 *   但也不能直接把类型字节塞进 0x02 的 payload 头：那是【破坏性
 *   变更】。两板固件版本不同步时（现场单独升级一块板），旧接收端
 *   会把类型字节当成第一个数据点，整条曲线右移一格 —— 表现为
 *   "看起来有曲线但全是错的"，比明显报错更难排查。
 *   新开命令字则天然向后兼容：旧接收端收到 0x06 直接走 default 丢弃。
 */
 
#define DISP_CURVE_ECHO      0x01   /* 回波曲线：adc_buf 原始包络 */
#define DISP_CURVE_FALSE     0x02   /* 虚假回波曲线：空罐学习的基线 */
/* 0x03 预留给"输出走势曲线"，落地时在此登记 */

/* 上行命令 */
#define DISP_CMD_REQ_ECHO  0x81
#define DISP_CMD_REQ_MEAS  0x82
#define DISP_CMD_KEY       0x83
#define DISP_CMD_SET_PARAM 0x84   /* 设置参数：param_id(u8) + value(f32) = 5B */
#define DISP_CMD_SET_STR   0x85   /* 设置字符串参数：param_id(u8) + len(u8) + data = 2B+len */
#define DISP_CMD_REQ_INFO  0x86   /* 请求传感器信息（无 payload） */
#define DISP_CMD_REQ_PARAM_DUMP 0x87  /* 请求全量配置（无 payload） */

/* 固定长度 */
#define DISP_ECHO_LEN      128u
#define DISP_MEAS_LEN      14u
#define DISP_DIAG_LEN      14u   /* reliability(u8) status(u8) peakMinEmpty(f32) peakMaxEmpty(f32) temperature(f32) */
#define DISP_INFO_LEN      3u    /* sensorType(u8) verMajor(u8) verMinor(u8) */
#define DISP_ECHO_TYPED_LEN 129u /* curveType(u8) + 128×u8 */
/* PARAM_DUMP(0x05) 负载为 gRadarConfig 字段按以下固定顺序逐字段 memcpy 序列化
 * （避免结构体 padding 差异，双板必须严格按此顺序读写，共 81 字节）：
 *   [0..3]   lowAdjustPct   (f32)
 *   [4..7]   lowAdjustVal   (f32)
 *   [8..11]  highAdjustPct  (f32)
 *   [12..15] highAdjustVal  (f32)
 *   [16]     matType        (u8)
 *   [17]     matFastChange  (u8)
 *   [18]     matFirstWave   (u8)
 *   [19]     matSurfAngle   (u8)
 *   [20]     matFoamDust    (u8)
 *   [21]     matSmallDK     (u8)
 *   [22]     matPipe        (u8)
 *   [23..26] pipeDiameter   (f32)
 *   [27..30] dampTime       (f32)
 *   [31]     outMap         (u8)
 *   [32]     scaleUnit      (u8)
 *   [33..36] scaleVal       (f32)
 *   [37..40] rangeSetting   (f32)
 *   [41..44] blindZone      (f32)
 *   [45]     currMode       (u8)
 *   [46]     currFault      (u8)
 *   [47]     currMin        (u8)
 *   [48]     servReset      (u8)
 *   [49]     servUnit       (u8)
 *   [50]     servHART       (u8)
 *   [51]     servHARTAddr   (u8)
 *   [52..55] servOffset     (f32)
 *   [56..59] threshEcho     (f32)
 *   [60..63] threshEnv      (f32)
 *   [64]     diagSim        (u8)
 *   [65..80] sensorTag      (char[16])
 */

/* 协议帧最大长度（含头尾），用于本地缓冲 */
#define DISP_FRAME_MAX     (2u + 1u + 1u + 255u + 1u)   /* 260 */

/* 单帧 payload 的最大字节数（受 LEN 字段为 u8 限制）。
 * 接收侧缓冲按此尺寸开辟，新增命令时不必再回头数最长帧。
 * ★注意：这是【协议允许】的上限，不是当前用到的长度。
 *   当前最长的是 0x06 ECHO_TYPED = 129。 */
#define DISP_PAYLOAD_MAX   255u

/* CRC 校验值计算：对 CMD、LEN 及 PAYLOAD 逐字节异或。
 * 发送端把返回值填入帧尾，接收端用同样算法比对以判定帧是否正确。
 * 参数：
 *   cmd     - 命令字（参与异或）
 *   payload - 负载数据指针（可为 NULL，此时只异或 CMD^LEN）
 *   len     - 负载字节数
 * 返回：1 字节校验值 */
static inline uint8_t Disp_CRC(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    uint8_t crc = cmd;
    if (payload != 0 && len > 0u)
    {
        crc ^= len;
        for (uint8_t i = 0u; i < len; i++)
        {
            crc ^= payload[i];
        }
    }
    else
    {
        crc ^= len;   /* len == 0 时仅 CMD ^ LEN(=0) */
    }
    return crc;
}

/* ============================================================
 * 参数编号（SET_PARAM / SET_STR 的 param_id 字段）
 * 两端（主板 L496 / 显示板 F1）必须保持完全一致。
 * 与显示板 oled_ui.h 中 RADAR_PARAM 字段一一对应。
 * ============================================================ */
typedef enum
{
    DPARAM_LOW_ADJ_PCT = 0,   /* 低位调整(%) */
    DPARAM_LOW_ADJ_VAL,       /* 低位调整(距离) */
    DPARAM_HIGH_ADJ_PCT,      /* 高位调整(%) */
    DPARAM_HIGH_ADJ_VAL,      /* 高位调整(距离) */
    DPARAM_MAT_TYPE,          /* 物料类型 */
    DPARAM_MAT_FAST_CHANGE,   /* 物料快速变化 */
    DPARAM_MAT_FIRST_WAVE,    /* 首波选择 */
    DPARAM_MAT_SURF_ANGLE,    /* 表面波动/堆角 */
    DPARAM_MAT_FOAM_DUST,     /* 泡沫/粉尘 */
    DPARAM_MAT_SMALL_DK,      /* DK值小 */
    DPARAM_MAT_PIPE,          /* 导波管测量 */
    DPARAM_PIPE_DIAMETER,     /* 导波管直径 */
    DPARAM_DAMP_TIME,         /* 阻尼时间 */
    DPARAM_OUT_MAP,           /* 输出映射 */
    DPARAM_SCALE_UNIT,        /* 定标单位 */
    DPARAM_SCALE_VAL,         /* 定标值 */
    DPARAM_RANGE_SETTING,     /* 量程设定 */
    DPARAM_BLIND_ZONE,        /* 盲区范围 */
    DPARAM_CURR_MODE,         /* 电流输出模式 */
    DPARAM_CURR_FAULT,        /* 故障电流 */
    DPARAM_CURR_MIN,          /* 最小电流 */
    DPARAM_SERV_RESET,        /* 复位 */
    DPARAM_SERV_UNIT,         /* 测量单位 */
    DPARAM_SERV_HART,         /* HART工作模式 */
    DPARAM_SERV_HART_ADDR,    /* HART地址 */
    DPARAM_SERV_OFFSET,       /* 距离偏量 */
    DPARAM_THRESH_ECHO,       /* 回波阈值 */
    DPARAM_THRESH_ENV,        /* 包络阈值 */
    DPARAM_DIAG_SIM,          /* 仿真 */
    DPARAM_SENSOR_TAG,        /* 传感器标签(字符串，用 SET_STR) */
    DPARAM_SERV_FALSE_ECHO,   /* 虚假回波学习（触发空罐基线重学习） */
    DPARAM_COUNT
} DISP_PARAM_ID;

#endif /* __DISP_PROTO_H */
