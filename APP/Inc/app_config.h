#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#include <stdint.h>
#include "dispproto.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============ 主板（L496）运行配置层 ============
 * 显示板经 SET_PARAM / SET_STR 上行写入 gRadarConfig；测距主循环（app_radar.c）
 * 每轮测量后调用 ApplyRange / ApplyDamping / IsSim 消费配置。
 * 字段与 oled_ui.h 的 RADAR_PARAM、dispproto.h 的 DISP_PARAM_ID 严格对齐。 */

typedef struct
{
    /* 1. 基本设置 */
    float lowAdjustPct;
    float lowAdjustVal;
    float highAdjustPct;
    float highAdjustVal;
    uint8_t matType;
    uint8_t matFastChange;
    uint8_t matFirstWave;
    uint8_t matSurfAngle;
    uint8_t matFoamDust;
    uint8_t matSmallDK;
    uint8_t matPipe;
    float pipeDiameter;
    float dampTime;
    uint8_t outMap;
    uint8_t scaleUnit;
    float scaleVal;
    float rangeSetting;
    float blindZone;
    /* 2. 服务 / 输出 */
    uint8_t currMode;
    uint8_t currFault;
    uint8_t currMin;
    uint8_t servReset;
    uint8_t servUnit;
    uint8_t servHART;
    uint8_t servHARTAddr;
    float servOffset;
    float threshEcho;
    float threshEnv;
    /* 3. 仿真 / 标识 */
    uint8_t diagSim;
    char sensorTag[16];
} RadarConfig_t;

/* 全局配置实例（主板侧唯一拷贝） */
extern RadarConfig_t gRadarConfig;

/* 初始化：填入与显示板默认一致的初值 */
void App_Config_Init(void);

/* 显示板上行写入数值型参数（float 承载，uint8 字段按值强转） */
/* 由显示板上行 SET_PARAM 写入单个浮点/整型参数，并立即落盘 EEPROM。
 * 返回值（供 ACK 0x07 回给显示板，取值见 dispproto.h 的 DISP_ACK_*）：
 *   0 = DISP_ACK_OK         已写入 gRadarConfig 且 EEPROM 落盘成功
 *   1 = DISP_ACK_EEPROM_ERR RAM 已更新但 EEPROM 写失败（断电即丢，必须让用户知道）
 *   2 = DISP_ACK_BAD_ID     id 无对应分支，未做任何动作
 * ⚠ 返回 0 才代表"这次调整真的持久化了"；调用方（app_disp.c）据此回 ACK。 */
uint8_t App_Config_SetParam(DISP_PARAM_ID id, float value);

/* 显示板上行写入字符串型参数（当前仅 sensorTag，对应 DPARAM_SENSOR_TAG）。
 * 返回值语义同 App_Config_SetParam。 */
uint8_t App_Config_SetStr(DISP_PARAM_ID id, const char *str);

/* 复位动作（语义对齐飞卓 §4.3 + 显示板 dict_reset）：
 *   mode=1 基本复位（仅"基本设置"页参数恢复工厂缺省）
 *   mode=2 工厂设置（全部参数恢复工厂缺省）
 *   mode=3 测量峰值（主板无该状态可清，实为 no-op）
 *   其他值 不动作
 * 返回 0=已执行。
 * ⚠ 复位【只做复位】，绝不触发空罐基线学习——学习归 App_Config_LearnFalseEcho()。*/
uint8_t App_Config_Reset(uint8_t mode);

/* 虚假回波学习（空罐基线）。
 * 由 DPARAM_SERV_FALSE_ECHO 的"更新/新建"档触发；"删除"档走 Algo_ClearBaseline()。*/
void App_Config_LearnFalseEcho(void);

/* 量程 + 盲区限幅：
 *   - 上限夹取到「有效量程」= App_Config_GetEffectiveRange()
 *   - 下限夹取到 blindZone（低于盲区视为无效，强制到盲区边界）
 *   仅做数值夹取，不改变“测量成功/失败”语义。 */
float App_Config_ApplyRange(float dist);

/* 有效量程（米）—— 量程语义的【单一真值来源】（2026-10-22 修 R-06）。
 *   rangeSetting > 0 且 ≤ 77m  ⇒ 返回 rangeSetting；
 *   否则（0 / 负数 / NaN / 超上限）⇒ 返回【当前采样档位的物理量程】
 *     （档 1 = 15.39m … 档 5 = 76.97m），与 RangeToLevel 的"0 → 档 1"自洽。
 * ⚠ 不可再写成"rangeSetting<=0 表示不限制"：采样窗本身就有上限，声称不限制会让
 *   窗外读数被当作有效值上报。 */
float App_Config_GetEffectiveRange(void);

/* 阻尼一阶低通：dampTime 越大越平滑；dampTime<=0 时直通。 */
float App_Config_ApplyDamping(float newVal);

/* 仿真模式是否开启（由 DPARAM_DIAG_SIM 控制） */
uint8_t App_Config_IsSim(void);

/* 仿真距离（占位：固定输出 2.0 m，待接真实仿真模型后替换） */
float App_Config_SimDistance(void);

/* 传感器温度：读取 STLM75M2F 实测温度；读取失败时兜底返回 25.0℃ */
float App_Config_GetSensorTemp(void);

/* 把 gRadarConfig 写入外部 EEPROM（24LC256）。
 * 格式：magic(2B='A5 5A') + version(1B=1) + crc16(2B, 小端) + payload(81B)
 * 返回 0=成功，非 0=失败。不自动调用，由上层在合适时机触发。 */
uint8_t App_Config_SaveToEEPROM(void);

/* 从 EEPROM 读取并校验后写回 gRadarConfig。
 * 返回 0=成功（magic/version/CRC 均通过），非 0=失败（保持 gRadarConfig 不变）。 */
uint8_t App_Config_LoadFromEEPROM(void);

/* ============ 空罐基线持久化（24LC256 EEPROM）============
 * 基线数据量较大（最大 5120×2B = 10KB），与配置区分开存储。
 * 存储格式（起始地址 CFG_EEPROM_BASELINE_ADDR）：
 *   Magic(2B='BA5E') + version(1B=3) + count(2B, 小端) + crc16(2B, 小端) + data(10240B, 小端 uint16)
 *   共 10247 字节。CRC 计算范围 = count(2B) + data(10240B)。
 * 档位校验：加载时若 count 与当前量程档位数不符，视为无效基线、按「无基线」处理
 *   （测量 SUPPRESSED，须人工进菜单学）。
 * ⚠ version = 2（2026-10-09 修 A1 时自增）：adc_buf[0] 的含义从「S4↑ 后第一个点」改为
 *   「测距零点附近」，下标整体前移约 968 点 ⇒ version 1 的旧基线一律作废重学。
 * ⚠ version = 3（2026-10-10 K 翻倍 + 2026-10-14 零点捕获触发）：每点距离 30.068 → 15.034mm、
 *   最大点数 3072 → 5120（基线 6144 → 10240B），且 adc_buf[0] 再改为「真零点后的第一拍」
 *   ⇒ version 2 的旧基线一律作废重学。
 */

/* 把当前算法层基线写入 EEPROM。
 * 返回 0=成功，非 0=失败（I2C 错误或无基线可存）。 */
uint8_t App_Config_SaveBaselineToEEPROM(void);

/* 从 EEPROM 读取基线并写入算法层（Algo_SetBaseline）。
 * 返回 0=成功（magic/version/CRC/档位 均通过），非 0=失败（算法层基线保持不变）。
 * 档位不匹配时返回 1，调用方可据此触发重学。 */
uint8_t App_Config_LoadBaselineFromEEPROM(void);

/* 擦除 EEPROM 中的基线（写掉 Magic 字段）。
 * 与 Algo_ClearBaseline() 配对使用：删除虚假回波时同时清 RAM 和 EEPROM。 */
uint8_t App_Config_ClearBaselineInEEPROM(void);

/* 按 dispproto.h PARAM_DUMP 布局序列化 cfg 到 buf（81 字节）。
 * 供 app_disp.c 的 PARAM_DUMP 下行与 EEPROM 持久化复用，保证双板布局单一来源。 */
void App_Config_Serialize(const RadarConfig_t *cfg, uint8_t *buf);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CONFIG_H */
