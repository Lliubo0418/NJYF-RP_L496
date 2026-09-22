#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#include <stdint.h>
#include "dispproto.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 主板（L496）运行配置层
 *
 * 显示板（F1）上所有"需要发往主板"的参数，统一经 SET_PARAM / SET_STR
 * 上行写入本模块的 gRadarConfig。测距主循环（app_radar.c）每轮测量后
 * 调用 ApplyRange / ApplyDamping / IsSim 等接口消费这些配置。
 *
 * 字段与显示板 oled_ui.h 的 RADAR_PARAM 中“需发 L496”部分一一对应；
 * 与 dispproto.h 的 DISP_PARAM_ID 枚举严格对齐。
 * ============================================================ */

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

/* 显示板上行写入单个数值型参数（float 承载；uint8 字段按值强转） */
void App_Config_SetParam(DISP_PARAM_ID id, float value);

/* 显示板上行写入字符串型参数（当前仅 sensorTag，对应 DPARAM_SENSOR_TAG） */
void App_Config_SetStr(DISP_PARAM_ID id, const char *str);

/* 复位动作（语义对齐飞卓 §4.3 + 显示板 dict_reset 的下发值）：
 *   mode = 1 → 基本复位：仅"基本设置"页参数恢复工厂缺省
 *   mode = 2 → 工厂设置：全部参数恢复工厂缺省
 *   mode = 3 → 测量峰值：主板无该状态可清，实为 no-op
 *   其他值   → 不动作
 * 返回 0=已执行。
 * ⚠ 复位【只做复位】，绝不触发空罐基线学习——学习归 App_Config_LearnFalseEcho()。
 *   （历史缺陷：曾把 mode==1 实现成"学习空罐基线"，造成带料状态下误学习、
 *     物料回波被永久扣除并写入 EEPROM，导致持续性误测。）*/
uint8_t App_Config_Reset(uint8_t mode);

/* 虚假回波学习（空罐基线）。
 * 由 DPARAM_SERV_FALSE_ECHO 的"更新/新建"档触发；"删除"档走 Algo_ClearBaseline()。*/
void App_Config_LearnFalseEcho(void);

/* 量程 + 盲区限幅：
 *   - rangeSetting>0 时上限夹取到 rangeSetting
 *   - 下限夹取到 blindZone（低于盲区视为无效，强制到盲区边界）
 *   仅做数值夹取，不改变“测量成功/失败”语义。 */
float App_Config_ApplyRange(float dist);

/* 阻尼一阶低通：dampTime 越大越平滑；dampTime<=0 时直通。 */
float App_Config_ApplyDamping(float newVal);

/* 仿真模式是否开启（由 DPARAM_DIAG_SIM 控制） */
uint8_t App_Config_IsSim(void);

/* 仿真距离（占位：固定输出 2.0 m，待接真实仿真模型后替换） */
float App_Config_SimDistance(void);

/* 传感器温度（占位：返回 25.0℃，待接 NTC/内部温度传感器后替换） */
float App_Config_GetSensorTemp(void);

/* 把 gRadarConfig 写入外部 EEPROM（24LC256）。
 * 格式：magic(2B='A5 5A') + version(1B=1) + crc16(2B, 小端) + payload(81B)
 * 返回 0=成功，非 0=失败。不自动调用，由上层在合适时机触发。 */
uint8_t App_Config_SaveToEEPROM(void);

/* 从 EEPROM 读取并校验后写回 gRadarConfig。
 * 返回 0=成功（magic/version/CRC 均通过），非 0=失败（保持 gRadarConfig 不变）。 */
uint8_t App_Config_LoadFromEEPROM(void);

/* 按 dispproto.h PARAM_DUMP 布局序列化 cfg 到 buf（81 字节）。
 * 供 app_disp.c 的 PARAM_DUMP 下行与 EEPROM 持久化复用，保证双板布局单一来源。 */
void App_Config_Serialize(const RadarConfig_t *cfg, uint8_t *buf);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CONFIG_H */
