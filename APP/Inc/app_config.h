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
 * 显示板（F1）上所有“需要下发到主板”的参数，统一经 SET_PARAM / SET_STR
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

/* 显示板下行写入单个数值型参数（float 承载；uint8 字段按值强转） */
void App_Config_SetParam(DISP_PARAM_ID id, float value);

/* 显示板下行写入字符串型参数（当前仅 sensorTag，对应 DPARAM_SENSOR_TAG） */
void App_Config_SetStr(DISP_PARAM_ID id, const char *str);

/* 复位动作：
 *   mode 与显示板复位选项对齐 —— 0=取消, 1=全部复位(重新学习空罐基线), 2=仅累计流量
 *   返回 0=已执行。累计流量为显示板本地量，此处仅处理空罐基线重学习。 */
uint8_t App_Config_Reset(uint8_t mode);

/* 虚假回波学习（空罐基线） */
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

#ifdef __cplusplus
}
#endif

#endif /* __APP_CONFIG_H */
