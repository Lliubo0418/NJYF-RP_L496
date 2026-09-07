#include "app_config.h"
#include "app_algorithm.h"   /* Algo_LearnEmptyTank */
#include <string.h>

/* 主板侧配置唯一实例 */
RadarConfig_t gRadarConfig;

/* 阻尼滤波状态（一阶低通） */
static float s_damp_filtered = 0.0f;
static uint8_t s_damp_init   = 0U;

void App_Config_Init(void)
{
    memset(&gRadarConfig, 0, sizeof(gRadarConfig));

    /* 初值与显示板 RADAR_PARAM 默认一致（见 oled_ui.c UI_Init / 结构体初始化） */
    gRadarConfig.lowAdjustPct   = 0.0f;
    gRadarConfig.lowAdjustVal   = 0.0f;
    gRadarConfig.highAdjustPct  = 100.0f;
    gRadarConfig.highAdjustVal  = 0.0f;
    gRadarConfig.matType        = 0U;
    gRadarConfig.matFastChange  = 0U;
    gRadarConfig.matFirstWave   = 1U;
    gRadarConfig.matSurfAngle   = 0U;
    gRadarConfig.matFoamDust    = 0U;
    gRadarConfig.matSmallDK      = 0U;
    gRadarConfig.matPipe        = 0U;
    gRadarConfig.pipeDiameter   = 100.0f;
    gRadarConfig.dampTime       = 0.0f;
    gRadarConfig.outMap         = 0U;
    gRadarConfig.scaleUnit      = 0U;
    gRadarConfig.scaleVal       = 0.0f;
    gRadarConfig.rangeSetting   = 0.0f;   /* 0 = 不限制上限 */
    gRadarConfig.blindZone      = 0.0f;
    gRadarConfig.currMode       = 0U;
    gRadarConfig.currFault      = 0U;
    gRadarConfig.currMin        = 0U;
    gRadarConfig.servReset      = 0U;
    gRadarConfig.servUnit       = 0U;
    gRadarConfig.servHART       = 0U;
    gRadarConfig.servHARTAddr   = 0U;
    gRadarConfig.servOffset     = 0.0f;
    gRadarConfig.threshEcho     = 0.0f;   /* 0 = 沿用自适应 3σ 阈值 */
    gRadarConfig.threshEnv      = 0.0f;
    gRadarConfig.diagSim        = 0U;
    strncpy(gRadarConfig.sensorTag, "SENSOR", sizeof(gRadarConfig.sensorTag) - 1);
}

void App_Config_SetParam(DISP_PARAM_ID id, float value)
{
    switch (id)
    {
        /* 基本设置 */
        case DPARAM_LOW_ADJ_PCT:    gRadarConfig.lowAdjustPct   = value; break;
        case DPARAM_LOW_ADJ_VAL:    gRadarConfig.lowAdjustVal   = value; break;
        case DPARAM_HIGH_ADJ_PCT:   gRadarConfig.highAdjustPct  = value; break;
        case DPARAM_HIGH_ADJ_VAL:   gRadarConfig.highAdjustVal  = value; break;
        case DPARAM_MAT_TYPE:       gRadarConfig.matType        = (uint8_t)value; break;
        case DPARAM_MAT_FAST_CHANGE:gRadarConfig.matFastChange  = (uint8_t)value; break;
        case DPARAM_MAT_FIRST_WAVE: gRadarConfig.matFirstWave   = (uint8_t)value; break;
        case DPARAM_MAT_SURF_ANGLE: gRadarConfig.matSurfAngle  = (uint8_t)value; break;
        case DPARAM_MAT_FOAM_DUST:  gRadarConfig.matFoamDust    = (uint8_t)value; break;
        case DPARAM_MAT_SMALL_DK:   gRadarConfig.matSmallDK     = (uint8_t)value; break;
        case DPARAM_MAT_PIPE:       gRadarConfig.matPipe        = (uint8_t)value; break;
        case DPARAM_PIPE_DIAMETER:  gRadarConfig.pipeDiameter   = value; break;
        case DPARAM_DAMP_TIME:      gRadarConfig.dampTime       = value; break;
        case DPARAM_OUT_MAP:        gRadarConfig.outMap         = (uint8_t)value; break;
        case DPARAM_SCALE_UNIT:     gRadarConfig.scaleUnit      = (uint8_t)value; break;
        case DPARAM_SCALE_VAL:       gRadarConfig.scaleVal       = value; break;
        case DPARAM_RANGE_SETTING:  gRadarConfig.rangeSetting   = value; break;
        case DPARAM_BLIND_ZONE:     gRadarConfig.blindZone      = value; break;

        /* 服务 / 输出 */
        case DPARAM_CURR_MODE:      gRadarConfig.currMode       = (uint8_t)value; break;
        case DPARAM_CURR_FAULT:     gRadarConfig.currFault      = (uint8_t)value; break;
        case DPARAM_CURR_MIN:       gRadarConfig.currMin        = (uint8_t)value; break;
        case DPARAM_SERV_RESET:
            gRadarConfig.servReset = (uint8_t)value;
            if (value >= 1.0f) App_Config_Reset((uint8_t)value);  /* 非零即触发复位 */
            break;
        case DPARAM_SERV_UNIT:      gRadarConfig.servUnit       = (uint8_t)value; break;
        case DPARAM_SERV_HART:      gRadarConfig.servHART       = (uint8_t)value; break;
        case DPARAM_SERV_HART_ADDR: gRadarConfig.servHARTAddr   = (uint8_t)value; break;
        case DPARAM_SERV_OFFSET:    gRadarConfig.servOffset     = value; break;
        case DPARAM_THRESH_ECHO:    gRadarConfig.threshEcho     = value; break;
        case DPARAM_THRESH_ENV:     gRadarConfig.threshEnv      = value; break;

        /* 仿真 */
        case DPARAM_DIAG_SIM:       gRadarConfig.diagSim        = (uint8_t)value; break;
        case DPARAM_SERV_FALSE_ECHO: App_Config_LearnFalseEcho(); break;  /* 触发虚假回波学习 */

        /* 字符串型不应走这里，忽略 */
        case DPARAM_SENSOR_TAG:
        default:
            break;
    }
}

void App_Config_SetStr(DISP_PARAM_ID id, const char *str)
{
    if (str == 0) return;
    switch (id)
    {
        case DPARAM_SENSOR_TAG:
            strncpy(gRadarConfig.sensorTag, str, sizeof(gRadarConfig.sensorTag) - 1);
            gRadarConfig.sensorTag[sizeof(gRadarConfig.sensorTag) - 1] = '\0';
            break;
        default:
            break;
    }
}

uint8_t App_Config_Reset(uint8_t mode)
{
    /* 复位全部：重新学习空罐基线（当前采样若未完成则等待下一轮采样时由算法自动补学） */
    if (mode == 1U)
    {
        App_Config_LearnFalseEcho();
        s_damp_init = 0U;   /* 清空阻尼历史，避免旧值平滑拖尾 */
    }
    /* mode == 2（仅累计流量）为显示板本地量，主板无需动作 */
    return 0U;
}

void App_Config_LearnFalseEcho(void)
{
    /* 内部已做“采样是否完成”的保护；未完成时本次为 no-op，
     * 可由显示板在采样稳定后再次下发复位触发。 */
    Algo_LearnEmptyTank();
}

float App_Config_ApplyRange(float dist)
{
    /* 下限：盲区 */
    if (gRadarConfig.blindZone > 0.0f && dist < gRadarConfig.blindZone)
        dist = gRadarConfig.blindZone;

    /* 上限：量程（rangeSetting<=0 表示不限制） */
    if (gRadarConfig.rangeSetting > 0.0f && dist > gRadarConfig.rangeSetting)
        dist = gRadarConfig.rangeSetting;

    return dist;
}

float App_Config_ApplyDamping(float newVal)
{
    if (gRadarConfig.dampTime <= 0.0f)
    {
        s_damp_filtered = newVal;
        s_damp_init     = 1U;
        return newVal;
    }

    /* 一阶低通：alpha = 1/(1 + dampTime*K)，dampTime 越大平滑越强 */
    float alpha = 1.0f / (1.0f + gRadarConfig.dampTime * 5.0f);
    if (alpha > 1.0f) alpha = 1.0f;
    if (alpha < 0.01f) alpha = 0.01f;

    if (!s_damp_init)
    {
        s_damp_filtered = newVal;
        s_damp_init     = 1U;
    }
    else
    {
        s_damp_filtered = s_damp_filtered + alpha * (newVal - s_damp_filtered);
    }
    return s_damp_filtered;
}

uint8_t App_Config_IsSim(void)
{
    return gRadarConfig.diagSim ? 1U : 0U;
}

float App_Config_SimDistance(void)
{
    /* 占位：固定输出 2.0 m。待接入真实仿真模型（按量程/低位高位插值）后替换。 */
    return 2.0f;
}

float App_Config_GetSensorTemp(void)
{
    /* 占位：返回 25.0℃。待接入 NTC / 内部温度传感器后替换。 */
    return 25.0f;
}
