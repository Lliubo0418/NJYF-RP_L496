#include "app_config.h"
#include "app_algorithm.h"   /* Algo_LearnEmptyTank */
#include "bsp_timer.h"       /* BSP_Range_SetLevel：量程档位 → ADC 点数 / S3 脉宽 */
#include "24lc256.h"         /* EEPROM 持久化 */
#include "stlm75m2f.h"       /* 真实温度 */
#include <string.h>

#define CFG_EEPROM_ADDR     0x0000u     /* EEPROM 存储起始地址 */
#define CFG_MAGIC_0         0xA5u
#define CFG_MAGIC_1         0x5Au
#define CFG_VERSION         1u
#define CFG_PAYLOAD_LEN     81u         /* 与 dispproto.h PARAM_DUMP 布局一致 */
#define CFG_HEADER_LEN      5u          /* magic(2) + version(1) + crc16(2) */

/* 前向声明：LoadFromEEPROM 在 RangeToLevel 定义之前调用 */
static uint8_t App_Config_RangeToLevel(float range_m);

/* CRC16-CCITT（poly=0x1021, init=0xFFFF），用于 EEPROM 配置校验 */
static uint16_t App_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++)
        {
            if (crc & 0x8000u)
                crc = (crc << 1) ^ 0x1021u;
            else
                crc = crc << 1;
        }
    }
    return crc;
}

/* 按 dispproto.h PARAM_DUMP 布局序列化 gRadarConfig（81 字节，与显示板严格一致） */
void App_Config_Serialize(const RadarConfig_t *cfg, uint8_t *buf)
{
    uint16_t off = 0;
    memcpy(&buf[off], &cfg->lowAdjustPct, 4); off += 4;
    memcpy(&buf[off], &cfg->lowAdjustVal, 4); off += 4;
    memcpy(&buf[off], &cfg->highAdjustPct, 4); off += 4;
    memcpy(&buf[off], &cfg->highAdjustVal, 4); off += 4;
    buf[off++] = cfg->matType;
    buf[off++] = cfg->matFastChange;
    buf[off++] = cfg->matFirstWave;
    buf[off++] = cfg->matSurfAngle;
    buf[off++] = cfg->matFoamDust;
    buf[off++] = cfg->matSmallDK;
    buf[off++] = cfg->matPipe;
    memcpy(&buf[off], &cfg->pipeDiameter, 4); off += 4;
    memcpy(&buf[off], &cfg->dampTime, 4); off += 4;
    buf[off++] = cfg->outMap;
    buf[off++] = cfg->scaleUnit;
    memcpy(&buf[off], &cfg->scaleVal, 4); off += 4;
    memcpy(&buf[off], &cfg->rangeSetting, 4); off += 4;
    memcpy(&buf[off], &cfg->blindZone, 4); off += 4;
    buf[off++] = cfg->currMode;
    buf[off++] = cfg->currFault;
    buf[off++] = cfg->currMin;
    buf[off++] = cfg->servReset;
    buf[off++] = cfg->servUnit;
    buf[off++] = cfg->servHART;
    buf[off++] = cfg->servHARTAddr;
    memcpy(&buf[off], &cfg->servOffset, 4); off += 4;
    memcpy(&buf[off], &cfg->threshEcho, 4); off += 4;
    memcpy(&buf[off], &cfg->threshEnv, 4); off += 4;
    buf[off++] = cfg->diagSim;
    memcpy(&buf[off], cfg->sensorTag, 16); off += 16;
}

static void Config_Deserialize(RadarConfig_t *cfg, const uint8_t *buf)
{
    uint16_t off = 0;
    float f; uint8_t u;
    memcpy(&f, &buf[off], 4); cfg->lowAdjustPct = f; off += 4;
    memcpy(&f, &buf[off], 4); cfg->lowAdjustVal = f; off += 4;
    memcpy(&f, &buf[off], 4); cfg->highAdjustPct = f; off += 4;
    memcpy(&f, &buf[off], 4); cfg->highAdjustVal = f; off += 4;
    u = buf[off++]; cfg->matType = u;
    u = buf[off++]; cfg->matFastChange = u;
    u = buf[off++]; cfg->matFirstWave = u;
    u = buf[off++]; cfg->matSurfAngle = u;
    u = buf[off++]; cfg->matFoamDust = u;
    u = buf[off++]; cfg->matSmallDK = u;
    u = buf[off++]; cfg->matPipe = u;
    memcpy(&f, &buf[off], 4); cfg->pipeDiameter = f; off += 4;
    memcpy(&f, &buf[off], 4); cfg->dampTime = f; off += 4;
    u = buf[off++]; cfg->outMap = u;
    u = buf[off++]; cfg->scaleUnit = u;
    memcpy(&f, &buf[off], 4); cfg->scaleVal = f; off += 4;
    memcpy(&f, &buf[off], 4); cfg->rangeSetting = f; off += 4;
    memcpy(&f, &buf[off], 4); cfg->blindZone = f; off += 4;
    u = buf[off++]; cfg->currMode = u;
    u = buf[off++]; cfg->currFault = u;
    u = buf[off++]; cfg->currMin = u;
    u = buf[off++]; cfg->servReset = u;
    u = buf[off++]; cfg->servUnit = u;
    u = buf[off++]; cfg->servHART = u;
    u = buf[off++]; cfg->servHARTAddr = u;
    memcpy(&f, &buf[off], 4); cfg->servOffset = f; off += 4;
    memcpy(&f, &buf[off], 4); cfg->threshEcho = f; off += 4;
    memcpy(&f, &buf[off], 4); cfg->threshEnv = f; off += 4;
    u = buf[off++]; cfg->diagSim = u;
    memcpy(cfg->sensorTag, &buf[off], 16); off += 16;
    cfg->sensorTag[15] = '\0';
}

uint8_t App_Config_SaveToEEPROM(void)
{
    uint8_t frame[CFG_HEADER_LEN + CFG_PAYLOAD_LEN];
    frame[0] = CFG_MAGIC_0;
    frame[1] = CFG_MAGIC_1;
    frame[2] = CFG_VERSION;
    App_Config_Serialize(&gRadarConfig, &frame[CFG_HEADER_LEN]);
    uint16_t crc = App_CRC16(&frame[CFG_HEADER_LEN], CFG_PAYLOAD_LEN);
    frame[3] = (uint8_t)(crc & 0xFFu);
    frame[4] = (uint8_t)(crc >> 8);

    HAL_StatusTypeDef st = EEPROM_WriteBytes(CFG_EEPROM_ADDR, frame, sizeof(frame));
    return (st == HAL_OK) ? 0u : 1u;
}

uint8_t App_Config_LoadFromEEPROM(void)
{
    uint8_t frame[CFG_HEADER_LEN + CFG_PAYLOAD_LEN];
    HAL_StatusTypeDef st = EEPROM_ReadBytes(CFG_EEPROM_ADDR, frame, sizeof(frame));
    if (st != HAL_OK) return 1u;

    if (frame[0] != CFG_MAGIC_0 || frame[1] != CFG_MAGIC_1) return 2u;
    if (frame[2] != CFG_VERSION) return 3u;

    uint16_t crc_stored = (uint16_t)frame[3] | ((uint16_t)frame[4] << 8);
    uint16_t crc_calc   = App_CRC16(&frame[CFG_HEADER_LEN], CFG_PAYLOAD_LEN);
    if (crc_stored != crc_calc) return 4u;

    Config_Deserialize(&gRadarConfig, &frame[CFG_HEADER_LEN]);
    /* 加载后同步量程档位 */
    BSP_Range_SetLevel(App_Config_RangeToLevel(gRadarConfig.rangeSetting));
    return 0u;
}

/* 主板侧配置唯一实例 */
RadarConfig_t gRadarConfig;

/* 阻尼滤波状态（一阶低通） */
static float s_damp_filtered = 0.0f;
static uint8_t s_damp_init   = 0U;

/* 量程设定值（米）→ ADC 采样档位
 * 每点距离 ≈ 30.1mm（K=79763），1024 点覆盖 30.8m，3×1024=3072 点覆盖 92.4m（含 70m/84m）：
 *   0~28m  → 档位1(1024点，覆盖30.8m)
 *   28~56m → 档位2(2048点，覆盖61.6m)
 *   ≥56m   → 档位3(3072点，覆盖92.4m，含70m/84m)
 * value<=0（显示板默认"不限制"）按档位1处理，保证上电采样窗口最短。 */
static uint8_t App_Config_RangeToLevel(float range_m)
{
    if (range_m <= 28.0f) return 1U;
    if (range_m <= 56.0f) return 2U;
    return 3U;
}

/* ---- 默认值加载（拆分：基本 / 全部），供 Init 与"复位"两条路径复用 ----
 *
 * 飞卓 §4.3 定义了两级复位：
 *   基本复位 → 只恢复【基本设置页】参数
 *   工厂设置 → 恢复【全部参数】
 * 两者的差别就在于"是否包含服务/输出页与仿真页"。*/

/* 基本设置页字段恢复工厂缺省（对应飞卓"基本复位"） */
static void App_Config_LoadDefaultsBasic(void)
{
    gRadarConfig.lowAdjustPct   = 0.0f;
    gRadarConfig.lowAdjustVal   = 0.0f;
    gRadarConfig.highAdjustPct  = 100.0f;
    gRadarConfig.highAdjustVal  = 0.0f;
    gRadarConfig.matType        = 0U;
    gRadarConfig.matFastChange  = 0U;
    gRadarConfig.matFirstWave   = 1U;
    gRadarConfig.matSurfAngle   = 0U;
    gRadarConfig.matFoamDust    = 0U;
    gRadarConfig.matSmallDK     = 0U;
    gRadarConfig.matPipe        = 0U;
    gRadarConfig.pipeDiameter   = 100.0f;
    gRadarConfig.dampTime       = 0.0f;
    gRadarConfig.outMap         = 0U;
    gRadarConfig.scaleUnit      = 0U;
    gRadarConfig.scaleVal       = 0.0f;
    gRadarConfig.rangeSetting   = 0.0f;
    BSP_Range_SetLevel(1U);
    gRadarConfig.blindZone      = 0.0f;
}

/* 全部字段恢复工厂缺省（对应飞卓"工厂设置"）。
 * 注意：本函数【只恢复配置】，绝不触发空罐基线学习——学习属于
 * "服务 → 虚假回波"的职责（历史缺陷：曾把学习错放在复位里）。*/
static void App_Config_LoadDefaults(void)
{
    memset(&gRadarConfig, 0, sizeof(gRadarConfig));
    App_Config_LoadDefaultsBasic();   /* 基本设置页 */

    /* 服务 / 输出页 */
    gRadarConfig.currMode       = 0U;
    gRadarConfig.currFault      = 0U;
    gRadarConfig.currMin        = 0U;
    gRadarConfig.servReset      = 0U;
    gRadarConfig.servUnit       = 0U;
    gRadarConfig.servHART       = 0U;
    gRadarConfig.servHARTAddr   = 0U;
    gRadarConfig.servOffset     = 0.0f;
    gRadarConfig.threshEcho     = 0.0f;
    gRadarConfig.threshEnv      = 0.0f;

    /* 仿真 / 标识页 */
    gRadarConfig.diagSim        = 0U;
    strncpy(gRadarConfig.sensorTag, "SENSOR", sizeof(gRadarConfig.sensorTag) - 1);

    s_damp_init = 0U;   /* 清空阻尼历史，避免旧值平滑拖尾 */
}

void App_Config_Init(void)
{
    App_Config_LoadDefaults();   /* 上电初值 = 全部参数的工厂缺省 */
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
        case DPARAM_RANGE_SETTING:
            gRadarConfig.rangeSetting = value;
            /* 量程米值 → 采样档位（1~5）：下一次测量序列起 ADC 点数与 S3 脉宽生效；
             * 切档后旧空罐基线点数不匹配，算法层会自动重新学习基线 */
            BSP_Range_SetLevel(App_Config_RangeToLevel(value));
            break;
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
        case DPARAM_SERV_FALSE_ECHO:
            /* 虚假回波操作档位（对齐飞卓 p11-12 + 显示板 dict_flsEcho）：
             *   显示板 MENU_SERV_FALSE_ECHO 是 4 选项菜单（删除/更新/新建/编辑），
             *   min=0/max=3，下发值 = 索引 + 1，故：
             *     value = 1 → 删除：清除已存基线
             *     value = 2 → 更新：重新学习（飞卓语义"真实回波之后保持不变"）
             *     value = 3 → 新建：重新学习（飞卓语义"真实回波之后清零"）
             *     value = 4 → 编辑：两点+幅度生成曲线
             *
             * ⚠ 当前实现状态：2/3 都做"直接学习"（尚未区分"真实回波之后"的分段处理），
             *   4（编辑）尚未实现。这是分两期的计划——先做 1/2/3，编辑后续补。
             * ⚠ 历史缺陷：原实现无论收到哪个档位都只做学习，
             *   导致选"删除"反而去学习基线（语义完全相反）。*/
            if (value == 1.0f)
            {
                Algo_ClearBaseline();          /* 删除 */
            }
            else if (value == 2.0f || value == 3.0f)
            {
                App_Config_LearnFalseEcho();   /* 更新 / 新建（暂同实现，分段处理待补） */
            }
            else
            {
                /* value==4（编辑）暂未实现；未知值不动作 */
            }
            break;

        /* 字符串型不应走这里，忽略 */
        case DPARAM_SENSOR_TAG:
        default:
            break;
    }

    /* 参数变更后持久化到 EEPROM（24LC256），断电不丢失。
     * SET_PARAM 由显示板确认键触发，一次确认一次写入，不会频繁磨损。 */
    App_Config_SaveToEEPROM();
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

    /* 字符串参数变更后持久化到 EEPROM */
    (void)App_Config_SaveToEEPROM();
}

uint8_t App_Config_Reset(uint8_t mode)
{
    /* 复位档位语义：与显示板 MENU_SERV_RESET(dict_reset) 的下发值严格对齐。
     *   显示板菜单项 min=0/max=2，上报值 = 索引 + 1，故：
     *     mode = 1 → 基本复位（"基本设置"页参数恢复工厂缺省）
     *     mode = 2 → 工厂设置（"全部参数"恢复工厂缺省）
     *     mode = 3 → 测量峰值（清诊断峰值统计）
     *
     * ⚠ 历史缺陷（2026-09-20 修正）：原实现把 mode==1 当作"学习空罐基线"，
     *   而 mode==2/3 全为 no-op ⇒ 三项语义全错，其中"工厂设置"会在带料状态下
     *   重新学习空罐基线，把物料回波当固定障碍物永久扣除并写 EEPROM，
     *   造成持续性误测。此处彻底纠正：
     *   ① 复位只做复位，绝不触发学习；
     *   ② 空罐基线学习归"服务 → 虚假回波"（DPARAM_SERV_FALSE_ECHO）职责。*/
    switch (mode)
    {
    case 1U:   /* 基本复位：仅基本设置页字段 */
        App_Config_LoadDefaultsBasic();
        break;

    case 2U:   /* 工厂设置：全部参数（含服务/输出、仿真/标识） */
        App_Config_LoadDefaults();
        break;

    case 3U:   /* 测量峰值：主板无"测量峰值统计"状态可清，实为 no-op。
                * 注：显示板的"测量峰值"是 MEAS 帧带来的只读诊断量（peak_count），
                * 不是主板维护的累加统计，故此处无需动作。
                * ⚠ 不要在这里调 Algo_ClearBaseline()——那属于"虚假回波→删除"。*/
        break;

    default:   /* 0 或其他：不动作 */
        return 0U;
    }

    (void)App_Config_SaveToEEPROM();   /* 复位结果持久化 */
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
    /* 读取 STLM75M2F 温度传感器；失败时兜底返回 25.0℃ */
    float temp = 25.0f;
    if (STLM75_ReadTemp(&temp) != HAL_OK)
    {
        temp = 25.0f;
    }
    return temp;
}
