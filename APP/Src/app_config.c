#include "app_config.h"
#include "app_algorithm.h"   /* Algo_LearnBegin / Algo_SetBaseline */
#include "bsp_timer.h"       /* BSP_Range_SetLevel：量程档位 → ADC 点数 / S3 脉宽 */
#include "24lc256.h"         /* EEPROM 持久化 */
#include "stlm75m2f.h"       /* 真实温度 */
#include <string.h>
#include <stdio.h>           /* printf（经 bsp_usart.c 的 fputc → UART4）*/

#define CFG_EEPROM_ADDR     0x0000u     /* EEPROM 配置区起始地址 */
#define CFG_MAGIC_0         0xA5u
#define CFG_MAGIC_1         0x5Au
#define CFG_VERSION         1u
#define CFG_PAYLOAD_LEN     81u         /* 与 dispproto.h PARAM_DUMP 布局一致 */
#define CFG_HEADER_LEN      5u          /* magic(2) + version(1) + crc16(2) */

/* ---- 空罐基线 EEPROM 存储（与配置区分区，避免互相覆盖）----
 * 24LC256 = 32KB，配置区只用前 86B，基线放在 0x0100 之后。
 * 格式：Magic(2B) + version(1B) + count(2B, 小端) + crc16(2B, 小端) + data(10240B, 小端 uint16)
 * CRC 计算范围 = count(2B) + data(10240B)，算法复用 App_CRC16（CRC16-CCITT）。
 * ⚠ 占用 0x0100 ~ 0x28FF（10247B），24LC256（32KB）余 ~22KB，够用。 */
#define CFG_EEPROM_BASELINE_ADDR    0x0100u
#define CFG_BASELINE_MAGIC_0        0xBAu
#define CFG_BASELINE_MAGIC_1        0x5Eu
/* ⚠ 基线格式或「adc_buf[0] 的含义」一变就必须自增版本号：旧基线会被
 *   LoadBaselineFromEEPROM 判 version 不符而拒收 ⇒ 回到「无基线」，测量 SUPPRESSED，须菜单手动学。
 *   当前 v3：每点 15.034mm、最大 5120 点（基线 10240B）；adc_buf[0] = 真零点(S6↓ 捕获)后的第一拍。 */
#define CFG_BASELINE_VERSION        3u
#define CFG_BASELINE_HEADER_LEN     7u          /* magic(2) + version(1) + count(2) + crc16(2) */
#define CFG_BASELINE_DATA_BYTES     (ADC_SAMPLE_COUNT_MAX * 2u)   /* 5120 × 2 = 10240B */

static uint8_t App_Config_RangeToLevel(float range_m);

/* CRC16-CCITT（poly=0x1021, init=0xFFFF），用于 EEPROM 配置 & 基线校验 */
static uint16_t App_CRC16_Accumulate(const uint8_t *data, uint16_t len, uint16_t crc)
{
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

static uint16_t App_CRC16(const uint8_t *data, uint16_t len)
{
    return App_CRC16_Accumulate(data, len, 0xFFFFu);
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
    /* ⚠ 必须同步量程档位：BSP_Range 档位状态不随 EEPROM 恢复，否则采样点数与量程错配 */
    BSP_Range_SetLevel(App_Config_RangeToLevel(gRadarConfig.rangeSetting));
    /* ⚠ 盲区也必须同步给算法：它决定寻峰前屏蔽多少点（0 ⇒ 算法退回默认 1 m） */
    Algo_SetBlindZoneM(gRadarConfig.blindZone);
    return 0u;
}

/* ============================================================
 *  空罐基线 EEPROM 持久化
 *  存储格式见 app_config.h 说明 + CFG_BASELINE_* 常量。
 *  注意：
 *   - 基线数据量大（10KB），写入耗时约 160ms（每页 5ms × 160 页），只在用户主动
 *     触发学习/删除时操作，不在测量主循环里写，避免阻塞测距。
 *   - CRC 计算覆盖 count(2B) + data(10240B)，共 10242B；用 App_CRC16（CRC16-CCITT）。
 * ============================================================ */

uint8_t App_Config_SaveBaselineToEEPROM(void)
{
    const uint16_t *baseline;
    uint16_t count;
    HAL_StatusTypeDef st;

    /* 无基线则不存 */
    if (Algo_HasBaseline() == 0U)
        return 1u;

    baseline = Algo_GetBaseline();
    if (baseline == (const uint16_t *)0)
        return 1u;

    count = BSP_Range_GetSampleCount();
    if (count == 0U || count > ADC_SAMPLE_COUNT_MAX)
        return 1u;

    /* ---- 组装头部 ---- */
    {
        uint8_t hdr[CFG_BASELINE_HEADER_LEN];
        uint16_t crc;

        hdr[0] = CFG_BASELINE_MAGIC_0;
        hdr[1] = CFG_BASELINE_MAGIC_1;
        hdr[2] = CFG_BASELINE_VERSION;
        hdr[3] = (uint8_t)(count & 0xFFu);          /* count 小端 */
        hdr[4] = (uint8_t)(count >> 8);

        /* CRC：先算 count(2B)，再累计 data(10240B) */
        crc = App_CRC16(&hdr[3], 2u);
        /* 基线是 uint16 数组，按字节逐点送 CRC（STM32 小端，直接 memcpy 语义一致）
         * ⚠ 这里的 (const uint8_t *) 转换是刻意的：把 uint16 序列按字节喂给 CRC，
         *   与下面写 EEPROM 时的同一处理保持一致，勿改成按 uint16 逐点算。 */
        crc = App_CRC16_Accumulate((const uint8_t *)baseline, CFG_BASELINE_DATA_BYTES, crc);

        hdr[5] = (uint8_t)(crc & 0xFFu);           /* crc16 小端 */
        hdr[6] = (uint8_t)(crc >> 8);

        st = EEPROM_WriteBytes(CFG_EEPROM_BASELINE_ADDR, hdr, CFG_BASELINE_HEADER_LEN);
        if (st != HAL_OK) return 2u;
    }

    /* ---- 写入基线数据（10240B，24LC256 页写自动分页）----
     * ⚠ 基线是 uint16_t 数组，STM32 小端，直接按字节写入 EEPROM 即为小端布局；
     *   读取时也直接 memcpy 到 uint16_t 数组，两端一致。 */
    st = EEPROM_WriteBytes(CFG_EEPROM_BASELINE_ADDR + CFG_BASELINE_HEADER_LEN,
                           (uint8_t *)baseline, CFG_BASELINE_DATA_BYTES);
    if (st != HAL_OK) return 3u;

    return 0u;
}

uint8_t App_Config_LoadBaselineFromEEPROM(void)
{
    /* 静态缓冲：放 .bss 段，不占栈；L496 SRAM 320KB，6KB 完全可接受。
     * 用静态而非 malloc：嵌入式避免堆碎片化，且加载只在上电做一次。 */
    static uint16_t s_baseline_tmp[ADC_SAMPLE_COUNT_MAX];
    uint8_t hdr[CFG_BASELINE_HEADER_LEN];
    uint16_t stored_count;
    uint16_t cur_count;
    uint16_t crc_stored, crc_calc;
    HAL_StatusTypeDef st;

    /* 读头部 */
    st = EEPROM_ReadBytes(CFG_EEPROM_BASELINE_ADDR, hdr, CFG_BASELINE_HEADER_LEN);
    if (st != HAL_OK) return 1u;

    /* Magic 校验 */
    if (hdr[0] != CFG_BASELINE_MAGIC_0 || hdr[1] != CFG_BASELINE_MAGIC_1)
        return 2u;

    /* Version 校验 */
    if (hdr[2] != CFG_BASELINE_VERSION)
        return 3u;

    stored_count = (uint16_t)hdr[3] | ((uint16_t)hdr[4] << 8);
    crc_stored   = (uint16_t)hdr[5] | ((uint16_t)hdr[6] << 8);

    /* 档位校验：存储的基线点数必须与当前档位一致，否则视为无效（切档后基线作废，须手动重学）
     * ⚠ 不能用 BSP_Range_GetSampleCount()：它返回已快照的 adc_target_count，而开机加载基线时
     *   采样从未启动、只会返回初值 1024（第一档），非第一档的 EEPROM 基线会永远校验失败。
     *   用 BSP_Range_GetRequestSampleCount()：请求档位对应点数，开机即有效。 */
    cur_count = BSP_Range_GetRequestSampleCount();
    if (stored_count != cur_count)
        return 4u;
    if (stored_count == 0U || stored_count > ADC_SAMPLE_COUNT_MAX)
        return 5u;

    /* 读基线数据到临时缓冲 */
    st = EEPROM_ReadBytes(CFG_EEPROM_BASELINE_ADDR + CFG_BASELINE_HEADER_LEN,
                          (uint8_t *)s_baseline_tmp, CFG_BASELINE_DATA_BYTES);
    if (st != HAL_OK) return 6u;

    /* CRC 校验：计算范围 = count(2B) + data(10240B) */
    crc_calc = App_CRC16(&hdr[3], 2u);
    crc_calc = App_CRC16_Accumulate((const uint8_t *)s_baseline_tmp,
                                     CFG_BASELINE_DATA_BYTES, crc_calc);
    if (crc_calc != crc_stored)
        return 7u;

    /* 校验全部通过，写入算法层 */
    Algo_SetBaseline(s_baseline_tmp);

    /* ★ 2026-10-30：导入成功也打一行摘要。
     *   此前只有「学习完成」才打 baseline dump，导入成功一个字都不出 ⇒ 串口上无法区分
     *   「用的是 EEPROM 里的旧基线」和「压根没导入」。两条互斥判据：
     *     ① 有 baseline restored 行 = 用的就是 EEPROM 那份（比对 n/mean 能认出是哪次学的）
     *     ② 只有 SUPPRESSED           = 没导入（无基线或下面 7 道校验任一失败）⇒ 等人进菜单手动学
     *   n 必须等于当前档位点数；min≈max≈0 说明这份基线可疑
     *   （全 0 会让差分退化成恒等 = 等于没抑制，却报「有基线」）。
     *   开销：上电时扫一遍 N 点（≤5120），几毫秒，且只在导入成功时走一次。 */
    {
        uint16_t i;
        uint16_t vmin = 0xFFFFU;
        uint16_t vmax = 0U;
        uint32_t sum  = 0U;

        for (i = 0U; i < stored_count; i++)
        {
            if (s_baseline_tmp[i] < vmin) vmin = s_baseline_tmp[i];
            if (s_baseline_tmp[i] > vmax) vmax = s_baseline_tmp[i];
            sum += (uint32_t)s_baseline_tmp[i];
        }
        printf("[RADAR] baseline restored from EEPROM: n=%u min=%u max=%u mean=%u crc=OK\r\n",
               (unsigned)stored_count, (unsigned)vmin, (unsigned)vmax,
               (unsigned)((stored_count != 0U) ? (sum / (uint32_t)stored_count) : 0U));
    }

    return 0u;
}

uint8_t App_Config_ClearBaselineInEEPROM(void)
{
    uint8_t zero[2] = {0, 0};
    HAL_StatusTypeDef st;

    /* 把 Magic 字段写 0，即可让后续加载时判定为"无有效基线"；
     * 不用擦除整块 6KB，省时间。 */
    st = EEPROM_WriteBytes(CFG_EEPROM_BASELINE_ADDR, zero, 2u);
    return (st == HAL_OK) ? 0u : 1u;
}

/* 主板侧配置唯一实例 */
RadarConfig_t gRadarConfig;

/* 阻尼滤波状态（一阶低通） */
static float s_damp_filtered = 0.0f;
static uint8_t s_damp_init   = 0U;

/* 量程设定值（米）→ ADC 采样档位。每点 15.034mm（K=159526，刻度是 2026-10-10 前的 2 倍细）：
 *   1 → 1024 点 ≈ 15.4m；2 → 2048 ≈ 30.8m；3 → 3072 ≈ 46.2m；
 *   4 → 4096 ≈ 61.6m；5 → 5120 ≈ 77.0m。
 * ★ 2026-10-14 由 3 档扩到 5 档：K 翻倍后每点距离减半，旧映射（≤28→1、≤56→2、其余→3）
 *   会让 28m 设定只拿到 1024 点（15.4m）、75m 设定只拿到 3072 点（46.2m），量程不足。
 * ⚠ value<=0（显示板默认"不限制"）按档位 1 处理，保证上电采样窗口最短 */
static uint8_t App_Config_RangeToLevel(float range_m)
{
    /* ⚠ NaN 防护（2026-10-22，R-06）：下面所有 `<=` 比较对 NaN 均为假 ⇒ NaN 会一路落到
     *   return 5（5120 点），把每周期采样时间从 16.4ms 拉到 81.9ms（见 R-21）。
     *   用 `!(x > 0)` 而不是 `x <= 0` 才能把 NaN 一起挡住。 */
    if (!(range_m > 0.0f))      return 1U;   /* 0 / 负数 / NaN → 档 1（与"未设置"语义一致） */
    if (range_m <= 15.0f) return 1U;
    if (range_m <= 30.0f) return 2U;
    if (range_m <= 45.0f) return 3U;
    if (range_m <= 60.0f) return 4U;
    return 5U;
}

/* ============ 有效量程（量程语义的单一真值来源，2026-10-22 R-06）============
 * 背景：rangeSetting = 0 曾被两处按不同语义解释 ——
 *   App_Config_RangeToLevel(0) → 档 1（采样窗仅 15.4m）；
 *   App_Config_ApplyRange() 的 `rangeSetting > 0` → 视为"不限制上限"。
 *   采样窗只有 15.4m 却声明"不限制"，二者互相矛盾：30m 罐的回波在 15.4m 之外根本采不到，
 *   却不会报量程超限（表现为"高料位永远测不到"）。
 * 统一后：rangeSetting <= 0 / 非有限值 / 超上限 ⇒ 有效量程 = 【当前档位的物理量程】，
 *   与 RangeToLevel 的"0 → 档 1"完全自洽（15.39m），且以后改档自动跟随。 */
#define CFG_DIST_PER_SAMPLE_M   0.015034f   /* = c/(2·62.5kHz)/K，K = RADAR_HW_TIME_STRETCH(159526)
                                             * ⚠ 与 app_algorithm.h 同源但本层不引算法头（避免反向依赖），
                                             *   改 K 必须同步改这里。 */
#define CFG_RANGE_MAX_M         77.0f       /* 5 档上限 = 5120 × 15.034mm ≈ 76.97m */

float App_Config_GetEffectiveRange(void)
{
    float r = gRadarConfig.rangeSetting;

    if (!(r > 0.0f) || (r > CFG_RANGE_MAX_M))   /* `!(r>0)` 同时挡住 0/负数/NaN */
    {
        r = (float)BSP_Range_GetRequestSampleCount() * CFG_DIST_PER_SAMPLE_M;
    }
    return r;
}

/* ---- 默认值加载（基本 / 全部），供 Init 与"复位"两条路径复用 ----
 * 飞卓 §4.3 两级复位：基本复位只恢复【基本设置页】，工厂设置恢复【全部参数】 */

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

/* 全部字段恢复工厂缺省（对应飞卓"工厂设置"）
 * ⚠ 只恢复配置，绝不触发空罐基线学习——学习归"服务 → 虚假回波"职责 */
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

    /* 盲区恢复工厂缺省（blindZone=0）⇒ 算法侧退回默认 1 m，不可关闭 */
    Algo_SetBlindZoneM(gRadarConfig.blindZone);
}

void App_Config_Init(void)
{
    App_Config_LoadDefaults();   /* 上电初值 = 全部参数的工厂缺省 */
}

uint8_t App_Config_SetParam(DISP_PARAM_ID id, float value)
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
            /* 量程米值 → 采样档位：下一次测量序列起 ADC 点数与 S3 脉宽生效；
             * ⚠ 切档后旧空罐基线点数不匹配 ⇒ 基线作废、测量 SUPPRESSED，须菜单手动重学 */
            BSP_Range_SetLevel(App_Config_RangeToLevel(value));
            break;
        case DPARAM_BLIND_ZONE:
            gRadarConfig.blindZone = value;
            /* 立即生效：算法在下一次测量寻峰前按新盲区屏蔽（0 ⇒ 退回默认 1 m）*/
            Algo_SetBlindZoneM(value);
            break;

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
            /* 虚假回波档位（对齐飞卓 p11-12；下发值 = 显示板索引 + 1）：
             * 1=删除 / 2=更新 / 3=新建 / 4=编辑；⚠ 2/3 暂同为"直接学习"，4 未实现
             * ⚠ 本函数运行在主循环（Disp_RecvPoll 调用），EEPROM 写入阻塞 ~100ms 可接受 */
            if (value == 1.0f)
            {
                Algo_ClearBaseline();          /* RAM 层清基线 */
                App_Config_ClearBaselineInEEPROM();  /* EEPROM 同步清掉 */
            }
            else if (value == 2.0f || value == 3.0f)
            {
                App_Config_LearnFalseEcho();   /* 更新 / 新建 */
            }
            else
            {
                /* value==4（编辑）暂未实现；未知值不动作 */
            }
            break;

        /* 字符串型不应走这里：sensorTag 必须走 0x85 SET_STR。
         * ⚠ 这里直接返回"未知 ID"，且【不落盘】—— 若仍走末尾的统一落盘，
         *   会把一次无效请求变成一次无谓的 EEPROM 写（阻塞主循环且毫无意义）。 */
        case DPARAM_SENSOR_TAG:
        default:
            return DISP_ACK_BAD_ID;
    }

    /* 参数变更后持久化到 EEPROM，断电不丢失（由显示板确认键触发，一次确认一次写入）。
     * ★ 返回值即 ACK 结果：0=落盘成功；非 0=RAM 已改但 EEPROM 写失败，
     *   由 app_disp.c 原样回给显示板，避免"界面显示已改、实际断电即丢"。 */
    return App_Config_SaveToEEPROM();
}

uint8_t App_Config_SetStr(DISP_PARAM_ID id, const char *str)
{
    if (str == 0) return DISP_ACK_BAD_FRAME;

    switch (id)
    {
        case DPARAM_SENSOR_TAG:
            strncpy(gRadarConfig.sensorTag, str, sizeof(gRadarConfig.sensorTag) - 1);
            gRadarConfig.sensorTag[sizeof(gRadarConfig.sensorTag) - 1] = '\0';
            break;
        default:
            return DISP_ACK_BAD_ID;
    }

    /* 字符串参数变更后持久化到 EEPROM，返回值即 ACK 结果 */
    return App_Config_SaveToEEPROM();
}

uint8_t App_Config_Reset(uint8_t mode)
{
    /* 复位档位（下发值 = 显示板索引 + 1）：1=基本复位 / 2=工厂设置 / 3=测量峰值
     * ⚠ 复位只做复位，绝不在此触发空罐基线学习（学习归 DPARAM_SERV_FALSE_ECHO） */
    switch (mode)
    {
    case 1U:   /* 基本复位：仅基本设置页字段 */
        App_Config_LoadDefaultsBasic();
        break;

    case 2U:   /* 工厂设置：全部参数（含服务/输出、仿真/标识） */
        App_Config_LoadDefaults();
        break;

    case 3U:   /* 测量峰值：主板无对应统计状态，实为 no-op
                * ⚠ 不要在此调 Algo_ClearBaseline()——那属于"虚假回波→删除" */
        break;

    default:   /* 0 或其他：不动作 */
        return 0U;
    }

    (void)App_Config_SaveToEEPROM();   /* 复位结果持久化 */
    return 0U;
}

void App_Config_LearnFalseEcho(void)
{
    /* ★ 2026-10-29：改成【多帧异步】学习。
     *   原实现是单帧同步：把当次 adc_buf 直接拷成基线 ⇒ 那一帧的噪声被永久钉进基线
     *   （delta = adc − baseline，基线逐点误差会 1:1 进入差分信号）。
     *   现在只发一个学习会话，由 app_radar 每完成一次采样推一拍，攒够 ALGO_LEARN_FRAMES
     *   帧做逐点平均，再落盘。
     *   ⚠ 因此本函数返回时基线【还没更新】—— 显示板的 SAVED OK 只代表"命令已受理"，
     *     真正完成看调试口 [ALGO] learn OK 与 [RADAR] baseline learned & saved 两行。
     *   ⚠ 会话是异步的另一个硬原因：8 帧 ≈ 4.3 s，同步等会阻塞主循环，USART 收帧全丢。 */
    uint8_t r = Algo_LearnBegin();

    if (r == 1U)
    {
        printf("[CFG] false-echo learn BUSY: already running (%u/%u frames)\r\n",
               (unsigned)Algo_LearnProgress(), (unsigned)ALGO_LEARN_FRAMES);
        return;
    }
    if (r != 0U)
    {
        printf("[CFG] false-echo learn NOT STARTED (code=%u: sample not ready / bad count)\r\n",
               (unsigned)r);
        return;
    }

    /* 启动成功：这里【不】写 EEPROM —— 等 Algo_LearnTick 判定学完（返回 1）后，
     * 由 app_radar 那边调 App_Config_SaveBaselineToEEPROM()。提前写只会把旧基线落盘。 */
    printf("[CFG] false-echo learn STARTED: %u frames, keep the tank EMPTY for ~%.1f s\r\n",
           (unsigned)ALGO_LEARN_FRAMES, (double)((float)ALGO_LEARN_FRAMES * 0.535f));
}

float App_Config_ApplyRange(float dist)
{
    /* 上限 = 有效量程（单一真值来源，见 App_Config_GetEffectiveRange）。
     * ★ 2026-10-22 R-06：原判据是 `rangeSetting > 0`，使 rangeSetting==0 落到"不限制"，
     *   与 RangeToLevel 的"0 → 档 1（15.4m）"矛盾；现在恒有上限，且该上限与采样窗一致。
     * ⚠ 有效量程只做"数值夹取"，不改测量成功/失败语义（盲区同理，见 R-24）。 */
    float eff_range = App_Config_GetEffectiveRange();

    if (gRadarConfig.blindZone > 0.0f && dist < gRadarConfig.blindZone)
        dist = gRadarConfig.blindZone;

    if (dist > eff_range)
        dist = eff_range;

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

    /* 一阶低通：alpha = dt/(tau+dt)，dt 取「平均测量间隔」535ms ⇒ 系数 1.87 = 1/0.535s。
     *
     * ★ 2026-10-29 修「菜单秒数名不副实」：原式是 alpha = 1/(1 + dampTime*5.0f)，
     *   那个 5 就是「每秒更新 5 次」，等价于把 dt 写死成 200ms（因为 1/(1+5T) ≡ 0.2/(T+0.2)）。
     *   但本机实际测量率只有 1.87Hz —— S3 成对发送：组内间隔 4T≈178ms、组间 20T≈892ms，
     *   平均 535ms 一次 ⇒ 菜单写 3 实际时间常数 tau = 8.0s（差 2.67 倍）。
     *   现场症状即「回波波形早就跟上了，距离显示要好几秒才爬到准值」。
     *   把系数换成 1.87 之后，tau 精确等于菜单秒数（仿真实测 2/3/5/8/10/15s 全部对得上）。
     *
     * ⚠ 迁移：老值 x 2.67 = 新值（老 3 对应新 8）。不换算就沿用旧值会让平滑度明显变差。
     * ⚠ 1.87 是「平均」测量率，换量程档位（采样点数 1024~5120）会有约 ±10% 漂移；
     *   若要把这 10% 也消掉，需改成实测 dt（HAL_GetTick 差值），属另一档改动。*/
    float alpha = 1.0f / (1.0f + gRadarConfig.dampTime * 1.87f);
    if (alpha > 1.0f) alpha = 1.0f;
    /* 下限 0.002 ⇒ tau 上限约 266s，菜单最大 100s 不会被钳（原来 0.01 会把 >52s 全钳成 53s）*/
    if (alpha < 0.002f) alpha = 0.002f;

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
    /* 占位实现：固定输出 2.0 m */
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
