#include "app_algorithm.h"
#include "bsp_timer.h"    /* adc_buf, ADC_SAMPLE_COUNT_MAX, BSP_Range_GetSampleCount, BSP_ADCsamp_IsDone */
#include <math.h>
#include <string.h>
#include <stdio.h>

/* ============================================================
 *  算法原理（4 步流水线）
 *    1 滑动平均：窗 W=7，3σ 噪声降到约 38%，峰幅度几乎不变
 *    2 自适应阈值：μ + k·σ（k=3 ≈ 99.7% 置信），避免误判噪声
 *    3 局部极大值检测：> 阈值且 ≥ 左右邻居；间距 < MIN_DIST 只留更强者
 *    4 抛物线插值：峰及左右 3 点求顶点偏移，精度 ±1 → ±0.1 采样间隔
 * ============================================================ */

/* ---- 内部静态缓冲区：按最大量程档 5120 点静态分配，运行时按当前档位数处理 ---- */
static float s_smoothed[ADC_SAMPLE_COUNT_MAX];    /* Algo_FindPeaks 滑动平均输出缓冲 */
static float s_measure_sm[ADC_SAMPLE_COUNT_MAX];  /* 调用方输入缓冲（ADC 数据或 delta 差分）*/

/* ---- ADC 失败哨兵：ADCS7476 读取失败时 BSP 写入 adc_buf 的 0xFFFF ----
 * ⚠ 必须在【转 float / 进基线之前】剔除：滑动平均会把它摊给邻居，且会被永久写进
 *   s_baseline ⇒ 该点此后每周期恒为巨负值、被截零，永久丢失。
 * ⚠ 一次失败 = 时间轴断裂（吃掉约 10 个 TIM7 周期却只前进 1 个下标）：落在回波峰
 *   之前会使距离少算约 30cm，且不置 adc_error、静默返回成功。
 *   ⇒ 弃用判据按【位置】而非【数量】：失败点全在回波峰之后才保留；基线学习 / 累加
 *     出现任何失败点即整次放弃。
 * ⚠ 哨兵读取点（改值须同步改）：Algo_FixupSentinelU16 / Algo_FindStrongestPeak /
 *   Algo_MeasureDistance，以及 app_disp.c 的曲线构造（显示用，按 0 处理不插值）。
 * ⚠ DMA 写入口（bsp_spi.c）只掩码不写哨兵、当前不可达；恢复 DMA 路径时须在失败分支
 *   补写 0xFFFFU，否则剔除逻辑对 DMA 采到的失败点完全无效。
 */
#define ALGO_ADC_FAIL_SENTINEL  0xFFFFU
#define ALGO_IS_ADC_FAIL(v)     ((uint16_t)(v) == ALGO_ADC_FAIL_SENTINEL)

/* 失败点【数量】硬门槛：总数超过它则不论位置一律弃用（"SPI 持续恶化"的兜底闸门）。*/
#define ALGO_ADC_FAIL_MAX_POINTS  8U

/* adc_fail_count 为连续计数且连续 3 次即停 TIM7 ⇒ 能到算法层的失败段恒 ≤ 2，判据按【位置】。*/

/**
  * @brief  把 buf 中的 0xFFFF 哨兵段用左右有效点线性插值填充
  * @note   就地修改 buf；返回被插值替换掉的哨兵点数；*first_idx = 最靠前的哨兵下标（无哨兵则写 count）。
  *         开头/结尾哨兵段用最近有效值水平外推；整段全失败则不改动，由调用方按返回值弃用。
  * @param  buf       : [in,out] uint16 序列（adc_buf 或 s_baseline），可为 NULL
  * @param  count     : 数据点数
  * @param  first_idx : [out] 最靠前的哨兵下标（可为 NULL）
  * @retval 被插值替换掉的哨兵点数
  * @complexity 时间 O(N)，空间 O(1)
  */
static uint16_t Algo_FixupSentinelU16(uint16_t *buf, uint16_t count, uint16_t *first_idx)
{
    uint16_t i;
    uint16_t fixed = 0U;

    if (first_idx != (uint16_t *)0)
        *first_idx = count;      /* 默认"无失败点" */

    if (buf == (uint16_t *)0 || count == 0U)
        return 0U;

    i = 0U;
    while (i < count)
    {
        if (!ALGO_IS_ADC_FAIL(buf[i]))
        {
            i++;
            continue;
        }

        /* 找本段失败区间的左右端点：run = [i, k-1]，共 (k - i) 个 */
        {
            uint16_t k     = i;
            uint16_t left  = 0U;
            uint16_t right = 0U;
            int      has_l = 0;
            int      has_r = 0;
            uint16_t run;

            while (k < count && ALGO_IS_ADC_FAIL(buf[k]))
                k++;
            run = (uint16_t)(k - i);

            if (fixed == 0U && first_idx != (uint16_t *)0)
                *first_idx = i;      /* 最靠前的失败点下标（位置判据用）*/

            if (i > 0U)          { left  = (uint16_t)(i - 1U); has_l = 1; }
            if (k < count)       { right = k;                  has_r = 1; }

            if (has_l && has_r)
            {
                uint16_t span = (uint16_t)(right - left);
                int32_t  v0   = (int32_t)buf[left];
                int32_t  v1   = (int32_t)buf[right];
                uint16_t t;
                for (t = 0U; t < run; t++)
                {
                    int32_t num = (int32_t)(t + 1U);
                    int32_t v   = v0 + (int32_t)(((v1 - v0) * num) / (int32_t)span);
                    if (v < 0)     v = 0;
                    if (v > 0x0FFF) v = 0x0FFF;   /* 12 位 ADC 值域 */
                    buf[i + t] = (uint16_t)v;
                }
            }
            else if (has_l)
            {
                /* 尾部段：用左侧值水平外推（保持连续，不制造边缘凹陷）*/
                uint16_t v = buf[left];
                uint16_t t;
                for (t = 0U; t < run; t++)
                    buf[i + t] = v;
            }
            else if (has_r)
            {
                /* 头部段：用右侧值水平外推 */
                uint16_t v = buf[right];
                uint16_t t;
                for (t = 0U; t < run; t++)
                    buf[i + t] = v;
            }
            /* 两侧都没有（整段全失败）→ 一个都不改，交给调用方按 fixed 判定弃用 */

            fixed = (uint16_t)(fixed + run);
            i = k;
        }
    }

    return fixed;
}

/**
  * @brief  哨兵剔除（float 版，用于无基线降级路径）
  * @note   与 uint16 版同构，但输出到 dst（源 adc_buf 是 volatile，转 float 后插值以避免改动物理采样缓存）
  * @note   *first_idx 与 uint16 版同义（最靠前哨兵下标，无则写 count）
  * @retval 被插值替换掉的哨兵点数
  */
static uint16_t Algo_FixupSentinelFloat(const uint16_t *src, float *dst,
                                        uint16_t count, uint16_t *first_idx)
{
    uint16_t i;
    uint16_t fixed = 0U;

    if (first_idx != (uint16_t *)0)
        *first_idx = count;      /* 默认"无失败点" */

    if (src == (const uint16_t *)0 || dst == (float *)0 || count == 0U)
        return 0U;

    /* 先整体转 float，哨兵原样带入 */
    for (i = 0U; i < count; i++)
        dst[i] = (float)src[i];

    i = 0U;
    while (i < count)
    {
        if (!ALGO_IS_ADC_FAIL(src[i]))
        {
            i++;
            continue;
        }

        {
            uint16_t k     = i;
            uint16_t left  = 0U;
            uint16_t right = 0U;
            int      has_l = 0;
            int      has_r = 0;
            uint16_t run;

            while (k < count && ALGO_IS_ADC_FAIL(src[k]))
                k++;
            run = (uint16_t)(k - i);

            if (fixed == 0U && first_idx != (uint16_t *)0)
                *first_idx = i;      /* 最靠前的失败点下标（位置判据用）*/

            if (i > 0U)    { left  = (uint16_t)(i - 1U); has_l = 1; }
            if (k < count) { right = k;                  has_r = 1; }

            if (has_l && has_r)
            {
                uint16_t span = (uint16_t)(right - left);
                float    v0   = dst[left];
                float    v1   = dst[right];
                uint16_t t;
                for (t = 0U; t < run; t++)
                {
                    float num = (float)(t + 1U);
                    dst[i + t] = v0 + (v1 - v0) * num / (float)span;
                }
            }
            else if (has_l)
            {
                uint16_t t;
                for (t = 0U; t < run; t++)
                    dst[i + t] = dst[left];
            }
            else if (has_r)
            {
                uint16_t t;
                for (t = 0U; t < run; t++)
                    dst[i + t] = dst[right];
            }

            fixed = (uint16_t)(fixed + run);
            i = k;
        }
    }

    return fixed;
}

/**
  * @brief  滑动平均滤波：对输入序列做奇数窗口滑动平均，抑制高频随机噪声
  * @note   窗口边界处自动收缩（不补零），首尾点参与运算的邻居数 < win
  * @param  src   : 输入 float 序列指针（ADC 值或差分信号），不可为 NULL
  * @param  dst   : 输出滤波后 float 数组指针，长度 >= count
  * @param  count : 数据点数（= BSP_Range_GetSampleCount()，1024~5120）
  * @param  win   : 滑动窗口宽度（必须为奇数，默认 7）；越大越平滑但峰越钝
  * @retval 无（结果写入 dst）
  * @complexity  时间 O(N*W)，空间 O(1) 额外
  */
static void smooth_data(const float *src, float *dst, uint16_t count, uint8_t win)
{
    uint8_t half = win / 2U;
    for (uint16_t i = 0; i < count; i++)
    {
        float    sum = 0.0f;
        uint16_t cnt = 0;
        int32_t  j;
        int32_t  start = (int32_t)i - (int32_t)half;
        int32_t  end   = (int32_t)i + (int32_t)half;

        if (start < 0)         start = 0;
        if (end >= (int32_t)count) end = (int32_t)count - 1;

        for (j = start; j <= end; j++)
        {
            sum += src[j];
            cnt++;
        }
        dst[i] = sum / (float)cnt;
    }
}

/**
  * @brief  统计均值与标准差：两趟遍历计算序列的 μ 和 σ，用于自适应阈值生成
  * @note   标准差为总体标准差（除以 N 而非 N-1）
  * @param  data : 输入 float 序列指针（通常为滤波后数据）
  * @param  count: 数据点数
  * @param  mean : [out] 输出均值 μ
  * @param  std  : [out] 输出标准差 σ
  * @retval 无（结果写入 *mean 和 *std）
  * @complexity  时间 O(N) 两趟遍历，空间 O(1)
  */
static void compute_stats(const float *data, uint16_t count, float *mean, float *std)
{
    float sum = 0.0f;
    uint16_t i;

    for (i = 0; i < count; i++)
        sum += data[i];
    *mean = sum / (float)count;

    float var = 0.0f;
    for (i = 0; i < count; i++)
    {
        float d = data[i] - *mean;
        var += d * d;
    }
    *std = sqrtf(var / (float)count);
}

/**
  * @brief  抛物线插值求亚采样峰位：用峰左/峰/峰右 3 点拟合抛物线求顶点偏移
  * @note   取峰左(idx-1)/峰(idx)/峰右(idx+1)三点，顶点偏移 δ = (y0-y2) / (2·(y0-2·y1+y2))，
  *         位置 = idx + δ，δ 限幅 [-0.5, +0.5]（精度 ±1 → ±0.1 采样间隔）
  * @param  data : 滤波后 float 序列指针
  * @param  count: 序列长度，用于边界检查
  * @param  idx  : 局部极大值的整数下标（1 <= idx <= count-2）
  * @retval 精确峰位置 = idx + δ（float）
  * @complexity  时间 O(1)，空间 O(1)
  */
static float refine_peak(const float *data, uint16_t count, uint16_t idx)
{
    if (idx == 0U || idx >= count - 1U)
        return (float)idx;

    float y0 = data[idx - 1U];
    float y1 = data[idx];
    float y2 = data[idx + 1U];

    float denom = 2.0f * (y0 - 2.0f * y1 + y2);
    if (fabsf(denom) < 1e-6f)
        return (float)idx;

    float delta = (y0 - y2) / denom;
    if (delta > 0.5f)  delta = 0.5f;
    if (delta < -0.5f) delta = -0.5f;

    return (float)idx + delta;
}

/* ============ 半高宽连通区质心（治宽峰峰位乱跳）============
 * 问题：回波顶部是一片 40~50 点的宽"高原"，顶上几个锯齿尖幅度只差 10~25%，
 *   取【全场幅度最大者】时谁高一点点峰位就跳到谁头上 —— 实测 σ 0.124 m、单帧最大跳 0.37 m。
 *   后续 App_Config_ApplyDamping 只是把台阶糊成斜坡，压不掉跳变本身。
 * 做法：以选中峰为种子，在【平滑序列】上【连通】扩到 >= 峰值×半高比，取幅度加权质心。
 *   锯齿尖的半高边界很稳（实测连通区 36~49 点），质心锁的是整块回波包络，
 *   对顶部谁高谁低不敏感 ⇒ 实测 σ 0.124 → 0.028 m，跳 >0.15 m 的帧 9/27 → 0/27。
 * 兜底（返回负值 ⇒ 调用方沿用 refine_peak）：连通区宽 <3 点（已退化成单点）、
 *   >160 点（整段被抬高，质心会被两侧无关数据拖走）、加权和 <=0（除零保护）。
 * ⚠ 只读 s_smoothed（Algo_FindPeaks 最后填的那份），不重算平滑，保证与寻峰同一路信号、
 *   且下标与 peaks[].index 严格对齐。
 * ⚠ 口径变化：同一回波上质心比 refine_peak 大（1.2 m 处 +38 mm）—— 质心锁包络中点，
 *   refine_peak 落在平台的某个局部极大上。偏量 servOffset 按质心口径标定，
 *   退回 refine_peak 会系统性偏小。
 * ============================================================ */
#define ALGO_CENTROID_HALF_NUM    1U     /* 半高比 = NUM / DEN = 0.5 */
#define ALGO_CENTROID_HALF_DEN    2U
#define ALGO_CENTROID_MIN_WIDTH   3U     /* 连通区最小宽度（点）*/
#define ALGO_CENTROID_MAX_WIDTH   160U   /* 连通区最大宽度（点）；实测回波半高宽 36~53 点 */

/**
  * @brief  半高宽连通区幅度加权质心（抑制宽峰顶部锯齿导致的峰位乱跳）
  * @param  data  : 平滑后的 float 序列（s_smoothed）
  * @param  count : 序列长度
  * @param  idx   : 种子峰整数下标（须 < count）
  * @param  out_lo: [out] 连通区左边界（可为 NULL）
  * @param  out_hi: [out] 连通区右边界（可为 NULL）
  * @retval >=0 质心位置（float，含小数）；<0 表示兜底条件命中，调用方须沿用原峰位
  * @complexity  时间 O(N)（只在连通区内循环），空间 O(1)
  */
static float Algo_CentroidHalfRegion(const float *data, uint16_t count, uint16_t idx,
                                     uint16_t *out_lo, uint16_t *out_hi)
{
    float    peak, half;
    uint16_t a, b, w;
    uint32_t k;
    float    sum = 0.0f, wsum = 0.0f;

    if (data == NULL || count == 0U || idx >= count)
        return -1.0f;

    peak = data[idx];
    if (peak <= 0.0f)
        return -1.0f;                        /* 种子点无幅度，无从谈起 */

    half = peak * ((float)ALGO_CENTROID_HALF_NUM / (float)ALGO_CENTROID_HALF_DEN);

    /* ⚠ 必须"连通"地扩：一旦邻点低于半高就停，绝不跳过凹口去够更远的强点，
     *   否则会把两个分离的回波并成一个、质心落在中间的谷里。*/
    a = idx;
    while (a > 0U && data[a - 1U] >= half)
        a--;
    b = idx;
    while ((uint32_t)b + 1U < (uint32_t)count && data[b + 1U] >= half)
        b++;

    w = (uint16_t)(b - a + 1U);
    if (w < ALGO_CENTROID_MIN_WIDTH || w > ALGO_CENTROID_MAX_WIDTH)
        return -1.0f;

    /* 权重用 (k - a) 而非 k：把乘法量级从 ~1e8 压到 ~1e6，float 精度足够 */
    for (k = (uint32_t)a; k <= (uint32_t)b; k++)
    {
        sum  += data[k];
        wsum += data[k] * (float)(k - (uint32_t)a);
    }
    if (sum <= 0.0f)
        return -1.0f;

    if (out_lo != NULL) *out_lo = a;
    if (out_hi != NULL) *out_hi = b;

    return (float)a + (wsum / sum);
}

/* ============================================================
 *  公开接口
 * ============================================================ */

uint8_t Algo_FindPeaks(const float *data, uint16_t count,Algo_Peak_t *peaks, uint8_t max_peaks)
{
    uint8_t  peak_count   = 0;
    uint16_t last_peak_idx = 0;
    uint16_t i;
    float    mean, std_dev, threshold;

    if (data == NULL || peaks == NULL || count == 0U || max_peaks == 0U)
        return 0;

    /* 滑动平均滤波（输出到 s_smoothed；⚠ data 不可指向 s_smoothed）*/
    smooth_data(data, s_smoothed, count, ALGO_SMOOTH_WINDOW);

    compute_stats(s_smoothed, count, &mean, &std_dev);
    threshold = mean + ((float)ALGO_THRESHOLD_K_NUM / (float)ALGO_THRESHOLD_K_DEN) * std_dev;

    for (i = 1U; i < count - 1U; i++)
    {
        if (s_smoothed[i] < threshold)
            continue;
        if (s_smoothed[i] < s_smoothed[i - 1U])
            continue;
        if (s_smoothed[i] < s_smoothed[i + 1U])
            continue;

        /* 候选峰：与前峰距离不足则只保留更强者 */
        if (peak_count > 0U && (i - last_peak_idx) < ALGO_PEAK_MIN_DIST)
        {
            if (s_smoothed[i] > s_smoothed[last_peak_idx])
            {
                peaks[peak_count - 1U].index     = i;
                peaks[peak_count - 1U].amplitude = s_smoothed[i];
                peaks[peak_count - 1U].position  = refine_peak(s_smoothed, count, i);
                peaks[peak_count - 1U].raw_value = (uint16_t)data[i];
                last_peak_idx = i;
            }
        }
        else if (peak_count < max_peaks)
        {
            peaks[peak_count].index     = i;
            peaks[peak_count].amplitude = s_smoothed[i];
            peaks[peak_count].position  = refine_peak(s_smoothed, count, i);
            peaks[peak_count].raw_value = (uint16_t)data[i];
            peak_count++;
            last_peak_idx = i;
        }
    }

    return peak_count;
}

uint8_t Algo_FindStrongestPeak(Algo_Peak_t *peak)
{
    Algo_Peak_t peaks[ALGO_MAX_PEAKS];
    uint8_t    n;
    uint8_t    best;
    uint16_t   i;
    uint16_t   count;     /* 本次采样点数（随量程档位 1024~5120）*/
    uint16_t   fixed     = 0U;   /* 本次被插值替换掉的哨兵点数 */
    uint16_t   first_idx = 0U;   /* 最靠前的哨兵下标 */

    if (peak == NULL)
        return 1;

    if (BSP_ADCsamp_IsDone() == 0U)
        return 1;

    extern volatile uint8_t adc_error;
    if (adc_error)
        return 1;

    count = BSP_Range_GetSampleCount();

    /* adc_buf 转 float 存 s_measure_sm 后统一调用 Algo_FindPeaks。
     * ⚠ 哨兵必须在【进平滑/统计之前】剔除：填 0 会在波形上留洞，被平滑摊给邻居、累及峰形；
     * 失败点成片时插值不可信，直接放弃本次（返回 1）。*/
    {
        /* adc_buf 是 volatile：用 (void *) 两步转换去 volatile，避免 "discards qualifiers" 警告 */
        fixed = Algo_FixupSentinelFloat((const uint16_t *)(void *)adc_buf,
                                        s_measure_sm, count, &first_idx);
    }
    if (fixed > ALGO_ADC_FAIL_MAX_POINTS)
    {
        printf("[ALGO] ADC fail points too many (%u > %u), skip this sweep\r\n",
               (unsigned)fixed, (unsigned)ALGO_ADC_FAIL_MAX_POINTS);
        return 1;
    }

    n = Algo_FindPeaks(s_measure_sm, count,
                       peaks, ALGO_MAX_PEAKS);
    if (n == 0U)
        return 1;

    best = 0;
    for (i = 1; i < n; i++)
    {
        if (peaks[i].amplitude > peaks[best].amplitude)
            best = i;
    }

    /* ⚠ 位置判据：失败点落在峰【处或之前】则峰之前的时间轴已断、距离必偏，弃用本次；
     * 全部失败点在峰之后则峰之前时间轴完好、距离仍可信，保留。*/
    if (fixed > 0U && first_idx <= peaks[best].index)
    {
        printf("[ALGO] ADC fail at idx %u <= peak idx %u: time-axis broken, sweep discarded\r\n",
               (unsigned)first_idx, (unsigned)peaks[best].index);
        return 1;
    }

    *peak = peaks[best];
    return 0;
}

void Algo_PrintResult(const Algo_Peak_t *peaks, uint8_t peak_count)
{
    uint8_t i;

    printf("\n===== ADC Peak Detection (%d peaks) =====\n", peak_count);

    for (i = 0; i < peak_count; i++)
    {
        printf("  Peak %d: idx=%u, pos=%.2f, amp=%.1f, raw=%u\n",
               i + 1,
               (unsigned)peaks[i].index,
               peaks[i].position,
               peaks[i].amplitude,
               (unsigned)peaks[i].raw_value);
    }

    if (peak_count == 0U)
    {
        printf("  No peaks above threshold.\n");
    }
}


/* ============================================================
 *  Echo-Discovery 物位测量算法实现
 * ============================================================ */

/* ---- 内部状态 ---- */
static uint16_t s_baseline[ADC_SAMPLE_COUNT_MAX];   /* 虚假回波基线（空罐学习结果）*/
static uint32_t s_baseline_accum[ADC_SAMPLE_COUNT_MAX]; /* 多次平均累加器 */
static uint8_t  s_has_baseline = 0;             /* 1=已学习过基线 */
static uint16_t s_baseline_count = 0;           /* 基线学习时的采样点数；与当前档位不符则基线失效 */
static float    s_dist_per_sample = 0.0f;       /* 距离标定：米/采样点 */
static float    s_zero_offset = 0.0f;           /* 零点偏移：米 */

/* ---- 近区盲区（米）：由 App_Config 的 blindZone 下发，见 Algo_SetBlindZoneM() ----
 * 为什么必须有：基线差分只能消掉「空罐时也存在且稳定」的成分。一旦基线缺失、跨档失效、
 *   或人为关闭学习，发射直耦 / 天线近区固有峰就会以全场最大幅度夺标，报出一个"很近的
 *   假料位"——这比测不到更危险（上位机会当真值用）。故盲区内一律置 0，让它不可能成峰。
 * ⚠ 默认 1.0 m 是硬件指标（不是算法调参项）：配置为 0 时不可关闭，只能退回默认。
 * ⚠ 点数 = meters / s_dist_per_sample，在每次测量时现算 ⇒ K 或标定变了自动跟随，
 *   不会再出现「改 K 后 34 点只剩半米盲区」那种漏同步。 */
#define ALGO_BLINDZONE_DEFAULT_M   1.0f
static float    s_blindzone_m = ALGO_BLINDZONE_DEFAULT_M;

void Algo_FinalizeBaseline(uint8_t avg_count)
{
    uint16_t i;
    uint16_t count;

    if (avg_count == 0U)
        return;

    count = BSP_Range_GetSampleCount();
    for (i = 0; i < count; i++)
    {
        s_baseline[i] = (uint16_t)(s_baseline_accum[i] / (uint32_t)avg_count);
        s_baseline_accum[i] = 0U;  /* 清零，为下次学习准备 */
    }

    s_baseline_count = count;
    s_has_baseline = 1;
}

/* ==========================================================================
 *  空罐基线【多帧学习】会话（2026-10-29）
 *
 *  为什么不再用单帧：单帧基线把「那一帧的噪声」永久钉进基线，而 delta = adc − baseline，
 *  基线每点的误差会 1:1 叠加到差分信号上 ⇒ 一次噪声被固化成之后每帧都在减的偏置。
 *  多帧平均压 √N（N=8 ⇒ 2.83 倍），且基线是「空罐固有背景」这种确定性成分，平均无偏。
 *
 *  ★ 为什么【不用】逐点取最大值：最大值是有偏估计、系统性偏高 ⇒ delta 系统性偏小
 *    ⇒ 真实回波被削弱、灵敏度下降、小回波直接消失。基线要的是「典型值」不是「最坏值」。
 *    真正需要防的不是噪声（平均能压），而是【偶发异常帧】—— 见下面门 2。
 * ==========================================================================*/
#define ALGO_LEARN_TRY_MAX    24U    /* 最多尝试帧数（含被丢弃的），≈13 s 兜底防卡死 */
#define ALGO_LEARN_FRAMES_MIN  4U    /* 至少接受 4 帧才算成功，否则判失败、保留旧基线 */
/* 帧间单点超差门限（LSB）。★2026-11-02 由 200 提到 400：120 s / 107 帧实测，近场拖尾段
 *   （0.66~1.0 m）帧间 σ 达 346 LSB，200 只相当于 0.58σ ⇒ 逐点比较时约六成点都「超差」。
 *   按本函数真实逻辑（TRY_MAX=24、接受满 8 帧）在 24 个起点上复跑：TOL=200 成功率仅 50%
 *   （12/24 起点是「首帧接受、之后 23 帧全拒」），TOL=400 为 100%；400 LSB ≈ 9.8% FS，
 *   对整帧异常仍够用。⚠ 它挡的是整帧异常，不是随机噪声（噪声由多帧平均压）。*/
#define ALGO_LEARN_FRAME_TOL  400
/* 允许超差点占比（%），超过则整帧丢弃。★2026-11-02 由 10 提到 20：换成 TOL=400 后
 *   最差的一帧超差占比约 9.9%，正好卡在 10% 门槛上看运气 ⇒ 留一倍余量。
 *   真异常帧（人走过 / 遮挡 / 振动）是过半数的点都变，20% 照样能挡。*/
#define ALGO_LEARN_FRAME_PCT  20U

static uint8_t  s_learn_active = 0U;
static uint8_t  s_learn_got    = 0U;   /* 已接受帧数 */
static uint8_t  s_learn_try    = 0U;   /* 已尝试帧数（含丢弃）*/
static uint16_t s_learn_count  = 0U;   /* 会话开始时的点数，中途变化即作废 */

uint8_t Algo_LearnIsActive(void) { return s_learn_active; }
uint8_t Algo_LearnProgress(void) { return s_learn_got; }

void Algo_LearnCancel(void)
{
    s_learn_active = 0U;
    s_learn_got    = 0U;
    s_learn_try    = 0U;
    s_learn_count  = 0U;
}

uint8_t Algo_LearnBegin(void)
{
    uint16_t count;

    if (s_learn_active != 0U)
        return 1U;                          /* 正在学：拒绝重入，避免半截累加器被复用 */

    count = BSP_Range_GetSampleCount();
    if ((count == 0U) || (count > ADC_SAMPLE_COUNT_MAX))
        return 2U;                          /* 采样未就绪/点数非法 */

    memset(s_baseline_accum, 0, sizeof(s_baseline_accum));
    s_learn_got    = 0U;
    s_learn_try    = 0U;
    s_learn_count  = count;
    s_learn_active = 1U;
    printf("[ALGO] learn BEGIN: %u frames target (%u pts/frame)\r\n",
           (unsigned)ALGO_LEARN_FRAMES, (unsigned)count);
    return 0U;
}

/* 收尾判据：攒够帧数 / 达到尝试上限。返回 Tick 的对外语义。 */
static uint8_t algo_learn_settle(void)
{
    if (s_learn_got >= ALGO_LEARN_FRAMES)
    {
        Algo_FinalizeBaseline(s_learn_got);
        printf("[ALGO] learn OK: %u/%u frames averaged\r\n",
               (unsigned)s_learn_got, (unsigned)ALGO_LEARN_FRAMES);
        s_learn_active = 0U;
        return 1U;
    }
    if (s_learn_try >= ALGO_LEARN_TRY_MAX)
    {
        if (s_learn_got >= ALGO_LEARN_FRAMES_MIN)
        {
            /* 够用但不完美：接受。少于 MIN 才判失败（宁可慢一点也不要用 1~2 帧的脏基线）*/
            Algo_FinalizeBaseline(s_learn_got);
            printf("[ALGO] learn OK (try limit %u): %u frames averaged\r\n",
                   (unsigned)ALGO_LEARN_TRY_MAX, (unsigned)s_learn_got);
            s_learn_active = 0U;
            return 1U;
        }
        printf("[ALGO] learn FAILED: only %u accepted in %u tries (need >= %u) "
               "- old baseline untouched\r\n",
               (unsigned)s_learn_got, (unsigned)s_learn_try,
               (unsigned)ALGO_LEARN_FRAMES_MIN);
        Algo_LearnCancel();
        return 2U;
    }
    return 0U;
}

uint8_t Algo_LearnTick(void)
{
    uint16_t count;
    uint16_t i;
    extern volatile uint8_t adc_error;

    if (s_learn_active == 0U)
        return 3U;                          /* 无会话 */
    if (BSP_ADCsamp_IsDone() == 0U)
        return 0U;                          /* 本拍还没采完，下拍再来 */

    s_learn_try++;

    /* ---- 门 0：点数变了（中途切档）⇒ 累加器已错位，整会话作废 ---- */
    count = BSP_Range_GetSampleCount();
    if (count != s_learn_count)
    {
        printf("[ALGO] learn ABORT: sample count changed %u -> %u (range switched)\r\n",
               (unsigned)s_learn_count, (unsigned)count);
        Algo_LearnCancel();
        return 2U;
    }

    /* ---- 门 1：采样质量。坏帧不计入，但也不判失败（继续等下一帧）---- */
    if (adc_error)
    {
        printf("[ALGO] learn: try %u skipped (adc_error)\r\n", (unsigned)s_learn_try);
        return algo_learn_settle();
    }
    if (Algo_FixupSentinelU16((uint16_t *)(void *)adc_buf, count, (uint16_t *)0) > 0U)
    {
        /* 与单帧学习同判据：出现任何失败点即不用这一帧。
         * 多次平均下坏采样混进累加器后无法区分，会永久带偏平均结果。*/
        printf("[ALGO] learn: try %u skipped (ADC fail points)\r\n", (unsigned)s_learn_try);
        return algo_learn_settle();
    }

    /* ---- 门 2：帧间一致性。与「已累加均值」逐点比，超差点占比过大 ⇒ 整帧丢弃 ----
     * 整帧丢而不是逐点丢：真实干扰（振动/搅拌/SPI 抖动/人走过）通常污染整帧，
     * 逐点保留会把残差钉进基线；而基线本身是平滑包络，不该有孤立尖点。
     * 第一帧无条件接受，作为后续比较的参考。*/
    if (s_learn_got > 0U)
    {
        uint32_t bad = 0U;
        for (i = 0U; i < count; i++)
        {
            int32_t mean = (int32_t)(s_baseline_accum[i] / (uint32_t)s_learn_got);
            int32_t dif  = (int32_t)adc_buf[i] - mean;
            if (dif < 0) dif = -dif;
            if (dif > (int32_t)ALGO_LEARN_FRAME_TOL)
                bad++;
        }
        if ((bad * 100U) > ((uint32_t)count * (uint32_t)ALGO_LEARN_FRAME_PCT))
        {
            printf("[ALGO] learn: try %u REJECTED, %u/%u pts off >%u LSB\r\n",
                   (unsigned)s_learn_try, (unsigned)bad, (unsigned)count,
                   (unsigned)ALGO_LEARN_FRAME_TOL);
            return algo_learn_settle();
        }
    }

    /* ---- 接受本帧 ---- */
    for (i = 0U; i < count; i++)
        s_baseline_accum[i] += adc_buf[i];
    s_learn_got++;

    printf("[ALGO] learn: frame %u/%u accepted\r\n",
           (unsigned)s_learn_got, (unsigned)ALGO_LEARN_FRAMES);

    return algo_learn_settle();
}

uint8_t Algo_HasBaseline(void)
{
    return s_has_baseline;
}

void Algo_ClearBaseline(void)
{
    /* 清除虚假回波基线。清 s_has_baseline 后 Algo_MeasureDistance 走无基线分支（不自动重学，
     * 只能由显示板「服务 → 虚假回波 → 更新/新建」手动触发多帧学习）。
     * ⚠ 若正有一个学习会话在跑，先取消 —— 否则它会继续往累加器里加，最终把基线又写回来。*/
    Algo_LearnCancel();
    s_has_baseline    = 0;
    s_baseline_count  = 0;
    memset(s_baseline, 0, sizeof(s_baseline));
    memset(s_baseline_accum, 0, sizeof(s_baseline_accum));
}

const uint16_t *Algo_GetBaseline(void)
{
    return s_has_baseline ? s_baseline : (const uint16_t *)0;
}

void Algo_SetBaseline(const uint16_t *baseline)
{
    if (baseline == (const uint16_t *)0)
        return;

    memcpy(s_baseline, baseline, sizeof(s_baseline));
    /* ⚠ 不可用 BSP_Range_GetSampleCount()：它返回已快照的 adc_target_count，而本函数在开机
     *   恢复 EEPROM 基线时被调用，采样从未启动、只会拿到初值 1024（第一档），非第一档基线
     *   会被记错点数 → 首次测量即判「档位不符」强制重学、白载。
     *   用 BSP_Range_GetRequestSampleCount()：语义 = 下一次测量序列将采用的点数
     *   （与 BSP_Pulse_Start 的快照同源）。 */
    s_baseline_count = BSP_Range_GetRequestSampleCount();
    s_has_baseline = 1;
}

void Algo_SetCalibration(float dist_per_sample, float zero_offset)
{
    s_dist_per_sample = dist_per_sample;
    s_zero_offset     = zero_offset;
}

void Algo_InitRadarCalibration(float zero_offset)
{
    /* dist_per_sample = RADAR_SPEED_OF_LIGHT / (2·RADAR_TIM7_FREQUENCY) / RADAR_HW_TIME_STRETCH。
     * RADAR_HW_TIME_STRETCH 为硬件时间轴拉伸(放大)比例：前端把极短回波时延拉伸 K 倍后送 ADC，
     * 使 62.5kHz 可分辨米级距离；调整该常量即得到合适的每采样点米数。*/
    s_dist_per_sample = RADAR_SPEED_OF_LIGHT / (2.0f * RADAR_TIM7_FREQUENCY) / RADAR_HW_TIME_STRETCH;
    s_zero_offset     = zero_offset;
}

void Algo_SetBlindZoneM(float meters)
{
    /* NaN 也走默认分支：!(x > 0) 对 NaN 为真 —— 用 x<=0 会漏掉 NaN */
    s_blindzone_m = (meters > 0.0f) ? meters : ALGO_BLINDZONE_DEFAULT_M;
}

float Algo_GetBlindZoneM(void)
{
    return s_blindzone_m;
}

uint8_t Algo_MeasureDistance(Algo_RadarResult_t *result)
{
    Algo_Peak_t peaks[ALGO_MAX_PEAKS];
    uint8_t    n;
    uint8_t    best;
    uint8_t    i;
    uint16_t   j;
    uint16_t   count;     /* 本次采样点数（随量程档位 1024~5120）*/

    if (result == NULL)
        return 4;

    if (BSP_ADCsamp_IsDone() == 0U)
        return 2;

    extern volatile uint8_t adc_error;
    if (adc_error)
        return 3;

    count = BSP_Range_GetSampleCount();

    /* 失败点统计：需在选峰之后按【位置】判决，故提到函数作用域 */
    uint16_t fail_cur  = 0U;      /* 本次采样中的哨兵点数 */
    uint16_t fail_bas  = 0U;      /* 基线上被插值替换掉的哨兵点数 */
    uint16_t first_cur = count;   /* 本次采样中最靠前的哨兵下标（无则 = count）*/
    uint16_t first_bas = count;   /* 基线中最靠前的哨兵下标（无则 = count）*/

/*=========================================================================
 *  数据预处理：空罐基线差分（量产模式）
 *    delta = adc_buf - baseline，负值截零，抑制静态虚假回波。
 *    无基线或档位变化时返回 1，上层据此打 SUPPRESSED、等人工进菜单学基线。
 *  ⚠ 差分必须【双侧对称插值】：只补 cur 会在峰上凿洞，只补 bas 会凭空造出
 *    横跨满量程的假强峰并压过真实回波 —— 故两侧用同一策略补齐，得 d = 插值(cur) - 插值(bas)。
 *  ⚠ 兜底：插值后任一侧仍是哨兵则判 d = 0，绝不让 0xFFFF 进入差分产生 ±60000 级假峰；
 *    失败点成片时插值不可信 → 返回 4 弃用本次测量。
 *=========================================================================*/
    if (s_has_baseline == 0U)
        return 1;                            /* 无基线 */
    if (s_baseline_count != count)
        return 1;                            /* 档位变化：旧基线失效 */

    /*=====================================================================
     *  近区盲区终点（由 blindZone 配置驱动，单位米 → 点数）
     *    为什么必须有：差分只能消掉「空罐时也存在且稳定」的成分。基线缺失、跨档失效、
     *    或人为关闭学习时，发射直耦 / 天线近区固有峰会以全场最大幅度夺标，报出一个
     *    "很近的假料位" —— 这比测不到更危险（上位机会当真值用）。故盲区内一律置 0。
     *    M = s_blindzone_m / s_dist_per_sample，每次现算 ⇒ 改 K 或重新标定自动跟随，
     *    不会再出现「34 点是旧刻度 1 m、改 K 后只剩半米」那种漏同步。
     *    ⚠ 起算点是 tx_sample_offset（常态 0，降级路径才非 0），与距离公式
     *       distance = (pos - tx_offset) × dist_per_sample 同一原点。
     *    ⚠ 先算好再进差分循环：置 0 与统计跳过在同一次遍历里完成，
     *       这样 delta[max] / delta_sum 不含盲区内的直耦峰，日志不会自相矛盾。
     *=====================================================================*/
    uint32_t zero_off  = (uint32_t)tx_sample_offset;   /* 读一次存局部：volatile 多次读可能不一致 */
    uint32_t blind_end = zero_off;
    if (s_dist_per_sample > 1e-9f)
    {
        /* 向上取整（不是四舍五入）：盲区宁可略大、绝不可略小 —— 少屏蔽半格就等于把直耦峰放回来。
         * 实测 1.00 m / 0.0150341 = 66.515 点 ⇒ 取 67 点（0.992 m 不足 1 m，故必须进位）。*/
        float    pts_f = s_blindzone_m / s_dist_per_sample;
        uint32_t pts   = (uint32_t)pts_f;
        if (pts_f > (float)pts)
            pts++;
        blind_end += pts;
        if (blind_end > (uint32_t)count)
            blind_end = (uint32_t)count;
    }
    else
    {
        printf("[ALGO] blind: dist_per_sample not calibrated, blind zone DISABLED\r\n");
    }

    {
        float    dmax = 0.0f; uint16_t dmax_idx = 0; float dsum = 0.0f;

        /* 两侧各自独立插值，让 d = 插值(cur) - 插值(bas)。
         * ⚠ 传 s_baseline 地址是有意的：顺手写回、清掉残留的哨兵污染，免每周期重复处理。*/
        fail_cur = Algo_FixupSentinelU16((uint16_t *)(void *)adc_buf, count, &first_cur);
        fail_bas = Algo_FixupSentinelU16(s_baseline, count, &first_bas);

        /* ⚠ 数量硬门槛（不论位置）：失败点多至此说明 SPI 持续恶化，无条件弃用；主判据在选峰之后 */
        if (fail_cur > ALGO_ADC_FAIL_MAX_POINTS)
        {
            printf("[ALGO] measure: ADC fail points too many (%u > %u), sweep discarded\r\n",
                   (unsigned)fail_cur, (unsigned)ALGO_ADC_FAIL_MAX_POINTS);
            result->position   = 0.0f;
            result->delta_amp  = 0.0f;
            result->distance   = 0.0f;
            result->peak_count = 0;
            return 4;
        }
        if (fail_bas > ALGO_ADC_FAIL_MAX_POINTS)
        {
            /* 基线损坏到无法插值：丢弃基线、返回 1 重学（基线不可信时差分无意义）*/
            printf("[ALGO] measure: baseline unusable (%u fail points), re-learn forced\r\n",
                   (unsigned)fail_bas);
            Algo_ClearBaseline();
            return 1;
        }

        for (j = 0; j < count; j++)
        {
            uint16_t cur;
            uint16_t bas;
            int32_t d;

            /* 近区盲区：置 0 并跳过统计（不参与 dmax/dsum，日志里的 delta[max] 就不会来自直耦峰）*/
            if ((uint32_t)j < blind_end)
            {
                s_measure_sm[j] = 0.0f;
                continue;
            }

            cur = adc_buf[j];
            bas = s_baseline[j];
            /* 兜底：插值后仍残留哨兵则判 0，避免 0xFFFF 进入差分产生假峰 */
            if (ALGO_IS_ADC_FAIL(cur) || ALGO_IS_ADC_FAIL(bas))
                d = 0;
            else
                d = (int32_t)cur - (int32_t)bas;
            s_measure_sm[j] = (d > 0) ? (float)d : 0.0f;
            if (s_measure_sm[j] > dmax) { dmax = s_measure_sm[j]; dmax_idx = j; }
            dsum += s_measure_sm[j];
        }
        printf("[ALGO] count=%u delta[max]=%.0f@%u delta_sum=%.0f (baseline diff, fixup cur/bas=%u/%u)\r\n",
               (unsigned)count, dmax, (unsigned)dmax_idx, dsum,
               (unsigned)fail_cur, (unsigned)fail_bas);
    }

    /* 盲区已在上面差分时置 0（未参与 dmax/dsum），这里只做日志与异常提示。
     * ⚠ 置 0 会略微拉低后面统计的均值/σ（阈值更灵敏），属可接受副作用。*/
    if (blind_end > zero_off)
    {
        printf("[ALGO] blind: muted [%u,%u) = %.3f m (cfg %.2f m)\r\n",
               (unsigned)zero_off, (unsigned)blind_end,
               (double)((float)(blind_end - zero_off) * s_dist_per_sample),
               (double)s_blindzone_m);

        /* 盲区 ≥ 整帧点数 ⇒ 全部置 0，必定「无回波」。这属于配置错误，明说不让现场猜。*/
        if (blind_end >= (uint32_t)count)
        {
            printf("[ALGO] blind: WARN cfg %.2f m covers the whole sweep (%u pts), no echo possible\r\n",
                   (double)s_blindzone_m, (unsigned)count);
        }
    }

    /* 统一调用 Algo_FindPeaks（内部完成平滑→统计→阈值→寻峰→插值）*/
    n = Algo_FindPeaks(s_measure_sm, count, peaks, ALGO_MAX_PEAKS);
    if (n == 0U)
    {
        result->position   = 0.0f;
        result->delta_amp  = 0.0f;
        result->distance   = 0.0f;
        result->peak_count = 0;
        return 4;
    }

    best = 0;
    for (i = 1; i < n; i++)
    {
        if (peaks[i].amplitude > peaks[best].amplitude)
            best = i;
    }

/*=========================================================================
 *  位置主判据 —— 必须放在【选峰之后】（要拿峰下标当参照）
 *   失败点落在【峰处或峰之前】：该点之后的下标↔时间已错位，峰被记在偏早的下标上
 *   → 距离少算，且过程【静默】（adc_error 只在连续失败 3 次时才置位）。
 *   ⚠ 宁可丢这次测量，也不报一个错的距离；失败点全在峰之后则距离仍可信 → 保留。
 *=========================================================================*/
    if ((fail_cur > 0U && first_cur <= peaks[best].index) ||
        (fail_bas > 0U && first_bas <= peaks[best].index))
    {
        /* 打印中 cur@/bas@ 为该侧最靠前的失败点下标；该侧无失败点时是默认填充的 count，不参与判据 */
        printf("[ALGO] measure: ADC fail at/before peak "
               "(cur n=%u first@%u | bas n=%u first@%u | peak@%u), "
               "time-axis broken, sweep discarded\r\n",
               (unsigned)fail_cur, (unsigned)first_cur,
               (unsigned)fail_bas, (unsigned)first_bas,
               (unsigned)peaks[best].index);
        result->position   = 0.0f;
        result->delta_amp  = 0.0f;
        result->distance   = 0.0f;
        result->peak_count = 0;
        return 4;
    }
    else if (fail_cur > 0U || fail_bas > 0U)
    {
/*=========================================================================
 *  保留分支：失败点全部落在【峰之后】
 *   距离可信（只取 peak_idx 与 tx_offset，均在失败点之前）；
 *   ⚠ 但回波曲线后段水平畸变（Disp_SendPackEchoEx 直读 adc_buf，失败点之后下标↔时间前移）。
 *   此处不返回 4：不牺牲主交付物"距离"去换曲线美观，改由打印提示。
 *=========================================================================*/
        printf("[ALGO] measure: ADC fail AFTER peak "
               "(cur n=%u first@%u | bas n=%u first@%u | peak@%u): "
               "distance still valid, but echo-curve TAIL is time-shifted "
               "(~10 samples) - visual distortion only\r\n",
               (unsigned)fail_cur, (unsigned)first_cur,
               (unsigned)fail_bas, (unsigned)first_bas,
               (unsigned)peaks[best].index);
    }
    else
    {
        /* 无失败点，正常路径，无需提示 */
    }

    /* 计算距离：测距起点 t₀ = S6 下降沿（tx_sample_offset，常态恒 0），回波时刻 = 峰值位置，
     * 距离 = (position - t₀) × dist_per_sample
     * ⚠ 用上面存下的 zero_off 而不是再读一次 tx_sample_offset：与盲区屏蔽同一原点，
     *   避免两次读 volatile 之间被改写导致"屏蔽按 A 原点、距离按 B 原点"。
     *
     * ★ 峰位用【半高宽连通区质心】，不再是"全场最大值那一点"（见 Algo_CentroidHalfRegion）。
     *   质心不可用时（连通区退化/过宽/除零）自动退回 refine_peak，行为与改动前一致。 */
    {
        float    pos = peaks[best].position;      /* 默认：原抛物线插值峰位 */
        uint16_t lo  = 0U, hi = 0U;
        float    c   = Algo_CentroidHalfRegion(s_smoothed, count, peaks[best].index, &lo, &hi);

        if (c >= 0.0f)
            pos = c;

        printf("[ALGO] peak pos=%.2f (centroid %s, idx=%u region=[%u,%u] w=%u)\r\n",
               (double)pos, (c >= 0.0f) ? "on" : "FALLBACK",
               (unsigned)peaks[best].index, (unsigned)lo, (unsigned)hi,
               (unsigned)(hi - lo + 1U));

        result->position   = pos;
        result->delta_amp  = peaks[best].amplitude;
        result->peak_count = n;

        if (tx_sample_valid)
        {
            result->distance = (pos - (float)zero_off) * s_dist_per_sample;
        }
        else
        {
            /* 未捕获到测距起点：退回零偏模式（需标定 zero_offset）*/
            result->distance = pos * s_dist_per_sample + s_zero_offset;
        }
    }

    return 0;
}

void Algo_PrintBaseline(void)
{
    uint16_t i;

    if (s_has_baseline == 0U)
    {
        printf("\n[Baseline] Not learned yet.\n");
        return;
    }

    printf("\n===== Baseline (Empty Tank Echo) =====\n");
    printf("  Count: %u points\n", (unsigned)s_baseline_count);

    /* 每隔 50 个点打印一次，避免串口刷屏 */
    for (i = 0U; i < s_baseline_count; i += 50U)
    {
        printf("  [%4u] %4u", (unsigned)i, (unsigned)s_baseline[i]);
        if (i + 50U < s_baseline_count)
            printf("  [%4u] %4u", (unsigned)(i + 25U), (unsigned)s_baseline[i + 25U]);
        printf("\n");
    }

    printf("  Calibration: dist_per_sample=%.4f m, zero_offset=%.2f m\n",
           s_dist_per_sample, s_zero_offset);
}
