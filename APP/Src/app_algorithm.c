#include "app_algorithm.h"
#include "bsp_timer.h"    /* adc_buf, ADC_SAMPLE_COUNT_MAX, BSP_Range_GetSampleCount, BSP_ADCsamp_IsDone */
#include <math.h>
#include <string.h>
#include <stdio.h>

/* ============================================================
 *  算法原理
 *
 *  1. 滑动平均滤波
 *     对每个点取前后各 W/2 个邻居求平均，压制高频随机噪声。
 *     W=7 时 3σ 噪声约降到 1/√7 ≈ 38%，峰幅度几乎不变。
 *
 *  2. 自适应阈值
 *     统计全段均值 μ 和标准差 σ，阈值 = μ + k·σ。
 *     只有高于此阈值的峰才认为有信号，避免噪声被误判。
 *     k=3 对应 99.7% 置信度（正态噪声几乎不可能越过）。
 *
 *  3. 局部极大值检测
 *     满足 smoothed[i] > threshold 且 ≥ 左右邻居即为候选峰。
 *     与上一个已确认峰间距 < MIN_DIST 时只保留更强者。
 *
 *  4. 抛物线插值
 *     取峰及左右 3 点拟合抛物线，求顶点偏移 → 亚采样位置。
 *     使峰位置精度从 ±1 个采样间隔提升到 ±0.1 左右。
 * ============================================================ */

/* ---- 内部静态缓冲区（按最大量程档 3072 点静态分配，运行时按当前档位数处理）---- */
static float s_smoothed[ADC_SAMPLE_COUNT_MAX];    /* Algo_FindPeaks 滑动平均输出缓冲（3072×4B=12KB）*/
static float s_measure_sm[ADC_SAMPLE_COUNT_MAX];  /* 调用方输入缓冲（ADC 数据或 delta 差分，12KB）*/

/**
  * @brief  滑动平均滤波：对输入序列做奇数窗口滑动平均，抑制高频随机噪声
  * @note   窗口边界处自动收缩（不补零），首尾点参与运算的邻居数 < win
  *         W=7 时 3σ 噪声约降到 1/√7 ≈ 38%，峰幅度几乎不变
  * @param  src   : 输入 float 序列指针（ADC 值或差分信号），不可为 NULL
  * @param  dst   : 输出滤波后 float 数组指针，长度 >= count，调用者保证空间充足
  * @param  count : 数据点数（随量程档位 1024~3072，= BSP_Range_GetSampleCount()）
  * @param  win   : 滑动窗口宽度（必须为奇数，默认 7）；越大越平滑但峰越钝
  * @retval 无（结果写入 dst）
  * @complexity  时间 O(N*W)，空间 O(1) 额外
  */
/* ---- 滑动平均 ---- */
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
  * @note   标准差为总体标准差（除以 N 而非 N-1）；含一次 sqrtf 调用
  *         阈值 = mean + k*std，k=3 对应 99.7% 置信度
  * @param  data : 输入 float 序列指针（通常为滤波后数据）
  * @param  count: 数据点数
  * @param  mean : [out] 输出均值 μ
  * @param  std  : [out] 输出标准差 σ
  * @retval 无（结果写入 *mean 和 *std）
  * @complexity  时间 O(N) 两趟遍历，空间 O(1)
  */
/* ---- 统计均值与标准差 ---- */
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
  * @note   y0,y1,y2 为峰左(idx-1)、峰(idx)、峰右(idx+1)三点值
  *         顶点偏移 δ = (y0 - y2) / (2·(y0 - 2·y1 + y2))
  *         精确位置 = idx + δ，δ 限幅在 [-0.5, +0.5]
  *         使峰位置精度从 ±1 采样间隔提升到 ±0.1 左右
  * @param  data : 滤波后 float 序列指针
  * @param  count: 序列长度，用于边界检查
  * @param  idx  : 局部极大值的整数下标（1 <= idx <= count-2）
  * @retval 精确峰位置 = idx + δ（float）
  * @complexity  时间 O(1)，空间 O(1)
  */
/* ---- 抛物线插值求亚采样峰位 ---- */
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
    /* 限制偏移在 [-0.5, +0.5] */
    if (delta > 0.5f)  delta = 0.5f;
    if (delta < -0.5f) delta = -0.5f;

    return (float)idx + delta;
}

/* ============================================================
 *  公开接口
 * ============================================================ */

uint8_t Algo_FindPeaks(const float *data, uint16_t count,
                       Algo_Peak_t *peaks, uint8_t max_peaks)
{
    uint8_t  peak_count   = 0;
    uint16_t last_peak_idx = 0;
    uint16_t i;
    float    mean, std_dev, threshold;

    if (data == NULL || peaks == NULL || count == 0U || max_peaks == 0U)
        return 0;

    /* 1. 滑动平均滤波（输出到 s_smoothed，data 不可指向 s_smoothed）*/
    smooth_data(data, s_smoothed, count, ALGO_SMOOTH_WINDOW);

    /* 2. 自适应阈值 */
    compute_stats(s_smoothed, count, &mean, &std_dev);
    threshold = mean + ((float)ALGO_THRESHOLD_K_NUM / (float)ALGO_THRESHOLD_K_DEN) * std_dev;

    /* 3. 局部极大值检测 */
    for (i = 1U; i < count - 1U; i++)
    {
        if (s_smoothed[i] < threshold)
            continue;
        if (s_smoothed[i] < s_smoothed[i - 1U])
            continue;
        if (s_smoothed[i] < s_smoothed[i + 1U])
            continue;

        /* 候选峰：检查与前一个峰的距离 */
        if (peak_count > 0U && (i - last_peak_idx) < ALGO_PEAK_MIN_DIST)
        {
            /* 太近：只保留更强的那个 */
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
    uint16_t   count;     /* 本次采样点数（随量程档位 1024~3072）*/

    if (peak == NULL)
        return 1;

    /* 采样未完成 */
    if (BSP_ADCsamp_IsDone() == 0U)
        return 1;

    /* 采样出错（adc_error） */
    extern volatile uint8_t adc_error;
    if (adc_error)
        return 1;

    /* 本次采样点数随量程档位变化（1024~3072），从 BSP 取生效值 */
    count = BSP_Range_GetSampleCount();

    /* 将 uint16_t adc_buf 转为 float 存入 s_measure_sm，再统一调用 Algo_FindPeaks */
    for (i = 0; i < count; i++)
        s_measure_sm[i] = (float)adc_buf[i];

    n = Algo_FindPeaks(s_measure_sm, count,
                       peaks, ALGO_MAX_PEAKS);
    if (n == 0U)
        return 1;

    /* 选幅度最大的 */
    best = 0;
    for (i = 1; i < n; i++)
    {
        if (peaks[i].amplitude > peaks[best].amplitude)
            best = i;
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
static uint16_t s_baseline[ADC_SAMPLE_COUNT_MAX];   /* 虚假回波基线（空罐学习结果，按最大档分配）*/
static uint32_t s_baseline_accum[ADC_SAMPLE_COUNT_MAX]; /* 多次平均累加器 */
static uint8_t  s_has_baseline = 0;             /* 1=已学习过基线 */
static uint16_t s_baseline_count = 0;           /* 基线学习时的采样点数；与当前档位不符则基线失效 */
static float    s_dist_per_sample = 0.0f;       /* 距离标定：米/采样点 */
static float    s_zero_offset = 0.0f;           /* 零点偏移：米 */

void Algo_LearnEmptyTank(void)
{
    uint16_t i;
    uint16_t count;

    if (BSP_ADCsamp_IsDone() == 0U)
        return;

    extern volatile uint8_t adc_error;
    if (adc_error)
        return;

    count = BSP_Range_GetSampleCount();
    for (i = 0; i < count; i++)
        s_baseline[i] = adc_buf[i];

    s_baseline_count = count;   /* 记录基线对应的档位，切档后 Algo_MeasureDistance 自动重学 */
    s_has_baseline = 1;
}

void Algo_AccumulateBaseline(void)
{
    uint16_t i;
    uint16_t count;

    if (BSP_ADCsamp_IsDone() == 0U)
        return;

    extern volatile uint8_t adc_error;
    if (adc_error)
        return;

    count = BSP_Range_GetSampleCount();
    for (i = 0; i < count; i++)
        s_baseline_accum[i] += adc_buf[i];
}

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

uint8_t Algo_HasBaseline(void)
{
    return s_has_baseline;
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
    s_baseline_count = BSP_Range_GetSampleCount();
    s_has_baseline = 1;
}

void Algo_SetCalibration(float dist_per_sample, float zero_offset)
{
    s_dist_per_sample = dist_per_sample;
    s_zero_offset     = zero_offset;
}

void Algo_InitRadarCalibration(float zero_offset)
{
    /* dist_per_sample = c / (2 * f_tim7) / K
     * = 3e8 m/s / (2 * 62500 Hz) / K = 2400 / K m/点
     * K 为硬件时间轴拉伸(放大)比例(RADAR_HW_TIME_STRETCH)：
     *   射频/模拟前端把极短回波时延拉伸 K 倍后送 ADC，使 62.5kHz 可分辨米级距离。
     * 2400 m/点是 K=1 的未缩放参考值；调整 RADAR_HW_TIME_STRETCH 即得到合适的每采样点米数。 */
    s_dist_per_sample = RADAR_SPEED_OF_LIGHT / (2.0f * RADAR_TIM7_FREQUENCY) / RADAR_HW_TIME_STRETCH;
    s_zero_offset     = zero_offset;
}

uint8_t Algo_MeasureDistance(Algo_RadarResult_t *result)
{
    Algo_Peak_t peaks[ALGO_MAX_PEAKS];
    uint8_t    n;
    uint8_t    best;
    uint8_t    i;
    uint16_t   j;
    uint16_t   count;     /* 本次采样点数（随量程档位 1024~3072）*/

    if (result == (Algo_RadarResult_t *)0)
        return 4;

    if (s_has_baseline == 0U)
        return 1;

    if (BSP_ADCsamp_IsDone() == 0U)
        return 2;

    extern volatile uint8_t adc_error;
    if (adc_error)
        return 3;

    /* 量程档位变化后采样点数改变，旧基线已失效：返回 1 由上层自动重新学习空罐基线 */
    count = BSP_Range_GetSampleCount();
    if (s_baseline_count != count)
        return 1;

    /* 1. 计算差分信号 delta = adc_buf - baseline，负值截零，存入 s_measure_sm */
    for (j = 0; j < count; j++)
    {
        int32_t d = (int32_t)adc_buf[j] - (int32_t)s_baseline[j];
        s_measure_sm[j] = (d > 0) ? (float)d : 0.0f;
    }

    /* 2. 统一调用 Algo_FindPeaks（内部 smooth→stats→threshold→localmax→refine）*/
    n = Algo_FindPeaks(s_measure_sm, count, peaks, ALGO_MAX_PEAKS);
    if (n == 0U)
    {
        result->position   = 0.0f;
        result->delta_amp  = 0.0f;
        result->distance   = 0.0f;
        result->peak_count = 0;
        return 4;
    }

    /* 3. 选幅度最大的峰 */
    best = 0;
    for (i = 1; i < n; i++)
    {
        if (peaks[i].amplitude > peaks[best].amplitude)
            best = i;
    }

    /* 4. 计算距离
     * 发射时刻 = S6 下降沿（采样期间捕获，对应 tx_sample_offset）
     * 回波时刻 = peak_position（峰值在采样数组中的位置）
     * 距离 = (peak_pos - tx_sample_offset) × dist_per_sample
     */
    result->position   = peaks[best].position;
    result->delta_amp  = peaks[best].amplitude;
    result->peak_count = n;

    if (tx_sample_valid)
    {
        result->distance = (peaks[best].position - (float)tx_sample_offset) * s_dist_per_sample;
    }
    else
    {
        /* 未捕获到发射时刻：退回零偏模式（需标定 zero_offset）*/
        result->distance = peaks[best].position * s_dist_per_sample + s_zero_offset;
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
