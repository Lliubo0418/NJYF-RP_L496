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

/* ============================================================
 * ★ADC 失败哨兵 0xFFFF —— 算法层必须剔除（§3.10 #C3，2026-09-20 修复）
 *
 * 背景：bsp_timer.c:585 在 ADCS7476 读取失败时，把 0xFFFF 写进 adc_buf
 *       作为"此点无效"的标记（注释自述"供算法层识别并跳过"）。
 *       但修复前【算法层从未识别它】—— 全 app_algorithm.c 无 0xFFFF 判定。
 *
 * 危害（不是"略微失真"，是"直接测不出"）：
 *   ① 阈值爆掉：真实回波是 12 位值(0~4095)。哨兵 65535 ≈ 满量程的 16 倍。
 *      compute_stats 把它算进 mean/σ → σ 被抬到数千。
 *      实测估算（1024 点，干净 σ≈35）：混入 1 个哨兵 → σ≈1985、阈值≈8018，
 *      【已高于 12 位满量程 4095】⟹ 所有真实峰都不达阈值，
 *       Algo_FindPeaks 返回 0 → Algo_MeasureDistance 返回 4 → 本次测量直接失败。
 *   ② 污染基线（更严重）：Algo_LearnEmptyTank / Algo_AccumulateBaseline / 
 *      Algo_FinalizeBaseline 三处都是【逐点原样拷贝 adc_buf】。若学习空罐那一次
 *      采样含哨兵，0xFFFF 会被【永久烧进 s_baseline】。
 *      此后 Algo_MeasureDistance 做差分时：
 *        d = (int32_t)adc_buf[j] - (int32_t)s_baseline[j] = 正常值 - 65535 ≈ -61440
 *      ⟹ 该点在【每一个后续测量周期】都恒为巨大负值、被截零，
 *         等于永久性丢失一个采样点；若哨兵落在真实回波峰上，
 *         该峰被永久扣掉 → 持续性误测（"重新学习基线"也救不回来，
 *         因为重学时若再次失败又会写进去）。
 *   ③ 冒充极强峰：smooth_data 会把 65535 平均给左右邻居，
 *      peaks[].raw_value 也会记成 65535 —— 即使侥幸不爆阈值，
 *      它也会压过真实回波被选成"最强峰"（选峰逻辑是取 amplitude 最大者）。
 *
 * 为什么必须在【转 float / 存基线】时剔除，而不是"在阈值比较处跳过"：
 *   滑动平均会把哨兵摊到窗口内所有邻居上；只在比较处跳过，邻居已被污染，
 *   且 raw_value 仍是 65535。必须让哨兵【在进入任何统计/滤波之前】就消失。
 *
 * ★★★ 决定性前提：一次 ADC 失败 ≠ 一个坏点，而是【一段时间轴断裂】★★★
 *   测距靠的是【数组下标 ↔ 时间】的线性对应（distance = (peak_idx - tx_offset)
 *   × dist_per_sample，1 点 = 30.07mm）。这条对应关系只在 TIM7 每个周期
 *   各采 1 点时才成立。而失败路径会破坏它（逐条核对过，非推测）：
 *     ① TIM7 周期 16μs：PSC=40-1 / ARR=32-1，80MHz/1280 = 62.5kHz（tim.c:228-230）；
 *     ② 失败来自 ADCS7476_Read 的 RXNE/BSY 超时，超时是 2000 次循环
 *        （adcs7476.c:31 / :53）；本工程 -O1（uvprojx:318，adcs7476.c 无文件级
 *        覆盖）⇒ 单次约 160μs（保守取 150~250μs）≈ 【10~16 个 TIM7 周期】；
 *     ③ 这 160μs 全在 TIM7 的 ISR 里。TIM7 照常溢出，但 UIF 是【单标志】——
 *        期间溢出的十几次只留下 1 个 pending；HAL 在进回调前就清了 UIF；
 *     ④ 每次 ISR 只写 1 个 adc_buf 槽位、adc_sample_count 只 +1。
 *   ⟹ 一次失败白白吃掉 ~10 个采样周期的时间，却只前进 1 个下标。
 *     该点之后的所有下标都比"理想时刻"晚了 ~160μs，
 *     若失败发生在【发射时刻 tx_offset 之后、回波峰之前】，
 *     峰就会被记在比真实位置早 ~10 个点的下标上 →
 *     距离【少算 ~10 × 30.07mm ≈ 30cm】。
 *   ⟹ 且这是【静默错误】：adc_error 只在连续失败 3 次时才置位，
 *     孤立的 1~2 次失败不会产生任何告警，测量照样返回 0（成功），
 *     只是给了一个错的距离。这比"测不出来"危险得多。
 *
 * 由此得出的策略（2026-09-20 二次修订）：
 *   方案 A（邻域线性插值）【只能补"值"，补不了"时间"】——
 *     它把哨兵点用左右有效点连线填充，消除波形上的凹陷
 *     （填 0 会让 W=7 的平滑把这个凹陷摊给左右各 3 个邻居，落在峰上会使峰
 *      变矮/变宽/被劈成两个，再经 PEAK_MIN_DIST=10 去重可能选错峰或检不到）。
 *     但它【不能】修复上面那条时间轴断裂。
 *     ⚠ 因此 A 绝不能单独作为"让这次测量照常成功"的依据——
 *       那等于把一个必然偏 ~30cm 的结果包装成一次成功测量。
 *   ⟹ A 在本文件中的定位降为：① 显示/统计用的波形修补；
 *      ② 供"失败点全部位于回波峰之后"这类安全情形下复用。
 *
 *   方案 B（弃用）才是唯一正确的兜底，且判据要按【位置】而不是【数量】：
 *     · 失败点全部在【回波峰之后】→ 峰之前的时间轴完好 → 距离仍可信 → 保留；
 *     · 只要有失败点落在【回波峰处或之前】→ 时间轴在峰之前已断 → 必错 → 弃用；
 *     · 基线学习/累加没有"峰"可参照，且基线必须全程时间准确
 *       → 出现【任何】一个失败点就整次不学/不累加。
 *     · 另设 ALGO_ADC_FAIL_MAX_POINTS 作为"SPI 正在恶化"的硬门槛：
 *       失败点总数超过它（不论位置）一律弃用。
 *
 * ★BSP 层决策记录（2026-09-20，用户拍板 →【方案甲】）：
 *   bsp_timer.h 的 ADC_FAIL_MAX 【维持 3，不改】。三个候选：
 *     甲（已采用）：维持 3 —— 由本文件的位置判据来做精确弃用。
 *         优点：失败点落在回波峰【之后】时，这次采样仍可用，可用率最高；
 *               且"连续 3 次才置 adc_error"这个粗判据继续留着兜底。
 *         代价：坏采样要采完才被算法层丢弃，多花一点时间（无功能影响）。
 *     乙：降到 1 —— 首次失败即停 TIM7 + 置 adc_error。
 *         否决理由：本层的【位置】判据已能给出比它精确得多的结论，
 *                   降到 1 只是用"更低可用率"换一个本层已经提供的信号。
 *     丙：失败时把被吃掉的周期数补进 adc_sample_count，从根本上重新线性。
 *         否决理由：最彻底，但要改 16μs 的 TIM7 ISR，风险量级不同，不在本期。
 *   ⟹ 甲方案下【唯一遗留副作用】：失败点【之后】的时间轴仍然错位。
 *     不影响距离（只取 peak_idx 与 tx_offset，都在失败点之前），
 *     但会让【回波曲线后段】水平畸变（Disp_BuildEchoEx 直接读 adc_buf）。
 *     本文件在"保留分支"里打印提示行显式暴露（见 §3.6）。
 *
 * 【哨兵的全部读取点（改哨兵值时必须同步改这些）】
 *   app_algorithm.c  Algo_FixupSentinelU16   定义处（本文件，唯一的插值实现）
 *   app_algorithm.c  Algo_FindStrongestPeak  转换 float 前先插值（无基线降级路径）
 *   app_algorithm.c  Algo_LearnEmptyTank     进基线前先插值；失败过密则整次不学
 *   app_algorithm.c  Algo_AccumulateBaseline 累加前先插值；失败过密则整次不累加
 *   app_algorithm.c  Algo_MeasureDistance    cur/bas 各自插值后再差分；过密则返回 4；
 *                                           §3.5 位置判据弃用 / §3.6 保留分支打印
 *   app_disp.c       Disp_BuildCurveNorm     按 0 处理（画图专用，不参与测距，保持轻量）
 *   app_disp.c       Disp_BuildEchoEx        → 内部转调 Disp_BuildCurveNorm，同上
 *   ⚠ 上面两个 app_disp 的读取点是【显示用】的，故意与算法层策略不同：
 *     它们不插值、直接按 0 画，因为"这里没采到数"在屏幕上是可接受的，
 *     且可避免在显示路径上再分配一份 N 点缓冲。若今后要求曲线也连续，
 *     可复用 Algo_FixupSentinelU16，但需注意它写的是 uint16 数组。
 *   ⟹ 新增任何读 adc_buf 的代码，都请加进这个清单。
 *
 * 【adc_buf 有两个写入口，语义不同 —— 务必知道】
 *   ① bsp_timer.c:607（阻塞版，现行生效）：失败时写 0xFFFFU 哨兵 ← 本块针对它。
 *   ② bsp_spi.c:55（DMA 版，HAL_SPI_RxCpltCallback）：`&= 0x0FFFU`，
 *      【只掩码、不写哨兵】。该路径当前不可达（bsp_timer.c:624 #if 0，
 *      唯一置 adc_dma_busy=1 的语句也在其中 ⇒ 回调在入口即返回）。
 *      ⚠ 若将来恢复 DMA 路径，必须在失败分支补写 0xFFFFU，
 *        否则本文件与 app_disp.c 的全部剔除逻辑对 DMA 采到的失败点【完全无效】。
 * ============================================================ */
#define ALGO_ADC_FAIL_SENTINEL  0xFFFFU
#define ALGO_IS_ADC_FAIL(v)     ((uint16_t)(v) == ALGO_ADC_FAIL_SENTINEL)

/* 方案 B 的【数量】硬门槛：失败点总数超过它 → 不论位置一律弃用。
 * 定位：这是"SPI 正在恶化"的兜底闸门，不是主判据（主判据是【位置】，见上）。
 * 取值 8 的理由：占到这个量级说明已不是偶发毛刺，而是总线在持续出问题，
 * 继续采信风险远大于丢一次测量的代价。
 * ⚠ 若日后放宽/收紧，请同时回看文件上方 A/B 说明与报告 §3.10c。 */
#define ALGO_ADC_FAIL_MAX_POINTS  8U

/* ★一个结构性事实（决定了"按段长判据"没有意义，故不设段长阈值）：
 * bsp_timer.c 的 adc_fail_count 是【连续】计数、成功即清零，
 * 且 ++adc_fail_count >= ADC_FAIL_MAX(=3) 就停 TIM7 并置 adc_error。
 * ⇒ 凡是能走到算法层的采样（四个入口都先判 adc_error 才处理），
 *   其【连续】失败段长度恒 ≤ 2 —— 第 3 个连续失败根本不会产生，
 *   它已经把采样中止了。所以"失败成片"在结构上不成立，
 *   真实形态只有"孤立的单点或相邻两点"，判据必须按【位置】而非【段长】。*/

/**
  * @brief  哨兵剔除 v2（方案 A）：把 buf 中的 0xFFFF 段用左右有效点线性插值填充
  * @note   - 就地修改 buf（原地插值，不额外分配 N 点缓冲）
  *         - 返回值 = 插值掉的哨兵点总数，供方案 B 的【数量】硬门槛使用
 *         - *first_idx = 最靠前的哨兵下标；无哨兵时写 count ——
 *           这是方案 B 的【位置】主判据（失败点在峰之前 = 时间轴已断）
 *         - 越界情形：开头/结尾的哨兵段没有单侧邻居 → 用最近的那个有效值
 *           水平外推（保持连续，避免把边界值拉到 0 造成边缘假凹陷）
 *         - 全部无效（整段都是哨兵）→ 一个都不改，返回 count，由调用方弃用
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
            uint16_t left  = 0U;      /* 左侧最近有效点下标 */
            uint16_t right = 0U;      /* 右侧最近有效点下标 */
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
                /* 两侧都有 → 线性插值：span 为跨过的距离 */
                uint16_t span = (uint16_t)(right - left);
                int32_t  v0   = (int32_t)buf[left];
                int32_t  v1   = (int32_t)buf[right];
                uint16_t t;
                for (t = 0U; t < run; t++)
                {
                    /* 第 (t+1) 步落点：v0 + (v1-v0)*(t+1)/span */
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
  * @brief  哨兵剔除 v2（float 版，用于无基线降级路径）
  * @note   与 uint16 版同构，但输出到 dst（源 adc_buf 是 volatile，
  *         直接转 float 后插值可避免改动物理采样缓存）
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

    /* 先转换（哨兵原样带入，下一步统一处理）*/
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
    uint16_t   fixed     = 0U;   /* 本次被插值替换掉的哨兵点数 */
    uint16_t   first_idx = 0U;   /* 最靠前的哨兵下标 */

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

    /* 将 uint16_t adc_buf 转为 float 存入 s_measure_sm，再统一调用 Algo_FindPeaks。
     * ★哨兵剔除（#C3, 方案 A）：0xFFFF 必须在【进入平滑/统计之前】就换成
     *   邻域插值结果。旧的"填 0"策略会在波形上留一个洞，W=7 的平滑把这个洞
     *   摊给左右各 3 个邻居（合计最多 6 个正常点被带偏），落在峰上会使峰变矮/
     *   变宽/被劈成两个，再经 PEAK_MIN_DIST=10 去重后可能选错峰或检不到。
     *   插值则让波形无缝连续，峰形不受影响（详见文件上方 A+B 说明块）。
     * ★方案 B：若失败点成片（数量或单段长度超限），说明采样已不可信，
     *   插值等于凭空连线，直接放弃本次（返回 1，与"无峰"同义）。*/
    /* 见文件上方 A+B 说明：A 只补"值"，位置判据才是主判据，故先记下 fixed/first。*/
    {
        /* adc_buf 是 volatile：与 app_radar.c 的 Disp_BuildEchoEx 调用写法一致，
         * 用 (void *) 两步转换去掉 volatile，避免 "discards qualifiers" 警告。*/
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

    /* 选幅度最大的 */
    best = 0;
    for (i = 1; i < n; i++)
    {
        if (peaks[i].amplitude > peaks[best].amplitude)
            best = i;
    }

    /* ★方案 B 主判据（位置）：失败点落在峰【处或之前】→ 峰之前的时间轴已断，
     *   距离必偏（每次失败约吃掉 10 个采样周期 ≈ 30cm），必须弃用本次。
     *   全部失败点在峰之后 → 峰之前时间轴完好 → 距离仍可信，保留。*/
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

    /* ★哨兵剔除（#C3）：基线学习【不插值、直接整次不学】。
     *   与测量路径不同，这里没有"回波峰"可作位置参照，而且基线是后续所有周期
     *   的差分基准、必须全程时间准确：只要有一个采样点失败，该点之后的
     *   下标↔时间就已经错位（每次失败约吃掉 10 个采样周期），
     *   整条基线的时间轴都不再可信。所以判据取【最严：出现任何失败点即弃用】。
     *   为什么也不能填 0 了事：填 0 会在基线里留下凹点，使此后每周期差分
     *   d = cur - bas 在该点恒偏高，等于制造一个不会消失的固定假小峰。
     *   弃用代价只是一次不学（下一轮再试），远小于基线被污染的代价。*/
    {
        uint16_t fixed = Algo_FixupSentinelU16((uint16_t *)(void *)adc_buf, count,
                                               (uint16_t *)0);
        if (fixed > 0U)
        {
            printf("[ALGO] learn: %u ADC fail point(s), baseline NOT updated\r\n",
                   (unsigned)fixed);
            return;
        }
    }

    for (i = 0; i < count; i++)
    {
        s_baseline[i] = adc_buf[i];
    }

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

    /* ★哨兵剔除（#C3）：与 Algo_LearnEmptyTank 同判据 —— 出现任何失败点即不累加。
     *   多次平均模式下，一次坏采样混进 s_baseline_accum 之后再也区分不出来，
     *   平均结果会被永久带偏，所以这里比测量路径更严。*/
    {
        uint16_t fixed = Algo_FixupSentinelU16((uint16_t *)(void *)adc_buf, count,
                                               (uint16_t *)0);
        if (fixed > 0U)
        {
            printf("[ALGO] accumulate: %u ADC fail point(s), sample NOT accumulated\r\n",
                   (unsigned)fixed);
            return;
        }
    }

    for (i = 0; i < count; i++)
    {
        s_baseline_accum[i] += adc_buf[i];
    }
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

void Algo_ClearBaseline(void)
{
    /* 清除已学习的虚假回波基线（对应飞卓"虚假回波 → 删除"）。
     * 清 s_has_baseline 后，Algo_MeasureDistance 会走无基线分支（不做基线扣除），
     * 下一次测量序列满足条件时可由算法自动重学。 */
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

    if (BSP_ADCsamp_IsDone() == 0U)
        return 2;

    extern volatile uint8_t adc_error;
    if (adc_error)
        return 3;

    count = BSP_Range_GetSampleCount();

    /* 失败点统计（要在选峰之后才能按【位置】判决，故提到函数作用域）*/
    uint16_t fail_cur  = 0U;      /* 本次采样中的哨兵点数 */
    uint16_t fail_bas  = 0U;      /* 基线上修复掉的哨兵点数 */
    uint16_t first_cur = count;   /* 本次采样中最靠前的哨兵下标（无则 = count）*/
    uint16_t first_bas = count;   /* 基线中最靠前的哨兵下标（无则 = count）*/

/*=========================================================================
 *  数据预处理：空罐基线差分（量产模式）
 *    delta = adc_buf - baseline，负值截零，抑制静态虚假回波
 *    无基线（未学习）或档位变化（旧基线失效）时返回 1，
 *    由上层 app_radar 触发 Algo_LearnEmptyTank 自动（重新）学习。
 *
 *  ★哨兵剔除（#C3, 方案 A+B）—— 差分【必须双边对称插值】，理由：
 *    差分是两个数组相减，任一侧有洞都会破坏结果，且两侧故障形态不同：
 *      · 若只把 cur 的哨兵填 0、bas 正常：d = 0 - bas < 0 → 截零，
 *        该点变成"洞"（回波峰被凿掉一块）；
 *      · 若只把 bas 的哨兵填 0、cur 正常：d = cur - 0 ≈ +2000 →
 *        【凭空造出一个横跨满量程的假强峰】，且它必然压过真实回波被选走。
 *    所以两侧都要用同样的插值策略补齐，让 d = 插值(cur) - 插值(bas)，
 *    语义上等价于"该点两边的信号都保持连续"，不会造峰也不会凿洞。
 *    此外仍需保留原判断：基线里若【残留历史哨兵】（旧固件写入、或本次修复前
 *    已落盘的数据），插值会把它当有效端点传播，故最后再兜一层"任一侧是
 *    哨兵则判 0"。正常路径下这一步不会命中（插值已清除）。
 *  ★方案 B：失败点成片时插值不可信 → 返回 4 弃用本次测量（不写任何状态）。
 *=========================================================================*/
    if (s_has_baseline == 0U)
        return 1;                            /* 无基线：返回 1 触发上层 Algo_LearnEmptyTank */
    if (s_baseline_count != count)
        return 1;                            /* 档位变化：旧基线失效，返回 1 重学 */
    {
        float    dmax = 0.0f; uint16_t dmax_idx = 0; float dsum = 0.0f;

        /* 两侧各自独立插值，让 d = 插值(cur) - 插值(bas)。
         * 注意 s_baseline 是【持久状态】，传它的地址进去会把插值结果写回
         * s_baseline 本身 —— 这是有意的：顺手修掉历史上残留的哨兵污染，
         * 否则每个周期都要重复处理同一批坏点。*/
        fail_cur = Algo_FixupSentinelU16((uint16_t *)(void *)adc_buf, count, &first_cur);
        fail_bas = Algo_FixupSentinelU16(s_baseline, count, &first_bas);

        /* 方案 B 的【数量】硬门槛（不论位置）：失败点多到这个程度说明 SPI 正在
         * 持续恶化，已不是偶发毛刺，无条件下判弃用。主判据在下面的选峰之后。*/
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
            /* 基线自身损坏到无法插值 → 丢弃基线，返回 1 让上层重学。
             * 不能继续测：基线不可信时差分结果毫无意义。*/
            printf("[ALGO] measure: baseline unusable (%u fail points), re-learn forced\r\n",
                   (unsigned)fail_bas);
            Algo_ClearBaseline();
            return 1;
        }

        for (j = 0; j < count; j++)
        {
            uint16_t cur = adc_buf[j];
            uint16_t bas = s_baseline[j];
            int32_t d;
            /* 兜底：插值后理论上不该再有哨兵；若仍有（整段全失败等极端情形），
             * 判 0 保守处理，绝不让 0xFFFF 进入差分产生 ±60000 级假峰。*/
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

    /* 统一调用 Algo_FindPeaks（内部 smooth→stats→threshold→localmax→refine）*/
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

/*=========================================================================
 *  3.5 ★方案 B 主判据（位置）—— 必须放在【选峰之后】，因为要拿峰下标当参照
 *   只要失败点落在【峰处或峰之前】，该点之后的 下标↔时间 就已经错位
 *   （一次 ADC 失败约吃掉 10 个 TIM7 周期 ≈ 160μs ≈ 10 个采样点 ≈ 30cm），
 *   峰会被记在偏早的下标上 → 距离少算。这种错是【静默的】：
 *   adc_error 只在连续失败 3 次时才置位，孤立失败不产生任何告警。
 *   ⟹ 宁可丢这次测量，也不能报一个错的距离。
 *   反之，若失败点全部在峰之后，峰之前的时间轴完好，距离仍可信 → 保留。
 *=========================================================================*/
    if ((fail_cur > 0U && first_cur <= peaks[best].index) ||
        (fail_bas > 0U && first_bas <= peaks[best].index))
    {
        /* 打印里的 cur@/bas@ 是【该侧最靠前的失败点下标】；
         * 若该侧 n=0（无失败点），这个值是默认填充的 count，不参与判据。*/
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
 *  3.6 ★保留分支（方案甲 = BSP 层维持 ADC_FAIL_MAX=3 的【唯一遗留副作用】）
 *   能走到这里说明：失败点全部落在【峰之后】。
 *     · 距离【可信】—— 距离只取 peak_idx 与 tx_offset，都在失败点之前，
 *       时间轴错位发生在峰之后，不影响这两个量。
 *     · 但【回波曲线后段会水平畸变】—— Disp_BuildEchoEx 直接读 adc_buf，
 *       失败点之后的下标↔时间整体前移了约 10 个采样点，
 *       表现为曲线尾部被"拉伸/平移"，越靠后偏差越大。
 *   ⟹ 这里【不返回 4】：距离是本次测量的主交付物，牺牲它去换曲线的美观
 *      不划算；改由打印提示行把这件事显式暴露出来，便于台架核对。
 *      若后续发现曲线畸变不可接受，再走方案丙（在 ISR 里把被吃掉的周期数
 *      补进 adc_sample_count，从根本上重新线性）——那要改 16μs 的 ISR，
 *      风险量级不同，不在本期做。
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
