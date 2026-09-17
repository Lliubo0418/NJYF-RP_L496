#ifndef __APP_ALGORITHM_H
#define __APP_ALGORITHM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 *  雷达 ADC 信号处理算法
 *
 *  输入：adc_buf[1024~3072]（62.5kHz 采样，12-bit ADC 值，点数随量程档位变化）
 *  处理流程：
 *    1. 滑动平均滤波（去高频噪声）
 *    2. 统计自适应阈值（mean + k*std，抑制噪声底）
 *    3. 局部极大值检测（找峰值）
 *    4. 抛物线插值精化（亚采样精度定位）
 *  输出：峰值位置 + 幅度 → 对应目标距离 + 信号强度
 * ============================================================ */

/* 滤波与检测参数（可按实际信号特征调整）*/
#define ALGO_SMOOTH_WINDOW     7U    /* 滑动平均窗口（奇数，越大越平滑但峰越钝）*/
#define ALGO_PEAK_MIN_DIST     10U   /* 相邻峰最小间距（避免重复检同一个峰）*/
#define ALGO_THRESHOLD_K_NUM   3U   /* 阈值系数分子：mean + (K_NUM/K_DEN)*std */
#define ALGO_THRESHOLD_K_DEN    1U   /* 阈值系数分母 */
#define ALGO_MAX_PEAKS         8U   /* 最多返回的峰值个数 */

/* 峰值结果 */
typedef struct {
    uint16_t index;       /* 峰在数组中的整数下标（0~3071）*/
    float    position;    /* 插值后的亚采样位置（更精确）*/
    float    amplitude;   /* 峰处滤波后的幅值 */
    uint16_t raw_value;   /* 峰处原始 ADC 值 */
} Algo_Peak_t;

/**
  * @brief  从浮点数据中找峰值（统一接口，ADC 原始值或 delta 差分信号均可）
  * @param  data : 输入 float 序列（ADC 原始值或 delta 差分），不可指向 s_smoothed
  * @param  count : 数据点数
  * @param  peaks : 输出峰值数组（调用者分配）
  * @param  max_peaks : peaks 数组容量
  * @retval 检测到的峰个数（0 = 未检测到有意义的峰）
  * @note   内部使用 s_smoothed 做平滑输出缓冲，输入 data 不可指向 s_smoothed
  */

uint8_t Algo_FindPeaks(const float *data, uint16_t count,
                       Algo_Peak_t *peaks, uint8_t max_peaks);

/**
  * @brief  从 adc_buf 找最强峰：内部转 float 后调用 Algo_FindPeaks，选幅度最大者
  * @param  peak : [out] 输出最强峰信息
  * @retval 0 = 成功, 1 = 未找到（采样未完成/出错/信号太弱）
  */

uint8_t Algo_FindStrongestPeak(Algo_Peak_t *peak);

/**
  * @brief  打印检测结果（经 printf → UART4 调试口）
  * @param  peaks : 峰值数组
  * @param  peak_count : 峰个数
  */

void Algo_PrintResult(const Algo_Peak_t *peaks, uint8_t peak_count);


/* ============================================================
 *  Echo-Discovery 雷达物位测量算法
 *
 *  雷达时序：
 *    1. S6 输入捕获测低电平脉宽，满足 22-23ms 区间后延时 15ms 启动采样
 *    2. ADC 采样窗口 = 目标点数 × 16μs，采样期间遇 S6 下降沿 = 发射时刻 t₀  
 *    3. 记录发射时刻对应的 ADC 采样点序号 tx_sample_offset
 *    4. 回波到达时刻 = 峰值在采样数组中的位置 peak_position
 *
 *  距离公式（TOF 模式，tx_sample_valid=1）：
 *    distance = (peak_position - tx_sample_offset) × dist_per_sample
 *
 *  距离公式（零偏模式，tx_sample_valid=0，需标定 zero_offset）：
 *    distance = peak_position × dist_per_sample + zero_offset
 *
 *  其中：
 *    c = 299792458.0f m/s（光速）
 *    TIM7_FREQ = 62500 Hz（周期 16μs）
 *    硬件放大(时间轴拉伸)比例 K = RADAR_HW_TIME_STRETCH
 *    dist_per_sample = c / (2 * TIM7_FREQ) / K = 2400 / K m/点
 *    （2400 m/点是 K=1 的未缩放参考值；实际每采样点米数由硬件放大比例 K 决定，
 *     待硬件实测标定。调整 RADAR_HW_TIME_STRETCH 即可得到合适的每采样点米数。）
 * ============================================================ */

/* 雷达物理常数 */
#define RADAR_SPEED_OF_LIGHT    299792458.0f      /* 光速 m/s */
#define RADAR_TIM7_FREQUENCY    62500.0f    /* TIM7 采样频率 Hz（80MHz / 40 / 32）*/

/* 硬件放大(时间轴拉伸)比例 K（即用户所说的硬件放大比例）：
 * 射频/模拟前端把极短的回波时延拉伸 K 倍后再送 ADC 采样，使 62.5kHz 采样可分辨米级距离。
 * 实际 米/采样点 = (c / (2 * TIM7_FREQ)) / K = 2400 / K。
 * K 由硬件实测标定，暂置 1.0f（未放大，等效 2400 m/采样点）。
 * 后续只需调整此值即可得到合适的每采样点米数。 */
#define RADAR_HW_TIME_STRETCH    79763.0f

/* 距离标定参数（由应用层设置）*/
typedef struct {
    float dist_per_sample;  /* 每个采样点对应的距离（米）= (c/(2*f_tim7)) / K，K=硬件放大比例(见 RADAR_HW_TIME_STRETCH) */
    float zero_offset;      /* 发射时刻(t=0)到 adc_buf[0] 对应的距离偏移（米），需实测标定 */
} Algo_Calibration_t;

/* 雷达测量结果 */
typedef struct {
    float    position;    /* 峰在差分信号中的亚采样位置 */
    float    delta_amp;   /* 差分信号在峰处的幅度（当前-基线）*/
    float    distance;    /* 计算得到的距离（米）*/
    uint8_t  peak_count;  /* 差分信号中检到的峰总数 */
} Algo_RadarResult_t;

/**
  * @brief  空罐学习：将当前 adc_buf 存为虚假回波基线（单次模式）
  * @note   调用前需确保 BSP_ADCsamp_IsDone()=1（采样完成） 若要多次平均降噪，先调 Algo_AccumulateBaseline 再调 Algo_FinalizeBaseline
  * @note   调用前需确保采样完成且无错误
  */

void Algo_LearnEmptyTank(void);

/**
  * @brief  累加当前 adc_buf 到基线累加器（多次平均模式，配合 FinalizeBaseline 使用）
  * @note   每次调用前需确保采样完成。累加完毕后调 Algo_FinalizeBaseline。
  */

void Algo_AccumulateBaseline(void);

/**
  * @brief  完成基线学习（累加值除以次数 → baseline）
  * @param  avg_count : 累加次数（不可为 0）
  */

void Algo_FinalizeBaseline(uint8_t avg_count);

/**
  * @brief  是否已有基线（空罐学习过）
  * @retval 1=有基线, 0=无
  */

uint8_t Algo_HasBaseline(void);

/**
  * @brief  获取基线数据指针（用于保存到 EEPROM/Flash）
  * @retval 基线数组指针，NULL 表示无基线
  */

const uint16_t *Algo_GetBaseline(void);

/**
  * @brief  从外部存储加载基线（EEPROM/Flash 恢复）
  * @param  baseline : 基线数据指针（ADC_SAMPLE_COUNT_MAX=3072 个 16-bit，按当前档位只用前 N 个）
  */

void Algo_SetBaseline(const uint16_t *baseline);

/**
  * @brief  手动设置距离标定参数
  * @param  dist_per_sample : 每采样点对应距离（米）
  * @param  zero_offset : 零点偏移（米）
  */

void Algo_SetCalibration(float dist_per_sample, float zero_offset);

/**
  * @brief  自动计算并设置 dist_per_sample（基于光速、TIM7 频率与硬件放大比例 K） dist_per_sample = c / (2 * f) / K = (3e8 / (2 * 62500)) / K = 2400 / K m （2400 为 K=1 的未缩放参考值；K 见 RADAR_HW_TIME_STRETCH，由硬件标定）
  * @param  zero_offset : 发射时刻到 adc_buf[0] 的距离偏移（米），需实测标定
  */

void Algo_InitRadarCalibration(float zero_offset);

/**
  * @brief  雷达物位测量：delta差分→Algo_FindPeaks寻峰→选最强峰→TOF测距
  * @param  result : [out] 测量结果（位置、幅度、距离、峰数）
  * @retval 0=成功, 1=无基线, 2=采样未完成, 3=采样出错, 4=未检测到回波
  */

uint8_t Algo_MeasureDistance(Algo_RadarResult_t *result);

/**
  * @brief  打印基线波形到串口（调试用，每隔50点采样打印）
  */

void Algo_PrintBaseline(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_ALGORITHM_H */
