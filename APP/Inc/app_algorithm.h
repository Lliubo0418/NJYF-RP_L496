#ifndef __APP_ALGORITHM_H
#define __APP_ALGORITHM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============ 雷达 ADC 信号处理算法 ============
 * 输入 adc_buf[1024~5120]（62.5kHz，12-bit，点数随量程档位）。
 * 流程：滑动平均 → 统计自适应阈值(mean+k*std) → 局部极大值寻峰 → 抛物线插值。
 * 输出：峰值位置 + 幅度 → 目标距离 + 信号强度。 */

/* 滤波与检测参数 */
#define ALGO_SMOOTH_WINDOW     7U    /* 滑动平均窗口（必须奇数）*/
#define ALGO_PEAK_MIN_DIST     10U
#define ALGO_THRESHOLD_K_NUM   3U   /* 阈值系数分子：mean + (K_NUM/K_DEN)*std */
#define ALGO_THRESHOLD_K_DEN    1U
#define ALGO_MAX_PEAKS         8U

typedef struct {
    uint16_t index;       /* 峰在数组中的下标 */
    float    position;    /* 抛物线插值后的亚采样位置 */
    float    amplitude;   /* 峰处滤波后的幅值 */
    uint16_t raw_value;   /* 峰处原始 ADC 值 */
} Algo_Peak_t;

/**
  * @brief  从 float 序列中找峰值（ADC 原始值或 delta 差分均可）
  * @param  data : 输入 float 序列（ADC 原始值或 delta 差分）
  * @param  count : 数据点数
  * @param  peaks : 输出峰值数组（调用者分配）
  * @param  max_peaks : peaks 数组容量
  * @retval 检测到的峰个数（0 = 无有意义峰）
  * @note   内部用 s_smoothed 做平滑缓冲 ⇒ data 不可指向 s_smoothed
  */

uint8_t Algo_FindPeaks(const float *data, uint16_t count,
                       Algo_Peak_t *peaks, uint8_t max_peaks);

/**
  * @brief  从 adc_buf 中找最强峰
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


/* ============ Echo-Discovery 雷达物位测量算法 ============
 * 时序（2026-10-14 起：零点由 S6↓ 捕获给出）：测量序列在 S6↑ 启动 → S4↑(S6↑+6.570ms) 起 ADC →
 * 【写入起点】由 TIM2 捕获 S6↓ 实时触发（不再跨周期预测）⇒ adc_buf[0] = 测距起点 t₀，
 * S3↓ = t₀ + N×16μs 收尾；采样窗内的 S6↓ 即 t₀，故 tx_sample_offset 恒为 0（±1 拍），
 * 回波峰位置 peak_position 以 adc_buf 下标计。
 * 距离(TOF, tx_sample_valid=1)：distance = (peak_position - tx_sample_offset) × dist_per_sample
 * 距离(零偏, tx_sample_valid=0)：distance = peak_position × dist_per_sample + zero_offset
 * 其中 dist_per_sample = c/(2·TIM7_FREQ)/K = 2400/K（K = RADAR_HW_TIME_STRETCH）。 */

#define RADAR_SPEED_OF_LIGHT    299792458.0f
#define RADAR_TIM7_FREQUENCY    62500.0f    /* TIM7 采样频率 Hz */

/* 硬件放大(时间轴拉伸)比例 K：前端把回波时延拉伸 K 倍后再送 ADC，使 62.5kHz 采样可分辨米级距离。
 * 每采样点 = (c/(2·TIM7_FREQ))/K = 2398.34/K。
 * ★ K 不是独立标定值：由「S6 周期」决定（K = f_T × P_S6）；周期由 S7/S9 闭环锁在 F_IC 周期窗内。
 * ⚠ 物理硬线（L496时序.md §4）：S6 频率必须 ≥ 22.4382Hz ⇔ 周期 ≤ 44567µs —— 这是 K 准确的前提。
 *   改周期窗上沿或改硬件周期都必须回来核对此常量，否则距离刻度同比例偏（窗宽 100µs ⇒ ±0.11%）。
 * ★ 定案：1 格 = 1.05mm 判为【光程（双程）】⇒ 单程 0.525mm ⇒ K 由 79763 翻倍为 159526，
 *   每采样点 30.068 → 15.034mm。依据：① K 文档 §二自注同此换算；② 方案报告 §6.2 鉴相器取原始
 *   晶振级；③ 实测「卷尺 2.2m ↔ 回波峰点 183」（34 帧质心 183.0±0.9）落在 15.034mm/点上。
 * ⚠ 未验项：只有 1 个卷尺点，第 2 个未做；若复核否证，退回单程解读 K = 79763（30.068mm/点）。
 * ⚠ 与「档位 3→5（3072→5120 点）」配套：只改 K 不改点数会让各档量程腰斩。 */
#define RADAR_HW_TIME_STRETCH    159526.0f

/* 距离标定参数（应用层设置）*/
typedef struct {
    float dist_per_sample;  /* 每采样点距离(米) = 2400/K，K 见 RADAR_HW_TIME_STRETCH */
    float zero_offset;      /* 测距起点(t=0)到 adc_buf[0] 的距离偏移(米)，需实测标定 */
} Algo_Calibration_t;

typedef struct {
    float    position;    /* 峰在差分信号中的亚采样位置 */
    float    delta_amp;   /* 峰处差分幅度（当前-基线）*/
    float    distance;    /* 距离（米）*/
    uint8_t  peak_count;  /* 检到的峰总数 */
} Algo_RadarResult_t;

/* Algo_FinalizeBaseline：累加值 / 帧数 → baseline（多帧学习会话内部调用，见下方说明） */
void Algo_FinalizeBaseline(uint8_t avg_count);

/* ==========================================================================
 *  空罐基线【多帧学习】会话
 *  单帧基线会把「那一帧的噪声」永久钉进基线，基线逐点误差会 1:1 叠加到 delta 上。
 *  多帧平均把噪声压 √N（N=8 时 2.83 倍）；**不用逐点最大值**：最大值系统性偏高
 *  ⇒ delta 系统性偏小 ⇒ 真实小回波被削弱甚至消失。
 *  会话是【异步】的：Begin 只置状态，之后每完成一次采样调一次 Tick，攒够帧数才算完。
 *  绝不能在 SetParam / 主循环里同步等 8 帧 —— 那会阻塞主循环 4.3s，USART 收帧全丢。
 * ==========================================================================*/

/**
  * @brief  目标帧数（8 帧 ≈ 4.3 s @1.87Hz）。想改平滑度就改这个数（4~16 合理）。
  */
#define ALGO_LEARN_FRAMES      8U

/**
  * @brief  开始一次多帧学习会话：清累加器、置状态，立即返回（不阻塞）
  * @retval 0=已启动, 1=已在学习中（拒绝重入）, 2=采样未就绪/点数非法
  */
uint8_t Algo_LearnBegin(void);

/**
  * @brief  学习会话推进一拍：每完成一次 ADC 采样（adc_buf 有效）调用一次
  * @note   内部含两道门：① 采样质量（adc_error / 哨兵点）；② 帧间一致性
  *         （与已累加均值逐点比，超差点占比过大 ⇒ 整帧丢弃，不是逐点丢）
  * @retval 0=进行中, 1=刚完成且基线有效, 2=失败（旧基线保持不变）, 3=当前无会话（空闲）
  */
uint8_t Algo_LearnTick(void);

/**
  * @brief  是否有学习会话在跑
  */
uint8_t Algo_LearnIsActive(void);

/**
  * @brief  已接受的帧数（用于日志/界面进度）
  */
uint8_t Algo_LearnProgress(void);

/**
  * @brief  取消当前学习会话（不触碰已有基线）
  * @note   Algo_ClearBaseline() 内部会自动调用
  */
void Algo_LearnCancel(void);

/**
  * @brief  是否已有基线（空罐学习过）
  * @retval 1=有基线, 0=无
  */

uint8_t Algo_HasBaseline(void);

/**
  * @brief  清除已学习的虚假回波基线（对应飞卓"虚假回波 → 删除"）
  * @note   清零基线与 s_has_baseline；此后测量只报 SUPPRESSED，直到重新学出基线。
  */

void Algo_ClearBaseline(void);

/**
  * @brief  获取基线数据指针（用于保存到 EEPROM/Flash）
  * @retval 基线数组指针，NULL 表示无基线
  */

const uint16_t *Algo_GetBaseline(void);

/**
  * @brief  从外部存储加载基线（EEPROM/Flash 恢复）
  * @param  baseline : 基线数据指针（ADC_SAMPLE_COUNT_MAX 个 16-bit，当前档位只用前 N 个）
  */

void Algo_SetBaseline(const uint16_t *baseline);

/**
  * @brief  手动设置距离标定参数
  * @param  dist_per_sample : 每采样点对应距离（米）
  * @param  zero_offset : 零点偏移（米）
  */

void Algo_SetCalibration(float dist_per_sample, float zero_offset);

/**
  * @brief  自动计算并设置 dist_per_sample = 2400 / K（K 见 RADAR_HW_TIME_STRETCH）
  * @param  zero_offset : 测距起点到 adc_buf[0] 的距离偏移（米），需实测标定
  */

void Algo_InitRadarCalibration(float zero_offset);

/**
  * @brief  设置近区盲区（米）——盲区内的差分值在寻峰前被置 0，不可能被选为回波
  * @param  meters : 盲区距离（米）；<=0 或 NaN 时退回默认 1.0 m（盲区由硬件决定，不允许关掉）
  * @note   点数在每次测量时按 dist_per_sample 现算 ⇒ 改 K / 重新标定后自动跟随，不需重编
  */
void Algo_SetBlindZoneM(float meters);

/**
  * @brief  读取当前生效的盲区（米），供显示/诊断核对
  */
float Algo_GetBlindZoneM(void);

/**
  * @brief  雷达物位测量：差分 → 寻峰 → 选最强峰 → TOF 测距
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
