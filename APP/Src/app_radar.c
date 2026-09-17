#include "app_radar.h"
#include "app_algorithm.h"
#include "app_config.h"

/* 硬件驱动头文件按需引入 */
#include "adcs7476.h"
#include "ad5421.h"
#include "bsp_timer.h"
#include "bsp_usart.h"
#include "app_disp.h"
#include "app_hart.h"
#include <stdio.h>
#include <string.h>

/* F_IC 阈值统一在 bsp_timer.h 定义（目标中心 22875μs，死区 ±500μs）。
 * 注意：阈值曾被误改为 27920/28920（中心 28420）—— 把上电未收敛的实测脉宽
 * ~28.4ms 误当成目标值，已回退为 22375/23375（中心 22875）。
 * 测量序列的启动在 TIM2 S6↑ 捕获 ISR（见 bsp_timer.c），主循环只做校正。 */
#define F_IC_CENTER_US     ((F_IC_LOWER_US + F_IC_UPPER_US) / 2U)

/* 宽脉冲粗调：误差 ≥ F_IC_WIDE_TH_US 时用 S7_S9_WIDE_US 宽脉冲加快上电充电/放电，
 * 否则固定 2μs 精调（可按实际收敛速度调整这两个宏） */
#define F_IC_WIDE_TH_US     400U
#define S7_S9_WIDE_US        20U
#define S7_S9_FINE_US         2U

/* 粗调锁存：宽脉冲(20μs)单批校正量约为样机 2μs 脉冲的 10 倍，误差在
 * 400μs 边界附近易过冲震荡，导致后期脉宽一直是 20μs 不回落。
 * 一旦误差 < F_IC_WIDE_TH_US 即锁存"已进入精调"，此后固定 2μs（对齐样机），
 * 只有上电粗充阶段才允许宽脉冲。复位/重新上电后自动恢复粗调。 */
static uint8_t s_coarse_done = 0U;

/* 偏差分段 */
#define F_IC_SEG1_US         50U
#define F_IC_SEG2_US         200U
#define F_IC_SEG4_US         400U
#define F_IC_SEG8_US         1200U

#define MAX_S7_S9_PULSE      50U





uint32_t pulse_us;       //temp

/**
  * @brief  根据脉宽偏差量分段决定 S7/S9 校正脉冲个数
  * @note   偏差越大脉冲越多，实现非线性收敛：
  *         <50μs→1, <200μs→2, <400μs→4, <1200μs→8, >=1200μs→10
  * @param  error_us : 当前脉宽与目标中心的偏差（微秒）
  * @retval 脉冲个数（1/2/4/8/10）
  * @complexity  时间 O(1) 固定4级比较，空间 O(1)
  */
static uint8_t Radar_GetPulseCount(uint32_t error_us)
{
    if (error_us < F_IC_SEG1_US)
    {
        return 1U;
    }
    if (error_us < F_IC_SEG2_US)
    {
        return 2U;
    }
    if (error_us < F_IC_SEG4_US)
    {
        return 4U;
    }
    if (error_us < F_IC_SEG8_US)
    {
        return 8U;
    }

    return MAX_S7_S9_PULSE;
}

/* ============================================================
 *  采样消费管理 + 滤波/测距接入（仅在本文件内实现）
 * ============================================================ */
static uint8_t            s_sample_consumed = 0U;   /* 当前采样是否已跑过算法 */
static Algo_RadarResult_t s_last_result;            /* 最近一次测量结果，供调试查看 */

/**
  * @brief  对"已采满"的 adc_buf 运行滤波 + 回波发现测距
  * @note   仅当 BSP_ADCsamp_IsDone()==1 时调用（adc_buf 已稳定，无 ISR 竞争）
  *         流程：Algo_MeasureDistance → 处理返回码（无基线则自动学习空罐基线）
  *         → 成功则 printf 调试 + Disp_DownSendMeas 经 USART1 上报显示板
  * @retval 无（结果存入 s_last_result，副作用为串口输出 + 下行帧发送）
  * @complexity  时间 O(N*W)（由 Algo_MeasureDistance 主导），空间 O(N) 栈
  */
static void App_Radar_ProcessSample(void)
{
    uint8_t ret = Algo_MeasureDistance(&s_last_result);

    if (ret == 1U)
    {
        /* 无基线：首次上电以当前采样作为空罐基线，便于直接观察测距。
         * 量产应从 EEPROM/Flash 加载基线（App_Radar_LoadBaseline），
         * 此处仅为调试/验证，下一轮采样即可出距离。*/
        Algo_LearnEmptyTank();
        printf("[RADAR] no baseline -> learned empty-tank baseline; distance from next cycle\r\n");
        return;
    }
    if (ret == 2U)
    {
        printf("[RADAR] sample not done (unexpected)\r\n");
        return;
    }
    if (ret == 3U)
    {
        printf("[RADAR] sample error (adc_error=1)\r\n");
        return;
    }
    if (ret == 4U)
    {
        printf("[RADAR] no echo detected (ret=4)\r\n");
        return;
    }

    /* ret == 0：滤波 + 测距成功 */
    printf("[RADAR] distance=%.3f m | peak_pos=%.2f | delta_amp=%.1f | peaks=%u | mode=%s\r\n",
           s_last_result.distance,
           s_last_result.position,
           s_last_result.delta_amp,
           (unsigned)s_last_result.peak_count,
           (tx_sample_valid ? "TOF" : "calib"));

    /* ===== 应用主板配置：仿真 / 量程盲区限幅 / 阻尼 ===== */
    if (App_Config_IsSim())
    {
        s_last_result.distance = App_Config_SimDistance();
    }
    s_last_result.distance = App_Config_ApplyRange(s_last_result.distance);
    s_last_result.distance = App_Config_ApplyDamping(s_last_result.distance);

    /* 更新 HART 主变量（PV = 物位/距离） */
    App_HART_SetPV(s_last_result.distance);

    /* ===== 协议发送接入点 =====
     * MEAS 已自动下发；回波/诊断已接入。 */
    Disp_DownSendMeas(&s_last_result, (tx_sample_valid ? 1U : 0U));

    /* 回波包络：按比例从 adc_buf 抽点归一化为 128 点下行（R4 修复） */
    {
        static uint8_t echo128[DISP_ECHO_LEN];
        uint16_t n = BSP_Range_GetSampleCount();
        if (n == 0U || n > ADC_SAMPLE_COUNT_MAX) n = ADC_SAMPLE_COUNT_MAX;
        Disp_BuildEcho((const uint16_t *)(void *)adc_buf, n, echo128);
        Disp_DownSendEcho(echo128);
    }

    /* 诊断信息（含传感器温度） */
    {
        uint8_t reliability = (s_last_result.peak_count > 0U) ? 100U : 0U;
        uint8_t status      = (s_last_result.peak_count > 0U) ? 0x01U : 0x00U;
        Disp_DownSendDiag(reliability, status, 0.0f, 0.0f, App_Config_GetSensorTemp());
    }

    /* ===== 4-20mA 输出（R7 修复）：距离 ↔ 电流线性标定 =====
     * lowAdjustVal  → 4mA  (DAC=0)
     * highAdjustVal → 20mA (DAC=65535)
     * 超界钳位；currFault 决定无效测量时的故障电流。 */
    {
        uint16_t dac = 0U;
        if (s_last_result.peak_count > 0U)
        {
            float lo = gRadarConfig.lowAdjustVal;
            float hi = gRadarConfig.highAdjustVal;
            float span = hi - lo;
            float d  = s_last_result.distance;
            if (span <= 0.0f) span = 1.0f;
            if (d <= lo)      dac = 0U;
            else if (d >= hi) dac = 65535U;
            else              dac = (uint16_t)((d - lo) / span * 65535.0f);
        }
        else
        {
            /* 无效测量：故障电流 —— 0=3.6mA(DAC≈0), 1=22.8mA(DAC=65535), 2/3=保持 */
            switch (gRadarConfig.currFault)
            {
                case 0:  dac = 0U;       break;   /* 3.6mA */
                case 1:  dac = 65535U;   break;   /* 22.8mA */
                default: break;                  /* 保持上一次值 */
            }
        }
        AD5421_SetDACOutput(dac);
    }
}

void App_Radar_Init(void)
{
    /* 算法侧标定 + 运行配置初始化。
     * zero_offset 为发射时刻到 adc_buf[0] 的距离偏移（米），需现场标定，
     * 此处先置 0：若 tx_sample_valid 有效则走飞行时间法、不依赖该值。*/
    App_Config_Init();
    Algo_InitRadarCalibration(0.0f);
}

void App_Radar_Run(void)
{
    uint32_t pulse_us;
    uint32_t error_us;
    uint8_t  pulse_count;

    /* ============================================================
     * 滤波 + 测距接入点（核心）
     *   TIM7 采样满目标点数（1024~3072）后置 adc_done=1，且 ISR 已自行关闭 TIM7，
     *   不会再写 adc_buf，此时在主循环读取 adc_buf 是安全的（无 ISR 竞争）。
     *   用 s_sample_consumed 保证“每完成一次采样只跑一次算法”：
     *     - adc_done==1 且未消费 -> 跑算法，置消费标志
     *     - adc_done==0（采样进行中）-> 清消费标志，等采满
     * ============================================================ */
    if (BSP_ADCsamp_IsDone())
    {
        if (!s_sample_consumed)
        {
            s_sample_consumed = 1U;
            App_Radar_ProcessSample();
        }
        /* 已消费且 adc_done 仍为 1：静默等待下一轮 BSP_ADCsamp_Start 把 adc_done 清 0 */
    }
    else
    {
        s_sample_consumed = 0U;   /* 采样进行中：标记为未消费，下一轮采满即处理 */
    }

    /* 无新捕获 / 上次 S7/S9 未发完，直接返回 */
    if (!f_ic_new)
    {
        return;
    }
    if ((s7_remain != 0U) || (s9_remain != 0U))
    {
        return;
    }

    /* 原子读取脉宽并消费标志 */
    __disable_irq();
    pulse_us = f_ic_val;
    f_ic_new = 0U;
    __enable_irq();

    /* 低于死区下界：S9 充电，往中心纠正
     * 仅设置 s9_remain 与脉宽，实际脉冲由 TIM2 下降沿 ISR 在 S6↓+76μs 触发
     * error 用距下界 F_IC_LOWER_US 的偏差（与 S7 路径距上界对称），
     * 这样脉冲接近死区边界时 error→0 < 400 能触发粗→精切换 */
    if (pulse_us < F_IC_LOWER_US)
    {
        error_us    = F_IC_LOWER_US - pulse_us;
        pulse_count = Radar_GetPulseCount(error_us);
        /* 上电大偏差且未精调过：宽脉冲快速充电；否则固定 2μs 精调（样机行为） */
        if (error_us < F_IC_WIDE_TH_US)
        {
            s_coarse_done = 1U;   /* 锁存：已进入精调区 */
        }
        BSP_s9_set_width_us((!s_coarse_done && (error_us >= F_IC_WIDE_TH_US))
                            ? S7_S9_WIDE_US : S7_S9_FINE_US);
        s9_remain   = pulse_count;
        printf("[RADAR] pulse=%u us, error=%u us -> S9 x %u (%s)\r\n", pulse_us, error_us,
               pulse_count, (!s_coarse_done && (error_us >= F_IC_WIDE_TH_US)) ? "wide" : "fine");
        return;
    }

    /* 高于死区上界：S7 放电，按上边界纠正
     * 仅设置 s7_remain 与脉宽，实际脉冲由 TIM2 下降沿 ISR 在 S6↓+76μs 触发 */
    if (pulse_us > F_IC_UPPER_US)
    {
        error_us    = pulse_us - F_IC_UPPER_US;
        pulse_count = Radar_GetPulseCount(error_us);
        if (error_us < F_IC_WIDE_TH_US)
        {
            s_coarse_done = 1U;   /* 锁存：已进入精调区 */
        }
        BSP_s7_set_width_us((!s_coarse_done && (error_us >= F_IC_WIDE_TH_US))
                            ? S7_S9_WIDE_US : S7_S9_FINE_US);
        s7_remain   = pulse_count;
        printf("[RADAR] pulse=%u us, error=%u us -> S7 x %u (%s)\r\n", pulse_us, error_us,
               pulse_count, (!s_coarse_done && (error_us >= F_IC_WIDE_TH_US)) ? "wide" : "fine");
        return;
    }

    /* 死区内：无需校正。测量序列（S3/S4/ADC）已由 TIM2 捕获 ISR 在 S6↑ 时
     * 自动启动（脉宽进入 [F_IC_MEAS_LOWER_US, F_IC_MEAS_UPPER_US] 即触发），
     * 不再经主循环/TIM5 启动，避免调度延迟影响 S3↑ = S6↑+5.9ms 的相位精度 */
}

/* ===================== 调试标定通道（UART4） =====================
 * 与双板串口协议完全独立，仅用于产线/调试时通过 UART4(printf口) 修正算法标定。
 * 命令格式：CAL <dist_per_sample_m> <zero_offset_m>
 *   例：CAL 0.0301 0.5
 * 解析成功后调用 Algo_SetCalibration 并回显确认。 */
extern UART_HandleTypeDef huart4;

void App_Debug_Task(void)
{
    static char line[64];
    static uint8_t pos = 0U;
    uint8_t b;

    while (HAL_UART_Receive(&huart4, &b, 1, 0) == HAL_OK)
    {
        if (b == '\r' || b == '\n')
        {
            if (pos > 0U)
            {
                line[pos] = '\0';
                float dps = 0.0f, off = 0.0f;
                if (sscanf(line, "CAL %f %f", &dps, &off) == 2)
                {
                    Algo_SetCalibration(dps, off);
                    printf("[DBG] calibration set: dist_per_sample=%.4f m, zero_offset=%.4f m\r\n", dps, off);
                }
                pos = 0U;
            }
        }
        else if (pos < sizeof(line) - 1U)
        {
            line[pos++] = (char)b;
        }
    }
}
