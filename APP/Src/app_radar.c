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

/* F_IC 阈值唯一定义在 bsp_timer.h（F_IC_LOWER_US / F_IC_UPPER_US），
 * 本文不再复述具体数值，避免"改一处忘一处"。
 * 2026-09-20 口径确认：下界 22875 / 上界 23975（几何中心 23425，死区 [22875,23975]）。
 * 测量序列的启动在 TIM2 S6↑ 捕获 ISR（见 bsp_timer.c），主循环只做校正。 */
#define F_IC_CENTER_US ((F_IC_LOWER_US + F_IC_UPPER_US) / 2U)

/* 宽脉冲粗调：误差 ≥ F_IC_WIDE_TH_US 时用 S7_S9_WIDE_US(40μs) 宽脉冲加快上电充电/放电，
 * 否则用 S7_S9_FINE_US(20μs) 精调。实时按 error 切换，不锁存——过冲到 >400μs
 * 自动切回粗调快速回拉，避免锁死后精调区卡死震荡（曾出现 5s 才收敛的故障）。
 * 注意：bsp_timer.c 的 S7S9_MIN/MAX_PULSE_TICK 钳位范围为 20~40μs（200~400 ticks），
 *       故 WIDE_US/FINE_US 必须落在该范围内，否则会被钳位失效。 */
#define F_IC_WIDE_TH_US 400U
#define S7_S9_WIDE_US  40U   /* 粗调 40μs（对应 bsp_timer.c 400 ticks） */
#define S7_S9_FINE_US  20U   /* 精调 20μs（对应 bsp_timer.c 200 ticks） */

/* 偏差分段 */
#define F_IC_SEG1_US 50U
#define F_IC_SEG2_US 100U
#define F_IC_SEG4_US 400U
#define F_IC_SEG8_US 1200U

#define MAX_S7_S9_PULSE 50U

uint32_t pulse_us; // temp

/**
 * @brief  根据脉宽偏差量分段决定 S7/S9 校正脉冲个数
 * @note   偏差越大脉冲越多，实现非线性收敛（与下方实现一一对应）：
 *         <50μs→2, <100μs→4, <400μs→8, <1200μs→16, >=1200μs→50
 * @param  error_us : 当前脉宽与目标窗口边界的偏差（微秒）。
 *                   S9 路径 = F_IC_LOWER_US - pulse_us（距下界）；
 *                   S7 路径 = pulse_us - F_IC_UPPER_US（距上界）。
 *                   注意不是"与中心 23425 的偏差"——用中心会让误差恒 ≥550，
 *                   导致粗调恒锁存（历史故障，见代码阅读指南 §4）。
 * @retval 脉冲个数（2/4/8/16/50）
 * @complexity  时间 O(1) 固定4级比较，空间 O(1)
 */
static uint8_t Radar_GetPulseCount(uint32_t error_us)
{
    if (error_us < F_IC_SEG1_US)
    {
        return 2U;
    }
    if (error_us < F_IC_SEG2_US)
    {
        return 4U;
    }
    if (error_us < F_IC_SEG4_US)
    {
        return 8U;
    }
    if (error_us < F_IC_SEG8_US)
    {
        return 16U;
    }

    return MAX_S7_S9_PULSE;
}

/* ============================================================
 *  采样消费管理 + 滤波/测距接入（仅在本文件内实现）
 * ============================================================ */
static uint8_t s_sample_consumed = 0U;   /* 当前采样是否已跑过算法 */
static Algo_RadarResult_t s_last_result; /* 最近一次测量结果，供调试查看 */

/* 显示板 REQ_MEAS / KEY(K1) 请求即使在测量失败时也必须闭环回复：
 * 回一帧全零（peak_count=0）的无效 MEAS，避免一直无回波时显示板永久等待（死锁）。
 * 成功路径在 App_Radar_ProcessSample 末尾用真实帧回复并消费标志，不走这里。 */
static void App_Radar_ReplyMeasIfFailed(void)
{
    if (Disp_ConsumeMeasReq())
    {
        Algo_RadarResult_t invalid;
        memset(&invalid, 0, sizeof(invalid));
        Disp_DownSendMeas(&invalid, 0U);
        printf("[RADAR] REQ_MEAS replied: invalid measurement (peak_count=0)\r\n");
    }
}

/**
 * @brief  对"已采满"的 adc_buf 运行滤波 + 回波发现测距
 * @note   仅当 BSP_ADCsamp_IsDone()==1 时调用（adc_buf 已稳定，无 ISR 竞争）
 *         流程：Algo_MeasureDistance → 处理返回码（无基线则自动学习空罐基线）
 *         → 成功则 printf 调试 + Disp_DownSendMeas 经 USART1 下发显示板
 * @retval 无（结果存入 s_last_result，副作用为串口输出 + 下行帧发送）
 * @complexity  时间 O(N*W)（由 Algo_MeasureDistance 主导），空间 O(N) 栈
 */
static void App_Radar_ProcessSample(void)
{
    uint8_t ret = Algo_MeasureDistance(&s_last_result);

    if (ret == 1U)
    {
        /* 无基线（首次上电）或档位变化：以当前空罐采样学习虚假回波基线，
         * 下一轮采样起按 adc-baseline 差分测距。
         * 也可由显示板 DPARAM_SERV_FALSE_ECHO 触发 App_Config 主动重学。*/
        Algo_LearnEmptyTank();
        printf("[RADAR] no baseline -> learned empty-tank baseline; distance from next cycle\r\n");
        App_Radar_ReplyMeasIfFailed();
        return;
    }
    if (ret == 2U)
    {
        printf("[RADAR] sample not done (unexpected)\r\n");
        App_Radar_ReplyMeasIfFailed();
        return;
    }
    if (ret == 3U)
    {
        printf("[RADAR] sample error (adc_error=1)\r\n");
        App_Radar_ReplyMeasIfFailed();
        return;
    }
    if (ret == 4U)
    {
        printf("[RADAR] no echo detected (ret=4)\r\n");
        App_Radar_ReplyMeasIfFailed();
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
    /* 本轮 MEAS 即对 REQ_MEAS / KEY=0x01 的响应，消费补发标志 */
    (void)Disp_ConsumeMeasReq();

    /* 回波包络：按比例从 adc_buf 抽点归一化为 128 点下行（R4 修复） */
    {
        static uint8_t echo128[DISP_ECHO_LEN];
        uint16_t n = BSP_Range_GetSampleCount();
        uint16_t norm_peak;
        if (n == 0U || n > ADC_SAMPLE_COUNT_MAX)
            n = ADC_SAMPLE_COUNT_MAX;
        norm_peak = Disp_BuildEchoEx((const uint16_t *)(void *)adc_buf, n, echo128);
        Disp_DownSendEcho(echo128);

        /* 虚假回波曲线：空罐学习基线的 128 点包络。
         * 已通过带类型的 0x06 帧下发，显示板按类型分缓冲存放，不再与回波曲线互相覆盖。
         * 归一化基准复用上面的 norm_peak —— 若按基线自身峰值归一化，会把微弱背景
         * 放大到满屏，看起来像有强回波，是误导性的显示（详见 Disp_BuildCurveNorm 注释）。*/
        if (Algo_HasBaseline())
        {
            const uint16_t *bl = Algo_GetBaseline();
            if (bl != (const uint16_t *)0)
            {
                static uint8_t echo_false[DISP_ECHO_LEN];
                (void)Disp_BuildCurveNorm(bl, n, norm_peak, echo_false);
                Disp_DownSendEchoTyped(DISP_CURVE_FALSE, echo_false);
            }
        }
    }

    /* 诊断信息（含传感器温度） */
    {
        uint8_t reliability = (s_last_result.peak_count > 0U) ? 100U : 0U;
        uint8_t status = (s_last_result.peak_count > 0U) ? 0x01U : 0x00U;
        Disp_DownSendDiag(reliability, status, 0.0f, 0.0f, App_Config_GetSensorTemp());
    }

    /* ===== 4-20mA 输出（R7 修复）：距离 ↔ 电流线性标定 =====
     * lowAdjustVal  → 4mA  (DAC=0)
     * highAdjustVal → 20mA (DAC=65535)
     * 超界钳位；currMode 决定正/反作用；currFault 决定无效测量时的故障电流。 */
    {
        uint16_t dac = 0U;
        uint8_t  skip_dac = 0U;   /* 1=本次不重写 DAC（故障档"无变化"/硬件报警已验证）*/
        if (s_last_result.peak_count > 0U)
        {
            float lo = gRadarConfig.lowAdjustVal;
            float hi = gRadarConfig.highAdjustVal;
            float span = hi - lo;
            float d = s_last_result.distance;
            if (span <= 0.0f)
                span = 1.0f;
            if (d <= lo)
                dac = 0U;
            else if (d >= hi)
                dac = 65535U;
            else
                dac = (uint16_t)((d - lo) / span * 65535.0f);

            /* ---- currMode：4~20mA / 20~4mA 反作用（对齐飞卓 p12 §4.2 原文）----
             * 飞卓原文：
             *   "4~20mA表示低料位对应4mA，高料位对应20mA；
             *    20~4mA表示低料位对应20mA，高料位对应4mA。"
             * 显示板 dict_currMode = {"4~20mA", "20~4mA"}，索引 0 = 正作用（默认）、
             * 索引 1 = 反作用，与显示板 MENU_CURRENT_MODE 的 min=0/max=1 一致。
             *
             * ⚠ 字段归属纠正（本期首要澄清点）：
             *   做"电流反作用"的字段是 currMode，不是 outMap。
             *   outMap 在显示板上的文案是「输出映射 / Out Map」，选项 {"线性","锥筒"}，
             *   语义是"容器形状 → 体积映射"（线性罐 / 锥筒罐），属算法层，
             *   与 4~20mA 正反作用无关。此前方案 §3.9 把 outMap 当成反作用开关，
             *   是字段张冠李戴 —— 若照那个做，菜单里选"锥筒"会翻转电流，
             *   而"输出模式"选 20~4mA 反而不生效，与显示板文案完全对不上。
             *
             * ⚠ 反作用只对【有效测量的线性映射段】生效，不影响故障电流分支：
             *   故障档走 skip_dac / AD5421_ForceAlarm() 硬件通路，与此处无关。
             * ⚠ 上线前必须"已知料位 → 测电流"双向验证：反作用若接反，
             *   低料位会读 20mA（完全反向），是现场最容易被误判为"传感器坏了"的故障。*/
            if (gRadarConfig.currMode == 1U)   /* 1 = 20~4mA 反作用 */
            {
                dac = (uint16_t)(65535U - dac);
            }
        }
        else
        {
            /* 无效测量：故障电流 —— 档位语义对齐飞卓（p13 §4.2）：
             *   0 = 无变化  ：保持上一次输出值（skip_dac=1，不重写 DAC）
             *   1 = 20.5mA  ：高报警（用芯片硬件报警命令，实测确认实际 mA）
             *   2 = 22.0mA  ：高报警（同上；若与档1需区分，须实测后改用标定码值）
             * 注：飞卓正文另有 "<3.8mA" 低报警档，当前显示板无此选项，暂不实现。
             * ⚠ 20.5/22.0mA 无法用 4~20mA 量程的 DAC 码值直接输出（满码仅 20.0mA），
             *   故走 AD5421_ForceAlarm() 硬件报警通路。上线前必须用电流表实测三档实际值。
             * 用 skip_dac 标志而非 return —— 避免"函数末尾新增收尾代码被静默跳过"。*/
            switch (gRadarConfig.currFault)
            {
            case 0U:
                skip_dac = 1U;                       /* 无变化：保持上一次输出 */
                break;
            case 1U:
                (void)AD5421_ForceAlarm();           /* 高报警档1 */
                skip_dac = 1U;
                break;
            case 2U:
                (void)AD5421_ForceAlarm();           /* 高报警档2 */
                skip_dac = 1U;
                break;
            default:
                skip_dac = 1U;                       /* 未知档：保持，不输出未定义电流 */
                break;
            }
        }
        if (skip_dac == 0U)
        {
            /* ---- currMin：最小电流下界（4mA / 3.8mA）----
             * 对齐飞卓 p13 §4.2："最小电流用于选择输出最小电流为4mA 或3.8mA。"
             * 显示板 dict_minCurr = {"4mA","3.8mA"}，索引 0 = 4mA、1 = 3.8mA，
             * 与 MENU_CURRENT_MIN 的 min=0/max=1 一致。
             *
             * 语义：这是【环路电流的下限地板】，用于抑制"料位低于低位调整点"时
             *       电流继续下探。当前线性映射在 d<=lo 时已经钳到 DAC=0（=4mA），
             *       所以两个档位的差别只在"地板取 4mA 还是取 3.8mA"。
             *
             * ⚠⚠ 未闭合项：3.8mA 在当前量程下【无法用 0..65535 的码值表达】。
             *   AD5421 本工程现行标定是 lowAdjustVal→DAC 0 = 4mA（见上方注释），
             *   即 DAC 0 已经是 4mA 这条端点。要输出 3.8mA 必须给出一个
             *   【小于 0 的码值】——物理上不存在。可行路径只有两条：
             *     ① 把 RANGE0/RANGE1 跳线改到 "3.8~21mA" 档（ad5421.h:75），
             *        此时 3.8mA 对应 DAC 0，代价是 20mA 上界跟着变成 21mA；
             *     ② 保持 4~20mA 档，用 16 位 DAC 的【负码值偏移】做不到，
             *        只能在标定表里多标一个点，实际仍是 4mA 附近，无法真正到 3.8mA。
             *   两条路都【必须实测】，不能凭线性外推硬算 —— 与 §3.4 已定的纪律一致
             *   （"代码里的静态事实"不能推成"运行期后果"，DAC 码↔mA 必须实测）。
             *
             *   因此本档位先留【占位宏 + 显式告警】，不猜一个数就写死：
             *   在实测出 3.8mA 对应码值前，currMin==1 与 currMin==0 行为一致（都是 4mA），
             *   并在下方注释保留待办。这样"选了不生效"是显式可查的，
             *   而不是静默输出一个错误电流。*/
#if defined(AD5421_DAC_3P8MA_CALIBRATED)
            if (gRadarConfig.currMin == 1U)
            {
                dac = AD5421_DAC_3P8MA;   /* 3.8mA 地板（实测标定值，见宏定义处）*/
            }
#endif
            /* currMin == 0（4mA）时不做任何修改：dac 已钳在 >= 0，即 >= 4mA 端点。*/
            AD5421_SetDACOutput(dac);
        }
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
    uint8_t pulse_count;

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
        s_sample_consumed = 0U; /* 采样进行中：标记为未消费，下一轮采满即处理 */
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
        error_us = F_IC_LOWER_US - pulse_us;
        pulse_count = Radar_GetPulseCount(error_us);
        /* 实时按 error 选脉宽：≥400μs 用 40μs 粗调快速逼近/回拉，<400μs 用 20μs 精调。
         * 不锁存，过冲到 >400μs 自动切回粗调快速回正，避免锁死后精调区卡死震荡。 */
        BSP_s9_set_width_us((error_us >= F_IC_WIDE_TH_US) ? S7_S9_WIDE_US : S7_S9_FINE_US);
        s9_remain = pulse_count;
        printf("[RADAR] pulse=%u us, error=%u us -> S9 x %u (%s)\r\n", pulse_us, error_us,
               pulse_count, (error_us >= F_IC_WIDE_TH_US) ? "wide" : "fine");
        return;
    }

    /* 高于死区上界：S7 放电，按上边界纠正
     * 仅设置 s7_remain 与脉宽，实际脉冲由 TIM2 下降沿 ISR 在 S6↓+76μs 触发 */
    if (pulse_us > F_IC_UPPER_US)
    {
        error_us = pulse_us - F_IC_UPPER_US;
        pulse_count = Radar_GetPulseCount(error_us);
        /* 实时按 error 选脉宽：≥400μs 用 40μs 粗调快速逼近/回拉，<400μs 用 20μs 精调 */
        BSP_s7_set_width_us((error_us >= F_IC_WIDE_TH_US) ? S7_S9_WIDE_US : S7_S9_FINE_US);
        s7_remain = pulse_count;
        printf("[RADAR] pulse=%u us, error=%u us -> S7 x %u (%s)\r\n", pulse_us, error_us,
               pulse_count, (error_us >= F_IC_WIDE_TH_US) ? "wide" : "fine");
        return;
    }

    /* 死区内：无需校正。测量序列（S3/S4/ADC）已由 TIM2 捕获 ISR 在 S6↑ 时
     * 自动启动（脉宽进入 [F_IC_MEAS_LOWER_US, F_IC_MEAS_UPPER_US] 即触发），
     * 不经主循环启动，避免调度延迟影响 S3↑ = S6↑+5.9ms 的相位精度 */
}

/* ===================== 调试标定通道（UART4） =====================
 * 与双板串口协议完全独立，仅用于产线/调试时通过 UART4(printf口) 修正算法标定。
 *
 * 启用方式：在 app_radar.h 里取消 DEBUG_CAL_CHANNEL 的注释。
 * 命令格式：CAL <dist_per_sample_m> <zero_offset_m>
 *   例：CAL 0.0301 0.5
 *
 * ⚠ 两个参数的效力【并不对等】，原注释"两个参数都能调"是不准确的：
 *   - dist_per_sample：参与两条路径（app_algorithm.c:447 的 TOF 式
 *     与 :452 的降级式），**真正有效**。
 *   - zero_offset：**仅在 tx_sample_valid == 0 的降级路径生效**
 *     （app_algorithm.c:452）。正常运行时采样窗内必然捕获到 S6↓，
 *     tx_sample_valid 恒为 1（走 :447 的飞行时间公式），
 *     该式里【没有 zero_offset 这一项】—— 即正常模式下这个值不参与任何计算。
 *
 * ⚠ 本通道【不写 EEPROM，断电即失效】。开机 App_Radar_Init() 会用编译期常量
 *   RADAR_HW_TIME_STRETCH 重算 dist_per_sample（app_algorithm.c:369）覆盖掉
 *   这里的设置。**标定结果必须固化回 RADAR_HW_TIME_STRETCH 后重新编译**，
 *   否则下次上电就丢了。
 *
 * ⚠ 安全：UART4 即 printf 口，本通道【无鉴权】。故 app_radar.h 里默认关闭，
 *   量产固件不得打开。 */
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
