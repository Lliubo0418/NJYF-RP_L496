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

/* 窗口阈值唯一定义在 bsp_timer.h（S6 周期窗 F_IC_PERIOD_LOWER_US / F_IC_PERIOD_UPPER_US），
 * 本文不复述数值。测量序列的启动在 TIM2 S6↑ 捕获 ISR（见 bsp_timer.c），主循环只做校正。
 * ★ 2026-10-13：S7/S9 校正的被控量由【S6 低电平宽度】改为【S6 周期】
 *   （TIM2 改测上升沿→上升沿；K = f_T × 周期）。低电平窗同日退役。 */
#define F_IC_CENTER_US ((F_IC_LOWER_US + F_IC_UPPER_US) / 2U)

/* 宽脉冲粗调：error ≥ F_IC_WIDE_TH_US 用 S7_S9_WIDE_US(40μs)，否则用 S7_S9_FINE_US(20μs)。
 * ⚠ 实时按 error 切换、不锁存（锁存会在精调区卡死震荡），且两者必须落在
 *   bsp_timer.c 的 S7S9_MIN/MAX_PULSE_TICK 钳位范围 20~40μs 内，否则会被钳位失效。 */
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
 * @brief  根据被控量偏差分段决定 S7/S9 校正脉冲个数
 * @note   偏差越大脉冲越多，实现非线性收敛
 * @param  error_us : 距目标窗口边界的偏差
 *                   ★ 2026-10-13 起为【S6 周期】偏差：
 *                     S9 = F_IC_PERIOD_LOWER_US - period_us；S7 = period_us - F_IC_PERIOD_UPPER_US
 *                   ⚠ 不可改为"与窗口中心的偏差"：误差会恒 ≥ 半窗宽，导致粗调恒锁存（粗调区永不退出）
 * @retval 脉冲个数
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

/* ⚠ 显示板 REQ_MEAS / KEY(K1) 在测量失败时也必须回一帧全零（peak_count=0）的 MEAS，
 * 否则无回波时显示板会永久等待（死锁）。成功路径在 App_Radar_ProcessSample 末尾用真实帧回复。 */
static void App_Radar_ReplyMeasIfFailed(void)
{
    if (Disp_RecvTakeMeasReq())
    {
        Algo_RadarResult_t invalid;
        memset(&invalid, 0, sizeof(invalid));
        Disp_SendMeas(&invalid, 0U);
        printf("[RADAR] REQ_MEAS replied: invalid measurement (peak_count=0)\r\n");
    }
}

/**
  * @brief  写 4-20mA 输出（有效测量走线性映射，无效测量走 currFault 故障档）
  * @param  meas_valid : 1=本次测量有效（distance 可信）；0=无效（无回波 / 采样错误 / 零点漏捕）
  * @note   ★ 2026-10-27：原实现把这段逻辑内联在 App_Radar_ProcessSample 的 ret==0 分支里，
  *         导致【失败路径完全不碰 DAC】—— 一旦测量持续失败（含 R-13 修好后的"零点漏捕"），
  *         电流会永久冻结在最后一个有效值，currFault 配置的报警档形同虚设。
  *         现抽成独立函数，失败分支也必须调用（meas_valid=0）。
  *         ⚠ 默认配置 currFault=0（无变化）时本函数在失败分支不写 DAC，行为与旧版完全一致，
  *           所以这次重构对出厂默认没有任何行为改变。
  * @complexity 时间 O(1)（currFault=1/2 时含一次 AD5421 报警指令），空间 O(1)
  */
static void App_Radar_ApplyCurrentOut(uint8_t meas_valid)
{
    uint16_t dac = 0U;
    uint8_t  skip_dac = 0U;   /* 1=本次不重写 DAC（故障档"无变化"/硬件报警已验证）*/

    if (meas_valid != 0U)
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

        /* ---- currMode：4~20mA / 20~4mA 反作用（对齐飞卓 p12 §4.2）----
         * 显示板 dict_currMode 索引 0=正作用(默认)、1=反作用，与 MENU_CURRENT_MODE min/max=0/1 一致。
         * ⚠ 反作用字段是 currMode，不是 outMap（outMap 是"容器形状→体积映射"，属算法层）。
         * ⚠ 只对【有效测量的线性映射段】生效，不影响故障电流分支（走 skip_dac / AD5421_ForceAlarm）。
         * ⚠ 上线前必须"已知料位→测电流"双向验证：接反则低料位读 20mA，易被误判为传感器故障。*/
        if (gRadarConfig.currMode == 1U)   /* 1 = 20~4mA 反作用 */
        {
            dac = (uint16_t)(65535U - dac);
        }
    }
    else
    {
        /* 无效测量：故障电流档 currFault（对齐飞卓 p13 §4.2）：
         *   0=无变化（保持上次输出）；1=20.5mA、2=22.0mA 高报警。
         * ⚠ 20.5/22.0mA 超出 4~20mA 满码 20.0mA，必须走 AD5421_ForceAlarm() 硬件通路；上线前须实测三档。
         * ⚠ 用 skip_dac 而非 return，避免函数末尾收尾代码被静默跳过。*/
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
        /* ---- currMin：最小电流下界（4mA / 3.8mA，对齐飞卓 p13 §4.2）----
         * 显示板 dict_minCurr 索引 0=4mA、1=3.8mA，与 MENU_CURRENT_MIN min/max=0/1 一致。
         * 语义：抑制"料位低于低位调整点"时电流继续下探的地板；d<=lo 时已钳到 DAC=0（=4mA）。
         * ⚠ 未闭合项：3.8mA 在现行标定（lowAdjustVal→DAC 0 = 4mA）下无法用 0..65535 码值表达，
         *   须先把 AD5421 RANGE0/RANGE1 跳线改到 "3.8~21mA" 档（上界随之变 21mA）并实测；
         *   故此处只留占位宏 + 显式告警：实测前 currMin==1 与 0 行为一致（都是 4mA）。*/
#if defined(AD5421_DAC_3P8MA_CALIBRATED)
        if (gRadarConfig.currMin == 1U)
        {
            dac = AD5421_DAC_3P8MA;   /* 3.8mA 地板（实测标定值，见宏定义处）*/
        }
#endif
        /* currMin == 0（4mA）：dac 已钳在 >= 0，无需修改 */
        AD5421_SetDACOutput(dac);
    }
}

/**
 * @brief  对"已采满"的 adc_buf 运行滤波 + 回波发现测距
 * @note   仅当 BSP_ADCsamp_IsDone()==1 时调用（adc_buf 已稳定，无 ISR 竞争）
 * @retval 无（结果存入 s_last_result，副作用为串口输出 + 下行帧发送）
 * @complexity  时间 O(N*W)（由 Algo_MeasureDistance 主导），空间 O(N) 栈
 */
static void App_Radar_ProcessSample(void)
{
    /* ---- 空罐基线【多帧学习】会话：每完成一次采样推一拍 ----
     * 放在测距之前：Algo_MeasureDistance 会改写 adc_buf 里的哨兵点（Algo_FixupSentinelU16），
     * 学习要的是"本帧原始数据"，先喂更干净。
     * ⚠ 学完才落盘：每帧都写会白写 8 次 10 KB（R-07：单次写入阻塞 0.88~1.6 s）。*/
    if (Algo_LearnIsActive())
    {
        uint8_t ls = Algo_LearnTick();
        if (ls == 1U)
        {
            uint8_t e = App_Config_SaveBaselineToEEPROM();
            printf("[RADAR] baseline learned (%u frames) & saved: eeprom=%u\r\n",
                   (unsigned)Algo_LearnProgress(), (unsigned)e);
        }
        else if (ls == 2U)
        {
            printf("[RADAR] baseline learn FAILED - old baseline kept\r\n");
        }
    }

    uint8_t ret = Algo_MeasureDistance(&s_last_result);

    if (ret == 1U)
    {
        /* ★ 2026-10-29：取消「无基线 / 切档时自动学习」。
         *   基线只有两个来源：① 上电从 EEPROM 导入；② 显示板「服务 → 虚假回波 → 新建/更新」
         *   手动触发多帧学习（Algo_LearnBegin，唯一调用点 App_Config_LearnFalseEcho）。
         *   不做自动学习：无法确认「此刻罐是空的」，有料时会把料面学成背景、此后差分一直减掉
         *   真实回波 —— 距离静默错误且不报警，比测不到更危险（R-04 只兜住点数变化那一半）。
         *   ⇒ 没基线就明确不测，强制操作人先确认空罐再手动学。
         *   ⚠ 学习会话进行中不打这条：那段时间注定没基线，每帧打一遍会刷屏。 */
        if (Algo_LearnIsActive() == 0U)
        {
            printf("[RADAR] no baseline (or range changed): measurement SUPPRESSED. "
                   "Learn it first: menu Service > False Echo > New/Update\r\n");
        }
        App_Radar_ReplyMeasIfFailed();
        App_Radar_ApplyCurrentOut(0U);   /* 无效测量 → currFault 故障档（默认 0=保持，行为不变）*/
        return;
    }
    if (ret == 2U)
    {
        printf("[RADAR] sample not done (unexpected)\r\n");
        App_Radar_ReplyMeasIfFailed();
        App_Radar_ApplyCurrentOut(0U);
        return;
    }
    if (ret == 3U)
    {
        /* ★ 2026-10-27 修 R-13：区分「零点漏捕」与「ADC 连续失败」——两者 adc_error 都为 1，
         *   但前者是 S6↓ 没进来（查占空比突变 / S6 硬件），后者是 SPI/ADC 坏了（查接线/供电）。
         *   ⚠ 这条日志是 R-13 唯一可现场定位的线索：修复后此故障不再表现为"假距离"，
         *     而是表现为"测量停更 + 电流按 currFault 走"，否则无法与其它失败区分。*/
        if (adc_lead_expired)
        {
            printf("[RADAR] zero-cross MISSED (adc_lead_expired=1): S6 falling edge not captured "
                   "within lead window -> sample discarded (was: silent 37~50m fake distance)\r\n");
        }
        else
        {
            printf("[RADAR] sample error (adc_error=1)\r\n");
        }
        App_Radar_ReplyMeasIfFailed();
        App_Radar_ApplyCurrentOut(0U);
        return;
    }
    if (ret == 4U)
    {
        printf("[RADAR] no echo detected (ret=4)\r\n");
        App_Radar_ReplyMeasIfFailed();
        App_Radar_ApplyCurrentOut(0U);
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

    /* ★ 2026-10-29：把「距离偏量」(servOffset) 真正接到测距出口。
     *   此前该参数只被「存」——结构体字段、EEPROM 序列化、SetParam 赋值、显示板菜单，
     *   主控与显示板【都没有任何消费点】⇒ 菜单里改它照样回 SAVED OK，读数一毫米不变。
     *   语义与显示板菜单一致：菜单填的是「直接加到测量值上的修正量」，
     *   即读数偏高 0.85 m 就填 -0.85，所以这里是【加法】不是减法。
     *   位置放在 ApplyDamping 之后：常数在滤波前后减数学等价（filter(x-c)=filter(x)-c），
     *   放后面不影响盲区/量程夹取，且能让 4-20mA、HART PV、显示板、上位机四条路一致。
     *   负值截 0：菜单范围 ±10 m，偏量可能大于实测距离，负距离对下游无意义。 */
    s_last_result.distance += gRadarConfig.servOffset;
    if (s_last_result.distance < 0.0f)
        s_last_result.distance = 0.0f;

    /* 更新 HART 主变量（PV = 物位/距离） */
    App_HART_SetPV(s_last_result.distance);

    /* ===== 协议发送接入点（MEAS / 回波 / 诊断） ===== */
    Disp_SendMeas(&s_last_result, (tx_sample_valid ? 1U : 0U));
    /* 本轮 MEAS 即对 REQ_MEAS / KEY=0x01 的响应，消费补发标志 */
    (void)Disp_RecvTakeMeasReq();

    /* 回波包络：★ 从测距零点 adc_buf[tx_sample_offset] 起取，按比例抽点归一化为 128 点下行。
     * ⚠ 2026-10-14：零点由 S6↓ 捕获实时触发 ⇒ tx_sample_offset 常态 = 0；仍按它对齐是为了
     *   兜住「捕获被漏掉」的降级路径。可用点数 = N − offset。 */
    {
        static uint8_t echo128[DISP_ECHO_LEN];
        uint16_t n   = BSP_Range_GetSampleCount();
        uint16_t off = tx_sample_offset;
        uint16_t norm_peak;
        if (n == 0U || n > ADC_SAMPLE_COUNT_MAX)
            n = ADC_SAMPLE_COUNT_MAX;
        if (off >= n) off = 0U;          /* 越界保护 */
        norm_peak = Disp_SendPackEchoEx((const uint16_t *)(void *)adc_buf + off,
                                     (uint16_t)(n - off), echo128);
        Disp_SendEcho(echo128);

        /* 虚假回波曲线：空罐学习基线的 128 点包络（与回波同源同偏移）。
         * ⚠ 归一化基准必须复用上面的 norm_peak —— 按基线自身峰值归一化会把微弱背景放大成假强回波 */
        if (Algo_HasBaseline())
        {
            const uint16_t *bl = Algo_GetBaseline();
            if (bl != (const uint16_t *)0)
            {
                static uint8_t echo_false[DISP_ECHO_LEN];
                (void)Disp_SendPackCurveNorm(bl + off, (uint16_t)(n - off), norm_peak, echo_false);
                Disp_SendEchoTyped(DISP_CURVE_FALSE, echo_false);
            }
        }
    }

    /* 诊断信息（含传感器温度） */
    {
        uint8_t reliability = (s_last_result.peak_count > 0U) ? 100U : 0U;
        uint8_t status = (s_last_result.peak_count > 0U) ? 0x01U : 0x00U;
        Disp_SendDiag(reliability, status, 0.0f, 0.0f, App_Config_GetSensorTemp());
    }

    /* ===== 4-20mA 输出（2026-10-27 抽成独立函数）=====
     * lowAdjustVal → 4mA (DAC=0)，highAdjustVal → 20mA (DAC=65535)，超界钳位；
     * currMode 决定正/反作用，currFault 决定无效测量时的故障电流。
     * ⚠ 抽成 App_Radar_ApplyCurrentOut() 的原因：原内联写法使【失败路径完全不碰 DAC】，
     *   测量持续失败时电流会冻结在最后有效值、currFault 报警档失效。见该函数注释。 */
    App_Radar_ApplyCurrentOut(1U);
}

void App_Radar_Init(void)
{
    /* 算法侧标定 + 运行配置初始化。zero_offset 需现场标定，此处置 0：
     * tx_sample_valid 有效时走飞行时间法、不依赖该值。*/
    App_Config_Init();
    Algo_InitRadarCalibration(0.0f);
}

void App_Radar_Run(void)
{
    uint32_t pulse_us;
    uint32_t period_us;
    uint32_t error_us;
    uint8_t pulse_count;

    /* ★ 2026-10-22 修 R-11：S7/S9 连发超时兜底。必须放在本函数所有 return 之前 ——
     *   下面有多处提前 return（f_ic_new=0、period_us=0 等），remain 卡住时恰恰会一直命中
     *   这些 return，兜底就永远跑不到。超时判定与恢复动作都在 BSP 层，此处只负责上报。*/
    if (BSP_S7S9_CheckTimeout() != 0U)
    {
        printf("[RADAR] S7/S9 remain timeout (UEV lost / burst-type mismatch) -> "
               "forced clear, AGC resumed\r\n");
    }

    /* ★ 2026-10-27 修 R-12：TIM3 S3/S4 序列超期自愈上报。
     *   与上面 R-11 同理，必须放在所有 return 之前 —— 序列超期时 BSP_Pulse_Start 不再被调用，
     *   adc_done 停在 1 或 0，下面的分支随时可能提前 return，告警就永远打不出来。
     *   ⚠ 这条日志是这个故障【唯一】的现场线索：修复前它表现为"永久停测、只能复位"，
     *     修复后表现为"偶发丢 1~2 次测量 + 本行告警"。持续增长 ⇒ 查 TIM3 侧 EMI / CCER 被误写。*/
    {
        static uint16_t s_tim3_recover_seen = 0U;
        uint16_t now = BSP_Timer_GetTim3RecoverCount();
        if (now != s_tim3_recover_seen)
        {
            s_tim3_recover_seen = now;
            printf("[RADAR] TIM3 S3/S4 sequence overran one ARR period (compare edge lost) -> "
                   "forced idle, self-heal #%u; measurement resumes on next S6 rising edge\r\n",
                   (unsigned)now);
        }
    }

    /* ============================================================
     * 滤波 + 测距接入点（核心）
     *   adc_done=1 时 ISR 已关闭 TIM7、不再写 adc_buf，主循环读取无 ISR 竞争；
     *   s_sample_consumed 保证每完成一次采样只跑一次算法。
     * ============================================================ */
    if (BSP_ADCsamp_IsDone())
    {
        if (!s_sample_consumed)
        {
            s_sample_consumed = 1U;
            App_Radar_ProcessSample();
        }
        /* 已消费但 adc_done 仍为 1：等待 BSP_ADCsamp_Start 清 0 */
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

    /* 原子读取被控量并消费标志
     * ★ 2026-10-13 起闭环改锁【S6 周期】(上升沿→上升沿，K = f_T × 周期)：
     *   S6 非理想方波，低电平宽度只是周期的间接量，锁低电平 ≠ 锁周期。
     *   pulse_us 仍读出，仅用于打印对照。 */
    __disable_irq();
    period_us = BSP_GetS6PeriodUs();
    pulse_us  = f_ic_val;
    f_ic_new  = 0U;
    __enable_irq();

    /* ★ S6 是前提条件，尚未测到周期（=0）时什么都不做：既无从判 S7/S9 的方向与偏差，
     *   也不该往后走 S3/ADC。等下一个 S6↑ 捕获出周期再判。
     *   周期需相邻两个上升沿才能算出 ⇒ 上电后首发的 S6↑ 必然走到这里（一次，44ms 量级）。 */
    if (period_us == 0U)
    {
        return;
    }

    /* 低于周期窗下界：S9 充电。error 取距下界 F_IC_PERIOD_LOWER_US 的偏差（与 S7 距上界对称），
     * ⚠ 这样接近边界时 error→0 < 400 才能触发粗→精切换；脉冲由 TIM2 下降沿 ISR 在 S6↓+76μs 触发
     * ⚠ error 现为【周期】偏差，F_IC_SEG* / F_IC_WIDE_TH_US 分段阈值暂沿用原值 —— 若实测发现
     *   S7/S9 是"占空比不变"型（周期变化是低电平变化的 2 倍），再统一放大分段阈值。 */
    if (period_us < F_IC_PERIOD_LOWER_US)
    {
        error_us = F_IC_PERIOD_LOWER_US - period_us;
        pulse_count = Radar_GetPulseCount(error_us);
        BSP_s9_set_width_us((error_us >= F_IC_WIDE_TH_US) ? S7_S9_WIDE_US : S7_S9_FINE_US);
        s9_remain = pulse_count;
        printf("[RADAR] period=%u us (low=%u), error=%u us -> S9 x %u (%s)\r\n",
               period_us, pulse_us, error_us,
               pulse_count, (error_us >= F_IC_WIDE_TH_US) ? "wide" : "fine");
        return;
    }

    /* 高于周期窗上界：S7 放电。脉冲由 TIM2 下降沿 ISR 在 S6↓+76μs 触发 */
    if (period_us > F_IC_PERIOD_UPPER_US)
    {
        error_us = period_us - F_IC_PERIOD_UPPER_US;
        pulse_count = Radar_GetPulseCount(error_us);
        BSP_s7_set_width_us((error_us >= F_IC_WIDE_TH_US) ? S7_S9_WIDE_US : S7_S9_FINE_US);
        s7_remain = pulse_count;
        printf("[RADAR] period=%u us (low=%u), error=%u us -> S7 x %u (%s)\r\n",
               period_us, pulse_us, error_us,
               pulse_count, (error_us >= F_IC_WIDE_TH_US) ? "wide" : "fine");
        return;
    }

    /* 周期窗内：无需校正。测量序列（S3/S4/ADC）由 TIM2 捕获 ISR 在 S6↑ 时自动启动
     * （周期落 [F_IC_PERIOD_MEAS_LOWER_US, F_IC_PERIOD_MEAS_UPPER_US] 即触发）、
     * 不经主循环，以免调度延迟影响 S3↑ = S6↑+5.9ms 的相位精度 */
}

/* ===================== 调试标定通道（UART4） =====================
 * 与双板串口协议独立，仅供产线/调试用。启用：在 app_radar.h 取消 DEBUG_CAL_CHANNEL 注释。
 * 命令：CAL <dist_per_sample_m> <zero_offset_m>（例：CAL 0.0150 0.5）
 *
 * ⚠ 两参数效力不对等：dist_per_sample 参与 TOF 与降级两条路径，真正有效；
 *   zero_offset 仅在 tx_sample_valid == 0 的降级路径（Algo_MeasureDistance）生效，
 *   正常模式 tx_sample_valid 恒为 1、该式不含 zero_offset 项。
 * ⚠ 本通道不写 EEPROM、断电即失效；开机 App_Radar_Init() 会用 RADAR_HW_TIME_STRETCH
 *   重算 dist_per_sample 覆盖此设置 —— 标定结果必须固化回该常量后重新编译。
 * ⚠ 安全：UART4 即 printf 口、无鉴权，app_radar.h 默认关闭，量产固件不得打开。 */
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
