#include "bsp_timer.h"
#include "bsp_spi.h"
#include "adcs7476.h"
/*
  TIM2: 输入捕获 S6
  TIM3: S3 S4 S10
  TIM4: S7 S9 电容充放电
  TIM7: 溢出中断驱动 ADCS7476 采样（SPI1 + DMA，1024~3072 点随量程档位变化）
*/

/* ============ 时序常量（对应 L496时序.md 第6节）============ */
/* TIM3 @0.5MHz (PSC=160-1, 1 tick = 2μs)。
 * 不用 1MHz 的原因：TIM3 是 16 位定时器，1μs/tick 最大仅 65.5ms，
 * 而 70m 档 S3 结束沿在 S6↑+107.82ms → 必须降频到 2μs/tick（可覆盖 131ms）。
 * 时序语义（μs）保持不变：S3↑=S6↑+5.9ms（样机 5.94ms，留 40μs 反应时间）。 */
#define TIM3_TICK_US         2U     /* TIM3 计数单位（μs/tick）*/
#define US2TICK(us)          ((us) / TIM3_TICK_US)
#define S3_START_TICK        US2TICK(5900U)    /* 2950：S3↑ = S6↑+5.9ms */
/* S3 脉宽随量程档位展宽：采样点数×16μs(TIM7 62.5kHz) + 20ms 宽裕
 *   1024→36.4ms  2048→52.8ms  3072→69.2ms
 * 必须覆盖 ADC 采样窗口（S4↑@6.57ms + 点数×16μs），20ms 为尾部余量。 */
#define S3_SAMPLE_TICK       US2TICK(16U)      /* 8：每个 ADC 点 16μs */
#define S3_WIDTH_MARGIN_TICK US2TICK(20000U)   /* 10000：尾部 20ms 宽裕 */
#define S4_START_TICK        US2TICK(6350U)    /* 3175：S4↓ = S3↑+450μs */
#define S4_END_TICK          US2TICK(6570U)    /* 3285：S4↑，S4 脉宽 220μs（样机 206μs），结束沿启动 ADC */

/* ============ S3 测量调度（S3 两个一组，样机 0905 CSV 实测规律）============
 * 1) 组内：第二个 = 第一个 + 4T（52/52 个无批次间隔全部为 4T，铁证）；
 * 2) 组间：下一组第一个 = 组内第二个 + 20T（组首到组首 24T ≈ 1.098s ≈ 1.1s）；
 * 3) 校正推迟：每批 S7/S9 校正把下一次测量「推到」批次周期+3T（样机为
 *    最后批次+3T 才测量）。批次按 2T 节流连发时，每多一次校正测量顺延
 *    ≈91.5ms（2T），与样机/用户实测（1.1s→1.19s→1.28s）一致。
 *    注意是「推到 max()」而非「累加 +=」：累加在连续校正时会越推越远，
 *    把调度点永久甩开 → BSP_Pulse_Start 不再触发（历史失控 bug）；
 *    max() 模型天然有界（next ≤ 当前+3T），结构性杜绝失控。 */
#define S3_PAIR_GAP_T        4U   /* 组内间隔：+4T */
#define S3_GROUP_GAP_T      20U   /* 组间间隔：组内第二个 → 下一组第一个 +20T */
#define S3_BATCH_PUSH_T      3U   /* 校正批次把下一次测量推到 批次周期+3T */

/* TIM4 @10MHz (1 tick = 0.1μs) */
#define S7S9_DELAY_TICK         760U  /* 首脉冲延时：S6↓ + 76μs（规范固定，仅首脉冲）*/
#define S7S9_REPEAT_DELAY_TICK   40U  /* 连发间隔 4μs：连发不需要 76μs 延时（旧实现误用导致 ~80μs 间隔）*/
/* ============ S7/S9 校正脉冲脉宽（TIM4 @10MHz，1 tick = 0.1μs）============
 * ★唯一定义处：其他文件（app_radar.c 的 S7_S9_FINE_US/WIDE_US、bsp_timer.h 注释）
 *   只准引用"20~40μs"这个结论，不准复述 tick 数值。
 *
 * 2026-09-20 修正（§3.10 #C2）：原处遗留一组【旧的 2.0/20μs 定义】被注释掉后
 *   与当前生效定义并存，极易被误读为"钳位是 2~20μs"。
 *   ⚠ 若误按旧注释把钳位改回 20~200 ticks：
 *     · S7_S9_FINE_US(20μs) = 200 ticks 会【正好落在旧上限】被钳成上限值；
 *     · S7_S9_WIDE_US(40μs) = 400 ticks 直接【超出旧上限】也被钳到 200 ticks；
 *     ⟹ 粗调与精调【退化成同一个脉宽】，宽脉冲快充/快放的机制完全失效，
 *        表现为"上电收敛极慢"（历史上出现过 5s 才收敛的故障）。
 *   故此处删除旧定义，只保留一组生效值。 */
#define S7S9_FINE_PULSE_TICK     200U  /* 精调脉宽 20μs（S7/S9 一致）*/
#define S7S9_WIDE_PULSE_TICK    400U  /* 粗调脉宽 40μs：上电大偏差时快速充电/放电 */
#define S7S9_MIN_PULSE_TICK      200U  /* 脉宽下限 20μs */
#define S7S9_MAX_PULSE_TICK     400U  /* 脉宽上限 40μs */
#define S7S9_BATCH_PERIOD_T      2U   /* 批次节流：每 2 个 S6 周期最多一批（样机 91.5ms/批）*/

volatile uint8_t f_ic_sta = 0;
volatile uint32_t f_ic_val = 0;
volatile uint8_t f_ic_new = 0;   /* 1=有新的 f_ic_val 可读 */

/* 发射时刻记录：ADC 采样期间 S6 下降沿对应的采样点序号 */
volatile uint8_t  tx_sample_valid = 0;    /* 1=本次采样已记录到发射时刻 */
volatile uint16_t tx_sample_offset = 0;   /* 发射时刻对应的 ADC 采样点序号（0~3071）*/

static uint32_t tim2_start = 0;
static uint32_t tim2_end = 0;
static uint32_t tim2_pulse = 0;

static uint8_t tim3_s3_state = 0;
static uint8_t tim3_s8_state = 0;
static uint8_t tim3_s4_state = 0;
static uint8_t tim3_s10_state = 0;
static uint8_t tim4_s7_state = 0;
static uint8_t tim4_s9_state = 0;

volatile uint8_t s7_remain = 0;
volatile uint8_t s9_remain = 0;

/* TIM4 运行参数（脉宽可由应用层运行时调整，延时随首脉冲/连发切换）*/
static volatile uint16_t s7_width_tick   = S7S9_FINE_PULSE_TICK; /* S7 当前脉宽 */
static volatile uint16_t s9_width_tick   = S7S9_FINE_PULSE_TICK; /* S9 当前脉宽 */
static volatile uint16_t s7s9_delay_tick = S7S9_DELAY_TICK;      /* 当前脉冲延时 */
static volatile uint8_t  tim4_burst_type = 0U;   /* 0=空闲 1=S7 连发中 2=S9 连发中 */

/* S3 测量调度（S6 周期计数，在 TIM2 捕获 ISR 中维护；S3 两个一组）*/
static volatile uint32_t s6_cycle_cnt   = 0U;     /* S6↑ 周期计数器 */
static volatile uint32_t s3_next_cycle  = 0U;     /* 下一次允许启动测量的周期序号 */
static volatile uint8_t  s3_pair_wait   = 0U;     /* 0=下一发是组内第一个 1=组内第二个 */
static volatile uint32_t s_last_batch_cycle = 0U; /* 最近一批校正所在周期（节流用）*/

/* ============ TIM7 驱动 ADC 采样状态 ============ */
extern SPI_HandleTypeDef hspi1;     /* CubeMX 生成（spi.c）*/
extern TIM_HandleTypeDef htim7;    /* CubeMX 生成（tim.c）*/
extern TIM_HandleTypeDef htim2;    /* CubeMX 生成（tim.c），提供 TIM2 CNT 作为测距时间基准 */

volatile uint16_t adc_buf[ADC_SAMPLE_COUNT_MAX]; /* ADC 采样缓冲区（12-bit 有效值，按最大 3072 点分配）*/
volatile uint16_t adc_sample_count = 0;      /* 已采样点数 */
volatile uint16_t adc_target_count = ADC_SAMPLE_BASE; /* 本次采样目标点数（BSP_Pulse_Start 快照，TIM7 ISR 读）*/
volatile uint8_t  adc_dma_busy = 0;         /* 1=SPI1-DMA 接收进行中 */
volatile uint8_t  adc_done = 0;            /* 1=采样结束（成功采满 adc_target_count 或失败中止）*/
volatile uint8_t  adc_error = 0;           /* 1=因连续 ADC_FAIL_MAX 次失败而中止 */
static volatile uint8_t adc_fail_count = 0; /* 连续失败计数（成功一次即清零）*/
volatile uint8_t adc_active = 0;            /* 1=采样流程进行中（从 Start 到 done，供测量触发互斥；bsp_spi.c DMA 回调也会清零）*/

/* 量程档位（display 经 DPARAM_RANGE_SETTING 下发 → app_config 调 BSP_Range_SetLevel）。
 * s_range_level_req 由主循环写；adc_target_count 在每次测量序列开始
 * （BSP_Pulse_Start）时快照，保证采样进行中改档不影响本次采样。
 * S3 第二沿在 OC 回调中直接由同一快照 adc_target_count 计算，无需另存宽度。 */
static volatile uint8_t s_range_level_req = 1U;          /* 请求档位 1~3 */

void BSP_Pulse_Start(void)
{
  HAL_NVIC_DisableIRQ(TIM3_IRQn);

  /* 量程档位快照：本次测量序列的 ADC 目标点数由此固定。
   * 在 TIM2 ISR 内执行，主循环不会穿插；后续 BSP_ADCsamp_Start（S4 结束沿）
   * 与 TIM3 S3 第二沿回调均使用同一 adc_target_count。 */
  adc_target_count = (uint16_t)ADC_SAMPLE_BASE * (uint16_t)s_range_level_req;

  /*  强制停止定时器 */
  htim3.Instance->CR1 &= ~TIM_CR1_CEN;

  __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC4 | TIM_IT_CC1 | TIM_IT_CC3);

  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC4 | TIM_FLAG_CC1 | TIM_FLAG_CC3 | TIM_FLAG_UPDATE);

  tim3_s10_state = 2;   /* S10 常高，不参与 TIM3 中断 */
  tim3_s3_state = 0;
  tim3_s4_state = 0;

  /* 重置 HAL 内部状态，避免后续调用 HAL 函数时返回 BUSY*/
  htim3.State = HAL_TIM_STATE_READY;
  htim3.Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;

  __HAL_TIM_SET_COUNTER(&htim3, 0);

  /* S10 (PC9) 常高 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

  /* S3↑ @ S3_START_TICK(5900μs)；S4↓ @ S4_START_TICK(6350μs)，单位 2μs/tick */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, S3_START_TICK);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, S4_START_TICK);

  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1 | TIM_IT_CC3);

  TIM_CCxChannelCmd(htim3.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
  TIM_CCxChannelCmd(htim3.Instance, TIM_CHANNEL_3, TIM_CCx_ENABLE);

  htim3.Instance->CR1 |= TIM_CR1_CEN;

  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

void BSP_s7_pulse(void)
{
    HAL_NVIC_DisableIRQ(TIM4_IRQn);

    /* 1. 停定时器，关所有 CC 中断，清所有标志 */
    TIM4->CR1  &= ~TIM_CR1_CEN;
    TIM4->DIER &= ~(TIM_DIER_CC3IE | TIM_DIER_CC4IE);
    TIM4->SR    = 0;

    tim4_s7_state = 0;
    tim4_s9_state = 0;

    /* 2. CH4 (S9) 空闲：PWM2 + CCR4=ARR+1 → OCREF 始终 0，极性 LOW → 输出 HIGH（空闲） */
    TIM4->CCR4  = s7s9_delay_tick + s7_width_tick + 1U;
    TIM4->CCER |= TIM_CCER_CC4E;
    TIM4->CCER |= TIM_CCER_CC4P;

    /* 3. CH3 (S7) 延时正脉冲：PWM2，CCR3=延时，ARR=延时+脉宽
     *    PWM2: CNT>=CCR → OCREF=1；极性 HIGH → 输出 = OCREF
     *    0~延时: OCREF=0 → 输出 LOW（空闲）；延时~ARR: OCREF=1 → 输出 HIGH（脉冲） */
    TIM4->ARR   = s7s9_delay_tick + s7_width_tick;
    TIM4->CCR3  = s7s9_delay_tick;
    TIM4->CCER |= TIM_CCER_CC3E;
    TIM4->CCER &= ~TIM_CCER_CC3P;   /* 极性 HIGH */

    /* 4. 计数器清零 */
    TIM4->CNT   = 0;

    /* 5. 开 CC3 中断（CNT 达到 ARR 时触发更新中断，脉冲结束）
     *    注：PWM2 模式下用更新中断（ARR 溢出）标记脉冲结束更可靠 */
    TIM4->DIER |= TIM_DIER_UIE;

    htim4.Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
    htim4.State   = HAL_TIM_STATE_READY;

    /* 6. 启动定时器 */
    TIM4->CR1 |= TIM_CR1_CEN;

    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}


void BSP_s9_pulse(void)
{
    HAL_NVIC_DisableIRQ(TIM4_IRQn);

    /* 1. 停定时器，关所有 CC 中断，清所有标志 */
    TIM4->CR1  &= ~TIM_CR1_CEN;
    TIM4->DIER &= ~(TIM_DIER_CC3IE | TIM_DIER_CC4IE);
    TIM4->SR    = 0;

    tim4_s7_state = 0;
    tim4_s9_state = 0;

    /* 2. CH3 (S7) 空闲：PWM2 + CCR3=ARR+1 → OCREF 始终 0，极性 HIGH → 输出 LOW（空闲） */
    TIM4->CCR3  = s7s9_delay_tick + s9_width_tick + 1U;
    TIM4->CCER |= TIM_CCER_CC3E;
    TIM4->CCER &= ~TIM_CCER_CC3P;   /* 极性 HIGH */

    /* 3. CH4 (S9) 延时负脉冲：PWM2，CCR4=延时，ARR=延时+脉宽
     *    PWM2: CNT>=CCR → OCREF=1；极性 LOW → 输出 = !OCREF
     *    0~延时: OCREF=0 → 输出 HIGH（空闲）；延时~ARR: OCREF=1 → 输出 LOW（脉冲） */
    TIM4->ARR   = s7s9_delay_tick + s9_width_tick;
    TIM4->CCR4  = s7s9_delay_tick;
    TIM4->CCER |= TIM_CCER_CC4P;    /* 极性 LOW */
    TIM4->CCER |= TIM_CCER_CC4E;

    /* 4. 计数器清零 */
    TIM4->CNT   = 0;

    /* 5. 开更新中断（ARR 溢出 = 脉冲结束） */
    TIM4->DIER |= TIM_DIER_UIE;

    htim4.Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
    htim4.State   = HAL_TIM_STATE_READY;

    /* 6. 启动定时器 */
    TIM4->CR1 |= TIM_CR1_CEN;

    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

/* 设置 S7 脉宽（μs，钳位 2~20μs）：大偏差用宽脉冲加速收敛，精调固定 2μs */
void BSP_s7_set_width_us(uint16_t width_us)
{
    uint16_t tick = (uint16_t)(width_us * 10U);   /* μs → 0.1μs tick */
    if (tick < S7S9_MIN_PULSE_TICK) { tick = S7S9_MIN_PULSE_TICK; }
    if (tick > S7S9_MAX_PULSE_TICK) { tick = S7S9_MAX_PULSE_TICK; }
    s7_width_tick = tick;
}

/* 设置 S9 脉宽（μs，钳位 2~20μs） */
void BSP_s9_set_width_us(uint16_t width_us)
{
    uint16_t tick = (uint16_t)(width_us * 10U);   /* μs → 0.1μs tick */
    if (tick < S7S9_MIN_PULSE_TICK) { tick = S7S9_MIN_PULSE_TICK; }
    if (tick > S7S9_MAX_PULSE_TICK) { tick = S7S9_MAX_PULSE_TICK; }
    s9_width_tick = tick;
}


void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  /*---------------- TIM3 ----------------*/
  if (htim->Instance == TIM3)
  {
    switch (htim->Channel)
    {
    case HAL_TIM_ACTIVE_CHANNEL_1:
      /* S3↑@5900μs：第二沿 = S3_START_TICK + 本档位 S3 脉宽
       * 脉宽(tick) = adc_target_count×8(16μs/点) + 10000(20ms 宽裕)
       * 默认档 2950+18192=21142(42.28ms)；70m 档 2950+50960=53910(107.82ms) */
      if (tim3_s3_state == 0)
      {
        uint32_t s3_end_tick = S3_START_TICK
                             + (uint32_t)adc_target_count * S3_SAMPLE_TICK
                             + S3_WIDTH_MARGIN_TICK;
        tim3_s3_state = 1;
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, s3_end_tick);
      }
      else
      {
        __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
        TIM_CCxChannelCmd(htim->Instance, TIM_CHANNEL_1, TIM_CCx_DISABLE);
        tim3_s3_state = 2;
      }
      break;

    case HAL_TIM_ACTIVE_CHANNEL_3:
      /* S4↓@6350μs：第二沿 S4_END_TICK=3285(6570μs)，脉宽 220μs */
      if (tim3_s4_state == 0)
      {
        tim3_s4_state = 1;
        GPIOC->BSRR = (uint32_t)GPIO_PIN_8 << 16U;   /* S4↓ (PC8 LOW) */
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, S4_END_TICK);
      }
      else
      {
        GPIOC->BSRR = GPIO_PIN_8;   /* S4↑ (PC8 HIGH) */
        __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC3);
        TIM_CCxChannelCmd(htim->Instance, TIM_CHANNEL_3, TIM_CCx_DISABLE);
        tim3_s4_state = 2;
        /* S4 脉冲结束沿 → 启动 ADC（TIM7 驱动目标点数 SPI 采样）*/
        BSP_ADCsamp_Start();
      }
      break;
    
    default:
      break;
    }

    /* S3、S4 均完成（S10 常高无需等待）→ 停止 TIM3 */
    if (tim3_s3_state == 2 && tim3_s4_state == 2)
    {
      htim->Instance->CR1 &= ~TIM_CR1_CEN;
      htim->Instance->SR = 0;
      tim3_s3_state = 0;
      tim3_s4_state = 0;
    }
  }
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      if (f_ic_sta == 0)
      {
        /* 下降沿捕获（S6↓）：记录起始值，切换为上升沿捕获 */
        tim2_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        TIM_RESET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1);
        TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);
        f_ic_sta = 1;  /* 等待上升沿 */

        /* 如果 ADC 正在采样中，此下降沿 = 发射时刻 t₀
         * 记录当前已采样点数作为偏移量 */
        if (!adc_done && adc_sample_count > 0U)
        {
          tx_sample_offset = adc_sample_count;
          tx_sample_valid = 1;
        }

        /* S6↓ + 76μs 触发 S7/S9 校正脉冲（由 TIM4 硬件延时保证精度）
         * s7_remain/s9_remain 与脉宽由主循环在上一个 S6↑ 时设定；
         * 仅首脉冲用规范延时 76μs，连发脉冲在 TIM4 溢出 ISR 中改用短间隔；
         * 仅在 TIM4 空闲时启动，避免打断正在进行的脉冲序列。
         * 批次节流：每 2T 最多一批（样机 91.5ms/批）。每启动一批，把下一次
         * S3 测量「推到」本批次周期+3T（样机：最后批次+3T 才测量）——
         * 推到而非累加，s3_next_cycle ≤ 当前+3T 恒有界，连续校正不会失控。 */
        if ((TIM4->CR1 & TIM_CR1_CEN) == 0U)
        {
          /* 有符号比较：测量时锚点被设到 +1T，同周期 S6↓ 差值为 -1，
           * 无符号比较会下溢放行 → 必须用 int32_t */
          if (((int32_t)s6_cycle_cnt - (int32_t)s_last_batch_cycle) >= (int32_t)S7S9_BATCH_PERIOD_T)
          {
            uint8_t corr_started = 0U;

            if (s7_remain > 0U)
            {
              tim4_burst_type = 1U;
              s7s9_delay_tick = S7S9_DELAY_TICK;
              corr_started = 1U;
              BSP_s7_pulse();
            }
            else if (s9_remain > 0U)
            {
              tim4_burst_type = 2U;
              s7s9_delay_tick = S7S9_DELAY_TICK;
              corr_started = 1U;
              BSP_s9_pulse();
            }

            if (corr_started != 0U)
            {
              uint32_t push_cycle = s6_cycle_cnt + S3_BATCH_PUSH_T;
              s_last_batch_cycle = s6_cycle_cnt;
              if (s3_next_cycle < push_cycle)
              {
                s3_next_cycle = push_cycle;
              }
            }
          }
        }
      }
      else
      {
        /* 上升沿捕获（S6↑）：计算低电平脉宽，切换回下降沿 */
        uint32_t tim2_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        if (tim2_end >= tim2_start)
          f_ic_val = (tim2_end - tim2_start);
        else
          f_ic_val = (0xFFFFFFFFU - tim2_start + tim2_end + 1U);  /* TIM2 32位溢出处理 */
        f_ic_sta = 0;  /* 捕获完成，结果可读 */
        f_ic_new = 1;  /* 通知应用层：有新的脉宽值可读 */

        TIM_RESET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1);
        TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);

        /* S3 测量调度：每个 S6↑ 周期计数 +1，到达调度点才允许测量。
         * S3 两个一组（样机 0905 CSV）：组内 +4T，组间 +20T（组首到组首
         * 24T ≈ 1.1s）；每次校正批次由 S6↓ 分支把测量推到 批次+3T。 */
        s6_cycle_cnt++;

        /* 测量序列自动触发：脉宽进入目标窗口、上一轮 ADC 已结束、TIM3 空闲、
         * 到达调度点。在捕获 ISR 内直接启动 BSP_Pulse_Start
         * （S3↑ = 此时刻 + 5.9ms，样机 5.94ms，含 40μs 反应时间裕量），
         * 不经主循环，消除算法处理/调度延迟对 S3 相位的影响。
         * 注意：目标窗口 = [F_IC_LOWER_US, F_IC_UPPER_US]（见 bsp_timer.h，
         * 2026-09-20 确认 22875~23975，几何中心 23425）。上电未收敛时实测
         * 脉宽可达 ~28.4ms，严禁把阈值改成实测值（那只是尚未充电到位）。 */
        if ((!adc_active) &&
            ((TIM3->CR1 & TIM_CR1_CEN) == 0U) &&
            (s6_cycle_cnt >= s3_next_cycle) &&
            (f_ic_val >= F_IC_MEAS_LOWER_US) && (f_ic_val <= F_IC_MEAS_UPPER_US))
        {
          if (s3_pair_wait != 0U)
          {
            /* 本发是组内第二个：配对完成，+20T 安排下一组 */
            s3_pair_wait = 0U;
            s3_next_cycle = s6_cycle_cnt + S3_GROUP_GAP_T;
          }
          else
          {
            /* 本发是组内第一个：+4T 安排组内第二个 */
            s3_pair_wait = 1U;
            s3_next_cycle = s6_cycle_cnt + S3_PAIR_GAP_T;
          }
          /* 批次节流锚点：测量后首批校正在 +3T（样机行为，避免打扰测量周期） */
          s_last_batch_cycle = s6_cycle_cnt + 1U;
          BSP_Pulse_Start();
        }
      }
    }
  }
}

void BSP_ADCsamp_Start(void)
{
  HAL_NVIC_DisableIRQ(TIM7_IRQn);

  /* 停定时器，清标志 */
  __HAL_TIM_DISABLE(&htim7);
  __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
  __HAL_TIM_SET_COUNTER(&htim7, 0);

  /* 复位采样状态 */
  adc_sample_count = 0;
  adc_dma_busy     = 0;
  adc_done         = 0;
  adc_active       = 1;
  adc_error        = 0;
  adc_fail_count   = 0;

  /* 复位发射时刻记录（等待本次采样期间的 S6 下降沿）*/
  tx_sample_valid  = 0;
  tx_sample_offset = 0;

  /* 确保 SPI1 空闲（避免上一次未完成的 DMA 状态卡住）*/
  hspi1.State   = HAL_SPI_STATE_READY;
  hspi1.ErrorCode = HAL_SPI_ERROR_NONE;

  /* 开溢出中断并启动定时器 */
  __HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);
  __HAL_TIM_ENABLE(&htim7);

  HAL_NVIC_EnableIRQ(TIM7_IRQn);
}

/* ============ 量程档位 API（display DPARAM_RANGE_SETTING → app_config 调用）============ */
void BSP_Range_SetLevel(uint8_t level)
{
  /* 非法档位钳为 1（0~28m，1024 点），保证 adc_target_count 与缓冲区始终有效。
   * 最大倍数 ADC_SAMPLE_MULT_MAX=3（3072 点覆盖 92m，含 70m/84m 量程）。 */
  if ((level == 0U) || (level > ADC_SAMPLE_MULT_MAX))
  {
    level = 1U;
  }
  /* 只更新请求值；adc_target_count 在下一次 BSP_Pulse_Start（TIM2 S6↑ ISR）
   * 快照，采样进行中改档不会影响本次测量 */
  s_range_level_req = level;
}

uint8_t BSP_Range_GetLevel(void)
{
  return s_range_level_req;
}

uint16_t BSP_Range_GetSampleCount(void)
{
  /* 返回当前生效点数（采样完成后由算法层主循环读取，值在整个采样期稳定）*/
  return adc_target_count;
}

uint8_t BSP_ADCsamp_IsDone(void)
{
  return adc_done;
}

/* TIM4 / TIM7 溢出中断回调 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* ---------------- TIM4：S7/S9 校正脉冲结束（PWM2 更新中断 = 脉冲结束）---------------- */
  if (htim->Instance == TIM4)
  {
    __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
    htim->Instance->CR1 &= ~TIM_CR1_CEN;
    htim->Instance->SR   = 0;
    htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
    htim->State   = HAL_TIM_STATE_READY;

    /* S7/S9 连发：每发完一个 remain--，还有则用短间隔重发。
     * 只有首脉冲需要规范的 S6↓+76μs 延时；连发间隔 4μs 即可
     * （旧实现把 76μs 套用到每个连发脉冲，造成 ~80μs 的不必要间隔）。
     * 另一批次的 S9/S7 等待下一个 S6↓ 再启动，保证其首脉冲时序。 */
    if (tim4_burst_type == 1U)
    {
      if (s7_remain > 0U)
      {
        s7_remain--;
        if (s7_remain > 0U)
        {
          s7s9_delay_tick = S7S9_REPEAT_DELAY_TICK;
          BSP_s7_pulse();
        }
      }
      if (s7_remain == 0U)
      {
        tim4_burst_type = 0U;
      }
    }
    else if (tim4_burst_type == 2U)
    {
      if (s9_remain > 0U)
      {
        s9_remain--;
        if (s9_remain > 0U)
        {
          s7s9_delay_tick = S7S9_REPEAT_DELAY_TICK;
          BSP_s9_pulse();
        }
      }
      if (s9_remain == 0U)
      {
        tim4_burst_type = 0U;
      }
    }
    else
    {
      tim4_burst_type = 0U;
    }
    return;
  }

  /* ---------------- TIM7：ADC 采样循环 ---------------- */
  if (htim->Instance != TIM7)
  {
    return;
  }

  /* 已采满本档位目标点数（1024~3072）：停止 TIM7 */
  if (adc_sample_count >= adc_target_count)
  {
    __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
    __HAL_TIM_DISABLE(htim);
    adc_done   = 1;
    adc_active = 0;
    return;
  }

  /* ============ 阻塞版（当前启用）============
   * 单次约 14μs（CS 拉低 + 12.8μs SPI + CS 拉高），TIM7 周期 16μs，余 2μs。
   * 优先级（tim.c 实测，旧注释写"TIM7=2"是错的）：
   *       TIM2=0；TIM3=1；TIM4=1；TIM7=1 —— TIM7 与 TIM3/TIM4【同级】，
   *       同级不可互相抢占，只有 TIM2(0) 可抢占它们。
   *
   * ★失败保护的正确理解（2026-09-20 修正，勿再按旧注释理解）：
   *   连续 ADC_FAIL_MAX 次失败即停 TIM7 + 置 adc_error，避免 ISR 卡死 ——
   *   这一条只保证【不卡死】，【不保证测量仍可信】。
   *
   *   旧注释写"ADCS7476_Read 内部 Timeout=1ms"——错。实际是【2000 次轮询循环】
   *   （adcs7476.c:31 RXNE / :53 BSY），本工程 -O1 下单次约 160μs（150~250μs）。
   *
   *   关键后果：这 160μs 全花在 TIM7 的 ISR 里，而 TIM7 照常溢出、
   *   UIF 是【单标志】（期间溢出的十几次只留 1 个 pending），每次 ISR 又只写
   *   1 个 adc_buf 槽位 ⇒ 【一次失败白吃 ~10 个采样周期的时间，却只前进 1 个下标】。
   *   测距依赖"下标 ↔ 时间"线性对应（distance = (peak_idx - tx_offset) × 30.07mm），
   *   该点之后的下标就都比理想时刻晚了 ~160μs；若失败发生在发射时刻之后、
   *   回波峰之前，峰会记在偏早 ~10 点的下标上 ⇒ 【距离少算约 30cm】。
   *   而且这是静默错误：孤立失败不会置 adc_error，测量照样返回成功，只是给错值。
   *
   * ⟹ 因此"连续 3 次才报错"这个保护【覆盖不全】：真正需要挡的是【单次】失败。
   *   这一层由算法层补齐：按"失败点是否位于回波峰之前"做弃用判据（app_algorithm.c）。
   *
   * ★决策记录（2026-09-20，用户拍板 → 采用【方案甲】）：
   *   本层【维持 ADC_FAIL_MAX = 3，不做改动】。三个候选与本层结论：
   *     甲（已采用）：维持 3，由算法层精确判据弃用。
   *        ⇒ 失败点落在【回波峰之后】时，该次采样【仍然可用】，可用率最高。
   *          代价：坏采样会采完才被算法层丢弃，多花一点时间（无功能影响）。
   *     乙：降到 1，首次失败即停 TIM7 + 置 adc_error。
   *        ⇒ 故障信号明确（返回 3 而非 4），但会连"失败在峰之后、本可用"的采样一起丢掉。
   *     丙：失败时把被吃掉的周期数补进 adc_sample_count（跳过槽位填哨兵），
   *        使 下标↔时间 重新线性。
   *        ⇒ 最彻底（修好后连插值都可安全使用），但要动 16μs 周期的 ISR，风险最高。
   *   未采用乙/丙的理由：乙牺牲可用率换来的只是"信号更明确"，而算法层已能区分；
   *   丙虽最彻底，但要在 TIM7 ISR 内测经过时间，改动风险大于收益。
   *   ⚠ 甲方案下唯一的遗留副作用：失败点【之后】的时间轴仍然错位，
   *     这不影响距离（只取峰下标与 tx_offset），但会使【回波曲线后段】出现水平畸变
   *     （Disp_BuildEchoEx 直接读 adc_buf）。算法层已在该分支打印提示行。
   */
  if (ADCS7476_Read((uint16_t *)&adc_buf[adc_sample_count]) == HAL_OK)
  {
    adc_fail_count = 0;
    adc_sample_count++;
  }
  else
  {
    /* 失败也前进一位（避免卡在坏样本上），写入标记值 0xFFFF 供算法层识别。
     *
     * ★哨兵契约（§3.10 #C3，2026-09-20 已闭合）：写 0xFFFF 的同时，
     *   读取侧必须【全部】做剔除，否则 0xFFFF(65535) 会被当成
     *   "比 12 位满量程还大 16 倍的真实回波"，后果不是略微失真而是直接测不出：
     *   分路径看（2026-09-20 修正：早先笼统说"σ 被抬到数千"是不准确的）：
     *     · 【无基线降级路径】Algo_FindStrongestPeak 直接把 adc_buf 转 float
     *       送 Algo_FindPeaks → compute_stats 真的会把 σ 抬到数千、阈值 >4095
     *       → 所有真实峰不达阈值 → 检不到峰。这条成立。
     *     · 【量产路径】Algo_MeasureDistance 先做 cur-bas 差分写进
     *       s_measure_sm，哨兵在【进 compute_stats 之前】就已被处理掉，
     *       ⇒ "σ 爆掉"这条【不成立】，早先的估算是错的。
     *       该路径真正的危害是：① 基线被永久污染（下条）；② 更根本的是
     *       【时间轴断裂】——见本函数上方的失败保护说明，一次失败吃掉
     *       ~10 个采样周期却只前进 1 个下标，会让距离静默少算约 30cm。
     *     · 三个基线函数：0xFFFF 被永久烧进 s_baseline，
     *       后续每周期的差分恒为 ≈ -61440 并被截零 → 永久丢点、峰被扣掉；
     *     · app_disp.c  Disp_BuildCurveNorm：maxv 被抬到 65535
     *       → 整条回波曲线在屏上压成贴底平线。
     *   读取侧【均已处理】：插值修补 + 按"失败点是否位于回波峰之前"弃用
     *   （详见 app_algorithm.c 顶部说明块）。
     *   日后新增读取 adc_buf 的代码，必须同步处理哨兵。
     *   ⟹ 若改为其他哨兵值，务必全库 grep 0xFFFF 同步修改读取侧。
     *
     *   【另一条写入口：bsp_spi.c:55 —— 当前不可达】adc_buf 还有第二个生产者：
     *   HAL_SPI_RxCpltCallback 里的 `adc_buf[adc_sample_count] &= 0x0FFFU`（DMA 版）。
     *   但 DMA 路径整个处于 #if 0（见本文件下方 :616），且唯一把 adc_dma_busy 置 1
     *   的语句就在该 #if 0 内，故 adc_dma_busy 恒为 0、回调在 :45 立即返回，该写入
     *   永不执行。⚠ 注意两者语义不同：DMA 版【只掩码不写哨兵】——一旦将来恢复 DMA
     *   路径，失败样本不会被标记，算法层的剔除逻辑对它将【完全无效】，
     *   必须同步改造（在 DMA 失败分支补写 0xFFFFU）。*/
    adc_buf[adc_sample_count] = 0xFFFFU;
    adc_sample_count++;
    if (++adc_fail_count >= ADC_FAIL_MAX)
    {
      __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
      __HAL_TIM_DISABLE(htim);
      adc_done   = 1;
      adc_error  = 1;
      adc_active = 0;
    }
  }

  /* ============ DMA 异步版（已停用，保留备查）============
   * TIM7 周期 16μs 太短，DMA 单次循环约 7μs setup + 12.8μs SPI + 7μs complete
   * 共 ~27μs，比 TIM7 周期长，每次溢出都会撞上 adc_dma_busy=1 而跳过，
   * 实际只能跑到 ~31kHz，无法达到 62.5kHz。要恢复时取消下面注释并删上面阻塞版即可。
   */
#if 0
  if (adc_dma_busy)
  {
    return;
  }

  adc_dma_busy = 1;
  if (ADCS7476_Read_DMA((uint16_t *)&adc_buf[adc_sample_count]) != HAL_OK)
  {
    adc_dma_busy = 0;
  }
#endif
}