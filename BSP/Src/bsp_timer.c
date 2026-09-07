#include "bsp_timer.h"
#include "bsp_spi.h"
#include "adcs7476.h"
/*
  TIM2: 输入捕获 S6
  TIM3: S3 S4 S10
  TIM4: S7 S9 电容充放电
  TIM7: 溢出中断驱动 ADCS7476 采样（SPI1 + DMA，1024 点）
*/

/* ============ 时序常量（对应 L496时序.md 第6节）============ */
/* TIM3 @1MHz (1 tick = 1μs) */
/* S3↑ = S6↑ + 5.9ms（样机 5.94ms，留 40μs 反应时间；本固件在 S6↑ 捕获 ISR
 * 内直接启动 TIM3，ISR 延迟 ~2μs，S3↑ 实际 ≈ S6↑+5.902ms，落在窗口内） */
#define S3_START_US      5900U    /* S3↑ 时刻：S6↑ + 5.9ms */
#define S3_WIDTH_US     36000U    /* S3 脉宽：36ms */
#define S3_END_US       (S3_START_US + S3_WIDTH_US)   /* 41900μs */
#define S4_DELAY_US       450U    /* S4↓ 相对 S3↑ 的延时 */
#define S4_START_US      (S3_START_US + S4_DELAY_US)  /* 6350μs */
#define S4_WIDTH_US       220U    /* S4 脉宽：220μs（样机 206μs） */
#define S4_END_US        (S4_START_US + S4_WIDTH_US)  /* 6570μs */

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
#define S7S9_FINE_PULSE_TICK     20U  /* 精调脉宽 2.0μs（S7/S9 一致）*/
#define S7S9_WIDE_PULSE_TICK    200U  /* 粗调脉宽 20μs：上电大偏差时快速充电/放电 */
#define S7S9_MIN_PULSE_TICK      20U  /* 脉宽下限 2.0μs */
#define S7S9_MAX_PULSE_TICK     200U  /* 脉宽上限 20μs */
#define S7S9_BATCH_PERIOD_T      2U   /* 批次节流：每 2 个 S6 周期最多一批（样机 91.5ms/批）*/

volatile uint8_t f_ic_sta = 0;
volatile uint32_t f_ic_val = 0;
volatile uint8_t f_ic_new = 0;   /* 1=有新的 f_ic_val 可读 */

/* 发射时刻记录：ADC 采样期间 S6 下降沿对应的采样点序号 */
volatile uint8_t  tx_sample_valid = 0;    /* 1=本次采样已记录到发射时刻 */
volatile uint16_t tx_sample_offset = 0;   /* 发射时刻对应的 ADC 采样点序号（0~999）*/

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

/* ============ TIM5 精确定时延 ============ */
extern TIM_HandleTypeDef htim5;        /* CubeMX 生成（tim.c）*/



/* ============ TIM7 驱动 ADC 采样状态 ============ */
extern SPI_HandleTypeDef hspi1;     /* CubeMX 生成（spi.c）*/
extern TIM_HandleTypeDef htim7;    /* CubeMX 生成（tim.c）*/
extern TIM_HandleTypeDef htim2;    /* CubeMX 生成（tim.c），提供 TIM2 CNT 作为测距时间基准 */

volatile uint16_t adc_buf[ADC_SAMPLE_COUNT]; /* ADC 采样缓冲区（12-bit 有效值）*/
volatile uint16_t adc_sample_count = 0;      /* 已采样点数 */
volatile uint8_t  adc_dma_busy = 0;         /* 1=SPI1-DMA 接收进行中 */
volatile uint8_t  adc_done = 0;            /* 1=采样结束（成功满 1024 或失败中止）*/
volatile uint8_t  adc_error = 0;           /* 1=因连续 ADC_FAIL_MAX 次失败而中止 */
static volatile uint8_t adc_fail_count = 0; /* 连续失败计数（成功一次即清零）*/
static volatile uint8_t adc_active = 0;     /* 1=采样流程进行中（从 Start 到 done，供测量触发互斥）*/

void BSP_Pulse_Start(void)
{
  HAL_NVIC_DisableIRQ(TIM3_IRQn);

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

  /* S3↑ @ S3_START_US(5900μs)；S4↓ @ S4_START_US(6350μs) */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, S3_START_US);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, S4_START_US);

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
      /* S3↑ @5900μs：设置第二沿为 S3_END_US(41900μs)，脉宽 = 41900-5900 = 36000μs */
      if (tim3_s3_state == 0)
      {
        tim3_s3_state = 1;
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, S3_END_US);
      }
      else
      {
        __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
        TIM_CCxChannelCmd(htim->Instance, TIM_CHANNEL_1, TIM_CCx_DISABLE);
        tim3_s3_state = 2;
      }
      break;

    case HAL_TIM_ACTIVE_CHANNEL_3:
      /* S4↓ @6350μs：设置第二沿为 S4_END_US(6556μs)，脉宽 = 6556-6350 = 206μs */
      if (tim3_s4_state == 0)
      {
        tim3_s4_state = 1;
        GPIOC->BSRR = (uint32_t)GPIO_PIN_8 << 16U;   /* S4↓ (PC8 LOW) */
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, S4_END_US);
      }
      else
      {
        GPIOC->BSRR = GPIO_PIN_8;   /* S4↑ (PC8 HIGH) */
        __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC3);
        TIM_CCxChannelCmd(htim->Instance, TIM_CHANNEL_3, TIM_CCx_DISABLE);
        tim3_s4_state = 2;
        /* S4 脉冲结束沿 → 启动 ADC（TIM7 驱动 1024 点 SPI 采样）*/
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
         * 不经主循环/TIM5，消除算法处理/调度延迟对 S3 相位的影响。
         * 注意：目标中心 = 22875μs（22375/23375 ±500）。上电未收敛时实测
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

void BSP_TIM5_Delay_us(uint32_t delay_us)
{
  HAL_NVIC_DisableIRQ(TIM5_IRQn);

  /* 停定时器，清标志，清计数器 */
  __HAL_TIM_DISABLE(&htim5);
  __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
  __HAL_TIM_SET_COUNTER(&htim5, 0);

  /* 设置延时周期（ARR=delay_us - 1，最小延时 1μs）*/
  if (delay_us == 0U)
  {
    delay_us = 1U;
  }
  __HAL_TIM_SET_AUTORELOAD(&htim5, delay_us - 1U);

  /* 重置 HAL 状态，防止返回 BUSY */
  htim5.State   = HAL_TIM_STATE_READY;
  htim5.Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;

  /* 清标志 + 开溢出中断 + 启动 */
  __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);
  __HAL_TIM_ENABLE(&htim5);

  HAL_NVIC_EnableIRQ(TIM5_IRQn);
}

uint8_t BSP_ADCsamp_IsDone(void)
{
  return adc_done;
}

/* TIM4 / TIM5 / TIM7 溢出中断回调 */
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

  /* ---------------- TIM5：精确定时延（单次，到点置标志即停）---------------- */
  if (htim->Instance == TIM5)
  {
    __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
    __HAL_TIM_DISABLE(htim);
    BSP_Pulse_Start();
    return;
  }

  /* ---------------- TIM7：ADC 采样循环 ---------------- */
  if (htim->Instance != TIM7)
  {
    return;
  }

  /* 已采满 1024 点：停止 TIM7 */
  if (adc_sample_count >= ADC_SAMPLE_COUNT)
  {
    __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
    __HAL_TIM_DISABLE(htim);
    adc_done   = 1;
    adc_active = 0;
    return;
  }

  /* ============ 阻塞版（当前启用）============
   * 单次约 14μs（CS 拉低 + 12.8μs SPI + CS 拉高），TIM7 周期 16μs，余 2μs。
   * 优先级：TIM2=0 > TIM3/TIM4=1 > TIM7=2，故 TIM3/TIM4 可抢占 TIM7 ISR，
   *       S3/S4/S7/S9 的时序精度不受 ADC 采样阻塞影响。
   *       TIM2 IC 优先级更高（0,0）可抢占，不受影响。
   * 失败保护：连续 ADC_FAIL_MAX 次失败即停 TIM7 + 置 adc_error，避免 ISR 卡死。
   *           ADCS7476_Read 内部 Timeout=1ms，单次最坏 1ms，连续 3 次 = 3ms，
   *           远小于 TIM7 整个采样窗（16ms），不会拖垮系统。
   */
  if (ADCS7476_Read((uint16_t *)&adc_buf[adc_sample_count]) == HAL_OK)
  {
    adc_fail_count = 0;
    adc_sample_count++;
  }
  else
  {
    /* 失败也前进一位（避免卡在坏样本上），写入标记值供算法层识别并跳过 */
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