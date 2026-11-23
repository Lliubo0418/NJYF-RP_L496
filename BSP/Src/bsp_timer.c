#include "bsp_timer.h"
#include "bsp_spi.h"
#include "adcs7476.h"
/*
  TIM2: 输入捕获 S6
  TIM3: S3 S4 S10
  TIM4: S7 S9 电容充放电
  TIM7: 溢出中断驱动 ADCS7476 采样（SPI1 寄存器层阻塞读，1024~5120 点随量程档位变化）
*/

/* ============ 时序常量（对应 L496时序.md 第6节）============
 * ⚠ TIM3 取 2μs/tick（PSC=160-1 → 0.5MHz），不可改回 1μs/tick：16 位定时器在
 *   1μs/tick 下最大仅 65.5ms，覆盖不了最高档 S3 结束沿（新式 = 零点 + 5120×16μs ≈ 99ms）。 */
#define TIM3_TICK_US         2U     /* TIM3 计数单位（μs/tick）*/
#define US2TICK(us)          ((us) / TIM3_TICK_US)
#define S3_START_TICK        US2TICK(5900U)    /* S3↑ = S6↑+5.9ms */
#define S3_SAMPLE_TICK       US2TICK(16U)      /* 每个 ADC 点 16μs */
#define S4_START_TICK        US2TICK(6350U)    /* S4↓ = S3↑+450μs */
#define S4_END_TICK          US2TICK(6570U)    /* S4↑ 结束沿启动 ADC（S4 脉宽 220μs）*/

/* ============ 采样窗定位（S6↓ 捕获【实时触发】零点，不预测）============
 * S6 的下降沿本身就是 ADC 零点 t₀；捕获到 S6↓ 的那一拍 = adc_buf[0]。
 *   ① ADC 硬件起点仍是 S4↑（= S6↑+6.570ms），前导段照常转换但不入 adc_buf；
 *   ② 捕获到 S6↓：adc_lead_left 清零 ⇒ 下一拍起写 adc_buf[0]、tx_sample_offset = 0；
 *   ③ 同一次捕获把 S3↓ 重设为「当前 CNT + N×16μs」，S3 门精确罩住真实采样窗
 *      （S3↑ 仍固定 +5.9ms、早于 S4↑；S3↓ 初值只是兜底，真零点一到即被覆盖）。
 * ⚠ 严禁改成「用上一周期外推本周零点」：闭环被控量是【实测 S6 周期】，外推会让量值失真。
 * ⚠ S6 周期未测到(=0) 或未落周期窗时，不得启动任何测量。 */
#define S4_ADC_START_US       (S4_END_TICK * TIM3_TICK_US)  /* ADC 起点 = S6↑+6570μs */
#define ADC_SAMPLE_PERIOD_US  16U              /* TIM7 采样周期（62.5kHz）*/
/* ============ 前导上限（R-13：由固定 4096 点改为「N + 余量」）============
 * 前导 = S4↑(ADC 起点 6.570ms) → S6↓(零点) 之间丢弃的点数。到顶仍未等到 S6↓ ⇒ 零点漏捕。
 * 现方案：上限 = 本次采样点数 + ADC_LEAD_MARGIN_PTS，在 BSP_ADCsamp_Start 按 adc_target_count 装载：
 *     前导兜底绝对 = 6570 + (N+1024)×16µs = 22954 + 16N
 *     S3↓ 兜底     = 35000 + 16N      ⇒ 两者恒差 12.05ms（与档位无关），各档余量一致。
 * ⚠ 余量下限：等零点的最坏情况 = (S6_HIGH_MAX_US − S4_ADC_START_US)/16µs = 1777 点，
 *   故上限必须 ≥ 1777 点；本方案档1 = 2048 点，余 271 点（4.3ms）。改余量前先核这个下界。
 * ⚠ 上限 + 前导 必须仍在 TIM3 ARR(0xFFFF=131.07ms) 内：档5 = 104.87ms，余 26.2ms。*/
#define ADC_LEAD_MARGIN_PTS   1024U            /* 前导余量（点）= 16.384ms，各档统一 */
#define S3_ARR_MAX_TICK       0xFFFFU          /* TIM3 提到 16 位满量程（131ms@0.5MHz）：
                                                * 写进 ARR；CCR1 上限另见 S3_CCR1_MAX_TICK。*/
/* ★ 2026-10-27 修 R-12：CCR1 上限取 ARR−1，不用 ARR。
 *   钳到 ARR(65535) 时，CNT 65534→65535 那一拍产生 CC1 比较匹配、下一拍就是 UEV，
 *   两者只差 1 tick(2µs)；中断延迟一旦超过 2µs，CC1IF 与 UIF 会同时挂起。
 *   HAL_TIM_IRQHandler 的处理次序（先 CC 后 UPDATE）本身是安全的，但把 CCR1 压到 65534
 *   可以【结构性】消除这个同 tick 竞争：CC1 必定比 UEV 早 1 tick 置位。
 *   代价：S3 门提前 2µs 关闭（≈0.125 个采样点），可忽略。*/
#define S3_CCR1_MAX_TICK      (S3_ARR_MAX_TICK - 1U)
#define S6_HIGH_MAX_US        35000U           /* 上一周期实测「高电平段」(周期−低电平) 的上界。
                                                * ⚠ 只作【合理性判据】与 S3↓ 初值基准，不参与零点估计。*/

/* ============ S3 测量调度（S3 两个一组）============
 * 组内第二个 = 第一个 + 4T；组间下一组第一个 = 组内第二个 + 20T。
 * ⚠ 校正批次把下一次测量推到 本批次周期+3T，且必须取 max() 而非 += ：累加在连续
 *   校正时会越推越远而永不触发 BSP_Pulse_Start；max() 天然有界（next ≤ 当前+3T）。 */
#define S3_PAIR_GAP_T        4U   /* 组内间隔：+4T */
#define S3_GROUP_GAP_T      20U   /* 组间间隔：组内第二个 → 下一组第一个 +20T */
#define S3_BATCH_PUSH_T      3U   /* 校正批次把下一次测量推到 批次周期+3T */

/* TIM4 @10MHz (1 tick = 0.1μs) */
#define S7S9_DELAY_TICK         760U  /* 首脉冲延时：S6↓ + 76μs（规范固定，仅首脉冲）*/
#define S7S9_REPEAT_DELAY_TICK   40U  /* 连发间隔 4μs（连发不适用 76μs 延时）*/
/* ============ S7/S9 校正脉冲脉宽（TIM4 @10MHz，1 tick = 0.1μs）============
 * 唯一定义处：其他文件（app_radar.c 的 S7_S9_FINE_US/WIDE_US、bsp_timer.h）只准
 *   引用"20~40μs"结论，不准复述 tick 数值。
 * ⚠ 钳位必须是 20~40μs（200~400 tick），不可改回旧的 20~200 tick 上限：那样
 *   FINE(20μs)=200 tick 会正好落在旧上限被钳成上限值、WIDE(40μs)=400 tick 也
 *   被钳到 200 tick → 粗调与精调退化为同一脉宽，宽脉冲快充/快放失效，上电收敛极慢。 */
#define S7S9_FINE_PULSE_TICK     200U  /* 精调脉宽 20μs（S7/S9 一致）*/
#define S7S9_WIDE_PULSE_TICK    400U  /* 粗调脉宽 40μs：上电大偏差时快速充电/放电 */
#define S7S9_MIN_PULSE_TICK      200U  /* 脉宽下限 20μs */
#define S7S9_MAX_PULSE_TICK     400U  /* 脉宽上限 40μs */
#define S7S9_BATCH_PERIOD_T      2U   /* 批次节流：每 2 个 S6 周期最多一批 */

/* ★★ 2026-10-14 输出冻结修复
 *  现象：实测 S9 输出卡在低电平 126.9ms（把 S6 周期顶到 2.12xT，闭环用 S7 放电 12s 才拉回）。
 *  机理：旧实现靠「更新中断里手动 CR1&=~CEN 停表」结束脉冲，而有效电平窗只有脉宽(20~40us)；
 *        且 TIM4 与 TIM7 同为抢占优先级 1（同级不可互抢），TIM7 采样 ISR 单次阻塞约 14us
 *        ⇒ 停表指令常被推迟到 CNT 已越过 CCR 的有效窗内 ⇒ 输出被冻在该电平。
 *  修复（①为主、②兜底）：
 *    ① CR1.OPM = 单脉冲模式：硬件在 UEV 自动清 CEN，此时 CNT 已回 0 ⇒ CNT<CCR ⇒ 输出回空闲；
 *       全程不经中断 ⇒ 停表点确定，且脉宽精确等于 ARR-CCR。见 BSP_s7_pulse()/BSP_s9_pulse()。
 *    ② ISR 里先把 CCR3/CCR4 抬到 0xFFFF(>ARR) 再停表：PWM2 下 CNT<CCR 恒成立 ⇒ OCREF=0
 *       ⇒ CH3(S7,极性HIGH)输出 LOW、CH4(S9,极性LOW)输出 HIGH，均为空闲。见
 *       HAL_TIM_PeriodElapsedCallback() 的 TIM4 分支。
 *  ⚠ OC3PE/OC4PE 未置位（HAL_TIM_OC_ConfigChannel 不设预装载）⇒ CCR 写入立即进入比较单元。 */

volatile uint8_t f_ic_sta = 0;
volatile uint32_t f_ic_val = 0;
volatile uint8_t f_ic_new = 0;   /* 1=有新的 f_ic_val 可读 */

/* 测距起点记录：ADC 采样期间 S6 下降沿对应的采样点序号（即测距零点 t₀，非「发射时刻」）*/
volatile uint8_t  tx_sample_valid = 0;    /* 1=本次采样已记录到测距起点 */
volatile uint16_t tx_sample_offset = 0;   /* 测距起点对应的 ADC 采样点序号（0~5119）*/

/* ============ S6 周期测量（唯一被控量）============
 * TIM2 测【上升沿→上升沿】的 S6 周期：相邻两次 S6↑ 的捕获差值即周期（TIM2 @1μs/tick）。
 * 同一路捕获里，S6↓ 那一沿用于【实时触发】ADC 零点（见上），不再据此预测任何东西。
 * ★ 2026-10-13：不做任何范围钳位 —— 捕获到什么就是什么；唯一保护是「首帧不参与」
 *   （上电第一次 S6↑ 前面没有上升沿可比，算不出周期）。
 * ★ 2026-10-14：零点改由 S6↓ 捕获实时触发；f_ic_val（低电平）只用于闸门合理性判据与上报。 */
static volatile uint32_t s6_period_us    = 0U;   /* 上一周期实测周期（μs）；★ 0 = 尚未测到（无效）。
                                                  * 周期需相邻两个 S6↑ 才能算出 ⇒ 上电后首发的 S6↑
                                                  * 必然尚未就绪。S6 是前提条件，0 时调用方不得动作。
                                                  * 对外只读见 BSP_GetS6PeriodUs() */
static uint32_t          tim2_last_up_us = 0U;                     /* 上一 S6↑ 的 TIM2 计数值 */
static volatile uint16_t adc_lead_left   = ADC_LEAD_MARGIN_PTS; /* 前导剩余点数（TIM7 ISR 递减，归零后才写 adc_buf）。
                                                          * ★ 起点由 BSP_ADCsamp_Start 按 adc_target_count 装载为
                                                          *   「N + ADC_LEAD_MARGIN_PTS」（2026-10-27 修 R-13）；
                                                          *   捕获到 S6↓ 时被 TIM7 落实清零。此处初值仅为静态初始化。*/
static uint16_t          s_adc_scratch   = 0U;   /* 前导段丢弃槽（ADCS7476_Read 需要一个落点）*/

/* ★ 2026-10-22 修 R-10（ADC 零点切换中断竞争）：
 *   TIM2 = 抢占优先级 0，会打断 TIM7（优先级 1）的「读 adc_sample_count → 写 adc_buf[idx]
 *   → 自增」与「判断 adc_lead_left>0 → 自减」序列。旧实现在 TIM2 ISR 里直接清这两个量：
 *     ① 清零落在 TIM7 的"读"与"自增"之间 ⇒ 该拍样点写进【旧下标】、adc_buf[0] 不是零点拍
 *        （整体偏 1 点 ≈15mm），而 tx_sample_valid 仍为 1 ⇒ 静默错误；
 *     ② 清零落在"判断>0"与"自减"之间 ⇒ uint16_t 下溢为 65535，前导再等满一个上限
 *        （当前 = N+1024 点 ≈ 32~98ms）之后又延长，测量停摆约 1s 后用错位数据出一次错误结果。
 *   ⇒ 现在 TIM2 只置本标志（单字节写、原子），由 TIM7 在【自身上下文】里落实清计数/清前导，
 *     与写入序列同属一个不被 TIM2 打断的区间。代价：零点最多晚 1 拍（16μs ≈ 15mm）落实，
 *     与旧实现本来存在的 ±1 拍捕获抖动同量级。 */
static volatile uint8_t  adc_zero_pending = 0U;  /* 1=S6↓ 零点沿已捕获，待 TIM7 落实 */

static uint32_t tim2_start = 0;
static uint32_t tim2_end = 0;
static uint32_t tim2_pulse = 0;

static uint8_t tim3_s3_state = 0;
static uint8_t tim3_s8_state = 0;
static uint8_t tim3_s4_state = 0;
static uint8_t tim3_s10_state = 0;

/* ★ 2026-10-27 修 R-12：TIM3 序列超期自愈。
 * tim3_uev_stuck = 连续「UEV 到来时 CEN 仍为 1、且期间没有任何比较事件」的次数；
 * 达到 2 即判定序列已超期（必有比较沿丢失），强制收尾。任何比较事件都会把它清零。 */
static volatile uint8_t  tim3_uev_stuck   = 0U;
volatile uint16_t        tim3_recover_cnt = 0U;   /* 累计强制收尾次数（主循环据此打告警）*/
static uint8_t tim4_s7_state = 0;
static uint8_t tim4_s9_state = 0;

volatile uint8_t s7_remain = 0;
volatile uint8_t s9_remain = 0;

/* ============ S7/S9 连发超时兜底（2026-10-22 修 R-11）============
 * remain 只在 TIM4 更新中断里递减；若某个 UEV 丢失、或 tim4_burst_type 与实际
 * remain 不一致导致走不到递减分支，remain 永不归零，而主循环
 * `if (s7_remain || s9_remain) return;`（app_radar.c）会【永久】阻止新的校正决策
 * ⇒ AGC 闭环停摆，S6 周期漂出窗口后测量序列不再启动（只能复位）。
 * 兜底：主循环每轮调 BSP_S7S9_CheckTimeout()，超过阈值未收尾即强制清零并停表。
 * ⚠ 阈值必须远大于一批脉冲的真实耗时（≤50 发 × (40μs 脉宽 + 4μs 间隔) ≈ 2.2ms），
 *   又要远小于"闭环卡死"的可容忍时长 ⇒ 取 200ms（≈4.5 个 S6 周期）。
 * ⚠ 打点时机：主循环设脉宽时（BSP_s7/s9_set_width_us，紧跟其后就置 remain）与
 *   TIM2 实际起发时各打一次，避免"刚置 remain、还没到 S6↓"就被误判超时。 */
#define S7S9_TIMEOUT_MS   200U
static volatile uint32_t s7s9_arm_tick = 0U;

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

volatile uint16_t adc_buf[ADC_SAMPLE_COUNT_MAX]; /* ADC 采样缓冲区（12-bit 有效值，按最大 5120 点分配）*/
volatile uint16_t adc_sample_count = 0;      /* 已采样点数 */
volatile uint16_t adc_target_count = ADC_SAMPLE_BASE; /* 本次采样目标点数（BSP_Pulse_Start 快照，TIM7 ISR 读）*/
volatile uint8_t  adc_dma_busy = 0;         /* 1=SPI1-DMA 接收进行中 */
volatile uint8_t  adc_done = 0;            /* 1=采样结束（成功采满 adc_target_count 或失败中止）*/
volatile uint8_t  adc_error = 0;           /* 1=因连续 ADC_FAIL_MAX 次失败而中止 */
volatile uint8_t  adc_lead_expired = 0;    /* ★ 1=前导兜底耗尽仍未捕获 S6↓（零点漏捕，R-13）。
                                            * 与 adc_error 同时为 1（都属于"本次数据不可用"），
                                            * 但排查方向完全不同：这是 S6↓ 没进来，不是 ADC 坏了。
                                            * 由 BSP_ADCsamp_Start 清零。*/
static volatile uint8_t adc_fail_count = 0; /* 连续失败计数（成功一次即清零）*/
volatile uint8_t adc_active = 0;            /* 1=采样流程进行中（从 Start 到 done，供测量触发互斥；bsp_spi.c DMA 回调也会清零）*/

/* 量程档位（display 经 DPARAM_RANGE_SETTING → app_config 调 BSP_Range_SetLevel）：
 * ⚠ s_range_level_req 由主循环写；adc_target_count 在 BSP_Pulse_Start 快照，
 *   采样进行中改档不影响本次采样（S3 第二沿也用同一快照）。 */
static volatile uint8_t s_range_level_req = 1U;          /* 请求档位 1~5（1 档 = 1024 点 ≈ 15.4m）*/

/* ★ 2026-10-27 修 R-12：把 TIM3 恢复到「一次正常测量结束后的样子」。
 * 为什么需要它：CEN 只由「S3/S4 两沿都走完」清除，没有任何超时路径 ⇒ 只要丢一个比较沿，
 *   CEN 永为 1，而捕获 ISR 里 (TIM3->CR1 & CEN)==0 的启动闸门从此恒假 ⇒ 永久停测（无 IWDG，
 *   只能复位）。见 HAL_TIM_PeriodElapsedCallback() 的 TIM3 分支。
 * ⚠ 输出电平必须还原到「空闲」，不能就地冻结：
 *   OC TOGGLE 模式下每个正常序列恰好翻转 2 次（偶数），结束时 OCxRef 回到启动前的值（复位值 0）。
 *   超期收尾时若少翻/多翻一次，下一次 BSP_Pulse_Start 的第一沿会反相 ⇒ S3/S4 门整体颠倒，
 *   而且不报错 —— 比停测更危险。故此处用「临时 force inactive(OCxM=100) 把 OCxRef 强制回 0，
 *   随即恢复 toggle(OCxM=011)」来还原，与翻转次数无关、立即生效、不依赖比较事件。
 *   这是本文件唯一的 CCMR 写入，只动 OC1M/OC3M 低 3 位，其余位原样保留。
 *   （与 TIM4 的 PWM2 无关：CCMR2 的 PWM2 配置由 HAL_TIM_OC_ConfigChannel 一次性写入。）
 * ⚠ 不动 ADC（TIM7 / adc_*）：采样由 S4↑ 启动、按点数自然结束，零点由 TIM2 捕获，
 *   都与 TIM3 无关 ⇒ 本次数据仍可正常处理，不丢弃。
 * ⚠ 调用上下文：TIM3 中断内，或已 HAL_NVIC_DisableIRQ(TIM3_IRQn) 的位置。 */
static void Tim3_ForceIdle(void)
{
  htim3.Instance->CR1 &= ~TIM_CR1_CEN;

  __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC1 | TIM_IT_CC3);

  /* OCxRef 强制回空闲（0）：先 force inactive，再恢复 toggle 模式（OCxRef 保持不变）*/
  TIM3->CCMR1 = (uint16_t)((TIM3->CCMR1 & ~(7U << 4)) | (4U << 4));
  TIM3->CCMR1 = (uint16_t)((TIM3->CCMR1 & ~(7U << 4)) | (3U << 4));
  TIM3->CCMR2 = (uint16_t)((TIM3->CCMR2 & ~(7U << 4)) | (4U << 4));
  TIM3->CCMR2 = (uint16_t)((TIM3->CCMR2 & ~(7U << 4)) | (3U << 4));

  /* 与正常结束路径一致：关两路输出 + 引脚回空闲 + 清状态 */
  TIM_CCxChannelCmd(htim3.Instance, TIM_CHANNEL_1, TIM_CCx_DISABLE);
  TIM_CCxChannelCmd(htim3.Instance, TIM_CHANNEL_3, TIM_CCx_DISABLE);

  GPIOC->BSRR = (uint32_t)GPIO_PIN_2 << 16U;   /* PC2(IO4) 测量预告拉低 */
  GPIOC->BSRR = GPIO_PIN_8;                    /* S4(PC8) 回空闲高 */

  htim3.Instance->SR = 0U;                     /* 清 UIF/CC1IF/CC3IF，防中断风暴 */

  tim3_s3_state  = 0U;
  tim3_s4_state  = 0U;
  tim3_uev_stuck = 0U;
}

void BSP_Pulse_Start(void)
{
  HAL_NVIC_DisableIRQ(TIM3_IRQn);

  /* 量程档位快照：本次测量序列的 ADC 目标点数由此固定（后续 BSP_ADCsamp_Start
   * 与 TIM3 S3 第二沿回调共用同一 adc_target_count）。
   * 公式唯一定义在 BSP_Range_GetRequestSampleCount()，此处只做赋值。 */
  adc_target_count = BSP_Range_GetRequestSampleCount();

  htim3.Instance->CR1 &= ~TIM_CR1_CEN;

  __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC4 | TIM_IT_CC1 | TIM_IT_CC3);

  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC4 | TIM_FLAG_CC1 | TIM_FLAG_CC3 | TIM_FLAG_UPDATE);

  tim3_s10_state = 2;   /* S10 常高，不参与 TIM3 中断 */
  tim3_s3_state = 0;
  tim3_s4_state = 0;
  tim3_uev_stuck = 0;   /* R-12：新序列重新计"无进展 UEV"次数 */

  /* 重置 HAL 内部状态，避免后续调用 HAL 函数时返回 BUSY*/
  htim3.State = HAL_TIM_STATE_READY;
  htim3.Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;

  /* ★ TIM3 ARR 提到 16 位满量程（131ms）：S6↓ 重设 S3↓ 走「当前 CNT + N×16μs」，
   * 档5 稳态 ≈ 零点(22.1ms) + 5120×16μs = 104.0ms ⇒ 52010 tick；S3↑ 兜底值（按零点上界
   * 35ms 起算）= 58460 tick。两者虽都还在原 ARR 59999(120ms) 之内，但 58460 仅剩 2.6% 余量，
   * 且 S6_HIGH_MAX_US 后续还可能放宽，故直接提到满量程把余量做足。
   * ⚠ ARR 有预装载（AutoReloadPreload=ENABLE），必须打 UG 让它立即生效；UG 同时把
   *   CNT 复零，与下一行 SET_COUNTER(0) 同向，无害。UG 顺带置了 UIF ——
   *   ★ 2026-10-27 起 TIM3 的 UIE 是开着的（R-12 自愈窗口），所以开 CEN 之前必须清掉它，
   *     见下方 __HAL_TIM_CLEAR_FLAG。 */
  htim3.Instance->ARR = S3_ARR_MAX_TICK;
  htim3.Instance->EGR = TIM_EGR_UG;

  __HAL_TIM_SET_COUNTER(&htim3, 0);

  /* S10 (PC9) 常高 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

  /* S3↑ @ S3_START_TICK(5900μs)；S4↓ @ S4_START_TICK(6350μs)，单位 2μs/tick */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, S3_START_TICK);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, S4_START_TICK);

  /* ★ 2026-10-27 修 R-12：顺带开更新中断，作为「序列没能在 1 个 ARR 周期内收尾」的自愈窗口。
   * ⚠ 上面 EGR=UG 会立刻置 UIF，必须在这里先清掉：否则 HAL_NVIC_EnableIRQ 那一刻就进一次 UEV，
   *   被自愈逻辑当成"卡死"、把刚启动的序列当场收掉（这正是 R-12 最容易踩的坑）。 */
  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1 | TIM_IT_CC3 | TIM_IT_UPDATE);

  TIM_CCxChannelCmd(htim3.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
  TIM_CCxChannelCmd(htim3.Instance, TIM_CHANNEL_3, TIM_CCx_ENABLE);

  htim3.Instance->CR1 |= TIM_CR1_CEN;

  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

void BSP_s7_pulse(void)
{
    HAL_NVIC_DisableIRQ(TIM4_IRQn);

    TIM4->CR1  &= ~TIM_CR1_CEN;
    TIM4->DIER &= ~(TIM_DIER_CC3IE | TIM_DIER_CC4IE);
    TIM4->SR    = 0;

    tim4_s7_state = 0;
    tim4_s9_state = 0;

    /* CH4 (S9) 空闲：PWM2 + CCR4=ARR+1 → OCREF 恒 0，极性 LOW → 输出 HIGH（空闲）*/
    TIM4->CCR4  = s7s9_delay_tick + s7_width_tick + 1U;
    TIM4->CCER |= TIM_CCER_CC4E;
    TIM4->CCER |= TIM_CCER_CC4P;

    /* CH3 (S7) 延时正脉冲：PWM2 + 极性 HIGH —— 0~延时输出 LOW（空闲），
     * 延时~ARR 输出 HIGH（脉冲）*/
    TIM4->ARR   = s7s9_delay_tick + s7_width_tick;
    TIM4->CCR3  = s7s9_delay_tick;
    TIM4->CCER |= TIM_CCER_CC3E;
    TIM4->CCER &= ~TIM_CCER_CC3P;   /* 极性 HIGH */

    TIM4->CNT   = 0;

    /* 开更新中断（ARR 溢出 = 脉冲结束）；PWM2 下比 CC 中断更可靠 */
    TIM4->DIER |= TIM_DIER_UIE;

    htim4.Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
    htim4.State   = HAL_TIM_STATE_READY;

    /* ★① 单脉冲模式：CEN=1 后硬件数到 UEV 自动清 CEN（CNT->0 ⇒ CNT<CCR ⇒ 输出回空闲），
     * 不再依赖中断及时停表。OPM 位常驻 CR1，故每次 CEN=1 只发一发。 */
    TIM4->CR1 |= TIM_CR1_OPM;
    TIM4->CR1 |= TIM_CR1_CEN;

    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}


void BSP_s9_pulse(void)
{
    HAL_NVIC_DisableIRQ(TIM4_IRQn);

    TIM4->CR1  &= ~TIM_CR1_CEN;
    TIM4->DIER &= ~(TIM_DIER_CC3IE | TIM_DIER_CC4IE);
    TIM4->SR    = 0;

    tim4_s7_state = 0;
    tim4_s9_state = 0;

    /* CH3 (S7) 空闲：PWM2 + CCR3=ARR+1 → OCREF 恒 0，极性 HIGH → 输出 LOW（空闲）*/
    TIM4->CCR3  = s7s9_delay_tick + s9_width_tick + 1U;
    TIM4->CCER |= TIM_CCER_CC3E;
    TIM4->CCER &= ~TIM_CCER_CC3P;   /* 极性 HIGH */

    /* CH4 (S9) 延时负脉冲：PWM2 + 极性 LOW —— 0~延时输出 HIGH（空闲），
     * 延时~ARR 输出 LOW（脉冲）*/
    TIM4->ARR   = s7s9_delay_tick + s9_width_tick;
    TIM4->CCR4  = s7s9_delay_tick;
    TIM4->CCER |= TIM_CCER_CC4P;    /* 极性 LOW */
    TIM4->CCER |= TIM_CCER_CC4E;

    TIM4->CNT   = 0;

    /* 开更新中断（ARR 溢出 = 脉冲结束）*/
    TIM4->DIER |= TIM_DIER_UIE;

    htim4.Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
    htim4.State   = HAL_TIM_STATE_READY;

    /* ★① 单脉冲模式：见 BSP_s7_pulse() 同处说明 */
    TIM4->CR1 |= TIM_CR1_OPM;
    TIM4->CR1 |= TIM_CR1_CEN;

    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

/* 设置 S7 脉宽（μs，钳位 20~40μs）：大偏差用 40μs 宽脉冲加速收敛，精调用 20μs */
void BSP_s7_set_width_us(uint16_t width_us)
{
    /* ⚠ 2026-10-22：中间量用 uint32_t，先算后钳位。原写法 `(uint16_t)(width_us*10U)`
     *   在 width_us ≥ 6554 时先在 32 位域回绕再截断（65540 → 4），钳位方向反转（R-14）。 */
    uint32_t t = (uint32_t)width_us * 10U;        /* μs → 0.1μs tick */
    if (t < S7S9_MIN_PULSE_TICK) { t = S7S9_MIN_PULSE_TICK; }
    if (t > S7S9_MAX_PULSE_TICK) { t = S7S9_MAX_PULSE_TICK; }
    s7_width_tick  = (uint16_t)t;
    s7s9_arm_tick  = HAL_GetTick();   /* 超时兜底计时起点（R-11）*/
}

/* 设置 S9 脉宽（μs，钳位 20~40μs） */
void BSP_s9_set_width_us(uint16_t width_us)
{
    uint32_t t = (uint32_t)width_us * 10U;        /* μs → 0.1μs tick（理由同 BSP_s7_set_width_us）*/
    if (t < S7S9_MIN_PULSE_TICK) { t = S7S9_MIN_PULSE_TICK; }
    if (t > S7S9_MAX_PULSE_TICK) { t = S7S9_MAX_PULSE_TICK; }
    s9_width_tick  = (uint16_t)t;
    s7s9_arm_tick  = HAL_GetTick();   /* 超时兜底计时起点（R-11）*/
}

/* S7/S9 连发超时检查（主循环调用，见文件内 S7S9_TIMEOUT_MS 说明）。
 * 返回 0 = 未超时/无需处理，1 = 本次已强制清零（调用方可据此打印告警）。 */
uint8_t BSP_S7S9_CheckTimeout(void)
{
    if ((s7_remain == 0U) && (s9_remain == 0U))
    {
        return 0U;
    }
    if ((uint32_t)(HAL_GetTick() - s7s9_arm_tick) < S7S9_TIMEOUT_MS)
    {
        return 0U;
    }

    /* 强制收敛：停表 + 两路输出回空闲电平（与 TIM4 UEV 分支同一收尾动作）*/
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
    TIM4->CR1  &= ~TIM_CR1_CEN;
    TIM4->DIER &= ~TIM_DIER_UIE;
    TIM4->CCR3  = 0xFFFFU;   /* PWM2 + CCR>ARR ⇒ OCREF=0 ⇒ CH3(S7) LOW  = 空闲 */
    TIM4->CCR4  = 0xFFFFU;   /* 同上 ⇒ CH4(S9) HIGH = 空闲 */
    TIM4->SR    = 0U;
    htim4.Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
    htim4.State   = HAL_TIM_STATE_READY;
    HAL_NVIC_EnableIRQ(TIM4_IRQn);

    s7_remain       = 0U;
    s9_remain       = 0U;
    tim4_burst_type = 0U;
    tim4_s7_state   = 0U;
    tim4_s9_state   = 0U;
    s7s9_arm_tick   = HAL_GetTick();
    return 1U;
}


void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  /*---------------- TIM3 ----------------*/
  if (htim->Instance == TIM3)
  {
    switch (htim->Channel)
    {
    case HAL_TIM_ACTIVE_CHANNEL_1:
      tim3_uev_stuck = 0;   /* R-12：本序列有比较事件进展 ⇒ 清"无进展"计数 */
      /* S3↑ → 先装载 S3↓ 初值。★ S3↓ = 零点 + 点数×16μs，真零点由 S6↓ 捕获实时给出。
       * 采样窗 = [S4↑(ADC 起点), S3↓]，S3 门恰好把整窗罩住（S3↑ 仍固定 +5.9ms，早于 S4↑）。
       * ⚠ 这里只是【兜底初值】：S6↓ 捕获（TIM2 ISR）随即用「当前 CNT + N×16μs」覆盖它。
       *   初值取「零点上界 + N×16μs」，保证不会早于真零点就关断（否则采样窗被截短）。 */
      if (tim3_s3_state == 0)
      {
        uint32_t s3_end_tick = US2TICK(S6_HIGH_MAX_US)
                             + (uint32_t)adc_target_count * S3_SAMPLE_TICK;
        if (s3_end_tick > (uint32_t)S3_CCR1_MAX_TICK) { s3_end_tick = S3_CCR1_MAX_TICK; }
        tim3_s3_state = 1;
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, s3_end_tick);
      }
      else
      {
        /* ★ S3↓（CH1 第二沿）：拉低 PC2(IO4)。PC2 由此与 S3 门同宽
         * （起点 = S6↓、提前 S3↑ 约 35ms；终点 = 本处 S3↓）。
         * 用 BSRR 高 16 位写 = 原子清位，1 条指令，与 S4 的写法一致。 */
        GPIOC->BSRR = (uint32_t)GPIO_PIN_2 << 16U;   /* PC2 拉低 */
        __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
        TIM_CCxChannelCmd(htim->Instance, TIM_CHANNEL_1, TIM_CCx_DISABLE);
        tim3_s3_state = 2;
      }
      break;

    case HAL_TIM_ACTIVE_CHANNEL_3:
      tim3_uev_stuck = 0;   /* R-12：本序列有比较事件进展 ⇒ 清"无进展"计数 */
      /* S4↓：S4 第二沿 = S4_END_TICK（S4 脉宽 220μs）*/
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

        /* ADC 采样中：此下降沿【就是】测距零点 t₀ —— 实时触发，不做任何预测。
         * ① 结束前导：adc_lead_left = 0 ⇒ 下一拍 TIM7 起写 adc_buf[0]，即零点那一拍；
         * ② tx_sample_offset = 0（零点 = adc_buf[0]，无需再按偏移量纠）；
         * ③ 同一次捕获把 S3↓ 重设为「当前 CNT + N×16μs」⇒ S3 门精确罩住真实采样窗。
         * ⚠ 只认【本次采样窗内的第一个 S6↓】：档5 采样跨周期（N = 5120 点 × 16μs = 81.9ms > 1 个
         *   S6 周期 44.57ms），窗内还会撞到第二个 S6↓，不加 tx_sample_valid 门会被覆盖成错误偏移。
         * ⚠ TIM3 CNT 为 2μs/tick ⇒ N×16μs = N×8 tick。OC TOGGLE 模式下 CCxPE=0，
         *   直接写 CCR1 立即生效（无预装载）。 */
        if (adc_active && !adc_done && !tx_sample_valid)
        {
          /* ★ 2026-10-22 R-10：本 ISR 抢占优先级 0，不得直接清 adc_lead_left /
           *   adc_sample_count（会与 TIM7 的读-写-自增 / 判断-自减交错）。只置标志，
           *   由 TIM7 在自身上下文落实，详见 adc_zero_pending 定义处说明。 */
          adc_zero_pending = 1U;   /* ★ 立即结束前导（由 TIM7 落实，最晚 1 拍） */
          tx_sample_offset = 0U;   /* 零点 = adc_buf[0] */
          tx_sample_valid  = 1U;   /* 门住窗内的第二个 S6↓；TIM7 不写此变量，无竞争 */

          if (((TIM3->CR1 & TIM_CR1_CEN) != 0U) && (tim3_s3_state == 1U))
          {
            uint32_t s3_end_tick = (uint32_t)TIM3->CNT
                                 + (uint32_t)adc_target_count * S3_SAMPLE_TICK;
            TIM3->CCR1 = (s3_end_tick > (uint32_t)S3_CCR1_MAX_TICK)
                       ? (uint16_t)S3_CCR1_MAX_TICK : (uint16_t)s3_end_tick;
          }
        }

        /* S6↓+76μs 触发 S7/S9 校正脉冲（TIM4 硬件延时）：仅在 TIM4 空闲时启动，避免打断
         * 进行中的脉冲序列；批次节流每 2T 一批，⚠ 并把下次 S3 测量按 max 推到 本批+3T。 */
        if ((TIM4->CR1 & TIM_CR1_CEN) == 0U)
        {
          /* ⚠ 必须用 int32_t 有符号比较：锚点可被设到 +1T，同周期差值为 -1，
           *   无符号比较会下溢放行 */
          if (((int32_t)s6_cycle_cnt - (int32_t)s_last_batch_cycle) >= (int32_t)S7S9_BATCH_PERIOD_T)
          {
            uint8_t corr_started = 0U;

            s7s9_arm_tick = HAL_GetTick();   /* 超时兜底：实际起发时续期（R-11）*/

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

        /* ★ PC2(IO4) 预拉高 —— 送给外部电路的「测量预告」信号。
         * 相位：本次 S6↓ 到「下一个 S6↑ 启动的 S3↑」= 低电平段 + 5.9ms ≈ 35ms，
         *       即用户口径「每次 S3 拉高前约 35ms 拉高 PC2」。
         * 拉低由 S3↓（TIM3 CH1 第二沿，见 HAL_TIM_OC_DelayElapsedCallback）负责。
         * ⚠ 这里是【预判】：只看调度点与周期窗，不看 adc_active/TIM3 忙 —— 那两个条件要等
         *   下一个 S6↑ 才知道。若那个 S6↑ 最终没启动 S3，由上升沿分支的兜底把 PC2 拉低，
         *   不会卡高。用 BSRR 直接写（1 条指令），满足中断内耗时上限。 */
        if ((s6_period_us != 0U) &&
            (s6_period_us >= F_IC_PERIOD_MEAS_LOWER_US) && (s6_period_us <= F_IC_PERIOD_MEAS_UPPER_US) &&
            ((s6_cycle_cnt + 1U) >= s3_next_cycle))
        {
          GPIOC->BSRR = GPIO_PIN_2;   /* PC2 拉高 */
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

        /* 上一 S6 周期（μs）：TIM2 @1μs/tick ⇒ 相邻两次 S6↑ 的捕获差值即周期。
         * 用途：① 测量触发闸门（周期窗，见下）；② 应用层 S7/S9 闭环被控量
         *   （app_radar.c 经 BSP_GetS6PeriodUs()）。**不用于预测零点** —— 零点由 S6↓ 捕获给出。
         * ★ 2026-10-13：**不做任何范围钳位** —— 捕获到什么就是什么（用户口径「完全不过滤」）。
         *   唯一的保护是「首帧不参与」：上电第一次 S6↑ 前面没有上升沿可比，算不出周期。 */
        if (tim2_last_up_us != 0U)
        {
          s6_period_us = (tim2_end >= tim2_last_up_us)
                       ? (tim2_end - tim2_last_up_us)
                       : (0xFFFFFFFFU - tim2_last_up_us + tim2_end + 1U);
        }
        tim2_last_up_us = tim2_end;

        TIM_RESET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1);
        TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);

        /* S6↑ 周期计数 +1；组内 +4T、组间 +20T，校正批次由 S6↓ 分支推后（见上） */
        s6_cycle_cnt++;

        /* 触发条件（★ 2026-10-13：闸门由【S6 低电平窗】改为【S6 周期窗】）：
         * ① S6 周期落 [F_IC_PERIOD_MEAS_LOWER_US, F_IC_PERIOD_MEAS_UPPER_US]（周期是 K 的唯一
         *    依据，必须达标）；
         * ② 上一周期实测「高电平段」(周期−低电平) 落 [S4_ADC_START_US, S6_HIGH_MAX_US]
         *    （★ 合理性判据、不是预测：保证 S6↓ 零点确实落在 S4↑ 之后、采样窗有效）；
         * ③ ADC 已结束、TIM3 空闲、到达调度点 → 捕获 ISR 内直接 BSP_Pulse_Start；
         * ⚠ 阈值严禁改成上电实测值。
         * ⚠ 不再用 f_ic_val（低电平）当闸门：低电平 = 周期 × 占空比，占空比不受控，
         *   与周期窗并存会隐含「占空比 ≈50.5%」，实测不符时两窗互相矛盾、测量永不启动。
         * ⚠ 闸门①同时挡住【上电首周期】：那时 s6_period_us 尚未测到(=0)、f_ic_val 也为 0，
         *   0 < 周期窗下界即失败，不会误启动测量。 */
        if ((!adc_active) &&
            ((TIM3->CR1 & TIM_CR1_CEN) == 0U) &&
            (s6_cycle_cnt >= s3_next_cycle) &&
            (s6_period_us != 0U) &&  /* 0 = 尚未测到周期：S6 是前提，未就绪不得启动任何测量 */
            (s6_period_us >= F_IC_PERIOD_MEAS_LOWER_US) && (s6_period_us <= F_IC_PERIOD_MEAS_UPPER_US) &&
            ((s6_period_us - f_ic_val) >= S4_ADC_START_US) &&
            ((s6_period_us - f_ic_val) <= S6_HIGH_MAX_US))
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
          /* 批次节流锚点：测量后首批校正在 +3T（避免打扰测量周期） */
          s_last_batch_cycle = s6_cycle_cnt + 1U;
          BSP_Pulse_Start();
        }
        else
        {
          /* ★ 兜底：本 S6↑ 没启动 S3（adc_active / TIM3 忙 / 闸门未过）⇒ 撤销上一次 S6↓
           * 的 PC2 预拉高，否则 PC2 会一直卡高到下一次 S3↓ 才被清。
           * 正常路径下不触发（组内 +4T、组间 +20T，前一个 S3 早已结束）。 */
          GPIOC->BSRR = (uint32_t)GPIO_PIN_2 << 16U;   /* PC2 拉低 */
        }
      }
    }
  }
}

void BSP_ADCsamp_Start(void)
{
  HAL_NVIC_DisableIRQ(TIM7_IRQn);

  __HAL_TIM_DISABLE(&htim7);
  __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
  __HAL_TIM_SET_COUNTER(&htim7, 0);

  /* 复位采样状态 */
  adc_sample_count = 0;
  adc_dma_busy     = 0;
  adc_done         = 0;
  adc_active       = 1;
  adc_error        = 0;
  adc_lead_expired = 0;
  adc_fail_count   = 0;

  /* 前导上限装载：前导段照常转换、不入 adc_buf；S6↓ 捕获（TIM2 ISR）置 adc_zero_pending，
   * 由 TIM7 ISR 落实清 0 后才开始写入 ⇒ adc_buf[0] = 零点那一拍。
   * ★ 2026-10-27 修 R-13：上限 = 本次采样点数 + 固定余量，使「前导兜底」始终比「S3↓ 兜底」
   *   早 12.05ms（与档位无关）。用固定 4096 会让档1/档2 的前导越过 S3 门，见宏定义处推导。
   * ★ 本值只是「等不到 S6↓」的兜底，非零点估计。*/
  adc_lead_left    = (uint16_t)(adc_target_count + ADC_LEAD_MARGIN_PTS);
  adc_zero_pending = 0;    /* 清掉上一次残留的零点沿标志（R-10）*/

  /* 复位测距起点记录（等待本次采样期间的 S6 下降沿）*/
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
  /* 非法档位钳为 1（≈15m，1024 点），保证 adc_target_count 与缓冲区始终有效。
   * 最大倍数 ADC_SAMPLE_MULT_MAX=5（5120 点 × 15.034mm ≈ 76.9m）。 */
  if ((level == 0U) || (level > ADC_SAMPLE_MULT_MAX))
  {
    level = 1U;
  }
  /* 只更新请求值；adc_target_count 在下一次 BSP_Pulse_Start 快照 */
  s_range_level_req = level;
}

uint8_t BSP_Range_GetLevel(void)
{
  return s_range_level_req;
}

uint16_t BSP_Range_GetRequestSampleCount(void)
{
  /* 请求档位对应的采样点数 = 下一次 BSP_Pulse_Start 将快照进 adc_target_count 的值。
   * ⚠ 与 BSP_Range_GetSampleCount() 的区别：本函数在【采样序列启动前】读也有效
   *   （开机恢复 EEPROM 基线、基线档位校验等场景），而后者那时只会返回初值 1024。 */
  return (uint16_t)ADC_SAMPLE_BASE * (uint16_t)s_range_level_req;
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

/* S6 周期（μs，上升沿→上升沿）。TIM2 捕获 ISR 每周期更新；读单字无撕裂风险。
 * 用途：应用层 S7/S9 闭环被控量（app_radar.c）、测量触发闸门（捕获 ISR）。
 * ★ 返回 0 = 尚未测到周期（无兜底值）。S6 是所有后续操作的前提：调用方必须先判 0，
 *   为 0 时不得据此发 S7/S9、也不得启动 S3/ADC 测量，只等下一个 S6↑ 再读。*/
uint32_t BSP_GetS6PeriodUs(void)
{
  return s6_period_us;
}

/* ★ 2026-10-27 修 R-12：TIM3 序列自愈次数（主循环读，变化即打告警）*/
uint16_t BSP_Timer_GetTim3RecoverCount(void)
{
  return tim3_recover_cnt;
}

/* TIM4 / TIM7 / TIM3(自愈) 溢出中断回调 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* ---------------- TIM4：S7/S9 校正脉冲结束（PWM2 更新中断 = 脉冲结束）---------------- */
  if (htim->Instance == TIM4)
  {
    /* ★② 兜底：先把两路输出强制回空闲电平，再停表（详见文件头「输出冻结修复」）。
     * PWM2：CCR>ARR ⇒ CNT 恒 < CCR ⇒ OCREF=0 ⇒ CH3(S7)输出 LOW、CH4(S9)输出 HIGH（空闲）。 */
    htim->Instance->CCR3 = 0xFFFFU;
    htim->Instance->CCR4 = 0xFFFFU;

    __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
    htim->Instance->CR1 &= ~TIM_CR1_CEN;   /* ★① 起效后硬件已清 CEN，此处仅为状态收敛 */
    htim->Instance->SR   = 0;
    htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
    htim->State   = HAL_TIM_STATE_READY;

    /* S7/S9 连发：每发完一个 remain--，还有则短间隔(4μs)重发；只有首脉冲用 76μs 延时。
     * ⚠ 另一批次需等下一个 S6↓ 再启动，以保证其首脉冲时序。 */
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

  /* ---------------- TIM3：S3/S4 序列超期自愈（2026-10-27 修 R-12）----------------
   * 不变量：一次 BSP_Pulse_Start 启动的序列，4 个比较沿最晚在
   *   S6_HIGH_MAX_US + N×16µs = 104.87ms（档5）内全部到齐，远早于 ARR 满量程 131.07ms
   *   ⇒ 【UEV 到来时 CEN 还是 1，就说明这个序列已经超期】，必定丢了某个比较沿
   *     （EMI 误清 CC1IF/CC3IF、CCER 被误写、状态位被写坏……）。
   * 旧实现没有自恢复路径 ⇒ CEN 永为 1 ⇒ 捕获 ISR 的启动闸门恒假 ⇒ 永久停测，只能复位。
   * 处理：连续 2 次这样的 UEV（期间没有任何比较事件）⇒ 强制收尾；下一个 S6↑ 即可重新起测，
   *   代价最多 1~2 次测量。
   * ⚠ 为什么是「连续 2 次」而不是 1 次：档5 真零点 >49.15ms 时 CCR1 会撞上限，
   *   比较沿与 UEV 只差 1 tick，极端情况下第一次 UEV 可能先于 CC1 被处理；
   *   要求连续两次都无进展可把这一次误判完全排除，恢复延迟仍只有 262ms。
   * ⚠ CEN 已为 0 时什么都不做（序列正常收尾，UEV 只是最后那一拍）。 */
  if (htim->Instance == TIM3)
  {
    if ((TIM3->CR1 & TIM_CR1_CEN) == 0U)
    {
      tim3_uev_stuck = 0U;
    }
    else if (++tim3_uev_stuck >= 2U)
    {
      Tim3_ForceIdle();
      if (tim3_recover_cnt < 0xFFFFU) { tim3_recover_cnt++; }
    }
    return;
  }

  /* ---------------- TIM7：ADC 采样循环 ---------------- */
  if (htim->Instance != TIM7)
  {
    return;
  }

  /* 已采满本档位目标点数（1024~5120）：停止 TIM7 */
  if (adc_sample_count >= adc_target_count)
  {
    __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
    __HAL_TIM_DISABLE(htim);
    adc_done   = 1;
    adc_active = 0;
    return;
  }

  /* ★ 2026-10-22 R-10：落实 TIM2 捕获到的零点沿 —— 清前导 + 清计数必须在本 ISR 里做，
   *   与本拍「写 adc_buf[adc_sample_count] → 自增」处于同一序列，TIM2 不再碰这两个量。
   *   落实后 adc_lead_left 立即为 0 ⇒ 跳过下面的前导分支，本拍直接写 adc_buf[0]。 */
  if (adc_zero_pending != 0U)
  {
    adc_zero_pending = 0U;
    adc_lead_left    = 0U;
    adc_sample_count = 0U;
  }

  /* ============ 前导段：S4↑(ADC 起点) → S6↓(零点捕获) ============
   * ADC 起点仍在 S4↑（硬件不动），但结果先丢进 s_adc_scratch、不写 adc_buf；
   * 被 TIM2 的 S6↓ 捕获清零后（或到「N + ADC_LEAD_MARGIN_PTS」兜底）才开始写 ⇒ adc_buf[0] = 零点那一拍，
   * 缓冲只装 N 点（档5 的 5120 点因此刚好装下）。
   * ⚠ 必须照常发起 SPI 读：ADCS7476 靠 CS/SCLK 的读时序推进转换节拍，
   *   前导段若跳过读，转换节拍与 TIM7 失步、后续点全错位。
   * ⚠ 前导段的读失败【不计入】adc_fail_count：这些点根本不入缓冲，
   *   计数只会白白触发 ADC_FAIL_MAX 中止。*/
  if (adc_lead_left > 0U)
  {
    adc_lead_left--;
    (void)ADCS7476_Read(&s_adc_scratch);

    /* ★ 2026-10-27 修 R-13：前导兜底耗尽仍未捕获 S6↓ ⇒ 本次零点未知。
     *   旧实现在这里「静默开始写 adc_buf」，而 adc_buf[0] 实为零点后 37~50ms 的样本，
     *   tx_sample_valid 仍为 0 ⇒ 算法走零偏路径（s_zero_offset=0）算出 37~50m 的
     *   假距离，再被 App_Config_ApplyRange 钳成满量程；基线学习也不看 tx_sample_valid，
     *   会把这条错位曲线学成空罐基线，此后永久带偏。
     *   ⇒ 现在直接中止本次测量：不写 adc_buf、置 adc_error（算法层据此弃用）。
     * ⚠ 判据落在这「刚好归零」的一拍：若 S6↓ 在同一拍被捕获，上面 adc_zero_pending
     *   分支已把 adc_lead_left 清 0 并提前 return，走不到这里 ⇒ 不会误伤。
     * ⚠ 还要再查一次 adc_zero_pending：TIM2（优先级 0）可能恰好在本拍的「adc_lead_left--」
     *   与下面这条判断之间抢占并置位 pending。那时 lead 已为 0 但 pending=1，
     *   若只判 lead==0 就会误判成漏捕、白丢一次有效测量。
     *   ⇒ 加上 pending==0 后，窗口只剩「pending 置位之前」，那才是真的没捕获到。
     * ⚠ 置 adc_lead_expired 只为让上层区分「漏捕零点」与「ADC 连续失败」——两者
     *   adc_error 都是 1，但排查方向完全相反。*/
    if ((adc_lead_left == 0U) && (adc_zero_pending == 0U))
    {
      __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
      __HAL_TIM_DISABLE(htim);
      adc_done         = 1U;
      adc_error        = 1U;
      adc_lead_expired = 1U;
      adc_active       = 0U;
    }
    return;
  }

  /* ============ 阻塞版（当前启用）============
   * 单次约 14µs，TIM7 周期 16µs。优先级：TIM2=0；TIM3/TIM4/TIM7=1（同级不互相抢占）。
   * ⚠ 连续 ADC_FAIL_MAX 次失败即停 TIM7 + 置 adc_error —— 只保证【不卡死】，不保证测量
   *   可信：超时是 2000 次轮询（非 1ms）、单次约 160µs 且全花在 ISR 内，TIM7 照常溢出
   *   而 UIF 单标志 ⇒ 一次失败白吃约 10 个采样周期只前进 1 个下标，失败点在回波峰之前
   *   时距离少算约 30cm 且静默返回成功。精确弃用由算法层按【失败点位置】完成
   *   （Algo_MeasureDistance）。
   * ⚠ 副作用：失败点【之后】时间轴仍错位，会使回波曲线后段水平畸变（Disp_SendPackEchoEx
   *   直接读 adc_buf），不影响距离。
   */
  if (ADCS7476_Read((uint16_t *)&adc_buf[adc_sample_count]) == HAL_OK)
  {
    adc_fail_count = 0;
    adc_sample_count++;
  }
  else
  {
    /* 失败也前进一位（避免卡在坏样本上），写入标记值 0xFFFF 供算法层识别。
     * ⚠ 哨兵契约：写 0xFFFF 的同时，读取侧必须【全部】剔除，否则 65535 会被当成
     *   真实回波。读取侧（Algo_FindStrongestPeak / Algo_MeasureDistance /
     *   Disp_SendPackCurveNorm）均已处理：
     *   插值修补 + 按"失败点是否在回波峰之前"弃用（详见 app_algorithm.c 顶部说明块）。
     *   后果：无基线降级路径检不到峰；量产路径基线被永久污染，且时间轴断裂使距离
     *   静默少算约 30cm。日后新增读取 adc_buf 的代码必须同步处理哨兵。
     * ⚠ 若改为其他哨兵值，须全库 grep 0xFFFF 同步修改读取侧。
     * ⚠ 另一条写入口：bsp_spi.c 的 HAL_SPI_RxCpltCallback（DMA 版）只掩码不写哨兵，且
     *   当前不可达（DMA 路径处于 #if 0）；恢复 DMA 路径时须在失败分支补写 0xFFFFU，
     *   否则剔除逻辑对 DMA 采到的失败点完全无效。*/
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
   * TIM7 周期 16μs 太短，DMA 单次循环约 7μs setup + 12.8μs SPI（当时 SCK=1.25MHz）+ 7μs complete
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