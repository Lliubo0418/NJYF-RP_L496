#ifndef __BSP_TIMER_H__
#define __BSP_TIMER_H__

#include "main.h"
#include "tim.h"

extern volatile uint8_t f_ic_sta;
extern volatile uint32_t f_ic_val;  /* S6 低电平宽度（S6↓→S6↑，μs）；⚠ 这是低电平、不是周期，
                                     *   周期见 BSP_GetS6PeriodUs()（上升沿→上升沿） */
extern volatile uint8_t f_ic_new;   /* 输入捕获完成标志：1=有新的 f_ic_val 可读，0=已消费 */

/* ============ 测距起点记录（Echo-Discovery 测距用）============
 * ADC 采样期间遇到的 S6 下降沿 = 测距起点 t₀（距离 0 位）。⚠ 它不是「发射时刻」——
 * 发射与接收连续进行，S6↓ 只是本次扫描的 0 相位点（措辞约定见 L496时序.md §1）。
 * 距离 = (peak_pos - tx_sample_offset) × dist_per_sample。
 * ⚠ 2026-10-14 起【写入起点】由 S6↓ 捕获实时触发（前导段不入缓冲）⇒ 零点即 adc_buf[0]，
 *   tx_sample_offset 常态恒为 0；非 0 只可能出现在捕获被漏掉的降级路径。 */
extern volatile uint8_t  tx_sample_valid;     /* 1=本次采样已记录到测距起点，0=未记录 */
extern volatile uint16_t tx_sample_offset;    /* 测距起点对应的 ADC 采样点序号（0~5119）*/

void BSP_Pulse_Start(void);
/* ⚠ 2026-10-14：S7/S9 脉冲改由 TIM4 单脉冲模式(OPM)产生 —— 硬件在 UEV 自动停表并
 *   让输出回空闲，避免「更新中断里手动停表」被 TIM7 采样 ISR 挤延迟后把输出冻住
 *   （详见 bsp_timer.c 文件头「输出冻结修复」）。调用语义不变。 */
void BSP_s7_pulse(void);
void BSP_s9_pulse(void);

/* ============ TIM7 驱动 ADC 采样（SPI1 寄存器层阻塞读）============
 * 每点距离 ≈ (c/2)×16μs / RADAR_HW_TIME_STRETCH(159526) ≈ 15.034mm（@TIM7 62.5kHz）。
 * 量程分档（BSP_Range_SetLevel，由 DPARAM_RANGE_SETTING 换算；1 档 = 1024 点 ≈ 15.4m）：
 *   1 ≈ 15m/1024 点；2 ≈ 30m/2048；3 ≈ 45m/3072；4 ≈ 60m/4096；5 ≈ 75m/5120。
 * 缓冲区按最大 5120 点静态分配，实际点数运行时由 adc_target_count 决定。
 * ⚠ 2026-10-14：ADC 硬件起点仍是 S4↑(S6↑+6.570ms)，但【写入起点】改由 S6↓ 捕获实时触发
 *   （前导段只转换不入缓冲）⇒ adc_buf[0] = 零点那一拍、只装 N 点，缓冲无需扩容
 *   （档5 的 5120 点刚好装下）。★ 不预测零点：捕获到什么就是什么。
 *   S3↓ 随真零点重设为「零点 + N×16μs」（不再有旧的固定 20ms 尾巴）。 */
#define ADC_SAMPLE_BASE       1024U                 /* 单档基础点数 */
#define ADC_SAMPLE_MULT_MAX   5U                    /* 最大倍数（5 → 5120 点 ≈ 77.0m；2026-10-14 由 3 扩）*/
#define ADC_SAMPLE_COUNT_MAX  (ADC_SAMPLE_BASE * ADC_SAMPLE_MULT_MAX)  /* = 5120（唯一派生处；⚠ 改它须同步核 EEPROM 基线与档位映射）*/
#define ADC_FAIL_MAX          3U                     /* 连续失败上限，超过即中止采样（防卡死）*/

/* ADC 采样状态（extern，由 bsp_timer.c 定义；DMA 备用路径由 bsp_spi.c 的 RxCplt 读写）*/
extern volatile uint16_t adc_buf[ADC_SAMPLE_COUNT_MAX]; /* 采样缓冲区（12-bit 有效值）*/
extern volatile uint16_t adc_sample_count;            /* 已采样点数 */
extern volatile uint16_t adc_target_count;            /* 本次采样目标点数（1024~5120，序列开始时快照）*/
extern volatile uint8_t  adc_dma_busy;              /* 1=SPI1-DMA 接收进行中（DMA 备用路径，当前停用） */
extern volatile uint8_t  adc_done;                  /* 1=采样结束（成功或中止都置位）*/
extern volatile uint8_t  adc_error;                 /* 1=采样因连续失败而中止（硬件异常）*/
extern volatile uint8_t  adc_lead_expired;          /* 1=前导兜底耗尽仍未捕获 S6↓（零点漏捕，R-13）。
                                                     * 与 adc_error 同时为 1，但这不是 ADC 故障而是
                                                     * 「S6↓ 没进来」；排查方向：占空比突变 / S6 硬件。*/
extern volatile uint8_t  adc_active;                /* 1=采样流程进行中（bsp_timer.c 互斥 + bsp_spi.c DMA 回调清零）*/

/* ============ 量程档位 → 采样点数 ============
 * level 1~5 对应 ADC 点数 1024 / 2048 / 3072 / 4096 / 5120（≈15.4 / 30.8 / 46.2 / 61.6 / 77.0 m），
 * 由 app_config 的 rangeSetting(m) 经 ≤15 / ≤30 / ≤45 / ≤60 / 其余 映射而来。
 * ⚠ S3 不再有"按档位的脉宽表"：其结束沿由 adc_target_count × 16μs 实时算出，随点数自动伸缩。
 * SetLevel 只更新"请求档位"，下次 BSP_Pulse_Start 时快照生效，采样中改档不影响本次。 */
void     BSP_Range_SetLevel(uint8_t level);   /* 设置量程档位（<1 或 >ADC_SAMPLE_MULT_MAX 钳为 1）*/
uint8_t  BSP_Range_GetLevel(void);            /* 当前请求档位 1~5 */
uint16_t BSP_Range_GetSampleCount(void);      /* 【采样后读】已快照的本次生效点数（1024 的 1~5 倍）*/
uint16_t BSP_Range_GetRequestSampleCount(void); /* 【采样前读】请求档位对应点数 = 下一次 BSP_Pulse_Start 将快照的值 */

extern volatile uint8_t s7_remain;
extern volatile uint8_t s9_remain;

/* S7/S9 连发超时检查（2026-10-22 修 R-11）。
 * remain 只在 TIM4 更新中断里递减，一旦 UEV 丢失/burst_type 与 remain 不一致，
 * remain 永不归零 ⇒ 主循环的 `if (s7_remain || s9_remain) return;` 会永久阻止新校正。
 * 主循环每轮调用一次即可：超时（>200ms 未收尾）强制停表、两路输出回空闲、remain 清零。
 * 返回 0 = 无需处理；1 = 本次已强制清零（调用方应打印告警）。 */
uint8_t BSP_S7S9_CheckTimeout(void);

/* ============ F_IC 低电平窗（已退役 · 仅作 K 的刻度锚点）============
 * ⚠ 2026-10-13 起既不参与测量触发、也不作 S7/S9 闭环被控量，两者一律以「S6 周期」为准。
 *   保留下面三个常量只为换算 K：K = f_T × (F_IC_LOWER_US / duty)，duty ≈ 50.5%。
 * ⚠ 改 F_IC_LOWER_US = 改距离刻度，必须与 RADAR_HW_TIME_STRETCH 一起核对，不可单独动。 */
#define F_IC_LOWER_US        22509U
#define F_IC_UPPER_US        23609U
#define F_IC_DEADBAND_US      100U

/* ============ S6 周期窗口（上升沿→上升沿，μs）============
 * 【唯一定义处 · 其他文件只准引用宏名，不准复述数值】
 * 目标 T* = 44.5668ms = 44567μs；窗口 [T*, T*+100μs] = [44567, 44667]。
 * 用途：① 测量触发闸门；② S7/S9 闭环被控量。
 * 上电未收敛时周期可达 ~45.76ms > 上限 ⇒ 闭环从长周期侧往下压（S7 放电），稳态停点 ≈ 上沿。
 * ⚠ K = f_T × 周期，改任一边界 = 改距离刻度，必须与 RADAR_HW_TIME_STRETCH 一起核对。
 * ⚠ 严禁把阈值改成「上电实测值」。 */
#define F_IC_PERIOD_LOWER_US      44567U   /* = T* = 44.5668ms ⇔ 22.4382Hz（周期下限）*/
#define F_IC_PERIOD_UPPER_US      44667U   /* = T* + 100μs = 44.6668ms（上限）*/
#define F_IC_PERIOD_MEAS_LOWER_US (F_IC_PERIOD_LOWER_US - F_IC_DEADBAND_US)
#define F_IC_PERIOD_MEAS_UPPER_US (F_IC_PERIOD_UPPER_US + F_IC_DEADBAND_US)

/* 读上一次实测的 S6 周期（上升沿→上升沿，μs）。
 * ★ 返回 0 = 尚未测到周期（无兜底值）。S6 是前提条件：为 0 时调用方不得发 S7/S9、
 *   也不得启动 S3/ADC 测量，只等下一个 S6↑ 再读。
 * 用途：应用层 S7/S9 闭环被控量（app_radar.c）、测量触发闸门（bsp_timer.c）。 */
uint32_t BSP_GetS6PeriodUs(void);

/* 设置 S7/S9 校正脉冲脉宽（μs）。
 * ⚠ 钳位范围【唯一定义处】在 bsp_timer.c 的 S7S9_MIN_PULSE_TICK / MAX_PULSE_TICK，当前为 20~40μs。
 * 用法：上电大偏差时用 40μs 宽脉冲快充/快放，接近目标后切 20μs 精调。 */
void BSP_s7_set_width_us(uint16_t width_us);
void BSP_s9_set_width_us(uint16_t width_us);

/* ============ TIM7 驱动 ADCS7476 采样（SPI1 寄存器层阻塞读，1024~5120 点随档位）============
 * TIM7 溢出周期 16μs；SPI1 分频 32（SCK 2.5MHz），16bit 纯传输 6.4μs。
 * 溢出 ISR 直接调用 ADCS7476_Read()：拉低 CS、写 DR、等 RXNE/BSY、释放 CS 并提取 12-bit。
 * 单次含 CS 翻转与轮询等软件开销约 14μs，相对 16μs 周期余量约 2μs。
 * ⚠ SPI+DMA 异步路径已停用（见 bsp_timer.c 的 DMA 异步版说明块），仅为备查保留；
 *   ADCS7476_Read_DMA() 实现处于 #if 0，当前不可调用。 */

void BSP_ADCsamp_Start(void);

/* 采样是否完成：1=目标点数已采完，可读取缓冲区；0=进行中 */
uint8_t BSP_ADCsamp_IsDone(void);

/* TIM3 S3/S4 序列超期自愈次数（2026-10-27 修 R-12）。
 * 返回累计「序列没能在一个 ARR 周期(131ms)内收尾、被强制收尾」的次数。
 * 主循环每轮读一次即可：值变化 ⇒ 本次运行发生过比较沿丢失，应打告警
 * （表现为"偶发丢 1~2 次测量"，不是停测；持续增长则查 TIM3 侧 EMI / CCER 被误写）。 */
uint16_t BSP_Timer_GetTim3RecoverCount(void);

#endif