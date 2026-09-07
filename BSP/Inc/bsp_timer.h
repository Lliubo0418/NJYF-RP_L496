#ifndef __BSP_TIMER_H__
#define __BSP_TIMER_H__

#include "main.h"
#include "tim.h"

extern volatile uint8_t f_ic_sta;
extern volatile uint32_t f_ic_val;
extern volatile uint8_t f_ic_new;   /* 输入捕获完成标志：1=有新的 f_ic_val 可读，0=已消费 */

/* ============ 发射时刻记录（Echo-Discovery 测距用）============
 * 原理：ADC 采样期间遇到的 S6 下降沿 = 发射时刻 t₀
 *       记录该下降沿对应的 ADC 采样点序号，距离 = (peak_pos - tx_sample_offset) × dist_per_sample
 * ============================================================================================= */
extern volatile uint8_t  tx_sample_valid;     /* 1=本次采样已记录到发射时刻，0=未记录 */
extern volatile uint16_t tx_sample_offset;    /* 发射时刻对应的 ADC 采样点序号（0~999）*/

void BSP_Pulse_Start(void);
void BSP_s7_pulse(void);
void BSP_s9_pulse(void);

/* ============ TIM7 驱动 ADC 采样（SPI1 + DMA）============ */
#define ADC_SAMPLE_COUNT   1024U    /* 采样点数 */
#define ADC_FAIL_MAX        3U       /* 连续失败次数上限，超过则中止采样避免卡死 */

/* ADC 采样状态（extern，由 bsp_timer.c 定义；bsp_spi.c 的 RxCplt 直接读写）*/
extern volatile uint16_t adc_buf[ADC_SAMPLE_COUNT];   /* 采样缓冲区（12-bit 有效值）*/
extern volatile uint16_t adc_sample_count;            /* 已采样点数 */
extern volatile uint8_t  adc_dma_busy;              /* 1=SPI1-DMA 接收进行中 */
extern volatile uint8_t  adc_done;                  /* 1=采样结束（成功或中止都置位）*/
extern volatile uint8_t  adc_error;                 /* 1=采样因连续失败而中止（硬件异常）*/

extern volatile uint8_t s7_remain;
extern volatile uint8_t s9_remain;

/* ============ F_IC 目标窗口（μs，单一来源，L496时序.md §4/§6）============
 * 用户确认实测最佳值：LOWER=22875, UPPER=23975（样机对齐效果最好）；
 * 测量触发窗口再外扩 F_IC_DEADBAND_US。TIM2 S6↑ 捕获 ISR 据此自动启动
 * 测量序列，主循环据此做校正。
 * 注意：上电未收敛时实测脉宽可达 ~28.4ms，严禁把阈值改成实测值。 */
#define F_IC_LOWER_US        22875U
#define F_IC_UPPER_US        23975U
#define F_IC_DEADBAND_US      100U
#define F_IC_MEAS_LOWER_US   (F_IC_LOWER_US - F_IC_DEADBAND_US)
#define F_IC_MEAS_UPPER_US   (F_IC_UPPER_US + F_IC_DEADBAND_US)

/* 设置 S7/S9 校正脉冲脉宽（μs，钳位 2~20）：
 * 上电大偏差时用宽脉冲快速充电/放电，接近目标后固定 2μs 精调 */
void BSP_s7_set_width_us(uint16_t width_us);
void BSP_s9_set_width_us(uint16_t width_us);

/* ============ TIM5 精确定时延（用于触发 BSP_Pulse_Start 前的延时）============ */
extern volatile uint8_t tim5_delay_done;   /* 1=TIM5 延时到，可执行 BSP_Pulse_Start */

/**
 * @brief  启动 TIM5 精确定时延时（单次，溢出即停）
 * @param  delay_us  延时微秒（PSC=80-1 → 1μs/tick，最大延时 ≈ 4294s）
 * @note   溢出中断中会置 tim5_delay_done=1 并自动停 TIM5，
 *         应用层检测到标志后需自行清零 tim5_delay_done。
 */

void BSP_TIM5_Delay_us(uint32_t delay_us);

/* ============================================================
 *  TIM7 驱动 ADCS7476 采样（SPI1 + DMA，共 1024 点）
 *
 *  时序：TIM7 溢出周期 16 μs，SPI1(16bit, SCK=1.25MHz) 单次
 *  传输 12.8 μs。溢出中断中只调用 ADCS7476_Read_DMA 发起非阻塞
 *  SPI-DMA 接收（CS 拉低 + HAL_SPI_Receive_DMA）；CS 拉高、
 *  12-bit 提取、计数 +1、满点停 TIM7 等收尾工作在 bsp_spi.c 的
 *  HAL_SPI_RxCpltCallback 中直接处理。
 *
 *  关于“能否在溢出中断里直接调用阻塞版 ADCS7476_Read”：
 *  阻塞读 ≈ 12.8μs(SPI) + 1μs(CS) ≈ 14μs，与 16μs 周期仅余
 *  2μs，CPU 87% 时间在中断里，稍有延迟即丢点 —— 不推荐。
 *  故此处用 SPI+DMA 异步方案。
 * ============================================================ */

void BSP_ADCsamp_Start(void);

/* 采样是否完成：1=1024 点已采完，可读取缓冲区；0=进行中 */
uint8_t BSP_ADCsamp_IsDone(void);

#endif