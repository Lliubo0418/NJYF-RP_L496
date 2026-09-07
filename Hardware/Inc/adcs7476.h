#ifndef __ADCS7476_H
#define __ADCS7476_H

#include "bsp_spi.h"
#include <stdint.h>

/**
  * @brief  从 ADCS7476 读取一次转换结果（寄存器层实现，绕过 HAL 轮询开销）
  * @param  pValue: 存储 ADC 值的指针 (0~4095)
  * @retval HAL 状态
  * @note   专为 TIM7 高频采样优化。HAL_SPI_Receive 在 -O0 + Flash 4 wait state
  *         下每次有 ~10μs 轮询/函数调用开销，直接操作寄存器可压到 ~13μs。
  *         时间预算：CS 0.1μs + 写 DR 0.1μs + 等 RXNE 12.8μs + 读 DR 0.1μs
  *                  + 等 BSY 0.1μs + CS 0.1μs = ~13.4μs（< TIM7 周期 16μs）。
  */

HAL_StatusTypeDef ADCS7476_Read(uint16_t *pValue);


/**
  * @brief  以 SPI+DMA 非阻塞方式启动一次 ADCS7476 读取
  * @note   拉低 CS 后调用 HAL_SPI_Receive_DMA 启动 1 个 16-bit 接收；
  *         启动失败会自动拉高 CS。收尾（CS 拉高 + 提取 12-bit + 停 TIM7）
  *         由 bsp_spi.c 的 HAL_SPI_RxCpltCallback 直接处理。
  * @param  pValue: 存放结果的指针，需在 DMA 完成前保持有效
  * @retval HAL 状态
  */

HAL_StatusTypeDef ADCS7476_Read_DMA(uint16_t *pValue);

#endif
