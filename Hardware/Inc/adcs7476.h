#ifndef __ADCS7476_H
#define __ADCS7476_H

#include "bsp_spi.h"
#include <stdint.h>

/**
  * @brief  从 ADCS7476 读取一次转换结果（寄存器层实现，绕过 HAL 轮询开销）
  * @param  pValue: 存储 ADC 值的指针 (0~4095)
  * @retval HAL 状态
  * @note   专为 TIM7 高频采样优化：直接操作寄存器，耗时 ~13.4μs（< TIM7 周期 16μs），
  *         而 HAL_SPI_Receive 在 -O0 + Flash 4 wait state 下有 ~10μs 额外开销。
  */

HAL_StatusTypeDef ADCS7476_Read(uint16_t *pValue);


/**
  * @brief  【已停用】以 SPI+DMA 非阻塞方式启动一次 ADCS7476 读取（实现在 adcs7476.c 的 #if 0 内，当前不可调用）
  * @note   拉低 CS 后启动 HAL_SPI_Receive_DMA（1 个 16-bit 接收），启动失败自动拉高 CS；
  *         收尾（CS 拉高 + 提取 12-bit + 停 TIM7）由 bsp_spi.c 的 HAL_SPI_RxCpltCallback 处理。
  * @param  pValue: 存放结果的指针，需在 DMA 完成前保持有效
  * @retval HAL 状态
  */

HAL_StatusTypeDef ADCS7476_Read_DMA(uint16_t *pValue);

#endif
