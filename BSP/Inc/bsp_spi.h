#ifndef __BSP_SPI_H
#define __BSP_SPI_H

#include "stm32l4xx_hal.h"
#include <stdint.h>

// 声明 CubeMX 生成的 SPI 句柄
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

// 函数声明：通过 SPI 发送并接收数据（全双工）
/**
  * @brief  通过 SPI 发送并接收数据（全双工）
  * @param  hspi:    SPI 句柄指针
  * @param  pTxData: 待发送数据的指针
  * @param  pRxData: 接收数据缓冲区的指针
  * @param  Size:    数据长度（字节）
  * @param  Timeout: 超时时间（毫秒）
  * @retval HAL 状态
  */

HAL_StatusTypeDef BSP_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);

// 函数声明：通过 SPI 仅发送数据
/**
  * @brief  通过 SPI 仅发送数据
  * @param  hspi:    SPI 句柄指针
  * @param  pData:   待发送数据的指针
  * @param  Size:    数据长度（字节）
  * @param  Timeout: 超时时间（毫秒）
  * @retval HAL 状态
  */

HAL_StatusTypeDef BSP_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);

// 函数声明：通过 SPI 仅接收数据
/**
  * @brief  通过 SPI 仅接收数据
  * @param  hspi:    SPI 句柄指针
  * @param  pData:   接收数据缓冲区的指针
  * @param  Size:    数据长度（字节）
  * @param  Timeout: 超时时间（毫秒）
  * @retval HAL 状态
  */

HAL_StatusTypeDef BSP_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);

// 函数声明：通过 SPI 以 DMA 方式发送数据
/**
  * @brief  通过 SPI 以 DMA 方式发送数据
  * @param  hspi:  SPI 句柄指针
  * @param  pData: 待发送数据的指针
  * @param  Size:  数据长度（字节）
  * @retval HAL 状态
  */

HAL_StatusTypeDef BSP_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);

// 函数声明：通过 SPI 以 DMA 方式接收数据
/**
  * @brief  通过 SPI 以 DMA 方式接收数据
  * @param  hspi:  SPI 句柄指针
  * @param  pData: 接收数据缓冲区的指针
  * @param  Size:  数据长度（字节）
  * @retval HAL 状态
  */

HAL_StatusTypeDef BSP_SPI_Receive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);

// 函数声明：通过 SPI 以 DMA 方式全双工发送并接收数据
/**
  * @brief  通过 SPI 以 DMA 方式全双工发送并接收数据
  * @param  hspi:    SPI 句柄指针
  * @param  pTxData: 待发送数据的指针
  * @param  pRxData: 接收数据缓冲区的指针
  * @param  Size:    数据长度（字节）
  * @retval HAL 状态
  */

HAL_StatusTypeDef BSP_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);

#endif