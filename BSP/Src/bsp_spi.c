#include "bsp_spi.h"
#include "bsp_gpio.h"
#include "bsp_timer.h"

HAL_StatusTypeDef BSP_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
{
    return HAL_SPI_TransmitReceive(hspi, pTxData, pRxData, Size, Timeout);
}

HAL_StatusTypeDef BSP_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return HAL_SPI_Transmit(hspi, pData, Size, Timeout);
}

HAL_StatusTypeDef BSP_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return HAL_SPI_Receive(hspi, pData, Size, Timeout);
}

HAL_StatusTypeDef BSP_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
    return HAL_SPI_Transmit_DMA(hspi, pData, Size);
}

HAL_StatusTypeDef BSP_SPI_Receive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
    return HAL_SPI_Receive_DMA(hspi, pData, Size);
}

HAL_StatusTypeDef BSP_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
    return HAL_SPI_TransmitReceive_DMA(hspi, pTxData, pRxData, Size);
}

/**
  * @brief  SPI DMA 接收完成回调
  * @note   TIM7 驱动的 ADCS7476 采样流程在此收尾：
  *         - 拉高 CS，结束本次转换
  *         - 提取 12-bit 有效数据
  *         - 计数 +1，满目标点数停止 TIM7
  */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    /* 仅处理 TIM7 驱动的 SPI1 RX-DMA 采样 */
    if (hspi->Instance != SPI1 || !adc_dma_busy)
    {
        return;
    }

    /* 拉高 CS，结束本次 ADC 转换 */
    BSP_GPIO_SPI1_CS_Deselect();
    BSP_GPIO_ADC_CS_X_Deselect();

    /* 提取 12 位有效数据：[0 0 0 0 D11..D0] */
    adc_buf[adc_sample_count] &= 0x0FFFU;
    adc_sample_count++;

    /* 采满本档位目标点数（1024~3072）：停止 TIM7 */
    if (adc_sample_count >= adc_target_count)
    {
        __HAL_TIM_DISABLE_IT(&htim7, TIM_IT_UPDATE);
        __HAL_TIM_DISABLE(&htim7);
        adc_done = 1;
        adc_active = 0;
    }

    adc_dma_busy = 0;
}
