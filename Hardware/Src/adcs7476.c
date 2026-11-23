#include "adcs7476.h"
#include "bsp_gpio.h"

extern SPI_HandleTypeDef hspi1;

HAL_StatusTypeDef ADCS7476_Read(uint16_t *pValue)
{
    uint32_t timeout;

    if (pValue == NULL) {
        return HAL_ERROR;
    }

    /* CS 低有效、板上【无反相器】(PA4=SPI1_CS / PC4=ADC_CS_X)：MCU 拉低 = 芯片 CS LOW 选中
     * BSRR 语义：低 16 位写 1 = 置位(HIGH)；高 16 位写 1 = 复位(LOW) */
    SPI1_CS_GPIO_Port->BSRR   = (uint32_t)SPI1_CS_Pin  << 16U;
    ADC_CS_X_GPIO_Port->BSRR  = (uint32_t)ADC_CS_X_Pin << 16U;

    /* 保护性检查：SPE 已由 CubeMX 使能 */
    if ((SPI1->CR1 & SPI_CR1_SPE) == 0U)
    {
        SPI1->CR1 |= SPI_CR1_SPE;
    }

    /* 写 dummy 0xFFFF 到 DR 启动 SCK 时钟生成 */
    *(__IO uint16_t *)&SPI1->DR = 0xFFFFU;

    /* 等 RXNE；超时 2000 次轮询 ≈250μs@80MHz / 500μs@40MHz，远大于 6.4μs 传输（16bit @ SCK 2.5MHz） */
    timeout = 2000U;
    while ((SPI1->SR & SPI_SR_RXNE) == 0U)
    {
        if (--timeout == 0U)
        {
            /* 超时：释放 CS 后返回（置位 → MCU 端 HIGH → 芯片 CS HIGH）*/
            SPI1_CS_GPIO_Port->BSRR  = SPI1_CS_Pin;
            ADC_CS_X_GPIO_Port->BSRR = ADC_CS_X_Pin;
            return HAL_TIMEOUT;
        }
    }

    {
        uint16_t rxData = (uint16_t)*(__IO uint16_t *)&SPI1->DR;

        /* 等 BSY 清零：正常几微秒内即落。超时 2000 次轮询 ≈250μs@80MHz / 500μs@40MHz。
         * ⚠ BSY 不落说明 SPI 时钟/引脚异常，必须释放 CS 返回，不可在 16μs 采样 ISR 里裸等 */
        timeout = 2000U;
        while (SPI1->SR & SPI_SR_BSY)
        {
            if (--timeout == 0U)
            {
                SPI1_CS_GPIO_Port->BSRR  = SPI1_CS_Pin;
                ADC_CS_X_GPIO_Port->BSRR = ADC_CS_X_Pin;
                return HAL_TIMEOUT;
            }
        }

        /* 释放 CS 结束转换（置位 → MCU 端 HIGH → 芯片 CS HIGH）*/
        SPI1_CS_GPIO_Port->BSRR  = SPI1_CS_Pin;
        ADC_CS_X_GPIO_Port->BSRR = ADC_CS_X_Pin;

        *pValue = rxData & 0x0FFFU;
    }

    return HAL_OK;
}

/* ⚠ DMA 异步版已停用（TIM7 16μs 周期下 DMA 固定开销过大，实际采样率仅 ~31kHz）；
 * 恢复时须同时取消本 #if 0 与 bsp_spi.c / bsp_timer.c 对应开关 */
#if 0
HAL_StatusTypeDef ADCS7476_Read_DMA(uint16_t *pValue)
{
    HAL_StatusTypeDef status;

    if (pValue == NULL) {
        return HAL_ERROR;
    }

    // 1. 拉低 CS，启动转换
    BSP_GPIO_SPI1_CS_Select();
    BSP_GPIO_ADC_CS_X_Select();

    // 2. 启动 SPI1 RX DMA 接收 1 个 16-bit
    status = BSP_SPI_Receive_DMA(&hspi1, (uint8_t *)pValue, 1);

    // 3. 启动失败：拉高 CS 释放总线
    if (status != HAL_OK) {
        BSP_GPIO_SPI1_CS_Deselect();
        BSP_GPIO_ADC_CS_X_Deselect();
    }

    return status;
}
#endif