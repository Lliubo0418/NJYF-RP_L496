#include "adcs7476.h"
#include "bsp_gpio.h"

// 外部引用 SPI 句柄（假设使用 SPI1）
extern SPI_HandleTypeDef hspi1;

HAL_StatusTypeDef ADCS7476_Read(uint16_t *pValue)
{
    uint32_t timeout;

    if (pValue == NULL) {
        return HAL_ERROR;
    }

    /* 1. 拉低 CS，启动转换（直接 BSRR 写，省去 4 次 HAL_GPIO_WritePin 函数调用开销）
     *    板上 CS 经反相器：MCU SET → 芯片 CS LOW（选中）*/
    SPI1_CS_GPIO_Port->BSRR   = SPI1_CS_Pin;            /* SET   PA4 → CS LOW */
    ADC_CS_X_GPIO_Port->BSRR  = ADC_CS_X_Pin;           /* SET   PC4 → CS LOW */

    /* 2. 确保 SPI 已使能（CubeMX 已使能，这里只做保护性检查）*/
    if ((SPI1->CR1 & SPI_CR1_SPE) == 0U)
    {
        SPI1->CR1 |= SPI_CR1_SPE;
    }

    /* 3. 写 dummy 到 DR，启动 SCK 时钟生成 */
    *(__IO uint16_t *)&SPI1->DR = 0xFFFFU;

    /* 4. 等 RXNE（接收完成），带超时保护（硬件挂掉时不会卡死）*/
    /*    2000 次循环 ≈ 250μs@80MHz / 500μs@40MHz，远大于 12.8μs SPI 传输 */
    timeout = 2000U;
    while ((SPI1->SR & SPI_SR_RXNE) == 0U)
    {
        if (--timeout == 0U)
        {
            /* SPI 挂了，释放 CS 后返回（BSRR 高 16 位 = RESET）*/
            SPI1_CS_GPIO_Port->BSRR  = (uint32_t)SPI1_CS_Pin  << 16U;
            ADC_CS_X_GPIO_Port->BSRR = (uint32_t)ADC_CS_X_Pin << 16U;
            return HAL_TIMEOUT;
        }
    }

    /* 5. 读 DR 取出 16-bit 数据 */
    {
        uint16_t rxData = (uint16_t)*(__IO uint16_t *)&SPI1->DR;

        /* 6. 等 BSY 清零（确保最后一个 SCK 边沿结束）*/
        while (SPI1->SR & SPI_SR_BSY) { }

        /* 7. 拉高 CS，结束本次转换（BSRR 高 16 位 = RESET → MCU 端 LOW → 芯片 CS HIGH）*/
        SPI1_CS_GPIO_Port->BSRR  = (uint32_t)SPI1_CS_Pin  << 16U;
        ADC_CS_X_GPIO_Port->BSRR = (uint32_t)ADC_CS_X_Pin << 16U;

        /* 8. 提取 12 位有效数据 */
        *pValue = rxData & 0x0FFFU;
    }

    return HAL_OK;
}

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