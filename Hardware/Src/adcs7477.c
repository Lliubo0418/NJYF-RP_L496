#include "adcs7477.h"
#include "bsp_gpio.h"

extern SPI_HandleTypeDef hspi1;

HAL_StatusTypeDef ADCS7477_Read(uint16_t *pValue)
{
    uint8_t rxData[2] = {0};
    HAL_StatusTypeDef status;

    if (pValue == NULL) {
        return HAL_ERROR;
    }

    BSP_GPIO_SPI1_CS_Select();
    BSP_GPIO_ADC_CS_X_Select();

    /* 16 个 SCK：前 4 位无效前导 0 + 10 位有效数据 + 2 位尾随 0 */
    status = BSP_SPI_Receive(&hspi1, rxData, 2, 100);

    BSP_GPIO_SPI1_CS_Deselect();
    BSP_GPIO_ADC_CS_X_Deselect();

    if (status != HAL_OK) {
        return status;
    }

    *pValue = ((rxData[0] << 8) | rxData[1]) >> 2;

    return HAL_OK;
}