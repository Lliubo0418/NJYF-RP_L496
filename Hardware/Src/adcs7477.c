#include "adcs7477.h"
#include "bsp_gpio.h"

// 外部引用 SPI 句柄（假设使用 SPI1）
extern SPI_HandleTypeDef hspi1;

HAL_StatusTypeDef ADCS7477_Read(uint16_t *pValue)
{
    uint8_t rxData[2] = {0};
    HAL_StatusTypeDef status;

    if (pValue == NULL) {
        return HAL_ERROR;
    }

    // 1. 拉低 CS，启动转换
    BSP_GPIO_SPI1_CS_Select();
    BSP_GPIO_ADC_CS_X_Select();

    // 2. 通过 SPI 发送 16 个时钟周期，读取 16 位数据
    //    前 4 位为无效前导零，随后 10 位为有效数据，最后 2 位为尾随零[reference:3]
    status = BSP_SPI_Receive(&hspi1, rxData, 2, 100);

    // 3. 拉高 CS，结束转换
    BSP_GPIO_SPI1_CS_Deselect();
    BSP_GPIO_ADC_CS_X_Deselect();

    if (status != HAL_OK) {
        return status;
    }

    // 4. 组合并提取有效 10 位数据
    //    数据格式: [0 0 0 0 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0 0 0]
    //    将接收到的两个字节组合后右移 2 位
    *pValue = ((rxData[0] << 8) | rxData[1]) >> 2;

    return HAL_OK;
}