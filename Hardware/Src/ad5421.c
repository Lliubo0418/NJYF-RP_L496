#include "ad5421.h"
#include <string.h>

// 外部引用 SPI 句柄（假设使用 SPI2）
extern SPI_HandleTypeDef hspi2;

HAL_StatusTypeDef AD5421_WriteRegister(uint8_t cmd, uint16_t data)
{
    uint8_t txBuffer[3];

    // 构建 24 位数据帧：MSB 在前[reference:10]
    txBuffer[0] = cmd;
    txBuffer[1] = (uint8_t)(data >> 8);
    txBuffer[2] = (uint8_t)(data & 0xFF);

    // 拉低 SYNC，开始通信
    AD5421_SYNC_LOW();

    // 发送 3 个字节
    HAL_StatusTypeDef status = BSP_SPI_Transmit(&hspi2, txBuffer, 3, 100);

    // 拉高 SYNC，锁存数据[reference:11]
    AD5421_SYNC_HIGH();

    return status;
}

HAL_StatusTypeDef AD5421_ReadRegister(uint8_t cmd, uint16_t *pData)
{
    uint8_t txBuffer[3];
    uint8_t rxBuffer[3] = {0};
    HAL_StatusTypeDef status;

    if (pData == NULL) {
        return HAL_ERROR;
    }

    // 步骤1：发送读命令（数据部分为无关位，填 0）
    txBuffer[0] = cmd;
    txBuffer[1] = 0x00;
    txBuffer[2] = 0x00;

    AD5421_SYNC_LOW();
    status = BSP_SPI_TransmitReceive(&hspi2, txBuffer, rxBuffer, 3, 100);
    AD5421_SYNC_HIGH();

    if (status != HAL_OK) {
        return status;
    }

    // 步骤2：执行一次 NOP 写操作，以从 SDO 获取数据[reference:13]
    // 注意：NOP 写操作的数据位是无关的，但会触发 SDO 输出上一次读命令请求的数据
    txBuffer[0] = AD5421_CMD_NOP;
    txBuffer[1] = 0x00;
    txBuffer[2] = 0x00;

    AD5421_SYNC_LOW();
    status = BSP_SPI_TransmitReceive(&hspi2, txBuffer, rxBuffer, 3, 100);
    AD5421_SYNC_HIGH();

    if (status != HAL_OK) {
        return status;
    }

    // 从接收到的数据中提取 16 位有效数据
    *pData = (rxBuffer[1] << 8) | rxBuffer[2];

    return HAL_OK;
}

HAL_StatusTypeDef AD5421_Init(void)
{
    HAL_StatusTypeDef status;

    // 1. 复位芯片（复位后需等待至少 50µs）[reference:14]
    status = AD5421_WriteRegister(AD5421_CMD_RESET, 0x0000);
    if (status != HAL_OK) return status;
    HAL_Delay(1); // 延时 1ms，远大于 50µs

    // 2. 配置控制寄存器（举例：使能内部 ADC，禁用自动故障回读等）
    //    具体配置值需根据应用需求设置
    //    0xFC80 是一个示例值，启用内部 ADC，选择测量 VLOOP[reference:15]
    status = AD5421_WriteRegister(AD5421_CMD_WRITE_CONTROL, 0xFC80);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

HAL_StatusTypeDef AD5421_SetDACOutput(uint16_t dacCode)
{
    // 1. 写入 DAC 寄存器
    HAL_StatusTypeDef status = AD5421_WriteRegister(AD5421_CMD_WRITE_DAC, dacCode);
    if (status != HAL_OK) return status;

    // 2. 发送 LOAD DAC 命令，将 DAC 寄存器值更新到输出[reference:16]
    return AD5421_WriteRegister(AD5421_CMD_LOAD_DAC, 0x0000);
}

HAL_StatusTypeDef AD5421_ReadFaultRegister(uint16_t *pFaultData)
{
    return AD5421_ReadRegister(AD5421_CMD_READ_FAULT, pFaultData);
}