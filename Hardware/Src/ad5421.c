#include "ad5421.h"
#include <string.h>

extern SPI_HandleTypeDef hspi2;

HAL_StatusTypeDef AD5421_WriteRegister(uint8_t cmd, uint16_t data)
{
    uint8_t txBuffer[3];

    /* 24 位帧：cmd + 16 位数据，MSB 在前 */
    txBuffer[0] = cmd;
    txBuffer[1] = (uint8_t)(data >> 8);
    txBuffer[2] = (uint8_t)(data & 0xFF);

    /* ⚠ SYNC 低有效：低电平期间发送，上升沿锁存 */
    AD5421_SYNC_LOW();

    HAL_StatusTypeDef status = BSP_SPI_Transmit(&hspi2, txBuffer, 3, 100);

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

    /* 读命令：数据位无关，填 0 */
    txBuffer[0] = cmd;
    txBuffer[1] = 0x00;
    txBuffer[2] = 0x00;

    AD5421_SYNC_LOW();
    status = BSP_SPI_TransmitReceive(&hspi2, txBuffer, rxBuffer, 3, 100);
    AD5421_SYNC_HIGH();

    if (status != HAL_OK) {
        return status;
    }

    /* ⚠ 必须再发一次 NOP，SDO 才输出上一次读命令请求的数据 */
    txBuffer[0] = AD5421_CMD_NOP;
    txBuffer[1] = 0x00;
    txBuffer[2] = 0x00;

    AD5421_SYNC_LOW();
    status = BSP_SPI_TransmitReceive(&hspi2, txBuffer, rxBuffer, 3, 100);
    AD5421_SYNC_HIGH();

    if (status != HAL_OK) {
        return status;
    }

    *pData = (rxBuffer[1] << 8) | rxBuffer[2];

    return HAL_OK;
}

HAL_StatusTypeDef AD5421_Init(void)
{
    HAL_StatusTypeDef status;

    /* 复位后须等待 ≥50µs 才能下发命令 */
    status = AD5421_WriteRegister(AD5421_CMD_RESET, 0x0000);
    if (status != HAL_OK) return status;
    HAL_Delay(1);

    /* ⚠ 0xFC80 = 使能内部 ADC + 选择测量 VLOOP；改值须查数据手册 */
    status = AD5421_WriteRegister(AD5421_CMD_WRITE_CONTROL, 0xFC80);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

HAL_StatusTypeDef AD5421_SetDACOutput(uint16_t dacCode)
{
    HAL_StatusTypeDef status = AD5421_WriteRegister(AD5421_CMD_WRITE_DAC, dacCode);
    if (status != HAL_OK) return status;

    /* ⚠ DAC 寄存器写入不生效，须再发 LOAD DAC(0x05) 才更新输出 */
    return AD5421_WriteRegister(AD5421_CMD_LOAD_DAC, 0x0000);
}

HAL_StatusTypeDef AD5421_ReadFaultRegister(uint16_t *pFaultData)
{
    return AD5421_ReadRegister(AD5421_CMD_READ_FAULT, pFaultData);
}

/* 强制输出报警电流：按 ALARM_CURRENT_DIRECTION 引脚输出芯片自身的低报警(典型 3.2mA)
 * 或高报警(典型 22.8/24mA，视量程) —— 硬件行为，与 DAC 寄存器无关（命令数据位无效）。
 * ⚠ 实际 mA 取决于 RANGE 引脚跳线 + ALARM_CURRENT_DIRECTION 接法；飞卓文档的
 *   20.5/22.0mA 仅供参考，上线前必须实测，不符则改用"DAC 码值 + 实测标定表"。 */
HAL_StatusTypeDef AD5421_ForceAlarm(void)
{
    return AD5421_WriteRegister(AD5421_CMD_FORCE_ALARM, 0x0000);
}