#include "bsp_gpio.h"

void BSP_GPIO_SPI1_CS_Select(void)   { HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);   }
void BSP_GPIO_SPI1_CS_Deselect(void) { HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); }

void BSP_GPIO_SPI2_CS_Select(void)   { HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET); }
void BSP_GPIO_SPI2_CS_Deselect(void) { HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);   }

void BSP_GPIO_ADC_CS_X_Select(void)    { HAL_GPIO_WritePin(ADC_CS_X_GPIO_Port, ADC_CS_X_Pin, GPIO_PIN_SET);   }
void BSP_GPIO_ADC_CS_X_Deselect(void)  { HAL_GPIO_WritePin(ADC_CS_X_GPIO_Port, ADC_CS_X_Pin, GPIO_PIN_RESET); }

/* ---------- RS485 方向（USART3, DE=PA8）---------- */
void BSP_GPIO_RS485_TxMode(void) { HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);   }
void BSP_GPIO_RS485_RxMode(void) { HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET); }

void BSP_GPIO_HART_SetTransmit(void) { HAL_GPIO_WritePin(HART_RTS_GPIO_Port, HART_RTS_Pin, GPIO_PIN_RESET); }
void BSP_GPIO_HART_SetReceive(void)  { HAL_GPIO_WritePin(HART_RTS_GPIO_Port, HART_RTS_Pin, GPIO_PIN_SET);   }

void BSP_GPIO_HART_Reset(bool assert)
{
    HAL_GPIO_WritePin(HART_RESET_GPIO_Port, HART_RESET_Pin,
                      assert ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

bool BSP_GPIO_HART_CarrierDetect(void)
{
    /* PA11 = EXTI 输入；返回 true=高电平(有载波)，false=低电平(空闲) */
    return (HAL_GPIO_ReadPin(HART_CD_GPIO_Port, HART_CD_Pin) == GPIO_PIN_SET);
}

void BSP_GPIO_EEPROM_WriteProtect(bool enable)
{
    HAL_GPIO_WritePin(EEPROM_WP_GPIO_Port, EEPROM_WP_Pin,
                      enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool BSP_GPIO_TEMP_IntActive(void)
{
    return (HAL_GPIO_ReadPin(TEMP_INT_GPIO_Port, TEMP_INT_Pin) == GPIO_PIN_RESET);
}

bool BSP_GPIO_DAC_FaultActive(void)
{
    return (HAL_GPIO_ReadPin(AD5241_FAULT_GPIO_Port, AD5241_FAULT_Pin) == GPIO_PIN_RESET);
}

void BSP_GPIO_S5_Enable(bool on)
{
    HAL_GPIO_WritePin(S5_EN_GPIO_Port, S5_EN_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void BSP_GPIO_IO1_Set(bool on)   { HAL_GPIO_WritePin(IO1_GPIO_Port, IO1_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }
void BSP_GPIO_IO2_Set(bool on)   { HAL_GPIO_WritePin(IO2_GPIO_Port, IO2_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }
void BSP_GPIO_IO3_Set(bool on)   { HAL_GPIO_WritePin(IO3_GPIO_Port, IO3_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }
//void BSP_GPIO_IO2B5_Set(bool on) { HAL_GPIO_WritePin(IO2B5_GPIO_Port, IO2B5_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }
