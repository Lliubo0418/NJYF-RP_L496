#ifndef __ADCS7477_H
#define __ADCS7477_H

#include "bsp_spi.h"
#include <stdint.h>

/**
  * @brief  从 ADCS7477 读取一次转换结果
  * @param  pValue: 存储 ADC 值的指针 (0~1023)
  * @retval HAL 状态
  */

HAL_StatusTypeDef ADCS7477_Read(uint16_t *pValue);

#endif
