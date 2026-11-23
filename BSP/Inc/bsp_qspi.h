#ifndef __BSP_QSPI_H
#define __BSP_QSPI_H

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// CubeMX 生成的 QSPI 句柄
extern QSPI_HandleTypeDef hqspi;

typedef enum {
    BSP_QSPI_STATUS_IDLE = 0,
    BSP_QSPI_STATUS_BUSY,      // DMA传输进行中
    BSP_QSPI_STATUS_ERROR
} BSP_QSPI_Status_t;

// DMA 完成回调
typedef void (*BSP_QSPI_Callback_t)(bool success);

// -------------------- 初始化/反初始化 --------------------
HAL_StatusTypeDef BSP_QSPI_Init(void);
HAL_StatusTypeDef BSP_QSPI_DeInit(void);

// -------------------- 阻塞模式（无DMA）--------------------
// addrMode 取 QSPI_ADDRESS_NONE/1_LINE/4_LINES；无地址相位的命令（JEDEC ID 0x9F、读状态 0x05、写使能 0x06）必须传 QSPI_ADDRESS_NONE
HAL_StatusTypeDef BSP_QSPI_CommandWrite(uint8_t cmd, uint32_t addrMode, uint32_t addr, uint8_t *pData, uint32_t Size, uint32_t Timeout);
HAL_StatusTypeDef BSP_QSPI_CommandRead(uint8_t cmd, uint32_t addrMode, uint32_t addr, uint8_t *pData, uint32_t Size, uint32_t Timeout);
HAL_StatusTypeDef BSP_QSPI_CommandOnly(uint8_t cmd, uint32_t addrMode, uint32_t addr, uint32_t Timeout);

// -------------------- DMA模式（推荐大块数据传输）--------------------
// 以下函数返回后 DMA 在后台进行，需经状态查询或回调获知完成
HAL_StatusTypeDef BSP_QSPI_CommandWrite_DMA(uint8_t cmd, uint32_t addr, uint8_t *pData, uint32_t Size);
HAL_StatusTypeDef BSP_QSPI_CommandRead_DMA(uint8_t cmd, uint32_t addr, uint8_t *pData, uint32_t Size);
HAL_StatusTypeDef BSP_QSPI_CommandOnly_DMA(uint8_t cmd, uint32_t addrMode, uint32_t addr);

// -------------------- DMA 状态和回调管理 --------------------
BSP_QSPI_Status_t BSP_QSPI_GetStatus(void);
void BSP_QSPI_RegisterCallback(BSP_QSPI_Callback_t callback);
void BSP_QSPI_WaitForComplete(uint32_t timeout_ms);

#endif