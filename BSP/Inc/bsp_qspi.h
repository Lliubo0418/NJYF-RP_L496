#ifndef __BSP_QSPI_H
#define __BSP_QSPI_H

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// 声明 CubeMX 生成的 QSPI 句柄（名称可能与你的工程不同，请修改）
extern QSPI_HandleTypeDef hqspi;

// 传输状态枚举
typedef enum {
    BSP_QSPI_STATUS_IDLE = 0,
    BSP_QSPI_STATUS_BUSY,      // DMA传输进行中
    BSP_QSPI_STATUS_ERROR
} BSP_QSPI_Status_t;

// DMA完成回调函数类型
typedef void (*BSP_QSPI_Callback_t)(bool success);

// -------------------- 初始化/反初始化 --------------------
HAL_StatusTypeDef BSP_QSPI_Init(void);
HAL_StatusTypeDef BSP_QSPI_DeInit(void);

// -------------------- 阻塞模式（无DMA）--------------------
// addrMode: QSPI_ADDRESS_NONE / QSPI_ADDRESS_1_LINE / QSPI_ADDRESS_4_LINES
//           无地址相位的命令（如 JEDEC ID 0x9F、读状态寄存器 0x05、写使能 0x06 等）必须传 QSPI_ADDRESS_NONE
HAL_StatusTypeDef BSP_QSPI_CommandWrite(uint8_t cmd, uint32_t addrMode, uint32_t addr, uint8_t *pData, uint32_t Size, uint32_t Timeout);
HAL_StatusTypeDef BSP_QSPI_CommandRead(uint8_t cmd, uint32_t addrMode, uint32_t addr, uint8_t *pData, uint32_t Size, uint32_t Timeout);
HAL_StatusTypeDef BSP_QSPI_CommandOnly(uint8_t cmd, uint32_t addrMode, uint32_t addr, uint32_t Timeout);

// -------------------- DMA模式（推荐大块数据传输）--------------------
// 注意：以下函数返回后，DMA传输在后台进行，需通过状态查询或回调获知完成
HAL_StatusTypeDef BSP_QSPI_CommandWrite_DMA(uint8_t cmd, uint32_t addr, uint8_t *pData, uint32_t Size);
HAL_StatusTypeDef BSP_QSPI_CommandRead_DMA(uint8_t cmd, uint32_t addr, uint8_t *pData, uint32_t Size);
HAL_StatusTypeDef BSP_QSPI_CommandOnly_DMA(uint8_t cmd, uint32_t addrMode, uint32_t addr);

// -------------------- DMA 状态和回调管理 --------------------
BSP_QSPI_Status_t BSP_QSPI_GetStatus(void);
void BSP_QSPI_RegisterCallback(BSP_QSPI_Callback_t callback);
void BSP_QSPI_WaitForComplete(uint32_t timeout_ms);  // 等待传输完成（阻塞等待）

#endif