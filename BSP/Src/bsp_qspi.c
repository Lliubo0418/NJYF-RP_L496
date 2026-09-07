#include "bsp_qspi.h"

// 外部QSPI句柄（由CubeMX生成）
extern QSPI_HandleTypeDef hqspi;

// 静态状态变量
static BSP_QSPI_Status_t qspi_status = BSP_QSPI_STATUS_IDLE;
static BSP_QSPI_Callback_t user_callback = NULL;

// -------------------- 内部辅助 --------------------
static void BSP_QSPI_SetStatus(BSP_QSPI_Status_t status)
{
    qspi_status = status;
}

// -------------------- 初始化/反初始化 --------------------
HAL_StatusTypeDef BSP_QSPI_Init(void)
{
    // CubeMX 已经在 main() 中调用了 MX_QUADSPI_Init()，
    // 这里仅重置状态，若需额外操作可在此添加。
    BSP_QSPI_SetStatus(BSP_QSPI_STATUS_IDLE);
    return HAL_OK;
}

HAL_StatusTypeDef BSP_QSPI_DeInit(void)
{
    return HAL_QSPI_DeInit(&hqspi);
}

// -------------------- 阻塞模式（无DMA）--------------------

HAL_StatusTypeDef BSP_QSPI_CommandWrite(uint8_t cmd, uint32_t addrMode, uint32_t addr, uint8_t *pData, uint32_t Size, uint32_t Timeout)
{
    QSPI_CommandTypeDef sCommand = {0};
    HAL_StatusTypeDef status;

    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = cmd;
    sCommand.AddressMode       = addrMode;
    sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
    sCommand.Address           = addr;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;

    if (pData != NULL && Size > 0) {
        sCommand.DataMode = QSPI_DATA_1_LINE;
        sCommand.NbData = Size;
    } else {
        sCommand.DataMode = QSPI_DATA_NONE;
        sCommand.NbData = 0;
    }
    sCommand.DummyCycles = 0;
    sCommand.SIOOMode    = QSPI_SIOO_INST_EVERY_CMD;

    status = HAL_QSPI_Command(&hqspi, &sCommand, Timeout);
    if (status != HAL_OK) return status;

    if (pData != NULL && Size > 0) {
        status = HAL_QSPI_Transmit(&hqspi, pData, Timeout);
    }
    return status;
}

HAL_StatusTypeDef BSP_QSPI_CommandRead(uint8_t cmd, uint32_t addrMode, uint32_t addr, uint8_t *pData, uint32_t Size, uint32_t Timeout)
{
    QSPI_CommandTypeDef sCommand = {0};
    HAL_StatusTypeDef status;

    if (pData == NULL || Size == 0) return HAL_ERROR;

    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = cmd;
    sCommand.AddressMode       = addrMode;
    sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
    sCommand.Address           = addr;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = QSPI_DATA_1_LINE;
    sCommand.NbData            = Size;
    sCommand.DummyCycles       = 0;
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    status = HAL_QSPI_Command(&hqspi, &sCommand, Timeout);
    if (status != HAL_OK) return status;

    return HAL_QSPI_Receive(&hqspi, pData, Timeout);
}

HAL_StatusTypeDef BSP_QSPI_CommandOnly(uint8_t cmd, uint32_t addrMode, uint32_t addr, uint32_t Timeout)
{
    QSPI_CommandTypeDef sCommand = {0};

    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = cmd;
    sCommand.AddressMode       = addrMode;
    sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
    sCommand.Address           = addr;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = QSPI_DATA_NONE;
    sCommand.NbData            = 0;
    sCommand.DummyCycles       = 0;
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    return HAL_QSPI_Command(&hqspi, &sCommand, Timeout);
}

// -------------------- DMA模式（单DMA通道时分复用）--------------------

HAL_StatusTypeDef BSP_QSPI_CommandWrite_DMA(uint8_t cmd, uint32_t addr, uint8_t *pData, uint32_t Size)
{
    QSPI_CommandTypeDef sCommand = {0};
    HAL_StatusTypeDef status;

    if (pData == NULL || Size == 0) return HAL_ERROR;

    // 1. 配置命令结构体（包含数据长度）
    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = cmd;
    sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
    sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
    sCommand.Address           = addr;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = QSPI_DATA_1_LINE;
    sCommand.NbData            = Size;          // 数据长度
    sCommand.DummyCycles       = 0;
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    // 2. 发送命令和地址（同步阻塞）
    status = HAL_QSPI_Command(&hqspi, &sCommand, 100);
    if (status != HAL_OK) {
        BSP_QSPI_SetStatus(BSP_QSPI_STATUS_ERROR);
        return status;
    }

    // 3. 启动 DMA 发送数据（非阻塞）
    BSP_QSPI_SetStatus(BSP_QSPI_STATUS_BUSY);
    status = HAL_QSPI_Transmit_DMA(&hqspi, pData);
    if (status != HAL_OK) {
        BSP_QSPI_SetStatus(BSP_QSPI_STATUS_ERROR);
    }
    return status;
}

HAL_StatusTypeDef BSP_QSPI_CommandRead_DMA(uint8_t cmd, uint32_t addr, uint8_t *pData, uint32_t Size)
{
    QSPI_CommandTypeDef sCommand = {0};
    HAL_StatusTypeDef status;

    if (pData == NULL || Size == 0) return HAL_ERROR;

    // 1. 配置命令结构体
    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = cmd;
    sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
    sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
    sCommand.Address           = addr;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = QSPI_DATA_1_LINE;
    sCommand.NbData            = Size;
    sCommand.DummyCycles       = 0;
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    // 2. 发送命令和地址（同步阻塞）
    status = HAL_QSPI_Command(&hqspi, &sCommand, 100);
    if (status != HAL_OK) {
        BSP_QSPI_SetStatus(BSP_QSPI_STATUS_ERROR);
        return status;
    }

    // 3. 启动 DMA 接收数据（非阻塞）
    BSP_QSPI_SetStatus(BSP_QSPI_STATUS_BUSY);
    status = HAL_QSPI_Receive_DMA(&hqspi, pData);
    if (status != HAL_OK) {
        BSP_QSPI_SetStatus(BSP_QSPI_STATUS_ERROR);
    }
    return status;
}

HAL_StatusTypeDef BSP_QSPI_CommandOnly_DMA(uint8_t cmd, uint32_t addrMode, uint32_t addr)
{
    // 无数据阶段，直接用同步命令即可，不需要 DMA
    // 但为了接口统一，我们仍调用同步命令并返回
    return BSP_QSPI_CommandOnly(cmd, addrMode, addr, 100);
}

// -------------------- 状态和回调管理 --------------------
BSP_QSPI_Status_t BSP_QSPI_GetStatus(void)
{
    return qspi_status;
}

void BSP_QSPI_RegisterCallback(BSP_QSPI_Callback_t callback)
{
    user_callback = callback;
}

void BSP_QSPI_WaitForComplete(uint32_t timeout_ms)
{
    uint32_t tickstart = HAL_GetTick();
    while (qspi_status == BSP_QSPI_STATUS_BUSY) {
        if ((HAL_GetTick() - tickstart) >= timeout_ms) {
            break;
        }
        // 可在此加入低功耗等待或任务调度
    }
}

// -------------------- HAL 回调函数（由HAL库自动调用）--------------------

/**
  * @brief  QSPI DMA传输完成回调
  * @note   无论是发送还是接收，完成后都会进入此回调
  */
void HAL_QSPI_CmdCpltCallback(QSPI_HandleTypeDef *hqspi)
{
    BSP_QSPI_SetStatus(BSP_QSPI_STATUS_IDLE);
    if (user_callback != NULL) {
        user_callback(true);
    }
}

/**
  * @brief  QSPI DMA传输错误回调
  */
void HAL_QSPI_ErrorCallback(QSPI_HandleTypeDef *hqspi)
{
    BSP_QSPI_SetStatus(BSP_QSPI_STATUS_ERROR);
    if (user_callback != NULL) {
        user_callback(false);
    }
}

/**
  * @brief  QSPI 中止完成回调（超时或手动中止）
  */
void HAL_QSPI_AbortCpltCallback(QSPI_HandleTypeDef *hqspi)
{
    BSP_QSPI_SetStatus(BSP_QSPI_STATUS_IDLE);
    if (user_callback != NULL) {
        user_callback(false);
    }
}