#include "w25q128jv.h"
#include <string.h>

// 超时定义（单位：毫秒）
#define W25Q_TIMEOUT_WRITE_ENABLE       (10)
#define W25Q_TIMEOUT_PAGE_PROGRAM       (10)    // 典型 0.7ms ~ 3ms
#define W25Q_TIMEOUT_SECTOR_ERASE       (500)   // 典型 45ms ~ 400ms
#define W25Q_TIMEOUT_BLOCK_ERASE_32     (1500)  // 典型 120ms ~ 1600ms
#define W25Q_TIMEOUT_BLOCK_ERASE_64     (2000)  // 典型 150ms ~ 2000ms
#define W25Q_TIMEOUT_CHIP_ERASE         (30000) // 典型 15s ~ 30s
#define W25Q_TIMEOUT_STATUS_READ        (10)

// 外部 QSPI 句柄
extern QSPI_HandleTypeDef hqspi;

// -------------------- 内部静态函数（状态寄存器读取）--------------------
static HAL_StatusTypeDef W25Q128_ReadStatusReg(uint8_t cmd, uint8_t *pStatus)
{
    if (pStatus == NULL) return HAL_ERROR;
    // 读状态寄存器命令无需地址相位
    return BSP_QSPI_CommandRead(cmd, QSPI_ADDRESS_NONE, 0, pStatus, 1, W25Q_TIMEOUT_STATUS_READ);
}

// 内部：写状态寄存器（命令 + 1字节数据，无地址相位）
static HAL_StatusTypeDef W25Q128_WriteStatusReg(uint8_t cmd, uint8_t status)
{
    HAL_StatusTypeDef ret;
    ret = W25Q128_WriteEnable();
    if (ret != HAL_OK) return ret;

    // 写状态寄存器：指令 + 1字节数据，无地址相位
    ret = BSP_QSPI_CommandWrite(cmd, QSPI_ADDRESS_NONE, 0, &status, 1, W25Q_TIMEOUT_WRITE_ENABLE);
    if (ret != HAL_OK) return ret;

    return W25Q128_WaitForWriteComplete(W25Q_TIMEOUT_PAGE_PROGRAM);
}

// -------------------- 公共接口实现 --------------------

HAL_StatusTypeDef W25Q128_Init(void)
{
    uint32_t id = 0;
    
    // 1. 初始化 QSPI 硬件
    if (BSP_QSPI_Init() != HAL_OK) {
        return HAL_ERROR;
    }
    
    // 2. 读取 JEDEC ID 验证连接
    if (W25Q128_ReadJEDECID(&id) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // W25Q128JV 的 JEDEC ID: 0xEF4018
    // 制造商 ID: 0xEF (Winbond)
    // 设备 ID: 0x4018
    if ((id & 0xFFFFFF) != 0xEF4018) {
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

HAL_StatusTypeDef W25Q128_ReadJEDECID(uint32_t *pID)
{
    uint8_t idBuf[3] = {0};
    HAL_StatusTypeDef ret;
    
    if (pID == NULL) return HAL_ERROR;
    
    // JEDEC ID (0x9F) 只有指令 + 3字节数据，无地址相位
    ret = BSP_QSPI_CommandRead(W25Q_CMD_READ_ID, QSPI_ADDRESS_NONE, 0, idBuf, 3, 100);
    if (ret != HAL_OK) return ret;
    
    *pID = (idBuf[0] << 16) | (idBuf[1] << 8) | idBuf[2];
    return HAL_OK;
}

HAL_StatusTypeDef W25Q128_ReadUniqueID(uint8_t *pUniqueID)
{
    if (pUniqueID == NULL) return HAL_ERROR;
    // 0x4B 命令：4字节地址 (通常为0) + 1字节Dummy + 8字节数据
    // 注意：此命令使用 32 位地址模式，需特殊处理
    // 简化实现：使用标准 SPI 方式读取
    QSPI_CommandTypeDef sCommand = {0};
    
    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = W25Q_CMD_READ_UNIQUE_ID;
    sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
    sCommand.AddressSize       = QSPI_ADDRESS_32_BITS;  // 注意：32位地址
    sCommand.Address           = 0x00000000;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = QSPI_DATA_1_LINE;
    sCommand.NbData            = 8;
    sCommand.DummyCycles       = 1;  // 1 个 Dummy 周期
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
    
    HAL_StatusTypeDef ret = HAL_QSPI_Command(&hqspi, &sCommand, 100);
    if (ret != HAL_OK) return ret;
    
    return HAL_QSPI_Receive(&hqspi, pUniqueID, 100);
}

HAL_StatusTypeDef W25Q128_ReadStatusReg1(uint8_t *pStatus)
{
    return W25Q128_ReadStatusReg(W25Q_CMD_READ_STATUS_REG1, pStatus);
}

HAL_StatusTypeDef W25Q128_ReadStatusReg2(uint8_t *pStatus)
{
    return W25Q128_ReadStatusReg(W25Q_CMD_READ_STATUS_REG2, pStatus);
}

HAL_StatusTypeDef W25Q128_ReadStatusReg3(uint8_t *pStatus)
{
    return W25Q128_ReadStatusReg(W25Q_CMD_READ_STATUS_REG3, pStatus);
}

HAL_StatusTypeDef W25Q128_WriteStatusReg1(uint8_t status)
{
    return W25Q128_WriteStatusReg(W25Q_CMD_WRITE_STATUS_REG1, status);
}

HAL_StatusTypeDef W25Q128_WriteStatusReg2(uint8_t status)
{
    return W25Q128_WriteStatusReg(W25Q_CMD_WRITE_STATUS_REG2, status);
}

HAL_StatusTypeDef W25Q128_WriteStatusReg3(uint8_t status)
{
    return W25Q128_WriteStatusReg(W25Q_CMD_WRITE_STATUS_REG3, status);
}

HAL_StatusTypeDef W25Q128_WaitForWriteComplete(uint32_t Timeout)
{
    uint8_t status = 0;
    uint32_t tickstart = HAL_GetTick();
    HAL_StatusTypeDef ret;
    
    do {
        ret = W25Q128_ReadStatusReg1(&status);
        if (ret != HAL_OK) return ret;
        
        // 检查 BUSY 位 (bit 0)
        if ((status & W25Q_STATUS_BUSY) == 0) {
            return HAL_OK;
        }
        HAL_Delay(1);
    } while ((HAL_GetTick() - tickstart) < Timeout);
    
    return HAL_TIMEOUT;
}

HAL_StatusTypeDef W25Q128_WriteEnable(void)
{
    HAL_StatusTypeDef ret;
    // 写使能 0x06：仅指令，无地址无数据
    ret = BSP_QSPI_CommandOnly(W25Q_CMD_WRITE_ENABLE, QSPI_ADDRESS_NONE, 0, W25Q_TIMEOUT_WRITE_ENABLE);
    if (ret != HAL_OK) return ret;
    
    // 可选：检查 WEL 位确认写使能成功
    uint8_t status;
    ret = W25Q128_ReadStatusReg1(&status);
    if (ret != HAL_OK) return ret;
    if ((status & W25Q_STATUS_WEL) == 0) {
        return HAL_ERROR;  // 写使能失败
    }
    return HAL_OK;
}

HAL_StatusTypeDef W25Q128_WriteDisable(void)
{
    // 写禁止 0x04：仅指令，无地址无数据
    return BSP_QSPI_CommandOnly(W25Q_CMD_WRITE_DISABLE, QSPI_ADDRESS_NONE, 0, W25Q_TIMEOUT_WRITE_ENABLE);
}

// -------------------- 擦除操作 --------------------

HAL_StatusTypeDef W25Q128_EraseSector(uint32_t SectorAddr)
{
    if (SectorAddr % W25Q128_SECTOR_SIZE != 0) {
        return HAL_ERROR;
    }
    
    HAL_StatusTypeDef ret = W25Q128_WriteEnable();
    if (ret != HAL_OK) return ret;
    
    ret = BSP_QSPI_CommandOnly(W25Q_CMD_SECTOR_ERASE, QSPI_ADDRESS_1_LINE, SectorAddr, 100);
    if (ret != HAL_OK) return ret;
    
    return W25Q128_WaitForWriteComplete(W25Q_TIMEOUT_SECTOR_ERASE);
}

HAL_StatusTypeDef W25Q128_EraseBlock32(uint32_t BlockAddr)
{
    if (BlockAddr % (32 * 1024) != 0) {
        return HAL_ERROR;
    }
    
    HAL_StatusTypeDef ret = W25Q128_WriteEnable();
    if (ret != HAL_OK) return ret;
    
    ret = BSP_QSPI_CommandOnly(W25Q_CMD_BLOCK_ERASE_32, QSPI_ADDRESS_1_LINE, BlockAddr, 100);
    if (ret != HAL_OK) return ret;
    
    return W25Q128_WaitForWriteComplete(W25Q_TIMEOUT_BLOCK_ERASE_32);
}

HAL_StatusTypeDef W25Q128_EraseBlock64(uint32_t BlockAddr)
{
    if (BlockAddr % W25Q128_BLOCK_SIZE != 0) {
        return HAL_ERROR;
    }
    
    HAL_StatusTypeDef ret = W25Q128_WriteEnable();
    if (ret != HAL_OK) return ret;
    
    ret = BSP_QSPI_CommandOnly(W25Q_CMD_BLOCK_ERASE_64, QSPI_ADDRESS_1_LINE, BlockAddr, 100);
    if (ret != HAL_OK) return ret;
    
    return W25Q128_WaitForWriteComplete(W25Q_TIMEOUT_BLOCK_ERASE_64);
}

HAL_StatusTypeDef W25Q128_EraseChip(void)
{
    HAL_StatusTypeDef ret = W25Q128_WriteEnable();
    if (ret != HAL_OK) return ret;
    
    // 全片擦除 0xC7：仅指令，无地址无数据
    ret = BSP_QSPI_CommandOnly(W25Q_CMD_CHIP_ERASE, QSPI_ADDRESS_NONE, 0, 100);
    if (ret != HAL_OK) return ret;
    
    return W25Q128_WaitForWriteComplete(W25Q_TIMEOUT_CHIP_ERASE);
}

// -------------------- 读取操作（阻塞模式）--------------------

HAL_StatusTypeDef W25Q128_ReadBytes(uint32_t ReadAddr, uint8_t *pBuffer, uint32_t NumByteToRead)
{
    if (pBuffer == NULL || NumByteToRead == 0) {
        return HAL_ERROR;
    }
    if ((ReadAddr + NumByteToRead) > W25Q128_TOTAL_SIZE) {
        return HAL_ERROR;  // 地址越界
    }
    return BSP_QSPI_CommandRead(W25Q_CMD_READ_DATA, QSPI_ADDRESS_1_LINE, ReadAddr, pBuffer, NumByteToRead, 100);
}

// -------------------- 写入操作（阻塞模式，自动跨页处理）--------------------

HAL_StatusTypeDef W25Q128_WriteBytes(uint32_t WriteAddr, uint8_t *pBuffer, uint32_t NumByteToWrite)
{
    uint32_t offset = 0;
    uint32_t bytes_to_write;
    HAL_StatusTypeDef ret;
    
    if (pBuffer == NULL || NumByteToWrite == 0) {
        return HAL_ERROR;
    }
    if ((WriteAddr + NumByteToWrite) > W25Q128_TOTAL_SIZE) {
        return HAL_ERROR;  // 地址越界
    }
    
    while (NumByteToWrite > 0) {
        // 计算当前页剩余空间
        uint32_t page_remain = W25Q128_PAGE_SIZE - (WriteAddr % W25Q128_PAGE_SIZE);
        bytes_to_write = (NumByteToWrite < page_remain) ? NumByteToWrite : page_remain;
        
        // 1. 写使能
        ret = W25Q128_WriteEnable();
        if (ret != HAL_OK) return ret;
        
        // 2. 页编程（指令 + 24位地址 + 数据）
        ret = BSP_QSPI_CommandWrite(W25Q_CMD_PAGE_PROGRAM, QSPI_ADDRESS_1_LINE, WriteAddr, pBuffer + offset, bytes_to_write, 100);
        if (ret != HAL_OK) return ret;

        // 3. 等待编程完成
        ret = W25Q128_WaitForWriteComplete(W25Q_TIMEOUT_PAGE_PROGRAM);
        if (ret != HAL_OK) return ret;

        // 4. 更新变量
        WriteAddr += bytes_to_write;
        offset += bytes_to_write;
        NumByteToWrite -= bytes_to_write;
    }

    return HAL_OK;
}

// -------------------- DMA 模式读写（非阻塞）--------------------

HAL_StatusTypeDef W25Q128_ReadBytes_DMA(uint32_t ReadAddr, uint8_t *pBuffer, uint32_t NumByteToRead)
{
    if (pBuffer == NULL || NumByteToRead == 0) {
        return HAL_ERROR;
    }
    if ((ReadAddr + NumByteToRead) > W25Q128_TOTAL_SIZE) {
        return HAL_ERROR;
    }
    return BSP_QSPI_CommandRead_DMA(W25Q_CMD_READ_DATA, ReadAddr, pBuffer, NumByteToRead);
}

HAL_StatusTypeDef W25Q128_WriteBytes_DMA(uint32_t WriteAddr, uint8_t *pBuffer, uint32_t NumByteToWrite)
{
    uint32_t offset = 0;
    uint32_t bytes_to_write;
    HAL_StatusTypeDef ret;
    
    if (pBuffer == NULL || NumByteToWrite == 0) {
        return HAL_ERROR;
    }
    if ((WriteAddr + NumByteToWrite) > W25Q128_TOTAL_SIZE) {
        return HAL_ERROR;
    }
    
    while (NumByteToWrite > 0) {
        // 计算当前页剩余空间
        uint32_t page_remain = W25Q128_PAGE_SIZE - (WriteAddr % W25Q128_PAGE_SIZE);
        bytes_to_write = (NumByteToWrite < page_remain) ? NumByteToWrite : page_remain;
        
        // 1. 写使能（阻塞）
        ret = W25Q128_WriteEnable();
        if (ret != HAL_OK) return ret;
        
        // 2. 页编程（DMA方式）
        ret = BSP_QSPI_CommandWrite_DMA(W25Q_CMD_PAGE_PROGRAM, WriteAddr, pBuffer + offset, bytes_to_write);
        if (ret != HAL_OK) return ret;
        
        // 3. 等待 DMA 传输完成
        //    注意：调用者需要在回调中判断何时完成，或使用 BSP_QSPI_WaitForComplete()
        //    由于每页传输完后需要等待 Flash 内部编程完成，这里简单采用阻塞等待
        //    实际应用中建议使用回调机制，此处为了简化使用阻塞等待
        BSP_QSPI_WaitForComplete(100);
        if (BSP_QSPI_GetStatus() == BSP_QSPI_STATUS_ERROR) {
            return HAL_ERROR;
        }
        
        // 4. 等待 Flash 内部编程完成（DMA传输完成后，Flash仍在编程）
        ret = W25Q128_WaitForWriteComplete(W25Q_TIMEOUT_PAGE_PROGRAM);
        if (ret != HAL_OK) return ret;
        
        // 5. 更新变量
        WriteAddr += bytes_to_write;
        offset += bytes_to_write;
        NumByteToWrite -= bytes_to_write;
    }

    return HAL_OK;
}

// -------------------- Quad 四线模式（阻塞）--------------------

HAL_StatusTypeDef W25Q128_EnableQuad(void)
{
    uint8_t sr2 = 0;
    HAL_StatusTypeDef ret;

    ret = W25Q128_ReadStatusReg2(&sr2);
    if (ret != HAL_OK) return ret;

    if (sr2 & W25Q_STATUS2_QE) {
        return HAL_OK;  // 已使能
    }

    sr2 |= W25Q_STATUS2_QE;
    ret = W25Q128_WriteStatusReg2(sr2);
    if (ret != HAL_OK) return ret;

    // 回读校验
    ret = W25Q128_ReadStatusReg2(&sr2);
    if (ret != HAL_OK) return ret;
    return (sr2 & W25Q_STATUS2_QE) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef W25Q128_DisableQuad(void)
{
    uint8_t sr2 = 0;
    HAL_StatusTypeDef ret;

    ret = W25Q128_ReadStatusReg2(&sr2);
    if (ret != HAL_OK) return ret;

    if ((sr2 & W25Q_STATUS2_QE) == 0) {
        return HAL_OK;  // 已禁能
    }

    sr2 &= ~W25Q_STATUS2_QE;
    return W25Q128_WriteStatusReg2(sr2);
}

HAL_StatusTypeDef W25Q128_ReadBytes_Quad(uint32_t ReadAddr, uint8_t *pBuffer, uint32_t NumByteToRead)
{
    QSPI_CommandTypeDef sCommand = {0};
    HAL_StatusTypeDef ret;

    if (pBuffer == NULL || NumByteToRead == 0) return HAL_ERROR;
    if ((ReadAddr + NumByteToRead) > W25Q128_TOTAL_SIZE) return HAL_ERROR;

    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = W25Q_CMD_QUAD_READ;
    sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
    sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
    sCommand.Address           = ReadAddr;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = QSPI_DATA_4_LINES;   // 数据四线
    sCommand.NbData            = NumByteToRead;
    sCommand.DummyCycles       = W25Q_DUMMY_QUAD_READ; // 0x6B 需 8 个 dummy
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    ret = HAL_QSPI_Command(&hqspi, &sCommand, 100);
    if (ret != HAL_OK) return ret;

    return HAL_QSPI_Receive(&hqspi, pBuffer, 100);
}

HAL_StatusTypeDef W25Q128_ReadBytes_Quad_DMA(uint32_t ReadAddr, uint8_t *pBuffer, uint32_t NumByteToRead)
{
    QSPI_CommandTypeDef sCommand = {0};
    HAL_StatusTypeDef ret;
    uint32_t tickstart;

    if (pBuffer == NULL || NumByteToRead == 0) return HAL_ERROR;
    if ((ReadAddr + NumByteToRead) > W25Q128_TOTAL_SIZE) return HAL_ERROR;

    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.Instruction       = W25Q_CMD_QUAD_READ;
    sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
    sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
    sCommand.Address           = ReadAddr;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DataMode          = QSPI_DATA_4_LINES;    // 数据四线
    sCommand.NbData            = NumByteToRead;
    sCommand.DummyCycles       = W25Q_DUMMY_QUAD_READ; // 0x6B 需 8 个 dummy
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    ret = HAL_QSPI_Command(&hqspi, &sCommand, 100);
    if (ret != HAL_OK) return ret;

    ret = HAL_QSPI_Receive_DMA(&hqspi, pBuffer);
    if (ret != HAL_OK) return ret;

    /* 轮询 HAL 状态等待 DMA 接收完成（DMA1_Ch5 完成中断会将状态置回 READY）*/
    tickstart = HAL_GetTick();
    while (HAL_QSPI_GetState(&hqspi) != HAL_QSPI_STATE_READY) {
        if ((HAL_GetTick() - tickstart) > 1000) {
            HAL_QSPI_Abort(&hqspi);
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef W25Q128_WriteBytes_Quad(uint32_t WriteAddr, uint8_t *pBuffer, uint32_t NumByteToWrite)
{
    uint32_t offset = 0;
    uint32_t bytes_to_write;
    HAL_StatusTypeDef ret;
    QSPI_CommandTypeDef sCommand = {0};

    if (pBuffer == NULL || NumByteToWrite == 0) return HAL_ERROR;
    if ((WriteAddr + NumByteToWrite) > W25Q128_TOTAL_SIZE) return HAL_ERROR;

    while (NumByteToWrite > 0) {
        uint32_t page_remain = W25Q128_PAGE_SIZE - (WriteAddr % W25Q128_PAGE_SIZE);
        bytes_to_write = (NumByteToWrite < page_remain) ? NumByteToWrite : page_remain;

        // 1. 写使能
        ret = W25Q128_WriteEnable();
        if (ret != HAL_OK) return ret;

        // 2. 四线页编程（指令1线 + 地址1线 + 数据4线）
        sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
        sCommand.Instruction       = W25Q_CMD_QUAD_PAGE_PROGRAM;
        sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
        sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
        sCommand.Address           = WriteAddr;
        sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
        sCommand.DataMode          = QSPI_DATA_4_LINES;   // 数据四线
        sCommand.NbData            = bytes_to_write;
        sCommand.DummyCycles       = 0;
        sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

        ret = HAL_QSPI_Command(&hqspi, &sCommand, 100);
        if (ret != HAL_OK) return ret;

        ret = HAL_QSPI_Transmit(&hqspi, pBuffer + offset, 100);
        if (ret != HAL_OK) return ret;

        // 3. 等待 Flash 内部编程完成
        ret = W25Q128_WaitForWriteComplete(W25Q_TIMEOUT_PAGE_PROGRAM);
        if (ret != HAL_OK) return ret;

        // 4. 更新变量
        WriteAddr += bytes_to_write;
        offset += bytes_to_write;
        NumByteToWrite -= bytes_to_write;
    }

    return HAL_OK;
}

