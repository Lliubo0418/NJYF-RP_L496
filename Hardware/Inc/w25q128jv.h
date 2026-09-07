#ifndef __W25Q128JV_H
#define __W25Q128JV_H

#include "bsp_qspi.h"
#include <stdint.h>

// -------------------- 芯片基本信息 --------------------
#define W25Q128_PAGE_SIZE           (256)    // 页大小 256 字节
#define W25Q128_SECTOR_SIZE         (4096)   // 扇区大小 4KB
#define W25Q128_BLOCK_SIZE          (65536)  // 块大小 64KB
#define W25Q128_TOTAL_SIZE          (0x1000000) // 16MB = 16 * 1024 * 1024

// -------------------- 指令码定义 --------------------
// 写使能/禁止
#define W25Q_CMD_WRITE_ENABLE       (0x06)
#define W25Q_CMD_WRITE_DISABLE      (0x04)

// 读状态寄存器
#define W25Q_CMD_READ_STATUS_REG1   (0x05)
#define W25Q_CMD_READ_STATUS_REG2   (0x35)
#define W25Q_CMD_READ_STATUS_REG3   (0x15)

// 写状态寄存器
#define W25Q_CMD_WRITE_STATUS_REG1  (0x01)
#define W25Q_CMD_WRITE_STATUS_REG2  (0x31)
#define W25Q_CMD_WRITE_STATUS_REG3  (0x11)

// 读写数据
#define W25Q_CMD_PAGE_PROGRAM       (0x02)   // 页编程
#define W25Q_CMD_READ_DATA          (0x03)   // 标准读取
#define W25Q_CMD_FAST_READ          (0x0B)   // 快速读取（带Dummy）

// 擦除
#define W25Q_CMD_SECTOR_ERASE       (0x20)   // 扇区擦除 (4KB)
#define W25Q_CMD_BLOCK_ERASE_32     (0x52)   // 32KB 块擦除
#define W25Q_CMD_BLOCK_ERASE_64     (0xD8)   // 64KB 块擦除
#define W25Q_CMD_CHIP_ERASE         (0xC7)   // 全片擦除 (或 0x60)

// 其他
#define W25Q_CMD_READ_ID            (0x9F)   // 读取 JEDEC ID
#define W25Q_CMD_READ_UNIQUE_ID     (0x4B)   // 读取唯一 ID (64位)
#define W25Q_CMD_POWER_DOWN         (0xB9)   // 进入掉电模式
#define W25Q_CMD_RELEASE_POWER_DOWN (0xAB)   // 退出掉电模式
#define W25Q_CMD_READ_SFDP          (0x5A)   // 读取 SFDP 参数

// -------------------- Quad 四线命令定义 --------------------
#define W25Q_CMD_QUAD_READ          (0x6B)   // Quad Output Fast Read   (1-1-4)
#define W25Q_CMD_QUAD_IO_READ       (0xEB)   // Quad I/O Fast Read      (1-4-4)
#define W25Q_CMD_QUAD_PAGE_PROGRAM  (0x32)   // Quad Input Page Program (1-1-4)

// 四线快速读 Dummy 周期数
#define W25Q_DUMMY_QUAD_READ        (8)      // 0x6B 需要 8 个 dummy 周期
#define W25Q_DUMMY_QUAD_IO_READ     (4)      // 0xEB 需要 4 个 dummy 周期

// -------------------- 状态寄存器位定义 --------------------
#define W25Q_STATUS_BUSY            (0x01)   // bit 0: 忙标志 (1=忙)
#define W25Q_STATUS_WEL             (0x02)   // bit 1: 写使能锁存 (1=已使能)
#define W25Q_STATUS_BP0             (0x04)   // bit 2: 块保护位0
#define W25Q_STATUS_BP1             (0x08)   // bit 3: 块保护位1
#define W25Q_STATUS_BP2             (0x10)   // bit 4: 块保护位2
#define W25Q_STATUS_TB              (0x20)   // bit 5: 顶部/底部保护
#define W25Q_STATUS_SRP0            (0x40)   // bit 6: 状态寄存器保护0
#define W25Q_STATUS_SRP1            (0x80)   // bit 7: 状态寄存器保护1

// 状态寄存器2 位定义
#define W25Q_STATUS2_QE             (0x02)   // SR2 bit 1: Quad 使能（置1后 IO2/IO3 作数据线，不再是 WP#/HOLD#）

// -------------------- 驱动接口函数 --------------------
/**
  * @brief  初始化 W25Q128
  * @retval HAL 状态
  */

HAL_StatusTypeDef W25Q128_Init(void);

// 读取 ID
/**
  * @brief  读取 JEDEC ID (3字节)
  * @param  pID: 存储 ID 的指针 (0xXXYYYY: XX=制造商, YYYY=设备ID)
  * @retval HAL 状态
  */

HAL_StatusTypeDef W25Q128_ReadJEDECID(uint32_t *pID);
/**
  * @brief  读取 64 位唯一 ID
  * @param  pUniqueID: 8 字节缓冲区
  * @retval HAL 状态
  */

HAL_StatusTypeDef W25Q128_ReadUniqueID(uint8_t *pUniqueID); // 8 字节

// 读取状态寄存器
/**
  * @brief  读取状态寄存器 1
  */

HAL_StatusTypeDef W25Q128_ReadStatusReg1(uint8_t *pStatus);
/**
  * @brief  读取状态寄存器 2
  */

HAL_StatusTypeDef W25Q128_ReadStatusReg2(uint8_t *pStatus);
/**
  * @brief  读取状态寄存器 3
  */

HAL_StatusTypeDef W25Q128_ReadStatusReg3(uint8_t *pStatus);

// 写状态寄存器（需要先写使能）
/**
  * @brief  写状态寄存器 1
  */

HAL_StatusTypeDef W25Q128_WriteStatusReg1(uint8_t status);
/**
  * @brief  写状态寄存器 2
  */

HAL_StatusTypeDef W25Q128_WriteStatusReg2(uint8_t status);
/**
  * @brief  写状态寄存器 3
  */

HAL_StatusTypeDef W25Q128_WriteStatusReg3(uint8_t status);

// 等待写操作完成
/**
  * @brief  等待内部写操作完成（轮询 BUSY 位）
  * @param  Timeout: 超时时间（毫秒）
  * @retval HAL 状态
  */

HAL_StatusTypeDef W25Q128_WaitForWriteComplete(uint32_t Timeout);

// 写使能/禁止
/**
  * @brief  写使能
  * @retval HAL 状态
  */

HAL_StatusTypeDef W25Q128_WriteEnable(void);
/**
  * @brief  写禁止
  */

HAL_StatusTypeDef W25Q128_WriteDisable(void);

// -------------------- 擦除操作 --------------------
/**
  * @brief  扇区擦除 (4KB)
  * @param  SectorAddr: 扇区起始地址（必须 4KB 对齐）
  */

HAL_StatusTypeDef W25Q128_EraseSector(uint32_t SectorAddr);   // 4KB
/**
  * @brief  32KB 块擦除
  * @param  BlockAddr: 块起始地址（必须 32KB 对齐）
  */

HAL_StatusTypeDef W25Q128_EraseBlock32(uint32_t BlockAddr);   // 32KB
/**
  * @brief  64KB 块擦除
  * @param  BlockAddr: 块起始地址（必须 64KB 对齐）
  */

HAL_StatusTypeDef W25Q128_EraseBlock64(uint32_t BlockAddr);   // 64KB
/**
  * @brief  全片擦除（非常耗时，请谨慎使用）
  */

HAL_StatusTypeDef W25Q128_EraseChip(void);

// -------------------- 读写操作（阻塞模式）--------------------
// 读取任意长度数据（无跨页限制）
/**
  * @brief  从 W25Q128 读取任意长度数据
  * @param  ReadAddr: 起始地址
  * @param  pBuffer:  存储数据的指针
  * @param  NumByteToRead: 要读取的字节数
  * @retval HAL 状态
  */

HAL_StatusTypeDef W25Q128_ReadBytes(uint32_t ReadAddr, uint8_t *pBuffer, uint32_t NumByteToRead);

// 写入任意长度数据（自动处理跨页拆分）
/**
  * @brief  向 W25Q128 写入任意长度数据（自动处理跨页拆分）
  * @param  WriteAddr: 起始写入地址
  * @param  pBuffer:   数据源指针
  * @param  NumByteToWrite: 要写入的字节数
  * @retval HAL 状态
  */

HAL_StatusTypeDef W25Q128_WriteBytes(uint32_t WriteAddr, uint8_t *pBuffer, uint32_t NumByteToWrite);

// -------------------- 读写操作（DMA模式）--------------------
// 用于大块数据传输，非阻塞，需配合回调或状态查询
/**
  * @brief  使用 DMA 从 W25Q128 读取数据（非阻塞）
  * @note   传输完成后会触发 HAL_QSPI_CmdCpltCallback 回调
  */

HAL_StatusTypeDef W25Q128_ReadBytes_DMA(uint32_t ReadAddr, uint8_t *pBuffer, uint32_t NumByteToRead);
/**
  * @brief  使用 DMA 向 W25Q128 写入数据（非阻塞）
  * @note   - 自动处理跨页拆分，但每次页编程后会等待完成
  *         - 每页传输完成后会触发 HAL_QSPI_CmdCpltCallback 回调
  *         - 如果写入多页，回调会被触发多次
  */

HAL_StatusTypeDef W25Q128_WriteBytes_DMA(uint32_t WriteAddr, uint8_t *pBuffer, uint32_t NumByteToWrite);

// -------------------- Quad 四线模式（阻塞）--------------------
/**
  * @brief  使能 Quad 模式（置 Status Register 2 的 QE 位）
  * @note   置位后 IO2/IO3 不再作为 WP#/HOLD#，转为数据线。
  *         QE 置位后单线命令仍可正常使用，单线/四线可并存。
  *         四线读写前必须调用本函数一次。
  * @retval HAL 状态
  */

// 注意：四线读写前必须先调用 W25Q128_EnableQuad() 置 QE 位
//       QE 置位后单线命令仍可正常使用，二者可并存
HAL_StatusTypeDef W25Q128_EnableQuad(void);     // 使能 Quad（置 SR2.QE）
/**
  * @brief  禁能 Quad 模式（清 Status Register 2 的 QE 位）
  * @note   清除后 IO2/IO3 恢复为 WP#/HOLD#，四线命令将失效。
  */

HAL_StatusTypeDef W25Q128_DisableQuad(void);    // 禁能 Quad（清 SR2.QE）
/**
  * @brief  四线快速读取（Quad Output Fast Read, 0x6B, 1-1-4）
  * @note   指令/地址走单线，数据走四线，吞吐量为单线 4 倍。
  *         使用前必须先调用 W25Q128_EnableQuad() 置 QE 位。
  * @param  ReadAddr: 起始地址
  * @param  pBuffer:  存储数据的指针
  * @param  NumByteToRead: 要读取的字节数
  * @retval HAL 状态
  */

HAL_StatusTypeDef W25Q128_ReadBytes_Quad(uint32_t ReadAddr, uint8_t *pBuffer, uint32_t NumByteToRead);
/**
  * @brief  四线 DMA 快速读取（Quad Output Fast Read, 0x6B, 1-1-4）
  * @note   DMA 以总线速率排空 FIFO，无逐字 CPU 开销，可发挥四线带宽。
  *         轮询接收(CPU 排 FIFO)慢于单线总线，四线优势体现不出来；DMA 才能释放四线性能。
  *         使用前必须先调用 W25Q128_EnableQuad() 置 QE 位。
  *         Cortex-M4 无 D-Cache，DMA 写入的内存对 CPU 直接可见，无需缓存维护。
  * @param  ReadAddr: 起始地址
  * @param  pBuffer:  存储数据的指针（须位于 DMA 可访问内存区）
  * @param  NumByteToRead: 要读取的字节数
  * @retval HAL 状态
  */

HAL_StatusTypeDef W25Q128_ReadBytes_Quad_DMA(uint32_t ReadAddr, uint8_t *pBuffer, uint32_t NumByteToRead);
/**
  * @brief  四线页编程（Quad Input Page Program, 0x32, 1-1-4，自动跨页拆分）
  * @note   指令/地址走单线，数据走四线。
  *         使用前必须先调用 W25Q128_EnableQuad() 置 QE 位。
  * @param  WriteAddr: 起始写入地址
  * @param  pBuffer:   数据源指针
  * @param  NumByteToWrite: 要写入的字节数
  * @retval HAL 状态
  */

HAL_StatusTypeDef W25Q128_WriteBytes_Quad(uint32_t WriteAddr, uint8_t *pBuffer, uint32_t NumByteToWrite);

#endif