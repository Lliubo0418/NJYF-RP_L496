#include "24lc256.h"
#include <string.h> // for memcpy

// 等待 EEPROM 内部写周期完成
static HAL_StatusTypeDef EEPROM_WaitForWriteComplete(void)
{
    uint32_t tickstart = HAL_GetTick();
    HAL_StatusTypeDef status;

    do {
        // 每5ms检查一次设备是否就绪，直到超时（典型写周期5ms，最大10ms）[reference:5]
        status = BSP_I2C_IsDeviceReady(EEPROM_24LC256_ADDR, 1, 5);
        if (status == HAL_OK) {
            return HAL_OK;
        }
        HAL_Delay(1);
    } while ((HAL_GetTick() - tickstart) < 100); // 超时设为100ms

    return HAL_TIMEOUT;
}

HAL_StatusTypeDef EEPROM_WriteByte(uint16_t addr, uint8_t data)
{
    HAL_StatusTypeDef status;

    // 执行写操作
    status = BSP_I2C_Write(EEPROM_24LC256_ADDR, addr, I2C_MEMADD_SIZE_16BIT, &data, 1, 100);
    if (status != HAL_OK) {
        return status;
    }

    // 等待内部写周期完成
    return EEPROM_WaitForWriteComplete();
}

HAL_StatusTypeDef EEPROM_ReadByte(uint16_t addr, uint8_t *data)
{
    return BSP_I2C_Read(EEPROM_24LC256_ADDR, addr, I2C_MEMADD_SIZE_16BIT, data, 1, 100);
}

HAL_StatusTypeDef EEPROM_WritePage(uint16_t addr, uint8_t *pData, uint16_t len)
{
    HAL_StatusTypeDef status;

    if (pData == NULL || len == 0 || len > EEPROM_PAGE_SIZE) {
        return HAL_ERROR;
    }

    // 执行页写入操作
    status = BSP_I2C_Write(EEPROM_24LC256_ADDR, addr, I2C_MEMADD_SIZE_16BIT, pData, len, 100);
    if (status != HAL_OK) {
        return status;
    }

    // 等待内部写周期完成
    return EEPROM_WaitForWriteComplete();
}

HAL_StatusTypeDef EEPROM_ReadBytes(uint16_t addr, uint8_t *pData, uint16_t len)
{
    if (pData == NULL || len == 0) {
        return HAL_ERROR;
    }
    // 24LC256支持顺序读取，一次可读取任意长度
    return BSP_I2C_Read(EEPROM_24LC256_ADDR, addr, I2C_MEMADD_SIZE_16BIT, pData, len, 100);
}

HAL_StatusTypeDef EEPROM_WriteBytes(uint16_t addr, uint8_t *pData, uint16_t len)
{
    uint16_t offset = 0;
    uint16_t bytes_to_write;
    HAL_StatusTypeDef status;

    if (pData == NULL || len == 0) {
        return HAL_ERROR;
    }

    while (len > 0) {
        // 计算当前页剩余空间
        uint16_t page_remain = EEPROM_PAGE_SIZE - (addr % EEPROM_PAGE_SIZE);
        bytes_to_write = (len < page_remain) ? len : page_remain;

        // 执行页写入
        status = EEPROM_WritePage(addr, pData + offset, bytes_to_write);
        if (status != HAL_OK) {
            return status;
        }

        // 更新变量
        addr += bytes_to_write;
        offset += bytes_to_write;
        len -= bytes_to_write;
    }

    return HAL_OK;
}