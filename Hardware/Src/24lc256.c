#include "24lc256.h"
#include <string.h>

/* ⚠ I2C1 跑 100 kHz：HAL_I2C_Mem_Read() 的超时是【整段传输共享】（tickstart 只在函数入口取一次），
   10 KB 基线一次读完需约 926 ms，固定 100 ms 必然超时 ⇒ 必须分段，每段独立计 100 ms 预算 */
#define EEPROM_READ_CHUNK   (256u)

/* ⚠ 24LC256 写周期典型 5ms、最大 10ms：下面的 100ms 超时按此留有裕量 */
static HAL_StatusTypeDef EEPROM_WaitForWriteComplete(void)
{
    uint32_t tickstart = HAL_GetTick();
    HAL_StatusTypeDef status;

    do {
        status = BSP_I2C_IsDeviceReady(EEPROM_24LC256_ADDR, 1, 5);
        if (status == HAL_OK) {
            return HAL_OK;
        }
        HAL_Delay(1);
    } while ((HAL_GetTick() - tickstart) < 100);

    return HAL_TIMEOUT;
}

HAL_StatusTypeDef EEPROM_WriteByte(uint16_t addr, uint8_t data)
{
    HAL_StatusTypeDef status;

    status = BSP_I2C_Write(EEPROM_24LC256_ADDR, addr, I2C_MEMADD_SIZE_16BIT, &data, 1, 100);
    if (status != HAL_OK) {
        return status;
    }

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

    status = BSP_I2C_Write(EEPROM_24LC256_ADDR, addr, I2C_MEMADD_SIZE_16BIT, pData, len, 100);
    if (status != HAL_OK) {
        return status;
    }

    return EEPROM_WaitForWriteComplete();
}

HAL_StatusTypeDef EEPROM_ReadBytes(uint16_t addr, uint8_t *pData, uint16_t len)
{
    uint16_t offset = 0u;
    uint32_t cur_addr;

    if (pData == NULL || len == 0) {
        return HAL_ERROR;
    }

    while (len > 0u) {
        /* 按 256 字节分段：单段约 23 ms 远小于 100 ms 预算；上电读 10 KB 共约 0.95 s，运行期无调用 */
        uint16_t chunk = (len > EEPROM_READ_CHUNK) ? (uint16_t)EEPROM_READ_CHUNK : len;
        HAL_StatusTypeDef st;

        cur_addr = (uint32_t)addr + (uint32_t)offset;
        if (cur_addr > 0xFFFFu) {
            return HAL_ERROR;
        }

        st = BSP_I2C_Read(EEPROM_24LC256_ADDR, (uint16_t)cur_addr,
                              I2C_MEMADD_SIZE_16BIT, pData + offset, chunk, 100);
        if (st != HAL_OK) {
            return st;
        }

        offset += chunk;
        len    -= chunk;
    }

    return HAL_OK;
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
        /* ⚠ 24LC256 页写达页尾会回卷覆盖页首：必须按 64 字节页边界切分 */
        uint16_t page_remain = EEPROM_PAGE_SIZE - (addr % EEPROM_PAGE_SIZE);
        bytes_to_write = (len < page_remain) ? len : page_remain;

        status = EEPROM_WritePage(addr, pData + offset, bytes_to_write);
        if (status != HAL_OK) {
            return status;
        }

        addr += bytes_to_write;
        offset += bytes_to_write;
        len -= bytes_to_write;
    }

    return HAL_OK;
}