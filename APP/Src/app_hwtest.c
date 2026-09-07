/**
  * @file    test.c
  * @brief   外设测试函数实现
  * @note    每个测试函数独立运行，打印详细结果
  *          建议逐个调用，便于定位问题
  */

#include "app_hwtest.h"
#include "bsp_i2c.h"
#include "stlm75m2f.h"
#include "24lc256.h"
#include "bsp_spi.h"
#include "adcs7476.h"
#include "adcs7477.h"
#include "ad5421.h"
#include "bsp_qspi.h"
#include "w25q128jv.h"
#include "bsp_usart.h"
#include "bsp_gpio.h"
#include <stdio.h>
#include <string.h>

/* 辅助宏：打印分隔线 */
#define TEST_SEPARATOR(title) \
    printf("\n========== %s ==========\n", title)

#define TEST_RESULT(pass, msg) \
    printf("[%s] %s\n", (pass) ? "PASS" : "FAIL", msg)

/* ==================== I2C 总线扫描 ==================== */
void Test_I2C_Scan(void)
{
    printf("\n========== I2C Bus Scan ==========\n");
    uint8_t found = 0;
    for (uint16_t addr = 0x01; addr < 0x80; addr++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, addr, 3, 50) == HAL_OK) {
            printf("  Device found at 0x%02X\n", addr);
            found++;
        }
    }
    if (found == 0) {
        printf("  No devices found. Check connections/pull-ups.\n");
    } else {
        printf("  Total %d device(s) found.\n", found);
    }
}

/* ==================== 测试 STLM75M2F ==================== */
void Test_STLM75M2F(void)
{
    TEST_SEPARATOR("Testing STLM75M2F (Temperature Sensor)");
    
    // 1. 检查设备是否在线
    printf("Step 1: Check device ready (address 0x%02X)...\n", STLM75M2F_DEV_ADDR>>1);
    HAL_StatusTypeDef status = BSP_I2C_IsDeviceReady(STLM75M2F_DEV_ADDR, 10, 100);
    if (status != HAL_OK) {
        TEST_RESULT(0, "Device not ready");
        printf("  Please check:\n");
        printf("  - SCL/SDA connections and pull-up resistors (4.7kΩ)\n");
        printf("  - Power supply (3.3V)\n");
        printf("  - A0/A1/A2 pins = correct levels (currently expected 0x%02X)\n", STLM75M2F_DEV_ADDR);
        printf("  - I2C bus not busy (try scanning with Test_I2C_Scan())\n");
        return;
    }
    TEST_RESULT(1, "Device ready");
    
    // 2. 读取温度
    printf("Step 2: Read temperature...\n");
    float temperature = 0.0f;
    status = STLM75_ReadTemp(&temperature);
    if (status != HAL_OK) {
        TEST_RESULT(0, "Read temperature failed");
        return;
    }
    TEST_RESULT(1, "Temperature read");
    printf("  Temperature = %.2f °C\n", temperature);
    
    // 3. 多次读取验证稳定性
    printf("Step 3: Multiple readings (5 samples)...\n");
    for (int i = 0; i < 5; i++) {
        HAL_Delay(100);
        status = STLM75_ReadTemp(&temperature);
        if (status == HAL_OK) {
            printf("  Sample %d: %.2f °C\n", i+1, temperature);
        } else {
            printf("  Sample %d: FAILED\n", i+1);
        }
    }
}

/* ==================== 测试 24LC256 ==================== */
void Test_24LC256(void)
{
    TEST_SEPARATOR("Testing 24LC256 (EEPROM)");
    
    // 1. 检查设备是否在线
    printf("Step 1: Check device ready (address 0x%02X)...\n", EEPROM_24LC256_ADDR>>1);
    HAL_StatusTypeDef status = BSP_I2C_IsDeviceReady(EEPROM_24LC256_ADDR, 10, 100);
    if (status != HAL_OK) {
        TEST_RESULT(0, "Device not ready");
        printf("  Please check:\n");
        printf("  - SCL/SDA connections and pull-up resistors\n");
        printf("  - Power supply (3.3V)\n");
        printf("  - A0/A1/A2 pins = GND (address 0x50)\n");
        printf("  - WP pin = GND (write enable)\n");
        return;
    }
    TEST_RESULT(1, "Device ready");
    
    // 2. 单字节写入/读取
    printf("Step 2: Write/Read one byte...\n");
    uint16_t test_addr = 0x0000;
    uint8_t write_byte = 0xA5;
    uint8_t read_byte = 0;
    
    status = EEPROM_WriteByte(test_addr, write_byte);
    if (status != HAL_OK) {
        TEST_RESULT(0, "Write byte failed");
        return;
    }
    HAL_Delay(10);
    status = EEPROM_ReadByte(test_addr, &read_byte);
    if (status != HAL_OK) {
        TEST_RESULT(0, "Read byte failed");
        return;
    }
    if (read_byte == write_byte) {
        TEST_RESULT(1, "Byte test OK");
        printf("  Write: 0x%02X, Read: 0x%02X\n", write_byte, read_byte);
    } else {
        TEST_RESULT(0, "Byte mismatch");
        printf("  Wrote 0x%02X, read 0x%02X\n", write_byte, read_byte);
        return;
    }
    
    // 3. 字符串写入/读取
    printf("Step 3: Write/Read string...\n");
    const char *test_str = "Hello EEPROM!";
    uint16_t str_addr = 0x0010;
    uint8_t read_str[32] = {0};
    
    status = EEPROM_WriteBytes(str_addr, (uint8_t*)test_str, strlen(test_str) + 1);
    if (status != HAL_OK) {
        TEST_RESULT(0, "Write string failed");
        return;
    }
    HAL_Delay(10);
    status = EEPROM_ReadBytes(str_addr, read_str, strlen(test_str) + 1);
    if (status != HAL_OK) {
        TEST_RESULT(0, "Read string failed");
        return;
    }
    if (strcmp(test_str, (char*)read_str) == 0) {
        TEST_RESULT(1, "String test OK");
        printf("  Read: \"%s\"\n", read_str);
    } else {
        TEST_RESULT(0, "String mismatch");
        printf("  Wrote: \"%s\", Read: \"%s\"\n", test_str, read_str);
    }
}

/* ==================== 测试 ADCS7476 ==================== */
void Test_ADCS7476(void)
{
    TEST_SEPARATOR("Testing ADCS7476 (ADC)");
    
    // 1. 连续读取10次
    printf("Step 1: Read ADC values (10 samples)...\n");
    uint16_t adc_value = 0;
    HAL_StatusTypeDef status;
    uint32_t sum = 0;
    uint16_t min_val = 0xFFFF, max_val = 0;
    
    for (int i = 0; i < 10; i++) {
        status = ADCS7476_Read(&adc_value);
        if (status != HAL_OK) {
            TEST_RESULT(0, "Read sample failed");
            printf("  Sample %d error (err=%d)\n", i+1, status);
            return;
        }
        sum += adc_value;
        if (adc_value < min_val) min_val = adc_value;
        if (adc_value > max_val) max_val = adc_value;
        HAL_Delay(10);
    }
    uint16_t avg = sum / 10;
    float voltage = (float)avg * 3.3f / 4096.0f;
    TEST_RESULT(1, "10 samples read");
    printf("  Avg=%d, Min=%d, Max=%d, Voltage=%.3f V\n", avg, min_val, max_val, voltage);
    printf("  Note: If input is floating, readings may be near 0 or random.\n");
    printf("  Connect a known voltage (e.g., 1.65V) to verify accuracy.\n");
}

/* ==================== 测试 ADCS7477 ==================== */
void Test_ADCS7477(void)
{
    TEST_SEPARATOR("Testing ADCS7477 (ADC)");
    
    // 1. 连续读取10次
    printf("Step 1: Read ADC values (10 samples)...\n");
    uint16_t adc_value = 0;
    HAL_StatusTypeDef status;
    uint32_t sum = 0;
    uint16_t min_val = 0xFFFF, max_val = 0;
    
    for (int i = 0; i < 10; i++) {
        status = ADCS7477_Read(&adc_value);
        if (status != HAL_OK) {
            TEST_RESULT(0, "Read sample failed");
            printf("  Sample %d error (err=%d)\n", i+1, status);
            return;
        }
        sum += adc_value;
        if (adc_value < min_val) min_val = adc_value;
        if (adc_value > max_val) max_val = adc_value;
        HAL_Delay(10);
    }
    uint16_t avg = sum / 10;
    float voltage = (float)avg * 3.3f / 1024.0f;
    TEST_RESULT(1, "10 samples read");
    printf("  Avg=%d, Min=%d, Max=%d, Voltage=%.3f V\n", avg, min_val, max_val, voltage);
    printf("  Note: If input is floating, readings may be near 0 or random.\n");
    printf("  Connect a known voltage (e.g., 1.65V) to verify accuracy.\n");
}

/* ==================== 测试 AD5421 ==================== */
void Test_AD5421(void)
{
    TEST_SEPARATOR("Testing AD5421 (DAC)");
    
    // 1. 初始化芯片
    printf("Step 1: Initialize AD5421...\n");
    HAL_StatusTypeDef status = AD5421_Init();
    if (status != HAL_OK) {
        TEST_RESULT(0, "Init failed");
        printf("  Please check:\n");
        printf("  - SPI2 connections (SCK, MOSI, MISO, SYNC)\n");
        printf("  - Power supplies (AVDD, DVDD)\n");
        printf("  - External loop power (LOOP+ / LOOP-)\n");
        return;
    }
    TEST_RESULT(1, "Init OK");
    
    // 2. 读取故障寄存器
    printf("Step 2: Read Fault Register...\n");
    uint16_t fault = 0;
    status = AD5421_ReadFaultRegister(&fault);
    if (status != HAL_OK) {
        TEST_RESULT(0, "Read fault failed");
        return;
    }
    TEST_RESULT(1, "Fault register read");
    printf("  Fault Register = 0x%04X\n", fault);
    if (fault & 0x0001) {
        printf("  [WARN] Fault bit set (possibly no loop power or overcurrent)\n");
    } else {
        printf("  No fault detected.\n");
    }
    
    // 3. 设置输出电流 8mA 和 12mA
    printf("Step 3: Set output currents (8mA and 12mA)...\n");
    uint16_t dac_8mA = (uint16_t)(8.0f / 16.0f * 65535.0f);
    uint16_t dac_12mA = (uint16_t)(12.0f / 16.0f * 65535.0f);
    
    status = AD5421_SetDACOutput(dac_8mA);
    if (status == HAL_OK) {
        printf("  Set 8mA code 0x%04X OK\n", dac_8mA);
    } else {
        printf("  Set 8mA failed (err=%d)\n", status);
    }
    HAL_Delay(100);
    
    status = AD5421_SetDACOutput(dac_12mA);
    if (status == HAL_OK) {
        printf("  Set 12mA code 0x%04X OK\n", dac_12mA);
    } else {
        printf("  Set 12mA failed (err=%d)\n", status);
    }
    printf("  Please measure loop current to verify (should be ~8mA and ~12mA).\n");
}

/* ==================== 测试 W25Q128JV ==================== */
void Test_W25Q128JV(void)
{
    TEST_SEPARATOR("Testing W25Q128JV (Flash)");
    
    // 1. 初始化并读取 JEDEC ID
    printf("Step 1: Initialize and read JEDEC ID...\n");
    HAL_StatusTypeDef status = W25Q128_Init();
    if (status != HAL_OK) {
        TEST_RESULT(0, "Init failed");
        printf("  Please check:\n");
        printf("  - QSPI connections (CLK, IO0-IO3, CS)\n");
        printf("  - Power supply (3.3V)\n");
        printf("  - WP and HOLD pins = VCC (high)\n");
        printf("  - QSPI clock speed (try lowering prescaler in CubeMX)\n");
        return;
    }
    TEST_RESULT(1, "Init OK");
    
    uint32_t jedec_id = 0;
    status = W25Q128_ReadJEDECID(&jedec_id);
    if (status == HAL_OK) {
        printf("  JEDEC ID = 0x%06X (expected 0xEF4018)\n", (unsigned int)jedec_id);
        if (jedec_id != 0xEF4018) {
            printf("  [WARN] ID mismatch - may be different chip or connection issue.\n");
        }
    } else {
        printf("  [WARN] Could not read JEDEC ID (err=%d)\n", status);
    }
    
    // 2. 擦除扇区 0
    printf("Step 2: Erase sector 0 (address 0x000000)...\n");
    status = W25Q128_EraseSector(0x000000);
    if (status != HAL_OK) {
        TEST_RESULT(0, "Erase sector failed");
        return;
    }
    TEST_RESULT(1, "Sector erased");
    
    // 3. 写入/读取验证
    printf("Step 3: Write/Read 128 bytes...\n");
    uint8_t write_buf[128];
    uint8_t read_buf[128];
    for (int i = 0; i < 128; i++) {
        write_buf[i] = i & 0xFF;
    }
    
    status = W25Q128_WriteBytes(0x000000, write_buf, 128);
    if (status != HAL_OK) {
        TEST_RESULT(0, "Write failed");
        return;
    }
    TEST_RESULT(1, "Write OK");
    
    status = W25Q128_ReadBytes(0x000000, read_buf, 128);
    if (status != HAL_OK) {
        TEST_RESULT(0, "Read failed");
        return;
    }
    TEST_RESULT(1, "Read OK");
    
    if (memcmp(write_buf, read_buf, 128) == 0) {
        TEST_RESULT(1, "Data match!");
    } else {
        TEST_RESULT(0, "Data mismatch!");
        printf("  First 8 bytes: Write ");
        for (int i = 0; i < 8; i++) printf("%02X ", write_buf[i]);
        printf(", Read ");
        for (int i = 0; i < 8; i++) printf("%02X ", read_buf[i]);
        printf("\n");
    }
}

/* ==================== 测试 W25Q128JV 四线模式（对照单线测速）==================== */
void Test_W25Q128JV_Quad(void)
{
#define QUAD_TEST_SIZE   (4096)   /* 1 个扇区 */
    static uint8_t wbuf[QUAD_TEST_SIZE];
    static uint8_t rbuf[QUAD_TEST_SIZE];
    HAL_StatusTypeDef status;
    uint32_t t0, t1, us_1w_w, us_1w_r, us_q_w, us_q_r, us_q_r_dma, ratio;

    TEST_SEPARATOR("Testing W25Q128JV (Quad SPI)");

    /* 1. 初始化 + 使能 Quad */
    printf("Step 1: Init and enable Quad mode...\n");
    status = W25Q128_Init();
    if (status != HAL_OK) { TEST_RESULT(0, "Init failed"); return; }
    TEST_RESULT(1, "Init OK");

    status = W25Q128_EnableQuad();
    if (status != HAL_OK) { TEST_RESULT(0, "EnableQuad failed"); return; }
    TEST_RESULT(1, "Quad enabled (QE bit set)");

    /* 使能 DWT 周期计数器（微秒级测速）*/
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    /* 2. 准备数据 + 擦除 */
    for (int i = 0; i < QUAD_TEST_SIZE; i++) wbuf[i] = (uint8_t)(i & 0xFF);

    printf("Step 2: Erase sector 0 (%d bytes)...\n", QUAD_TEST_SIZE);
    status = W25Q128_EraseSector(0x000000);
    if (status != HAL_OK) { TEST_RESULT(0, "Erase failed"); return; }
    TEST_RESULT(1, "Sector erased");

    /* 3. 单线写测速（含 Flash 内部编程时间）*/
    printf("Step 3: 1-line write %d bytes...\n", QUAD_TEST_SIZE);
    t0 = DWT->CYCCNT;
    status = W25Q128_WriteBytes(0x000000, wbuf, QUAD_TEST_SIZE);
    t1 = DWT->CYCCNT;
    if (status != HAL_OK) { TEST_RESULT(0, "1-line write failed"); return; }
    us_1w_w = (t1 - t0) / (SystemCoreClock / 1000000);
    printf("  1-line write: %lu us\n", (unsigned long)us_1w_w);
    TEST_RESULT(1, "1-line write OK");

    /* 4. 单线读校验 + 测速（纯总线传输，无内部延时）*/
    printf("Step 4: 1-line read %d bytes...\n", QUAD_TEST_SIZE);
    memset(rbuf, 0, sizeof(rbuf));
    t0 = DWT->CYCCNT;
    status = W25Q128_ReadBytes(0x000000, rbuf, QUAD_TEST_SIZE);
    t1 = DWT->CYCCNT;
    if (status != HAL_OK) { TEST_RESULT(0, "1-line read failed"); return; }
    us_1w_r = (t1 - t0) / (SystemCoreClock / 1000000);
    printf("  1-line read: %lu us\n", (unsigned long)us_1w_r);
    if (memcmp(wbuf, rbuf, QUAD_TEST_SIZE) == 0) TEST_RESULT(1, "1-line data match");
    else { TEST_RESULT(0, "1-line data mismatch"); return; }

    /* 5. 四线写测速（重新擦除后写）*/
    printf("Step 5: Erase & quad write %d bytes...\n", QUAD_TEST_SIZE);
    status = W25Q128_EraseSector(0x000000);
    if (status != HAL_OK) { TEST_RESULT(0, "Erase failed"); return; }
    t0 = DWT->CYCCNT;
    status = W25Q128_WriteBytes_Quad(0x000000, wbuf, QUAD_TEST_SIZE);
    t1 = DWT->CYCCNT;
    if (status != HAL_OK) { TEST_RESULT(0, "Quad write failed"); return; }
    us_q_w = (t1 - t0) / (SystemCoreClock / 1000000);
    printf("  quad write: %lu us\n", (unsigned long)us_q_w);
    TEST_RESULT(1, "Quad write OK");

    /* 6. 四线读校验 + 测速（轮询：CPU 排 FIFO）*/
    printf("Step 6: quad read %d bytes (polling)...\n", QUAD_TEST_SIZE);
    memset(rbuf, 0, sizeof(rbuf));
    t0 = DWT->CYCCNT;
    status = W25Q128_ReadBytes_Quad(0x000000, rbuf, QUAD_TEST_SIZE);
    t1 = DWT->CYCCNT;
    if (status != HAL_OK) { TEST_RESULT(0, "Quad read failed"); return; }
    us_q_r = (t1 - t0) / (SystemCoreClock / 1000000);
    printf("  quad read (poll): %lu us\n", (unsigned long)us_q_r);
    if (memcmp(wbuf, rbuf, QUAD_TEST_SIZE) == 0) TEST_RESULT(1, "Quad data match");
    else { TEST_RESULT(0, "Quad data mismatch"); return; }

    /* 7. 四线 DMA 读校验 + 测速（DMA 以总线速率排 FIFO，无逐字 CPU 开销）*/
    printf("Step 7: quad read %d bytes (DMA)...\n", QUAD_TEST_SIZE);
    memset(rbuf, 0, sizeof(rbuf));
    t0 = DWT->CYCCNT;
    status = W25Q128_ReadBytes_Quad_DMA(0x000000, rbuf, QUAD_TEST_SIZE);
    t1 = DWT->CYCCNT;
    if (status != HAL_OK) { TEST_RESULT(0, "Quad DMA read failed"); return; }
    us_q_r_dma = (t1 - t0) / (SystemCoreClock / 1000000);
    printf("  quad read (DMA): %lu us\n", (unsigned long)us_q_r_dma);
    if (memcmp(wbuf, rbuf, QUAD_TEST_SIZE) == 0) TEST_RESULT(1, "Quad DMA data match");
    else { TEST_RESULT(0, "Quad DMA data mismatch"); return; }

    /* 8. 速度对比（整数运算，避免依赖浮点 printf）*/
    printf("Step 8: Speed comparison (size=%d)\n", QUAD_TEST_SIZE);
    printf("  Write : 1-line=%lu us, quad=%lu us", (unsigned long)us_1w_w, (unsigned long)us_q_w);
    if (us_q_w > 0) { ratio = us_1w_w * 100U / us_q_w; printf(", speedup=%lu.%02lux", (unsigned long)(ratio/100U), (unsigned long)(ratio%100U)); }
    printf("  (write dominated by Flash program time)\n");
    printf("  Read  : 1-line(poll)=%lu us, quad(poll)=%lu us", (unsigned long)us_1w_r, (unsigned long)us_q_r);
    if (us_q_r > 0) { ratio = us_1w_r * 100U / us_q_r; printf(", speedup=%lu.%02lux", (unsigned long)(ratio/100U), (unsigned long)(ratio%100U)); }
    printf("  (polling is CPU-bound, no speedup)\n");
    printf("  Read  : quad(DMA)=%lu us", (unsigned long)us_q_r_dma);
    if (us_q_r_dma > 0) { ratio = us_1w_r * 100U / us_q_r_dma; printf(", vs 1-line speedup=%lu.%02lux", (unsigned long)(ratio/100U), (unsigned long)(ratio%100U)); }
    printf("  (DMA unleashes quad bandwidth, expect ~3-4x)\n");
#undef QUAD_TEST_SIZE
}

/* ============================================================
 *  串口测试函数
 *
 *  UART4 = printf 调试口，无需单独测试函数，直接 printf 即可。
 *  USART1 = 显示屏通信口
 *  USART2 = HART (AD5700)
 *  USART3 = RS485（带 DE 方向控制）
 *
 *  测试方法：发送已知字符串，若短接 TX-RX（回环）则接收比对。
 *  无回环时发送成功即 PASS，接收超时属正常。
 * ============================================================ */

void Test_DISP(void)
{   
    TEST_SEPARATOR("Testing USART1 (Display Port)");

    const char *tx_str = "Hello USART1!\r\n";
    uint8_t rx_buf[32] = {0};
    uint16_t len = (uint16_t)strlen(tx_str);

    /* 1. 发送 */
    printf("Step 1: Send \"%s\"", tx_str);
    HAL_StatusTypeDef status = BSP_USART_Send(BSP_USART_INSTANCE_1, (const uint8_t *)tx_str, len, 100);
    if (status != HAL_OK) {
        TEST_RESULT(0, "Send failed");
        printf("  err=%d, check PA9(TX)/PA10(RX) wiring\n", status);
        return;
    }
    TEST_RESULT(1, "Send OK");

    /* 2. 回环接收（需短接 PA9-PA10）*/
    printf("Step 2: Receive echo (short PA9-PA10 to test loopback)...\n");
    status = BSP_USART_Receive(BSP_USART_INSTANCE_1, rx_buf, len, 200);
    if (status == HAL_OK && memcmp(tx_str, rx_buf, len) == 0) {
        TEST_RESULT(1, "Loopback OK");
        printf("  RX: \"%s\"", rx_buf);
    } else if (status == HAL_TIMEOUT) {
        printf("  [INFO] Receive timeout (normal if no loopback wire)\n");
    } else {
        TEST_RESULT(0, "Loopback mismatch");
        printf("  err=%d\n", status);
    }
}

void Test_HART(void)
{
    TEST_SEPARATOR("Testing USART2 (HART / AD5700)");

    const char *tx_str = "Hello HART!\r\n";
    uint8_t rx_buf[32] = {0};
    uint16_t len = (uint16_t)strlen(tx_str);

    /* 1. 发送（经 AD5700 调制为 FSK 输出）*/
    printf("Step 1: Send \"%s\"", tx_str);
    printf("  Note: AD5700 modulates UART data to FSK. Enable TX mode first.\n");
    BSP_GPIO_HART_SetTransmit();   /* AD5700 RTS=0 → 发送模式 */
    HAL_Delay(1);

    HAL_StatusTypeDef status = BSP_USART_Send(BSP_USART_INSTANCE_2, (const uint8_t *)tx_str, len, 100);
    if (status != HAL_OK) {
        BSP_GPIO_HART_SetReceive();
        TEST_RESULT(0, "Send failed");
        printf("  err=%d, check PD5(TX)/PD6(RX) and AD5700 wiring\n", status);
        return;
    }
    TEST_RESULT(1, "Send OK (FSK modulated output)");

    /* 2. 切回接收模式，尝试接收（需 HART 主设备回环或短接）*/
    BSP_GPIO_HART_SetReceive();   /* AD5700 RTS=1 → 接收模式 */
    printf("Step 2: Switch to RX mode, try receive echo...\n");
    status = BSP_USART_Receive(BSP_USART_INSTANCE_2, rx_buf, len, 200);
    if (status == HAL_OK && memcmp(tx_str, rx_buf, len) == 0) {
        TEST_RESULT(1, "Loopback OK");
        printf("  RX: \"%s\"", rx_buf);
    } else if (status == HAL_TIMEOUT) {
        printf("  [INFO] Receive timeout (normal — HART loopback needs external device)\n");
    } else {
        TEST_RESULT(0, "Receive error");
        printf("  err=%d\n", status);
    }
}

void Test_RS485(void)
{
    TEST_SEPARATOR("Testing USART3 (RS485)");

    const char *tx_str = "Hello RS485!\r\n";
    uint8_t rx_buf[32] = {0};
    uint16_t len = (uint16_t)strlen(tx_str);

    /* 1. 切发送模式 + 发送 */
    printf("Step 1: Set TX mode (DE=HIGH), send \"%s\"", tx_str);
    BSP_USART_RS485_SetTxMode(BSP_USART_INSTANCE_3);
    HAL_Delay(1);

    HAL_StatusTypeDef status = BSP_USART_Send(BSP_USART_INSTANCE_3, (const uint8_t *)tx_str, len, 100);
    if (status != HAL_OK) {
        BSP_USART_RS485_SetRxMode(BSP_USART_INSTANCE_3);
        TEST_RESULT(0, "Send failed");
        printf("  err=%d, check PD8(TX)/PD9(RX) and DE(PA8)\n", status);
        return;
    }
    TEST_RESULT(1, "Send OK");

    /* 2. 切接收模式 + 接收回环（需 RS485 收发器 A-B 回环或外接设备）*/
    printf("Step 2: Set RX mode (DE=LOW), try receive echo...\n");
    BSP_USART_RS485_SetRxMode(BSP_USART_INSTANCE_3);

    status = BSP_USART_Receive(BSP_USART_INSTANCE_3, rx_buf, len, 200);
    if (status == HAL_OK && memcmp(tx_str, rx_buf, len) == 0) {
        TEST_RESULT(1, "Loopback OK");
        printf("  RX: \"%s\"", rx_buf);
    } else if (status == HAL_TIMEOUT) {
        printf("  [INFO] Receive timeout (normal — needs RS485 loopback or peer device)\n");
    } else {
        TEST_RESULT(0, "Receive error");
        printf("  err=%d\n", status);
    }
}