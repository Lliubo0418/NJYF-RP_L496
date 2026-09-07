#ifndef __APP_HWTEST_H
#define __APP_HWTEST_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* I2C 设备测试 */
void Test_STLM75M2F(void);
void Test_24LC256(void);

/* SPI 设备测试 */
void Test_ADCS7476(void);
void Test_ADCS7477(void);
void Test_AD5421(void);

/* QSPI 设备测试 */
void Test_W25Q128JV(void);
void Test_W25Q128JV_Quad(void);   /* 四线模式对照测速 */

/* 辅助函数：I2C 总线扫描 */
void Test_I2C_Scan(void);

/* ---- USART1 测试（显示屏通信口）---- */

void Test_DISP(void);   /* USART1：显示屏通信口 */
/* ---- USART2 测试（HART / AD5700）---- */

void Test_HART(void);   /* USART2：HART (AD5700) */
/* ---- USART3 测试（RS485）---- */

void Test_RS485(void);   /* USART3：RS485 */

#ifdef __cplusplus
}
#endif

#endif /* __TEST_H */