#ifndef __BSP_USART_H
#define __BSP_USART_H

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// 枚举串口实例
typedef enum {
    BSP_USART_INSTANCE_1 = 0,   // 与STM32F1显示屏通信（自定义协议）
    BSP_USART_INSTANCE_2,       // HART通信（AD5700）
    BSP_USART_INSTANCE_3,       // RS485通信
    BSP_USART_INSTANCE_4,       // 调试串口（printf输出）
    BSP_USART_INSTANCE_MAX
} BSP_USART_Instance_t;

// 接收回调函数类型
typedef void (*BSP_USART_RxCallback_t)(BSP_USART_Instance_t instance, uint8_t data);

// 初始化所有串口
/**
  * @brief  BSP 层 USART 初始化
  * @note   CubeMX 已初始化各串口，此函数仅做必要的额外配置
  */

void BSP_USART_Init(void);

// 阻塞发送
/**
  * @brief  阻塞发送数据
  * @param  instance: 串口实例
  * @param  pData:    待发送数据指针
  * @param  Size:     数据长度（字节）
  * @param  Timeout:  超时时间（毫秒）
  * @retval HAL 状态
  */

HAL_StatusTypeDef BSP_USART_Send(BSP_USART_Instance_t instance, const uint8_t *pData, uint16_t Size, uint32_t Timeout);

// 阻塞接收
/**
  * @brief  阻塞接收数据
  * @param  instance: 串口实例
  * @param  pData:    接收缓冲区指针
  * @param  Size:     要接收的字节数
  * @param  Timeout:  超时时间（毫秒）
  * @retval HAL 状态
  */

HAL_StatusTypeDef BSP_USART_Receive(BSP_USART_Instance_t instance, uint8_t *pData, uint16_t Size, uint32_t Timeout);

// 中断发送（异步）
/**
  * @brief  中断发送（异步）
  * @param  instance: 串口实例
  * @param  pData:    待发送数据指针
  * @param  Size:     数据长度（字节）
  * @retval HAL 状态
  */

HAL_StatusTypeDef BSP_USART_Transmit_IT(BSP_USART_Instance_t instance, const uint8_t *pData, uint16_t Size);

// 中断接收（异步）
/**
  * @brief  中断接收（异步）
  * @param  instance: 串口实例
  * @param  pData:    接收缓冲区指针
  * @param  Size:     要接收的字节数
  * @retval HAL 状态
  */

HAL_StatusTypeDef BSP_USART_Receive_IT(BSP_USART_Instance_t instance, uint8_t *pData, uint16_t Size);

// 注册接收回调函数
/**
  * @brief  注册接收回调函数
  * @param  instance: 串口实例
  * @param  callback: 回调函数指针
  */

void BSP_USART_RegisterRxCallback(BSP_USART_Instance_t instance, BSP_USART_RxCallback_t callback);

// 取消中断接收
/**
  * @brief  取消中断接收
  * @param  instance: 串口实例
  * @retval HAL 状态
  */

HAL_StatusTypeDef BSP_USART_AbortReceive_IT(BSP_USART_Instance_t instance);

// RS485 收发模式切换（仅对 USART3 有效）
/**
  * @brief  设置 RS485 为发送模式（仅对 USART3 有效）
  * @param  instance: 串口实例
  */

void BSP_USART_RS485_SetTxMode(BSP_USART_Instance_t instance);
/**
  * @brief  设置 RS485 为接收模式（仅对 USART3 有效）
  * @param  instance: 串口实例
  */

void BSP_USART_RS485_SetRxMode(BSP_USART_Instance_t instance);

// 获取串口句柄
/**
  * @brief  获取串口句柄
  * @param  instance: 串口实例
  * @retval UART句柄指针，失败返回NULL
  */

UART_HandleTypeDef* BSP_USART_GetHandle(BSP_USART_Instance_t instance);

#endif