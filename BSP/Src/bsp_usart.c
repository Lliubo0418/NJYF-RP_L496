#include "bsp_usart.h"
#include "bsp_gpio.h"
#include <stdio.h>
#include <stdarg.h>

/* 串口用途与引脚（来自 usart.c MspInit）：
 *   UART4  调试 printf      PA0=TX  PA1=RX
 *   USART1 显示屏(STM32F1)  PA9=TX  PA10=RX
 *   USART2 HART(AD5700)     PA2=TX  PA3=RX   控制: RTS=PC0 CD=PC1 RST=PC2 (见 bsp_gpio)
 *   USART3 RS485            PC10=TX PC11=RX  方向: DE=PA8 (见 bsp_gpio)
 */
// 声明 CubeMX 生成的 UART 句柄
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;

// 映射枚举到句柄指针
static UART_HandleTypeDef* const uart_handles[BSP_USART_INSTANCE_MAX] = {
    &huart1,
    &huart2,
    &huart3,
    &huart4
};

// 存储每个串口的接收回调函数
static BSP_USART_RxCallback_t rx_callbacks[BSP_USART_INSTANCE_MAX] = {NULL};

// -------------------- 内部辅助函数 --------------------
static BSP_USART_Instance_t GetInstanceFromHandle(UART_HandleTypeDef *huart)
{
    for (int i = 0; i < BSP_USART_INSTANCE_MAX; i++) {
        if (uart_handles[i] == huart) {
            return (BSP_USART_Instance_t)i;
        }
    }
    return BSP_USART_INSTANCE_MAX;
}

// -------------------- 公共接口实现 --------------------

void BSP_USART_Init(void)
{
    // CubeMX 已经完成硬件初始化，此处无需重复操作
    // 如果需要额外配置（如中断优先级），可以在这里添加
}

HAL_StatusTypeDef BSP_USART_Send(BSP_USART_Instance_t instance, const uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    if (instance >= BSP_USART_INSTANCE_MAX || pData == NULL || Size == 0) {
        return HAL_ERROR;
    }
    return HAL_UART_Transmit(uart_handles[instance], (uint8_t*)pData, Size, Timeout);
}

HAL_StatusTypeDef BSP_USART_Receive(BSP_USART_Instance_t instance, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    if (instance >= BSP_USART_INSTANCE_MAX || pData == NULL || Size == 0) {
        return HAL_ERROR;
    }
    return HAL_UART_Receive(uart_handles[instance], pData, Size, Timeout);
}

HAL_StatusTypeDef BSP_USART_Transmit_IT(BSP_USART_Instance_t instance, const uint8_t *pData, uint16_t Size)
{
    if (instance >= BSP_USART_INSTANCE_MAX || pData == NULL || Size == 0) {
        return HAL_ERROR;
    }
    return HAL_UART_Transmit_IT(uart_handles[instance], (uint8_t*)pData, Size);
}

HAL_StatusTypeDef BSP_USART_Receive_IT(BSP_USART_Instance_t instance, uint8_t *pData, uint16_t Size)
{
    if (instance >= BSP_USART_INSTANCE_MAX || pData == NULL || Size == 0) {
        return HAL_ERROR;
    }
    return HAL_UART_Receive_IT(uart_handles[instance], pData, Size);
}

HAL_StatusTypeDef BSP_USART_AbortReceive_IT(BSP_USART_Instance_t instance)
{
    if (instance >= BSP_USART_INSTANCE_MAX) {
        return HAL_ERROR;
    }
    return HAL_UART_AbortReceive_IT(uart_handles[instance]);
}

void BSP_USART_RegisterRxCallback(BSP_USART_Instance_t instance, BSP_USART_RxCallback_t callback)
{
    if (instance < BSP_USART_INSTANCE_MAX) {
        rx_callbacks[instance] = callback;
    }
}

UART_HandleTypeDef* BSP_USART_GetHandle(BSP_USART_Instance_t instance)
{
    if (instance >= BSP_USART_INSTANCE_MAX) {
        return NULL;
    }
    return uart_handles[instance];
}

// -------------------- RS485 收发模式切换（USART3, DE=PC12）--------------------
// 收发器 DE 与 /RE 共接 PC12：HIGH=发送, LOW=接收。引脚操作集中在 bsp_gpio。

void BSP_USART_RS485_SetTxMode(BSP_USART_Instance_t instance)
{
    if (instance == BSP_USART_INSTANCE_3) {
        BSP_GPIO_RS485_TxMode();
    }
}

void BSP_USART_RS485_SetRxMode(BSP_USART_Instance_t instance)
{
    if (instance == BSP_USART_INSTANCE_3) {
        BSP_GPIO_RS485_RxMode();
    }
}

// -------------------- printf 重定向到 USART4（调试串口） --------------------

/**
  * @brief  重定向 printf 到 USART4（调试串口）
  * @note   这样可以直接使用 printf() 输出调试信息到 USART4
  */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart4, (uint8_t*)&ch, 1, 0xFFFF);
    return ch;
}

// -------------------- HAL 中断回调转发 --------------------

/**
  * @brief  UART 接收完成回调（由 HAL 库调用）
  * @param  huart: UART 句柄指针
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BSP_USART_Instance_t inst = GetInstanceFromHandle(huart);
    if (inst >= BSP_USART_INSTANCE_MAX) return;

    // 如果注册了回调，且当前是单字节接收完成（Size == 1），则触发
    if (rx_callbacks[inst] != NULL && huart->RxXferSize == 1) {
        uint8_t data = huart->pRxBuffPtr[0];
        rx_callbacks[inst](inst, data);
    }
}

// 可选的：发送完成回调（如果需要通知上层）
// void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
// {
//     // 处理发送完成事件
// }