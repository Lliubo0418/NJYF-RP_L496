#include "bsp_usart.h"
#include "bsp_gpio.h"
#include <stdio.h>
#include <stdarg.h>
#include "bsp_debug.h"   /* 串口调试总开关：DEBUG_PRINT_ENABLE=0 时 printf 编译期消除（须在 stdio.h 之后）*/

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
// 记录每个串口单字节接收的缓冲指针，供 ORE 恢复时重启接收
static uint8_t* s_rx_ptr[BSP_USART_INSTANCE_MAX] = {NULL};

/* DMA 接收：USART1 专用，根治 ORE。
 * DMA 硬件自动从 RDR 搬字节到缓冲区，不依赖 ISR 实时响应，
 * TIM7（优先级 1）占满 CPU 时 DMA 照收不误。空闲事件回调里批量投递到上层。 */
#define BSP_USART1_DMA_RX_BUF_SIZE 128u
static uint8_t s_dma_rx_buf[BSP_USART1_DMA_RX_BUF_SIZE];
static uint8_t s_use_dma[BSP_USART_INSTANCE_MAX] = {0};  /* 标记哪些串口用 DMA 接收 */

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
    s_rx_ptr[instance] = pData;   /* 记录缓冲指针，ORE 恢复时用 */
    return HAL_UART_Receive_IT(uart_handles[instance], pData, Size);
}

HAL_StatusTypeDef BSP_USART_ReceiveToIdle_DMA(BSP_USART_Instance_t instance)
{
    if (instance >= BSP_USART_INSTANCE_MAX) {
        return HAL_ERROR;
    }
    s_use_dma[instance] = 1U;   /* 标记 DMA 模式，错误回调据此选择重启方式 */
    return HAL_UARTEx_ReceiveToIdle_DMA(uart_handles[instance], s_dma_rx_buf, BSP_USART1_DMA_RX_BUF_SIZE);
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

// -------------------- DMA + 空闲事件回调 --------------------

/**
  * @brief  DMA 接收 + 空闲事件回调（由 HAL 库调用）
  * @note   发送方暂停（IDLE）或 DMA 缓冲满/半满时触发。
  *         此处把 DMA 缓冲中的字节逐个投递到上层回调（入环形缓冲），
  *         然后立即重启 DMA 接收下一批。全程不依赖 ISR 实时逐字节响应。
  * @param  huart: UART 句柄指针
  * @param  Size: 本次接收到的字节数
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    BSP_USART_Instance_t inst = GetInstanceFromHandle(huart);
    if (inst >= BSP_USART_INSTANCE_MAX) return;

    /* DMA 半传事件（缓冲收到一半，Size=64）：必须直接返回，不投递也不重启。
     * HAL 在 TOIDLE 模式下半满也会触发本回调；若此时重启 DMA，后半段传输被破坏，
     * 且前半批字节会在随后的 IDLE/TC 回调中被重复投递。 */
    if (huart->RxEventType == HAL_UART_RXEVENT_HT)
    {
        return;
    }

    /* IDLE（总线空闲）或 TC（128 字节缓冲满）：把接收到的字节逐个投递到上层
     * 回调（入环形缓冲），上层在 Disp_Poll 中解析。Size=本次 DMA 传输总字节数 */
    if (rx_callbacks[inst] != NULL && Size > 0)
    {
        uint8_t *p = huart->pRxBuffPtr;
        for (uint16_t i = 0; i < Size; i++)
        {
            rx_callbacks[inst](inst, p[i]);
        }
    }

    /* 重启 DMA 接收，等待下一批数据 */
    HAL_UARTEx_ReceiveToIdle_DMA(huart, s_dma_rx_buf, BSP_USART1_DMA_RX_BUF_SIZE);
}

/**
  * @brief  UART 错误回调：处理 ORE/NE/FE/PE 等错误，自动恢复接收
  * @note   IT 模式的串口（USART2/3/4）清错误后重启单字节 IT 接收；
  *         DMA 模式的串口（USART1）清错误后重启 DMA 接收。
  *         根因是优先级抢占导致 ISR 被阻塞；DMA 硬件搬运根治此问题。
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    BSP_USART_Instance_t inst = GetInstanceFromHandle(huart);
    if (inst >= BSP_USART_INSTANCE_MAX) return;

    /* 清所有错误标志 */
    __HAL_UART_CLEAR_OREFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    /* 重置 HAL 状态机，否则后续 Receive 会因 BUSY 返回错误 */
    huart->gState  = HAL_UART_STATE_READY;
    huart->RxState = HAL_UART_STATE_READY;

    if (s_use_dma[inst])
    {
        /* DMA 模式（USART1）：重启 DMA + 空闲接收 */
        HAL_UARTEx_ReceiveToIdle_DMA(huart, s_dma_rx_buf, BSP_USART1_DMA_RX_BUF_SIZE);
    }
    else if (s_rx_ptr[inst] != NULL)
    {
        /* IT 模式（USART2/3/4）：重启单字节 IT 接收 */
        HAL_UART_Receive_IT(huart, s_rx_ptr[inst], 1);
    }
}