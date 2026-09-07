#include "main.h"


static uint32_t fac_us;


/*
 * 初始化
 */
void delay_init(void)
{
    fac_us = HAL_RCC_GetHCLKFreq() / 1000000;
}



/*
 * us延时
 *
 * 不依赖中断
 * 可以在中断中使用
 */
void delay_us(uint32_t us)
{
    uint32_t start;
    uint32_t ticks;


    ticks = us * fac_us;


    start = SysTick->VAL;


    while((start - SysTick->VAL) < ticks)
    {
        if(SysTick->VAL > start)
        {
            /*
             * SysTick发生了一次重装
             */
            start += SysTick->LOAD + 1;
        }
    }
}



// #include "main.h"
// #include "delay.h"


// static uint32_t g_fac_us = 0;


// /**
//  * @brief 初始化微秒延时
//  */
// void delay_init(void)
// {
//     /*
//      *  SysTick时钟 = HCLK
//      */
//     g_fac_us = HAL_RCC_GetHCLKFreq() / 1000000U;
// }



// /**
//  * @brief us延时
//  *
//  * @param nus 微秒
//  *
//  * 可以在中断中调用
//  */
// void delay_us(uint32_t nus)
// {
//     uint32_t start;
//     uint32_t current;
//     uint32_t reload;


//     reload = SysTick->LOAD + 1U;


//     /*
//      * 当前计数值
//      */
//     start = SysTick->VAL;


//     /*
//      * 需要计数次数
//      */
//     uint32_t ticks = nus * g_fac_us;


//     uint32_t elapsed = 0;


//     while(elapsed < ticks)
//     {
//         current = SysTick->VAL;


//         if(current <= start)
//         {
//             elapsed += start - current;
//         }
//         else
//         {
//             /*
//              * SysTick溢出
//              */
//             elapsed += start + reload - current;
//         }


//         start = current;
//     }
// }




// void delay_ms(uint32_t nms)
// {
//     while(nms--)
//     {
//         delay_us(1000);
//     }
// }



// /*
//  * 重写HAL_Delay
//  */
// void HAL_Delay(uint32_t Delay)
// {
//     delay_ms(Delay);
// }