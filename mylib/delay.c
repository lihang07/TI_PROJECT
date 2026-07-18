#include "ti_msp_dl_config.h"

#include "mylib/delay.h"
#include "core_cm0plus.h"

volatile uint32_t systick_ms = 0;
#define CPU_FREQ_MHZ 32


static void delay_cycle(uint32_t cycles)
{
    while(cycles--)
    {
        __NOP();//防止优化
    }

}

void delay_ms(uint32_t ms)
{
    while(ms--)
    {
        delay_us(1000);
    }
}

void delay_us(uint32_t us)
{
    //1us = 32cycle(32Mhz)
    //while = 3~4 cycle
    //一个wile循环消耗4cycle,做8次循环
    uint32_t ticks = us * (CPU_FREQ_MHZ *1);
    delay_cycle(ticks);
   

    
}