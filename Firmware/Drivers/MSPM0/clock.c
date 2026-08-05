#include "ti_msp_dl_config.h"
#include "clock.h"

volatile unsigned long tick_ms;

int mspm0_delay_ms(unsigned long num_ms)
{
    /*
     * 起点时刻放在函数内部，避免不同上下文同时调用 delay 时互相覆盖。
     * 使用无符号作差，tick_ms 回绕时也能保持正确的延时判断。
     */
    unsigned long start = tick_ms;

    while ((tick_ms - start) < num_ms) {
        ;
    }

    return 0;
}

int mspm0_get_clock_ms(unsigned long *count)
{
    if (!count)
        return 1;
    count[0] = tick_ms;
    return 0;
}

void SysTick_Init(void)
{
    DL_SYSTICK_config(CPUCLK_FREQ/1000);
    NVIC_SetPriority(SysTick_IRQn, 0);
}
