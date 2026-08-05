#ifndef CLOCK_H_
#define CLOCK_H_

extern volatile unsigned long tick_ms;

int mspm0_delay_ms(unsigned long num_ms);
int mspm0_get_clock_ms(unsigned long *count);
void SysTick_Init(void);

#endif  /* CLOCK_H_ */
