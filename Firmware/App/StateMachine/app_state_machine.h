#ifndef APP_STATE_MACHINE_H_
#define APP_STATE_MACHINE_H_

#include <stdint.h>

/*
 * 状态机模块初始化函数。
 * 这里只负责把状态机上下文清零，并把初始状态设为初始化状态。
 * 真正的硬件初始化仍然在状态机运行到 INIT 状态时执行。
 */
void AppStateMachine_Init(void);

/*
 * 状态机单步运行函数。
 * 主循环每次调用一次，状态机会按既定顺序执行一个状态并切到下一个状态。
 */
void AppStateMachine_RunStep(void);

#endif /* APP_STATE_MACHINE_H_ */
