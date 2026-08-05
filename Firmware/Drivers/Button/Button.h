#ifndef BUTTON_H_
#define BUTTON_H_

#include <stdint.h>
#include "ti_msp_dl_config.h"

/*
 * 初始化按键与电机控制模块。
 * 这里只做按键去抖状态、电机默认目标转速和速度环启停状态的初始化。
 */
void Button_InitMotorControl(void);

/*
 * 周期扫描 B21、SW1、SW2。
 * 扫描到稳定按下后，会把事件锁存起来，等待状态机读取并清除。
 */
void Button_Task(void);

/*
 * 读取并清除一次性按键事件。
 * 返回 1 表示自上次读取后检测到了稳定按下事件。
 */
uint8_t Button_GetAndClearB21PressedEvent(void);
uint8_t Button_GetAndClearSW1PressedEvent(void);
uint8_t Button_GetAndClearSW2PressedEvent(void);

/*
 * 设置/读取默认巡线速度目标，单位为 RPM。
 * 状态机和蓝牙命令都会共用这一组目标值。
 */
void Button_SetSpeedTargets(float left_target_rpm, float right_target_rpm);
void Button_GetSpeedTargets(float *left_target_rpm, float *right_target_rpm);

/*
 * 启动或停止左右轮速度环。
 * 停止时保留目标值，便于下次直接恢复运行。
 */
void Button_StartMotorControl(void);
void Button_StopMotorControl(void);

/*
 * 查询当前速度环是否处于运行状态。
 * 用于状态机同步 SW2/蓝牙 START/STOP 触发后的运行状态。
 */
uint8_t Button_IsMotorControlRunning(void);

#endif
