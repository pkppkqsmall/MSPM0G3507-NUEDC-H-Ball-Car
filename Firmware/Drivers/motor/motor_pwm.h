#ifndef MOTOR_PWM_H_
#define MOTOR_PWM_H_

#include <stdint.h>
#include "ti_msp_dl_config.h"

/*
 * 速度采样周期，单位为 ms。
 * 如果主循环里用于测速的 (tick_ms - last_speed_tick) 周期发生变化，
 * 这里也要同步修改，保证转速换算结果正确。
 */
extern volatile uint32_t g_motor_speed_sample_time_ms;

/*
 * 电机死区补偿占空比，单位为百分比。
 * 当目标占空比大于 0 但又低于这个值时，会自动抬升到这个门槛，
 * 这样可以避开电机静摩擦导致的“有输出但不转”。
 */
extern volatile float g_motor_deadzone_duty_percent;

/*
 * 电机启动脉冲占空比，单位为百分比。
 * 电机刚启动时先用这个较大的占空比拉起，随后再回到维持占空比。
 */
extern volatile float g_motor_start_boost_duty_percent;

/*
 * 电机启动脉冲持续时间，单位为 ms。
 * 设为 0 时表示关闭启动脉冲功能。
 */
extern volatile uint32_t g_motor_start_boost_time_ms;

/*
 * 设置指定 PWM 通道的占空比。
 * channel 对应 PWM 通道号，duty_percent 为占空比百分比。
 */
void Set_PWM_DutyCycle(uint8_t channel, float duty_percent);

/*
 * 初始化电机 PWM 默认参数。
 * 包括死区、启动脉冲和上电关断，避免电机参数散落在按键模块里。
 */
void MotorPWM_InitDefaults(void);

/*
 * 强制关闭 PWM_0 的四路电机输出通道。
 * 这个函数主要用于上电或复位后的安全处理，确保左右轮的输入脚都先被拉成低电平，
 * 避免驱动模块在初始化瞬间因为引脚默认状态而误动作。
 */
void MotorPWM_StopAllChannels(void);

/*
 * 直接按“小车前进方向”为正输出左右轮占空比。
 * 该接口会绕过速度 PI，只用于已关闭速度环后的短时平滑停车；正常行驶
 * 仍应使用 MotorSpeedLoop_Set*WheelTargetRPM()。
 */
void MotorPWM_SetLeftWheelSignedDutyPercent(float signed_duty_percent);
void MotorPWM_SetRightWheelSignedDutyPercent(float signed_duty_percent);

/*
 * 将 AT8236 两路 H 桥的输入同时置高，进入低侧慢衰减刹车。
 * 仅用于短时主动制动，制动结束后应调用 MotorPWM_StopAllChannels()。
 */
void MotorPWM_BrakeAllChannels(void);

/*
 * 设置死区补偿占空比。
 * 一般可以填测试得到的“刚开始起转”的占空比。
 */
void MotorPWM_SetDeadzoneDutyPercent(float duty_percent);

/*
 * 读取当前死区补偿占空比。
 */
float MotorPWM_GetDeadzoneDutyPercent(void);

/*
 * 设置启动脉冲占空比。
 * 建议略高于死区占空比，保证电机更容易拉起来。
 */
void MotorPWM_SetStartBoostDutyPercent(float duty_percent);

/*
 * 读取当前启动脉冲占空比。
 */
float MotorPWM_GetStartBoostDutyPercent(void);

/*
 * 设置启动脉冲持续时间，单位为 ms。
 * 设为 0 时表示不使用启动脉冲。
 */
void MotorPWM_SetStartBoostTimeMs(uint32_t boost_time_ms);

/*
 * 读取当前启动脉冲持续时间，单位为 ms。
 */
uint32_t MotorPWM_GetStartBoostTimeMs(void);

/*
 * 对目标占空比做死区补偿。
 * 传入 0 会返回 0，传入非零且低于死区门槛的值会被自动抬高。
 */
float MotorPWM_ApplyDeadzoneCompensation(float duty_percent);

/*
 * 根据维持占空比计算建议使用的启动脉冲占空比。
 * 返回值会同时考虑死区补偿和启动脉冲门槛。
 */
float MotorPWM_GetStartupDutyPercent(float hold_duty_percent);

/*
 * 设置速度采样周期，单位为 ms。
 * 一般应与主循环中测速任务的执行周期保持一致。
 */
void MotorSpeed_SetSampleTimeMs(uint32_t sample_time_ms);

/*
 * 读取当前配置的速度采样周期，单位为 ms。
 */
uint32_t MotorSpeed_GetSampleTimeMs(void);

/*
 * 重置测速模块的历史计数。
 * 建议在开始测速前调用一次，避免第一次差分出现突变。
 */
void MotorSpeed_Reset(void);

/*
 * 根据本次经过的时间和编码器计数差值更新左右轮速度。
 * elapsed_ms 一般传入 tick_ms - last_speed_tick。
 * 返回 1 表示更新成功，返回 0 表示本次时间参数无效。
 */
uint8_t MotorSpeed_Update(uint32_t elapsed_ms);

/*
 * 获取当前使用的轮胎直径，单位为 mm。
 */
float MotorSpeed_GetWheelDiameterMm(void);

/*
 * 获取车轮输出轴每转一圈对应的编码器计数。
 * 当前程序按 A 相双边沿计数，实测约为 728。
 */
float MotorSpeed_GetWheelCountsPerRevolution(void);

/*
 * 获取左轮当前转速，单位为 RPM。
 */
float MotorSpeed_GetLeftWheelRPM(void);

/*
 * 获取右轮当前转速，单位为 RPM。
 */
float MotorSpeed_GetRightWheelRPM(void);

/*
 * 同时获取左右轮当前转速，单位为 RPM。
 * 传入的指针可以为 NULL，不需要的那一侧可以不取。
 */
void MotorSpeed_GetWheelRPM(float *left_rpm, float *right_rpm);

/*
 * 获取左轮当前线速度，单位为 mm/s。
 */
float MotorSpeed_GetLeftWheelLinearSpeedMmPerSec(void);

/*
 * 获取右轮当前线速度，单位为 mm/s。
 */
float MotorSpeed_GetRightWheelLinearSpeedMmPerSec(void);

/*
 * 同时获取左右轮当前线速度，单位为 mm/s。
 * 传入的指针可以为 NULL，不需要的那一侧可以不取。
 */
void MotorSpeed_GetWheelLinearSpeedMmPerSec(float *left_mm_per_sec, float *right_mm_per_sec);

/*
 * 获取左右编码器当前累计计数。
 * 这里返回的是自上电或复位以来的原始累计值，便于现场观察“一圈大约多少计数”。
 */
int32_t MotorSpeed_GetLeftEncoderTotalCount(void);
int32_t MotorSpeed_GetRightEncoderTotalCount(void);

/*
 * 获取按“小车前进方向”为正修正后的编码器累计计数。
 * 右轮原始编码器方向与左轮相反时，这组接口会做方向修正，更适合 OLED 现场观察。
 */
int32_t MotorSpeed_GetLeftEncoderForwardCount(void);
int32_t MotorSpeed_GetRightEncoderForwardCount(void);

/*
 * 设置左轮速度环的目标转速，单位为 RPM。
 * 传入正值表示正转，传入负值表示反转，传入 0 表示停车。
 */
void MotorSpeedLoop_SetLeftWheelTargetRPM(float target_rpm);
void MotorSpeedLoop_SetRightWheelTargetRPM(float target_rpm);

/*
 * 读取左轮速度环当前目标转速，单位为 RPM。
 */
float MotorSpeedLoop_GetLeftWheelTargetRPM(void);
float MotorSpeedLoop_GetRightWheelTargetRPM(void);

/*
 * 设置左轮速度环 PI 参数。
 * 这里先使用 PI 控制，kd 参数暂时不参与速度环计算。
 */
void MotorSpeedLoop_SetLeftWheelPI(float kp, float ki);
void MotorSpeedLoop_SetRightWheelPI(float kp, float ki);

/*
 * 读取左轮速度环当前使用的 PI 参数。
 * 如果某个指针传入 NULL，则表示不读取该项。
 */
void MotorSpeedLoop_GetLeftWheelPI(float *kp, float *ki);
void MotorSpeedLoop_GetRightWheelPI(float *kp, float *ki);

/*
 * 使能或关闭左轮速度环。
 * enable 传入 1 表示开启闭环，传入 0 表示关闭闭环并停车。
 */
void MotorSpeedLoop_EnableLeftWheel(uint8_t enable);
void MotorSpeedLoop_EnableRightWheel(uint8_t enable);

/*
 * 读取左轮速度环当前是否处于使能状态。
 */
uint8_t MotorSpeedLoop_IsLeftWheelEnabled(void);
uint8_t MotorSpeedLoop_IsRightWheelEnabled(void);

/*
 * 重置左轮速度环的积分项、启动脉冲状态和当前输出。
 * 一般在切换启停状态或大幅修改目标速度前调用。
 */
void MotorSpeedLoop_ResetLeftWheelController(void);
void MotorSpeedLoop_ResetRightWheelController(void);

/*
 * 更新一次左轮速度环。
 * 这个函数应放在测速更新之后调用，这样它拿到的是最新转速。
 * elapsed_ms 为本次控制周期实际经过的时间，通常直接传入测速周期。
 */
uint8_t MotorSpeedLoop_UpdateLeftWheel(uint32_t elapsed_ms);
uint8_t MotorSpeedLoop_UpdateRightWheel(uint32_t elapsed_ms);

/*
 * 读取左轮速度环当前输出到电机上的占空比。
 * 正值表示正转占空比，负值表示反转占空比。
 */
float MotorSpeedLoop_GetLeftWheelDutyPercent(void);
float MotorSpeedLoop_GetRightWheelDutyPercent(void);

#endif
