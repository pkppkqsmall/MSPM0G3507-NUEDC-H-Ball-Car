#ifndef STEPPER_MOTOR_H_
#define STEPPER_MOTOR_H_

#include <stdint.h>

/*
 * 电机本体为 1.8 度，即每圈 200 个整步。
 * STEPPER_MICROSTEP_DIVISOR 必须与 ZDT 上位机里的细分设置一致：
 * 例如细分设为 16 时，把这里改成 16，软件角度才与指令角度一致。
 */
#define STEPPER_FULL_STEPS_PER_REVOLUTION    (200UL)
#define STEPPER_MICROSTEP_DIVISOR            (16UL)
#define STEPPER_PULSES_PER_REVOLUTION        \
    (STEPPER_FULL_STEPS_PER_REVOLUTION * STEPPER_MICROSTEP_DIVISOR)

typedef enum {
    STEPPER_DIRECTION_POSITIVE = 0,
    STEPPER_DIRECTION_NEGATIVE
} StepperMotorDirection;

/* 初始化 STEP 定时器和 DIR，引脚保持低电平，不会自动启动电机。 */
void StepperMotor_Init(void);

/*
 * 非阻塞地启动一次指定脉冲数的运动。
 * 返回 1 表示成功启动，返回 0 表示参数无效或上一次运动尚未完成。
 */
uint8_t StepperMotor_StartMove(StepperMotorDirection direction,
                               uint32_t pulse_count);

/*
 * 非阻塞地启动整数圈运动，函数会根据当前细分自动换算目标脉冲数。
 * 返回 0 表示圈数为 0、换算溢出或电机仍在执行上一次运动。
 */
uint8_t StepperMotor_StartRevolutions(StepperMotorDirection direction,
                                      uint32_t revolutions);

/* 请求在当前 STEP 高脉冲结束后停止，软件位置保留已输出的脉冲数。 */
void StepperMotor_RequestStop(void);

/*
 * 连续目标位置模式供滚球控制使用。目标单位是相对软件零点的 STEP 脉冲，
 * StepperMotor_Task() 必须在主循环中持续调用。
 */
uint8_t StepperMotor_SetCurrentPositionZero(void);
uint8_t StepperMotor_EnablePositionTracking(uint8_t enabled);
uint8_t StepperMotor_SetTargetPositionPulses(int32_t target_pulses);
void StepperMotor_Task(void);
uint8_t StepperMotor_IsPositionTrackingEnabled(void);
int32_t StepperMotor_GetTargetPositionPulses(void);

uint8_t StepperMotor_IsBusy(void);
uint32_t StepperMotor_GetCurrentMovePulseCount(void);
int32_t StepperMotor_GetPositionPulses(void);

/*
 * 由累计指令脉冲换算软件位置角度，不是 ZDT 编码器反馈的真实机械角度。
 * 上电位置被当作 0 度，正方向增加、反方向减小。
 */
float StepperMotor_GetCommandAngleDeg(void);
int32_t StepperMotor_GetCompletedRevolutions(void);
float StepperMotor_GetWithinRevolutionAngleDeg(void);
uint32_t StepperMotor_GetPulsesPerRevolution(void);

#endif /* STEPPER_MOTOR_H_ */
