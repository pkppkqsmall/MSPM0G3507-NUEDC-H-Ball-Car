#include "UART.h"
#include "clock.h"
#include "motor_pwm.h"

/*
 * VOFA+ FireWater 采样帧格式：
 *   name:ch0,ch1,...\n
 * 注意：VOFA 依靠换行符判断一帧结束，所以 '\n' 不能省略。
 */
#define UART_VOFA_SPEED_PREFIX      "speed:"
#define UART_VOFA_FRAME_END         "\n"

/*
 * 普通上位机显示用：发送左右轮速度到 VOFA+。
 * 实际发送格式：speed:left_rpm,right_rpm\n
 * 例：speed:23.50,22.80
 */
void UART_BluetoothSendVofaWheelSpeed(float left_speed, float right_speed)
{
    UART_Printf(UART_VOFA_SPEED_PREFIX "%.2f,%.2f" UART_VOFA_FRAME_END,
                left_speed,
                right_speed);
}

/* 兼容旧接口名：旧代码如果还调用 UART_BluetoothSendWheelSpeed，也会走 VOFA 格式。 */
void UART_BluetoothSendWheelSpeed(float left_speed, float right_speed)
{
    UART_BluetoothSendVofaWheelSpeed(left_speed, right_speed);
}

/*
 * 按 llm-pid-tuner 要求的 CSV 格式上报左轮速度环数据。
 * 格式: timestamp_ms,setpoint,input,pwm,error,p,i,d
 * 以左轮为调参目标，调好后自动同步到右轮。
 */
void UART_BluetoothSendAiTuneSpeedCsv(void)
{
    float left_rpm;
    float left_target;
    float left_duty;
    float kp;
    float ki;
    float error;

    left_target = MotorSpeedLoop_GetLeftWheelTargetRPM();
    left_rpm = MotorSpeed_GetLeftWheelRPM();
    left_duty = MotorSpeedLoop_GetLeftWheelDutyPercent();
    MotorSpeedLoop_GetLeftWheelPI(&kp, &ki);
    error = left_target - left_rpm;

    UART_Printf("%lu,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,0.0000\r\n",
                (unsigned long) tick_ms,
                left_target,
                left_rpm,
                left_duty,
                error,
                kp,
                ki);
}

/* 兼容旧接口名：AI 调参仍然发送 llm-pid-tuner 需要的 CSV 数据帧。 */
void UART_LlmTunerSendCsv(void)
{
    UART_BluetoothSendAiTuneSpeedCsv();
}
