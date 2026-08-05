#include <stdint.h>
#include <stddef.h>

#include "app_speed_task.h"
#include "clock.h"
#include "motor_pwm.h"
#include "UART.h"

void AppSpeedTask_Init(AppSpeedTask *task)
{
    if (task == NULL) {
        return;
    }

    task->last_speed_tick = 0U;
}

void AppSpeedTask_Update(AppSpeedTask *task)
{
    uint32_t speed_elapsed_ms;

    if ((task == NULL) ||
        ((tick_ms - task->last_speed_tick) < g_motor_speed_sample_time_ms)) {
        return;
    }

    /*
     * 速度环按固定采样周期更新。
     * 普通模式输出 VOFA 曲线数据；AI 调参模式输出 CSV，供上位机分析 PID 表现。
     */
    speed_elapsed_ms = (uint32_t) (tick_ms - task->last_speed_tick);
    task->last_speed_tick = tick_ms;

    if (MotorSpeed_Update(speed_elapsed_ms) == 0U) {
        return;
    }

    (void) MotorSpeedLoop_UpdateLeftWheel(speed_elapsed_ms);
    (void) MotorSpeedLoop_UpdateRightWheel(speed_elapsed_ms);

    if (UART_LlmTunerIsEnabled() != 0U) {
        UART_BluetoothSendAiTuneSpeedCsv();
    } else {
        UART_BluetoothSendVofaWheelSpeed(MotorSpeed_GetLeftWheelRPM(),
                                         MotorSpeed_GetRightWheelRPM());
    }
}
