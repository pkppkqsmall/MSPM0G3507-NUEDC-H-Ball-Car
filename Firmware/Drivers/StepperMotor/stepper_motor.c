#include <stdint.h>

#include "ti_msp_dl_config.h"
#include "stepper_motor.h"

#define STEPPER_DIRECTION_SETUP_US      (20U)
#define STEPPER_CYCLES_PER_US           (CPUCLK_FREQ / 1000000U)

static volatile uint32_t g_move_target_pulses;
static volatile uint32_t g_move_pulse_count;
static volatile int32_t g_position_pulses;
static volatile int32_t g_position_target_pulses;
static volatile StepperMotorDirection g_move_direction;
static volatile uint8_t g_stepper_busy;
static volatile uint8_t g_stop_requested;
static volatile uint8_t g_position_tracking_enabled;

static void StepperMotor_DelayUs(uint32_t delay_us)
{
    /* 当前只用于 DIR 建立时间，20 us 的短延时不会影响主循环调度。 */
    delay_cycles(STEPPER_CYCLES_PER_US * delay_us);
}

static void StepperMotor_StopPulseOutput(void)
{
    DL_TimerG_stopCounter(PWM_STEPPER_INST);
    DL_TimerG_clearInterruptStatus(
        PWM_STEPPER_INST, DL_TIMERG_INTERRUPT_CC1_UP_EVENT);
}

static uint8_t StepperMotor_StartPulseOutput(
    StepperMotorDirection direction,
    uint32_t pulse_count)
{
    if ((pulse_count == 0U) ||
        (g_stepper_busy != 0U) ||
        ((direction != STEPPER_DIRECTION_POSITIVE) &&
         (direction != STEPPER_DIRECTION_NEGATIVE))) {
        return 0U;
    }

    NVIC_DisableIRQ(PWM_STEPPER_INST_INT_IRQN);
    StepperMotor_StopPulseOutput();
    NVIC_ClearPendingIRQ(PWM_STEPPER_INST_INT_IRQN);

    if (direction == STEPPER_DIRECTION_POSITIVE) {
        DL_GPIO_clearPins(GPIO_STEPPER_PORT, GPIO_STEPPER_DIR_PIN);
    } else {
        DL_GPIO_setPins(GPIO_STEPPER_PORT, GPIO_STEPPER_DIR_PIN);
    }
    StepperMotor_DelayUs(STEPPER_DIRECTION_SETUP_US);

    g_move_direction = direction;
    g_move_target_pulses = pulse_count;
    g_move_pulse_count = 0U;
    g_stepper_busy = 1U;
    g_stop_requested = 0U;

    DL_TimerG_setTimerCount(PWM_STEPPER_INST, 0U);
    NVIC_EnableIRQ(PWM_STEPPER_INST_INT_IRQN);
    DL_TimerG_startCounter(PWM_STEPPER_INST);
    return 1U;
}

void StepperMotor_Init(void)
{
    /*
     * STEP 由 TIMG12 CCP1 输出，SysConfig 将定时器配置为停止状态。
     * DIR 再次清零，软件位置以本次上电位置作为 0 度。
     */
    NVIC_DisableIRQ(PWM_STEPPER_INST_INT_IRQN);
    StepperMotor_StopPulseOutput();
    NVIC_ClearPendingIRQ(PWM_STEPPER_INST_INT_IRQN);

    DL_GPIO_clearPins(GPIO_STEPPER_PORT, GPIO_STEPPER_DIR_PIN);
    g_move_target_pulses = 0U;
    g_move_pulse_count = 0U;
    g_position_pulses = 0;
    g_position_target_pulses = 0;
    g_move_direction = STEPPER_DIRECTION_POSITIVE;
    g_stepper_busy = 0U;
    g_stop_requested = 0U;
    g_position_tracking_enabled = 0U;
}

uint8_t StepperMotor_StartMove(StepperMotorDirection direction,
                               uint32_t pulse_count)
{
    if ((pulse_count == 0U) ||
        (g_stepper_busy != 0U) ||
        ((direction != STEPPER_DIRECTION_POSITIVE) &&
         (direction != STEPPER_DIRECTION_NEGATIVE))) {
        return 0U;
    }

    g_position_tracking_enabled = 0U;
    g_position_target_pulses = g_position_pulses;
    return StepperMotor_StartPulseOutput(direction, pulse_count);
}

uint8_t StepperMotor_StartRevolutions(StepperMotorDirection direction,
                                      uint32_t revolutions)
{
    if ((revolutions == 0U) ||
        (revolutions > (UINT32_MAX / STEPPER_PULSES_PER_REVOLUTION))) {
        return 0U;
    }

    return StepperMotor_StartMove(
        direction, revolutions * STEPPER_PULSES_PER_REVOLUTION);
}

void StepperMotor_RequestStop(void)
{
    if (g_stepper_busy != 0U) {
        /*
         * 不在主循环直接停定时器，避免 STEP 恰好停在高电平。
         * CC1_UP 中断发生在高脉冲结束处，届时再关断最安全。
         */
        g_stop_requested = 1U;
    }
}

uint8_t StepperMotor_SetCurrentPositionZero(void)
{
    if ((g_stepper_busy != 0U) ||
        (g_position_tracking_enabled != 0U)) {
        return 0U;
    }

    NVIC_DisableIRQ(PWM_STEPPER_INST_INT_IRQN);
    g_position_pulses = 0;
    g_position_target_pulses = 0;
    NVIC_ClearPendingIRQ(PWM_STEPPER_INST_INT_IRQN);
    return 1U;
}

uint8_t StepperMotor_EnablePositionTracking(uint8_t enabled)
{
    if (enabled == 0U) {
        g_position_tracking_enabled = 0U;
        g_position_target_pulses = g_position_pulses;
        StepperMotor_RequestStop();
        return 1U;
    }

    if (g_stepper_busy != 0U) {
        return 0U;
    }

    g_position_target_pulses = g_position_pulses;
    g_position_tracking_enabled = 1U;
    return 1U;
}

uint8_t StepperMotor_SetTargetPositionPulses(int32_t target_pulses)
{
    if (g_position_tracking_enabled == 0U) {
        return 0U;
    }

    g_position_target_pulses = target_pulses;
    return 1U;
}

void StepperMotor_Task(void)
{
    int32_t current_position;
    int32_t target_position;

    if (g_position_tracking_enabled == 0U) {
        return;
    }

    current_position = g_position_pulses;
    target_position = g_position_target_pulses;

    if (g_stepper_busy != 0U) {
        if ((current_position == target_position) ||
            ((g_move_direction == STEPPER_DIRECTION_POSITIVE) &&
             (target_position < current_position)) ||
            ((g_move_direction == STEPPER_DIRECTION_NEGATIVE) &&
             (target_position > current_position))) {
            StepperMotor_RequestStop();
        }
        return;
    }

    if (current_position < target_position) {
        (void) StepperMotor_StartPulseOutput(
            STEPPER_DIRECTION_POSITIVE, UINT32_MAX);
    } else if (current_position > target_position) {
        (void) StepperMotor_StartPulseOutput(
            STEPPER_DIRECTION_NEGATIVE, UINT32_MAX);
    }
}

uint8_t StepperMotor_IsPositionTrackingEnabled(void)
{
    return g_position_tracking_enabled;
}

int32_t StepperMotor_GetTargetPositionPulses(void)
{
    return g_position_target_pulses;
}

uint8_t StepperMotor_IsBusy(void)
{
    return g_stepper_busy;
}

uint32_t StepperMotor_GetCurrentMovePulseCount(void)
{
    return g_move_pulse_count;
}

int32_t StepperMotor_GetPositionPulses(void)
{
    return g_position_pulses;
}

float StepperMotor_GetCommandAngleDeg(void)
{
    return ((float) g_position_pulses * 360.0f) /
           (float) STEPPER_PULSES_PER_REVOLUTION;
}

int32_t StepperMotor_GetCompletedRevolutions(void)
{
    return g_position_pulses / (int32_t) STEPPER_PULSES_PER_REVOLUTION;
}

float StepperMotor_GetWithinRevolutionAngleDeg(void)
{
    int32_t remaining_pulses =
        g_position_pulses % (int32_t) STEPPER_PULSES_PER_REVOLUTION;

    return ((float) remaining_pulses * 360.0f) /
           (float) STEPPER_PULSES_PER_REVOLUTION;
}

uint32_t StepperMotor_GetPulsesPerRevolution(void)
{
    return STEPPER_PULSES_PER_REVOLUTION;
}

void PWM_STEPPER_INST_IRQHandler(void)
{
    uint8_t position_target_reached;

    switch (DL_TimerG_getPendingInterrupt(PWM_STEPPER_INST)) {
        case DL_TIMERG_IIDX_CC1_UP:
            if (g_stepper_busy == 0U) {
                StepperMotor_StopPulseOutput();
                break;
            }

            g_move_pulse_count++;
            if (g_move_direction == STEPPER_DIRECTION_POSITIVE) {
                g_position_pulses++;
            } else {
                g_position_pulses--;
            }

            position_target_reached = 0U;
            if (g_position_tracking_enabled != 0U) {
                if (((g_move_direction ==
                      STEPPER_DIRECTION_POSITIVE) &&
                     (g_position_pulses >=
                      g_position_target_pulses)) ||
                    ((g_move_direction ==
                      STEPPER_DIRECTION_NEGATIVE) &&
                     (g_position_pulses <=
                      g_position_target_pulses))) {
                    position_target_reached = 1U;
                }
            }

            if ((g_stop_requested != 0U) ||
                (position_target_reached != 0U) ||
                ((g_position_tracking_enabled == 0U) &&
                 (g_move_pulse_count >= g_move_target_pulses))) {
                /*
                 * CC1_UP 发生在 10 us 高脉冲结束处，此时停止定时器，
                 * 可以保证脉冲数准确且 STEP 最终回到低电平。
                 */
                StepperMotor_StopPulseOutput();
                g_stepper_busy = 0U;
                g_stop_requested = 0U;
            }
            break;

        default:
            break;
    }
}
