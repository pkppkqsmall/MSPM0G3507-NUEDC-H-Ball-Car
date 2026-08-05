#include "Button.h"
#include "motor_pwm.h"
#include "clock.h"

/*
 * 默认巡线目标速度，单位为 RPM。
 * 蓝牙下发 TARGET 后，会覆盖这里保存的默认值。
 */
#define BUTTON_DEFAULT_LEFT_TARGET_RPM   (50.0f)
#define BUTTON_DEFAULT_RIGHT_TARGET_RPM  (50.0f)
#define BUTTON_DEBOUNCE_TIME_MS          (50U)

/* 三个物理按键的去抖状态。 */
static uint8_t g_b21_last_state = 1U;
static uint32_t g_b21_last_time = 0U;
static uint8_t g_sw1_last_state = 1U;
static uint32_t g_sw1_last_time = 0U;
static uint8_t g_sw2_last_state = 1U;
static uint32_t g_sw2_last_time = 0U;

/* 状态机读取前，按键事件会先锁存在这里。 */
static uint8_t g_b21_pressed_event = 0U;
static uint8_t g_sw1_pressed_event = 0U;
static uint8_t g_sw2_pressed_event = 0U;

/* 保存当前默认目标速度与运行状态。 */
static float g_button_left_target_rpm = BUTTON_DEFAULT_LEFT_TARGET_RPM;
static float g_button_right_target_rpm = BUTTON_DEFAULT_RIGHT_TARGET_RPM;
static uint8_t g_speed_loop_running = 0U;

/*
 * 对单个按键做下降沿检测和去抖。
 * 返回 1 表示本次检测到了一次稳定按下。
 */
static uint8_t Button_CheckPressedEvent(GPIO_Regs *port,
                                        uint32_t pin,
                                        uint8_t *last_state,
                                        uint32_t *last_time)
{
    uint8_t current_state;

    current_state = (DL_GPIO_readPins(port, pin) == 0U) ? 0U : 1U;

    if ((current_state == 0U) && (*last_state == 1U)) {
        if ((tick_ms - (*last_time)) > BUTTON_DEBOUNCE_TIME_MS) {
            *last_time = tick_ms;
            *last_state = current_state;
            return 1U;
        }
    }

    *last_state = current_state;
    return 0U;
}

/*
 * 按统一入口控制左右轮速度环启停。
 * 这样无论是状态机还是蓝牙命令，最终都共用同一套启停逻辑。
 */
static void Button_ApplySpeedLoopState(uint8_t enable)
{
    g_speed_loop_running = (enable != 0U) ? 1U : 0U;

    if (g_speed_loop_running != 0U) {
        MotorSpeedLoop_SetLeftWheelTargetRPM(g_button_left_target_rpm);
        MotorSpeedLoop_SetRightWheelTargetRPM(g_button_right_target_rpm);
        MotorSpeedLoop_EnableLeftWheel(1U);
        MotorSpeedLoop_EnableRightWheel(1U);
    } else {
        MotorSpeedLoop_SetLeftWheelTargetRPM(0.0f);
        MotorSpeedLoop_SetRightWheelTargetRPM(0.0f);
        MotorSpeedLoop_EnableLeftWheel(0U);
        MotorSpeedLoop_EnableRightWheel(0U);
    }
}

void Button_InitMotorControl(void)
{
    /*
     * 这里只初始化按键事件、默认目标速度和速度环运行标志。
     * PWM 死区、启动脉冲和上电关断由 MotorPWM_InitDefaults() 负责。
     */
    g_b21_last_state = 1U;
    g_b21_last_time = 0U;
    g_sw1_last_state = 1U;
    g_sw1_last_time = 0U;
    g_sw2_last_state = 1U;
    g_sw2_last_time = 0U;

    g_b21_pressed_event = 0U;
    g_sw1_pressed_event = 0U;
    g_sw2_pressed_event = 0U;

    g_button_left_target_rpm = BUTTON_DEFAULT_LEFT_TARGET_RPM;
    g_button_right_target_rpm = BUTTON_DEFAULT_RIGHT_TARGET_RPM;

    Button_ApplySpeedLoopState(0U);
}

void Button_Task(void)
{
    if (Button_CheckPressedEvent(GPIO_Button_PIN_Button_PORT,
                                 GPIO_Button_PIN_Button_PIN,
                                 &g_b21_last_state,
                                 &g_b21_last_time) != 0U) {
        g_b21_pressed_event = 1U;
    }

    if (Button_CheckPressedEvent(GPIO_Button_PIN_Button_SW1_PORT,
                                 GPIO_Button_PIN_Button_SW1_PIN,
                                 &g_sw1_last_state,
                                 &g_sw1_last_time) != 0U) {
        g_sw1_pressed_event = 1U;
    }

    if (Button_CheckPressedEvent(GPIO_Button_PIN_Button_SW2_PORT,
                                 GPIO_Button_PIN_Button_SW2_PIN,
                                 &g_sw2_last_state,
                                 &g_sw2_last_time) != 0U) {
        g_sw2_pressed_event = 1U;
    }
}

uint8_t Button_GetAndClearB21PressedEvent(void)
{
    uint8_t event = g_b21_pressed_event;
    g_b21_pressed_event = 0U;
    return event;
}

uint8_t Button_GetAndClearSW1PressedEvent(void)
{
    uint8_t event = g_sw1_pressed_event;
    g_sw1_pressed_event = 0U;
    return event;
}

uint8_t Button_GetAndClearSW2PressedEvent(void)
{
    uint8_t event = g_sw2_pressed_event;
    g_sw2_pressed_event = 0U;
    return event;
}

void Button_SetSpeedTargets(float left_target_rpm, float right_target_rpm)
{
    g_button_left_target_rpm = left_target_rpm;
    g_button_right_target_rpm = right_target_rpm;

    if (g_speed_loop_running != 0U) {
        MotorSpeedLoop_SetLeftWheelTargetRPM(g_button_left_target_rpm);
        MotorSpeedLoop_SetRightWheelTargetRPM(g_button_right_target_rpm);
    }
}

void Button_GetSpeedTargets(float *left_target_rpm, float *right_target_rpm)
{
    if (left_target_rpm != NULL) {
        *left_target_rpm = g_button_left_target_rpm;
    }

    if (right_target_rpm != NULL) {
        *right_target_rpm = g_button_right_target_rpm;
    }
}

void Button_StartMotorControl(void)
{
    Button_ApplySpeedLoopState(1U);
}

void Button_StopMotorControl(void)
{
    Button_ApplySpeedLoopState(0U);
    MotorPWM_StopAllChannels();
}

uint8_t Button_IsMotorControlRunning(void)
{
    return g_speed_loop_running;
}
