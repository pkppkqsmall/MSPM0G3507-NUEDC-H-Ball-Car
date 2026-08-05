#include "stepper_demo.h"

#include <stdint.h>
#include <stdio.h>

#include "bno085.h"
#include "clock.h"
#include "interrupt.h"
#include "oled_hardware_i2c.h"
#include "Sensor.h"
#include "stepper_motor.h"

#define STEPPER_DEMO_POWER_ON_WAIT_MS       (500UL)
#define STEPPER_DEMO_STOP_WAIT_MS           (1000UL)
#define STEPPER_DEMO_DISPLAY_PERIOD_MS      (100UL)
#define STEPPER_DEMO_BUTTON_DEBOUNCE_MS     (30UL)
#define SENSOR_CALIBRATION_ENTER_HOLD_MS    (4200UL)
#define SENSOR_CALIBRATION_SHORT_HOLD_MS    (150UL)
#define SENSOR_CALIBRATION_SETTLE_MS        (800UL)
#define SENSOR_CALIBRATION_DONE_HOLD_MS     (1200UL)
/* 修改这里即可设置每次正转和反转的机械圈数。 */
#define STEPPER_DEMO_MOVE_REVOLUTIONS       (1UL)

typedef enum {
    STEPPER_DEMO_POWER_ON_WAIT = 0,
    STEPPER_DEMO_POSITIVE_MOVE,
    STEPPER_DEMO_POSITIVE_STOP,
    STEPPER_DEMO_NEGATIVE_MOVE,
    STEPPER_DEMO_NEGATIVE_STOP
} StepperDemoState;

typedef enum {
    SENSOR_CALIBRATION_IDLE = 0,
    SENSOR_CALIBRATION_WAIT_MOTOR,
    SENSOR_CALIBRATION_ENTER_HOLD,
    SENSOR_CALIBRATION_ENTER_SETTLE,
    SENSOR_CALIBRATION_WAIT_BLACK,
    SENSOR_CALIBRATION_BLACK_HOLD,
    SENSOR_CALIBRATION_BLACK_SETTLE,
    SENSOR_CALIBRATION_WAIT_WHITE,
    SENSOR_CALIBRATION_WHITE_HOLD,
    SENSOR_CALIBRATION_DONE
} SensorCalibrationState;

static StepperDemoState g_demo_state;
static SensorCalibrationState g_calibration_state;
static unsigned long g_demo_state_start_ms;
static unsigned long g_calibration_state_start_ms;
static unsigned long g_last_display_ms;
static unsigned long g_b21_change_ms;
static uint8_t g_b21_raw_state;
static uint8_t g_b21_stable_state;
static uint8_t g_display_buffer[32];

static uint8_t StepperDemo_CheckB21Pressed(void)
{
    uint8_t current_state =
        (DL_GPIO_readPins(GPIO_Button_PIN_Button_PORT,
                          GPIO_Button_PIN_Button_PIN) == 0U) ? 0U : 1U;

    if (current_state != g_b21_raw_state) {
        g_b21_raw_state = current_state;
        g_b21_change_ms = tick_ms;
    }

    if ((current_state != g_b21_stable_state) &&
        ((tick_ms - g_b21_change_ms) >= STEPPER_DEMO_BUTTON_DEBOUNCE_MS)) {
        g_b21_stable_state = current_state;
        if (g_b21_stable_state == 0U) {
            return 1U;
        }
    }

    return 0U;
}

static void StepperDemo_FormatSensorLine(uint8_t sensor_value)
{
    uint8_t index;

    g_display_buffer[0] = 'G';
    g_display_buffer[1] = 'r';
    g_display_buffer[2] = 'a';
    g_display_buffer[3] = 'y';
    g_display_buffer[4] = ':';

    if (Sensor_IsReady() != 0U) {
        for (index = 0U; index < SENSOR_CHANNEL_COUNT; index++) {
            g_display_buffer[5U + index] =
                (((sensor_value >> (7U - index)) & 0x01U) != 0U) ? '1' : '0';
        }
        g_display_buffer[13] = ' ';
        g_display_buffer[14] = 'S';
        g_display_buffer[15] = 'R';
    } else {
        for (index = 0U; index < SENSOR_CHANNEL_COUNT; index++) {
            g_display_buffer[5U + index] = '-';
        }
        g_display_buffer[13] = ' ';
        g_display_buffer[14] = 'E';
        g_display_buffer[15] =
            (uint8_t) ('0' + (uint8_t) Sensor_GetLastError());
    }

    g_display_buffer[16] = '\0';
}

static void StepperDemo_SetCalibrationState(
    SensorCalibrationState state)
{
    g_calibration_state = state;
    g_calibration_state_start_ms = tick_ms;
}

static void StepperDemo_HandleCalibrationButton(void)
{
    switch (g_calibration_state) {
        case SENSOR_CALIBRATION_IDLE:
            /*
             * 请求在当前 STEP 高脉冲结束后停机，再开始传感器校准。
             * 已经发出的脉冲仍会计入软件位置，不会破坏角度累计。
             */
            Sensor_SetCalibrationKeyPressed(0U);
            StepperMotor_RequestStop();
            StepperDemo_SetCalibrationState(
                SENSOR_CALIBRATION_WAIT_MOTOR);
            break;

        case SENSOR_CALIBRATION_WAIT_BLACK:
            Sensor_SetCalibrationKeyPressed(1U);
            StepperDemo_SetCalibrationState(
                SENSOR_CALIBRATION_BLACK_HOLD);
            break;

        case SENSOR_CALIBRATION_WAIT_WHITE:
            Sensor_SetCalibrationKeyPressed(1U);
            StepperDemo_SetCalibrationState(
                SENSOR_CALIBRATION_WHITE_HOLD);
            break;

        default:
            break;
    }
}

static void StepperDemo_UpdateCalibration(void)
{
    switch (g_calibration_state) {
        case SENSOR_CALIBRATION_IDLE:
            break;

        case SENSOR_CALIBRATION_WAIT_MOTOR:
            if (StepperMotor_IsBusy() == 0U) {
                Sensor_SetCalibrationKeyPressed(1U);
                StepperDemo_SetCalibrationState(
                    SENSOR_CALIBRATION_ENTER_HOLD);
            }
            break;

        case SENSOR_CALIBRATION_ENTER_HOLD:
            if ((tick_ms - g_calibration_state_start_ms) >=
                SENSOR_CALIBRATION_ENTER_HOLD_MS) {
                Sensor_SetCalibrationKeyPressed(0U);
                StepperDemo_SetCalibrationState(
                    SENSOR_CALIBRATION_ENTER_SETTLE);
            }
            break;

        case SENSOR_CALIBRATION_ENTER_SETTLE:
            if ((tick_ms - g_calibration_state_start_ms) >=
                SENSOR_CALIBRATION_SETTLE_MS) {
                StepperDemo_SetCalibrationState(
                    SENSOR_CALIBRATION_WAIT_BLACK);
            }
            break;

        case SENSOR_CALIBRATION_WAIT_BLACK:
            break;

        case SENSOR_CALIBRATION_BLACK_HOLD:
            if ((tick_ms - g_calibration_state_start_ms) >=
                SENSOR_CALIBRATION_SHORT_HOLD_MS) {
                Sensor_SetCalibrationKeyPressed(0U);
                StepperDemo_SetCalibrationState(
                    SENSOR_CALIBRATION_BLACK_SETTLE);
            }
            break;

        case SENSOR_CALIBRATION_BLACK_SETTLE:
            if ((tick_ms - g_calibration_state_start_ms) >=
                SENSOR_CALIBRATION_SETTLE_MS) {
                StepperDemo_SetCalibrationState(
                    SENSOR_CALIBRATION_WAIT_WHITE);
            }
            break;

        case SENSOR_CALIBRATION_WAIT_WHITE:
            break;

        case SENSOR_CALIBRATION_WHITE_HOLD:
            if ((tick_ms - g_calibration_state_start_ms) >=
                SENSOR_CALIBRATION_SHORT_HOLD_MS) {
                Sensor_SetCalibrationKeyPressed(0U);
                StepperDemo_SetCalibrationState(
                    SENSOR_CALIBRATION_DONE);
            }
            break;

        case SENSOR_CALIBRATION_DONE:
            if ((tick_ms - g_calibration_state_start_ms) >=
                SENSOR_CALIBRATION_DONE_HOLD_MS) {
                g_demo_state = STEPPER_DEMO_POWER_ON_WAIT;
                g_demo_state_start_ms = tick_ms;
                StepperDemo_SetCalibrationState(
                    SENSOR_CALIBRATION_IDLE);
            }
            break;

        default:
            Sensor_SetCalibrationKeyPressed(0U);
            StepperDemo_SetCalibrationState(SENSOR_CALIBRATION_IDLE);
            break;
    }
}

static void StepperDemo_StartPositiveMove(void)
{
    if (StepperMotor_StartRevolutions(STEPPER_DIRECTION_POSITIVE,
                                      STEPPER_DEMO_MOVE_REVOLUTIONS) != 0U) {
        g_demo_state = STEPPER_DEMO_POSITIVE_MOVE;
    }
}

static void StepperDemo_StartNegativeMove(void)
{
    if (StepperMotor_StartRevolutions(STEPPER_DIRECTION_NEGATIVE,
                                      STEPPER_DEMO_MOVE_REVOLUTIONS) != 0U) {
        g_demo_state = STEPPER_DEMO_NEGATIVE_MOVE;
    }
}

static void StepperDemo_UpdateMotion(void)
{
    switch (g_demo_state) {
        case STEPPER_DEMO_POWER_ON_WAIT:
            if ((tick_ms - g_demo_state_start_ms) >=
                STEPPER_DEMO_POWER_ON_WAIT_MS) {
                StepperDemo_StartPositiveMove();
            }
            break;

        case STEPPER_DEMO_POSITIVE_MOVE:
            if (StepperMotor_IsBusy() == 0U) {
                g_demo_state = STEPPER_DEMO_POSITIVE_STOP;
                g_demo_state_start_ms = tick_ms;
            }
            break;

        case STEPPER_DEMO_POSITIVE_STOP:
            if ((tick_ms - g_demo_state_start_ms) >=
                STEPPER_DEMO_STOP_WAIT_MS) {
                StepperDemo_StartNegativeMove();
            }
            break;

        case STEPPER_DEMO_NEGATIVE_MOVE:
            if (StepperMotor_IsBusy() == 0U) {
                g_demo_state = STEPPER_DEMO_NEGATIVE_STOP;
                g_demo_state_start_ms = tick_ms;
            }
            break;

        case STEPPER_DEMO_NEGATIVE_STOP:
            if ((tick_ms - g_demo_state_start_ms) >=
                STEPPER_DEMO_STOP_WAIT_MS) {
                StepperDemo_StartPositiveMove();
            }
            break;

        default:
            g_demo_state = STEPPER_DEMO_POWER_ON_WAIT;
            g_demo_state_start_ms = tick_ms;
            break;
    }
}

static void StepperDemo_UpdateCalibrationDisplay(void)
{
    const uint8_t *status_text;
    const uint8_t *action_text = (const uint8_t *) "                ";

    switch (g_calibration_state) {
        case SENSOR_CALIBRATION_WAIT_MOTOR:
            status_text = (const uint8_t *) "Wait motor...   ";
            break;

        case SENSOR_CALIBRATION_ENTER_HOLD:
            status_text = (const uint8_t *) "Entering 4s...  ";
            break;

        case SENSOR_CALIBRATION_ENTER_SETTLE:
            status_text = (const uint8_t *) "Entering...     ";
            break;

        case SENSOR_CALIBRATION_WAIT_BLACK:
            status_text = (const uint8_t *) "Place BLACK     ";
            action_text = (const uint8_t *) "Press B21       ";
            break;

        case SENSOR_CALIBRATION_BLACK_HOLD:
        case SENSOR_CALIBRATION_BLACK_SETTLE:
            status_text = (const uint8_t *) "Saving BLACK... ";
            break;

        case SENSOR_CALIBRATION_WAIT_WHITE:
            status_text = (const uint8_t *) "Place WHITE     ";
            action_text = (const uint8_t *) "Press B21       ";
            break;

        case SENSOR_CALIBRATION_WHITE_HOLD:
            status_text = (const uint8_t *) "Saving WHITE... ";
            break;

        case SENSOR_CALIBRATION_DONE:
            status_text = (const uint8_t *) "Calibration OK  ";
            break;

        default:
            status_text = (const uint8_t *) "Calibration ERR ";
            break;
    }

    OLED_ShowString(0, 0, (uint8_t *) "Gray calibrate  ", 8);
    OLED_ShowString(0, 2, (uint8_t *) status_text, 8);
    OLED_ShowString(0, 4, (uint8_t *) action_text, 8);
    OLED_ShowString(0, 6, (uint8_t *) "KEY:PA13 B21    ", 8);
}

static void StepperDemo_UpdateDisplay(void)
{
    uint8_t sensor_value;

    if ((tick_ms - g_last_display_ms) < STEPPER_DEMO_DISPLAY_PERIOD_MS) {
        return;
    }
    g_last_display_ms = tick_ms;

    if (g_calibration_state != SENSOR_CALIBRATION_IDLE) {
        StepperDemo_UpdateCalibrationDisplay();
        return;
    }

    snprintf((char *) g_display_buffer,
             sizeof(g_display_buffer),
             "Turn:%+7ld      ",
             (long) StepperMotor_GetCompletedRevolutions());
    OLED_ShowString(0, 0, g_display_buffer, 8);

    snprintf((char *) g_display_buffer,
             sizeof(g_display_buffer),
             "Step:%+7.1f      ",
             StepperMotor_GetWithinRevolutionAngleDeg());
    OLED_ShowString(0, 2, g_display_buffer, 8);

    snprintf((char *) g_display_buffer,
             sizeof(g_display_buffer),
             "Yaw :%+7.1f      ",
             yaw);
    OLED_ShowString(0, 4, g_display_buffer, 8);

    sensor_value = Sensor_Read_Grayscale();
    StepperDemo_FormatSensorLine(sensor_value);
    OLED_ShowString(0, 6, g_display_buffer, 8);
}

void StepperDemo_Init(void)
{
    uint8_t bno085_ready;

    /*
     * STEP/DIR 先进入安全停止状态，再打开系统节拍。
     * OLED 和 BNO085 初始化都需要毫秒时基，因此随后开启全局中断。
     */
    StepperMotor_Init();
    SysTick_Init();
    __enable_irq();

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *) "BNO085 init...  ", 8);

    (void) Sensor_Init();
    bno085_ready = BNO085_Init();
    if (bno085_ready != 0U) {
        OLED_ShowString(0, 2, (uint8_t *) "BNO085 OK       ", 8);
    } else {
        OLED_ShowString(0, 2, (uint8_t *) "BNO085 FAIL     ", 8);
    }

    /*
     * BNO085 初始化完成后再使能 GPIO 组中断。
     * 初始化阶段仍可直接读取低有效 INT 引脚，不依赖 ISR。
     */
    Interrupt_Init();

    g_demo_state = STEPPER_DEMO_POWER_ON_WAIT;
    g_calibration_state = SENSOR_CALIBRATION_IDLE;
    g_demo_state_start_ms = tick_ms;
    g_calibration_state_start_ms = tick_ms;
    g_last_display_ms = tick_ms;

    g_b21_raw_state =
        (DL_GPIO_readPins(GPIO_Button_PIN_Button_PORT,
                          GPIO_Button_PIN_Button_PIN) == 0U) ? 0U : 1U;
    g_b21_stable_state = g_b21_raw_state;
    g_b21_change_ms = tick_ms;
}

void StepperDemo_RunStep(void)
{
    (void) BNO085_UpdateIfDataReady();

    if (StepperDemo_CheckB21Pressed() != 0U) {
        StepperDemo_HandleCalibrationButton();
    }

    StepperDemo_UpdateCalibration();
    if (g_calibration_state == SENSOR_CALIBRATION_IDLE) {
        StepperDemo_UpdateMotion();
    }

    StepperDemo_UpdateDisplay();
}
