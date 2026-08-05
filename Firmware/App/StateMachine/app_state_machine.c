#include <stddef.h>
#include <stdint.h>

#include "app_state_machine.h"
#include "app_angle_test.h"
#include "app_line_follow.h"
#include "app_oled_view.h"
#include "../SensorTask/app_sensor_task.h"
#include "../SpeedTask/app_speed_task.h"
#include "Button.h"
#include "UART.h"
#include "clock.h"
#include "interrupt.h"
#include "motor_pwm.h"
#include "mpu6050.h"
#include "oled_hardware_i2c.h"
#include "ti_msp_dl_config.h"

typedef enum {
    APP_STATE_INIT = 0,
    APP_STATE_INPUT,
    APP_STATE_SENSOR_SCAN,
    APP_STATE_MPU_UPDATE,
    APP_STATE_UART_RX,
    APP_STATE_UART_CMD,
    APP_STATE_ANGLE_TEST,
    APP_STATE_LINE_FOLLOW,
    APP_STATE_SPEED_CONTROL,
    APP_STATE_DISPLAY
} AppState;

/*
 * 状态机只保存调度所需的上下文。
 * 具体算法拆到 SensorTask、LineFollow、AngleTest、SpeedTask 等模块里，
 * 这里专注决定“下一步执行哪个任务”。
 */
typedef struct {
    AppState state;
    unsigned long last_display_time;
    uint8_t display_refresh_requested;
    uint8_t line_follow_active;

    AppSensorTask sensor;
    AppLineFollow line_follow;
    AppAngleTest angle_test;
    AppSpeedTask speed_task;

    char bt_line[UART_BT_LINE_BUFFER_SIZE];
} AppContext;

static AppContext g_app;

/* 速度环默认参数，蓝牙 PID/SET 命令仍然可以在线覆盖。 */
#define APP_SPEED_LOOP_DEFAULT_KP (0.1000f)
#define APP_SPEED_LOOP_DEFAULT_KI (0.2000f)
#define APP_SPEED_LOOP_DEFAULT_KD (0.0000f)

static void App_InitModules(AppContext *app)
{
    if (app == NULL) {
        return;
    }

    app->last_display_time = 0U;
    app->display_refresh_requested = 1U;
    app->line_follow_active = 0U;

    AppSensorTask_Init(&app->sensor);
    AppLineFollow_Init(&app->line_follow);
    AppAngleTest_Init(&app->angle_test);
    AppSpeedTask_Init(&app->speed_task);

    app->bt_line[0] = '\0';
}

static void App_ResetAngleTest(AppContext *app)
{
    if (app == NULL) {
        return;
    }

    AppAngleTest_Reset(&app->angle_test);
}

static void App_ResetMotionRuntime(AppContext *app)
{
    if (app == NULL) {
        return;
    }

    AppLineFollow_ResetRuntime(&app->line_follow);
    app->line_follow_active = 0U;
}

static uint8_t App_IsInputLocked(const AppContext *app)
{
    if (app == NULL) {
        return 1U;
    }

    return AppAngleTest_IsActive(&app->angle_test);
}

static void App_PrepareAiTuningSpeedOnlyMode(AppContext *app)
{
    if (app == NULL) {
        return;
    }

    /*
     * AI 调参只验证速度环，因此进入该模式时关闭巡线和角度测试，
     * 让串口采样只反映“目标速度 -> 实际速度”的速度环表现。
     */
    App_ResetMotionRuntime(app);
    App_ResetAngleTest(app);
}

static void App_StopRun(AppContext *app)
{
    if (app == NULL) {
        return;
    }

    Button_StopMotorControl();
    App_ResetMotionRuntime(app);
    App_ResetAngleTest(app);
    app->display_refresh_requested = 1U;
}

static void App_StartManualSpeedMode(AppContext *app)
{
    if (app == NULL) {
        return;
    }

    /*
     * 蓝牙 START 只验证左右轮速度环，不让灰度巡线改写目标速度。
     */
    App_ResetMotionRuntime(app);
    App_ResetAngleTest(app);
    MotorSpeed_Reset();
    Button_StartMotorControl();
    app->display_refresh_requested = 1U;
}

static void App_StartLineFollow(AppContext *app)
{
    if (app == NULL) {
        return;
    }

    App_ResetMotionRuntime(app);
    App_ResetAngleTest(app);
    UART_LlmTunerSetEnabled(0U);
    MotorSpeed_Reset();
    app->line_follow_active = 1U;
    Button_StartMotorControl();
    app->display_refresh_requested = 1U;
}

static void App_StartAngleTest(AppContext *app, float relative_target_deg)
{
    if (app == NULL) {
        return;
    }

    /*
     * ANGLE 命令用于单独验证角度环。
     * 开始前先退出巡线和 AI 调参，避免多个控制器同时改目标速度。
     */
    Button_StopMotorControl();
    App_ResetMotionRuntime(app);
    UART_LlmTunerSetEnabled(0U);
    MotorSpeed_Reset();

    AppAngleTest_Start(&app->angle_test,
                       relative_target_deg,
                       yaw,
                       (uint32_t) tick_ms);

    Button_StartMotorControl();
    UART_Printf("[ANGLE OK] Start %.1f deg, yaw=%.1f\r\n",
                relative_target_deg,
                yaw);
}

static void App_ProcessButtonEvents(AppContext *app)
{
    uint8_t b21_pressed;
    uint8_t sw2_pressed;

    if (app == NULL) {
        return;
    }

    /*
     * B21 与当前 MapRun 的习惯一致：停止时启动连续巡线，运行时立即停机。
     * SW2 也可以在停止状态启动巡线；SW1 暂无应用层功能，但仍清除其事件。
     */
    b21_pressed = Button_GetAndClearB21PressedEvent();
    (void) Button_GetAndClearSW1PressedEvent();
    sw2_pressed = Button_GetAndClearSW2PressedEvent();

    if (b21_pressed != 0U) {
        if (Button_IsMotorControlRunning() != 0U) {
            App_StopRun(app);
        } else if (App_IsInputLocked(app) == 0U) {
            App_StartLineFollow(app);
        }
    }

    if ((sw2_pressed != 0U) &&
        (Button_IsMotorControlRunning() == 0U) &&
        (App_IsInputLocked(app) == 0U)) {
        App_StartLineFollow(app);
    }
}

static void App_Init(AppContext *app)
{
    uint8_t mpu_ready;

    /*
     * AppStateMachine_Init 只清应用上下文；真正访问硬件的动作在 INIT 状态执行。
     */
    SYSCFG_DL_init();
    MotorPWM_InitDefaults();

    SysTick_Init();
    /* OLED 初始化里会使用毫秒延时，因此先允许 SysTick 运行。 */
    __enable_irq();

    UART_BluetoothInit();
    OLED_Init();

    mpu_ready = MPU6050_Init();
    if (mpu_ready != 0U) {
        OLED_ShowString(0, 7, (uint8_t *) "MPU6050 Success", 8);
    } else {
        OLED_ShowString(0, 7, (uint8_t *) "MPU6050 Fail   ", 8);
    }

    /*
     * 等 OLED/MPU6050 初始化完成后再打开 GPIO 中断。
     * MPU6050 INT 只置数据就绪标志，DMP 读取仍在主循环执行。
     */
    Interrupt_Init();

    PID_SetParameters(APP_SPEED_LOOP_DEFAULT_KP,
                      APP_SPEED_LOOP_DEFAULT_KI,
                      APP_SPEED_LOOP_DEFAULT_KD);
    MotorSpeedLoop_SetLeftWheelPI(g_pid_params.kp, g_pid_params.ki);
    MotorSpeedLoop_SetRightWheelPI(g_pid_params.kp, g_pid_params.ki);

    Button_InitMotorControl();
    MotorSpeed_SetSampleTimeMs(50U);
    MotorSpeed_Reset();

    App_InitModules(app);
    DL_TimerG_startCounter(PWM_0_INST);
}

static uint8_t App_HandleAngleTestCommand(AppContext *app,
                                          const char *command)
{
    float target_deg;
    AppAngleTestCommand result;

    if (app == NULL) {
        return 0U;
    }

    result = AppAngleTest_ParseCommand(command, &target_deg);
    switch (result) {
        case APP_ANGLE_TEST_COMMAND_QUERY:
            UART_Printf("[ANGLE NOW] active=%u target=%.1f err=%.1f\r\n",
                        AppAngleTest_IsActive(&app->angle_test),
                        AppAngleTest_GetTargetDeg(&app->angle_test),
                        AppAngleTest_GetLastErrorDeg(&app->angle_test));
            return 1U;

        case APP_ANGLE_TEST_COMMAND_START:
            App_StartAngleTest(app, target_deg);
            return 1U;

        case APP_ANGLE_TEST_COMMAND_ERROR:
            UART_SendString("[ANGLE ERR] Use: ANGLE,65 or ANGLE,-65\r\n");
            return 1U;

        default:
            return 0U;
    }
}

static void App_HandleBluetoothCommand(AppContext *app)
{
    if ((app == NULL) ||
        (UART_BluetoothReadLine(app->bt_line,
                                sizeof(app->bt_line)) == 0U)) {
        return;
    }

    if (UART_BluetoothHandlePidCommand(app->bt_line) != 0U) {
        MotorSpeedLoop_SetLeftWheelPI(g_pid_params.kp, g_pid_params.ki);
        MotorSpeedLoop_SetRightWheelPI(g_pid_params.kp, g_pid_params.ki);
        return;
    }

    if (UART_BluetoothHandleWheelTuneCommand(app->bt_line) != 0U) {
        return;
    }

    if (UART_BluetoothHandleStopCommand(app->bt_line) != 0U) {
        App_StopRun(app);
        return;
    }

    if (UART_BluetoothHandleStartCommand(app->bt_line) != 0U) {
        App_StartManualSpeedMode(app);
        return;
    }

    if (UART_BluetoothHandleEncoderQueryCommand(app->bt_line) != 0U) {
        return;
    }

    if (UART_BluetoothHandleCarStatusCommand(app->bt_line) != 0U) {
        return;
    }

    if (App_HandleAngleTestCommand(app, app->bt_line) != 0U) {
        return;
    }

    if (UART_BluetoothHandleLlmTunerCommand(app->bt_line) != 0U) {
        if (UART_LlmTunerIsEnabled() != 0U) {
            App_PrepareAiTuningSpeedOnlyMode(app);
        }
        return;
    }

    if (UART_BluetoothHandleSetCommand(app->bt_line) != 0U) {
        return;
    }

    (void) UART_BluetoothHandleTargetCommand(app->bt_line);
}

static void App_UpdateSensorScan(AppContext *app)
{
    if (app == NULL) {
        return;
    }

    AppSensorTask_Update(&app->sensor);
}

static void App_UpdateMpu6050(void)
{
    /*
     * MPU6050 的 GPIO 中断只负责置位数据就绪标志。
     * 主循环上下文负责 I2C/DMP 读取和姿态角浮点解算。
     */
    (void) MPU6050_UpdateIfDataReady();
}

static void App_UpdateAngleTest(AppContext *app)
{
    float current_yaw;
    float left_target_rpm;
    float right_target_rpm;
    uint8_t exit_by_timeout;

    if ((app == NULL) ||
        (AppAngleTest_IsActive(&app->angle_test) == 0U)) {
        return;
    }

    if ((MotorSpeedLoop_IsLeftWheelEnabled() == 0U) &&
        (MotorSpeedLoop_IsRightWheelEnabled() == 0U)) {
        App_ResetAngleTest(app);
        AppLineFollow_ResetCorrection(&app->line_follow);
        return;
    }

    current_yaw = yaw;
    if (AppAngleTest_Update(&app->angle_test,
                            current_yaw,
                            (uint32_t) tick_ms,
                            &left_target_rpm,
                            &right_target_rpm,
                            &exit_by_timeout) == 0U) {
        MotorSpeedLoop_SetLeftWheelTargetRPM(left_target_rpm);
        MotorSpeedLoop_SetRightWheelTargetRPM(right_target_rpm);
        AppLineFollow_SetDebugError(&app->line_follow,
                                    (right_target_rpm > 0.0f) ? -8 : 8);
    } else {
        UART_Printf("[ANGLE DONE] target=%.1f yaw=%.1f err=%.1f timeout=%u\r\n",
                    AppAngleTest_GetTargetDeg(&app->angle_test),
                    current_yaw,
                    AppAngleTest_GetLastErrorDeg(&app->angle_test),
                    exit_by_timeout);
        Button_StopMotorControl();
        App_ResetAngleTest(app);
        AppLineFollow_ResetCorrection(&app->line_follow);
    }
}

static void App_UpdateLineFollow(AppContext *app)
{
    if (app == NULL) {
        return;
    }

    AppLineFollow_UpdateMotorTargets(&app->line_follow,
                                     AppSensorTask_GetValue(&app->sensor));
}

static void App_UpdateSpeedControl(AppContext *app)
{
    if (app == NULL) {
        return;
    }

    AppSpeedTask_Update(&app->speed_task);
}

static void App_UpdateDisplay(AppContext *app)
{
    if (app == NULL) {
        return;
    }

    AppOledView_Update(AppSensorTask_GetValue(&app->sensor),
                       &app->display_refresh_requested,
                       &app->last_display_time);
}

static void App_SelectMotionState(AppContext *app)
{
    if (app == NULL) {
        return;
    }

    if (UART_LlmTunerIsEnabled() != 0U) {
        App_PrepareAiTuningSpeedOnlyMode(app);
        app->state = APP_STATE_SPEED_CONTROL;
        return;
    }

    if (AppAngleTest_IsActive(&app->angle_test) != 0U) {
        app->state = APP_STATE_ANGLE_TEST;
        return;
    }

    if ((Button_IsMotorControlRunning() == 0U) ||
        (app->line_follow_active == 0U)) {
        app->state = APP_STATE_SPEED_CONTROL;
        return;
    }

    app->state = APP_STATE_LINE_FOLLOW;
}

static void App_RunStateMachine(AppContext *app)
{
    /*
     * 单步状态机每次只执行一个状态，按键、串口、传感器、
     * 运动控制、速度环和显示依次轮转。
     */
    switch (app->state) {
        case APP_STATE_INIT:
            App_Init(app);
            app->state = APP_STATE_INPUT;
            break;

        case APP_STATE_INPUT:
            Button_Task();
            App_ProcessButtonEvents(app);
            app->state = APP_STATE_SENSOR_SCAN;
            break;

        case APP_STATE_SENSOR_SCAN:
            App_UpdateSensorScan(app);
            app->state = APP_STATE_MPU_UPDATE;
            break;

        case APP_STATE_MPU_UPDATE:
            App_UpdateMpu6050();
            app->state = APP_STATE_UART_RX;
            break;

        case APP_STATE_UART_RX:
            UART_BluetoothTask();
            app->state = APP_STATE_UART_CMD;
            break;

        case APP_STATE_UART_CMD:
            App_HandleBluetoothCommand(app);
            App_SelectMotionState(app);
            break;

        case APP_STATE_ANGLE_TEST:
            App_UpdateAngleTest(app);
            app->state = APP_STATE_SPEED_CONTROL;
            break;

        case APP_STATE_LINE_FOLLOW:
            App_UpdateLineFollow(app);
            app->state = APP_STATE_SPEED_CONTROL;
            break;

        case APP_STATE_SPEED_CONTROL:
            App_UpdateSpeedControl(app);
            app->state = APP_STATE_DISPLAY;
            break;

        case APP_STATE_DISPLAY:
            App_UpdateDisplay(app);
            app->state = APP_STATE_INPUT;
            break;

        default:
            app->state = APP_STATE_INPUT;
            break;
    }
}

void AppStateMachine_Init(void)
{
    g_app.state = APP_STATE_INIT;
    App_InitModules(&g_app);
}

void AppStateMachine_RunStep(void)
{
    App_RunStateMachine(&g_app);
}
