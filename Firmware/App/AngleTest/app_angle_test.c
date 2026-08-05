#include "app_angle_test.h"
#include <stddef.h>
#include <stdio.h>

#define APP_ANGLE_TEST_TIMEOUT_MS          (3000U)
#define APP_ANGLE_TEST_EXIT_CONFIRM_COUNT  (3U)

static char AppAngleTest_ToUpperAscii(char value)
{
    if ((value >= 'a') && (value <= 'z')) {
        return (char) (value - ('a' - 'A'));
    }

    return value;
}

static uint8_t AppAngleTest_IsCommandSeparator(char value)
{
    return (uint8_t) (((value == '\0') ||
                       (value == '?') ||
                       (value == ',') ||
                       (value == ':') ||
                       (value == '=') ||
                       (value == ' ')) ? 1U : 0U);
}

static uint8_t AppAngleTest_HasCommandPrefix(const char *command, const char *prefix)
{
    uint16_t index = 0U;

    if ((command == NULL) || (prefix == NULL)) {
        return 0U;
    }

    while (prefix[index] != '\0') {
        if (AppAngleTest_ToUpperAscii(command[index]) != AppAngleTest_ToUpperAscii(prefix[index])) {
            return 0U;
        }
        index++;
    }

    return AppAngleTest_IsCommandSeparator(command[index]);
}

static const char *AppAngleTest_GetCommandPayload(const char *command)
{
    if (command == NULL) {
        return NULL;
    }

    while ((*command != '\0') && (AppAngleTest_IsCommandSeparator(*command) == 0U)) {
        command++;
    }

    if ((*command == '\0') || (*command == '?')) {
        return NULL;
    }

    command++;
    while (*command == ' ') {
        command++;
    }

    return command;
}

void AppAngleTest_Init(AppAngleTest *test)
{
    if (test == NULL) {
        return;
    }

    AppAngleTest_Reset(test);
}

void AppAngleTest_Reset(AppAngleTest *test)
{
    if (test == NULL) {
        return;
    }

    test->active = 0U;
    test->exit_confirm_count = 0U;
    test->start_time = 0U;
    test->start_yaw = 0.0f;
    test->target_deg = 0.0f;
    AppAngleLoop_Reset(&test->loop);
}

uint8_t AppAngleTest_IsActive(const AppAngleTest *test)
{
    return (uint8_t) (((test != NULL) && (test->active != 0U)) ? 1U : 0U);
}

float AppAngleTest_GetTargetDeg(const AppAngleTest *test)
{
    if (test == NULL) {
        return 0.0f;
    }

    return test->target_deg;
}

float AppAngleTest_GetLastErrorDeg(const AppAngleTest *test)
{
    if (test == NULL) {
        return 0.0f;
    }

    return AppAngleLoop_GetLastErrorDeg(&test->loop);
}

AppAngleTestCommand AppAngleTest_ParseCommand(const char *command, float *target_deg)
{
    const char *payload;
    float parsed_target_deg;

    if (AppAngleTest_HasCommandPrefix(command, "ANGLE") == 0U) {
        return APP_ANGLE_TEST_COMMAND_NONE;
    }

    /*
     * 支持三种形式：
     * ANGLE? 查询当前角度测试状态；
     * ANGLE,65 执行正向相对旋转 65 度；
     * ANGLE,-65 执行反向相对旋转 65 度。
     */
    payload = AppAngleTest_GetCommandPayload(command);
    if (payload == NULL) {
        return APP_ANGLE_TEST_COMMAND_QUERY;
    }

    if ((sscanf(payload, "%f", &parsed_target_deg) != 1) ||
        (parsed_target_deg > 180.0f) ||
        (parsed_target_deg < -180.0f) ||
        ((parsed_target_deg > -1.0f) && (parsed_target_deg < 1.0f))) {
        return APP_ANGLE_TEST_COMMAND_ERROR;
    }

    if (target_deg != NULL) {
        *target_deg = parsed_target_deg;
    }

    return APP_ANGLE_TEST_COMMAND_START;
}

void AppAngleTest_Start(AppAngleTest *test,
                        float relative_target_deg,
                        float current_yaw_deg,
                        uint32_t current_time_ms)
{
    if (test == NULL) {
        return;
    }

    /*
     * 角度测试以“当前 yaw”为起点，只控制相对角度。
     * 这样 MPU6050 有零漂时，也不要求 yaw 必须正好从 0 度开始。
     */
    test->active = 1U;
    test->exit_confirm_count = 0U;
    test->start_time = current_time_ms;
    test->start_yaw = AppAngleLoop_NormalizeYawDeg(current_yaw_deg);
    test->target_deg = relative_target_deg;
    AppAngleLoop_StartRelative(&test->loop,
                               test->start_yaw,
                               relative_target_deg,
                               current_time_ms);
}

uint8_t AppAngleTest_Update(AppAngleTest *test,
                            float current_yaw_deg,
                            uint32_t current_time_ms,
                            float *left_target_rpm,
                            float *right_target_rpm,
                            uint8_t *timeout)
{
    float turn_wheel_target_rpm;
    uint32_t elapsed_ms;
    uint8_t exit_by_yaw;
    uint8_t exit_by_timeout;

    if (left_target_rpm != NULL) {
        *left_target_rpm = 0.0f;
    }

    if (right_target_rpm != NULL) {
        *right_target_rpm = 0.0f;
    }

    if (timeout != NULL) {
        *timeout = 0U;
    }

    if ((test == NULL) || (test->active == 0U)) {
        return 0U;
    }

    /*
     * 角度环只给出“哪一侧轮子应该转、目标 RPM 是多少”。
     * 实际 PWM 仍交给速度环处理，避免角度环直接绕过已有的电机保护和速度控制。
     */
    turn_wheel_target_rpm = AppAngleLoop_Update(&test->loop,
                                                AppAngleLoop_NormalizeYawDeg(current_yaw_deg),
                                                current_time_ms);

    if (turn_wheel_target_rpm >= 0.0f) {
        if (right_target_rpm != NULL) {
            *right_target_rpm = turn_wheel_target_rpm;
        }
    } else {
        if (left_target_rpm != NULL) {
            *left_target_rpm = -turn_wheel_target_rpm;
        }
    }

    elapsed_ms = (uint32_t) (current_time_ms - test->start_time);
    exit_by_yaw = AppAngleLoop_IsTargetReached(&test->loop);
    exit_by_timeout = (uint8_t) ((elapsed_ms >= APP_ANGLE_TEST_TIMEOUT_MS) ? 1U : 0U);

    /* 连续确认几次再退出，避免 yaw 单次抖动导致提前停车。 */
    if ((exit_by_yaw != 0U) || (exit_by_timeout != 0U)) {
        if (test->exit_confirm_count < 255U) {
            test->exit_confirm_count++;
        }
    } else {
        test->exit_confirm_count = 0U;
    }

    if (timeout != NULL) {
        *timeout = exit_by_timeout;
    }

    return (uint8_t) ((test->exit_confirm_count >= APP_ANGLE_TEST_EXIT_CONFIRM_COUNT) ? 1U : 0U);
}
