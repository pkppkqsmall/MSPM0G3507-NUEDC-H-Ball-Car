#include "app_angle_loop.h"
#include <stddef.h>
#include "../Common/app_math.h"

/*
 * 角度环默认参数。
 * 输出单位是 RPM，因为它作为速度环外环：角度环给目标速度，速度环负责落实 PWM。
 */
#define APP_ANGLE_LOOP_KP_RPM_PER_DEG          (0.35f)
#define APP_ANGLE_LOOP_KI_RPM_PER_DEG_SEC      (0.0f)
#define APP_ANGLE_LOOP_KD_RPM_PER_DEG_PER_SEC  (0.015f)
#define APP_ANGLE_LOOP_MIN_OUTPUT_RPM          (10.0f)
#define APP_ANGLE_LOOP_MAX_OUTPUT_RPM          (26.0f)
#define APP_ANGLE_LOOP_INTEGRAL_LIMIT          (80.0f)
#define APP_ANGLE_LOOP_TARGET_TOLERANCE_DEG    (3.0f)

float AppAngleLoop_NormalizeYawDeg(float yaw_deg)
{
    while (yaw_deg > 180.0f) {
        yaw_deg -= 360.0f;
    }

    while (yaw_deg < -180.0f) {
        yaw_deg += 360.0f;
    }

    return yaw_deg;
}

void AppAngleLoop_Init(AppAngleLoop *loop)
{
    AppAngleLoop_Reset(loop);
}

void AppAngleLoop_Reset(AppAngleLoop *loop)
{
    if (loop == NULL) {
        return;
    }

    loop->target_yaw_deg = 0.0f;
    loop->last_error_deg = 0.0f;
    loop->integral_deg_sec = 0.0f;
    loop->last_output_rpm = 0.0f;
    loop->target_direction = 0;
    loop->active = 0U;
    loop->first_update = 1U;
    loop->last_update_time_ms = 0U;
}

void AppAngleLoop_StartRelative(AppAngleLoop *loop,
                                float current_yaw_deg,
                                float relative_target_deg,
                                uint32_t current_time_ms)
{
    if (loop == NULL) {
        return;
    }

    loop->target_yaw_deg =
        AppAngleLoop_NormalizeYawDeg(current_yaw_deg + relative_target_deg);
    loop->last_error_deg =
        AppAngleLoop_NormalizeYawDeg(loop->target_yaw_deg - current_yaw_deg);
    loop->integral_deg_sec = 0.0f;
    loop->last_output_rpm = 0.0f;
    loop->target_direction = App_GetFloatDirection(relative_target_deg);
    loop->active = 1U;
    loop->first_update = 1U;
    loop->last_update_time_ms = current_time_ms;
}

float AppAngleLoop_Update(AppAngleLoop *loop,
                          float current_yaw_deg,
                          uint32_t current_time_ms)
{
    float error_deg;
    float previous_error_deg;
    float error_abs_deg;
    float elapsed_sec;
    float derivative_deg_per_sec = 0.0f;
    float output_rpm;
    float output_abs_rpm;
    int8_t output_direction;

    if ((loop == NULL) || (loop->active == 0U)) {
        return 0.0f;
    }

    previous_error_deg = loop->last_error_deg;
    error_deg = AppAngleLoop_NormalizeYawDeg(loop->target_yaw_deg - current_yaw_deg);

    if (loop->target_direction == 0) {
        loop->last_output_rpm = 0.0f;
        return 0.0f;
    }

    /*
     * 正向旋转时 target_direction 为正，反向旋转时为负。
     * 用 target_direction * error 判断是否已经接近目标，可以同时处理两个方向和 180/-180 溢出。
     */
    if (((float) loop->target_direction * error_deg) <= APP_ANGLE_LOOP_TARGET_TOLERANCE_DEG) {
        loop->last_error_deg = error_deg;
        loop->last_update_time_ms = current_time_ms;
        loop->last_output_rpm = 0.0f;
        return 0.0f;
    }

    if (loop->first_update != 0U) {
        elapsed_sec = 0.02f;
        derivative_deg_per_sec = 0.0f;
        loop->first_update = 0U;
    } else {
        uint32_t elapsed_ms = current_time_ms - loop->last_update_time_ms;
        if (elapsed_ms == 0U) {
            elapsed_ms = 1U;
        }
        elapsed_sec = (float) elapsed_ms / 1000.0f;
        derivative_deg_per_sec = (error_deg - previous_error_deg) / elapsed_sec;
    }

    loop->last_update_time_ms = current_time_ms;
    loop->last_error_deg = error_deg;

    loop->integral_deg_sec += error_deg * elapsed_sec;
    loop->integral_deg_sec = App_ClampFloat(loop->integral_deg_sec,
                                            -APP_ANGLE_LOOP_INTEGRAL_LIMIT,
                                            APP_ANGLE_LOOP_INTEGRAL_LIMIT);

    output_rpm =
        (APP_ANGLE_LOOP_KP_RPM_PER_DEG * error_deg) +
        (APP_ANGLE_LOOP_KI_RPM_PER_DEG_SEC * loop->integral_deg_sec) +
        (APP_ANGLE_LOOP_KD_RPM_PER_DEG_PER_SEC * derivative_deg_per_sec);

    output_direction = App_GetFloatDirection(output_rpm);
    if (output_direction != loop->target_direction) {
        output_rpm = 0.0f;
    }

    error_abs_deg = App_AbsFloat(error_deg);
    output_abs_rpm = App_AbsFloat(output_rpm);

    if (error_abs_deg > APP_ANGLE_LOOP_TARGET_TOLERANCE_DEG) {
        output_abs_rpm = App_ClampFloat(output_abs_rpm,
                                        APP_ANGLE_LOOP_MIN_OUTPUT_RPM,
                                        APP_ANGLE_LOOP_MAX_OUTPUT_RPM);
    }

    loop->last_output_rpm = output_abs_rpm * (float) loop->target_direction;
    return loop->last_output_rpm;
}

uint8_t AppAngleLoop_IsTargetReached(const AppAngleLoop *loop)
{
    if ((loop == NULL) || (loop->active == 0U)) {
        return 1U;
    }

    if (loop->target_direction == 0) {
        return 1U;
    }

    return (uint8_t) ((((float) loop->target_direction * loop->last_error_deg) <=
                       APP_ANGLE_LOOP_TARGET_TOLERANCE_DEG) ? 1U : 0U);
}

float AppAngleLoop_GetLastErrorDeg(const AppAngleLoop *loop)
{
    if (loop == NULL) {
        return 0.0f;
    }

    return loop->last_error_deg;
}
