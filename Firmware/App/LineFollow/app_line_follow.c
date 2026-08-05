#include "app_line_follow.h"
#include <stddef.h>
#include "../Common/app_math.h"
#include "Button.h"
#include "clock.h"
#include "motor_pwm.h"

/*
 * 巡线修正参数。
 * correction = error * gain，之后再做最大值限制和斜率限制，避免小车突然抽动。
 */
#define APP_LINE_FOLLOW_CORRECTION_GAIN_RPM          (1.25f)
#define APP_LINE_FOLLOW_MAX_CORRECTION_RPM           (14.0f)
#define APP_LINE_FOLLOW_CORRECTION_SLEW_RPM_PER_MS   (0.05f)
#define APP_LINE_FOLLOW_CORRECTION_SLEW_MAX_MS       (20U)
#define APP_LINE_FOLLOW_TARGET_SLEW_RPM_PER_STEP     (3.0f)
#define APP_LINE_FOLLOW_LOST_SPEED_SCALE             (0.65f)
#define APP_LINE_FOLLOW_LOST_HOLD_COUNT              (2U)
#define APP_LINE_FOLLOW_MIN_TARGET_RPM               (0.0f)

void AppLineFollow_Init(AppLineFollow *line)
{
    if (line == NULL) {
        return;
    }

    line->enabled = 1U;
    AppLineFollow_ResetRuntime(line);
}

void AppLineFollow_ResetRuntime(AppLineFollow *line)
{
    if (line == NULL) {
        return;
    }

    line->line_error = 0;
    line->last_line_error = 0;
    line->line_lost_count = 0U;
    AppLineFollow_ResetCorrection(line);
    AppLineFollow_ResetTargetRamp(line);
}

void AppLineFollow_ResetCorrection(AppLineFollow *line)
{
    if (line == NULL) {
        return;
    }

    line->line_correction_rpm = 0.0f;
    line->last_update_time = tick_ms;
}

void AppLineFollow_ResetTargetRamp(AppLineFollow *line)
{
    if (line == NULL) {
        return;
    }

    line->smooth_left_target_rpm = 0.0f;
    line->smooth_right_target_rpm = 0.0f;
    line->last_target_update_time = tick_ms;
    line->target_ramp_initialized = 0U;
}

uint8_t AppLineFollow_FilterSensorValue(uint8_t newest,
                                        uint8_t history1,
                                        uint8_t history2)
{
    return (uint8_t) ((newest & history1) | (newest & history2) | (history1 & history2));
}

int8_t AppLineFollow_CalculateError(uint8_t sensor_value, uint8_t *line_found)
{
    /*
     * 当前串行灰度 bit0 在最左侧，bit7 在最右侧。
     * 中间权重不能太小，否则只有压到最外侧时小车才有明显修正。
     */
    static const int8_t sensor_weights[8] = {-14, -9, -6, -2, 2, 6, 9, 14};
    int16_t weighted_sum = 0;
    int16_t active_count = 0;
    uint8_t index;

    if (line_found != NULL) {
        *line_found = 0U;
    }

    if (sensor_value == 0U) {
        return 0;
    }

    for (index = 0U; index < 8U; index++) {
        if (((sensor_value >> index) & 0x01U) != 0U) {
            weighted_sum += sensor_weights[index];
            active_count++;
        }
    }

    if (active_count == 0) {
        return 0;
    }

    if (line_found != NULL) {
        *line_found = 1U;
    }

    return (int8_t) (weighted_sum / active_count);
}

void AppLineFollow_SetDebugError(AppLineFollow *line, int8_t error)
{
    if (line == NULL) {
        return;
    }

    line->line_error = error;
}

static float AppLineFollow_ApplyCorrectionSlewLimit(AppLineFollow *line,
                                                    float target_correction_rpm)
{
    uint32_t elapsed_ms;
    float max_delta_rpm;
    float delta_rpm;

    if (line == NULL) {
        return target_correction_rpm;
    }

    elapsed_ms = (uint32_t) (tick_ms - line->last_update_time);
    if (elapsed_ms == 0U) {
        elapsed_ms = 1U;
    }

    if (elapsed_ms > APP_LINE_FOLLOW_CORRECTION_SLEW_MAX_MS) {
        elapsed_ms = APP_LINE_FOLLOW_CORRECTION_SLEW_MAX_MS;
    }

    line->last_update_time = tick_ms;

    max_delta_rpm = APP_LINE_FOLLOW_CORRECTION_SLEW_RPM_PER_MS * (float) elapsed_ms;
    delta_rpm = target_correction_rpm - line->line_correction_rpm;

    if (delta_rpm > max_delta_rpm) {
        delta_rpm = max_delta_rpm;
    } else if (delta_rpm < (-max_delta_rpm)) {
        delta_rpm = -max_delta_rpm;
    }

    line->line_correction_rpm += delta_rpm;
    return line->line_correction_rpm;
}

static float AppLineFollow_MoveToward(float current_value,
                                      float target_value,
                                      float max_delta)
{
    float delta = target_value - current_value;

    if (delta > max_delta) {
        delta = max_delta;
    } else if (delta < (-max_delta)) {
        delta = -max_delta;
    }

    return current_value + delta;
}

static void AppLineFollow_InitTargetRampFromSpeedLoop(AppLineFollow *line)
{
    if (line == NULL) {
        return;
    }

    /*
     * 巡线重新接管时从速度环当前目标开始累加，
     * 避免模式切换后目标速度发生突跳。
     */
    line->smooth_left_target_rpm = MotorSpeedLoop_GetLeftWheelTargetRPM();
    line->smooth_right_target_rpm = MotorSpeedLoop_GetRightWheelTargetRPM();
    line->last_target_update_time = tick_ms;
    line->target_ramp_initialized = 1U;
}

static void AppLineFollow_ApplyTargetSlewLimit(AppLineFollow *line,
                                               float *left_target_rpm,
                                               float *right_target_rpm)
{
    uint32_t elapsed_ms;
    uint32_t target_step_ms;
    float max_delta_rpm;
    uint8_t initialized_now = 0U;

    if ((line == NULL) ||
        (left_target_rpm == NULL) ||
        (right_target_rpm == NULL)) {
        return;
    }

    /*
     * 这里平滑的是“最终左右轮目标速度”，不是巡线误差积分。
     * 理论目标仍由传感器误差直接算出，累加器只负责让速度环设定值慢慢靠近它。
     */
    if (line->target_ramp_initialized == 0U) {
        AppLineFollow_InitTargetRampFromSpeedLoop(line);
        initialized_now = 1U;
    }

    target_step_ms = g_motor_speed_sample_time_ms;
    if (target_step_ms == 0U) {
        target_step_ms = 1U;
    }

    elapsed_ms = (uint32_t) (tick_ms - line->last_target_update_time);
    if ((initialized_now == 0U) && (elapsed_ms < target_step_ms)) {
        *left_target_rpm = line->smooth_left_target_rpm;
        *right_target_rpm = line->smooth_right_target_rpm;
        return;
    }

    line->last_target_update_time = tick_ms;
    max_delta_rpm = APP_LINE_FOLLOW_TARGET_SLEW_RPM_PER_STEP;

    line->smooth_left_target_rpm =
        AppLineFollow_MoveToward(line->smooth_left_target_rpm,
                                 *left_target_rpm,
                                 max_delta_rpm);
    line->smooth_right_target_rpm =
        AppLineFollow_MoveToward(line->smooth_right_target_rpm,
                                 *right_target_rpm,
                                 max_delta_rpm);

    *left_target_rpm = line->smooth_left_target_rpm;
    *right_target_rpm = line->smooth_right_target_rpm;
}

void AppLineFollow_UpdateMotorTargets(AppLineFollow *line, uint8_t sensor_value)
{
    float base_left_target;
    float base_right_target;
    float correction_rpm;
    float left_target_rpm;
    float right_target_rpm;
    uint8_t line_found = 0U;

    if ((line == NULL) || (line->enabled == 0U)) {
        return;
    }

    if ((MotorSpeedLoop_IsLeftWheelEnabled() == 0U) &&
        (MotorSpeedLoop_IsRightWheelEnabled() == 0U)) {
        AppLineFollow_ResetCorrection(line);
        AppLineFollow_ResetTargetRamp(line);
        return;
    }

    Button_GetSpeedTargets(&base_left_target, &base_right_target);
    line->line_error = AppLineFollow_CalculateError(sensor_value, &line_found);

    if (line_found != 0U) {
        line->line_lost_count = 0U;
        if (line->line_error != 0) {
            line->last_line_error = line->line_error;
        }
        correction_rpm = APP_LINE_FOLLOW_CORRECTION_GAIN_RPM *
                         (float) line->line_error;
    } else {
        if (line->line_lost_count < 255U) {
            line->line_lost_count++;
        }

        if (line->line_lost_count <= APP_LINE_FOLLOW_LOST_HOLD_COUNT) {
            correction_rpm = 0.5f * APP_LINE_FOLLOW_CORRECTION_GAIN_RPM *
                             (float) line->last_line_error;
        } else {
            correction_rpm = 0.0f;
        }
    }

    correction_rpm = App_ClampFloat(correction_rpm,
                                    -APP_LINE_FOLLOW_MAX_CORRECTION_RPM,
                                    APP_LINE_FOLLOW_MAX_CORRECTION_RPM);
    correction_rpm = AppLineFollow_ApplyCorrectionSlewLimit(line, correction_rpm);

    left_target_rpm = base_left_target + correction_rpm;
    right_target_rpm = base_right_target - correction_rpm;

    if (line_found == 0U) {
        left_target_rpm *= APP_LINE_FOLLOW_LOST_SPEED_SCALE;
        right_target_rpm *= APP_LINE_FOLLOW_LOST_SPEED_SCALE;
    }

    left_target_rpm = App_ClampFloat(left_target_rpm,
                                     APP_LINE_FOLLOW_MIN_TARGET_RPM,
                                     100.0f);
    right_target_rpm = App_ClampFloat(right_target_rpm,
                                      APP_LINE_FOLLOW_MIN_TARGET_RPM,
                                      100.0f);

    AppLineFollow_ApplyTargetSlewLimit(line, &left_target_rpm, &right_target_rpm);

    MotorSpeedLoop_SetLeftWheelTargetRPM(left_target_rpm);
    MotorSpeedLoop_SetRightWheelTargetRPM(right_target_rpm);
}
