#include "map_yaw_rate_controller.h"

#include <stddef.h>

/*
 * 约定：BNO085 左转 yaw/角速度为正，turn_rpm 为正时车辆左转。
 * 若实车手动向左旋转时测得角速度为负，应先修正 BNO085 的方向系数。
 */
#define MAP_YAW_LINE_TO_RATE_GAIN_DEG_S          (5.0f)
#define MAP_YAW_TARGET_RATE_LIMIT_DEG_S          (60.0f)
#define MAP_YAW_RATE_FILTER_ALPHA                (0.35f)
#define MAP_YAW_RATE_FEEDFORWARD_RPM_PER_DEG_S   (0.55f)
#define MAP_YAW_RATE_KP_RPM_PER_DEG_S            (0.15f)
#define MAP_YAW_TURN_LIMIT_RPM                   (40.0f)
#define MAP_YAW_SAMPLE_MAX_GAP_MS                (100UL)
#define MAP_YAW_SAMPLE_STALE_MS                  (150UL)
#define MAP_YAW_MAX_VALID_RATE_DEG_S             (360.0f)

static float MapYawRateController_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float MapYawRateController_Clamp(float value,
                                        float minimum,
                                        float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float MapYawRateController_NormalizeDelta(float delta_deg)
{
    while (delta_deg > 180.0f) {
        delta_deg -= 360.0f;
    }
    while (delta_deg < -180.0f) {
        delta_deg += 360.0f;
    }
    return delta_deg;
}

void MapYawRateController_Reset(MapYawRateController *controller)
{
    if (controller == NULL) {
        return;
    }

    controller->previous_yaw_deg = 0.0f;
    controller->measured_yaw_rate_deg_s = 0.0f;
    controller->target_yaw_rate_deg_s = 0.0f;
    controller->turn_rpm = 0.0f;
    controller->previous_sample_ms = 0UL;
    controller->last_sample_ms = 0UL;
    controller->has_previous_sample = 0U;
    controller->rate_valid = 0U;
}

void MapYawRateController_AddYawSample(
    MapYawRateController *controller,
    float yaw_deg,
    uint32_t sample_ms)
{
    uint32_t elapsed_ms;
    float yaw_delta_deg;
    float raw_yaw_rate_deg_s;

    if (controller == NULL) {
        return;
    }

    controller->last_sample_ms = sample_ms;

    if (controller->has_previous_sample == 0U) {
        controller->previous_yaw_deg = yaw_deg;
        controller->previous_sample_ms = sample_ms;
        controller->has_previous_sample = 1U;
        controller->rate_valid = 0U;
        return;
    }

    elapsed_ms = sample_ms - controller->previous_sample_ms;
    if (elapsed_ms == 0UL) {
        /* 同一毫秒内的重复服务不参与求导，等待下一次有效时间间隔。 */
        return;
    }

    if (elapsed_ms > MAP_YAW_SAMPLE_MAX_GAP_MS) {
        controller->previous_yaw_deg = yaw_deg;
        controller->previous_sample_ms = sample_ms;
        controller->measured_yaw_rate_deg_s = 0.0f;
        controller->rate_valid = 0U;
        return;
    }

    yaw_delta_deg =
        MapYawRateController_NormalizeDelta(
            yaw_deg - controller->previous_yaw_deg);
    raw_yaw_rate_deg_s =
        yaw_delta_deg * 1000.0f / (float) elapsed_ms;

    controller->previous_yaw_deg = yaw_deg;
    controller->previous_sample_ms = sample_ms;

    /*
     * 传感器复位或四元数重建可能造成单帧跳变。丢弃不可能的角速度，
     * 避免错误样本瞬间给出最大反向差速。
     */
    if (MapYawRateController_Abs(raw_yaw_rate_deg_s) >
        MAP_YAW_MAX_VALID_RATE_DEG_S) {
        controller->measured_yaw_rate_deg_s = 0.0f;
        controller->rate_valid = 0U;
        return;
    }

    if (controller->rate_valid == 0U) {
        controller->measured_yaw_rate_deg_s =
            raw_yaw_rate_deg_s;
    } else {
        controller->measured_yaw_rate_deg_s +=
            MAP_YAW_RATE_FILTER_ALPHA *
            (raw_yaw_rate_deg_s -
             controller->measured_yaw_rate_deg_s);
    }
    controller->rate_valid = 1U;
}

uint8_t MapYawRateController_Update(
    MapYawRateController *controller,
    float line_error,
    float target_yaw_rate_offset_deg_s,
    uint32_t now_ms,
    uint8_t enable_limits)
{
    float yaw_rate_error_deg_s;

    if (controller == NULL) {
        return 0U;
    }

    /*
     * 灰度误差为正表示黑线在车头右侧，因此目标应为右转负角速度。
     * M1/Q2 可关闭两层限幅以获得更强的外侧纠偏，其它题目继续保护。
     */
    controller->target_yaw_rate_deg_s =
        (-MAP_YAW_LINE_TO_RATE_GAIN_DEG_S * line_error) +
        target_yaw_rate_offset_deg_s;
    if (enable_limits != 0U) {
        controller->target_yaw_rate_deg_s =
            MapYawRateController_Clamp(
                controller->target_yaw_rate_deg_s,
                -MAP_YAW_TARGET_RATE_LIMIT_DEG_S,
                MAP_YAW_TARGET_RATE_LIMIT_DEG_S);
    }

    if (MapYawRateController_IsFresh(controller, now_ms) == 0U) {
        controller->measured_yaw_rate_deg_s = 0.0f;
        controller->turn_rpm = 0.0f;
        controller->rate_valid = 0U;
        return 0U;
    }

    yaw_rate_error_deg_s =
        controller->target_yaw_rate_deg_s -
        controller->measured_yaw_rate_deg_s;
    controller->turn_rpm =
        (MAP_YAW_RATE_FEEDFORWARD_RPM_PER_DEG_S *
         controller->target_yaw_rate_deg_s) +
        (MAP_YAW_RATE_KP_RPM_PER_DEG_S *
         yaw_rate_error_deg_s);
    if (enable_limits != 0U) {
        controller->turn_rpm =
            MapYawRateController_Clamp(controller->turn_rpm,
                                       -MAP_YAW_TURN_LIMIT_RPM,
                                       MAP_YAW_TURN_LIMIT_RPM);
    }
    return 1U;
}

uint8_t MapYawRateController_IsFresh(
    const MapYawRateController *controller,
    uint32_t now_ms)
{
    if ((controller == NULL) ||
        (controller->rate_valid == 0U)) {
        return 0U;
    }

    return ((now_ms - controller->last_sample_ms) <=
            MAP_YAW_SAMPLE_STALE_MS) ? 1U : 0U;
}

float MapYawRateController_GetTargetYawRate(
    const MapYawRateController *controller)
{
    return (controller == NULL) ?
        0.0f : controller->target_yaw_rate_deg_s;
}

float MapYawRateController_GetMeasuredYawRate(
    const MapYawRateController *controller)
{
    return (controller == NULL) ?
        0.0f : controller->measured_yaw_rate_deg_s;
}

float MapYawRateController_GetTurnRpm(
    const MapYawRateController *controller)
{
    return (controller == NULL) ? 0.0f : controller->turn_rpm;
}
