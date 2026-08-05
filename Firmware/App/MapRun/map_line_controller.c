#include "map_line_controller.h"

#include <stddef.h>

#define MAP_LINE_KP_RPM                          (2.50f)

uint8_t MapLineController_CountActive(uint8_t sensor_value)
{
    uint8_t count = 0U;

    while (sensor_value != 0U) {
        count += (uint8_t) (sensor_value & 0x01U);
        sensor_value >>= 1U;
    }

    return count;
}

static float MapLineController_CalculateRawError(uint8_t sensor_value,
                                                 uint8_t active_count)
{
    /*
     * 中间探头只做小幅修正，外侧探头使用递增的非线性权重。
     * 黑线靠近边缘时需要更强回线，避免高速下还没修正就冲出赛道。
     * 黑线偏右为正，上层会降低右轮目标，使车辆向右回线。
     */
    static const float sensor_positions[8] = {
        -16.0f, -8.0f, -4.0f, -2.0f,
          2.0f,  4.0f,  8.0f, 16.0f
    };
    float position_sum = 0.0f;
    uint8_t index;

    if (active_count == 0U) {
        return 0.0f;
    }

    for (index = 0U; index < 8U; index++) {
        if (((sensor_value >> index) & 0x01U) != 0U) {
            position_sum += sensor_positions[index];
        }
    }

    return position_sum / (float) active_count;
}

void MapLineController_Reset(MapLineController *controller)
{
    if (controller == NULL) {
        return;
    }

    controller->last_error = 0.0f;
    controller->correction_rpm = 0.0f;
}

void MapLineController_Update(MapLineController *controller,
                              uint8_t sensor_value,
                              MapLineResult *result)
{
    float raw_error;

    if ((controller == NULL) || (result == NULL)) {
        return;
    }

    result->active_count =
        MapLineController_CountActive(sensor_value);
    result->line_found = (result->active_count != 0U) ? 1U : 0U;

    /*
     * 全白时保留最后一次有效误差和 P 修正。
     * 上层同时保持基础速度，使左右轮延续丢线前的运动状态。
     */
    if (result->line_found == 0U) {
        result->error = controller->last_error;
        result->correction_rpm = controller->correction_rpm;
        return;
    }

    raw_error =
        MapLineController_CalculateRawError(sensor_value,
                                            result->active_count);

    controller->last_error = raw_error;
    controller->correction_rpm =
        MapLineController_CalculateCorrectionRpm(raw_error);

    result->error = controller->last_error;
    result->correction_rpm = controller->correction_rpm;
}

float MapLineController_CalculateCorrectionRpm(float error)
{
    return MAP_LINE_KP_RPM * error;
}
