#ifndef MAP_CURVE_HOLD_H_
#define MAP_CURVE_HOLD_H_

#include <stdint.h>

typedef struct {
    float previous_yaw_deg;
    float right_turn_deg;
    float exit_line_blend;
    uint8_t has_previous_yaw;
    uint8_t active;
    uint8_t aligning;
    uint8_t completed_curve_count;
    uint8_t confirm_count;
    uint8_t align_step;
} MapCurveHold;

/* 清除弯道累计角度和出弯对正状态，每次地图起跑前调用。 */
void MapCurveHold_Reset(MapCurveHold *controller);

/* 输入 BNO085 的新 yaw 样本，内部处理 +180/-180 度跨界。 */
void MapCurveHold_AddYawSample(MapCurveHold *controller,
                               float yaw_deg);

/*
 * 根据编码器路段、灰度位图和 BNO085 状态更新弯道保持状态。
 * 该功能只供已知赛道的两个顺时针半圆使用。
 */
void MapCurveHold_Update(MapCurveHold *controller,
                         uint32_t distance_counts,
                         uint8_t sensor_value,
                         float current_yaw_deg,
                         uint8_t yaw_fresh,
                         float measured_yaw_rate_deg_s);

/* 在弯道保持或出弯对正期间修正送入角速度环的灰度误差。 */
float MapCurveHold_ApplyLineError(
    const MapCurveHold *controller,
    float line_error);

uint8_t MapCurveHold_IsActive(const MapCurveHold *controller);
uint8_t MapCurveHold_IsAligning(const MapCurveHold *controller);
uint8_t MapCurveHold_GetCompletedCount(
    const MapCurveHold *controller);
float MapCurveHold_GetRightTurnDeg(
    const MapCurveHold *controller);
float MapCurveHold_GetExitLineBlend(
    const MapCurveHold *controller);

#endif /* MAP_CURVE_HOLD_H_ */
