#ifndef MAP_YAW_RATE_CONTROLLER_H_
#define MAP_YAW_RATE_CONTROLLER_H_

#include <stdint.h>

typedef struct {
    float previous_yaw_deg;
    float measured_yaw_rate_deg_s;
    float target_yaw_rate_deg_s;
    float turn_rpm;
    uint32_t previous_sample_ms;
    uint32_t last_sample_ms;
    uint8_t has_previous_sample;
    uint8_t rate_valid;
} MapYawRateController;

/* 清除角速度估计和串级控制状态，供初始化及每次起跑前调用。 */
void MapYawRateController_Reset(MapYawRateController *controller);

/*
 * 仅在 BNO085 确认产生新 yaw 样本时调用。
 * 函数内部处理 +180/-180 度跨界，并根据相邻样本估算角速度。
 */
void MapYawRateController_AddYawSample(
    MapYawRateController *controller,
    float yaw_deg,
    uint32_t sample_ms);

/*
 * 灰度误差外环生成目标角速度，角速度内环生成左右轮差速量。
 * target_yaw_rate_offset_deg_s 可叠加航向角外环给出的角速度修正。
 * enable_limits 为 0 时不限制目标角速度和 turn_rpm，供 M1/Q2 使用；
 * 其它题目保持限幅，避免影响已经完成实车调试的带球模式。
 * 返回 1 表示 BNO085 数据新鲜、串级结果可用；返回 0 时上层应回退。
 */
uint8_t MapYawRateController_Update(
    MapYawRateController *controller,
    float line_error,
    float target_yaw_rate_offset_deg_s,
    uint32_t now_ms,
    uint8_t enable_limits);

uint8_t MapYawRateController_IsFresh(
    const MapYawRateController *controller,
    uint32_t now_ms);
float MapYawRateController_GetTargetYawRate(
    const MapYawRateController *controller);
float MapYawRateController_GetMeasuredYawRate(
    const MapYawRateController *controller);
float MapYawRateController_GetTurnRpm(
    const MapYawRateController *controller);

#endif /* MAP_YAW_RATE_CONTROLLER_H_ */
