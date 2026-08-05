#ifndef APP_ANGLE_LOOP_H_
#define APP_ANGLE_LOOP_H_

#include <stdint.h>

typedef struct {
    float target_yaw_deg;
    float last_error_deg;
    float integral_deg_sec;
    float last_output_rpm;
    int8_t target_direction;       /* 1 表示正向旋转，-1 表示反向旋转，0 表示没有有效目标。 */
    uint8_t active;
    uint8_t first_update;
    uint32_t last_update_time_ms;
} AppAngleLoop;

/*
 * 初始化角度环上下文。
 * 角度环只负责把 yaw 误差换算成“转弯轮目标 RPM”，底层 PWM 仍由速度环完成。
 */
void AppAngleLoop_Init(AppAngleLoop *loop);

/* 清空角度环状态，通常在停车、测试结束、重新起跑时调用。 */
void AppAngleLoop_Reset(AppAngleLoop *loop);

/*
 * 开始一次相对角度控制。
 * current_yaw_deg：当前 yaw。
 * relative_target_deg：相对转角，正向旋转为正，反向旋转为负。
 * current_time_ms：当前系统 tick，单位 ms。
 */
void AppAngleLoop_StartRelative(AppAngleLoop *loop,
                                float current_yaw_deg,
                                float relative_target_deg,
                                uint32_t current_time_ms);

/*
 * 更新角度环，返回本周期建议的转弯轮目标 RPM。
 * 返回值为正表示正向旋转目标，返回值为负表示反向旋转目标。
 */
float AppAngleLoop_Update(AppAngleLoop *loop,
                          float current_yaw_deg,
                          uint32_t current_time_ms);

/* 判断角度是否已经到达目标；内部已处理 180/-180 度 yaw 溢出。 */
uint8_t AppAngleLoop_IsTargetReached(const AppAngleLoop *loop);

/* 读取最近一次 yaw 误差，方便调试或 OLED/串口显示。 */
float AppAngleLoop_GetLastErrorDeg(const AppAngleLoop *loop);

/* 把 yaw 统一归一化到 [-180, 180]，避免跨越 180 度时误判方向。 */
float AppAngleLoop_NormalizeYawDeg(float yaw_deg);

#endif
