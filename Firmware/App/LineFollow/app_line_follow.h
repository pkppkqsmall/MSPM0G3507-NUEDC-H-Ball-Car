#ifndef APP_LINE_FOLLOW_H_
#define APP_LINE_FOLLOW_H_

#include <stdint.h>

typedef struct {
    int8_t line_error;          /* 当前巡线误差：正数表示黑线偏右，负数表示黑线偏左。 */
    int8_t last_line_error;     /* 最近一次非 0 有效误差，用于短暂丢线时继续按原方向找线。 */
    float line_correction_rpm;  /* 已经经过斜率限制后的修正速度，单位 RPM。 */
    float smooth_left_target_rpm;   /* 累加平滑后实际下发给左轮速度环的目标 RPM。 */
    float smooth_right_target_rpm;  /* 累加平滑后实际下发给右轮速度环的目标 RPM。 */
    unsigned long last_update_time;
    unsigned long last_target_update_time;
    uint8_t line_lost_count;
    uint8_t target_ramp_initialized;
    uint8_t enabled;
} AppLineFollow;

/* 初始化巡线模块运行状态。 */
void AppLineFollow_Init(AppLineFollow *line);

/* 清空误差、丢线计数和修正量，常用于停车或重新起跑。 */
void AppLineFollow_ResetRuntime(AppLineFollow *line);

/* 只清空修正量斜率限制状态，避免上一阶段的修正直接带入下一阶段。 */
void AppLineFollow_ResetCorrection(AppLineFollow *line);

/* 清空目标速度累加器，下次巡线接管时会从当前速度环目标重新开始。 */
void AppLineFollow_ResetTargetRamp(AppLineFollow *line);

/* 对连续三次灰度采样做多数表决滤波，降低单次传感器抖动影响。 */
uint8_t AppLineFollow_FilterSensorValue(uint8_t newest,
                                        uint8_t history1,
                                        uint8_t history2);

/* 根据 8 路灰度值计算黑线偏移误差；line_found 返回是否看到了黑线。 */
int8_t AppLineFollow_CalculateError(uint8_t sensor_value, uint8_t *line_found);

/* 角度测试阶段写入调试误差，方便后续显示或串口观察。 */
void AppLineFollow_SetDebugError(AppLineFollow *line, int8_t error);

/* 按当前灰度值更新左右轮巡线目标速度。 */
void AppLineFollow_UpdateMotorTargets(AppLineFollow *line, uint8_t sensor_value);

#endif
