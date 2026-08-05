#ifndef MAP_LINE_CONTROLLER_H_
#define MAP_LINE_CONTROLLER_H_

#include <stdint.h>

typedef struct {
    float last_error;
    float correction_rpm;
} MapLineController;

typedef struct {
    float error;
    float correction_rpm;
    uint8_t active_count;
    uint8_t line_found;
} MapLineResult;

/* 清除灰度误差和 P 修正，供每次起跑前重新初始化。 */
void MapLineController_Reset(MapLineController *controller);

/*
 * 直接使用当前一帧灰度位图计算位置，并输出纯 P 修正。
 * bit0 为车头最左侧，bit7 为车头最右侧，黑线为 1。
 */
void MapLineController_Update(MapLineController *controller,
                              uint8_t sensor_value,
                              MapLineResult *result);

/* 统计当前位图中检测到黑线的探头数量。 */
uint8_t MapLineController_CountActive(uint8_t sensor_value);

/* 由指定误差计算灰度 P 回退修正，供题目专用误差整形后复用。 */
float MapLineController_CalculateCorrectionRpm(float error);

#endif /* MAP_LINE_CONTROLLER_H_ */
