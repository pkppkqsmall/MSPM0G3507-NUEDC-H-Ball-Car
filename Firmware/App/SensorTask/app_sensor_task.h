#ifndef APP_SENSOR_TASK_H_
#define APP_SENSOR_TASK_H_

#include <stdint.h>

typedef struct {
    uint8_t value;       /* 三次多数表决后的灰度值，1 表示黑线，0 表示白底。 */
    uint8_t history[3];  /* 最近三次原始采样，用于降低传感器瞬时抖动。 */
} AppSensorTask;

/* 初始化灰度传感器任务的历史缓存和输出值。 */
void AppSensorTask_Init(AppSensorTask *task);

/* 读取 8 路灰度传感器，并更新滤波后的灰度值。 */
void AppSensorTask_Update(AppSensorTask *task);

/* 获取当前滤波后的灰度值。 */
uint8_t AppSensorTask_GetValue(const AppSensorTask *task);

#endif /* APP_SENSOR_TASK_H_ */
