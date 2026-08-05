#include "app_sensor_task.h"

#include <stddef.h>

#include "app_line_follow.h"
#include "Sensor.h"

void AppSensorTask_Init(AppSensorTask *task)
{
    if (task == NULL) {
        return;
    }

    task->value = 0U;
    task->history[0] = 0U;
    task->history[1] = 0U;
    task->history[2] = 0U;
}

void AppSensorTask_Update(AppSensorTask *task)
{
    uint8_t sensor_raw;

    if (task == NULL) {
        return;
    }

    /*
     * 灰度传感器偶尔会有单次跳变，这里只负责采样和滤波；
     * 黑线位置误差的计算仍然交给巡线模块完成。
     */
    sensor_raw = Sensor_Read_Grayscale();
    task->history[2] = task->history[1];
    task->history[1] = task->history[0];
    task->history[0] = sensor_raw;
    task->value = AppLineFollow_FilterSensorValue(task->history[0],
                                                  task->history[1],
                                                  task->history[2]);
}

uint8_t AppSensorTask_GetValue(const AppSensorTask *task)
{
    if (task == NULL) {
        return 0U;
    }

    return task->value;
}
