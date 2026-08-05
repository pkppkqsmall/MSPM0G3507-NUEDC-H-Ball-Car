#ifndef APP_ANGLE_TEST_H_
#define APP_ANGLE_TEST_H_

#include <stdint.h>
#include "app_angle_loop.h"

typedef struct {
    uint8_t active;
    uint8_t exit_confirm_count;
    unsigned long start_time;
    float start_yaw;
    float target_deg;
    AppAngleLoop loop;
} AppAngleTest;

typedef enum {
    APP_ANGLE_TEST_COMMAND_NONE = 0,
    APP_ANGLE_TEST_COMMAND_QUERY,
    APP_ANGLE_TEST_COMMAND_START,
    APP_ANGLE_TEST_COMMAND_ERROR
} AppAngleTestCommand;

void AppAngleTest_Init(AppAngleTest *test);
void AppAngleTest_Reset(AppAngleTest *test);
uint8_t AppAngleTest_IsActive(const AppAngleTest *test);
float AppAngleTest_GetTargetDeg(const AppAngleTest *test);
float AppAngleTest_GetLastErrorDeg(const AppAngleTest *test);

/* 解析 ANGLE 命令。正角度正向旋转，负角度反向旋转。 */
AppAngleTestCommand AppAngleTest_ParseCommand(const char *command, float *target_deg);

/* 从当前 yaw 开始执行一次相对角度测试。 */
void AppAngleTest_Start(AppAngleTest *test,
                        float relative_target_deg,
                        float current_yaw_deg,
                        uint32_t current_time_ms);

/*
 * 更新角度测试，并返回本周期左右轮目标速度。
 * 返回 1 表示角度到达或超时，需要上层停车并复位。
 */
uint8_t AppAngleTest_Update(AppAngleTest *test,
                            float current_yaw_deg,
                            uint32_t current_time_ms,
                            float *left_target_rpm,
                            float *right_target_rpm,
                            uint8_t *timeout);

#endif
