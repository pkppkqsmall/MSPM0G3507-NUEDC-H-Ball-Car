#include "app_oled_view.h"
#include <stddef.h>
#include <stdio.h>
#include "clock.h"
#include "mpu6050.h"
#include "motor_pwm.h"
#include "oled_hardware_i2c.h"

static uint8_t g_oled_view_buffer[32];

static void AppOledView_FormatSensorBits(uint8_t sensor_value, uint8_t *buffer)
{
    uint8_t index;

    if (buffer == NULL) {
        return;
    }

    /* OLED 上按 8 -> 1 号传感器顺序显示，方便和车头实际左右方向对应。 */
    buffer[0] = 'g';
    buffer[1] = 'r';
    buffer[2] = 'a';
    buffer[3] = 'y';
    buffer[4] = ':';

    for (index = 0U; index < 8U; index++) {
        buffer[5U + index] =
            (((sensor_value >> (7U - index)) & 0x01U) != 0U) ? '1' : '0';
    }

    buffer[13] = '\0';
}

void AppOledView_Update(uint8_t sensor_value,
                        uint8_t *refresh_requested,
                        unsigned long *last_display_time)
{
    if ((refresh_requested == NULL) || (last_display_time == NULL)) {
        return;
    }

    /*
     * OLED 刷新比控制环慢很多，100ms 更新一次就够观察。
     * 这样可以减少 I2C/OLED 刷新占用，避免影响主循环实时性。
     */
    if ((tick_ms - (*last_display_time)) < 100U) {
        return;
    }

    *last_display_time = tick_ms;

    if (*refresh_requested != 0U) {
        OLED_Clear();
        *refresh_requested = 0U;
    }

    snprintf((char *) g_oled_view_buffer,
             sizeof(g_oled_view_buffer),
             "yaw:%+6.1f        ",
             yaw);
    OLED_ShowString(0, 0, g_oled_view_buffer, 8);

    AppOledView_FormatSensorBits(sensor_value, g_oled_view_buffer);
    OLED_ShowString(0, 2, g_oled_view_buffer, 8);

    /*
     * 显示“前进方向为正”的左右编码器累计计数。
     * 向前推车时左右应同号变化，比原始计数更直观。
     */
    snprintf((char *) g_oled_view_buffer,
             sizeof(g_oled_view_buffer),
             "encL:%ld        ",
             (long) MotorSpeed_GetLeftEncoderForwardCount());
    OLED_ShowString(0, 4, g_oled_view_buffer, 8);

    snprintf((char *) g_oled_view_buffer,
             sizeof(g_oled_view_buffer),
             "encR:%ld        ",
             (long) MotorSpeed_GetRightEncoderForwardCount());
    OLED_ShowString(0, 6, g_oled_view_buffer, 8);
}
