#ifndef APP_OLED_VIEW_H_
#define APP_OLED_VIEW_H_

#include <stdint.h>

/*
 * 刷新 OLED 状态页；refresh_requested 非 0 时会先清屏。
 */
void AppOledView_Update(uint8_t sensor_value,
                        uint8_t *refresh_requested,
                        unsigned long *last_display_time);

#endif
