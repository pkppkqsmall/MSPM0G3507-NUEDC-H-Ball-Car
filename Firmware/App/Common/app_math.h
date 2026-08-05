#ifndef APP_COMMON_APP_MATH_H_
#define APP_COMMON_APP_MATH_H_

#include <stdint.h>

/* App 层常用的小数学工具放在头文件里，避免每个模块重复写一遍。 */
static inline float App_AbsFloat(float value)
{
    return (value < 0.0f) ? (-value) : value;
}

static inline float App_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static inline int8_t App_GetFloatDirection(float value)
{
    if (value > 0.0f) {
        return 1;
    }

    if (value < 0.0f) {
        return -1;
    }

    return 0;
}

#endif
