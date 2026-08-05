#include "map_curve_hold.h"

#include <stddef.h>

/*
 * 第一、第二个右弯分别在 B、D 点附近。编码器只负责限制触发路段，
 * 最外两路可在宽窗口内触发；右侧第三路只在更靠近弯道的位置生效。
 * 这样可以比原逻辑提前建立右转，同时避免直线轻微摆动过早锁定半圆。
 */
#define MAP_CURVE_FIRST_ARM_COUNTS              (4300UL)
#define MAP_CURVE_FIRST_END_COUNTS             (12000UL)
#define MAP_CURVE_SECOND_ARM_COUNTS            (15000UL)
#define MAP_CURVE_SECOND_END_COUNTS            (22500UL)
#define MAP_CURVE_FIRST_EARLY_ARM_COUNTS         (4600UL)
#define MAP_CURVE_SECOND_EARLY_ARM_COUNTS       (15300UL)

#define MAP_CURVE_RIGHT_STRONG_ENTRY_MASK         (0xC0U)
#define MAP_CURVE_RIGHT_EARLY_ENTRY_MASK          (0x20U)
#define MAP_CURVE_ENTRY_CONFIRM_COUNT               (2U)
#define MAP_CURVE_CENTER_MASK                     (0x18U)
#define MAP_CURVE_CENTER_CONFIRM_COUNT              (3U)

/*
 * 半径约 0.5 m、75 RPM 时理论车身角速度约为 29 deg/s。
 * 用等效灰度误差 6 生成最低 30 deg/s 右转目标，防止灰度回中后过早直行。
 */
#define MAP_CURVE_MIN_RIGHT_LINE_ERROR             (6.0f)
#define MAP_CURVE_EXIT_MIN_LINE_ERROR               (2.0f)
#define MAP_CURVE_EXIT_TAPER_START_DEG             (170.0f)
#define MAP_CURVE_EXIT_MIN_RIGHT_TURN_DEG         (175.0f)
#define MAP_CURVE_FORCE_RELEASE_RIGHT_TURN_DEG     (190.0f)
#define MAP_CURVE_ALIGN_MAX_YAW_RATE_DEG_S           (5.0f)
#define MAP_CURVE_MAX_SAMPLE_DELTA_DEG              (45.0f)

/* MapCurveHold_Update() 每 10 ms 调用：先消旋 50 ms，再渐入灰度 200 ms。 */
#define MAP_CURVE_ALIGN_ZERO_STEPS                    (5U)
#define MAP_CURVE_ALIGN_BLEND_STEPS                  (20U)
#define MAP_CURVE_ALIGN_MAX_STEPS                    (40U)

static float MapCurveHold_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float MapCurveHold_NormalizeDelta(float delta_deg)
{
    while (delta_deg > 180.0f) {
        delta_deg -= 360.0f;
    }
    while (delta_deg < -180.0f) {
        delta_deg += 360.0f;
    }
    return delta_deg;
}

static uint8_t MapCurveHold_IsCenterDetected(uint8_t sensor_value)
{
    return ((sensor_value & MAP_CURVE_CENTER_MASK) != 0U) ?
        1U : 0U;
}

static uint8_t MapCurveHold_IsEntryWindow(
    const MapCurveHold *controller,
    uint32_t distance_counts)
{
    if (controller->completed_curve_count == 0U) {
        return ((distance_counts >= MAP_CURVE_FIRST_ARM_COUNTS) &&
                (distance_counts <= MAP_CURVE_FIRST_END_COUNTS)) ?
            1U : 0U;
    }

    if (controller->completed_curve_count == 1U) {
        return ((distance_counts >= MAP_CURVE_SECOND_ARM_COUNTS) &&
                (distance_counts <= MAP_CURVE_SECOND_END_COUNTS)) ?
            1U : 0U;
    }

    return 0U;
}

static uint8_t MapCurveHold_IsEntryDetected(
    const MapCurveHold *controller,
    uint32_t distance_counts,
    uint8_t sensor_value)
{
    if (MapCurveHold_IsEntryWindow(controller,
                                   distance_counts) == 0U) {
        return 0U;
    }

    if ((sensor_value & MAP_CURVE_RIGHT_STRONG_ENTRY_MASK) != 0U) {
        return 1U;
    }

    if ((sensor_value & MAP_CURVE_RIGHT_EARLY_ENTRY_MASK) == 0U) {
        return 0U;
    }

    if (controller->completed_curve_count == 0U) {
        return (distance_counts >=
                MAP_CURVE_FIRST_EARLY_ARM_COUNTS) ? 1U : 0U;
    }
    if (controller->completed_curve_count == 1U) {
        return (distance_counts >=
                MAP_CURVE_SECOND_EARLY_ARM_COUNTS) ? 1U : 0U;
    }

    return 0U;
}

static float MapCurveHold_GetMinimumRightLineError(
    const MapCurveHold *controller)
{
    float taper_progress;

    if (controller->right_turn_deg <=
        MAP_CURVE_EXIT_TAPER_START_DEG) {
        return MAP_CURVE_MIN_RIGHT_LINE_ERROR;
    }
    if (controller->right_turn_deg >=
        MAP_CURVE_EXIT_MIN_RIGHT_TURN_DEG) {
        return MAP_CURVE_EXIT_MIN_LINE_ERROR;
    }

    taper_progress =
        (controller->right_turn_deg -
         MAP_CURVE_EXIT_TAPER_START_DEG) /
        (MAP_CURVE_EXIT_MIN_RIGHT_TURN_DEG -
         MAP_CURVE_EXIT_TAPER_START_DEG);
    return MAP_CURVE_MIN_RIGHT_LINE_ERROR -
           (taper_progress *
            (MAP_CURVE_MIN_RIGHT_LINE_ERROR -
             MAP_CURVE_EXIT_MIN_LINE_ERROR));
}

static void MapCurveHold_Enter(MapCurveHold *controller,
                               float current_yaw_deg)
{
    controller->previous_yaw_deg = current_yaw_deg;
    controller->right_turn_deg = 0.0f;
    controller->exit_line_blend = 1.0f;
    controller->has_previous_yaw = 1U;
    controller->active = 1U;
    controller->aligning = 0U;
    controller->confirm_count = 0U;
    controller->align_step = 0U;
}

static void MapCurveHold_Finish(MapCurveHold *controller,
                                uint8_t start_aligning)
{
    controller->active = 0U;
    controller->aligning = start_aligning;
    controller->exit_line_blend =
        (start_aligning != 0U) ? 0.0f : 1.0f;
    controller->has_previous_yaw = 0U;
    controller->confirm_count = 0U;
    controller->align_step = 0U;
    if (controller->completed_curve_count < 2U) {
        controller->completed_curve_count++;
    }
}

void MapCurveHold_Reset(MapCurveHold *controller)
{
    if (controller == NULL) {
        return;
    }

    controller->previous_yaw_deg = 0.0f;
    controller->right_turn_deg = 0.0f;
    controller->exit_line_blend = 1.0f;
    controller->has_previous_yaw = 0U;
    controller->active = 0U;
    controller->aligning = 0U;
    controller->completed_curve_count = 0U;
    controller->confirm_count = 0U;
    controller->align_step = 0U;
}

void MapCurveHold_AddYawSample(MapCurveHold *controller,
                               float yaw_deg)
{
    float yaw_delta_deg;

    if ((controller == NULL) || (controller->active == 0U)) {
        return;
    }

    if (controller->has_previous_yaw == 0U) {
        controller->previous_yaw_deg = yaw_deg;
        controller->has_previous_yaw = 1U;
        return;
    }

    yaw_delta_deg =
        MapCurveHold_NormalizeDelta(
            yaw_deg - controller->previous_yaw_deg);
    controller->previous_yaw_deg = yaw_deg;

    /* BNO085 复位产生的大角度跳变不参与累计。 */
    if (MapCurveHold_Abs(yaw_delta_deg) >
        MAP_CURVE_MAX_SAMPLE_DELTA_DEG) {
        return;
    }

    /*
     * 弯道窗口只覆盖两个右半圆，因此这里累计实际转动量，不依赖
     * BNO085 安装后的 yaw 正负方向，避免转角被 0 下限持续夹住。
     */
    controller->right_turn_deg += MapCurveHold_Abs(yaw_delta_deg);
    if (controller->right_turn_deg > 360.0f) {
        controller->right_turn_deg = 360.0f;
    }
}

void MapCurveHold_Update(MapCurveHold *controller,
                         uint32_t distance_counts,
                         uint8_t sensor_value,
                         float current_yaw_deg,
                         uint8_t yaw_fresh,
                         float measured_yaw_rate_deg_s)
{
    uint8_t center_detected;

    if (controller == NULL) {
        return;
    }

    center_detected =
        MapCurveHold_IsCenterDetected(sensor_value);

    /*
     * 若第一段弯道完全没有触发，越过其窗口后仍允许第二段独立工作。
     * 这只更新路段编号，不会凭编码器直接强制车辆转向。
     */
    if ((controller->active == 0U) &&
        (controller->aligning == 0U) &&
        (controller->completed_curve_count == 0U) &&
        (distance_counts > MAP_CURVE_FIRST_END_COUNTS)) {
        controller->completed_curve_count = 1U;
    }

    if (controller->active != 0U) {
        /* BNO085 长时间失效时立即取消保持，回退到原灰度巡线。 */
        if (yaw_fresh == 0U) {
            controller->active = 0U;
            controller->exit_line_blend = 1.0f;
            controller->has_previous_yaw = 0U;
            controller->confirm_count = 0U;
            controller->align_step = 0U;
            return;
        }

        if ((controller->right_turn_deg >=
             MAP_CURVE_EXIT_MIN_RIGHT_TURN_DEG) &&
            (center_detected != 0U)) {
            if (controller->confirm_count <
                MAP_CURVE_CENTER_CONFIRM_COUNT) {
                controller->confirm_count++;
            }
        } else {
            controller->confirm_count = 0U;
        }

        if (controller->confirm_count >=
            MAP_CURVE_CENTER_CONFIRM_COUNT) {
            MapCurveHold_Finish(controller, 1U);
        } else if (controller->right_turn_deg >=
                   MAP_CURVE_FORCE_RELEASE_RIGHT_TURN_DEG) {
            /*
             * 即使中线漏检，达到 190 度也必须释放最小右转约束，
             * 避免车辆因单个传感器故障持续绕圈。
             */
            MapCurveHold_Finish(controller, 1U);
        }
        return;
    }

    if (controller->aligning != 0U) {
        if (yaw_fresh == 0U) {
            controller->aligning = 0U;
            controller->exit_line_blend = 1.0f;
            controller->confirm_count = 0U;
            controller->align_step = 0U;
            return;
        }

        if (controller->align_step < MAP_CURVE_ALIGN_MAX_STEPS) {
            controller->align_step++;
        }

        if (controller->align_step <= MAP_CURVE_ALIGN_ZERO_STEPS) {
            controller->exit_line_blend = 0.0f;
        } else if (controller->align_step <
                   (MAP_CURVE_ALIGN_ZERO_STEPS +
                    MAP_CURVE_ALIGN_BLEND_STEPS)) {
            controller->exit_line_blend =
                (float) (controller->align_step -
                         MAP_CURVE_ALIGN_ZERO_STEPS) /
                (float) MAP_CURVE_ALIGN_BLEND_STEPS;
        } else {
            controller->exit_line_blend = 1.0f;
        }

        if ((controller->exit_line_blend >= 1.0f) &&
            (center_detected != 0U) &&
            (MapCurveHold_Abs(measured_yaw_rate_deg_s) <=
             MAP_CURVE_ALIGN_MAX_YAW_RATE_DEG_S)) {
            if (controller->confirm_count <
                MAP_CURVE_CENTER_CONFIRM_COUNT) {
                controller->confirm_count++;
            }
        } else {
            controller->confirm_count = 0U;
        }

        if ((controller->confirm_count >=
             MAP_CURVE_CENTER_CONFIRM_COUNT) ||
            (controller->align_step >= MAP_CURVE_ALIGN_MAX_STEPS)) {
            controller->aligning = 0U;
            controller->exit_line_blend = 1.0f;
            controller->confirm_count = 0U;
            controller->align_step = 0U;
        }
        return;
    }

    if ((yaw_fresh != 0U) &&
        (MapCurveHold_IsEntryDetected(controller,
                                      distance_counts,
                                      sensor_value) != 0U)) {
        if (controller->confirm_count <
            MAP_CURVE_ENTRY_CONFIRM_COUNT) {
            controller->confirm_count++;
        }
        if (controller->confirm_count >=
            MAP_CURVE_ENTRY_CONFIRM_COUNT) {
            MapCurveHold_Enter(controller, current_yaw_deg);
        }
    } else {
        controller->confirm_count = 0U;
    }
}

float MapCurveHold_ApplyLineError(
    const MapCurveHold *controller,
    float line_error)
{
    float minimum_right_error;

    if (controller == NULL) {
        return line_error;
    }

    if (controller->aligning != 0U) {
        return line_error * controller->exit_line_blend;
    }

    /*
     * 右弯保持只抬高不足的右转请求。若灰度已经要求左修正，
     * 说明车辆可能转过头，必须允许传感器立即接管反向纠偏。
     */
    if ((controller->active != 0U) && (line_error >= 0.0f)) {
        minimum_right_error =
            MapCurveHold_GetMinimumRightLineError(controller);
        if (line_error < minimum_right_error) {
            return minimum_right_error;
        }
    }

    return line_error;
}

uint8_t MapCurveHold_IsActive(const MapCurveHold *controller)
{
    return (controller == NULL) ? 0U : controller->active;
}

uint8_t MapCurveHold_IsAligning(const MapCurveHold *controller)
{
    return (controller == NULL) ? 0U : controller->aligning;
}

uint8_t MapCurveHold_GetCompletedCount(
    const MapCurveHold *controller)
{
    return (controller == NULL) ?
        0U : controller->completed_curve_count;
}

float MapCurveHold_GetRightTurnDeg(
    const MapCurveHold *controller)
{
    return (controller == NULL) ?
        0.0f : controller->right_turn_deg;
}

float MapCurveHold_GetExitLineBlend(
    const MapCurveHold *controller)
{
    return (controller == NULL) ?
        1.0f : controller->exit_line_blend;
}
