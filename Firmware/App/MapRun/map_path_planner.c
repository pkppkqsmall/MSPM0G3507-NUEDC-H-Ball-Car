#include "map_path_planner.h"

#include <stddef.h>

#define MAP_PATH_STRAIGHT_SPEED_RPM              (100.0f)
#define MAP_PATH_CURVE_SPEED_RPM                  (95.0f)
#define MAP_PATH_FINISH_SPEED_RPM                 (60.0f)

/*
 * 65 mm 车轮、728 count/圈时，13 cm 前探距离约为 463 count。
 * 下列位置已经从几何路段边界减去了这段前探量。
 */
#define MAP_PATH_B_ENTRY_CENTER_COUNTS            (4884UL)
#define MAP_PATH_C_EXIT_CENTER_COUNTS             (10484UL)
#define MAP_PATH_D_ENTRY_CENTER_COUNTS            (15832UL)
#define MAP_PATH_A_EXIT_CENTER_COUNTS             (21432UL)
#define MAP_PATH_CURVE_ENTRY_ADVANCE_COUNTS         (500UL)
#define MAP_PATH_CURVE_EXIT_DELAY_COUNTS            (500UL)
#define MAP_PATH_TRANSITION_COUNTS                   (600UL)
#define MAP_PATH_FINISH_DECEL_COUNTS               (700UL)

#define MAP_PATH_B_CURVE_CENTER_COUNTS             \
    (MAP_PATH_B_ENTRY_CENTER_COUNTS -              \
     MAP_PATH_CURVE_ENTRY_ADVANCE_COUNTS)
#define MAP_PATH_C_CURVE_CENTER_COUNTS             \
    (MAP_PATH_C_EXIT_CENTER_COUNTS +               \
     MAP_PATH_CURVE_EXIT_DELAY_COUNTS)
#define MAP_PATH_D_CURVE_CENTER_COUNTS             \
    (MAP_PATH_D_ENTRY_CENTER_COUNTS -              \
     MAP_PATH_CURVE_ENTRY_ADVANCE_COUNTS)
#define MAP_PATH_A_CURVE_CENTER_COUNTS             \
    (MAP_PATH_A_EXIT_CENTER_COUNTS +               \
     MAP_PATH_CURVE_EXIT_DELAY_COUNTS)

/* 轮距 214.2 mm、赛道中心半径 500 mm，几何差速比例为 b/(2R)。 */
#define MAP_PATH_CURVE_DIFFERENTIAL_RATIO          (0.2142f)

/*
 * 100 RPM 附近实车存在轮胎侧滑和转向惯性，纯几何前馈转向不足。
 * 先按理论值的 1.5 倍补偿，95 RPM 时差速约为 30.5 RPM。
 */
#define MAP_PATH_CURVE_FEEDFORWARD_SCALE            (1.50f)

static float MapPathPlanner_Clamp01(float value)
{
    if (value < 0.0f) {
        return 0.0f;
    }

    if (value > 1.0f) {
        return 1.0f;
    }

    return value;
}

static float MapPathPlanner_SmoothStep(float progress)
{
    progress = MapPathPlanner_Clamp01(progress);
    return progress * progress * (3.0f - (2.0f * progress));
}

static float MapPathPlanner_TransitionAt(uint32_t distance_counts,
                                         uint32_t center_counts)
{
    uint32_t half_width = MAP_PATH_TRANSITION_COUNTS / 2UL;
    uint32_t start_counts =
        (center_counts > half_width) ?
        (center_counts - half_width) : 0UL;
    uint32_t end_counts = center_counts + half_width;

    if (distance_counts <= start_counts) {
        return 0.0f;
    }

    if (distance_counts >= end_counts) {
        return 1.0f;
    }

    return MapPathPlanner_SmoothStep(
        (float) (distance_counts - start_counts) /
        (float) (end_counts - start_counts));
}

static float MapPathPlanner_GetCurveBlend(uint32_t distance_counts)
{
    /*
     * 编码器累计值、起跑位置和轮胎打滑都可能带来数百计数误差。
     * 因此右弯提前进入、延后退出，并在更宽的区间内平滑变化。
     * 到理论 B/D 点之前前馈已经全开，避免高速下等待过久。
     */
    float curve_blend =
        MapPathPlanner_TransitionAt(
            distance_counts,
            MAP_PATH_B_CURVE_CENTER_COUNTS) -
        MapPathPlanner_TransitionAt(
            distance_counts,
            MAP_PATH_C_CURVE_CENTER_COUNTS) +
        MapPathPlanner_TransitionAt(
            distance_counts,
            MAP_PATH_D_CURVE_CENTER_COUNTS) -
        MapPathPlanner_TransitionAt(
            distance_counts,
            MAP_PATH_A_CURVE_CENTER_COUNTS);

    return MapPathPlanner_Clamp01(curve_blend);
}

static float MapPathPlanner_GetFinishBlend(uint32_t distance_counts)
{
    uint32_t finish_start_counts =
        MAP_PATH_A_EXIT_CENTER_COUNTS -
        MAP_PATH_FINISH_DECEL_COUNTS;

    if (distance_counts <= finish_start_counts) {
        return 0.0f;
    }

    if (distance_counts >= MAP_PATH_A_EXIT_CENTER_COUNTS) {
        return 1.0f;
    }

    return MapPathPlanner_SmoothStep(
        (float) (distance_counts - finish_start_counts) /
        (float) MAP_PATH_FINISH_DECEL_COUNTS);
}

static MapPathPhase MapPathPlanner_GetPhase(uint32_t distance_counts)
{
    if (distance_counts < MAP_PATH_B_CURVE_CENTER_COUNTS) {
        return MAP_PATH_PHASE_STRAIGHT_AB;
    }

    if (distance_counts < MAP_PATH_C_CURVE_CENTER_COUNTS) {
        return MAP_PATH_PHASE_CURVE_BC;
    }

    if (distance_counts < MAP_PATH_D_CURVE_CENTER_COUNTS) {
        return MAP_PATH_PHASE_STRAIGHT_CD;
    }

    if (distance_counts < MAP_PATH_A_EXIT_CENTER_COUNTS) {
        return MAP_PATH_PHASE_CURVE_DA;
    }

    return MAP_PATH_PHASE_FINISH;
}

void MapPathPlanner_GetCommand(uint32_t distance_counts,
                               MapPathCommand *command)
{
    float finish_blend;
    float path_base_speed_rpm;

    if (command == NULL) {
        return;
    }

    command->phase = MapPathPlanner_GetPhase(distance_counts);
    command->curve_blend =
        MapPathPlanner_GetCurveBlend(distance_counts);
    path_base_speed_rpm =
        MAP_PATH_STRAIGHT_SPEED_RPM +
        ((MAP_PATH_CURVE_SPEED_RPM -
          MAP_PATH_STRAIGHT_SPEED_RPM) *
         command->curve_blend);

    finish_blend =
        MapPathPlanner_GetFinishBlend(distance_counts);
    command->base_speed_rpm =
        path_base_speed_rpm +
        ((MAP_PATH_FINISH_SPEED_RPM -
          path_base_speed_rpm) *
         finish_blend);

    /*
     * 前馈随当前基础速度缩放，减速时仍保持相同理论转弯半径。
     * 正前馈表示左轮更快、右轮更慢，对应题图顺时针右弯。
     */
    command->curve_feedforward_rpm =
        command->base_speed_rpm *
        MAP_PATH_CURVE_DIFFERENTIAL_RATIO *
        MAP_PATH_CURVE_FEEDFORWARD_SCALE *
        command->curve_blend;
}

const char *MapPathPlanner_GetPhaseName(MapPathPhase phase)
{
    switch (phase) {
        case MAP_PATH_PHASE_STRAIGHT_AB:
            return "AB";
        case MAP_PATH_PHASE_CURVE_BC:
            return "BC";
        case MAP_PATH_PHASE_STRAIGHT_CD:
            return "CD";
        case MAP_PATH_PHASE_CURVE_DA:
            return "DA";
        case MAP_PATH_PHASE_FINISH:
            return "FIN";
        default:
            return "UNK";
    }
}
