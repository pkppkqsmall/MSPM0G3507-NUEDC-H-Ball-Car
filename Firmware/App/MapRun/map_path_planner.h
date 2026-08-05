#ifndef MAP_PATH_PLANNER_H_
#define MAP_PATH_PLANNER_H_

#include <stdint.h>

typedef enum {
    MAP_PATH_PHASE_STRAIGHT_AB = 0,
    MAP_PATH_PHASE_CURVE_BC,
    MAP_PATH_PHASE_STRAIGHT_CD,
    MAP_PATH_PHASE_CURVE_DA,
    MAP_PATH_PHASE_FINISH
} MapPathPhase;

typedef struct {
    MapPathPhase phase;
    float base_speed_rpm;
    float curve_feedforward_rpm;
    float curve_blend;
} MapPathCommand;

/* 根据一圈内的平均编码器路程生成基础速度和右弯差速前馈。 */
void MapPathPlanner_GetCommand(uint32_t distance_counts,
                               MapPathCommand *command);

/* 返回适合串口诊断显示的短路段名称。 */
const char *MapPathPlanner_GetPhaseName(MapPathPhase phase);

#endif /* MAP_PATH_PLANNER_H_ */
