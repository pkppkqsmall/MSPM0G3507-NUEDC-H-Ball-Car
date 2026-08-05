#ifndef MAP_RUN_H_
#define MAP_RUN_H_

/* 初始化题目地图运行所需的底盘、传感器、姿态和显示模块。 */
void MapRun_Init(void);

/* 主循环反复调用，执行按键、循迹、速度环、终点判断和显示任务。 */
void MapRun_RunStep(void);

#endif /* MAP_RUN_H_ */
