#ifndef APP_SPEED_TASK_H_
#define APP_SPEED_TASK_H_

typedef struct {
    unsigned long last_speed_tick;  /* 上一次速度环更新的 tick_ms。 */
} AppSpeedTask;

/* 初始化速度环任务的时间基准。 */
void AppSpeedTask_Init(AppSpeedTask *task);

/*
 * 按 g_motor_speed_sample_time_ms 固定周期更新测速和左右轮速度环。
 * 普通模式输出 VOFA 数据；AI 调参模式输出 llm-pid-tuner CSV。
 */
void AppSpeedTask_Update(AppSpeedTask *task);

#endif /* APP_SPEED_TASK_H_ */
