#ifndef BALL_BEAM_CONTROLLER_H_
#define BALL_BEAM_CONTROLLER_H_

#include <stdint.h>

#define BALL_BEAM_TARGET_MAX_ABS_MM    (120.0f)
#define BALL_BEAM_MAX_PREDICTION_DELAY_MS (200UL)
#define BALL_BEAM_MAX_LEVEL_TRIM_DEG      (2.0f)
#define BALL_BEAM_MAX_EXTERNAL_FEEDFORWARD_DEG (2.0f)
#define BALL_BEAM_DEFAULT_STALE_TIMEOUT_MS (150UL)
#define BALL_BEAM_MIN_STALE_TIMEOUT_MS      (50UL)
#define BALL_BEAM_MAX_STALE_TIMEOUT_MS    (1000UL)

typedef struct {
    float target_x_mm;
    float raw_x_mm;
    float filtered_x_mm;
    float last_valid_filtered_x_mm;
    float predicted_x_mm;
    float speed_reference_x_mm;
    float ball_speed_mm_s;
    float profile_target_speed_mm_s;
    float profile_max_speed_mm_s;
    float profile_position_rate_per_s;
    float profile_blend;
    float position_error_mm;
    float control_error_mm;
    float position_output_deg;
    float speed_output_deg;
    float profile_output_deg;
    float target_beam_angle_deg;
    float dynamic_angle_limit_deg;
    float kp_deg_per_mm;
    float ki_deg_per_mm_s;
    float kd_deg_per_mm_s;
    float integral_output_deg;
    float level_trim_deg;
    float external_feedforward_deg;
    float breakaway_output_deg;
    uint32_t camera_frame_id;
    uint32_t camera_timestamp_ms;
    uint32_t previous_camera_timestamp_ms;
    uint32_t speed_reference_timestamp_ms;
    uint32_t last_frame_period_ms;
    uint32_t last_speed_period_ms;
    uint32_t last_valid_receive_ms;
    uint32_t parsed_frame_count;
    uint32_t rejected_frame_count;
    uint32_t duplicate_frame_count;
    uint32_t timing_fault_count;
    uint32_t dropout_prediction_count;
    uint32_t dropout_recovery_count;
    uint32_t dropout_fault_count;
    uint32_t prediction_delay_ms;
    uint32_t stale_timeout_ms;
    uint32_t breakaway_stationary_ms;
    int32_t target_stepper_pulses;
    int8_t output_sign;
    int8_t breakaway_direction;
    uint8_t enabled;
    uint8_t measurement_valid;
    uint8_t stale;
    uint8_t has_camera_frame;
    uint8_t has_previous_sample;
    uint8_t has_speed_reference;
    uint8_t breakaway_active;
    uint8_t dropout_active;
    uint8_t dropout_consecutive_count;
} BallBeamController;

/* 初始化为只计算不驱动，默认目标位置为横梁中心 0 mm。 */
void BallBeamController_Init(BallBeamController *controller);

/*
 * 支持两种格式：
 * BALL,frame_id,timestamp_ms,x_mm,valid
 * [BALL] POS:+11.5cm
 *
 * 位置约定左侧为负、右侧为正。简化 cm 格式没有时间戳，因此使用
 * MSPM0 收到完整行的时刻计算 dt；标准格式优先使用摄像头时间戳。
 * 标准格式的短时 valid=0 会使用最后位置和球速预测，恢复有效帧后平滑
 * 接回实测位置；返回 1 表示收到并处理了有效格式的 BALL 帧。
 */
uint8_t BallBeamController_ProcessCameraLine(
    BallBeamController *controller,
    const char *line,
    uint32_t receive_ms);

/* 更新数据超时保护；超时后目标摆杆角自动回到 0 度。 */
void BallBeamController_Update(BallBeamController *controller,
                               uint32_t now_ms);

/* 按运行场景设置摄像头断帧容忍时间。 */
uint8_t BallBeamController_SetStaleTimeoutMs(
    BallBeamController *controller,
    uint32_t stale_timeout_ms);

void BallBeamController_SetEnabled(BallBeamController *controller,
                                   uint8_t enabled);
uint8_t BallBeamController_IsEnabled(
    const BallBeamController *controller);
uint8_t BallBeamController_IsControlReady(
    const BallBeamController *controller);

/* 目标位置允许设置在横梁中心两侧各 120 mm 范围内。 */
uint8_t BallBeamController_SetTargetPositionMm(
    BallBeamController *controller,
    float target_x_mm);

/* 设置位置外环的最高目标速度和“剩余距离 -> 目标速度”比例。 */
uint8_t BallBeamController_SetMotionProfile(
    BallBeamController *controller,
    float max_speed_mm_s,
    float position_rate_per_s);

/* 设置 PD 时会关闭并清空积分，保留原有命令兼容性。 */
uint8_t BallBeamController_SetGains(BallBeamController *controller,
                                    float kp_deg_per_mm,
                                    float kd_deg_per_mm_s);

/* 设置带门控和抗饱和的 PID 参数。 */
uint8_t BallBeamController_SetPIDGains(
    BallBeamController *controller,
    float kp_deg_per_mm,
    float ki_deg_per_mm_s,
    float kd_deg_per_mm_s);

/*
 * 按当前球速把摄像头位置外推指定时间，补偿图像处理和无线传输延迟。
 * 允许范围为 0~200 ms，设置 0 可完全关闭预测。
 */
uint8_t BallBeamController_SetPredictionDelayMs(
    BallBeamController *controller,
    uint32_t prediction_delay_ms);

/* 设置固定水平补偿角，用于抵消管道安装斜率，范围为 ±2 度。 */
uint8_t BallBeamController_SetLevelTrimDeg(
    BallBeamController *controller,
    float level_trim_deg);

/* 设置瞬态外部前馈角；关闭控制器时自动清零，不写入 PID 积分。 */
uint8_t BallBeamController_SetExternalFeedforwardDeg(
    BallBeamController *controller,
    float feedforward_deg);

/*
 * output_sign 只允许 +1 或 -1。
 * +1 表示正脉冲对应摆杆右端抬高；实机方向相反时设置为 -1。
 */
uint8_t BallBeamController_SetOutputSign(
    BallBeamController *controller,
    int8_t output_sign);

/*
 * 把物理摆杆角转换为步进目标脉冲。输入角不包含水平 trim，函数会自动
 * 叠加 trim、应用 output_sign，并限制在控制器允许的最大摆角内。
 */
int32_t BallBeamController_ConvertBeamAngleToPulses(
    const BallBeamController *controller,
    float beam_angle_deg);

int32_t BallBeamController_GetTargetStepperPulses(
    const BallBeamController *controller);

#endif /* BALL_BEAM_CONTROLLER_H_ */
