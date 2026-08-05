#include "ball_beam_controller.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define BALL_BEAM_FRAME_PREFIX                 "BALL,"
#define BALL_BEAM_FRAME_PREFIX_LENGTH          (5U)
#define BALL_BEAM_POSITION_CM_PREFIX           "[BALL] POS:"
#define BALL_BEAM_POSITION_CM_PREFIX_LENGTH    (11U)
#define BALL_BEAM_CM_TO_MM                     (10.0f)
#define BALL_BEAM_SPEED_ESTIMATION_PERIOD_MS    (75UL)
#define BALL_BEAM_SPEED_RESET_PERIOD_MS         (250UL)
#define BALL_BEAM_SPEED_POSITION_DEADBAND_MM     (1.0f)
#define BALL_BEAM_MAX_MEASUREMENT_ABS_MM       (200.0f)
#define BALL_BEAM_POSITION_FILTER_ALPHA        (0.5f)
#define BALL_BEAM_SPEED_FILTER_ALPHA           (0.45f)
#define BALL_BEAM_DROPOUT_PREDICTION_START_MS    (40UL)
#define BALL_BEAM_DROPOUT_PREDICTION_MAX_MS     (100UL)
#define BALL_BEAM_DROPOUT_PREDICTION_MAX_SHIFT_MM (12.0f)
#define BALL_BEAM_DEFAULT_KP_DEG_PER_MM         (0.0075f)
#define BALL_BEAM_DEFAULT_KI_DEG_PER_MM_S       (0.015f)
#define BALL_BEAM_DEFAULT_KD_DEG_PER_MM_S       (0.030f)
#define BALL_BEAM_DEFAULT_PREDICTION_DELAY_MS      (0UL)
#define BALL_BEAM_DEFAULT_LEVEL_TRIM_DEG          (0.0f)
#define BALL_BEAM_MAX_PREDICTION_SHIFT_MM         (20.0f)
#define BALL_BEAM_PROFILE_BLEND_START_MM            (6.0f)
#define BALL_BEAM_PROFILE_BLEND_FULL_MM             (15.0f)
#define BALL_BEAM_PROFILE_MAX_SPEED_MM_S           (60.0f)
#define BALL_BEAM_PROFILE_POSITION_RATE_PER_S       (1.2f)
#define BALL_BEAM_PROFILE_MIN_SPEED_MM_S            (10.0f)
#define BALL_BEAM_PROFILE_MAX_ALLOWED_SPEED_MM_S   (120.0f)
#define BALL_BEAM_PROFILE_MIN_POSITION_RATE_PER_S    (0.2f)
#define BALL_BEAM_PROFILE_MAX_POSITION_RATE_PER_S    (4.0f)
#define BALL_BEAM_PROFILE_BRAKE_ACCEL_MM_S2       (120.0f)
#define BALL_BEAM_MAX_KP_DEG_PER_MM             (0.200f)
#define BALL_BEAM_MAX_KI_DEG_PER_MM_S           (0.050f)
#define BALL_BEAM_MAX_KD_DEG_PER_MM_S           (0.050f)
#define BALL_BEAM_MAX_ANGLE_DEG                  (4.0f)
#define BALL_BEAM_MAX_DIRECT_ANGLE_DEG           (4.5f)
#define BALL_BEAM_BREAKAWAY_ANGLE_FLOOR_DEG      (4.0f)
#define BALL_BEAM_MIN_DYNAMIC_ANGLE_DEG          (1.5f)
#define BALL_BEAM_DYNAMIC_ERROR_GAIN_DEG_PER_MM  (0.035f)
#define BALL_BEAM_DYNAMIC_SPEED_GAIN_DEG_PER_MM_S (0.010f)
#define BALL_BEAM_DYNAMIC_TRIM_MARGIN_DEG        (0.5f)
#define BALL_BEAM_SETTLE_POSITION_MM             (2.0f)
#define BALL_BEAM_SETTLE_SPEED_MM_S             (10.0f)
#define BALL_BEAM_INTEGRAL_MAX_ERROR_MM         (20.0f)
#define BALL_BEAM_INTEGRAL_MAX_SPEED_MM_S       (80.0f)
#define BALL_BEAM_INTEGRAL_MAX_OUTPUT_DEG        (1.5f)
#define BALL_BEAM_BREAKAWAY_ENTER_ERROR_MM        (8.0f)
#define BALL_BEAM_BREAKAWAY_EXIT_ERROR_MM         (5.0f)
#define BALL_BEAM_BREAKAWAY_ENTER_SPEED_MM_S      (6.0f)
#define BALL_BEAM_BREAKAWAY_EXIT_SPEED_MM_S      (10.0f)
#define BALL_BEAM_BREAKAWAY_RELEASE_MOVE_MM        (1.5f)
#define BALL_BEAM_BREAKAWAY_WAIT_MS             (100UL)
#define BALL_BEAM_BREAKAWAY_START_MIN_DEG          (0.50f)
#define BALL_BEAM_BREAKAWAY_START_MAX_DEG          (1.00f)
#define BALL_BEAM_BREAKAWAY_START_ERROR_GAIN       (0.010f)
#define BALL_BEAM_BREAKAWAY_POSITIVE_MAX_DEG       (2.40f)
#define BALL_BEAM_BREAKAWAY_NEGATIVE_MAX_DEG       (2.40f)
#define BALL_BEAM_BREAKAWAY_RAMP_DEG_PER_S         (1.80f)
#define BALL_BEAM_BREAKAWAY_RAMP_TIME_LIMIT_MS   (1000UL)

static float BallBeamController_ClampFloat(float value,
                                           float minimum,
                                           float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float BallBeamController_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void BallBeamController_ResetBreakaway(
    BallBeamController *controller)
{
    controller->breakaway_output_deg = 0.0f;
    controller->breakaway_stationary_ms = 0UL;
    controller->breakaway_direction = 0;
    controller->breakaway_active = 0U;
}

static void BallBeamController_AccumulateBreakawayTime(
    BallBeamController *controller,
    uint32_t speed_period_ms)
{
    uint32_t maximum_time_ms =
        BALL_BEAM_BREAKAWAY_WAIT_MS +
        BALL_BEAM_BREAKAWAY_RAMP_TIME_LIMIT_MS;
    uint32_t remaining_ms;

    if (controller->breakaway_stationary_ms >= maximum_time_ms) {
        return;
    }

    remaining_ms =
        maximum_time_ms - controller->breakaway_stationary_ms;
    controller->breakaway_stationary_ms +=
        (speed_period_ms >= remaining_ms) ?
            remaining_ms : speed_period_ms;
}

static float BallBeamController_GetBreakawayOutputAbsDeg(
    const BallBeamController *controller)
{
    uint32_t ramp_time_ms = 0UL;
    float error_abs_mm;
    float start_output_deg;
    float maximum_output_deg;
    float output_deg;

    if (controller->breakaway_stationary_ms >
        BALL_BEAM_BREAKAWAY_WAIT_MS) {
        ramp_time_ms =
            controller->breakaway_stationary_ms -
            BALL_BEAM_BREAKAWAY_WAIT_MS;
    }

    error_abs_mm = BallBeamController_AbsFloat(
        controller->filtered_x_mm - controller->target_x_mm);
    start_output_deg = BallBeamController_ClampFloat(
        BALL_BEAM_BREAKAWAY_START_MIN_DEG +
            BALL_BEAM_BREAKAWAY_START_ERROR_GAIN * error_abs_mm,
        BALL_BEAM_BREAKAWAY_START_MIN_DEG,
        BALL_BEAM_BREAKAWAY_START_MAX_DEG);
    output_deg =
        start_output_deg +
        BALL_BEAM_BREAKAWAY_RAMP_DEG_PER_S *
            ((float) ramp_time_ms / 1000.0f);

    /*
     * 管道不同位置的起滚角差异较大，正、负方向均允许补偿到 2.4 度；
     * 总控制角仍受 4 度硬限位约束，球一旦移动 1.5 mm 就立即撤销。
     */
    maximum_output_deg =
        (controller->breakaway_direction < 0) ?
            BALL_BEAM_BREAKAWAY_NEGATIVE_MAX_DEG :
            BALL_BEAM_BREAKAWAY_POSITIVE_MAX_DEG;
    return BallBeamController_ClampFloat(
        output_deg,
        BALL_BEAM_BREAKAWAY_START_MIN_DEG,
        maximum_output_deg);
}

static float BallBeamController_GetDynamicAngleLimitDeg(
    const BallBeamController *controller)
{
    float error_abs_mm = BallBeamController_AbsFloat(
        controller->control_error_mm);
    float speed_abs_mm_s = BallBeamController_AbsFloat(
        controller->ball_speed_mm_s);
    float trim_floor_deg =
        BallBeamController_AbsFloat(controller->level_trim_deg) +
        BALL_BEAM_DYNAMIC_TRIM_MARGIN_DEG;
    float limit_deg =
        BALL_BEAM_MIN_DYNAMIC_ANGLE_DEG +
        BALL_BEAM_DYNAMIC_ERROR_GAIN_DEG_PER_MM * error_abs_mm +
        BALL_BEAM_DYNAMIC_SPEED_GAIN_DEG_PER_MM_S * speed_abs_mm_s;

    /*
     * 误差大时允许更强起滚，高速接近目标时保留足够制动角；接近静止
     * 目标后则自动收小。trim 是机械水平补偿，不能被动态限幅吃掉。
     * 已确认进入静摩擦脱困后，临时允许使用完整 4 度硬限位；球一恢复
     * 运动，breakaway 会退出，动态上限也立即恢复按误差和速度计算。
     */
    if (limit_deg < trim_floor_deg) {
        limit_deg = trim_floor_deg;
    }
    if ((controller->breakaway_active != 0U) &&
        (limit_deg < BALL_BEAM_BREAKAWAY_ANGLE_FLOOR_DEG)) {
        limit_deg = BALL_BEAM_BREAKAWAY_ANGLE_FLOOR_DEG;
    }
    return BallBeamController_ClampFloat(
        limit_deg,
        BALL_BEAM_MIN_DYNAMIC_ANGLE_DEG,
        BALL_BEAM_MAX_ANGLE_DEG);
}

static float BallBeamController_GetPredictedPositionMm(
    const BallBeamController *controller)
{
    float prediction_shift_mm;

    prediction_shift_mm =
        controller->ball_speed_mm_s *
        ((float) controller->prediction_delay_ms / 1000.0f);
    prediction_shift_mm =
        BallBeamController_ClampFloat(
            prediction_shift_mm,
            -BALL_BEAM_MAX_PREDICTION_SHIFT_MM,
            BALL_BEAM_MAX_PREDICTION_SHIFT_MM);

    return controller->filtered_x_mm + prediction_shift_mm;
}

static float BallBeamController_GetProfileTargetSpeedMmS(
    const BallBeamController *controller,
    float control_error_mm)
{
    float distance_mm =
        (control_error_mm < 0.0f) ?
            -control_error_mm : control_error_mm;
    float position_limited_speed_mm_s =
        controller->profile_position_rate_per_s * distance_mm;
    float braking_limited_speed_mm_s =
        sqrtf(2.0f *
              BALL_BEAM_PROFILE_BRAKE_ACCEL_MM_S2 *
              distance_mm);
    float target_speed_abs_mm_s =
        controller->profile_max_speed_mm_s;

    if (position_limited_speed_mm_s < target_speed_abs_mm_s) {
        target_speed_abs_mm_s = position_limited_speed_mm_s;
    }
    if (braking_limited_speed_mm_s < target_speed_abs_mm_s) {
        target_speed_abs_mm_s = braking_limited_speed_mm_s;
    }

    /*
     * control_error 为正表示球在目标右侧，期望速度应向左；反之向右。
     * 因此目标速度方向始终与位置误差相反。
     */
    return (control_error_mm > 0.0f) ?
        -target_speed_abs_mm_s : target_speed_abs_mm_s;
}

static float BallBeamController_GetProfileBlend(
    float control_error_mm)
{
    float distance_mm =
        (control_error_mm < 0.0f) ?
            -control_error_mm : control_error_mm;

    return BallBeamController_ClampFloat(
        (distance_mm - BALL_BEAM_PROFILE_BLEND_START_MM) /
        (BALL_BEAM_PROFILE_BLEND_FULL_MM -
         BALL_BEAM_PROFILE_BLEND_START_MM),
        0.0f,
        1.0f);
}

static void BallBeamController_SetSafeOutput(
    BallBeamController *controller)
{
    controller->position_error_mm = 0.0f;
    controller->control_error_mm = 0.0f;
    controller->predicted_x_mm = controller->filtered_x_mm;
    controller->profile_target_speed_mm_s = 0.0f;
    controller->profile_blend = 0.0f;
    controller->position_output_deg = 0.0f;
    controller->speed_output_deg = 0.0f;
    controller->profile_output_deg = 0.0f;
    controller->integral_output_deg = 0.0f;
    controller->external_feedforward_deg = 0.0f;
    BallBeamController_ResetBreakaway(controller);
    controller->dropout_active = 0U;
    controller->dropout_consecutive_count = 0U;
    controller->target_beam_angle_deg = 0.0f;
    controller->dynamic_angle_limit_deg = 0.0f;
    controller->target_stepper_pulses = 0;
}

/*
 * V12 使用 m1.5/18T 小齿轮驱动竖直齿条。表格根据 263.309 mm 有效
 * 臂长、-2.7771 度铰点局部角和 16 细分计算，并保留分段插值以便
 * 后续替换成实机标定值。
 */
static int32_t BallBeamController_AngleToPulses(float angle_deg)
{
    static const float beam_angle_deg[] = {
        -10.0f, -9.0f, -8.0f, -7.0f, -6.0f, -5.0f,
         -4.0f, -3.0f, -2.0f, -1.0f,  0.0f,  1.0f,
          2.0f,  3.0f,  4.0f,  5.0f,  6.0f,  7.0f,
          8.0f,  9.0f, 10.0f
    };
    static const int16_t stepper_pulses[] = {
        -1716, -1546, -1376, -1206, -1034, -863,
         -691,  -519,  -346,  -173,     0,  173,
          347,   520,   693,   867,  1040, 1213,
         1386,  1558,  1730
    };
    uint8_t index;
    float ratio;
    float pulse_value;

    if (angle_deg <= beam_angle_deg[0]) {
        return (int32_t) stepper_pulses[0];
    }

    for (index = 1U;
         index < (sizeof(beam_angle_deg) / sizeof(beam_angle_deg[0]));
         index++) {
        if (angle_deg <= beam_angle_deg[index]) {
            ratio =
                (angle_deg - beam_angle_deg[index - 1U]) /
                (beam_angle_deg[index] -
                 beam_angle_deg[index - 1U]);
            pulse_value =
                (float) stepper_pulses[index - 1U] +
                ratio *
                (float) (stepper_pulses[index] -
                         stepper_pulses[index - 1U]);

            return (pulse_value >= 0.0f) ?
                (int32_t) (pulse_value + 0.5f) :
                (int32_t) (pulse_value - 0.5f);
        }
    }

    return (int32_t)
        stepper_pulses[
            (sizeof(stepper_pulses) / sizeof(stepper_pulses[0])) - 1U];
}

static void BallBeamController_Recalculate(
    BallBeamController *controller)
{
    float control_angle_deg;
    float normal_control_angle_deg;
    float profile_control_angle_deg;
    float position_term_error_mm;
    float speed_term_mm_s;

    if ((controller->measurement_valid == 0U) ||
        (controller->stale != 0U)) {
        BallBeamController_SetSafeOutput(controller);
        return;
    }

    /*
     * x_mm 左负右正，摆杆正角表示右端抬高。
     * 球在右侧或向右运动时应抬高右端，因此位置项和速度项同号相加。
     * 若实机 DIR 与该正方向相反，只需把 output_sign 改为 -1。
     */
    controller->position_error_mm =
        controller->filtered_x_mm - controller->target_x_mm;
    controller->predicted_x_mm =
        BallBeamController_GetPredictedPositionMm(controller);
    controller->control_error_mm =
        controller->predicted_x_mm - controller->target_x_mm;
    controller->profile_target_speed_mm_s =
        BallBeamController_GetProfileTargetSpeedMmS(
            controller,
            controller->control_error_mm);
    controller->profile_blend =
        BallBeamController_GetProfileBlend(
            controller->control_error_mm);

    /*
     * 中心附近只关闭 P、D 的微小抖动，保留 I 项形成的水平偏差补偿。
     * 高速穿过中心时仍保留 D 项制动。
     */
    position_term_error_mm = controller->control_error_mm;
    speed_term_mm_s = controller->ball_speed_mm_s;
    if ((controller->control_error_mm >=
         -BALL_BEAM_SETTLE_POSITION_MM) &&
        (controller->control_error_mm <=
         BALL_BEAM_SETTLE_POSITION_MM)) {
        position_term_error_mm = 0.0f;
    }
    if ((controller->ball_speed_mm_s >=
         -BALL_BEAM_SETTLE_SPEED_MM_S) &&
        (controller->ball_speed_mm_s <=
         BALL_BEAM_SETTLE_SPEED_MM_S)) {
        speed_term_mm_s = 0.0f;
    }

    controller->position_output_deg =
        controller->kp_deg_per_mm * position_term_error_mm;
    controller->speed_output_deg =
        controller->kd_deg_per_mm_s * speed_term_mm_s;
    normal_control_angle_deg =
        controller->position_output_deg +
        controller->speed_output_deg;
    controller->profile_output_deg =
        controller->kd_deg_per_mm_s *
        (controller->ball_speed_mm_s -
         controller->profile_target_speed_mm_s);
    profile_control_angle_deg =
        controller->profile_output_deg;

    /*
     * 6 mm 内保留 PID 做最终位置保持，15 mm 外完全使用速度包络。
     * 中间区线性混合，避免控制角突然跳变。远端不叠加 I，防止回中
     * 制动阶段仍被旧积分持续推向目标。固定 trim 始终叠加，用于抵消
     * 管道安装斜率；external_feedforward 用于底盘加减速等已知扰动；
     * breakaway 只在球远离目标且持续静止时短暂帮助克服静摩擦。
     * 这些补偿都不改变 PID 参数。
     */
    control_angle_deg =
        normal_control_angle_deg *
            (1.0f - controller->profile_blend) +
        profile_control_angle_deg *
            controller->profile_blend +
        controller->integral_output_deg *
            (1.0f - controller->profile_blend) +
        controller->level_trim_deg +
        controller->external_feedforward_deg +
        controller->breakaway_output_deg;

    control_angle_deg *= (float) controller->output_sign;

    controller->dynamic_angle_limit_deg =
        BallBeamController_GetDynamicAngleLimitDeg(controller);

    controller->target_beam_angle_deg =
        BallBeamController_ClampFloat(
            control_angle_deg,
            -controller->dynamic_angle_limit_deg,
            controller->dynamic_angle_limit_deg);
    controller->target_stepper_pulses =
        BallBeamController_AngleToPulses(
            controller->target_beam_angle_deg);
}

/*
 * 摄像头短暂丢球时不采用无效帧中的 x=0，而从最后有效滤波位置按当前
 * 球速外推。速度随丢失时间线性衰减，因此位移使用其积分形式
 * v*t*(1-0.5*t/T)，并限制最大外推距离，避免旧速度无限推走控制目标。
 */
static uint8_t BallBeamController_ApplyDropoutPrediction(
    BallBeamController *controller,
    uint32_t receive_ms)
{
    uint32_t valid_age_ms;
    uint32_t prediction_age_ms;
    float progress;
    float prediction_shift_mm;

    if ((controller->measurement_valid == 0U) ||
        (controller->has_previous_sample == 0U)) {
        return 0U;
    }

    valid_age_ms = receive_ms - controller->last_valid_receive_ms;
    if (valid_age_ms > controller->stale_timeout_ms) {
        return 0U;
    }

    prediction_age_ms =
        (valid_age_ms > BALL_BEAM_DROPOUT_PREDICTION_MAX_MS) ?
            BALL_BEAM_DROPOUT_PREDICTION_MAX_MS : valid_age_ms;

    progress =
        (float) prediction_age_ms /
        (float) BALL_BEAM_DROPOUT_PREDICTION_MAX_MS;
    prediction_shift_mm =
        controller->ball_speed_mm_s *
        ((float) prediction_age_ms / 1000.0f) *
        (1.0f - (0.5f * progress));
    prediction_shift_mm = BallBeamController_ClampFloat(
        prediction_shift_mm,
        -BALL_BEAM_DROPOUT_PREDICTION_MAX_SHIFT_MM,
        BALL_BEAM_DROPOUT_PREDICTION_MAX_SHIFT_MM);
    controller->filtered_x_mm = BallBeamController_ClampFloat(
        controller->last_valid_filtered_x_mm + prediction_shift_mm,
        -BALL_BEAM_MAX_MEASUREMENT_ABS_MM,
        BALL_BEAM_MAX_MEASUREMENT_ABS_MM);
    if (controller->dropout_active == 0U) {
        controller->dropout_prediction_count++;
    }
    controller->dropout_active = 1U;
    controller->stale = 0U;

    /* 无实测位置时禁止静摩擦 kick，避免预测值触发额外强制起滚。 */
    BallBeamController_ResetBreakaway(controller);
    BallBeamController_Recalculate(controller);
    return 1U;
}

static void BallBeamController_UpdateBreakaway(
    BallBeamController *controller,
    uint32_t speed_period_ms)
{
    float error_mm;
    float error_abs_mm;
    float speed_abs_mm_s;
    int8_t error_direction;

    if ((controller->enabled == 0U) ||
        (controller->measurement_valid == 0U) ||
        (controller->stale != 0U)) {
        BallBeamController_ResetBreakaway(controller);
        return;
    }

    error_mm =
        controller->filtered_x_mm - controller->target_x_mm;
    error_abs_mm = BallBeamController_AbsFloat(error_mm);
    speed_abs_mm_s =
        BallBeamController_AbsFloat(controller->ball_speed_mm_s);
    error_direction = (error_mm > 0.0f) ? 1 : -1;

    if (controller->breakaway_active != 0U) {
        /*
         * 使用不同的进入/退出阈值形成滞回，避免速度在阈值附近时
         * 每帧反复启停。误差换向时必须重新等待，防止越过目标后
         * 立即向反方向施加补偿。
         */
        if ((error_abs_mm <= BALL_BEAM_BREAKAWAY_EXIT_ERROR_MM) ||
            (speed_abs_mm_s >=
             BALL_BEAM_BREAKAWAY_EXIT_SPEED_MM_S) ||
            (error_direction != controller->breakaway_direction)) {
            BallBeamController_ResetBreakaway(controller);
            return;
        }

        BallBeamController_AccumulateBreakawayTime(
            controller, speed_period_ms);
        controller->breakaway_output_deg =
            (float) controller->breakaway_direction *
            BallBeamController_GetBreakawayOutputAbsDeg(controller);
        return;
    }

    if ((error_abs_mm <= BALL_BEAM_BREAKAWAY_ENTER_ERROR_MM) ||
        (speed_abs_mm_s >=
         BALL_BEAM_BREAKAWAY_ENTER_SPEED_MM_S)) {
        BallBeamController_ResetBreakaway(controller);
        return;
    }

    if (controller->breakaway_direction != error_direction) {
        controller->breakaway_direction = error_direction;
        controller->breakaway_stationary_ms = 0UL;
    }

    if (controller->breakaway_stationary_ms <
        BALL_BEAM_BREAKAWAY_WAIT_MS) {
        uint32_t remaining_wait_ms =
            BALL_BEAM_BREAKAWAY_WAIT_MS -
            controller->breakaway_stationary_ms;

        controller->breakaway_stationary_ms +=
            (speed_period_ms >= remaining_wait_ms) ?
                remaining_wait_ms : speed_period_ms;
    }

    if (controller->breakaway_stationary_ms >=
        BALL_BEAM_BREAKAWAY_WAIT_MS) {
        controller->breakaway_active = 1U;
        controller->breakaway_output_deg =
            (float) controller->breakaway_direction *
            BallBeamController_GetBreakawayOutputAbsDeg(controller);
    }
}

static void BallBeamController_UpdateIntegral(
    BallBeamController *controller,
    uint32_t frame_period_ms)
{
    float error_mm;
    float candidate_integral_deg;
    float base_output_deg;
    float candidate_output_deg;
    float predicted_error_mm;

    if ((controller->enabled == 0U) ||
        (controller->ki_deg_per_mm_s <= 0.0f)) {
        return;
    }

    error_mm =
        controller->filtered_x_mm - controller->target_x_mm;

    /*
     * I 项只修正目标附近的固定偏差。进入速度包络区或高速运动时
     * 直接清空旧积分，避免回中制动阶段仍被积分推向目标。
     */
    if ((error_mm >= -BALL_BEAM_SETTLE_POSITION_MM) &&
        (error_mm <= BALL_BEAM_SETTLE_POSITION_MM)) {
        return;
    }
    if ((error_mm < -BALL_BEAM_INTEGRAL_MAX_ERROR_MM) ||
        (error_mm > BALL_BEAM_INTEGRAL_MAX_ERROR_MM) ||
        (controller->ball_speed_mm_s <
         -BALL_BEAM_INTEGRAL_MAX_SPEED_MM_S) ||
        (controller->ball_speed_mm_s >
         BALL_BEAM_INTEGRAL_MAX_SPEED_MM_S)) {
        controller->integral_output_deg = 0.0f;
        return;
    }

    candidate_integral_deg =
        controller->integral_output_deg +
        controller->ki_deg_per_mm_s *
            error_mm *
            ((float) frame_period_ms / 1000.0f);
    candidate_integral_deg =
        BallBeamController_ClampFloat(
            candidate_integral_deg,
            -BALL_BEAM_INTEGRAL_MAX_OUTPUT_DEG,
            BALL_BEAM_INTEGRAL_MAX_OUTPUT_DEG);

    predicted_error_mm =
        BallBeamController_GetPredictedPositionMm(controller) -
        controller->target_x_mm;
    base_output_deg =
        controller->kp_deg_per_mm * predicted_error_mm +
        controller->kd_deg_per_mm_s *
            controller->ball_speed_mm_s;
    candidate_output_deg =
        base_output_deg + candidate_integral_deg;

    /* 输出已经饱和且误差仍在同方向时，不再继续积累。 */
    if (((candidate_output_deg > BALL_BEAM_MAX_ANGLE_DEG) &&
         (error_mm > 0.0f)) ||
        ((candidate_output_deg < -BALL_BEAM_MAX_ANGLE_DEG) &&
         (error_mm < 0.0f))) {
        return;
    }

    controller->integral_output_deg =
        candidate_integral_deg;
}

static uint8_t BallBeamController_ParseU32(const char **cursor,
                                           uint32_t *value)
{
    const char *text;
    uint32_t parsed_value = 0UL;
    uint32_t digit;

    if ((cursor == NULL) || (*cursor == NULL) || (value == NULL) ||
        (**cursor < '0') || (**cursor > '9')) {
        return 0U;
    }

    text = *cursor;
    while ((*text >= '0') && (*text <= '9')) {
        digit = (uint32_t) (*text - '0');
        if (parsed_value >
            ((UINT32_MAX - digit) / 10UL)) {
            return 0U;
        }
        parsed_value = parsed_value * 10UL + digit;
        text++;
    }

    *cursor = text;
    *value = parsed_value;
    return 1U;
}

static uint8_t BallBeamController_ParseFloat(const char **cursor,
                                             float *value)
{
    char *end;
    float parsed_value;

    if ((cursor == NULL) || (*cursor == NULL) || (value == NULL)) {
        return 0U;
    }

    parsed_value = strtof(*cursor, &end);
    if ((end == *cursor) || (parsed_value != parsed_value) ||
        (parsed_value < -BALL_BEAM_MAX_MEASUREMENT_ABS_MM) ||
        (parsed_value > BALL_BEAM_MAX_MEASUREMENT_ABS_MM)) {
        return 0U;
    }

    *cursor = end;
    *value = parsed_value;
    return 1U;
}

static uint8_t BallBeamController_ParseStandardFrame(
    const char *line,
    uint32_t *frame_id,
    uint32_t *camera_timestamp_ms,
    float *raw_x_mm,
    uint32_t *valid_value)
{
    const char *cursor =
        line + BALL_BEAM_FRAME_PREFIX_LENGTH;

    if ((BallBeamController_ParseU32(&cursor, frame_id) == 0U) ||
        (*cursor != ',')) {
        return 0U;
    }
    cursor++;

    if ((BallBeamController_ParseU32(
             &cursor, camera_timestamp_ms) == 0U) ||
        (*cursor != ',')) {
        return 0U;
    }
    cursor++;

    if ((BallBeamController_ParseFloat(&cursor, raw_x_mm) == 0U) ||
        (*cursor != ',')) {
        return 0U;
    }
    cursor++;

    if ((BallBeamController_ParseU32(&cursor, valid_value) == 0U) ||
        (*cursor != '\0') || (*valid_value > 1UL)) {
        return 0U;
    }

    return 1U;
}

static uint8_t BallBeamController_ParsePositionCmFrame(
    const char *line,
    float *raw_x_mm)
{
    const char *cursor =
        line + BALL_BEAM_POSITION_CM_PREFIX_LENGTH;
    float raw_x_cm;

    if ((BallBeamController_ParseFloat(&cursor, &raw_x_cm) == 0U) ||
        (strcmp(cursor, "cm") != 0)) {
        return 0U;
    }

    *raw_x_mm = raw_x_cm * BALL_BEAM_CM_TO_MM;
    if ((*raw_x_mm < -BALL_BEAM_MAX_MEASUREMENT_ABS_MM) ||
        (*raw_x_mm > BALL_BEAM_MAX_MEASUREMENT_ABS_MM)) {
        return 0U;
    }

    return 1U;
}

void BallBeamController_Init(BallBeamController *controller)
{
    if (controller == NULL) {
        return;
    }

    memset(controller, 0, sizeof(*controller));
    controller->kp_deg_per_mm =
        BALL_BEAM_DEFAULT_KP_DEG_PER_MM;
    controller->ki_deg_per_mm_s =
        BALL_BEAM_DEFAULT_KI_DEG_PER_MM_S;
    controller->kd_deg_per_mm_s =
        BALL_BEAM_DEFAULT_KD_DEG_PER_MM_S;
    controller->prediction_delay_ms =
        BALL_BEAM_DEFAULT_PREDICTION_DELAY_MS;
    controller->stale_timeout_ms =
        BALL_BEAM_DEFAULT_STALE_TIMEOUT_MS;
    controller->level_trim_deg =
        BALL_BEAM_DEFAULT_LEVEL_TRIM_DEG;
    controller->profile_max_speed_mm_s =
        BALL_BEAM_PROFILE_MAX_SPEED_MM_S;
    controller->profile_position_rate_per_s =
        BALL_BEAM_PROFILE_POSITION_RATE_PER_S;
    /* 实机正方向：电机顺时针、齿条上升、横梁右端抬高。 */
    controller->output_sign = 1;
    controller->stale = 1U;
}

uint8_t BallBeamController_ProcessCameraLine(
    BallBeamController *controller,
    const char *line,
    uint32_t receive_ms)
{
    uint32_t frame_id;
    uint32_t camera_timestamp_ms;
    uint32_t valid_value;
    uint32_t frame_period_ms;
    uint32_t speed_period_ms;
    float raw_x_mm;
    float previous_filtered_x_mm;
    float raw_speed_mm_s;
    float speed_position_delta_mm;
    uint8_t uses_mcu_timestamp = 0U;
    uint8_t speed_period_valid = 0U;

    if ((controller == NULL) || (line == NULL)) {
        return 0U;
    }

    if (strncmp(line,
                BALL_BEAM_FRAME_PREFIX,
                BALL_BEAM_FRAME_PREFIX_LENGTH) == 0) {
        if (BallBeamController_ParseStandardFrame(
                line,
                &frame_id,
                &camera_timestamp_ms,
                &raw_x_mm,
                &valid_value) == 0U) {
            controller->rejected_frame_count++;
            return 0U;
        }
    } else if (strncmp(line,
                       BALL_BEAM_POSITION_CM_PREFIX,
                       BALL_BEAM_POSITION_CM_PREFIX_LENGTH) == 0) {
        if (BallBeamController_ParsePositionCmFrame(
                line, &raw_x_mm) == 0U) {
            controller->rejected_frame_count++;
            return 0U;
        }

        /*
         * 简化协议没有帧号和摄像头时间戳。每条完整串口行视为新帧，
         * 使用 MSPM0 收帧时刻计算 dt，精度低于标准协议但可以闭环测试。
         */
        frame_id = (controller->has_camera_frame != 0U) ?
            (controller->camera_frame_id + 1UL) : 1UL;
        camera_timestamp_ms = receive_ms;
        valid_value = 1UL;
        uses_mcu_timestamp = 1U;
    } else {
        return 0U;
    }

    if ((uses_mcu_timestamp == 0U) &&
        (controller->has_camera_frame != 0U) &&
        (frame_id == controller->camera_frame_id)) {
        controller->duplicate_frame_count++;
        return 0U;
    }

    controller->camera_frame_id = frame_id;
    controller->camera_timestamp_ms = camera_timestamp_ms;
    controller->has_camera_frame = 1U;
    controller->parsed_frame_count++;

    if (valid_value == 0UL) {
        controller->previous_camera_timestamp_ms = camera_timestamp_ms;
        if (controller->dropout_consecutive_count < UINT8_MAX) {
            controller->dropout_consecutive_count++;
        }
        if (BallBeamController_ApplyDropoutPrediction(
                controller, receive_ms) != 0U) {
            return 1U;
        }

        controller->dropout_fault_count++;
        controller->measurement_valid = 0U;
        controller->has_previous_sample = 0U;
        controller->has_speed_reference = 0U;
        controller->last_frame_period_ms = 0UL;
        controller->last_speed_period_ms = 0UL;
        controller->stale = 1U;
        BallBeamController_SetSafeOutput(controller);
        return 1U;
    }

    controller->raw_x_mm = raw_x_mm;
    if (controller->dropout_active != 0U) {
        controller->dropout_recovery_count++;
    }
    controller->dropout_active = 0U;
    controller->dropout_consecutive_count = 0U;

    if (controller->has_previous_sample == 0U) {
        controller->filtered_x_mm = raw_x_mm;
        controller->ball_speed_mm_s = 0.0f;
        controller->speed_reference_x_mm =
            controller->filtered_x_mm;
        controller->speed_reference_timestamp_ms =
            camera_timestamp_ms;
        controller->last_frame_period_ms = 0UL;
        controller->last_speed_period_ms = 0UL;
        controller->has_previous_sample = 1U;
        controller->has_speed_reference = 1U;
    } else {
        frame_period_ms =
            camera_timestamp_ms -
            controller->previous_camera_timestamp_ms;
        controller->last_frame_period_ms = frame_period_ms;
        previous_filtered_x_mm = controller->filtered_x_mm;
        controller->filtered_x_mm =
            previous_filtered_x_mm +
            BALL_BEAM_POSITION_FILTER_ALPHA *
            (raw_x_mm - previous_filtered_x_mm);

        /*
         * Kick 只负责克服静摩擦。相对测速基准已移动 1.5 mm 时立即撤销，
         * 不等待 75 ms 球速窗口完成，避免起滚后额外坡度继续把球推过头。
         * 若球稍后又在目标前停住，静止计时会重新开始并再次尝试起滚。
         */
        if ((controller->breakaway_active != 0U) &&
            (BallBeamController_AbsFloat(
                 controller->filtered_x_mm -
                 controller->speed_reference_x_mm) >=
             BALL_BEAM_BREAKAWAY_RELEASE_MOVE_MM)) {
            BallBeamController_ResetBreakaway(controller);
        }

        if (controller->has_speed_reference == 0U) {
            controller->speed_reference_x_mm =
                controller->filtered_x_mm;
            controller->speed_reference_timestamp_ms =
                camera_timestamp_ms;
            controller->has_speed_reference = 1U;
        } else {
            speed_period_ms =
                camera_timestamp_ms -
                controller->speed_reference_timestamp_ms;

            /*
             * 40 FPS 摄像头的毫米坐标会产生明显量化速度。累计约三帧
             * 再估算速度，并把 1 mm 内的位置变化视为静止，避免假速度
             * 反复开关制动和静摩擦补偿。
             */
            if (speed_period_ms >=
                BALL_BEAM_SPEED_ESTIMATION_PERIOD_MS) {
                controller->last_speed_period_ms =
                    speed_period_ms;

                if (speed_period_ms <=
                    BALL_BEAM_SPEED_RESET_PERIOD_MS) {
                    speed_position_delta_mm =
                        controller->filtered_x_mm -
                        controller->speed_reference_x_mm;
                    if (BallBeamController_AbsFloat(
                            speed_position_delta_mm) <=
                        BALL_BEAM_SPEED_POSITION_DEADBAND_MM) {
                        raw_speed_mm_s = 0.0f;
                    } else {
                        raw_speed_mm_s =
                            speed_position_delta_mm *
                            (1000.0f /
                             (float) speed_period_ms);
                    }
                    controller->ball_speed_mm_s +=
                        BALL_BEAM_SPEED_FILTER_ALPHA *
                        (raw_speed_mm_s -
                         controller->ball_speed_mm_s);
                    speed_period_valid = 1U;
                } else {
                    /*
                     * 长时间断帧后不能沿用旧速度；以本帧重建速度基准，
                     * 位置 P 项仍可立即恢复。
                     */
                    controller->timing_fault_count++;
                    controller->ball_speed_mm_s = 0.0f;
                    BallBeamController_ResetBreakaway(controller);
                }

                controller->speed_reference_x_mm =
                    controller->filtered_x_mm;
                controller->speed_reference_timestamp_ms =
                    camera_timestamp_ms;
            }
        }
    }

    controller->previous_camera_timestamp_ms =
        camera_timestamp_ms;
    controller->last_valid_filtered_x_mm =
        controller->filtered_x_mm;
    controller->last_valid_receive_ms = receive_ms;
    controller->measurement_valid = 1U;
    controller->stale = 0U;
    if (speed_period_valid != 0U) {
        BallBeamController_UpdateBreakaway(
            controller, speed_period_ms);
        BallBeamController_UpdateIntegral(
            controller, speed_period_ms);
    }
    BallBeamController_Recalculate(controller);
    return 1U;
}

void BallBeamController_Update(BallBeamController *controller,
                               uint32_t now_ms)
{
    uint32_t valid_age_ms;

    if (controller == NULL) {
        return;
    }

    valid_age_ms = now_ms - controller->last_valid_receive_ms;
    if ((controller->measurement_valid == 0U) ||
        (valid_age_ms > controller->stale_timeout_ms)) {
        if (controller->dropout_active != 0U) {
            controller->dropout_fault_count++;
        }
        controller->stale = 1U;
        controller->has_previous_sample = 0U;
        controller->has_speed_reference = 0U;
        controller->last_speed_period_ms = 0UL;
        BallBeamController_SetSafeOutput(controller);
        return;
    }

    /* 摄像头没有发送 valid=0、而是整帧缺失时，也跨过短暂断帧。 */
    if (valid_age_ms > BALL_BEAM_DROPOUT_PREDICTION_START_MS) {
        (void) BallBeamController_ApplyDropoutPrediction(
            controller, now_ms);
    }

    controller->stale = 0U;
}

uint8_t BallBeamController_SetStaleTimeoutMs(
    BallBeamController *controller,
    uint32_t stale_timeout_ms)
{
    if ((controller == NULL) ||
        (stale_timeout_ms < BALL_BEAM_MIN_STALE_TIMEOUT_MS) ||
        (stale_timeout_ms > BALL_BEAM_MAX_STALE_TIMEOUT_MS)) {
        return 0U;
    }

    controller->stale_timeout_ms = stale_timeout_ms;
    return 1U;
}

void BallBeamController_SetEnabled(BallBeamController *controller,
                                   uint8_t enabled)
{
    if (controller == NULL) {
        return;
    }

    controller->enabled = (enabled != 0U) ? 1U : 0U;
    if (controller->enabled == 0U) {
        BallBeamController_SetSafeOutput(controller);
    } else {
        BallBeamController_Recalculate(controller);
    }
}

uint8_t BallBeamController_IsEnabled(
    const BallBeamController *controller)
{
    return ((controller != NULL) &&
            (controller->enabled != 0U)) ? 1U : 0U;
}

uint8_t BallBeamController_IsControlReady(
    const BallBeamController *controller)
{
    return ((controller != NULL) &&
            (controller->measurement_valid != 0U) &&
            (controller->stale == 0U)) ? 1U : 0U;
}

uint8_t BallBeamController_SetTargetPositionMm(
    BallBeamController *controller,
    float target_x_mm)
{
    if ((controller == NULL) || (target_x_mm != target_x_mm) ||
        (target_x_mm < -BALL_BEAM_TARGET_MAX_ABS_MM) ||
        (target_x_mm > BALL_BEAM_TARGET_MAX_ABS_MM)) {
        return 0U;
    }

    controller->target_x_mm = target_x_mm;
    controller->integral_output_deg = 0.0f;
    controller->external_feedforward_deg = 0.0f;
    BallBeamController_ResetBreakaway(controller);
    BallBeamController_Recalculate(controller);
    return 1U;
}

uint8_t BallBeamController_SetMotionProfile(
    BallBeamController *controller,
    float max_speed_mm_s,
    float position_rate_per_s)
{
    if ((controller == NULL) ||
        (max_speed_mm_s != max_speed_mm_s) ||
        (position_rate_per_s != position_rate_per_s) ||
        (max_speed_mm_s < BALL_BEAM_PROFILE_MIN_SPEED_MM_S) ||
        (max_speed_mm_s > BALL_BEAM_PROFILE_MAX_ALLOWED_SPEED_MM_S) ||
        (position_rate_per_s <
         BALL_BEAM_PROFILE_MIN_POSITION_RATE_PER_S) ||
        (position_rate_per_s >
         BALL_BEAM_PROFILE_MAX_POSITION_RATE_PER_S)) {
        return 0U;
    }

    controller->profile_max_speed_mm_s = max_speed_mm_s;
    controller->profile_position_rate_per_s = position_rate_per_s;
    BallBeamController_Recalculate(controller);
    return 1U;
}

uint8_t BallBeamController_SetPIDGains(
    BallBeamController *controller,
    float kp_deg_per_mm,
    float ki_deg_per_mm_s,
    float kd_deg_per_mm_s)
{
    if ((controller == NULL) ||
        (kp_deg_per_mm != kp_deg_per_mm) ||
        (ki_deg_per_mm_s != ki_deg_per_mm_s) ||
        (kd_deg_per_mm_s != kd_deg_per_mm_s) ||
        (kp_deg_per_mm < 0.0f) ||
        (kp_deg_per_mm > BALL_BEAM_MAX_KP_DEG_PER_MM) ||
        (ki_deg_per_mm_s < 0.0f) ||
        (ki_deg_per_mm_s > BALL_BEAM_MAX_KI_DEG_PER_MM_S) ||
        (kd_deg_per_mm_s < 0.0f) ||
        (kd_deg_per_mm_s > BALL_BEAM_MAX_KD_DEG_PER_MM_S)) {
        return 0U;
    }

    controller->kp_deg_per_mm = kp_deg_per_mm;
    controller->ki_deg_per_mm_s = ki_deg_per_mm_s;
    controller->kd_deg_per_mm_s = kd_deg_per_mm_s;
    controller->integral_output_deg = 0.0f;
    BallBeamController_ResetBreakaway(controller);
    BallBeamController_Recalculate(controller);
    return 1U;
}

uint8_t BallBeamController_SetGains(BallBeamController *controller,
                                    float kp_deg_per_mm,
                                    float kd_deg_per_mm_s)
{
    return BallBeamController_SetPIDGains(
        controller, kp_deg_per_mm, 0.0f, kd_deg_per_mm_s);
}

uint8_t BallBeamController_SetPredictionDelayMs(
    BallBeamController *controller,
    uint32_t prediction_delay_ms)
{
    if ((controller == NULL) ||
        (prediction_delay_ms >
         BALL_BEAM_MAX_PREDICTION_DELAY_MS)) {
        return 0U;
    }

    controller->prediction_delay_ms = prediction_delay_ms;
    BallBeamController_Recalculate(controller);
    return 1U;
}

uint8_t BallBeamController_SetLevelTrimDeg(
    BallBeamController *controller,
    float level_trim_deg)
{
    if ((controller == NULL) ||
        (level_trim_deg != level_trim_deg) ||
        (level_trim_deg < -BALL_BEAM_MAX_LEVEL_TRIM_DEG) ||
        (level_trim_deg > BALL_BEAM_MAX_LEVEL_TRIM_DEG)) {
        return 0U;
    }

    controller->level_trim_deg = level_trim_deg;
    BallBeamController_ResetBreakaway(controller);
    BallBeamController_Recalculate(controller);
    return 1U;
}

uint8_t BallBeamController_SetExternalFeedforwardDeg(
    BallBeamController *controller,
    float feedforward_deg)
{
    if ((controller == NULL) ||
        (feedforward_deg != feedforward_deg) ||
        (feedforward_deg < -BALL_BEAM_MAX_EXTERNAL_FEEDFORWARD_DEG) ||
        (feedforward_deg > BALL_BEAM_MAX_EXTERNAL_FEEDFORWARD_DEG)) {
        return 0U;
    }

    if (BallBeamController_AbsFloat(
            controller->external_feedforward_deg -
            feedforward_deg) < 0.001f) {
        return 1U;
    }

    controller->external_feedforward_deg = feedforward_deg;
    BallBeamController_Recalculate(controller);
    return 1U;
}

uint8_t BallBeamController_SetOutputSign(
    BallBeamController *controller,
    int8_t output_sign)
{
    if ((controller == NULL) ||
        ((output_sign != 1) && (output_sign != -1))) {
        return 0U;
    }

    controller->output_sign = output_sign;
    BallBeamController_ResetBreakaway(controller);
    BallBeamController_Recalculate(controller);
    return 1U;
}

int32_t BallBeamController_ConvertBeamAngleToPulses(
    const BallBeamController *controller,
    float beam_angle_deg)
{
    float command_angle_deg;

    if ((controller == NULL) ||
        (beam_angle_deg != beam_angle_deg)) {
        return 0;
    }

    /*
     * 通用闭环仍由 4 度限位保护；直接角度接口额外保留 0.5 度，仅供
     * Q3 已确认静止时的短时脱困使用，避免转换层再次截断 4.5 度命令。
     */
    command_angle_deg = BallBeamController_ClampFloat(
        beam_angle_deg + controller->level_trim_deg,
        -BALL_BEAM_MAX_DIRECT_ANGLE_DEG,
        BALL_BEAM_MAX_DIRECT_ANGLE_DEG);
    command_angle_deg *= (float) controller->output_sign;

    return BallBeamController_AngleToPulses(command_angle_deg);
}

int32_t BallBeamController_GetTargetStepperPulses(
    const BallBeamController *controller)
{
    return (controller != NULL) ?
        controller->target_stepper_pulses : 0;
}
