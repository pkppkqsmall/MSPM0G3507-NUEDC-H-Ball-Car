#include "map_run.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ball_beam_controller.h"
#include "bno085.h"
#include "camera_link.h"
#include "clock.h"
#include "interrupt.h"
#include "map_curve_hold.h"
#include "map_line_controller.h"
#include "map_yaw_rate_controller.h"
#include "motor_pwm.h"
#include "oled_hardware_i2c.h"
#include "Sensor.h"
#include "stepper_motor.h"
#include "ti_msp_dl_config.h"

/*
 * 题目地图运行参数：灰度位置外环、BNO085 角速度内环，
 * 最内层仍由左右轮速度 PI 跟随目标 RPM。
 */
#define MAP_RUN_Q2_BASE_SPEED_RPM                (120.0f)
#define MAP_RUN_Q2_BRAKE_HOLD_MS                 (100UL)
#define MAP_RUN_Q3_START_CENTER_LIMIT_MM            (15.0f)
#define MAP_RUN_Q3_POSITIVE_TARGET_MIN_MM               (40.0f)
#define MAP_RUN_Q3_POSITIVE_SWITCH_MARGIN_MM              (1.0f)
#define MAP_RUN_Q3_POSITIVE_PREDICTION_S                 (0.85f)
#define MAP_RUN_Q3_POSITIVE_PREDICTION_MAX_SPEED_MM_S   (80.0f)
#define MAP_RUN_Q3_POSITIVE_RETURN_MIN_SPEED_MM_S      (5.0f)
#define MAP_RUN_Q3_NEGATIVE_HOLD_MM                 (-50.0f)
#define MAP_RUN_Q3_POSITIVE_DRIVE_ANGLE_DEG             (-4.50f)
#define MAP_RUN_Q3_LEFT_DRIVE_ANGLE_DEG                (3.0f)
#define MAP_RUN_Q3_NEGATIVE_BRAKE_TRIGGER_MM            (20.0f)
#define MAP_RUN_Q3_NEGATIVE_BRAKE_MIN_SPEED_MM_S      (-100.0f)
#define MAP_RUN_Q3_CATCH_BALANCE_BIAS_DEG               (1.8f)
#define MAP_RUN_Q3_CATCH_FRAME_SPEED_BLEND              (0.75f)
#define MAP_RUN_Q3_CATCH_FRAME_DELTA_MIN_MM              (0.5f)
#define MAP_RUN_Q3_CATCH_MAX_SPEED_MM_S                (250.0f)
#define MAP_RUN_Q3_CATCH_LOW_SPEED_MM_S                 (12.0f)
#define MAP_RUN_Q3_CATCH_REBOUND_CONFIRM_MM_S             (5.0f)
#define MAP_RUN_Q3_CATCH_LOW_SPEED_CONFIRM_MS           (75UL)
#define MAP_RUN_Q3_PRE_CATCH_TRIGGER_MM                  (-35.0f)
#define MAP_RUN_Q3_PRE_CATCH_MAX_SPEED_MM_S               (70.0f)
#define MAP_RUN_Q3_PRE_CATCH_ANGLE_DEG                     (2.2f)
#define MAP_RUN_Q3_CATCH_PULSE_MIN_MS                       (100UL)
#define MAP_RUN_Q3_CAPTURE_MAX_EXPECTED_SPEED_MM_S           (60.0f)
#define MAP_RUN_Q3_CAPTURE_POSITION_RATE_PER_S                (1.5f)
#define MAP_RUN_Q3_CAPTURE_SPEED_KD_DEG_PER_MM_S              (0.040f)
#define MAP_RUN_Q3_CAPTURE_MAX_CORRECTION_DEG                  (2.4f)
#define MAP_RUN_Q3_TERMINAL_HANDOFF_MIN_MM                  (-30.0f)
#define MAP_RUN_Q3_BRAKE_DISTANCE_MM                   (70.0f)
#define MAP_RUN_Q3_BRAKE_SPEED_MARGIN_MM_S             (40.0f)
#define MAP_RUN_Q3_ROLL_ACCEL_M_S2_PER_RAD              (7.007f)
#define MAP_RUN_Q3_RAD_TO_DEG                          (57.29578f)
#define MAP_RUN_Q3_BRAKE_MODEL_GAIN                     (3.00f)
#define MAP_RUN_Q3_BRAKE_MIN_ANGLE_DEG                  (1.5f)
#define MAP_RUN_Q3_BRAKE_MAX_ANGLE_DEG                  (3.0f)
#define MAP_RUN_Q3_HOLD_KP_DEG_PER_MM                     (0.060f)
#define MAP_RUN_Q3_HOLD_KI_DEG_PER_MM_S                   (0.050f)
#define MAP_RUN_Q3_HOLD_KD_DEG_PER_MM_S                   (0.040f)
#define MAP_RUN_Q3_HOLD_PROFILE_MAX_SPEED_MM_S             (35.0f)
#define MAP_RUN_Q3_HOLD_PROFILE_POSITION_RATE_PER_S         (1.20f)
#define MAP_RUN_Q3_HOLD_PROFILE_BLEND_START_MM               (6.0f)
#define MAP_RUN_Q3_HOLD_PROFILE_BLEND_FULL_MM               (15.0f)
#define MAP_RUN_Q3_HOLD_STICTION_ERROR_MIN_MM                (8.0f)
#define MAP_RUN_Q3_HOLD_STICTION_SPEED_MAX_MM_S               (6.0f)
#define MAP_RUN_Q3_HOLD_STICTION_FRAME_DELTA_MAX_MM            (0.5f)
#define MAP_RUN_Q3_HOLD_STICTION_BREAKAWAY_ANGLE_DEG            (4.5f)
#define MAP_RUN_Q3_HOLD_STICTION_CONFIRM_MS                  (150UL)
#define MAP_RUN_Q3_HOLD_STICTION_CONFIRM_TRAVEL_MM             (1.0f)
#define MAP_RUN_Q3_HOLD_STICTION_KICK_MS                     (100UL)
#define MAP_RUN_Q3_HOLD_STICTION_REARM_MS                    (250UL)
#define MAP_RUN_Q3_HOLD_STICTION_RELEASE_SPEED_MM_S             (8.0f)
#define MAP_RUN_Q3_HOLD_INITIAL_BIAS_DEG                  (1.8f)
#define MAP_RUN_Q3_HOLD_MAX_BIAS_DEG                      (2.0f)
#define MAP_RUN_Q3_HOLD_MAX_ANGLE_DEG                     (3.5f)
#define MAP_RUN_Q3_HOLD_STICTION_MAX_ANGLE_DEG            (4.5f)
#define MAP_RUN_Q3_HOLD_I_MAX_ERROR_MM                   (25.0f)
#define MAP_RUN_Q3_HOLD_I_MAX_SPEED_MM_S                 (10.0f)
#define MAP_RUN_Q3_HOLD_POSITION_DEADBAND_MM              (1.0f)
#define MAP_RUN_Q3_HOLD_SPEED_DEADBAND_MM_S               (2.0f)
#define MAP_RUN_Q3_HOLD_DIRECTION_DELTA_MM                 (0.25f)
#define MAP_RUN_Q3_HOLD_FRAME_SPEED_DELTA_MM               (0.50f)
#define MAP_RUN_Q3_HOLD_FRAME_SPEED_BLEND                  (0.75f)
#define MAP_RUN_Q3_HOLD_FRAME_PERIOD_MS                   (25.0f)
#define MAP_RUN_Q3_HOLD_MAX_FRAME_SPEED_MM_S              (80.0f)
#define MAP_RUN_Q3_HOLD_MAX_FRAME_MS                    (100UL)
#define MAP_RUN_Q3_HOLD_ANGLE_FILTER_ALPHA                 (0.50f)
#define MAP_RUN_Q3_HOLD_MAX_ANGLE_STEP_DEG                  (0.45f)
#define MAP_RUN_Q3_HOLD_ENTRY_ESCAPE_SPEED_MM_S            (15.0f)
#define MAP_RUN_Q3_HOLD_ENTRY_APPROACH_SPEED_MM_S          (30.0f)
#define MAP_RUN_Q3_HOLD_ENTRY_BRAKE_DISTANCE_MM            (25.0f)
#define MAP_RUN_Q3_HOLD_ENTRY_BRAKE_WINDOW_MS             (5000UL)
#define MAP_RUN_Q3_HOLD_ENTRY_BRAKE_ANGLE_DEG                (4.5f)
#define MAP_RUN_Q3_HOLD_STALE_BIAS_DEG                    (0.8f)
#define MAP_RUN_Q3_SETTLE_MIN_POSITION_MM             (-60.0f)
#define MAP_RUN_Q3_SETTLE_MAX_POSITION_MM             (-40.0f)
#define MAP_RUN_Q3_SETTLE_CONFIRM_MS                   (100UL)
#define MAP_RUN_Q3_SETTLE_CONFIRM_MAX_DELTA_MM           (1.0f)
#define MAP_RUN_Q3_SETTLE_VERIFY_MS                    (250UL)
#define MAP_RUN_Q3_SETTLE_VERIFY_MAX_DELTA_MM           (2.0f)
#define MAP_RUN_Q3_SETTLE_MAX_SPEED_MM_S                (8.0f)
#define MAP_RUN_Q3_SETTLE_FINISH_MAX_SPEED_MM_S         (4.0f)
#define MAP_RUN_Q3_SETTLE_MAX_FRAME_DELTA_MM             (1.0f)
#define MAP_RUN_Q3_STALE_TIMEOUT_MS                    (250UL)
#define MAP_RUN_Q4_STALE_TIMEOUT_MS                    (300UL)
#define MAP_RUN_BALL_SAFE_MAX_SPEED_MM_S              (60.0f)
#define MAP_RUN_BALL_SAFE_POSITION_RATE_PER_S          (1.2f)
#define MAP_RUN_Q3_TIME_LIMIT_MS                    (5000UL)
#define MAP_RUN_Q4_BASE_SPEED_RPM                 (88.0f)
#define MAP_RUN_Q4_START_SPEED_RPM                (30.0f)
#define MAP_RUN_Q4_START_HOLD_MS                  (200UL)
#define MAP_RUN_Q4_START_RAMP_MS                 (1400UL)
#define MAP_RUN_Q4_START_FF_PEAK_DEG              (1.30f)
#define MAP_RUN_Q4_START_PRELOAD_DEG               (0.85f)
#define MAP_RUN_Q4_START_PRELOAD_RAMP_MS          (180UL)
#define MAP_RUN_Q4_START_PRELOAD_FADE_MS          (600UL)
#define MAP_RUN_Q4_START_FF_RELEASE_SPEED_MM_S    (10.0f)
#define MAP_RUN_Q4_START_FF_RELEASE_MIN_X_MM        (-2.0f)
#define MAP_RUN_Q4_BRAKE_FF_PEAK_DEG              (0.70f)
#define MAP_RUN_Q4_FINAL_STOP_RAMP_MS             (900UL)
#define MAP_RUN_Q4_FINAL_STOP_FF_PEAK_DEG           (0.45f)
#define MAP_RUN_Q4_STOP_FF_DEG                      (0.80f)
#define MAP_RUN_Q4_STOP_FF_HOLD_MS                (300UL)
#define MAP_RUN_Q4_STOP_FF_FADE_MS                (500UL)
#define MAP_RUN_Q4_STOP_FF_RELEASE_SPEED_MM_S      (12.0f)
#define MAP_RUN_Q4_STOP_FF_DISABLE_X_MM             (-8.0f)
#define MAP_RUN_Q4_POSITION_GUARD_DEADBAND_MM       (2.0f)
#define MAP_RUN_Q4_POSITION_GUARD_GAIN_DEG_PER_MM   (0.055f)
#define MAP_RUN_Q4_POSITION_GUARD_MAX_DEG           (0.75f)
#define MAP_RUN_Q4_PARK_GUARD_DEADBAND_MM           (10.0f)
#define MAP_RUN_Q4_PARK_GUARD_GAIN_DEG_PER_MM        (0.10f)
#define MAP_RUN_Q4_PARK_GUARD_MAX_DEG                (1.00f)
#define MAP_RUN_Q4_PARK_GUARD_MAX_SPEED_MM_S         (6.0f)
#define MAP_RUN_Q4_VELOCITY_WINDOW_MIN_MS           (35UL)
#define MAP_RUN_Q4_VELOCITY_WINDOW_MAX_MS          (140UL)
#define MAP_RUN_Q4_VELOCITY_DELTA_DEADBAND_MM        (0.35f)
#define MAP_RUN_Q4_VELOCITY_FILTER_ALPHA             (0.65f)
#define MAP_RUN_Q4_VELOCITY_FAST_BLEND               (0.80f)
#define MAP_RUN_Q4_VELOCITY_REVERSAL_MIN_MM_S         (5.0f)
#define MAP_RUN_Q4_VELOCITY_MAX_MM_S                (120.0f)
#define MAP_RUN_Q4_VELOCITY_DAMPING_GAIN             (0.018f)
#define MAP_RUN_Q4_VELOCITY_DAMPING_MAX_DEG          (0.90f)
#define MAP_RUN_Q4_EXTERNAL_FF_MAX_DEG              (1.50f)
#define MAP_RUN_Q4_RECOVERY_MAX_SPEED_MM_S          (15.0f)
#define MAP_RUN_Q4_RECOVERY_POSITION_RATE_PER_S      (0.50f)
#define MAP_RUN_Q4_PARK_MAX_SPEED_MM_S              (15.0f)
#define MAP_RUN_Q4_PARK_POSITION_RATE_PER_S           (0.5f)
#define MAP_RUN_BALL_BASE_SPEED_RPM               (85.0f)
#define MAP_RUN_BALL_START_SPEED_RPM              (30.0f)
#define MAP_RUN_BALL_START_HOLD_MS                (200UL)
#define MAP_RUN_BALL_START_RAMP_MS               (1400UL)
#define MAP_RUN_BALL_START_FF_PEAK_DEG              (1.55f)
#define MAP_RUN_BALL_START_PRELOAD_DEG               (1.05f)
#define MAP_RUN_BALL_START_PRELOAD_RAMP_MS          (180UL)
#define MAP_RUN_BALL_START_PRELOAD_FADE_MS          (600UL)
#define MAP_RUN_BALL_START_EXTERNAL_FF_MAX_DEG        (2.20f)
#define MAP_RUN_BALL_START_FF_RELEASE_SPEED_MM_S     (10.0f)
#define MAP_RUN_BALL_START_FF_RELEASE_MIN_ERROR_MM    (-2.0f)
#define MAP_RUN_BALL_FINAL_STOP_RAMP_MS              (500UL)
#define MAP_RUN_BALL_FINAL_STOP_FF_PEAK_DEG            (0.80f)
#define MAP_RUN_BALL_TARGET_POSITION_MM_Q5           (0.0f)
#define MAP_RUN_BALL_START_TARGET_LIMIT_MM           (10.0f)
#define MAP_RUN_BALL_ALLOWED_ERROR_MM                 (10.0f)
#define MAP_RUN_BALL_TIME_LIMIT_MS                 (30000UL)
#define MAP_RUN_BALL_STALE_TIMEOUT_MS                (300UL)
#define MAP_RUN_BALL_RECOVERY_MAX_SPEED_MM_S          (20.0f)
#define MAP_RUN_BALL_RECOVERY_POSITION_RATE_PER_S       (0.75f)
#define MAP_RUN_BALL_PARK_MAX_SPEED_MM_S              (15.0f)
#define MAP_RUN_BALL_PARK_POSITION_RATE_PER_S           (0.50f)
#define MAP_RUN_BALL_LINE_REFERENCE_RPM           (80.0f)
#define MAP_RUN_BALL_CENTER_RECOVERY_RPM_PER_STEP  (1.0f)
#define MAP_RUN_BALL_MIN_TARGET_RPM                (25.0f)
#define MAP_RUN_BALL_MAX_OUTER_TARGET_RPM          (95.0f)
#define MAP_RUN_BALL_TARGET_RISE_RPM_PER_STEP       (3.0f)
#define MAP_RUN_BALL_OUTER_ERROR_START              (2.0f)
#define MAP_RUN_BALL_OUTER_ERROR_GAIN               (1.5f)
#define MAP_RUN_SPEED_TEST_DEFAULT_RPM           (50.0f)
#define MAP_RUN_LOW_TARGET_STOP_RPM               (30.0f)
#define MAP_RUN_CENTER_RECOVERY_RPM_PER_STEP       (2.0f)
#define MAP_RUN_STABLE_CENTER_ERROR_LIMIT          (2.1f)
#define MAP_RUN_STABLE_CENTER_ERROR_SCALE          (0.5f)
#define MAP_RUN_BALL_CENTER_ERROR_SCALE            (0.25f)

#define MAP_RUN_CONTROL_PERIOD_MS                (10UL)
#define MAP_RUN_SPEED_PERIOD_MS                  (20UL)
#define MAP_RUN_TELEMETRY_PERIOD_MS              (50UL)
#define MAP_RUN_BALL_LOG_PERIOD_MS               (100UL)
#define MAP_RUN_BALL_LAP_TELEMETRY_PERIOD_MS     (250UL)
#define MAP_RUN_BALL_POST_FINISH_TELEMETRY_MS   (5000UL)
#define MAP_RUN_DISPLAY_PERIOD_MS                (100UL)

/*
 * 调试蓝牙自动刷屏开关。关闭只影响主动推送，不影响摄像头解析、
 * 滚球控制以及 BALL?/CAM?/CAR? 等查询命令。
 */
#define MAP_RUN_AUTO_SPEED_TELEMETRY_ENABLED      (0U)
#define MAP_RUN_AUTO_CAMERA_FORWARD_ENABLED       (0U)

#define MAP_RUN_BUTTON_DEBOUNCE_MS               (30UL)
#define MAP_RUN_BUTTON_LONG_PRESS_MS             (1500UL)
#define MAP_RUN_TELEMETRY_TX_WAIT_LIMIT           (100000UL)
#define MAP_RUN_UART_RX_BUFFER_SIZE               (128U)
#define MAP_RUN_UART_LINE_BUFFER_SIZE             (64U)
#define MAP_RUN_SPEED_TEST_MAX_ABS_RPM             (150.0f)
#define MAP_RUN_ENCODER_FEEDBACK_TIMEOUT_MS        (400UL)

/*
 * 728 count/圈、65 mm 车轮时约为 35.65 count/cm。
 * 题图一圈约 21895 count；21300 count 后才允许 5 cm 启停线触发，
 * 防止普通赛道上的 2~3 路黑线在前半圈造成误停车。
 */
#define MAP_RUN_START_LINE_LEAVE_COUNTS          (120UL)
#define MAP_RUN_LAP_ARM_COUNTS                   (21300UL)
#define MAP_RUN_FINISH_LINE_ACTIVE_MIN           (3U)
#define MAP_RUN_NORMAL_LINE_ACTIVE_MAX           (3U)
#define MAP_RUN_START_LINE_LEAVE_CONFIRM         (3U)
#define MAP_RUN_FINISH_LINE_CONFIRM              (3U)

/*
 * Q5/Q6 不再依赖横线：按实测停车点减去 0.5 m，且累计转动满 360 度后缓停。
 * 若 IMU 未更新或安装方向配置错误，最大距离兜底可避免车辆无限绕圈。
 */
#define MAP_RUN_BALL_FIXED_STOP_COUNTS            (25590UL)
#define MAP_RUN_BALL_FIXED_STOP_MAX_COUNTS        (28220UL)
#define MAP_RUN_BALL_FIXED_STOP_TURN_DEG            (360.0f)
#define MAP_RUN_BALL_IMU_MIN_PLAUSIBLE_TURN_DEG      (30.0f)
#define MAP_RUN_BALL_YAW_MAX_SAMPLE_DELTA_DEG        (45.0f)

/* 最后一段用起跑航向约束出弯姿态，再由原角速度内环消除残余转动。 */
#define MAP_RUN_FINISH_HEADING_ARM_COUNTS        (21000UL)
#define MAP_RUN_FINISH_HEADING_KP_RATE_PER_DEG      (2.0f)
#define MAP_RUN_FINISH_HEADING_RATE_LIMIT_DEG_S    (40.0f)

/*
 * 第 4 问 A->B：65 mm 车轮、728 count/圈时，1.5 m 约为 5348 count。
 * 灰度探头在轮轴前方，先在接近 B 点时锁存右二探头，再等车轴通过 B。
 * 正常越过 B 点约 9.8 cm 后开始最终缓停；若右二探头没有可靠触发，
 * 则 5950 count 作为距离兜底停车条件。
 */
#define MAP_RUN_Q4_RIGHT_SECOND_MASK             (0x40U)
#define MAP_RUN_Q4_SENSOR_ARM_COUNTS             (4500UL)
#define MAP_RUN_Q4_BRAKE_START_COUNTS            (4950UL)
#define MAP_RUN_Q4_TARGET_STOP_COUNTS            (5700UL)
#define MAP_RUN_Q4_MAX_STOP_COUNTS               (5950UL)
#define MAP_RUN_Q4_APPROACH_SPEED_RPM              (35.0f)
#define MAP_RUN_Q4_TARGET_POSITION_MM                (0.0f)
#define MAP_RUN_Q4_START_CENTER_LIMIT_MM             (10.0f)
#define MAP_RUN_Q4_ALLOWED_ERROR_MM                  (10.0f)

#define SENSOR_CALIBRATION_ENTER_HOLD_MS         (4200UL)
#define SENSOR_CALIBRATION_SHORT_HOLD_MS         (150UL)
#define SENSOR_CALIBRATION_SETTLE_MS             (800UL)
#define SENSOR_CALIBRATION_DONE_HOLD_MS          (1200UL)

typedef enum {
    MAP_RUN_STATE_READY = 0,
    MAP_RUN_STATE_RUNNING,
    MAP_RUN_STATE_FINISHED
} MapRunState;

typedef enum {
    MAP_CONTROL_LINE_FOLLOW = 0,
    MAP_CONTROL_SPEED_TEST
} MapControlMode;

typedef enum {
    MAP_QUESTION_MODE_1_Q2 = 1,
    MAP_QUESTION_MODE_2_Q3,
    MAP_QUESTION_MODE_3_Q4,
    MAP_QUESTION_MODE_4_Q5,
    MAP_QUESTION_MODE_5_Q6
} MapQuestionMode;

typedef enum {
    MAP_FINISH_NONE = 0,
    MAP_FINISH_ONE_LAP,
    MAP_FINISH_Q3_COMPLETE,
    MAP_FINISH_Q4_B_PASSED,
    MAP_FINISH_MANUAL,
    MAP_FINISH_ENCODER_FAULT,
    MAP_FINISH_NO_START_LINE
} MapFinishReason;

typedef enum {
    MAP_Q3_STATE_IDLE = 0,
    MAP_Q3_STATE_TO_POSITIVE,
    MAP_Q3_STATE_DRIVE_NEGATIVE,
    MAP_Q3_STATE_BRAKE_NEGATIVE,
    MAP_Q3_STATE_LEVEL_SETTLE,
    MAP_Q3_STATE_HOLD_NEGATIVE
} MapQ3State;

typedef enum {
    SENSOR_CALIBRATION_IDLE = 0,
    SENSOR_CALIBRATION_WAIT_STEPPER,
    SENSOR_CALIBRATION_ENTER_HOLD,
    SENSOR_CALIBRATION_ENTER_SETTLE,
    SENSOR_CALIBRATION_WAIT_BLACK,
    SENSOR_CALIBRATION_BLACK_HOLD,
    SENSOR_CALIBRATION_BLACK_SETTLE,
    SENSOR_CALIBRATION_WAIT_WHITE,
    SENSOR_CALIBRATION_WHITE_HOLD,
    SENSOR_CALIBRATION_DONE
} SensorCalibrationState;

typedef struct {
    unsigned long change_ms;
    unsigned long press_start_ms;
    uint8_t raw_pressed;
    uint8_t stable_pressed;
    uint8_t pressed_event;
    uint8_t short_release_event;
    uint8_t long_release_event;
    uint8_t suppress_release;
} MapRunButton;

static MapRunState g_map_state;
static MapControlMode g_control_mode;
static MapQuestionMode g_question_mode;
static MapFinishReason g_finish_reason;
static MapQ3State g_q3_state;
static SensorCalibrationState g_calibration_state;
static MapRunButton g_b21_button;
static MapRunButton g_sw1_button;
static MapRunButton g_sw2_button;

static unsigned long g_run_start_ms;
static unsigned long g_run_elapsed_ms;
static unsigned long g_last_control_ms;
static unsigned long g_last_speed_ms;
static unsigned long g_last_telemetry_ms;
static unsigned long g_last_ball_log_ms;
static unsigned long g_last_ball_lap_telemetry_ms;
static unsigned long g_last_display_ms;
static unsigned long g_calibration_state_start_ms;
static unsigned long g_motor_brake_start_ms;
static unsigned long g_q4_feedforward_update_ms;
static unsigned long g_q4_stop_feedforward_start_ms;
static unsigned long g_q4_final_stop_start_ms;
static unsigned long g_ball_lap_final_stop_start_ms;

static int32_t g_left_start_count;
static int32_t g_right_start_count;
static int32_t g_watchdog_left_encoder_count;
static int32_t g_watchdog_right_encoder_count;
static unsigned long g_watchdog_left_feedback_ms;
static unsigned long g_watchdog_right_feedback_ms;
static uint32_t g_run_distance_counts;
static uint8_t g_sensor_value;
static uint8_t g_sensor_active_count;
static uint8_t g_start_line_left;
static uint8_t g_lap_finish_armed;
static uint8_t g_start_line_leave_count;
static uint8_t g_finish_line_count;
static uint8_t g_q4_right_second_seen;
static uint8_t g_q4_ball_limit_exceeded;
static uint8_t g_q4_stop_feedforward_active;
static uint8_t g_q4_final_stop_active;
static uint8_t g_q4_park_hold_active;
static uint8_t g_q4_camera_fallback_active;
static uint8_t g_ball_lap_camera_fallback_active;
static uint8_t g_ball_lap_limit_exceeded;
static uint8_t g_ball_lap_completed_within_time;
static uint8_t g_ball_lap_final_stop_active;
static uint8_t g_motor_brake_active;
static uint8_t g_q3_completed_within_time;
static uint8_t g_q3_time_warning_sent;
static uint8_t g_q3_saved_pid_valid;
static uint8_t g_q3_manual_beam_active;
static uint8_t g_q3_hold_entry_seed_pending;
static uint8_t g_q3_hold_stiction_candidate_active;
static uint8_t g_q3_hold_stiction_kick_active;
static uint8_t g_q3_settle_lock_active;
static uint8_t g_q3_settle_capture_active;
static uint8_t g_q3_capture_pd_active;
static uint8_t g_q3_brake_low_speed_active;
static uint32_t g_q3_last_camera_frame_id;
static uint32_t g_q3_phase_start_ms;
static uint32_t g_q3_motion_last_ms;
static uint32_t g_q3_brake_low_speed_start_ms;
static uint32_t g_q3_catch_pulse_start_ms;
static uint32_t g_q3_window_start_ms;
static uint32_t g_q3_hold_last_update_ms;
static uint32_t g_q3_hold_last_camera_frame_id;
static uint32_t g_q3_hold_stiction_candidate_start_ms;
static uint32_t g_q3_hold_stiction_kick_start_ms;
static uint32_t g_q3_hold_stiction_rearm_start_ms;
static float g_q3_window_start_x_mm;
static float g_q3_window_min_x_mm;
static float g_q3_window_max_x_mm;
static float g_q3_hold_bias_deg;
static float g_q3_hold_last_filtered_x_mm;
static float g_q3_hold_frame_delta_mm;
static float g_q3_hold_feedback_speed_mm_s;
static float g_q3_hold_profile_target_speed_mm_s;
static float g_q3_hold_profile_blend;
static float g_q3_hold_stiction_position_deg;
static float g_q3_hold_stiction_candidate_x_mm;
static float g_q3_hold_stiction_kick_deg;
static float g_q3_manual_beam_angle_deg;
static float g_q3_settle_lock_angle_deg;
static float g_q3_brake_planned_speed_mm_s;
static float g_q3_brake_angle_deg;
static float g_q3_motion_last_x_mm;
static float g_q3_fresh_speed_mm_s;
static float g_q3_saved_kp;
static float g_q3_saved_ki;
static float g_q3_saved_kd;
static float g_q4_max_ball_error_mm;
static float g_q4_acceleration_feedforward_deg;
static float g_q4_position_guard_deg;
static float g_q4_velocity_damping_deg;
static float g_q4_fast_ball_speed_mm_s;
static float g_q4_velocity_feedback_mm_s;
static float g_q4_velocity_reference_x_mm;
static uint32_t g_q4_velocity_reference_frame_id;
static uint32_t g_q4_velocity_reference_receive_ms;
static uint8_t g_q4_velocity_observer_valid;
static float g_q4_final_stop_start_rpm;
static float g_q4_final_stop_left_duty_percent;
static float g_q4_final_stop_right_duty_percent;
static float g_ball_lap_final_stop_start_rpm;
static float g_ball_lap_final_stop_left_duty_percent;
static float g_ball_lap_final_stop_right_duty_percent;
static float g_q6_target_position_mm;
static float g_ball_lap_active_target_mm;
static float g_ball_lap_max_error_mm;
static float g_line_error;
static float g_base_speed_rpm;
static float g_correction_rpm;
static MapLineController g_line_controller;
static MapLineResult g_line_result;
static MapYawRateController g_yaw_rate_controller;
static MapCurveHold g_curve_hold;
static BallBeamController g_ball_beam_controller;
static float g_center_speed_rpm;
static float g_target_yaw_rate_deg_s;
static float g_measured_yaw_rate_deg_s;
static float g_yaw_turn_rpm;
static uint8_t g_yaw_cascade_active;
static float g_finish_heading_reference_yaw_deg;
static float g_finish_heading_error_deg;
static float g_finish_heading_rate_offset_deg_s;
static uint8_t g_finish_heading_reference_valid;
static uint8_t g_finish_heading_active;
static float g_ball_lap_previous_yaw_deg;
static float g_ball_lap_right_turn_deg;
static uint8_t g_ball_lap_yaw_valid;
static float g_speed_test_left_target_rpm;
static float g_speed_test_right_target_rpm;
static uint8_t g_display_buffer[32];
static volatile uint8_t g_uart_rx_buffer[MAP_RUN_UART_RX_BUFFER_SIZE];
static volatile uint16_t g_uart_rx_write_index;
static volatile uint16_t g_uart_rx_read_index;
static volatile uint8_t g_uart_rx_overflow;
static char g_uart_line_buffer[MAP_RUN_UART_LINE_BUFFER_SIZE];
static uint16_t g_uart_line_length;
static uint32_t g_camera_forwarded_frame_count;
static uint32_t g_camera_control_frame_count;
static uint8_t g_ball_stepper_zeroed;
static uint8_t g_ball_log_enabled;

static void MapRun_StopMotors(MapFinishReason reason);
static void MapRun_BeginQ4FinalStop(void);
static void MapRun_UpdateQ4FinalStop(void);
static void MapRun_BeginBallLapFinalStop(void);
static void MapRun_UpdateBallLapFinalStop(void);
static void MapRun_StartSpeedTest(void);
static void MapRun_DisableBallBeam(void);
static void MapRun_SendBallLogHeader(void);
static void MapRun_SendBallLogSample(void);
static void MapRun_EnableAutomaticBallLog(const char *question_text);
static void MapRun_ResetQ3StictionAssist(void);
static float MapRun_AbsFloat(float value);
static void MapRun_ResetBallLapTurnTracking(uint8_t seed_current_yaw);

static uint8_t MapRun_IsQ3Mode(void)
{
    return (g_question_mode == MAP_QUESTION_MODE_2_Q3) ? 1U : 0U;
}

static uint8_t MapRun_IsQ4Mode(void)
{
    return (g_question_mode == MAP_QUESTION_MODE_3_Q4) ? 1U : 0U;
}

/* 这里只选择第五、六问专用的带球底盘参数，第四问球控单独接入。 */
static uint8_t MapRun_IsBallMode(void)
{
    return ((g_question_mode == MAP_QUESTION_MODE_4_Q5) ||
            (g_question_mode == MAP_QUESTION_MODE_5_Q6)) ? 1U : 0U;
}

static float MapRun_GetBallBaseSpeedRpm(void)
{
    unsigned long elapsed_ms;
    unsigned long ramp_elapsed_ms;
    float ramp_progress;
    float smooth_progress;

    if (g_map_state != MAP_RUN_STATE_RUNNING) {
        return MAP_RUN_BALL_BASE_SPEED_RPM;
    }

    elapsed_ms = tick_ms - g_run_start_ms;
    if (elapsed_ms < MAP_RUN_BALL_START_HOLD_MS) {
        return 0.0f;
    }

    ramp_elapsed_ms = elapsed_ms - MAP_RUN_BALL_START_HOLD_MS;
    if (ramp_elapsed_ms >= MAP_RUN_BALL_START_RAMP_MS) {
        return MAP_RUN_BALL_BASE_SPEED_RPM;
    }

    ramp_progress =
        (float) ramp_elapsed_ms / (float) MAP_RUN_BALL_START_RAMP_MS;
    smooth_progress =
        ramp_progress * ramp_progress *
        (3.0f - (2.0f * ramp_progress));
    return MAP_RUN_BALL_START_SPEED_RPM +
           (smooth_progress *
            (MAP_RUN_BALL_BASE_SPEED_RPM -
             MAP_RUN_BALL_START_SPEED_RPM));
}

/*
 * 第四问只规划底盘中心速度，不改变灰度和角速度串级的转向输出。
 * 先让滚球闭环稳定，再以当前底盘最低可用速度起步。三次平滑曲线
 * 在起点和终点的斜率均为 0，可减小轮速目标突变对钢球的冲击；
 * B 点前再提前降速，避免从巡航速度直接停车。
 */
static float MapRun_GetQ4BaseSpeedRpm(void)
{
    unsigned long elapsed_ms;
    unsigned long ramp_elapsed_ms;
    float planned_speed_rpm = MAP_RUN_Q4_BASE_SPEED_RPM;
    float ramp_progress;
    float smooth_progress;
    float brake_progress;
    float brake_speed_rpm;

    if (g_q4_final_stop_active != 0U) {
        elapsed_ms = tick_ms - g_q4_final_stop_start_ms;
        if (elapsed_ms >= MAP_RUN_Q4_FINAL_STOP_RAMP_MS) {
            return 0.0f;
        }

        ramp_progress =
            (float) elapsed_ms /
            (float) MAP_RUN_Q4_FINAL_STOP_RAMP_MS;
        smooth_progress =
            ramp_progress * ramp_progress *
            (3.0f - (2.0f * ramp_progress));
        return g_q4_final_stop_start_rpm *
               (1.0f - smooth_progress);
    }

    if (g_map_state != MAP_RUN_STATE_RUNNING) {
        return MAP_RUN_Q4_BASE_SPEED_RPM;
    }

    elapsed_ms = tick_ms - g_run_start_ms;
    if (elapsed_ms < MAP_RUN_Q4_START_HOLD_MS) {
        planned_speed_rpm = 0.0f;
    } else if (elapsed_ms <
               (MAP_RUN_Q4_START_HOLD_MS +
                MAP_RUN_Q4_START_RAMP_MS)) {
        ramp_elapsed_ms = elapsed_ms - MAP_RUN_Q4_START_HOLD_MS;
        ramp_progress =
            (float) ramp_elapsed_ms /
            (float) MAP_RUN_Q4_START_RAMP_MS;
        smooth_progress =
            ramp_progress * ramp_progress *
            (3.0f - (2.0f * ramp_progress));
        planned_speed_rpm =
            MAP_RUN_Q4_START_SPEED_RPM +
            (smooth_progress *
             (MAP_RUN_Q4_BASE_SPEED_RPM -
              MAP_RUN_Q4_START_SPEED_RPM));
    }

    if (g_run_distance_counts >= MAP_RUN_Q4_BRAKE_START_COUNTS) {
        if (g_run_distance_counts >= MAP_RUN_Q4_TARGET_STOP_COUNTS) {
            brake_speed_rpm = MAP_RUN_Q4_APPROACH_SPEED_RPM;
        } else {
            brake_progress =
                (float) (g_run_distance_counts -
                         MAP_RUN_Q4_BRAKE_START_COUNTS) /
                (float) (MAP_RUN_Q4_TARGET_STOP_COUNTS -
                         MAP_RUN_Q4_BRAKE_START_COUNTS);
            smooth_progress =
                brake_progress * brake_progress *
                (3.0f - (2.0f * brake_progress));
            brake_speed_rpm =
                MAP_RUN_Q4_BASE_SPEED_RPM -
                (smooth_progress *
                 (MAP_RUN_Q4_BASE_SPEED_RPM -
                  MAP_RUN_Q4_APPROACH_SPEED_RPM));
        }

        if (brake_speed_rpm < planned_speed_rpm) {
            planned_speed_rpm = brake_speed_rpm;
        }
    }

    return planned_speed_rpm;
}

static void MapRun_ResetQ4VelocityObserver(void)
{
    g_q4_fast_ball_speed_mm_s = 0.0f;
    g_q4_velocity_feedback_mm_s = 0.0f;
    g_q4_velocity_reference_x_mm = 0.0f;
    g_q4_velocity_reference_frame_id = 0UL;
    g_q4_velocity_reference_receive_ms = 0UL;
    g_q4_velocity_observer_valid = 0U;
}

/*
 * 通用滚球速度为抑制毫米量化噪声使用约三帧窗口，换向时会慢约一拍。
 * Q4 至 Q6 另外用 35 ms 短窗估速：忽略串口成组到达的过短间隔，但在
 * 检测到速度反号时立即采用新方向，避免阻尼继续沿旧方向给振荡补充能量。
 */
static void MapRun_UpdateQ4VelocityObserver(void)
{
    uint32_t frame_id;
    uint32_t receive_ms;
    uint32_t period_ms;
    float position_delta_mm;
    float raw_speed_mm_s;
    float slow_speed_mm_s;

    if (((MapRun_IsQ4Mode() == 0U) &&
         (MapRun_IsBallMode() == 0U)) ||
        (BallBeamController_IsEnabled(
             &g_ball_beam_controller) == 0U) ||
        (BallBeamController_IsControlReady(
             &g_ball_beam_controller) == 0U) ||
        (g_ball_beam_controller.dropout_active != 0U)) {
        MapRun_ResetQ4VelocityObserver();
        return;
    }

    frame_id = g_ball_beam_controller.camera_frame_id;
    receive_ms = g_ball_beam_controller.last_valid_receive_ms;
    slow_speed_mm_s = g_ball_beam_controller.ball_speed_mm_s;

    if (g_q4_velocity_observer_valid == 0U) {
        g_q4_velocity_reference_frame_id = frame_id;
        g_q4_velocity_reference_receive_ms = receive_ms;
        g_q4_velocity_reference_x_mm =
            g_ball_beam_controller.filtered_x_mm;
        g_q4_fast_ball_speed_mm_s = slow_speed_mm_s;
        g_q4_velocity_feedback_mm_s = slow_speed_mm_s;
        g_q4_velocity_observer_valid = 1U;
        return;
    }

    if (frame_id != g_q4_velocity_reference_frame_id) {
        period_ms = receive_ms - g_q4_velocity_reference_receive_ms;
        if (period_ms > MAP_RUN_Q4_VELOCITY_WINDOW_MAX_MS) {
            g_q4_velocity_reference_frame_id = frame_id;
            g_q4_velocity_reference_receive_ms = receive_ms;
            g_q4_velocity_reference_x_mm =
                g_ball_beam_controller.filtered_x_mm;
            g_q4_fast_ball_speed_mm_s = slow_speed_mm_s;
        } else if (period_ms >= MAP_RUN_Q4_VELOCITY_WINDOW_MIN_MS) {
            position_delta_mm =
                g_ball_beam_controller.filtered_x_mm -
                g_q4_velocity_reference_x_mm;
            if (MapRun_AbsFloat(position_delta_mm) <=
                MAP_RUN_Q4_VELOCITY_DELTA_DEADBAND_MM) {
                raw_speed_mm_s = 0.0f;
            } else {
                raw_speed_mm_s =
                    position_delta_mm *
                    (1000.0f / (float) period_ms);
            }

            if (raw_speed_mm_s > MAP_RUN_Q4_VELOCITY_MAX_MM_S) {
                raw_speed_mm_s = MAP_RUN_Q4_VELOCITY_MAX_MM_S;
            } else if (raw_speed_mm_s <
                       -MAP_RUN_Q4_VELOCITY_MAX_MM_S) {
                raw_speed_mm_s = -MAP_RUN_Q4_VELOCITY_MAX_MM_S;
            }

            if (((raw_speed_mm_s * g_q4_fast_ball_speed_mm_s) < 0.0f) &&
                (MapRun_AbsFloat(raw_speed_mm_s) >=
                 MAP_RUN_Q4_VELOCITY_REVERSAL_MIN_MM_S)) {
                g_q4_fast_ball_speed_mm_s = raw_speed_mm_s;
            } else {
                g_q4_fast_ball_speed_mm_s +=
                    MAP_RUN_Q4_VELOCITY_FILTER_ALPHA *
                    (raw_speed_mm_s - g_q4_fast_ball_speed_mm_s);
            }

            g_q4_velocity_reference_frame_id = frame_id;
            g_q4_velocity_reference_receive_ms = receive_ms;
            g_q4_velocity_reference_x_mm =
                g_ball_beam_controller.filtered_x_mm;
        }
    }

    if (((g_q4_fast_ball_speed_mm_s * slow_speed_mm_s) < 0.0f) &&
        (MapRun_AbsFloat(g_q4_fast_ball_speed_mm_s) >=
         MAP_RUN_Q4_VELOCITY_REVERSAL_MIN_MM_S)) {
        g_q4_velocity_feedback_mm_s = g_q4_fast_ball_speed_mm_s;
    } else {
        g_q4_velocity_feedback_mm_s =
            MAP_RUN_Q4_VELOCITY_FAST_BLEND *
                g_q4_fast_ball_speed_mm_s +
            (1.0f - MAP_RUN_Q4_VELOCITY_FAST_BLEND) *
                slow_speed_mm_s;
    }
}

/* 球已反向回中后及时撤掉正向停车前馈，负侧过深时也禁止继续向左推。 */
static float MapRun_ApplyQ4StopFeedforwardRelease(float feedforward_deg)
{
    float release_scale;

    if (BallBeamController_IsControlReady(
            &g_ball_beam_controller) == 0U) {
        return feedforward_deg;
    }
    if (g_ball_beam_controller.filtered_x_mm <
        MAP_RUN_Q4_STOP_FF_DISABLE_X_MM) {
        return 0.0f;
    }
    if (g_q4_velocity_feedback_mm_s >= 0.0f) {
        return feedforward_deg;
    }

    release_scale =
        1.0f +
        g_q4_velocity_feedback_mm_s /
            MAP_RUN_Q4_STOP_FF_RELEASE_SPEED_MM_S;
    if (release_scale <= 0.0f) {
        return 0.0f;
    }
    return feedforward_deg * release_scale;
}

/*
 * 实测小车向前加速时钢球向车后移动，并对应 X<0。摆杆负角会给钢球
 * 向 X 正方向的分力，因此起步使用负前馈；减速时方向相反。通过 B
 * 最后 35 RPM 到 0 使用独立 PWM 缓降，并叠加钟形正角；停车后短时保留
 * 接球正角，并在检测到小球反向回中后提前退出。所有前馈均不进入 PID 积分。
 */
static float MapRun_GetQ4AccelerationFeedforwardDeg(void)
{
    unsigned long elapsed_ms;
    unsigned long ramp_elapsed_ms;
    float progress;
    float normalized_acceleration;
    float terminal_preload;
    float feedforward_deg;
    float preload_scale;
    float release_scale;

    if (MapRun_IsQ4Mode() == 0U) {
        return 0.0f;
    }

    if (g_q4_stop_feedforward_active != 0U) {
        elapsed_ms = tick_ms - g_q4_stop_feedforward_start_ms;
        if (elapsed_ms < MAP_RUN_Q4_STOP_FF_HOLD_MS) {
            feedforward_deg = MAP_RUN_Q4_STOP_FF_DEG;
        } else if ((elapsed_ms - MAP_RUN_Q4_STOP_FF_HOLD_MS) <
                   MAP_RUN_Q4_STOP_FF_FADE_MS) {
            elapsed_ms -= MAP_RUN_Q4_STOP_FF_HOLD_MS;
            progress =
                (float) elapsed_ms /
                (float) MAP_RUN_Q4_STOP_FF_FADE_MS;
            normalized_acceleration =
                progress * progress * (3.0f - (2.0f * progress));
            feedforward_deg =
                MAP_RUN_Q4_STOP_FF_DEG *
                (1.0f - normalized_acceleration);
        } else {
            g_q4_stop_feedforward_active = 0U;
            return 0.0f;
        }

        return MapRun_ApplyQ4StopFeedforwardRelease(feedforward_deg);
    }

    /*
     * 停车保持阶段不再重新套用距离减速前馈。该前馈只用于抵消底盘
     * 减速度，车已停稳后继续保持正角会把钢球再次推向负侧。
     */
    if (g_q4_park_hold_active != 0U) {
        return 0.0f;
    }

    if (g_q4_final_stop_active != 0U) {
        elapsed_ms = tick_ms - g_q4_final_stop_start_ms;
        if (elapsed_ms >= MAP_RUN_Q4_FINAL_STOP_RAMP_MS) {
            return MapRun_ApplyQ4StopFeedforwardRelease(
                MAP_RUN_Q4_STOP_FF_DEG);
        }

        progress =
            (float) elapsed_ms /
            (float) MAP_RUN_Q4_FINAL_STOP_RAMP_MS;
        normalized_acceleration =
            4.0f * progress * (1.0f - progress);
        terminal_preload =
            progress * progress * (3.0f - (2.0f * progress));

        /*
         * 从距离减速阶段的 +0.70 度连续接管，再叠加钟形制动项，最终
         * 平滑落到停车保持角，避免终点触发瞬间出现前馈空档。
         */
        feedforward_deg =
            MAP_RUN_Q4_BRAKE_FF_PEAK_DEG *
                (1.0f - terminal_preload) +
            MAP_RUN_Q4_FINAL_STOP_FF_PEAK_DEG *
                normalized_acceleration +
            MAP_RUN_Q4_STOP_FF_DEG * terminal_preload;
        return MapRun_ApplyQ4StopFeedforwardRelease(feedforward_deg);
    }

    if (g_map_state != MAP_RUN_STATE_RUNNING) {
        return 0.0f;
    }

    elapsed_ms = tick_ms - g_run_start_ms;
    if (elapsed_ms < MAP_RUN_Q4_START_HOLD_MS) {
        if (elapsed_ms <=
            (MAP_RUN_Q4_START_HOLD_MS -
             MAP_RUN_Q4_START_PRELOAD_RAMP_MS)) {
            return 0.0f;
        }

        progress =
            (float) (elapsed_ms -
                     (MAP_RUN_Q4_START_HOLD_MS -
                      MAP_RUN_Q4_START_PRELOAD_RAMP_MS)) /
            (float) MAP_RUN_Q4_START_PRELOAD_RAMP_MS;
        normalized_acceleration =
            progress * progress * (3.0f - (2.0f * progress));
        return -MAP_RUN_Q4_START_PRELOAD_DEG *
               normalized_acceleration;
    }

    if ((elapsed_ms >= MAP_RUN_Q4_START_HOLD_MS) &&
        (elapsed_ms <
         (MAP_RUN_Q4_START_HOLD_MS + MAP_RUN_Q4_START_RAMP_MS))) {
        ramp_elapsed_ms = elapsed_ms - MAP_RUN_Q4_START_HOLD_MS;
        progress =
            (float) ramp_elapsed_ms /
            (float) MAP_RUN_Q4_START_RAMP_MS;
        normalized_acceleration =
            4.0f * progress * (1.0f - progress);
        feedforward_deg =
            -MAP_RUN_Q4_START_FF_PEAK_DEG *
            normalized_acceleration;

        preload_scale = 0.0f;
        if (ramp_elapsed_ms < MAP_RUN_Q4_START_PRELOAD_FADE_MS) {
            progress =
                (float) ramp_elapsed_ms /
                (float) MAP_RUN_Q4_START_PRELOAD_FADE_MS;
            normalized_acceleration =
                progress * progress * (3.0f - (2.0f * progress));
            preload_scale = 1.0f - normalized_acceleration;
        }
        feedforward_deg -=
            MAP_RUN_Q4_START_PRELOAD_DEG * preload_scale;

        /*
         * 球已回到中心附近并继续向 +X 运动时，保持负前馈会把它推过
         * 中心。先满足位置门槛，再按回中速度退出，避免仅由一帧速度
         * 尖峰在球仍位于负侧时过早撤掉起步补偿。
         */
        if ((BallBeamController_IsControlReady(
                  &g_ball_beam_controller) != 0U) &&
            (g_ball_beam_controller.filtered_x_mm >=
             MAP_RUN_Q4_START_FF_RELEASE_MIN_X_MM) &&
            (g_q4_velocity_feedback_mm_s > 0.0f)) {
            release_scale =
                1.0f -
                g_q4_velocity_feedback_mm_s /
                    MAP_RUN_Q4_START_FF_RELEASE_SPEED_MM_S;
            if (release_scale <= 0.0f) {
                return 0.0f;
            }
            if (release_scale < 1.0f) {
                feedforward_deg *= release_scale;
            }
        }
        return feedforward_deg;
    }

    if (g_run_distance_counts >= MAP_RUN_Q4_BRAKE_START_COUNTS) {
        if (g_run_distance_counts >= MAP_RUN_Q4_TARGET_STOP_COUNTS) {
            return MAP_RUN_Q4_BRAKE_FF_PEAK_DEG;
        }

        progress =
            (float) (g_run_distance_counts -
                     MAP_RUN_Q4_BRAKE_START_COUNTS) /
            (float) (MAP_RUN_Q4_TARGET_STOP_COUNTS -
                     MAP_RUN_Q4_BRAKE_START_COUNTS);
        normalized_acceleration =
            4.0f * progress * (1.0f - progress);

        /* 前半段沿原钟形曲线上升，达到峰值后保持到最终停车接管。 */
        if (progress >= 0.5f) {
            normalized_acceleration = 1.0f;
        }
        return MAP_RUN_Q4_BRAKE_FF_PEAK_DEG *
               normalized_acceleration;
    }

    return 0.0f;
}

/*
 * 第五、六问复用第四问已验证的启停补偿思路：起步前先建立负角预置，
 * S 曲线加速时叠加负向钟形前馈；终点 PWM 缓降时改用正向钟形前馈。
 * 这里仅补偿底盘纵向加减速，弯道差速仍由灰度和角速度串级负责。
 */
static float MapRun_GetBallLapAccelerationFeedforwardDeg(void)
{
    unsigned long elapsed_ms;
    unsigned long ramp_elapsed_ms;
    float progress;
    float normalized_acceleration;
    float preload_scale;
    float feedforward_deg;
    float release_scale;
    float relative_x_mm;

    if ((MapRun_IsBallMode() == 0U) ||
        (g_map_state != MAP_RUN_STATE_RUNNING)) {
        return 0.0f;
    }

    if (g_ball_lap_final_stop_active != 0U) {
        elapsed_ms = tick_ms - g_ball_lap_final_stop_start_ms;
        if (elapsed_ms >= MAP_RUN_BALL_FINAL_STOP_RAMP_MS) {
            return 0.0f;
        }

        progress =
            (float) elapsed_ms /
            (float) MAP_RUN_BALL_FINAL_STOP_RAMP_MS;
        normalized_acceleration =
            4.0f * progress * (1.0f - progress);
        return MAP_RUN_BALL_FINAL_STOP_FF_PEAK_DEG *
               normalized_acceleration;
    }

    elapsed_ms = tick_ms - g_run_start_ms;
    if (elapsed_ms < MAP_RUN_BALL_START_HOLD_MS) {
        if (elapsed_ms <=
            (MAP_RUN_BALL_START_HOLD_MS -
             MAP_RUN_BALL_START_PRELOAD_RAMP_MS)) {
            return 0.0f;
        }

        progress =
            (float) (elapsed_ms -
                     (MAP_RUN_BALL_START_HOLD_MS -
                      MAP_RUN_BALL_START_PRELOAD_RAMP_MS)) /
            (float) MAP_RUN_BALL_START_PRELOAD_RAMP_MS;
        normalized_acceleration =
            progress * progress * (3.0f - (2.0f * progress));
        return -MAP_RUN_BALL_START_PRELOAD_DEG *
               normalized_acceleration;
    }

    ramp_elapsed_ms = elapsed_ms - MAP_RUN_BALL_START_HOLD_MS;
    if (ramp_elapsed_ms >= MAP_RUN_BALL_START_RAMP_MS) {
        return 0.0f;
    }

    progress =
        (float) ramp_elapsed_ms /
        (float) MAP_RUN_BALL_START_RAMP_MS;
    normalized_acceleration =
        4.0f * progress * (1.0f - progress);
    feedforward_deg =
        -MAP_RUN_BALL_START_FF_PEAK_DEG *
        normalized_acceleration;

    preload_scale = 0.0f;
    if (ramp_elapsed_ms < MAP_RUN_BALL_START_PRELOAD_FADE_MS) {
        progress =
            (float) ramp_elapsed_ms /
            (float) MAP_RUN_BALL_START_PRELOAD_FADE_MS;
        normalized_acceleration =
            progress * progress * (3.0f - (2.0f * progress));
        preload_scale = 1.0f - normalized_acceleration;
    }
    feedforward_deg -=
        MAP_RUN_BALL_START_PRELOAD_DEG * preload_scale;

    /*
     * Q6 的目标可能不是 0，因此退出门槛必须使用相对目标误差。
     * 球已经回到目标附近并向 +X 运动时按速度撤掉负前馈，避免过冲。
     */
    relative_x_mm =
        g_ball_beam_controller.filtered_x_mm -
        g_ball_beam_controller.target_x_mm;
    if ((BallBeamController_IsControlReady(
              &g_ball_beam_controller) != 0U) &&
        (relative_x_mm >=
         MAP_RUN_BALL_START_FF_RELEASE_MIN_ERROR_MM) &&
        (g_ball_beam_controller.ball_speed_mm_s > 0.0f)) {
        release_scale =
            1.0f -
            g_ball_beam_controller.ball_speed_mm_s /
                MAP_RUN_BALL_START_FF_RELEASE_SPEED_MM_S;
        if (release_scale <= 0.0f) {
            return 0.0f;
        }
        if (release_scale < 1.0f) {
            feedforward_deg *= release_scale;
        }
    }

    return feedforward_deg;
}

/*
 * 第四至第六问的位置保护只在球静止、远离目标或回目标速度低于规划
 * 速度时介入；球已经快速回目标时逐步退出，让速度反馈提前反向制动，
 * 避免位置保护与制动项对打形成等幅振荡。
 */
static float MapRun_GetMovingBallPositionGuardDeg(void)
{
    float error_mm;
    float error_abs_mm;
    float ball_speed_mm_s;
    float toward_center_speed_mm_s;
    float planned_toward_speed_mm_s;
    float guard_scale = 1.0f;
    float guard_deg;

    if (((MapRun_IsQ4Mode() == 0U) &&
         (MapRun_IsBallMode() == 0U)) ||
        ((g_map_state != MAP_RUN_STATE_RUNNING) &&
         (g_q4_park_hold_active == 0U)) ||
        (BallBeamController_IsEnabled(
             &g_ball_beam_controller) == 0U) ||
        (BallBeamController_IsControlReady(
             &g_ball_beam_controller) == 0U)) {
        return 0.0f;
    }

    error_mm =
        g_ball_beam_controller.filtered_x_mm -
        g_ball_beam_controller.target_x_mm;
    error_abs_mm = MapRun_AbsFloat(error_mm);
    ball_speed_mm_s = g_q4_velocity_feedback_mm_s;
    if (error_abs_mm <= MAP_RUN_Q4_POSITION_GUARD_DEADBAND_MM) {
        return 0.0f;
    }

    /*
     * 停车后只在球已经超出题目允许的 +/-10 mm、且近似静止时补一段
     * 温和回中角。球一旦重新滚动便退出，避免位置项持续加速造成穿越
     * 中心后的二次过冲。
     */
    if (g_q4_park_hold_active != 0U) {
        if ((error_abs_mm <= MAP_RUN_Q4_PARK_GUARD_DEADBAND_MM) ||
            (MapRun_AbsFloat(ball_speed_mm_s) >
             MAP_RUN_Q4_PARK_GUARD_MAX_SPEED_MM_S)) {
            return 0.0f;
        }

        guard_deg =
            (error_abs_mm - MAP_RUN_Q4_PARK_GUARD_DEADBAND_MM) *
            MAP_RUN_Q4_PARK_GUARD_GAIN_DEG_PER_MM;
        if (guard_deg > MAP_RUN_Q4_PARK_GUARD_MAX_DEG) {
            guard_deg = MAP_RUN_Q4_PARK_GUARD_MAX_DEG;
        }
        return (error_mm < 0.0f) ? -guard_deg : guard_deg;
    }

    toward_center_speed_mm_s =
        (error_mm > 0.0f) ?
            -ball_speed_mm_s : ball_speed_mm_s;
    planned_toward_speed_mm_s = MapRun_AbsFloat(
        g_ball_beam_controller.profile_target_speed_mm_s);

    if (toward_center_speed_mm_s > 0.0f) {
        if ((planned_toward_speed_mm_s < 0.1f) ||
            (toward_center_speed_mm_s >= planned_toward_speed_mm_s)) {
            return 0.0f;
        }

        guard_scale =
            1.0f -
            toward_center_speed_mm_s / planned_toward_speed_mm_s;
    }

    guard_deg =
        (error_abs_mm - MAP_RUN_Q4_POSITION_GUARD_DEADBAND_MM) *
        MAP_RUN_Q4_POSITION_GUARD_GAIN_DEG_PER_MM *
        guard_scale;
    if (guard_deg > MAP_RUN_Q4_POSITION_GUARD_MAX_DEG) {
        guard_deg = MAP_RUN_Q4_POSITION_GUARD_MAX_DEG;
    }

    return (error_mm < 0.0f) ? -guard_deg : guard_deg;
}

/* Q4 至 Q6 使用短窗反馈增加速度阻尼，方向始终与小球速度相反。 */
static float MapRun_GetQ4VelocityDampingDeg(void)
{
    float damping_deg;

    if (((MapRun_IsQ4Mode() == 0U) &&
         (MapRun_IsBallMode() == 0U)) ||
        (BallBeamController_IsEnabled(
             &g_ball_beam_controller) == 0U) ||
        (BallBeamController_IsControlReady(
             &g_ball_beam_controller) == 0U)) {
        return 0.0f;
    }

    damping_deg =
        g_q4_velocity_feedback_mm_s *
        MAP_RUN_Q4_VELOCITY_DAMPING_GAIN;
    if (damping_deg > MAP_RUN_Q4_VELOCITY_DAMPING_MAX_DEG) {
        return MAP_RUN_Q4_VELOCITY_DAMPING_MAX_DEG;
    }
    if (damping_deg < -MAP_RUN_Q4_VELOCITY_DAMPING_MAX_DEG) {
        return -MAP_RUN_Q4_VELOCITY_DAMPING_MAX_DEG;
    }
    return damping_deg;
}

static void MapRun_UpdateMovingBallFeedforward(void)
{
    float external_feedforward_deg;
    float external_feedforward_limit_deg =
        MAP_RUN_Q4_EXTERNAL_FF_MAX_DEG;

    if ((tick_ms - g_q4_feedforward_update_ms) <
        MAP_RUN_CONTROL_PERIOD_MS) {
        return;
    }

    g_q4_feedforward_update_ms = tick_ms;
    MapRun_UpdateQ4VelocityObserver();
    if (MapRun_IsQ4Mode() != 0U) {
        g_q4_acceleration_feedforward_deg =
            MapRun_GetQ4AccelerationFeedforwardDeg();
    } else if (MapRun_IsBallMode() != 0U) {
        g_q4_acceleration_feedforward_deg =
            MapRun_GetBallLapAccelerationFeedforwardDeg();
    } else {
        g_q4_acceleration_feedforward_deg = 0.0f;
    }
    g_q4_position_guard_deg =
        MapRun_GetMovingBallPositionGuardDeg();
    g_q4_velocity_damping_deg =
        MapRun_GetQ4VelocityDampingDeg();
    external_feedforward_deg =
        g_q4_acceleration_feedforward_deg +
        g_q4_position_guard_deg +
        g_q4_velocity_damping_deg;
    /*
     * Q5/Q6 的 85 RPM 起步加速度高于 Q4，原共用的 1.5 度限幅会把
     * 预置角与钟形前馈同时截断。仅在起步阶段放宽限幅，巡航和停车
     * 继续使用 1.5 度，避免位置保护在正常行驶时过强。
     */
    if ((MapRun_IsBallMode() != 0U) &&
        (g_map_state == MAP_RUN_STATE_RUNNING) &&
        (g_ball_lap_final_stop_active == 0U) &&
        ((tick_ms - g_run_start_ms) <
         (MAP_RUN_BALL_START_HOLD_MS +
          MAP_RUN_BALL_START_RAMP_MS))) {
        external_feedforward_limit_deg =
            MAP_RUN_BALL_START_EXTERNAL_FF_MAX_DEG;
    }

    if (external_feedforward_deg > external_feedforward_limit_deg) {
        external_feedforward_deg = external_feedforward_limit_deg;
    } else if (external_feedforward_deg <
               -external_feedforward_limit_deg) {
        external_feedforward_deg = -external_feedforward_limit_deg;
    }
    (void) BallBeamController_SetExternalFeedforwardDeg(
        &g_ball_beam_controller,
        external_feedforward_deg);
}

static const char *MapRun_GetQ4SpeedPhaseText(void)
{
    if (g_q4_final_stop_active != 0U) {
        return "STOPPING";
    }
    if (g_map_state != MAP_RUN_STATE_RUNNING) {
        return "IDLE";
    }
    if ((tick_ms - g_run_start_ms) < MAP_RUN_Q4_START_HOLD_MS) {
        return "HOLD";
    }
    if ((tick_ms - g_run_start_ms) <
        (MAP_RUN_Q4_START_HOLD_MS + MAP_RUN_Q4_START_RAMP_MS)) {
        return "START";
    }
    if (g_run_distance_counts < MAP_RUN_Q4_BRAKE_START_COUNTS) {
        return "CRUISE";
    }
    return "BRAKE";
}

static float MapRun_GetCenterRecoveryRpmPerStep(void)
{
    if (MapRun_IsBallMode() != 0U) {
        return MAP_RUN_BALL_CENTER_RECOVERY_RPM_PER_STEP;
    }

    return MAP_RUN_CENTER_RECOVERY_RPM_PER_STEP;
}

static float MapRun_GetQuestionBaseSpeedRpm(void)
{
    switch (g_question_mode) {
        case MAP_QUESTION_MODE_1_Q2:
            return MAP_RUN_Q2_BASE_SPEED_RPM;

        case MAP_QUESTION_MODE_2_Q3:
            return 0.0f;

        case MAP_QUESTION_MODE_3_Q4:
            return MapRun_GetQ4BaseSpeedRpm();

        case MAP_QUESTION_MODE_4_Q5:
        case MAP_QUESTION_MODE_5_Q6:
            return MapRun_GetBallBaseSpeedRpm();

        default:
            return MAP_RUN_Q2_BASE_SPEED_RPM;
    }
}

static void MapRun_ResetYawCascade(void)
{
    MapYawRateController_Reset(&g_yaw_rate_controller);
    MapCurveHold_Reset(&g_curve_hold);
    g_center_speed_rpm = MapRun_GetQuestionBaseSpeedRpm();
    g_base_speed_rpm = g_center_speed_rpm;
    g_target_yaw_rate_deg_s = 0.0f;
    g_measured_yaw_rate_deg_s = 0.0f;
    g_yaw_turn_rpm = 0.0f;
    g_yaw_cascade_active = 0U;
    g_finish_heading_error_deg = 0.0f;
    g_finish_heading_rate_offset_deg_s = 0.0f;
    g_finish_heading_active = 0U;
}

/*
 * 地图应用内置精简蓝牙测速与速度环测试命令，不接入旧状态机。
 * VOFA FireWater 通道顺序：目标左、目标右、实测左、实测右。
 */
static uint8_t MapRun_SendTelemetryByte(uint8_t data)
{
    uint32_t wait_count = 0UL;

    while (DL_UART_Main_isTXFIFOFull(UART_0_INST)) {
        wait_count++;
        if (wait_count >= MAP_RUN_TELEMETRY_TX_WAIT_LIMIT) {
            return 0U;
        }
    }

    DL_UART_Main_transmitData(UART_0_INST, data);
    return 1U;
}

static void MapRun_SendBluetoothText(const char *text)
{
    if (text == NULL) {
        return;
    }

    while (*text != '\0') {
        if (MapRun_SendTelemetryByte((uint8_t) *text) == 0U) {
            return;
        }
        text++;
    }
}

static uint8_t MapRun_GetQuestionNumber(void)
{
    return (uint8_t) g_question_mode + 1U;
}

static uint8_t MapRun_IsQuestionModeReady(void)
{
    return ((g_question_mode == MAP_QUESTION_MODE_1_Q2) ||
            (g_question_mode == MAP_QUESTION_MODE_2_Q3) ||
            (g_question_mode == MAP_QUESTION_MODE_3_Q4) ||
            (g_question_mode == MAP_QUESTION_MODE_4_Q5) ||
            (g_question_mode == MAP_QUESTION_MODE_5_Q6)) ? 1U : 0U;
}

static void MapRun_SendQuestionModeStatus(void)
{
    char response[64];

    (void) snprintf(
        response,
        sizeof(response),
        "[MODE] M%u Q%u %s\r\n",
        (unsigned int) g_question_mode,
        (unsigned int) MapRun_GetQuestionNumber(),
        (MapRun_IsQuestionModeReady() != 0U) ? "READY" : "NOT READY");
    MapRun_SendBluetoothText(response);

    if (g_question_mode == MAP_QUESTION_MODE_5_Q6) {
        MapRun_SendBluetoothText(
            "[MODE] Q6: B21 captures current X and starts\r\n");
    }
}

static void MapRun_ChangeQuestionMode(int8_t direction)
{
    uint8_t next_mode = (uint8_t) g_question_mode;

    /*
     * 第三问完成后会保持终点自适应停球姿态。离开该模式时统一关闭
     * 滚球控制和位置跟踪，避免切换题目后步进机构仍在后台动作。
     */
    MapRun_DisableBallBeam();

    if (direction < 0) {
        next_mode = (next_mode <= (uint8_t) MAP_QUESTION_MODE_1_Q2) ?
            (uint8_t) MAP_QUESTION_MODE_5_Q6 : (next_mode - 1U);
    } else {
        next_mode = (next_mode >= (uint8_t) MAP_QUESTION_MODE_5_Q6) ?
            (uint8_t) MAP_QUESTION_MODE_1_Q2 : (next_mode + 1U);
    }

    g_question_mode = (MapQuestionMode) next_mode;
    g_control_mode = MAP_CONTROL_LINE_FOLLOW;
    g_map_state = MAP_RUN_STATE_READY;
    g_finish_reason = MAP_FINISH_NONE;
    g_run_elapsed_ms = 0UL;
    g_run_distance_counts = 0U;
    g_q4_right_second_seen = 0U;
    g_q4_ball_limit_exceeded = 0U;
    g_q4_max_ball_error_mm = 0.0f;
    g_q4_acceleration_feedforward_deg = 0.0f;
    g_q4_position_guard_deg = 0.0f;
    g_q4_velocity_damping_deg = 0.0f;
    MapRun_ResetQ4VelocityObserver();
    g_q4_feedforward_update_ms = tick_ms;
    g_q4_stop_feedforward_start_ms = tick_ms;
    g_q4_stop_feedforward_active = 0U;
    g_q4_final_stop_start_ms = tick_ms;
    g_q4_final_stop_active = 0U;
    g_q4_park_hold_active = 0U;
    g_q4_final_stop_start_rpm = 0.0f;
    g_q4_final_stop_left_duty_percent = 0.0f;
    g_q4_final_stop_right_duty_percent = 0.0f;
    g_ball_lap_final_stop_start_ms = tick_ms;
    g_ball_lap_final_stop_active = 0U;
    g_ball_lap_final_stop_start_rpm = 0.0f;
    g_ball_lap_final_stop_left_duty_percent = 0.0f;
    g_ball_lap_final_stop_right_duty_percent = 0.0f;
    g_q4_camera_fallback_active = 0U;
    g_ball_lap_camera_fallback_active = 0U;
    g_ball_lap_limit_exceeded = 0U;
    g_ball_lap_completed_within_time = 0U;
    g_ball_lap_active_target_mm =
        (g_question_mode == MAP_QUESTION_MODE_5_Q6) ?
            g_q6_target_position_mm : 0.0f;
    g_ball_lap_max_error_mm = 0.0f;
    g_q3_state = MAP_Q3_STATE_IDLE;
    g_q3_completed_within_time = 0U;
    g_q3_time_warning_sent = 0U;
    g_q3_hold_entry_seed_pending = 0U;
    MapRun_ResetQ3StictionAssist();
    g_q3_phase_start_ms = (uint32_t) tick_ms;
    g_finish_heading_reference_valid = 0U;
    MapRun_ResetBallLapTurnTracking(0U);
    MapRun_ResetYawCascade();
    g_last_display_ms = tick_ms - MAP_RUN_DISPLAY_PERIOD_MS;
    MapRun_SendQuestionModeStatus();
}

static void MapRun_SendSpeedTelemetry(void)
{
    char frame[64];
    int frame_length;
    int index;

    frame_length = snprintf(
        frame,
        sizeof(frame),
        "speed:%.1f,%.1f,%.1f,%.1f\n",
        MotorSpeedLoop_GetLeftWheelTargetRPM(),
        MotorSpeedLoop_GetRightWheelTargetRPM(),
        MotorSpeed_GetLeftWheelRPM(),
        MotorSpeed_GetRightWheelRPM());

    if (frame_length <= 0) {
        return;
    }

    if (frame_length >= (int) sizeof(frame)) {
        frame_length = (int) sizeof(frame) - 1;
    }

    for (index = 0; index < frame_length; index++) {
        if (MapRun_SendTelemetryByte((uint8_t) frame[index]) == 0U) {
            return;
        }
    }
}

/*
 * UART 中断只接收字节并写入环形缓冲，命令解析留在主循环，
 * 避免在中断里执行浮点解析、格式化或电机控制。
 */
void UART_0_INST_IRQHandler(void)
{
    while (DL_UART_isRXFIFOEmpty(UART_0_INST) == false) {
        uint8_t data = DL_UART_receiveData(UART_0_INST);
        uint16_t next_index =
            (uint16_t) ((g_uart_rx_write_index + 1U) %
                        MAP_RUN_UART_RX_BUFFER_SIZE);

        if (next_index == g_uart_rx_read_index) {
            g_uart_rx_overflow = 1U;
        } else {
            g_uart_rx_buffer[g_uart_rx_write_index] = data;
            g_uart_rx_write_index = next_index;
        }
    }
}

static float MapRun_AbsFloat(float value)
{
    return (value < 0.0f) ? (-value) : value;
}

static void MapRun_ResetQ3MotionWindow(void)
{
    g_q3_window_start_ms = (uint32_t) tick_ms;
    g_q3_window_start_x_mm =
        g_ball_beam_controller.filtered_x_mm;
    g_q3_window_min_x_mm = g_q3_window_start_x_mm;
    g_q3_window_max_x_mm = g_q3_window_start_x_mm;
}

static uint8_t MapRun_IsQ3MotionWindowStable(
    uint32_t required_ms,
    float maximum_delta_mm)
{
    float position_span_mm;

    if (g_ball_beam_controller.filtered_x_mm <
        g_q3_window_min_x_mm) {
        g_q3_window_min_x_mm =
            g_ball_beam_controller.filtered_x_mm;
    }
    if (g_ball_beam_controller.filtered_x_mm >
        g_q3_window_max_x_mm) {
        g_q3_window_max_x_mm =
            g_ball_beam_controller.filtered_x_mm;
    }

    /* 使用窗口内最大摆幅，避免小球往返后恰好回到起点而被误判停稳。 */
    position_span_mm =
        g_q3_window_max_x_mm - g_q3_window_min_x_mm;

    if (position_span_mm > maximum_delta_mm) {
        MapRun_ResetQ3MotionWindow();
        return 0U;
    }

    return (((uint32_t) tick_ms - g_q3_window_start_ms) >=
            required_ms) ? 1U : 0U;
}

static void MapRun_RestoreQ3PIDGains(void)
{
    if (g_q3_saved_pid_valid == 0U) {
        return;
    }

    (void) BallBeamController_SetPIDGains(
        &g_ball_beam_controller,
        g_q3_saved_kp,
        g_q3_saved_ki,
        g_q3_saved_kd);
}

static float MapRun_ClampFloat(float value,
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

static float MapRun_NormalizeAngleDelta(float delta_deg)
{
    while (delta_deg > 180.0f) {
        delta_deg -= 360.0f;
    }
    while (delta_deg < -180.0f) {
        delta_deg += 360.0f;
    }
    return delta_deg;
}

static void MapRun_ResetBallLapTurnTracking(uint8_t seed_current_yaw)
{
    g_ball_lap_right_turn_deg = 0.0f;
    g_ball_lap_previous_yaw_deg = yaw;
    g_ball_lap_yaw_valid = seed_current_yaw;
}

static void MapRun_AddBallLapYawSample(float current_yaw_deg)
{
    float yaw_delta_deg;

    if ((MapRun_IsBallMode() == 0U) ||
        (g_map_state != MAP_RUN_STATE_RUNNING)) {
        return;
    }

    if (g_ball_lap_yaw_valid == 0U) {
        g_ball_lap_previous_yaw_deg = current_yaw_deg;
        g_ball_lap_yaw_valid = 1U;
        return;
    }

    yaw_delta_deg = MapRun_NormalizeAngleDelta(
        current_yaw_deg - g_ball_lap_previous_yaw_deg);
    g_ball_lap_previous_yaw_deg = current_yaw_deg;

    /* BNO085 复位或异常跳变不能被误算成整圈转角。 */
    if (MapRun_AbsFloat(yaw_delta_deg) >
        MAP_RUN_BALL_YAW_MAX_SAMPLE_DELTA_DEG) {
        return;
    }

    /*
     * 停车只需要确认车身确实完成过一圈转向，不依赖 BNO085 的安装
     * 方向。累计相邻样本的绝对转角可避免顺时针符号相反时一直为 0。
     */
    g_ball_lap_right_turn_deg += MapRun_AbsFloat(yaw_delta_deg);
    g_ball_lap_right_turn_deg = MapRun_ClampFloat(
        g_ball_lap_right_turn_deg,
        0.0f,
        720.0f);
}

static void MapRun_UpdateFinishHeadingAssist(uint8_t yaw_rate_fresh)
{
    g_finish_heading_active = 0U;
    g_finish_heading_error_deg = 0.0f;
    g_finish_heading_rate_offset_deg_s = 0.0f;

    if ((g_question_mode != MAP_QUESTION_MODE_1_Q2) ||
        (g_map_state != MAP_RUN_STATE_RUNNING) ||
        (g_start_line_left == 0U) ||
        (g_run_distance_counts < MAP_RUN_FINISH_HEADING_ARM_COUNTS) ||
        (g_finish_heading_reference_valid == 0U) ||
        (yaw_rate_fresh == 0U)) {
        return;
    }

    /* 左转为正：目标航向减当前航向为正时，应给正的目标角速度。 */
    g_finish_heading_error_deg =
        MapRun_NormalizeAngleDelta(
            g_finish_heading_reference_yaw_deg - yaw);
    g_finish_heading_rate_offset_deg_s =
        MapRun_ClampFloat(
            MAP_RUN_FINISH_HEADING_KP_RATE_PER_DEG *
                g_finish_heading_error_deg,
            -MAP_RUN_FINISH_HEADING_RATE_LIMIT_DEG_S,
            MAP_RUN_FINISH_HEADING_RATE_LIMIT_DEG_S);
    g_finish_heading_active = 1U;
}

static float MapRun_CalculateQ3CatchAngleDeg(float speed_mm_s)
{
    float deceleration_m_s2;
    float damping_angle_deg;
    float catch_angle_deg;

    speed_mm_s = MapRun_ClampFloat(
        speed_mm_s,
        -MAP_RUN_Q3_CATCH_MAX_SPEED_MM_S,
        MAP_RUN_Q3_CATCH_MAX_SPEED_MM_S);

    /*
     * 使用固定 70 mm 制动距离计算速度能量，而不是用“距离 -5 cm
     * 还有多远”作分母。这样接近目标或穿过管道坡度奇异点时，角度不会
     * 因分母趋近于零而突变。+1.8 度是负端局部坡度的初始平衡估计，
     * 速度向左时减小角度反刹，速度反向后增大角度吸收回弹能量。
     */
    deceleration_m_s2 =
        (MapRun_AbsFloat(speed_mm_s) *
         MapRun_AbsFloat(speed_mm_s)) /
        (2.0f * MAP_RUN_Q3_BRAKE_DISTANCE_MM * 1000.0f);
    damping_angle_deg =
        deceleration_m_s2 /
        MAP_RUN_Q3_ROLL_ACCEL_M_S2_PER_RAD *
        MAP_RUN_Q3_RAD_TO_DEG *
        MAP_RUN_Q3_BRAKE_MODEL_GAIN;

    if (speed_mm_s < 0.0f) {
        catch_angle_deg =
            MAP_RUN_Q3_CATCH_BALANCE_BIAS_DEG -
            damping_angle_deg;
    } else if (speed_mm_s > 0.0f) {
        catch_angle_deg =
            MAP_RUN_Q3_CATCH_BALANCE_BIAS_DEG +
            damping_angle_deg;
    } else {
        catch_angle_deg =
            MAP_RUN_Q3_CATCH_BALANCE_BIAS_DEG;
    }

    return MapRun_ClampFloat(
        catch_angle_deg,
        -MAP_RUN_Q3_BRAKE_MAX_ANGLE_DEG,
        MAP_RUN_Q3_BRAKE_MAX_ANGLE_DEG);
}

static float MapRun_CalculateQ3BrakeMagnitudeDeg(float speed_mm_s)
{
    float deceleration_m_s2;
    float brake_angle_deg;

    /*
     * 恢复曾实测在 4.45 s 内完成的一组柔和制动模型。用当前左移速度
     * 加 40 mm/s 余量估计换坡期间继续积累的动量，再按 70 mm 距离
     * 计算制动角。角度只允许 1.5~3.0 度，避免提前制动时直接给满
     * -4.5 度，把球在到达 -5 cm 前推回去。
     */
    speed_mm_s = MapRun_ClampFloat(
        MapRun_AbsFloat(speed_mm_s) +
            MAP_RUN_Q3_BRAKE_SPEED_MARGIN_MM_S,
        0.0f,
        MAP_RUN_Q3_CATCH_MAX_SPEED_MM_S);
    deceleration_m_s2 =
        (speed_mm_s * speed_mm_s) /
        (2.0f * MAP_RUN_Q3_BRAKE_DISTANCE_MM * 1000.0f);
    brake_angle_deg =
        deceleration_m_s2 /
        MAP_RUN_Q3_ROLL_ACCEL_M_S2_PER_RAD *
        MAP_RUN_Q3_RAD_TO_DEG *
        MAP_RUN_Q3_BRAKE_MODEL_GAIN;

    return MapRun_ClampFloat(
        brake_angle_deg,
        MAP_RUN_Q3_BRAKE_MIN_ANGLE_DEG,
        MAP_RUN_Q3_BRAKE_MAX_ANGLE_DEG);
}

static float MapRun_UpdateQ3FreshSpeedMmS(void)
{
    uint32_t now_ms = (uint32_t) tick_ms;
    uint32_t frame_period_ms = now_ms - g_q3_motion_last_ms;
    float current_x_mm = g_ball_beam_controller.raw_x_mm;
    float frame_delta_mm = current_x_mm - g_q3_motion_last_x_mm;
    float speed_mm_s = g_ball_beam_controller.ball_speed_mm_s;

    /*
     * 通用球速跨多帧滤波，折返时方向变化会晚一拍。第三题额外按相邻
     * 有效帧计算即时速度，并与通用球速融合；坐标量化未变化时按 0 速
     * 处理，避免旧的高速值让制动持续过久。
     */
    if ((frame_period_ms > 0UL) &&
        (frame_period_ms <= MAP_RUN_Q3_STALE_TIMEOUT_MS)) {
        float frame_speed_mm_s = 0.0f;

        if (MapRun_AbsFloat(frame_delta_mm) >=
            MAP_RUN_Q3_CATCH_FRAME_DELTA_MIN_MM) {
            frame_speed_mm_s = MapRun_ClampFloat(
                frame_delta_mm * (1000.0f / (float) frame_period_ms),
                -MAP_RUN_Q3_CATCH_MAX_SPEED_MM_S,
                MAP_RUN_Q3_CATCH_MAX_SPEED_MM_S);
        }

        speed_mm_s =
            (1.0f - MAP_RUN_Q3_CATCH_FRAME_SPEED_BLEND) *
                speed_mm_s +
            MAP_RUN_Q3_CATCH_FRAME_SPEED_BLEND *
                frame_speed_mm_s;
    }

    g_q3_motion_last_ms = now_ms;
    g_q3_motion_last_x_mm = current_x_mm;
    return MapRun_ClampFloat(
        speed_mm_s,
        -MAP_RUN_Q3_CATCH_MAX_SPEED_MM_S,
        MAP_RUN_Q3_CATCH_MAX_SPEED_MM_S);
}

static void MapRun_SetQ3ManualBeamAngle(float beam_angle_deg)
{
    g_q3_manual_beam_active = 1U;
    g_q3_manual_beam_angle_deg = beam_angle_deg;
    BallBeamController_SetEnabled(&g_ball_beam_controller, 0U);

    if ((g_ball_stepper_zeroed != 0U) &&
        (StepperMotor_IsPositionTrackingEnabled() != 0U)) {
        (void) StepperMotor_SetTargetPositionPulses(
            BallBeamController_ConvertBeamAngleToPulses(
                &g_ball_beam_controller,
                beam_angle_deg));
    }
}

static float MapRun_GetQ3HoldProfileTargetSpeedMmS(
    float position_error_mm)
{
    float target_speed_abs_mm_s =
        MAP_RUN_Q3_HOLD_PROFILE_POSITION_RATE_PER_S *
        MapRun_AbsFloat(position_error_mm);

    if (target_speed_abs_mm_s >
        MAP_RUN_Q3_HOLD_PROFILE_MAX_SPEED_MM_S) {
        target_speed_abs_mm_s =
            MAP_RUN_Q3_HOLD_PROFILE_MAX_SPEED_MM_S;
    }

    /* 目标速度始终指向 -50 mm，接近终点时按距离自动降到 0。 */
    if (position_error_mm > 0.0f) {
        return -target_speed_abs_mm_s;
    }
    if (position_error_mm < 0.0f) {
        return target_speed_abs_mm_s;
    }
    return 0.0f;
}

static float MapRun_GetQ3CaptureTargetSpeedMmS(float current_x_mm)
{
    float position_error_mm =
        current_x_mm - MAP_RUN_Q3_NEGATIVE_HOLD_MM;

    /* 位置 P 只负责规划速度，接近 -50 mm 时预期速度自然降到 0。 */
    return MapRun_ClampFloat(
        -MAP_RUN_Q3_CAPTURE_POSITION_RATE_PER_S * position_error_mm,
        -MAP_RUN_Q3_CAPTURE_MAX_EXPECTED_SPEED_MM_S,
        MAP_RUN_Q3_CAPTURE_MAX_EXPECTED_SPEED_MM_S);
}

static float MapRun_CalculateQ3CapturePDAngleDeg(void)
{
    float target_speed_mm_s =
        MapRun_GetQ3CaptureTargetSpeedMmS(
            g_ball_beam_controller.raw_x_mm);
    float speed_error_mm_s =
        g_q3_fresh_speed_mm_s - target_speed_mm_s;
    float speed_correction_deg = MapRun_ClampFloat(
        MAP_RUN_Q3_CAPTURE_SPEED_KD_DEG_PER_MM_S *
            speed_error_mm_s,
        -MAP_RUN_Q3_CAPTURE_MAX_CORRECTION_DEG,
        MAP_RUN_Q3_CAPTURE_MAX_CORRECTION_DEG);

    /*
     * 正角驱动小球向负方向。实际速度比预期更负时减小角度进行反刹；
     * 实际速度不足时增大角度继续送球。捕获段不使用积分，避免积累后
     * 在 -50 mm 附近再次把球推出去。
     */
    g_q3_hold_profile_target_speed_mm_s = target_speed_mm_s;
    g_q3_hold_feedback_speed_mm_s = g_q3_fresh_speed_mm_s;
    g_q3_hold_profile_blend = 1.0f;
    return MapRun_ClampFloat(
        MAP_RUN_Q3_CATCH_BALANCE_BIAS_DEG + speed_correction_deg,
        MAP_RUN_Q3_CATCH_BALANCE_BIAS_DEG -
            MAP_RUN_Q3_CAPTURE_MAX_CORRECTION_DEG,
        MAP_RUN_Q3_CATCH_BALANCE_BIAS_DEG +
            MAP_RUN_Q3_CAPTURE_MAX_CORRECTION_DEG);
}

static float MapRun_GetQ3PositivePeakEstimateMm(void)
{
    float positive_speed_mm_s = MapRun_ClampFloat(
        g_ball_beam_controller.ball_speed_mm_s,
        0.0f,
        MAP_RUN_Q3_POSITIVE_PREDICTION_MAX_SPEED_MM_S);

    /*
     * 固定位置切换无法兼容每次不同的起滚速度。用平滑球速向前预测
     * 0.85 秒：覆盖摄像头帧间隔和步进换向滞后，低速时允许继续加速，
     * 高速时提前换坡，目标是让真实峰值
     * 落入 +40~+60 mm，而不是再次冲到 +100 mm 之外。
     */
    return g_ball_beam_controller.raw_x_mm +
           MAP_RUN_Q3_POSITIVE_PREDICTION_S * positive_speed_mm_s;
}

static float MapRun_GetQ3HoldProfileBlend(float position_error_mm)
{
    return MapRun_ClampFloat(
        (MapRun_AbsFloat(position_error_mm) -
         MAP_RUN_Q3_HOLD_PROFILE_BLEND_START_MM) /
            (MAP_RUN_Q3_HOLD_PROFILE_BLEND_FULL_MM -
             MAP_RUN_Q3_HOLD_PROFILE_BLEND_START_MM),
        0.0f,
        1.0f);
}

static void MapRun_ResetQ3StictionAssist(void)
{
    g_q3_hold_stiction_candidate_active = 0U;
    g_q3_hold_stiction_kick_active = 0U;
    g_q3_hold_stiction_candidate_start_ms = (uint32_t) tick_ms;
    g_q3_hold_stiction_kick_start_ms = (uint32_t) tick_ms;
    g_q3_hold_stiction_rearm_start_ms =
        (uint32_t) tick_ms - MAP_RUN_Q3_HOLD_STICTION_REARM_MS;
    g_q3_hold_stiction_candidate_x_mm =
        g_ball_beam_controller.filtered_x_mm;
    g_q3_hold_stiction_kick_deg = 0.0f;
    g_q3_hold_stiction_position_deg = 0.0f;
}

static void MapRun_UpdateQ3StictionAssist(
    float position_error_mm,
    float speed_feedback_mm_s)
{
    uint32_t now_ms = (uint32_t) tick_ms;
    uint8_t stiction_candidate;

    g_q3_hold_stiction_position_deg = 0.0f;

    /*
     * 4.5 度只用于确认后的短促破静摩擦。旧逻辑仅凭单帧没有位移就
     * 立即触发，摄像头 1 mm 量化会把仍在缓慢移动的小球误判为卡住，
     * 反复强推后形成大幅往返。动作出现位移、速度恢复或达到 100 ms
     * 都立即退出，并留出重触发冷却时间。
     */
    if (g_q3_hold_stiction_kick_active != 0U) {
        if ((MapRun_AbsFloat(g_q3_hold_frame_delta_mm) >
             MAP_RUN_Q3_HOLD_STICTION_FRAME_DELTA_MAX_MM) ||
            (MapRun_AbsFloat(speed_feedback_mm_s) >=
             MAP_RUN_Q3_HOLD_STICTION_RELEASE_SPEED_MM_S) ||
            ((now_ms - g_q3_hold_stiction_kick_start_ms) >=
             MAP_RUN_Q3_HOLD_STICTION_KICK_MS)) {
            g_q3_hold_stiction_kick_active = 0U;
            g_q3_hold_stiction_rearm_start_ms = now_ms;
            g_q3_hold_stiction_kick_deg = 0.0f;
        } else {
            g_q3_hold_stiction_position_deg =
                g_q3_hold_stiction_kick_deg;
            return;
        }
    }

    if ((now_ms - g_q3_hold_stiction_rearm_start_ms) <
        MAP_RUN_Q3_HOLD_STICTION_REARM_MS) {
        g_q3_hold_stiction_candidate_active = 0U;
        return;
    }

    stiction_candidate =
        ((MapRun_AbsFloat(position_error_mm) >=
          MAP_RUN_Q3_HOLD_STICTION_ERROR_MIN_MM) &&
         (MapRun_AbsFloat(speed_feedback_mm_s) <=
          MAP_RUN_Q3_HOLD_STICTION_SPEED_MAX_MM_S) &&
         (MapRun_AbsFloat(g_q3_hold_frame_delta_mm) <=
          MAP_RUN_Q3_HOLD_STICTION_FRAME_DELTA_MAX_MM)) ? 1U : 0U;

    if (stiction_candidate == 0U) {
        g_q3_hold_stiction_candidate_active = 0U;
        return;
    }

    if (g_q3_hold_stiction_candidate_active == 0U) {
        g_q3_hold_stiction_candidate_active = 1U;
        g_q3_hold_stiction_candidate_start_ms = now_ms;
        g_q3_hold_stiction_candidate_x_mm =
            g_ball_beam_controller.filtered_x_mm;
        return;
    }

    if (MapRun_AbsFloat(
            g_ball_beam_controller.filtered_x_mm -
            g_q3_hold_stiction_candidate_x_mm) >
        MAP_RUN_Q3_HOLD_STICTION_CONFIRM_TRAVEL_MM) {
        g_q3_hold_stiction_candidate_active = 0U;
        return;
    }

    if ((now_ms - g_q3_hold_stiction_candidate_start_ms) >=
        MAP_RUN_Q3_HOLD_STICTION_CONFIRM_MS) {
        g_q3_hold_stiction_candidate_active = 0U;
        g_q3_hold_stiction_kick_active = 1U;
        g_q3_hold_stiction_kick_start_ms = now_ms;
        g_q3_hold_stiction_kick_deg =
            (position_error_mm > 0.0f) ?
                MAP_RUN_Q3_HOLD_STICTION_BREAKAWAY_ANGLE_DEG :
                -MAP_RUN_Q3_HOLD_STICTION_BREAKAWAY_ANGLE_DEG;
        g_q3_hold_stiction_position_deg =
            g_q3_hold_stiction_kick_deg;
    }
}

static void MapRun_UpdateQ3AdaptiveHold(void)
{
    uint32_t frame_id;
    uint32_t frame_period_ms;
    float position_error_mm;
    float speed_mm_s;
    float position_feedback_mm;
    float speed_feedback_mm_s;
    float frame_speed_mm_s;
    float candidate_bias_deg;
    float normal_hold_angle_deg;
    float profile_hold_angle_deg;
    float hold_angle_limit_deg;
    float hold_angle_deg;
    float filtered_hold_angle_deg;
    float angle_step_deg;
    uint8_t fresh_motion_direction_valid;
    uint8_t fast_approach_brake_active;
    uint8_t fast_escape_brake_active;

    if (((g_q3_state != MAP_Q3_STATE_LEVEL_SETTLE) &&
         (g_q3_state != MAP_Q3_STATE_HOLD_NEGATIVE)) ||
        (BallBeamController_IsControlReady(
             &g_ball_beam_controller) == 0U)) {
        return;
    }

    frame_id = g_ball_beam_controller.camera_frame_id;
    if (frame_id == g_q3_hold_last_camera_frame_id) {
        return;
    }
    g_q3_hold_last_camera_frame_id = frame_id;
    g_q3_hold_frame_delta_mm =
        g_ball_beam_controller.filtered_x_mm -
        g_q3_hold_last_filtered_x_mm;
    g_q3_hold_last_filtered_x_mm =
        g_ball_beam_controller.filtered_x_mm;

    /*
     * 一旦低速进入合格区，验稳期间只保持已经学到的局部平衡角。
     * 不再叠加带相位滞后的 P/D，避免把本来已经停住的球再次推出去。
     */
    if (g_q3_settle_lock_active != 0U) {
        /*
         * 完成后保持已经实测有效的局部平衡角。摄像头速度和单帧位移
         * 会受量化影响，不能仅因速度尖峰就重新启动闭环；只有位置真正
         * 离开 -60~-40 mm 合格区时，才恢复自适应拉回。
         */
        if ((g_q3_state == MAP_Q3_STATE_HOLD_NEGATIVE) &&
            ((g_ball_beam_controller.filtered_x_mm <
              MAP_RUN_Q3_SETTLE_MIN_POSITION_MM) ||
             (g_ball_beam_controller.filtered_x_mm >
              MAP_RUN_Q3_SETTLE_MAX_POSITION_MM))) {
            g_q3_settle_lock_active = 0U;
            g_q3_settle_capture_active = 0U;
            MapRun_ResetQ3MotionWindow();
            MapRun_SendBluetoothText(
                "[Q3 HOLD] Left valid zone; adaptive hold restored\r\n");
        } else {
            g_q3_hold_feedback_speed_mm_s = 0.0f;
            g_q3_hold_profile_target_speed_mm_s = 0.0f;
            g_q3_hold_profile_blend = 0.0f;
            g_q3_hold_stiction_position_deg = 0.0f;
            g_q3_manual_beam_angle_deg = g_q3_settle_lock_angle_deg;
            g_q3_manual_beam_active = 1U;
            BallBeamController_SetEnabled(&g_ball_beam_controller, 0U);
            return;
        }
    }

    frame_period_ms =
        (uint32_t) tick_ms - g_q3_hold_last_update_ms;
    g_q3_hold_last_update_ms = (uint32_t) tick_ms;
    if (frame_period_ms > MAP_RUN_Q3_HOLD_MAX_FRAME_MS) {
        frame_period_ms = MAP_RUN_Q3_HOLD_MAX_FRAME_MS;
    }

    position_error_mm =
        g_ball_beam_controller.filtered_x_mm -
        MAP_RUN_Q3_NEGATIVE_HOLD_MM;
    speed_mm_s = g_ball_beam_controller.ball_speed_mm_s;

    /*
     * I 项只在目标附近且低速时学习当前位置的等效坡度。这样同一根管道
     * 不同位置的弯度、摩擦变化会形成不同保持偏置，又不会在高速制动时
     * 积分饱和。P、D 始终负责位置拉回和速度反刹。
     */
    if ((MapRun_AbsFloat(position_error_mm) <=
         MAP_RUN_Q3_HOLD_I_MAX_ERROR_MM) &&
        (MapRun_AbsFloat(speed_mm_s) <=
         MAP_RUN_Q3_HOLD_I_MAX_SPEED_MM_S)) {
        candidate_bias_deg =
            g_q3_hold_bias_deg +
            MAP_RUN_Q3_HOLD_KI_DEG_PER_MM_S *
                position_error_mm *
                ((float) frame_period_ms / 1000.0f);
        g_q3_hold_bias_deg = MapRun_ClampFloat(
            candidate_bias_deg,
            -MAP_RUN_Q3_HOLD_MAX_BIAS_DEG,
            MAP_RUN_Q3_HOLD_MAX_BIAS_DEG);
    }

    position_feedback_mm =
        (MapRun_AbsFloat(position_error_mm) <=
         MAP_RUN_Q3_HOLD_POSITION_DEADBAND_MM) ?
            0.0f : position_error_mm;
    speed_feedback_mm_s = speed_mm_s;

    /*
     * 聚合球速约三帧才更新，在折返点会明显滞后。球每帧移动达到
     * 0.5 mm 时，按摄像头 40 FPS 估算即时速度并提高其权重，让 D 项
     * 在本次运动方向上提前制动；低速时仍使用聚合速度，避免 1 mm
     * 坐标量化造成摆杆抖动。
     */
    if (MapRun_AbsFloat(g_q3_hold_frame_delta_mm) >=
        MAP_RUN_Q3_HOLD_FRAME_SPEED_DELTA_MM) {
        frame_speed_mm_s = MapRun_ClampFloat(
            g_q3_hold_frame_delta_mm *
                (1000.0f / MAP_RUN_Q3_HOLD_FRAME_PERIOD_MS),
            -MAP_RUN_Q3_HOLD_MAX_FRAME_SPEED_MM_S,
            MAP_RUN_Q3_HOLD_MAX_FRAME_SPEED_MM_S);
        speed_feedback_mm_s =
            (1.0f - MAP_RUN_Q3_HOLD_FRAME_SPEED_BLEND) *
                speed_feedback_mm_s +
            MAP_RUN_Q3_HOLD_FRAME_SPEED_BLEND *
                frame_speed_mm_s;
    } else if (((g_q3_hold_frame_delta_mm >
                 MAP_RUN_Q3_HOLD_DIRECTION_DELTA_MM) &&
                (speed_feedback_mm_s < 0.0f)) ||
               ((g_q3_hold_frame_delta_mm <
                 -MAP_RUN_Q3_HOLD_DIRECTION_DELTA_MM) &&
                (speed_feedback_mm_s > 0.0f))) {
        speed_feedback_mm_s = 0.0f;
    }

    if (MapRun_AbsFloat(speed_feedback_mm_s) <=
        MAP_RUN_Q3_HOLD_SPEED_DEADBAND_MM_S) {
        speed_feedback_mm_s = 0.0f;
    }
    g_q3_hold_feedback_speed_mm_s = speed_feedback_mm_s;

    g_q3_hold_profile_target_speed_mm_s =
        MapRun_GetQ3HoldProfileTargetSpeedMmS(
            position_error_mm);
    g_q3_hold_profile_blend =
        MapRun_GetQ3HoldProfileBlend(position_error_mm);

    MapRun_UpdateQ3StictionAssist(
        position_error_mm,
        speed_feedback_mm_s);

    normal_hold_angle_deg =
        g_q3_hold_bias_deg +
        MAP_RUN_Q3_HOLD_KP_DEG_PER_MM *
            position_feedback_mm +
        MAP_RUN_Q3_HOLD_KD_DEG_PER_MM_S *
            speed_feedback_mm_s;
    profile_hold_angle_deg =
        MAP_RUN_Q3_HOLD_KD_DEG_PER_MM_S *
            (speed_feedback_mm_s -
             g_q3_hold_profile_target_speed_mm_s) +
        g_q3_hold_stiction_position_deg;

    /*
     * 普通收敛仍限制为 3.5 度。误差超过 8 mm、低速且连续帧几乎
     * 不动时，q3push 才会按目标方向请求完整 4.5 度破静摩擦；高速
     * 远离目标时也允许相同上限用于反刹，正常滑行仍保持普通上限。
     */
    hold_angle_limit_deg =
        (g_q3_hold_stiction_position_deg != 0.0f) ?
            MAP_RUN_Q3_HOLD_STICTION_MAX_ANGLE_DEG :
            MAP_RUN_Q3_HOLD_MAX_ANGLE_DEG;

    /*
     * 小球高速远离 -50 mm 时，角度本质上是在反刹而不是继续驱动。
     * 允许短暂使用 4.5 度，避免普通 3.5 度上限让小球穿过合格区后
     * 再做一次大摆；一旦重新朝向目标，立即恢复普通上限。
     */
    fresh_motion_direction_valid =
        (((g_q3_hold_frame_delta_mm * speed_feedback_mm_s) > 0.0f) &&
         (MapRun_AbsFloat(g_q3_hold_frame_delta_mm) >=
          MAP_RUN_Q3_HOLD_FRAME_SPEED_DELTA_MM)) ? 1U : 0U;
    fast_approach_brake_active =
        ((fresh_motion_direction_valid != 0U) &&
         ((position_error_mm * speed_feedback_mm_s) < 0.0f) &&
         (MapRun_AbsFloat(position_error_mm) <=
          MAP_RUN_Q3_HOLD_ENTRY_BRAKE_DISTANCE_MM) &&
         (MapRun_AbsFloat(speed_feedback_mm_s) >=
          MAP_RUN_Q3_HOLD_ENTRY_APPROACH_SPEED_MM_S)) ? 1U : 0U;
    fast_escape_brake_active =
        ((fresh_motion_direction_valid != 0U) &&
         ((position_error_mm * speed_feedback_mm_s) > 0.0f) &&
         (MapRun_AbsFloat(speed_feedback_mm_s) >=
          MAP_RUN_Q3_HOLD_ENTRY_ESCAPE_SPEED_MM_S)) ? 1U : 0U;
    if (fast_escape_brake_active != 0U) {
        hold_angle_limit_deg =
            MAP_RUN_Q3_HOLD_STICTION_MAX_ANGLE_DEG;
    }

    /*
     * 远离终点时跟随速度包络，让小球先加速再随距离主动减速。局部
     * 平衡偏置只在接近 -50 mm 时随混合系数逐步介入，避免远端坡度
     * 尚未确定时偏置压过速度方向；进入 6 mm 内仍完全使用原位置反馈。
     */
    hold_angle_deg = MapRun_ClampFloat(
        normal_hold_angle_deg *
            (1.0f - g_q3_hold_profile_blend) +
        profile_hold_angle_deg *
            g_q3_hold_profile_blend,
        -hold_angle_limit_deg,
        hold_angle_limit_deg);

    /*
     * 摄像头速度在折返点附近会因量化而换向。先对目标角做一阶低通，
     * 再限制单帧角度变化，避免步进电机在相邻帧之间大幅反复换向。
     */
    if ((g_q3_hold_entry_seed_pending != 0U) &&
        ((fast_approach_brake_active != 0U) ||
         (fast_escape_brake_active != 0U))) {
        /*
         * 距目标 25 mm 内仍高速接近，或已经越过目标继续远离时，
         * 0.45 度/帧斜率来不及制动。只有单帧位移与融合速度方向
         * 一致才绕过一次斜率限制，避免滞后速度在折返点误触发。
         */
        g_q3_manual_beam_angle_deg =
            (speed_feedback_mm_s > 0.0f) ?
                MAP_RUN_Q3_HOLD_ENTRY_BRAKE_ANGLE_DEG :
                -MAP_RUN_Q3_HOLD_ENTRY_BRAKE_ANGLE_DEG;
        g_q3_hold_entry_seed_pending = 0U;
        MapRun_SendBluetoothText(
            (fast_approach_brake_active != 0U) ?
                "[Q3] Terminal approach brake seeded\r\n" :
                "[Q3] Terminal escape brake seeded\r\n");
    } else if (g_q3_hold_stiction_position_deg != 0.0f) {
        /*
         * 破静摩擦时不再叠加软件角度斜率。步进脉冲频率本身已经限制
         * 机械运动速度，直接给目标角可省去约 0.5 s 的逐帧爬坡；检测
         * 到球重新移动后，本分支自动退出并恢复滤波和斜率限制。
         */
        g_q3_manual_beam_angle_deg = hold_angle_deg;
    } else {
        filtered_hold_angle_deg =
            g_q3_manual_beam_angle_deg +
            MAP_RUN_Q3_HOLD_ANGLE_FILTER_ALPHA *
                (hold_angle_deg - g_q3_manual_beam_angle_deg);
        angle_step_deg = MapRun_ClampFloat(
            filtered_hold_angle_deg - g_q3_manual_beam_angle_deg,
            -MAP_RUN_Q3_HOLD_MAX_ANGLE_STEP_DEG,
            MAP_RUN_Q3_HOLD_MAX_ANGLE_STEP_DEG);
        g_q3_manual_beam_angle_deg = MapRun_ClampFloat(
            g_q3_manual_beam_angle_deg + angle_step_deg,
            -hold_angle_limit_deg,
            hold_angle_limit_deg);

        if ((g_q3_hold_entry_seed_pending != 0U) &&
            (((uint32_t) tick_ms - g_q3_phase_start_ms) >=
             MAP_RUN_Q3_HOLD_ENTRY_BRAKE_WINDOW_MS)) {
            g_q3_hold_entry_seed_pending = 0U;
        }
    }
    g_q3_manual_beam_active = 1U;
    BallBeamController_SetEnabled(&g_ball_beam_controller, 0U);
}

static int64_t MapRun_AbsInt64(int64_t value)
{
    return (value < 0) ? (-value) : value;
}

static uint32_t MapRun_GetDistanceCounts(void)
{
    int64_t left_delta =
        (int64_t) MotorSpeed_GetLeftEncoderForwardCount() -
        (int64_t) g_left_start_count;
    int64_t right_delta =
        (int64_t) MotorSpeed_GetRightEncoderForwardCount() -
        (int64_t) g_right_start_count;
    uint64_t average_delta =
        (uint64_t) (MapRun_AbsInt64(left_delta) +
                    MapRun_AbsInt64(right_delta)) / 2ULL;

    if (average_delta > UINT32_MAX) {
        return UINT32_MAX;
    }

    return (uint32_t) average_delta;
}

static void MapRun_DisableBallBeam(void)
{
    MapRun_RestoreQ3PIDGains();
    g_q3_saved_pid_valid = 0U;
    g_q3_manual_beam_active = 0U;
    g_q3_hold_entry_seed_pending = 0U;
    MapRun_ResetQ3StictionAssist();
    g_q3_settle_lock_active = 0U;
    g_q3_settle_capture_active = 0U;
    g_q3_capture_pd_active = 0U;
    g_q3_manual_beam_angle_deg = 0.0f;
    g_q3_settle_lock_angle_deg = 0.0f;
    g_q3_hold_bias_deg = 0.0f;
    g_q3_hold_last_update_ms = (uint32_t) tick_ms;
    g_q3_hold_last_camera_frame_id =
        g_ball_beam_controller.camera_frame_id;
    g_q3_hold_last_filtered_x_mm =
        g_ball_beam_controller.filtered_x_mm;
    g_q3_hold_frame_delta_mm = 0.0f;
    g_q3_hold_feedback_speed_mm_s = 0.0f;
    g_q3_hold_profile_target_speed_mm_s = 0.0f;
    g_q3_hold_profile_blend = 0.0f;
    g_q3_hold_stiction_position_deg = 0.0f;
    MapRun_ResetQ4VelocityObserver();
    (void) BallBeamController_SetStaleTimeoutMs(
        &g_ball_beam_controller,
        BALL_BEAM_DEFAULT_STALE_TIMEOUT_MS);
    BallBeamController_SetEnabled(&g_ball_beam_controller, 0U);
    (void) BallBeamController_SetMotionProfile(
        &g_ball_beam_controller,
        MAP_RUN_BALL_SAFE_MAX_SPEED_MM_S,
        MAP_RUN_BALL_SAFE_POSITION_RATE_PER_S);
    (void) StepperMotor_EnablePositionTracking(0U);
}

static void MapRun_StopMotors(MapFinishReason reason)
{
    char response[144];
    uint8_t q4_finish_hold =
        ((reason == MAP_FINISH_Q4_B_PASSED) &&
         (MapRun_IsQ4Mode() != 0U)) ? 1U : 0U;
    uint8_t ball_lap_finish_hold =
        ((reason == MAP_FINISH_ONE_LAP) &&
         (MapRun_IsBallMode() != 0U) &&
         (BallBeamController_IsEnabled(
              &g_ball_beam_controller) != 0U)) ? 1U : 0U;
    uint8_t moving_ball_finish_hold =
        ((q4_finish_hold != 0U) ||
         (ball_lap_finish_hold != 0U)) ? 1U : 0U;

    g_q4_final_stop_active = 0U;
    g_q4_park_hold_active = 0U;
    g_q4_final_stop_start_rpm = 0.0f;
    g_q4_final_stop_left_duty_percent = 0.0f;
    g_q4_final_stop_right_duty_percent = 0.0f;
    g_ball_lap_final_stop_active = 0U;
    g_ball_lap_final_stop_start_rpm = 0.0f;
    g_ball_lap_final_stop_left_duty_percent = 0.0f;
    g_ball_lap_final_stop_right_duty_percent = 0.0f;

    if (q4_finish_hold != 0U) {
        /*
         * 停车后换成更慢的回中包络，并将位置保护限制为仅在超出
         * +/-10 mm 且近似静止时介入，减少低摩擦情况下反复穿过中心。
         */
        g_q4_park_hold_active = 1U;
        (void) BallBeamController_SetMotionProfile(
            &g_ball_beam_controller,
            MAP_RUN_Q4_PARK_MAX_SPEED_MM_S,
            MAP_RUN_Q4_PARK_POSITION_RATE_PER_S);
        g_q4_stop_feedforward_start_ms = tick_ms;
        /*
         * 只有球仍位于正侧并继续向正侧滚动时才保留停车接球角。
         * 若球已经越过中心或已经反向，重新施加 +0.8 度会制造第二次
         * 负向过冲，应立即交给速度阻尼和停车位置保护。
         */
        g_q4_stop_feedforward_active =
            ((BallBeamController_IsControlReady(
                  &g_ball_beam_controller) != 0U) &&
             (g_ball_beam_controller.filtered_x_mm > 0.0f) &&
             (g_q4_velocity_feedback_mm_s > 0.0f)) ? 1U : 0U;
        g_q4_feedforward_update_ms =
            tick_ms - MAP_RUN_CONTROL_PERIOD_MS;
        MapRun_UpdateMovingBallFeedforward();
        BallBeamController_Update(
            &g_ball_beam_controller, (uint32_t) tick_ms);
        if (StepperMotor_IsPositionTrackingEnabled() != 0U) {
            (void) StepperMotor_SetTargetPositionPulses(
                BallBeamController_GetTargetStepperPulses(
                    &g_ball_beam_controller));
        }
    } else if (ball_lap_finish_hold != 0U) {
        /*
         * 第五、六问只要求通过 A 点。底盘停下后继续以温和包络保持
         * 本轮锁定目标，并复用停车位置保护；若钢球之后慢慢漂出
         * +/-10 mm，保护会给出温和回拉角，而不是在完成报文后失控。
         */
        g_q4_park_hold_active = 1U;
        (void) BallBeamController_SetMotionProfile(
            &g_ball_beam_controller,
            MAP_RUN_BALL_PARK_MAX_SPEED_MM_S,
            MAP_RUN_BALL_PARK_POSITION_RATE_PER_S);
        g_q4_acceleration_feedforward_deg = 0.0f;
        g_q4_position_guard_deg = 0.0f;
        g_q4_velocity_damping_deg = 0.0f;
        g_q4_feedforward_update_ms =
            tick_ms - MAP_RUN_CONTROL_PERIOD_MS;
        MapRun_UpdateMovingBallFeedforward();
        BallBeamController_Update(
            &g_ball_beam_controller, (uint32_t) tick_ms);
        if (StepperMotor_IsPositionTrackingEnabled() != 0U) {
            (void) StepperMotor_SetTargetPositionPulses(
                BallBeamController_GetTargetStepperPulses(
                    &g_ball_beam_controller));
        }
    }

    MotorSpeedLoop_SetLeftWheelTargetRPM(0.0f);
    MotorSpeedLoop_SetRightWheelTargetRPM(0.0f);
    MotorSpeedLoop_EnableLeftWheel(0U);
    MotorSpeedLoop_EnableRightWheel(0U);

    /*
     * 第 2 问没有小球，终点允许使用 AT8236 主动刹车缩短滑行。
     * 其他模式以及人工/安全停机保持全低关断，避免改变其机械响应。
     */
    if ((reason == MAP_FINISH_ONE_LAP) &&
        (g_question_mode == MAP_QUESTION_MODE_1_Q2)) {
        MotorPWM_BrakeAllChannels();
        g_motor_brake_start_ms = tick_ms;
        g_motor_brake_active = 1U;
    } else {
        MotorPWM_StopAllChannels();
        g_motor_brake_active = 0U;
    }
    g_yaw_cascade_active = 0U;
    g_finish_heading_active = 0U;

    if (q4_finish_hold == 0U) {
        g_q4_stop_feedforward_active = 0U;
    }

    /*
     * 第四问通过 B 后只停止底盘，滚球闭环继续保持中心位置，避免车身
     * 制动造成的小球扰动无人修正。人工停机和所有故障仍关闭步进输出。
     */
    if (!((moving_ball_finish_hold != 0U) &&
          (BallBeamController_IsEnabled(
               &g_ball_beam_controller) != 0U))) {
        MapRun_DisableBallBeam();
    }
    g_q3_state = MAP_Q3_STATE_IDLE;
    g_q3_phase_start_ms = (uint32_t) tick_ms;

    if (g_map_state == MAP_RUN_STATE_RUNNING) {
        g_run_elapsed_ms = tick_ms - g_run_start_ms;
    }

    if (ball_lap_finish_hold != 0U) {
        g_ball_lap_completed_within_time =
            (g_run_elapsed_ms <= MAP_RUN_BALL_TIME_LIMIT_MS) ? 1U : 0U;
        (void) snprintf(
            response,
            sizeof(response),
            "[Q%u %s] Lap=%.2fs Target=%+.1f X=%+.1f MaxErr=%.1f mm\r\n",
            (unsigned int) MapRun_GetQuestionNumber(),
            (g_ball_lap_completed_within_time == 0U) ? "LATE" :
            ((g_ball_lap_limit_exceeded != 0U) ? "OVER" : "PASS"),
            (float) g_run_elapsed_ms / 1000.0f,
            g_ball_lap_active_target_mm,
            g_ball_beam_controller.filtered_x_mm,
            g_ball_lap_max_error_mm);
        MapRun_SendBluetoothText(response);
    }

    g_map_state = MAP_RUN_STATE_FINISHED;
    g_finish_reason = reason;
}

static void MapRun_UpdateMotorBrake(void)
{
    if ((g_motor_brake_active != 0U) &&
        ((tick_ms - g_motor_brake_start_ms) >=
         MAP_RUN_Q2_BRAKE_HOLD_MS)) {
        MotorPWM_StopAllChannels();
        g_motor_brake_active = 0U;
    }
}

static void MapRun_ResetEncoderFeedbackWatchdog(void)
{
    g_watchdog_left_encoder_count =
        MotorSpeed_GetLeftEncoderForwardCount();
    g_watchdog_right_encoder_count =
        MotorSpeed_GetRightEncoderForwardCount();
    g_watchdog_left_feedback_ms = tick_ms;
    g_watchdog_right_feedback_ms = tick_ms;
}

static void MapRun_UpdateEncoderFeedbackWatchdog(void)
{
    int32_t left_count =
        MotorSpeed_GetLeftEncoderForwardCount();
    int32_t right_count =
        MotorSpeed_GetRightEncoderForwardCount();
    float left_target =
        MotorSpeedLoop_GetLeftWheelTargetRPM();
    float right_target =
        MotorSpeedLoop_GetRightWheelTargetRPM();
    uint8_t left_fault = 0U;
    uint8_t right_fault = 0U;

    if (left_count != g_watchdog_left_encoder_count) {
        g_watchdog_left_encoder_count = left_count;
        g_watchdog_left_feedback_ms = tick_ms;
    }

    if (right_count != g_watchdog_right_encoder_count) {
        g_watchdog_right_encoder_count = right_count;
        g_watchdog_right_feedback_ms = tick_ms;
    }

    /* 有意降低 PWM 的停车阶段不使用“目标非零但无计数”故障判定。 */
    if ((g_q4_final_stop_active != 0U) ||
        (g_ball_lap_final_stop_active != 0U)) {
        g_watchdog_left_feedback_ms = tick_ms;
        g_watchdog_right_feedback_ms = tick_ms;
        return;
    }

    if (MapRun_AbsFloat(left_target) < 0.01f) {
        g_watchdog_left_feedback_ms = tick_ms;
    } else if ((tick_ms - g_watchdog_left_feedback_ms) >=
               MAP_RUN_ENCODER_FEEDBACK_TIMEOUT_MS) {
        left_fault = 1U;
    }

    if (MapRun_AbsFloat(right_target) < 0.01f) {
        g_watchdog_right_feedback_ms = tick_ms;
    } else if ((tick_ms - g_watchdog_right_feedback_ms) >=
               MAP_RUN_ENCODER_FEEDBACK_TIMEOUT_MS) {
        right_fault = 1U;
    }

    if ((left_fault != 0U) || (right_fault != 0U)) {
        MapRun_StopMotors(MAP_FINISH_ENCODER_FAULT);

        if ((left_fault != 0U) && (right_fault != 0U)) {
            MapRun_SendBluetoothText(
                "[SAFETY STOP] No encoder feedback: LEFT,RIGHT\r\n");
        } else if (left_fault != 0U) {
            MapRun_SendBluetoothText(
                "[SAFETY STOP] No encoder feedback: LEFT\r\n");
        } else {
            MapRun_SendBluetoothText(
                "[SAFETY STOP] No encoder feedback: RIGHT\r\n");
        }
    }
}

static void MapRun_PrepareMotorStart(void)
{
    Sensor_SetCalibrationKeyPressed(0U);
    if (BallBeamController_IsEnabled(
            &g_ball_beam_controller) == 0U) {
        StepperMotor_RequestStop();
    }
    g_motor_brake_active = 0U;
    g_q4_final_stop_active = 0U;
    g_q4_park_hold_active = 0U;
    g_q4_final_stop_start_rpm = 0.0f;
    g_q4_final_stop_left_duty_percent = 0.0f;
    g_q4_final_stop_right_duty_percent = 0.0f;
    g_ball_lap_final_stop_start_ms = tick_ms;
    g_ball_lap_final_stop_active = 0U;
    g_ball_lap_final_stop_start_rpm = 0.0f;
    g_ball_lap_final_stop_left_duty_percent = 0.0f;
    g_ball_lap_final_stop_right_duty_percent = 0.0f;
    MotorSpeedLoop_SetLeftWheelTargetRPM(0.0f);
    MotorSpeedLoop_SetRightWheelTargetRPM(0.0f);
    MotorSpeedLoop_EnableLeftWheel(0U);
    MotorSpeedLoop_EnableRightWheel(0U);
    MotorPWM_StopAllChannels();
    MotorSpeed_Reset();
    MotorSpeedLoop_ResetLeftWheelController();
    MotorSpeedLoop_ResetRightWheelController();
}

static uint8_t MapRun_PrepareQ4BallControl(void)
{
    char response[96];
    float start_error_mm;

    if (g_ball_stepper_zeroed == 0U) {
        MapRun_SendBluetoothText(
            "[Q4 BLOCKED] Re-zero level beam with BALL,ZERO\r\n");
        return 0U;
    }
    if (BallBeamController_IsControlReady(
            &g_ball_beam_controller) == 0U) {
        MapRun_SendBluetoothText(
            "[Q4 BLOCKED] No fresh valid camera frame\r\n");
        return 0U;
    }

    start_error_mm = MapRun_AbsFloat(
        g_ball_beam_controller.filtered_x_mm -
        MAP_RUN_Q4_TARGET_POSITION_MM);
    if (start_error_mm > MAP_RUN_Q4_START_CENTER_LIMIT_MM) {
        (void) snprintf(
            response,
            sizeof(response),
            "[Q4 BLOCKED] Place ball at O; current X=%.1f mm\r\n",
            g_ball_beam_controller.filtered_x_mm);
        MapRun_SendBluetoothText(response);
        return 0U;
    }

    if ((StepperMotor_IsPositionTrackingEnabled() == 0U) &&
        (StepperMotor_EnablePositionTracking(1U) == 0U)) {
        MapRun_SendBluetoothText(
            "[Q4 BLOCKED] Stepper is busy\r\n");
        return 0U;
    }

    /*
     * 先关闭一次控制器以清空旧积分和静摩擦补偿，再用当前已调 PID
     * 持续跟踪中心 0 mm；不覆盖 BALL,PID 和 BALL,TRIM 的赛前标定值。
     * 第四问使用独立的位置-速度包络：距离远时先加速回拉，接近中心
     * 时目标速度按剩余距离降低，实际回拉过快则 D 项自动反向制动。
     */
    g_q3_manual_beam_active = 0U;
    BallBeamController_SetEnabled(&g_ball_beam_controller, 0U);
    g_q4_acceleration_feedforward_deg = 0.0f;
    g_q4_position_guard_deg = 0.0f;
    g_q4_velocity_damping_deg = 0.0f;
    MapRun_ResetQ4VelocityObserver();
    g_q4_feedforward_update_ms =
        tick_ms - MAP_RUN_CONTROL_PERIOD_MS;
    g_q4_stop_feedforward_start_ms = tick_ms;
    g_q4_stop_feedforward_active = 0U;
    g_q4_final_stop_start_ms = tick_ms;
    g_q4_final_stop_active = 0U;
    g_q4_park_hold_active = 0U;
    g_q4_final_stop_start_rpm = 0.0f;
    g_q4_final_stop_left_duty_percent = 0.0f;
    g_q4_final_stop_right_duty_percent = 0.0f;
    g_ball_lap_final_stop_start_ms = tick_ms;
    g_ball_lap_final_stop_active = 0U;
    g_ball_lap_final_stop_start_rpm = 0.0f;
    g_ball_lap_final_stop_left_duty_percent = 0.0f;
    g_ball_lap_final_stop_right_duty_percent = 0.0f;
    g_q4_camera_fallback_active = 0U;
    (void) BallBeamController_SetExternalFeedforwardDeg(
        &g_ball_beam_controller, 0.0f);
    (void) BallBeamController_SetStaleTimeoutMs(
        &g_ball_beam_controller,
        MAP_RUN_Q4_STALE_TIMEOUT_MS);
    (void) BallBeamController_SetTargetPositionMm(
        &g_ball_beam_controller,
        MAP_RUN_Q4_TARGET_POSITION_MM);
    (void) BallBeamController_SetMotionProfile(
        &g_ball_beam_controller,
        MAP_RUN_Q4_RECOVERY_MAX_SPEED_MM_S,
        MAP_RUN_Q4_RECOVERY_POSITION_RATE_PER_S);
    BallBeamController_SetEnabled(&g_ball_beam_controller, 1U);
    (void) StepperMotor_SetTargetPositionPulses(
        BallBeamController_GetTargetStepperPulses(
            &g_ball_beam_controller));

    g_q4_max_ball_error_mm = start_error_mm;
    g_q4_ball_limit_exceeded =
        (start_error_mm > MAP_RUN_Q4_ALLOWED_ERROR_MM) ? 1U : 0U;
    MapRun_SendBluetoothText(
        "[Q4 BALL] Target=0 mm; return envelope=15 mm/s, 0.50/s\r\n");
    return 1U;
}

static uint8_t MapRun_PrepareBallLapControl(void)
{
    char response[128];
    float target_position_mm;
    float start_error_mm;

    if (MapRun_IsBallMode() == 0U) {
        return 1U;
    }
    if (g_ball_stepper_zeroed == 0U) {
        MapRun_SendBluetoothText(
            "[BALL LAP BLOCKED] Re-zero level beam with BALL,ZERO\r\n");
        return 0U;
    }
    if (BallBeamController_IsControlReady(
            &g_ball_beam_controller) == 0U) {
        MapRun_SendBluetoothText(
            "[BALL LAP BLOCKED] No fresh valid camera frame\r\n");
        return 0U;
    }

    target_position_mm =
        (g_question_mode == MAP_QUESTION_MODE_4_Q5) ?
            MAP_RUN_BALL_TARGET_POSITION_MM_Q5 :
            g_q6_target_position_mm;
    start_error_mm = MapRun_AbsFloat(
        g_ball_beam_controller.filtered_x_mm - target_position_mm);
    if (start_error_mm > MAP_RUN_BALL_START_TARGET_LIMIT_MM) {
        (void) snprintf(
            response,
            sizeof(response),
            "[BALL LAP BLOCKED] Place ball at %+.1f mm; X=%+.1f mm\r\n",
            target_position_mm,
            g_ball_beam_controller.filtered_x_mm);
        MapRun_SendBluetoothText(response);
        return 0U;
    }

    if ((StepperMotor_IsPositionTrackingEnabled() == 0U) &&
        (StepperMotor_EnablePositionTracking(1U) == 0U)) {
        MapRun_SendBluetoothText(
            "[BALL LAP BLOCKED] Stepper is busy\r\n");
        return 0U;
    }

    /*
     * 每圈开始时清空上一轮积分和静摩擦状态，但保留赛前通过 BALL,PID
     * 与 BALL,TRIM 标定的参数。Q5 强制锁定 0 mm，Q6 使用独立保存值。
     */
    g_q3_manual_beam_active = 0U;
    BallBeamController_SetEnabled(&g_ball_beam_controller, 0U);
    (void) BallBeamController_SetExternalFeedforwardDeg(
        &g_ball_beam_controller, 0.0f);
    (void) BallBeamController_SetStaleTimeoutMs(
        &g_ball_beam_controller,
        MAP_RUN_BALL_STALE_TIMEOUT_MS);
    (void) BallBeamController_SetTargetPositionMm(
        &g_ball_beam_controller,
        target_position_mm);
    (void) BallBeamController_SetMotionProfile(
        &g_ball_beam_controller,
        MAP_RUN_BALL_RECOVERY_MAX_SPEED_MM_S,
        MAP_RUN_BALL_RECOVERY_POSITION_RATE_PER_S);
    BallBeamController_SetEnabled(&g_ball_beam_controller, 1U);
    (void) StepperMotor_SetTargetPositionPulses(
        BallBeamController_GetTargetStepperPulses(
            &g_ball_beam_controller));

    g_ball_lap_active_target_mm = target_position_mm;
    g_ball_lap_max_error_mm = start_error_mm;
    g_ball_lap_limit_exceeded =
        (start_error_mm > MAP_RUN_BALL_ALLOWED_ERROR_MM) ? 1U : 0U;
    g_ball_lap_completed_within_time = 0U;
    g_ball_lap_camera_fallback_active = 0U;
    g_q4_acceleration_feedforward_deg = 0.0f;
    g_q4_position_guard_deg = 0.0f;
    g_q4_velocity_damping_deg = 0.0f;
    g_ball_lap_final_stop_start_ms = tick_ms;
    g_ball_lap_final_stop_active = 0U;
    g_ball_lap_final_stop_start_rpm = 0.0f;
    g_ball_lap_final_stop_left_duty_percent = 0.0f;
    g_ball_lap_final_stop_right_duty_percent = 0.0f;
    g_q4_feedforward_update_ms =
        tick_ms - MAP_RUN_CONTROL_PERIOD_MS;

    (void) snprintf(
        response,
        sizeof(response),
        "[Q%u BALL] Target=%+.1f mm; envelope=20 mm/s, 0.75/s\r\n",
        (unsigned int) MapRun_GetQuestionNumber(),
        target_position_mm);
    MapRun_SendBluetoothText(response);
    return 1U;
}

/*
 * 第六问赛场不依赖蓝牙设定目标。短按 B21 起跑前先读取最新摄像头
 * 位置，并把该位置同时保存为本轮目标；随后仍由正常起跑检查统一确认。
 */
static uint8_t MapRun_CaptureQ6TargetFromCamera(void)
{
    char response[96];
    float target_position_mm;

    if (g_question_mode != MAP_QUESTION_MODE_5_Q6) {
        return 1U;
    }
    if (BallBeamController_IsControlReady(
            &g_ball_beam_controller) == 0U) {
        MapRun_SendBluetoothText(
            "[Q6 BLOCKED] No fresh valid camera frame\r\n");
        return 0U;
    }

    target_position_mm = g_ball_beam_controller.filtered_x_mm;
    if (BallBeamController_SetTargetPositionMm(
            &g_ball_beam_controller,
            target_position_mm) == 0U) {
        MapRun_SendBluetoothText(
            "[Q6 BLOCKED] Current X is outside -120..120 mm\r\n");
        return 0U;
    }

    g_q6_target_position_mm = target_position_mm;
    g_ball_lap_active_target_mm = target_position_mm;
    (void) snprintf(
        response,
        sizeof(response),
        "[Q6 TARGET] B21 captured X=%+.1f mm; starting\r\n",
        target_position_mm);
    MapRun_SendBluetoothText(response);
    return 1U;
}

static void MapRun_EnterQ3NegativeDrive(void)
{
    char response[128];

    /* 预测峰值进入合格区后返程，适应每轮起滚速度差异。 */
    g_q3_brake_planned_speed_mm_s = 0.0f;
    g_q3_brake_angle_deg = 0.0f;
    g_q3_settle_capture_active = 0U;
    g_q3_capture_pd_active = 0U;
    g_q3_brake_low_speed_active = 0U;
    g_q3_motion_last_ms = (uint32_t) tick_ms;
    g_q3_motion_last_x_mm =
        g_ball_beam_controller.raw_x_mm;
    g_q3_fresh_speed_mm_s = 0.0f;
    MapRun_SetQ3ManualBeamAngle(
        MAP_RUN_Q3_LEFT_DRIVE_ANGLE_DEG);

    g_q3_state = MAP_Q3_STATE_DRIVE_NEGATIVE;
    g_q3_phase_start_ms = (uint32_t) tick_ms;
    (void) snprintf(
        response,
        sizeof(response),
        "[Q3] Positive predict X=%.1f V=%.1f Peak=%.1f -> +3.0 deg\r\n",
        g_ball_beam_controller.raw_x_mm,
        g_ball_beam_controller.ball_speed_mm_s,
        MapRun_GetQ3PositivePeakEstimateMm());
    MapRun_SendBluetoothText(response);
}

static void MapRun_EnterQ3NegativeBrake(void)
{
    char response[128];

    g_q3_brake_planned_speed_mm_s =
        MapRun_AbsFloat(g_q3_fresh_speed_mm_s) +
        MAP_RUN_Q3_BRAKE_SPEED_MARGIN_MM_S;
    g_q3_brake_angle_deg =
        -MapRun_CalculateQ3BrakeMagnitudeDeg(
            g_q3_fresh_speed_mm_s);
    MapRun_SetQ3ManualBeamAngle(g_q3_brake_angle_deg);

    g_q3_state = MAP_Q3_STATE_BRAKE_NEGATIVE;
    g_q3_phase_start_ms = (uint32_t) tick_ms;
    g_q3_settle_capture_active = 0U;
    g_q3_capture_pd_active = 0U;
    g_q3_brake_low_speed_active = 0U;
    g_q3_brake_low_speed_start_ms = (uint32_t) tick_ms;

    (void) snprintf(
        response,
        sizeof(response),
        "[Q3] Early soft brake X=%.1f V=%+.1f Vplan=%.1f A=%+.2f\r\n",
        g_ball_beam_controller.raw_x_mm,
        g_q3_fresh_speed_mm_s,
        g_q3_brake_planned_speed_mm_s,
        g_q3_brake_angle_deg);
    MapRun_SendBluetoothText(response);
}

static void MapRun_ResumeQ3NegativeDrive(void)
{
    /*
     * 能量制动若因局部坡度或静摩擦在 -40 mm 之前停住，不能提前交给
     * 终点闭环。重新给固定 +3 度向负端送球，达到明确左向速度后再由
     * 原有能量制动捕获，避免远离目标时运行慢速位置包络。
     */
    g_q3_brake_planned_speed_mm_s = 0.0f;
    g_q3_brake_angle_deg = 0.0f;
    g_q3_settle_capture_active = 0U;
    g_q3_capture_pd_active = 0U;
    g_q3_brake_low_speed_active = 0U;
    g_q3_motion_last_ms = (uint32_t) tick_ms;
    g_q3_motion_last_x_mm =
        g_ball_beam_controller.raw_x_mm;
    g_q3_fresh_speed_mm_s = 0.0f;
    MapRun_SetQ3ManualBeamAngle(
        MAP_RUN_Q3_LEFT_DRIVE_ANGLE_DEG);
    g_q3_state = MAP_Q3_STATE_DRIVE_NEGATIVE;
    g_q3_phase_start_ms = (uint32_t) tick_ms;
    MapRun_SendBluetoothText(
        "[Q3] Catch short of -40; resume fixed +3.0 deg\r\n");
}

static void MapRun_UpdateQ3NegativeBrakeAngle(void)
{
    float candidate_angle_deg;
    uint8_t rebound_confirmed;

    rebound_confirmed =
        ((g_q3_fresh_speed_mm_s >=
          MAP_RUN_Q3_CATCH_REBOUND_CONFIRM_MM_S) &&
         (g_ball_beam_controller.ball_speed_mm_s >=
          MAP_RUN_Q3_CATCH_REBOUND_CONFIRM_MM_S)) ? 1U : 0U;

    if (g_q3_settle_capture_active != 0U) {
        /*
         * 先给 100 ms 短上抬波克服换向和机构间隙，随后切入位置-速度
         * 串级 PD。位置误差生成预期速度，速度偏差实时调整角度，因此
         * 不再用固定 +1.8 度放任小球大幅自由往返。
         */
        if (g_q3_capture_pd_active != 0U) {
            g_q3_brake_angle_deg =
                MapRun_CalculateQ3CapturePDAngleDeg();
        } else if (((uint32_t) tick_ms -
                    g_q3_catch_pulse_start_ms) >=
                   MAP_RUN_Q3_CATCH_PULSE_MIN_MS) {
            char response[144];

            g_q3_capture_pd_active = 1U;
            g_q3_brake_angle_deg =
                MapRun_CalculateQ3CapturePDAngleDeg();
            (void) snprintf(
                response,
                sizeof(response),
                "[Q3] Capture PD X=%.1f V=%+.1f Vref=%+.1f A=%+.2f\r\n",
                g_ball_beam_controller.raw_x_mm,
                g_q3_fresh_speed_mm_s,
                g_q3_hold_profile_target_speed_mm_s,
                g_q3_brake_angle_deg);
            MapRun_SendBluetoothText(response);
        }
    } else if (rebound_confirmed != 0U) {
        /* 未触发预捕获的异常路径仍可在真实回弹后切正角。 */
        g_q3_brake_planned_speed_mm_s =
            g_q3_fresh_speed_mm_s;
        g_q3_brake_angle_deg =
            MapRun_CalculateQ3CatchAngleDeg(
                g_q3_fresh_speed_mm_s);
    } else if (g_q3_fresh_speed_mm_s < 0.0f) {
        /*
         * 左移期间按最新速度重算，但只允许负制动角继续增强，不因
         * 单帧零速或假反向提前撤坡。
         */
        g_q3_brake_planned_speed_mm_s =
            MapRun_AbsFloat(g_q3_fresh_speed_mm_s) +
            MAP_RUN_Q3_BRAKE_SPEED_MARGIN_MM_S;
        candidate_angle_deg =
            -MapRun_CalculateQ3BrakeMagnitudeDeg(
                g_q3_fresh_speed_mm_s);
        if ((g_q3_brake_angle_deg >= 0.0f) ||
            (candidate_angle_deg < g_q3_brake_angle_deg)) {
            g_q3_brake_angle_deg = candidate_angle_deg;
        }
    }
    MapRun_SetQ3ManualBeamAngle(g_q3_brake_angle_deg);
}

static void MapRun_EnterQ3LevelSettle(void)
{
    char response[96];
    float initial_bias_deg = MAP_RUN_Q3_HOLD_INITIAL_BIAS_DEG;
    uint8_t captured_in_valid_zone = 0U;

    /*
     * 制动低速后从当前能量吸收角平滑交给终点控制。只把初值限制在
     * +/-2 度，随后由积分学习 -5 cm 附近真实坡度；不同管段不水平、
     * 弯度和静摩擦变化都由这个局部偏置吸收，而不是假设全管水平。
     */
    if (g_q3_state == MAP_Q3_STATE_BRAKE_NEGATIVE) {
        initial_bias_deg = MapRun_ClampFloat(
            g_q3_brake_angle_deg,
            -2.0f,
            2.0f);
    }
    g_q3_manual_beam_angle_deg = initial_bias_deg;
    g_q3_hold_bias_deg = initial_bias_deg;
    g_q3_hold_last_update_ms = (uint32_t) tick_ms;
    g_q3_hold_last_camera_frame_id =
        g_ball_beam_controller.camera_frame_id;
    g_q3_hold_last_filtered_x_mm =
        g_ball_beam_controller.filtered_x_mm;
    g_q3_hold_frame_delta_mm = 0.0f;
    g_q3_hold_feedback_speed_mm_s = 0.0f;
    g_q3_hold_profile_target_speed_mm_s = 0.0f;
    g_q3_hold_profile_blend = 0.0f;
    MapRun_ResetQ3StictionAssist();
    g_q3_hold_entry_seed_pending = 0U;
    g_q3_settle_lock_active = 0U;
    g_q3_settle_capture_active = 0U;
    g_q3_capture_pd_active = 0U;
    g_q3_settle_lock_angle_deg = g_q3_hold_bias_deg;
    (void) BallBeamController_SetTargetPositionMm(
        &g_ball_beam_controller,
        MAP_RUN_Q3_NEGATIVE_HOLD_MM);
    g_q3_manual_beam_active = 1U;
    BallBeamController_SetEnabled(&g_ball_beam_controller, 0U);
    g_q3_state = MAP_Q3_STATE_LEVEL_SETTLE;
    g_q3_phase_start_ms = (uint32_t) tick_ms;
    MapRun_ResetQ3MotionWindow();

    /*
     * 若能量制动已经把球低速送入 -60~-40 mm，直接冻结刚刚验证有效的
     * 制动角并开始验稳。此时再启动位置包络或静摩擦 Kick 只会把已经
     * 接近静止的球重新推出合格区。
     */
    if ((g_ball_beam_controller.filtered_x_mm >=
         MAP_RUN_Q3_SETTLE_MIN_POSITION_MM) &&
        (g_ball_beam_controller.filtered_x_mm <=
         MAP_RUN_Q3_SETTLE_MAX_POSITION_MM) &&
        (MapRun_AbsFloat(g_q3_fresh_speed_mm_s) <=
         MAP_RUN_Q3_CATCH_LOW_SPEED_MM_S) &&
        (MapRun_AbsFloat(
             g_ball_beam_controller.ball_speed_mm_s) <=
         MAP_RUN_Q3_CATCH_LOW_SPEED_MM_S)) {
        g_q3_settle_lock_angle_deg = initial_bias_deg;
        g_q3_settle_lock_active = 1U;
        MapRun_SetQ3ManualBeamAngle(initial_bias_deg);
        captured_in_valid_zone = 1U;
    }

    if (captured_in_valid_zone != 0U) {
        (void) snprintf(
            response,
            sizeof(response),
            "[Q3] Direct catch; angle frozen at %+.2f deg\r\n",
            initial_bias_deg);
    } else {
        (void) snprintf(
            response,
            sizeof(response),
            "[Q3] Low-speed local hold; bias=%+.2f deg\r\n",
            initial_bias_deg);
    }
    MapRun_SendBluetoothText(response);
}

static void MapRun_FinishQ3(void)
{
    char response[128];
    float finish_x_mm = g_ball_beam_controller.filtered_x_mm;
    float finish_speed_mm_s = g_ball_beam_controller.ball_speed_mm_s;

    /*
     * 小球进入 -4~-6 cm 并连续低动后，冻结当时的有效角度完成验稳。
     * 完成后不再运行会继续注入能量的 P/D，也不恢复通用控制器。
     */
    MapRun_RestoreQ3PIDGains();
    g_q3_saved_pid_valid = 0U;
    MotorSpeedLoop_SetLeftWheelTargetRPM(0.0f);
    MotorSpeedLoop_SetRightWheelTargetRPM(0.0f);
    MotorSpeedLoop_EnableLeftWheel(0U);
    MotorSpeedLoop_EnableRightWheel(0U);
    MotorPWM_StopAllChannels();

    g_run_elapsed_ms = tick_ms - g_run_start_ms;
    g_q3_completed_within_time =
        (g_run_elapsed_ms <= MAP_RUN_Q3_TIME_LIMIT_MS) ? 1U : 0U;
    g_q3_state = MAP_Q3_STATE_HOLD_NEGATIVE;
    g_map_state = MAP_RUN_STATE_FINISHED;
    g_finish_reason = MAP_FINISH_Q3_COMPLETE;

    (void) snprintf(
        response,
        sizeof(response),
        "[Q3 %s] Settled -50 mm, time=%.2f s, X=%.1f V=%.1f Hold=%+.2f Bias=%+.2f\r\n",
        (g_q3_completed_within_time != 0U) ? "PASS" : "LATE",
        (float) g_run_elapsed_ms / 1000.0f,
        finish_x_mm,
        finish_speed_mm_s,
        g_q3_settle_lock_angle_deg,
        g_q3_hold_bias_deg);
    MapRun_SendBluetoothText(response);
}

static void MapRun_UpdateQ3Sequence(void)
{
    if ((MapRun_IsQ3Mode() == 0U) ||
        (g_map_state != MAP_RUN_STATE_RUNNING)) {
        return;
    }

    if ((g_run_elapsed_ms > MAP_RUN_Q3_TIME_LIMIT_MS) &&
        (g_q3_time_warning_sent == 0U)) {
        g_q3_time_warning_sent = 1U;
        MapRun_SendBluetoothText(
            "[Q3 WARN] 5 s exceeded; control continues for diagnosis\r\n");
    }

    /* 主循环远快于摄像头，阶段切换只允许每个有效新帧判断一次。 */
    if ((BallBeamController_IsControlReady(
             &g_ball_beam_controller) == 0U) ||
        (g_ball_beam_controller.camera_frame_id ==
         g_q3_last_camera_frame_id)) {
        return;
    }
    g_q3_last_camera_frame_id =
        g_ball_beam_controller.camera_frame_id;
    g_q3_fresh_speed_mm_s = MapRun_UpdateQ3FreshSpeedMmS();

    if (g_q3_state == MAP_Q3_STATE_TO_POSITIVE) {
        /*
         * 固定 +5 mm 切换时峰值只有 +22 mm，固定 +20 mm 在高起滚速度
         * 时又冲到 +111 mm。改用平滑球速预测峰值，并给摄像头 1 mm
         * 量化误差留出切换裕量，让高速度轮次更早换坡、低速度轮次
         * 更晚换坡。
         */
        if (((MapRun_GetQ3PositivePeakEstimateMm() +
              MAP_RUN_Q3_POSITIVE_SWITCH_MARGIN_MM) >=
             MAP_RUN_Q3_POSITIVE_TARGET_MIN_MM) &&
            (g_ball_beam_controller.ball_speed_mm_s >=
             MAP_RUN_Q3_POSITIVE_RETURN_MIN_SPEED_MM_S)) {
            MapRun_EnterQ3NegativeDrive();
        }
        return;
    }

    if (g_q3_state == MAP_Q3_STATE_DRIVE_NEGATIVE) {
        /*
         * 历史实测中，高速左移后使用柔和制动曾在 4.45 s 内停到
         * -51.9 mm。仅确认速度为负还不够：约 -20~-70 mm/s 时提前
         * 制动会让球停在正区并反复补送，因此先建立至少 100 mm/s 的
         * 左移动量，再由 70 mm 模型和负端预捕获共同停车。
         */
        if ((g_ball_beam_controller.raw_x_mm <=
             MAP_RUN_Q3_NEGATIVE_BRAKE_TRIGGER_MM) &&
            (g_q3_fresh_speed_mm_s <=
             MAP_RUN_Q3_NEGATIVE_BRAKE_MIN_SPEED_MM_S)) {
            MapRun_EnterQ3NegativeBrake();
        }
        return;
    }

    if (g_q3_state == MAP_Q3_STATE_BRAKE_NEGATIVE) {
        /*
         * 先用负角吸收左移动能，再提前切到正向局部平衡角覆盖步进换向
         * 延迟。首次回弹不代表已经停住，不能立即交给终点 PID；只有
         * 即时速度和聚合速度同时连续低速 75 ms，才允许状态交接。
         */
        /*
         * 步进电机从 -3 度换到正角约需 100 ms。若等球真正回弹才换向，
         * 日志表明小球会从 -44 mm 重新滚回接近 0 mm。小球进入合格区
         * 前约 5 mm 且融合速度已经下降时提前预置局部平衡角，让机械
         * 换向时间与剩余左移动量重叠。单帧速度存在跳变和滞后，因此
         * 这里使用融合速度判断动量，避免高速时过早撤掉 -3 度反刹。
         */
        if ((g_q3_settle_capture_active == 0U) &&
            (g_ball_beam_controller.raw_x_mm <=
             MAP_RUN_Q3_PRE_CATCH_TRIGGER_MM) &&
            (MapRun_AbsFloat(
                 g_ball_beam_controller.ball_speed_mm_s) <=
             MAP_RUN_Q3_PRE_CATCH_MAX_SPEED_MM_S)) {
            char response[112];

            g_q3_settle_capture_active = 1U;
            g_q3_capture_pd_active = 0U;
            g_q3_catch_pulse_start_ms = (uint32_t) tick_ms;
            g_q3_brake_angle_deg =
                MAP_RUN_Q3_PRE_CATCH_ANGLE_DEG;
            g_q3_brake_low_speed_active = 0U;
            MapRun_SetQ3ManualBeamAngle(
                g_q3_brake_angle_deg);
            (void) snprintf(
                response,
                sizeof(response),
                "[Q3] Pre-catch X=%.1f Vf=%+.1f V=%+.1f A=%+.2f\r\n",
                g_ball_beam_controller.raw_x_mm,
                g_q3_fresh_speed_mm_s,
                g_ball_beam_controller.ball_speed_mm_s,
                g_q3_brake_angle_deg);
            MapRun_SendBluetoothText(response);
        }

        MapRun_UpdateQ3NegativeBrakeAngle();

        if (((g_q3_settle_capture_active == 0U) ||
             (g_q3_capture_pd_active != 0U)) &&
            (MapRun_AbsFloat(g_q3_fresh_speed_mm_s) <=
             MAP_RUN_Q3_CATCH_LOW_SPEED_MM_S) &&
            (MapRun_AbsFloat(
                 g_ball_beam_controller.ball_speed_mm_s) <=
             MAP_RUN_Q3_CATCH_LOW_SPEED_MM_S)) {
            if (g_q3_brake_low_speed_active == 0U) {
                g_q3_brake_low_speed_active = 1U;
                g_q3_brake_low_speed_start_ms =
                    (uint32_t) tick_ms;
            } else if (((uint32_t) tick_ms -
                        g_q3_brake_low_speed_start_ms) >=
                       MAP_RUN_Q3_CATCH_LOW_SPEED_CONFIRM_MS) {
                if (g_ball_beam_controller.raw_x_mm <
                    MAP_RUN_Q3_TERMINAL_HANDOFF_MIN_MM) {
                    MapRun_EnterQ3LevelSettle();
                } else {
                    MapRun_ResumeQ3NegativeDrive();
                }
            }
        } else {
            g_q3_brake_low_speed_active = 0U;
        }
        return;
    }

    if (g_q3_state == MAP_Q3_STATE_LEVEL_SETTLE) {
        if ((g_ball_beam_controller.filtered_x_mm <
             MAP_RUN_Q3_SETTLE_MIN_POSITION_MM) ||
            (g_ball_beam_controller.filtered_x_mm >
             MAP_RUN_Q3_SETTLE_MAX_POSITION_MM)) {
            g_q3_settle_capture_active = 0U;
            g_q3_settle_lock_active = 0U;
            MapRun_ResetQ3MotionWindow();
            return;
        }

        /*
         * 低动确认后冻结当时已经让小球接近静止的实际角度，不再额外
         * 改成“偏置 +/- 固定推力”。冻结期间若仍移动超过门槛，立即
         * 解锁并恢复自适应；否则连续验稳 250 ms 后完成。
         */
        if (g_q3_settle_lock_active != 0U) {
            if ((MapRun_AbsFloat(g_q3_hold_frame_delta_mm) >
                 MAP_RUN_Q3_SETTLE_MAX_FRAME_DELTA_MM) ||
                (MapRun_AbsFloat(
                     g_ball_beam_controller.ball_speed_mm_s) >
                 MAP_RUN_Q3_SETTLE_MAX_SPEED_MM_S) ||
                (MapRun_AbsFloat(
                     g_ball_beam_controller.filtered_x_mm -
                     g_q3_window_start_x_mm) >
                 MAP_RUN_Q3_SETTLE_VERIFY_MAX_DELTA_MM)) {
                g_q3_settle_capture_active = 0U;
                g_q3_settle_lock_active = 0U;
                MapRun_ResetQ3MotionWindow();
                MapRun_SendBluetoothText(
                    "[Q3] Frozen hold released; motion resumed\r\n");
                return;
            }

            if ((MapRun_AbsFloat(g_q3_hold_frame_delta_mm) <=
                  MAP_RUN_Q3_SETTLE_CONFIRM_MAX_DELTA_MM) &&
                (MapRun_AbsFloat(
                     g_ball_beam_controller.ball_speed_mm_s) <=
                 MAP_RUN_Q3_SETTLE_FINISH_MAX_SPEED_MM_S) &&
                (MapRun_IsQ3MotionWindowStable(
                     MAP_RUN_Q3_SETTLE_VERIFY_MS,
                     MAP_RUN_Q3_SETTLE_VERIFY_MAX_DELTA_MM) != 0U)) {
                MapRun_FinishQ3();
            }
            return;
        }

        if ((MapRun_AbsFloat(
                 g_q3_hold_feedback_speed_mm_s) <=
             MAP_RUN_Q3_SETTLE_MAX_SPEED_MM_S) &&
            (MapRun_AbsFloat(
                 g_ball_beam_controller.ball_speed_mm_s) <=
             MAP_RUN_Q3_SETTLE_MAX_SPEED_MM_S) &&
            (MapRun_AbsFloat(g_q3_hold_frame_delta_mm) <=
             MAP_RUN_Q3_SETTLE_MAX_FRAME_DELTA_MM)) {
            if (MapRun_IsQ3MotionWindowStable(
                    MAP_RUN_Q3_SETTLE_CONFIRM_MS,
                    MAP_RUN_Q3_SETTLE_CONFIRM_MAX_DELTA_MM) != 0U) {
                g_q3_settle_lock_angle_deg =
                    g_q3_manual_beam_angle_deg;
                g_q3_settle_capture_active = 0U;
                g_q3_settle_lock_active = 1U;
                g_q3_hold_entry_seed_pending = 0U;
                MapRun_ResetQ3StictionAssist();
                MapRun_ResetQ3MotionWindow();
                MapRun_SendBluetoothText(
                    "[Q3] Low motion confirmed; angle frozen, verifying 250 ms\r\n");
            }
            return;
        }

        g_q3_settle_capture_active = 0U;
        MapRun_ResetQ3MotionWindow();
        return;
    }
}

static uint8_t MapRun_StartQ3(void)
{
    char response[96];

    if (g_ball_stepper_zeroed == 0U) {
        MapRun_SendBluetoothText(
            "[Q3 BLOCKED] Re-zero level beam with BALL,ZERO\r\n");
        return 0U;
    }
    if (BallBeamController_IsControlReady(
            &g_ball_beam_controller) == 0U) {
        MapRun_SendBluetoothText(
            "[Q3 BLOCKED] No fresh valid camera frame\r\n");
        return 0U;
    }
    if (MapRun_AbsFloat(
            g_ball_beam_controller.filtered_x_mm) >
        MAP_RUN_Q3_START_CENTER_LIMIT_MM) {
        (void) snprintf(
            response,
            sizeof(response),
            "[Q3 BLOCKED] Place ball at O; current X=%.1f mm\r\n",
            g_ball_beam_controller.filtered_x_mm);
        MapRun_SendBluetoothText(response);
        return 0U;
    }
    if ((StepperMotor_IsPositionTrackingEnabled() == 0U) &&
        (StepperMotor_EnablePositionTracking(1U) == 0U)) {
        MapRun_SendBluetoothText(
            "[Q3 BLOCKED] Stepper is busy\r\n");
        return 0U;
    }

    /* 第三问允许一次摄像头短帧间隙，不因 183 ms 抖动重置末端速度。 */
    (void) BallBeamController_SetStaleTimeoutMs(
        &g_ball_beam_controller,
        MAP_RUN_Q3_STALE_TIMEOUT_MS);

    /* 保存用户当前 PID，第三问捕获完成或退出时必须原样恢复。 */
    g_q3_saved_kp = g_ball_beam_controller.kp_deg_per_mm;
    g_q3_saved_ki = g_ball_beam_controller.ki_deg_per_mm_s;
    g_q3_saved_kd = g_ball_beam_controller.kd_deg_per_mm_s;
    g_q3_saved_pid_valid = 1U;
    g_q3_manual_beam_active = 0U;
    g_q3_hold_entry_seed_pending = 0U;
    g_q3_settle_lock_active = 0U;
    g_q3_settle_capture_active = 0U;
    g_q3_capture_pd_active = 0U;
    g_q3_manual_beam_angle_deg = 0.0f;
    g_q3_settle_lock_angle_deg = 0.0f;
    g_q3_brake_planned_speed_mm_s = 0.0f;
    g_q3_brake_angle_deg = 0.0f;
    g_q3_brake_low_speed_active = 0U;
    g_q3_brake_low_speed_start_ms = (uint32_t) tick_ms;
    g_q3_motion_last_ms = (uint32_t) tick_ms;
    g_q3_motion_last_x_mm =
        g_ball_beam_controller.raw_x_mm;
    g_q3_fresh_speed_mm_s = 0.0f;
    g_q3_hold_profile_target_speed_mm_s = 0.0f;
    g_q3_hold_profile_blend = 0.0f;
    MapRun_ResetQ3StictionAssist();

    MapRun_SetQ3ManualBeamAngle(
        MAP_RUN_Q3_POSITIVE_DRIVE_ANGLE_DEG);

    g_run_start_ms = tick_ms;
    g_run_elapsed_ms = 0UL;
    g_last_control_ms = tick_ms;
    g_last_speed_ms = tick_ms;
    g_last_telemetry_ms = tick_ms;
    g_run_distance_counts = 0U;
    g_q3_state = MAP_Q3_STATE_TO_POSITIVE;
    g_q3_completed_within_time = 0U;
    g_q3_time_warning_sent = 0U;
    g_q3_phase_start_ms = (uint32_t) tick_ms;
    MapRun_ResetQ3MotionWindow();
    g_q3_last_camera_frame_id =
        g_ball_beam_controller.camera_frame_id;
    g_control_mode = MAP_CONTROL_LINE_FOLLOW;
    g_finish_reason = MAP_FINISH_NONE;
    g_map_state = MAP_RUN_STATE_RUNNING;

    /* M2/Q3 自动开启 100 ms 诊断，便于赛前直接复盘整段动作。 */
    MapRun_EnableAutomaticBallLog("Q3");

    MapRun_SendBluetoothText(
        "[Q3 START] predicted +40 peak; brake at +20; capture PD\r\n");
    return 1U;
}

static void MapRun_Start(void)
{
    uint8_t initial_sensor;
    uint8_t heading_reference_fresh;

    MapRun_PrepareMotorStart();

    /*
     * M1~M5 分别对应第 2~6 问。M2 是静止滚球时序，不启动底盘，
     * 也不要求灰度探头位于黑线上。
     */
    if (MapRun_IsQuestionModeReady() == 0U) {
        g_map_state = MAP_RUN_STATE_READY;
        g_finish_reason = MAP_FINISH_NONE;
        MapRun_SendBluetoothText(
            "[START BLOCKED] Selected mode is not implemented\r\n");
        MapRun_SendQuestionModeStatus();
        return;
    }

    if (MapRun_IsQ3Mode() != 0U) {
        if (MapRun_StartQ3() == 0U) {
            MapRun_DisableBallBeam();
            g_map_state = MAP_RUN_STATE_READY;
            g_finish_reason = MAP_FINISH_NONE;
        }
        return;
    }

    if ((MapRun_IsQ4Mode() != 0U) &&
        (MapRun_PrepareQ4BallControl() == 0U)) {
        MapRun_DisableBallBeam();
        g_map_state = MAP_RUN_STATE_READY;
        g_finish_reason = MAP_FINISH_NONE;
        return;
    }

    if ((MapRun_IsBallMode() != 0U) &&
        (MapRun_PrepareBallLapControl() == 0U)) {
        MapRun_DisableBallBeam();
        g_map_state = MAP_RUN_STATE_READY;
        g_finish_reason = MAP_FINISH_NONE;
        return;
    }

    initial_sensor = Sensor_Read_Grayscale();

    /*
     * 没有检测到黑线时禁止起跑，避免传感器未对准或断线时直接直行。
     * A 点横线允许多路同时为黑，因此这里只拦截全白结果。
     */
    if (MapLineController_CountActive(initial_sensor) == 0U) {
        MapRun_StopMotors(MAP_FINISH_NO_START_LINE);
        MapRun_SendBluetoothText(
            "[MAP BLOCKED] No black line under sensor\r\n");
        return;
    }

    g_left_start_count = MotorSpeed_GetLeftEncoderForwardCount();
    g_right_start_count = MotorSpeed_GetRightEncoderForwardCount();
    g_run_distance_counts = 0U;
    g_run_start_ms = tick_ms;
    g_run_elapsed_ms = 0UL;
    g_last_control_ms = tick_ms;
    g_last_speed_ms = tick_ms;
    g_last_telemetry_ms = tick_ms;
    MapRun_ResetEncoderFeedbackWatchdog();

    g_sensor_value = initial_sensor;
    MapLineController_Reset(&g_line_controller);
    MapLineController_Update(&g_line_controller,
                             initial_sensor,
                             &g_line_result);
    g_sensor_active_count = g_line_result.active_count;
    g_start_line_left = 0U;
    g_lap_finish_armed = 0U;
    g_start_line_leave_count = 0U;
    g_finish_line_count = 0U;
    g_q4_right_second_seen = 0U;
    g_line_error = g_line_result.error;
    g_correction_rpm = g_line_result.correction_rpm;

    /* 一圈结束时应回到起跑航向；只接受起跑前仍新鲜的 BNO085 样本。 */
    heading_reference_fresh =
        MapYawRateController_IsFresh(&g_yaw_rate_controller,
                                     (uint32_t) tick_ms);
    g_finish_heading_reference_yaw_deg = yaw;
    g_finish_heading_reference_valid =
        ((BNO085_IsReady() != 0U) &&
         (heading_reference_fresh != 0U)) ? 1U : 0U;
    MapRun_ResetBallLapTurnTracking(
        (MapRun_IsBallMode() != 0U) ?
            g_finish_heading_reference_valid : 0U);
    MapRun_ResetYawCascade();
    if (MapRun_IsQ4Mode() != 0U) {
        g_center_speed_rpm = 0.0f;
        g_base_speed_rpm = g_center_speed_rpm;
    } else if (MapRun_IsBallMode() != 0U) {
        g_center_speed_rpm = 0.0f;
        g_base_speed_rpm = g_center_speed_rpm;
    }
    g_control_mode = MAP_CONTROL_LINE_FOLLOW;
    g_finish_reason = MAP_FINISH_NONE;
    g_map_state = MAP_RUN_STATE_RUNNING;

    MotorSpeedLoop_EnableLeftWheel(1U);
    MotorSpeedLoop_EnableRightWheel(1U);
    MotorSpeedLoop_SetLeftWheelTargetRPM(g_base_speed_rpm);
    MotorSpeedLoop_SetRightWheelTargetRPM(g_base_speed_rpm);

    /* 第四问每次起跑都重新打印表头，便于完整截取本轮数据。 */
    if (MapRun_IsQ4Mode() != 0U) {
        MapRun_EnableAutomaticBallLog("Q4");
    } else if (MapRun_IsBallMode() != 0U) {
        /*
         * 长 BALLLOG 在 115200 波特率下会明显阻塞 10 ms 巡线任务。
         * 整圈模式改发精简状态；需要完整诊断时可手动 BALL,LOG,ON。
         */
        g_ball_log_enabled = 0U;
        g_last_ball_lap_telemetry_ms =
            tick_ms - MAP_RUN_BALL_LAP_TELEMETRY_PERIOD_MS;
        MapRun_SendBluetoothText(
            "[BALL TELEMETRY] Q5/Q6 compact CSV enabled at 250 ms\r\n");
        MapRun_SendBluetoothText(
            "# QxBALL=time_s,x_mm,set_mm,error_mm,v_mm_s,angle_deg,"
            "max_error_mm,cam,age_ms,d_count,turn_deg\r\n");
    }
}

static void MapRun_StartSpeedTest(void)
{
    MapRun_PrepareMotorStart();
    MapRun_ResetYawCascade();

    g_left_start_count = MotorSpeed_GetLeftEncoderForwardCount();
    g_right_start_count = MotorSpeed_GetRightEncoderForwardCount();
    g_run_distance_counts = 0U;
    g_run_start_ms = tick_ms;
    g_run_elapsed_ms = 0UL;
    g_last_control_ms = tick_ms;
    g_last_speed_ms = tick_ms;
    g_last_telemetry_ms = tick_ms;
    MapRun_ResetEncoderFeedbackWatchdog();
    g_control_mode = MAP_CONTROL_SPEED_TEST;
    g_finish_reason = MAP_FINISH_NONE;
    g_map_state = MAP_RUN_STATE_RUNNING;

    MotorSpeedLoop_EnableLeftWheel(1U);
    MotorSpeedLoop_EnableRightWheel(1U);
    MotorSpeedLoop_SetLeftWheelTargetRPM(
        g_speed_test_left_target_rpm);
    MotorSpeedLoop_SetRightWheelTargetRPM(
        g_speed_test_right_target_rpm);
}

static uint8_t MapRun_ReadBluetoothByte(uint8_t *data)
{
    if ((data == NULL) ||
        (g_uart_rx_read_index == g_uart_rx_write_index)) {
        return 0U;
    }

    *data = g_uart_rx_buffer[g_uart_rx_read_index];
    g_uart_rx_read_index =
        (uint16_t) ((g_uart_rx_read_index + 1U) %
                    MAP_RUN_UART_RX_BUFFER_SIZE);
    return 1U;
}

static const char *MapRun_SkipSpaces(const char *text)
{
    while ((text != NULL) &&
           ((*text == ' ') || (*text == '\t'))) {
        text++;
    }

    return text;
}

static uint8_t MapRun_ParseFloat(const char **cursor, float *value)
{
    const char *start;
    char *end;
    float parsed_value;

    if ((cursor == NULL) || (*cursor == NULL) || (value == NULL)) {
        return 0U;
    }

    start = MapRun_SkipSpaces(*cursor);
    parsed_value = strtof(start, &end);
    if ((end == start) || (parsed_value != parsed_value)) {
        return 0U;
    }

    *value = parsed_value;
    *cursor = MapRun_SkipSpaces(end);
    return 1U;
}

static uint8_t MapRun_ParseTargetCommand(
    const char *command,
    float *left_target,
    float *right_target)
{
    const char *cursor = command + 7;

    if (MapRun_ParseFloat(&cursor, left_target) == 0U) {
        return 0U;
    }

    if (*cursor == ',') {
        cursor++;
        if (MapRun_ParseFloat(&cursor, right_target) == 0U) {
            return 0U;
        }
    } else {
        *right_target = *left_target;
    }

    if (*cursor != '\0') {
        return 0U;
    }

    if ((*left_target < -MAP_RUN_SPEED_TEST_MAX_ABS_RPM) ||
        (*left_target > MAP_RUN_SPEED_TEST_MAX_ABS_RPM) ||
        (*right_target < -MAP_RUN_SPEED_TEST_MAX_ABS_RPM) ||
        (*right_target > MAP_RUN_SPEED_TEST_MAX_ABS_RPM)) {
        return 0U;
    }

    return 1U;
}

static uint8_t MapRun_ParsePidCommand(
    const char *command,
    float *kp,
    float *ki,
    float *kd)
{
    const char *cursor = command + 4;

    if ((MapRun_ParseFloat(&cursor, kp) == 0U) ||
        (*cursor != ',')) {
        return 0U;
    }
    cursor++;

    if ((MapRun_ParseFloat(&cursor, ki) == 0U) ||
        (*cursor != ',')) {
        return 0U;
    }
    cursor++;

    if ((MapRun_ParseFloat(&cursor, kd) == 0U) ||
        (*cursor != '\0')) {
        return 0U;
    }

    if ((*kp < 0.0f) || (*kp > 10.0f) ||
        (*ki < 0.0f) || (*ki > 10.0f) ||
        (*kd < 0.0f) || (*kd > 10.0f)) {
        return 0U;
    }

    return 1U;
}

static void MapRun_SendCarStatus(void)
{
    char response[160];
    float left_kp;
    float left_ki;
    float right_kp;
    float right_ki;

    MotorSpeedLoop_GetLeftWheelPI(&left_kp, &left_ki);
    MotorSpeedLoop_GetRightWheelPI(&right_kp, &right_ki);

    (void) snprintf(
        response,
        sizeof(response),
        "[CAR] Mode=%s M%u Q%u Run=%u Target=%.1f,%.1f "
        "Speed=%.1f,%.1f\r\n",
        (g_control_mode == MAP_CONTROL_SPEED_TEST) ?
            "SPEED" : "MAP",
        (unsigned int) g_question_mode,
        (unsigned int) MapRun_GetQuestionNumber(),
        (unsigned int) (g_map_state == MAP_RUN_STATE_RUNNING),
        MotorSpeedLoop_GetLeftWheelTargetRPM(),
        MotorSpeedLoop_GetRightWheelTargetRPM(),
        MotorSpeed_GetLeftWheelRPM(),
        MotorSpeed_GetRightWheelRPM());
    MapRun_SendBluetoothText(response);

    (void) snprintf(
        response,
        sizeof(response),
        "[CAR] LPI=%.4f,%.4f RPI=%.4f,%.4f "
        "Duty=%.1f,%.1f\r\n",
        left_kp,
        left_ki,
        right_kp,
        right_ki,
        MotorSpeedLoop_GetLeftWheelDutyPercent(),
        MotorSpeedLoop_GetRightWheelDutyPercent());
    MapRun_SendBluetoothText(response);
}

static void MapRun_SendLineStatus(void)
{
    char response[80];

    (void) snprintf(
        response,
        sizeof(response),
        "[LINE] G=%u%u%u%u%u%u%u%u E=%+.2f A=%u\r\n",
        (unsigned int) ((g_sensor_value >> 7U) & 0x01U),
        (unsigned int) ((g_sensor_value >> 6U) & 0x01U),
        (unsigned int) ((g_sensor_value >> 5U) & 0x01U),
        (unsigned int) ((g_sensor_value >> 4U) & 0x01U),
        (unsigned int) ((g_sensor_value >> 3U) & 0x01U),
        (unsigned int) ((g_sensor_value >> 2U) & 0x01U),
        (unsigned int) ((g_sensor_value >> 1U) & 0x01U),
        (unsigned int) (g_sensor_value & 0x01U),
        g_line_error,
        (unsigned int) g_sensor_active_count);
    MapRun_SendBluetoothText(response);
}

static const char *MapRun_GetQ3StageText(void)
{
    switch (g_q3_state) {
        case MAP_Q3_STATE_TO_POSITIVE:
            return "PUSH+";

        case MAP_Q3_STATE_DRIVE_NEGATIVE:
            return "RETURN";

        case MAP_Q3_STATE_BRAKE_NEGATIVE:
            if (g_q3_settle_capture_active != 0U) {
                return "PRE-CATCH";
            }
            return "V-CATCH";

        case MAP_Q3_STATE_LEVEL_SETTLE:
            if (g_q3_settle_lock_active != 0U) {
                return "VERIFY-350";
            }
            return "ADAPT-END";

        case MAP_Q3_STATE_HOLD_NEGATIVE:
            return "HOLD-END";

        default:
            return "IDLE";
    }
}

static void MapRun_SendMapStatus(void)
{
    char response[160];
    uint8_t yaw_rate_fresh;

    g_measured_yaw_rate_deg_s =
        MapYawRateController_GetMeasuredYawRate(
            &g_yaw_rate_controller);
    yaw_rate_fresh =
        MapYawRateController_IsFresh(&g_yaw_rate_controller,
                                     (uint32_t) tick_ms);

    (void) snprintf(
        response,
        sizeof(response),
        "[MAP] M%u Q%u D=%lu Base=%.1f Line=%+.2f P=%+.1f\r\n",
        (unsigned int) g_question_mode,
        (unsigned int) MapRun_GetQuestionNumber(),
        (unsigned long) g_run_distance_counts,
        g_base_speed_rpm,
        g_line_error,
        g_correction_rpm);
    MapRun_SendBluetoothText(response);

    if (MapRun_IsQ3Mode() != 0U) {
        (void) snprintf(
            response,
            sizeof(response),
            "[MAP] Q3 Stage=%s Time=%.2f X=%.1f Pred=%.1f "
            "Set=%.1f\r\n",
            MapRun_GetQ3StageText(),
            (float) g_run_elapsed_ms / 1000.0f,
            g_ball_beam_controller.filtered_x_mm,
            g_ball_beam_controller.predicted_x_mm,
            g_ball_beam_controller.target_x_mm);
        MapRun_SendBluetoothText(response);

        (void) snprintf(
            response,
            sizeof(response),
            "[MAP] Q3 V=%+.1f Fresh=%+.1f Catch=%+.2f Manual=%u "
            "Cmd=%+.2f Bias=%+.2f Brake=%+.2f Cap=%u Lock=%u\r\n",
            g_ball_beam_controller.ball_speed_mm_s,
            g_q3_fresh_speed_mm_s,
            g_q3_brake_angle_deg,
            (unsigned int) g_q3_manual_beam_active,
            g_q3_manual_beam_angle_deg,
            g_q3_hold_bias_deg,
            g_q3_brake_angle_deg,
            (unsigned int) g_q3_settle_capture_active,
            (unsigned int) g_q3_settle_lock_active);
        MapRun_SendBluetoothText(response);

        (void) snprintf(
            response,
            sizeof(response),
            "[MAP] Q3 EndVref=%+.1f Blend=%.2f Vfb=%+.1f\r\n",
            g_q3_hold_profile_target_speed_mm_s,
            g_q3_hold_profile_blend,
            g_q3_hold_feedback_speed_mm_s);
        MapRun_SendBluetoothText(response);
    }

    if (MapRun_IsBallMode() != 0U) {
        (void) snprintf(
            response,
            sizeof(response),
            "[MAP] Curve H=%u Align=%u N=%u Deg=%.1f Mix=%.2f\r\n",
            (unsigned int) MapCurveHold_IsActive(&g_curve_hold),
            (unsigned int) MapCurveHold_IsAligning(&g_curve_hold),
            (unsigned int)
                MapCurveHold_GetCompletedCount(&g_curve_hold),
            MapCurveHold_GetRightTurnDeg(&g_curve_hold),
            MapCurveHold_GetExitLineBlend(&g_curve_hold));
        MapRun_SendBluetoothText(response);

        (void) snprintf(
            response,
            sizeof(response),
            "[MAP] Q%u Ball En=%u Ready=%u Fallback=%u "
            "X=%+.1f Set=%+.1f Max=%.1f %s FF=%+.2f\r\n",
            (unsigned int) MapRun_GetQuestionNumber(),
            (unsigned int)
                BallBeamController_IsEnabled(
                    &g_ball_beam_controller),
            (unsigned int)
                BallBeamController_IsControlReady(
                    &g_ball_beam_controller),
            (unsigned int) g_ball_lap_camera_fallback_active,
            g_ball_beam_controller.filtered_x_mm,
            g_ball_lap_active_target_mm,
            g_ball_lap_max_error_mm,
            (g_ball_lap_limit_exceeded == 0U) ? "OK" : "OVER",
            g_ball_beam_controller.external_feedforward_deg);
        MapRun_SendBluetoothText(response);

        (void) snprintf(
            response,
            sizeof(response),
            "[MAP] FixedStop D=%lu/%lu(%lu) Turn=%.1f/%.1f Yaw=%u\r\n",
            (unsigned long) g_run_distance_counts,
            (unsigned long) MAP_RUN_BALL_FIXED_STOP_COUNTS,
            (unsigned long) MAP_RUN_BALL_FIXED_STOP_MAX_COUNTS,
            g_ball_lap_right_turn_deg,
            MAP_RUN_BALL_FIXED_STOP_TURN_DEG,
            (unsigned int) g_ball_lap_yaw_valid);
        MapRun_SendBluetoothText(response);
    }

    if (g_question_mode == MAP_QUESTION_MODE_3_Q4) {
        (void) snprintf(
            response,
            sizeof(response),
            "[MAP] Q4 Phase=%s Plan=%.1f Right2=%u Seen=%u\r\n",
            MapRun_GetQ4SpeedPhaseText(),
            MapRun_GetQ4BaseSpeedRpm(),
            (unsigned int)
                ((g_sensor_value & MAP_RUN_Q4_RIGHT_SECOND_MASK) != 0U),
            (unsigned int) g_q4_right_second_seen);
        MapRun_SendBluetoothText(response);

        (void) snprintf(
            response,
            sizeof(response),
            "[MAP] Q4 Brake=%lu Stop=%lu Max=%lu\r\n",
            (unsigned long) MAP_RUN_Q4_BRAKE_START_COUNTS,
            (unsigned long) MAP_RUN_Q4_TARGET_STOP_COUNTS,
            (unsigned long) MAP_RUN_Q4_MAX_STOP_COUNTS);
        MapRun_SendBluetoothText(response);

        (void) snprintf(
            response,
            sizeof(response),
            "[MAP] Q4 Ball En=%u Ready=%u Drop=%u Fallback=%u "
            "X=%+.1f V=%.1f/%.1f/%.1f FF=%+.2f "
            "Guard=%+.2f Damp=%+.2f Max=%.1f Limit=%s\r\n",
            (unsigned int)
                BallBeamController_IsEnabled(&g_ball_beam_controller),
            (unsigned int)
                BallBeamController_IsControlReady(
                    &g_ball_beam_controller),
            (unsigned int) g_ball_beam_controller.dropout_active,
            (unsigned int) g_q4_camera_fallback_active,
            g_ball_beam_controller.filtered_x_mm,
            g_ball_beam_controller.ball_speed_mm_s,
            g_q4_fast_ball_speed_mm_s,
            g_q4_velocity_feedback_mm_s,
            g_ball_beam_controller.external_feedforward_deg,
            g_q4_position_guard_deg,
            g_q4_velocity_damping_deg,
            g_q4_max_ball_error_mm,
            (g_q4_ball_limit_exceeded == 0U) ? "OK" : "OVER");
        MapRun_SendBluetoothText(response);

    }

    (void) snprintf(
        response,
        sizeof(response),
        "[MAP] YawRate T=%+.1f A=%+.1f Turn=%+.1f C=%u F=%u\r\n",
        g_target_yaw_rate_deg_s,
        g_measured_yaw_rate_deg_s,
        g_yaw_turn_rpm,
        (unsigned int) g_yaw_cascade_active,
        (unsigned int) yaw_rate_fresh);
    MapRun_SendBluetoothText(response);

    if (g_question_mode == MAP_QUESTION_MODE_1_Q2) {
        (void) snprintf(
            response,
            sizeof(response),
            "[MAP] FinishHeading Ref=%+.1f Err=%+.1f Add=%+.1f V=%u A=%u\r\n",
            g_finish_heading_reference_yaw_deg,
            g_finish_heading_error_deg,
            g_finish_heading_rate_offset_deg_s,
            (unsigned int) g_finish_heading_reference_valid,
            (unsigned int) g_finish_heading_active);
        MapRun_SendBluetoothText(response);
    }

    (void) snprintf(
        response,
        sizeof(response),
        "[MAP] Target=%.1f,%.1f Speed=%.1f,%.1f\r\n",
        MotorSpeedLoop_GetLeftWheelTargetRPM(),
        MotorSpeedLoop_GetRightWheelTargetRPM(),
        MotorSpeed_GetLeftWheelRPM(),
        MotorSpeed_GetRightWheelRPM());
    MapRun_SendBluetoothText(response);
}

static void MapRun_SendCameraStatus(void)
{
    char response[160];
    char latest_line[CAMERA_LINK_LINE_BUFFER_SIZE];
    uint32_t age_ms = 0UL;

    if (CameraLink_HasFrame() != 0U) {
        age_ms =
            (uint32_t) tick_ms - CameraLink_GetLastFrameMs();
    }

    (void) snprintf(
        response,
        sizeof(response),
        "[CAM] Frames=%lu Bytes=%lu Overflow=%lu Age=%lums\r\n",
        (unsigned long) CameraLink_GetFrameCount(),
        (unsigned long) CameraLink_GetRxByteCount(),
        (unsigned long) CameraLink_GetOverflowCount(),
        (unsigned long) age_ms);
    MapRun_SendBluetoothText(response);

    if (CameraLink_CopyLatestLine(latest_line,
                                  sizeof(latest_line)) != 0U) {
        MapRun_SendBluetoothText("[CAM] Data=");
        MapRun_SendBluetoothText(latest_line);
        MapRun_SendBluetoothText("\r\n");
    } else {
        MapRun_SendBluetoothText("[CAM] Data=NONE\r\n");
    }
}

static void MapRun_SendBallBeamStatus(void)
{
    char response[288];
    uint32_t valid_age_ms = 0UL;

    if (g_ball_beam_controller.measurement_valid != 0U) {
        valid_age_ms =
            (uint32_t) tick_ms -
            g_ball_beam_controller.last_valid_receive_ms;
    }

    (void) snprintf(
        response,
        sizeof(response),
        "[BALL] En=%u Manual=%u Zero=%u Ready=%u Stale=%u Drop=%u/%u Sign=%d "
        "Frame=%lu Age=%lums Timeout=%lums\r\n",
        (unsigned int)
            BallBeamController_IsEnabled(&g_ball_beam_controller),
        (unsigned int) g_q3_manual_beam_active,
        (unsigned int) g_ball_stepper_zeroed,
        (unsigned int)
            BallBeamController_IsControlReady(
                &g_ball_beam_controller),
        (unsigned int) g_ball_beam_controller.stale,
        (unsigned int) g_ball_beam_controller.dropout_active,
        (unsigned int)
            g_ball_beam_controller.dropout_consecutive_count,
        (int) g_ball_beam_controller.output_sign,
        (unsigned long) g_ball_beam_controller.camera_frame_id,
        (unsigned long) valid_age_ms,
        (unsigned long) g_ball_beam_controller.stale_timeout_ms);
    MapRun_SendBluetoothText(response);

    (void) snprintf(
        response,
        sizeof(response),
        "[BALL] X=%.1f Pred=%.1f Set=%.1f V=%+.1f "
        "E=%+.1f CtrlE=%+.1f "
        "Angle=%+.2f Limit=%.2f Pulse=%ld Pos=%ld Tgt=%ld\r\n",
        g_ball_beam_controller.filtered_x_mm,
        g_ball_beam_controller.predicted_x_mm,
        g_ball_beam_controller.target_x_mm,
        g_ball_beam_controller.ball_speed_mm_s,
        g_ball_beam_controller.position_error_mm,
        g_ball_beam_controller.control_error_mm,
        g_ball_beam_controller.target_beam_angle_deg,
        g_ball_beam_controller.dynamic_angle_limit_deg,
        (long)
            BallBeamController_GetTargetStepperPulses(
                &g_ball_beam_controller),
        (long) StepperMotor_GetPositionPulses(),
        (long) StepperMotor_GetTargetPositionPulses());
    MapRun_SendBluetoothText(response);

    (void) snprintf(
        response,
        sizeof(response),
        "[BALL] PID=%.4f,%.4f,%.4f Iout=%+.2f Trim=%+.2f FF=%+.2f "
        "Kick=%+.2f Active=%u Wait=%lums Delay=%lums "
        "Vref=%+.1f Blend=%.2f Profile=%.0f,%.2f "
        "Parsed=%lu Bad=%lu Dup=%lu DtErr=%lu Drop=%lu/%lu/%lu\r\n",
        g_ball_beam_controller.kp_deg_per_mm,
        g_ball_beam_controller.ki_deg_per_mm_s,
        g_ball_beam_controller.kd_deg_per_mm_s,
        g_ball_beam_controller.integral_output_deg,
        g_ball_beam_controller.level_trim_deg,
        g_ball_beam_controller.external_feedforward_deg,
        g_ball_beam_controller.breakaway_output_deg,
        (unsigned int) g_ball_beam_controller.breakaway_active,
        (unsigned long)
            g_ball_beam_controller.breakaway_stationary_ms,
        (unsigned long)
            g_ball_beam_controller.prediction_delay_ms,
        g_ball_beam_controller.profile_target_speed_mm_s,
        g_ball_beam_controller.profile_blend,
        g_ball_beam_controller.profile_max_speed_mm_s,
        g_ball_beam_controller.profile_position_rate_per_s,
        (unsigned long) g_ball_beam_controller.parsed_frame_count,
        (unsigned long) g_ball_beam_controller.rejected_frame_count,
        (unsigned long) g_ball_beam_controller.duplicate_frame_count,
        (unsigned long) g_ball_beam_controller.timing_fault_count,
        (unsigned long) g_ball_beam_controller.dropout_prediction_count,
        (unsigned long) g_ball_beam_controller.dropout_recovery_count,
        (unsigned long) g_ball_beam_controller.dropout_fault_count);
    MapRun_SendBluetoothText(response);
}

static void MapRun_SendBallLogHeader(void)
{
    MapRun_SendBluetoothText(
        "# BALLLOG=ms,frame,age,dt,vdt,en,stale,drop,raw,x,pred,set,v,e,ce,"
        "vref,blend,p,d,i,trim,ff,kick,kickms,kicka,profile,angle,limit,"
        "cmd,tgt,pos,lag,q3angle,q3bias,q3cap,q3lock,q3dx,q3vfb,"
        "q3vref,q3blend,q3push,q4vfast,q4vfb,q4guard,q4damp,q4accel\r\n");
}

static void MapRun_SendBallLogSample(void)
{
    char response[448];
    uint32_t valid_age_ms = 0UL;
    int32_t controller_target_pulses;
    int32_t stepper_target_pulses;
    int32_t stepper_position_pulses;

    if (g_ball_beam_controller.measurement_valid != 0U) {
        valid_age_ms =
            (uint32_t) tick_ms -
            g_ball_beam_controller.last_valid_receive_ms;
    }

    controller_target_pulses =
        BallBeamController_GetTargetStepperPulses(
            &g_ball_beam_controller);
    stepper_target_pulses =
        StepperMotor_GetTargetPositionPulses();
    stepper_position_pulses =
        StepperMotor_GetPositionPulses();

    (void) snprintf(
        response,
        sizeof(response),
        "BALLLOG,%lu,%lu,%lu,%lu,%lu,%u,%u,%u,"
        "%.1f,%.1f,%.1f,%.1f,%+.1f,%+.1f,%+.1f,%+.1f,%.2f,"
        "%+.2f,%+.2f,%+.2f,%+.2f,%+.2f,%+.2f,%lu,%u,%+.2f,%+.2f,%.2f,"
        "%ld,%ld,%ld,%ld,%+.2f,%+.2f,%u,%u,%+.2f,%+.1f,%+.1f,%.2f,%+.2f,"
        "%+.1f,%+.1f,%+.2f,%+.2f,%+.2f\r\n",
        (unsigned long) tick_ms,
        (unsigned long) g_ball_beam_controller.camera_frame_id,
        (unsigned long) valid_age_ms,
        (unsigned long) g_ball_beam_controller.last_frame_period_ms,
        (unsigned long) g_ball_beam_controller.last_speed_period_ms,
        (unsigned int)
            BallBeamController_IsEnabled(&g_ball_beam_controller),
        (unsigned int) g_ball_beam_controller.stale,
        (unsigned int) g_ball_beam_controller.dropout_active,
        g_ball_beam_controller.raw_x_mm,
        g_ball_beam_controller.filtered_x_mm,
        g_ball_beam_controller.predicted_x_mm,
        g_ball_beam_controller.target_x_mm,
        g_ball_beam_controller.ball_speed_mm_s,
        g_ball_beam_controller.position_error_mm,
        g_ball_beam_controller.control_error_mm,
        g_ball_beam_controller.profile_target_speed_mm_s,
        g_ball_beam_controller.profile_blend,
        g_ball_beam_controller.position_output_deg,
        g_ball_beam_controller.speed_output_deg,
        g_ball_beam_controller.integral_output_deg,
        g_ball_beam_controller.level_trim_deg,
        g_ball_beam_controller.external_feedforward_deg,
        g_ball_beam_controller.breakaway_output_deg,
        (unsigned long)
            g_ball_beam_controller.breakaway_stationary_ms,
        (unsigned int) g_ball_beam_controller.breakaway_active,
        g_ball_beam_controller.profile_output_deg,
        g_ball_beam_controller.target_beam_angle_deg,
        g_ball_beam_controller.dynamic_angle_limit_deg,
        (long) controller_target_pulses,
        (long) stepper_target_pulses,
        (long) stepper_position_pulses,
        (long) (stepper_target_pulses -
                stepper_position_pulses),
        g_q3_manual_beam_angle_deg,
        g_q3_hold_bias_deg,
        (unsigned int) g_q3_settle_capture_active,
        (unsigned int) g_q3_settle_lock_active,
        g_q3_hold_frame_delta_mm,
        g_q3_hold_feedback_speed_mm_s,
        g_q3_hold_profile_target_speed_mm_s,
        g_q3_hold_profile_blend,
        g_q3_hold_stiction_position_deg,
        g_q4_fast_ball_speed_mm_s,
        g_q4_velocity_feedback_mm_s,
        g_q4_position_guard_deg,
        g_q4_velocity_damping_deg,
        g_q4_acceleration_feedforward_deg);
    MapRun_SendBluetoothText(response);
}

static void MapRun_UpdateBallLog(void)
{
    if ((g_ball_log_enabled == 0U) ||
        ((tick_ms - g_last_ball_log_ms) <
         MAP_RUN_BALL_LOG_PERIOD_MS)) {
        return;
    }

    g_last_ball_log_ms = tick_ms;
    MapRun_SendBallLogSample();
}

/*
 * 第五、六问只回传滚球闭环的关键量，避免长 BALLLOG 周期性阻塞巡线。
 * 用户手动开启完整日志后暂停本输出，防止两套数据同时占用调试串口。
 */
static void MapRun_UpdateBallLapTelemetry(void)
{
    char response[160];
    const char *camera_state;
    uint32_t valid_age_ms = 0UL;
    uint8_t post_finish_telemetry_active = 0U;
    float ball_error_mm;

    if ((g_map_state == MAP_RUN_STATE_FINISHED) &&
        (g_finish_reason == MAP_FINISH_ONE_LAP) &&
        ((tick_ms - g_run_start_ms) <=
         (g_run_elapsed_ms +
          MAP_RUN_BALL_POST_FINISH_TELEMETRY_MS))) {
        post_finish_telemetry_active = 1U;
    }

    if ((MapRun_IsBallMode() == 0U) ||
        ((g_map_state != MAP_RUN_STATE_RUNNING) &&
         (post_finish_telemetry_active == 0U)) ||
        (g_ball_log_enabled != 0U) ||
        ((tick_ms - g_last_ball_lap_telemetry_ms) <
         MAP_RUN_BALL_LAP_TELEMETRY_PERIOD_MS)) {
        return;
    }

    g_last_ball_lap_telemetry_ms = tick_ms;
    if (g_ball_beam_controller.has_camera_frame != 0U) {
        valid_age_ms =
            (uint32_t) tick_ms -
            g_ball_beam_controller.last_valid_receive_ms;
    }

    if (g_ball_beam_controller.has_camera_frame == 0U) {
        camera_state = "NONE";
    } else if ((g_ball_lap_camera_fallback_active != 0U) ||
               (g_ball_beam_controller.stale != 0U)) {
        camera_state = "LOST";
    } else if (g_ball_beam_controller.dropout_active != 0U) {
        camera_state = "PRED";
    } else {
        camera_state = "OK";
    }

    ball_error_mm =
        g_ball_beam_controller.filtered_x_mm -
        g_ball_lap_active_target_mm;
    (void) snprintf(
        response,
        sizeof(response),
        "Q%uBALL,%.2f,%+.1f,%+.1f,%+.1f,%+.1f,%+.2f,%.1f,%s,%lu,%lu,%.1f\r\n",
        (unsigned int) MapRun_GetQuestionNumber(),
        (float) (tick_ms - g_run_start_ms) / 1000.0f,
        g_ball_beam_controller.filtered_x_mm,
        g_ball_lap_active_target_mm,
        ball_error_mm,
        g_ball_beam_controller.ball_speed_mm_s,
        g_ball_beam_controller.target_beam_angle_deg,
        g_ball_lap_max_error_mm,
        camera_state,
        (unsigned long) valid_age_ms,
        (unsigned long) g_run_distance_counts,
        g_ball_lap_right_turn_deg);
    MapRun_SendBluetoothText(response);
}

static void MapRun_ProcessBallCameraData(void)
{
    char latest_line[CAMERA_LINK_LINE_BUFFER_SIZE];
    uint32_t frame_count = CameraLink_GetFrameCount();

    if (frame_count == g_camera_control_frame_count) {
        return;
    }

    if (CameraLink_CopyLatestLine(latest_line,
                                  sizeof(latest_line)) != 0U) {
        (void) BallBeamController_ProcessCameraLine(
            &g_ball_beam_controller,
            latest_line,
            CameraLink_GetLastFrameMs());
    }

    g_camera_control_frame_count = frame_count;
}

static void MapRun_EnableAutomaticBallLog(const char *question_text)
{
    char response[96];

    g_ball_log_enabled = 1U;
    g_last_ball_log_ms = tick_ms;
    (void) snprintf(
        response,
        sizeof(response),
        "[BALL LOG AUTO] %s diagnostic CSV enabled at 100 ms\r\n",
        (question_text != NULL) ? question_text : "BALL");
    MapRun_SendBluetoothText(response);
    MapRun_SendBallLogHeader();
    MapRun_SendBallLogSample();
}

static void MapRun_UpdateQ4BallControl(void)
{
    char response[144];
    float ball_error_mm;

    if ((MapRun_IsQ4Mode() == 0U) ||
        (g_map_state != MAP_RUN_STATE_RUNNING)) {
        return;
    }

    /*
     * 比赛优先保证底盘完成 A->B。短时丢球由控制器预测；持续失联时
     * 底盘继续巡线，摆杆只保留可确定的起步/减速前馈，不再使用旧位置
     * 反馈。视觉恢复后控制器会从新帧自动重新接管。
     */
    if ((BallBeamController_IsEnabled(
             &g_ball_beam_controller) == 0U) ||
        (BallBeamController_IsControlReady(
             &g_ball_beam_controller) == 0U)) {
        if (g_q4_camera_fallback_active == 0U) {
            g_q4_camera_fallback_active = 1U;
            (void) snprintf(
                response,
                sizeof(response),
                "[Q4 CAMERA LOST] Frame=%lu Rx=%lu Ovf=%lu; beam FF only\r\n",
                (unsigned long) CameraLink_GetFrameCount(),
                (unsigned long) CameraLink_GetRxByteCount(),
                (unsigned long) CameraLink_GetOverflowCount());
            MapRun_SendBluetoothText(response);
        }
        return;
    }

    if (g_q4_camera_fallback_active != 0U) {
        g_q4_camera_fallback_active = 0U;
        MapRun_SendBluetoothText(
            "[Q4 CAMERA OK] Ball control resumed\r\n");
    }

    ball_error_mm = MapRun_AbsFloat(
        g_ball_beam_controller.raw_x_mm -
        MAP_RUN_Q4_TARGET_POSITION_MM);
    if (ball_error_mm > g_q4_max_ball_error_mm) {
        g_q4_max_ball_error_mm = ball_error_mm;
    }
    if (ball_error_mm > MAP_RUN_Q4_ALLOWED_ERROR_MM) {
        g_q4_ball_limit_exceeded = 1U;
    }
}

static void MapRun_UpdateBallLapControl(void)
{
    char response[144];
    float ball_error_mm;

    if ((MapRun_IsBallMode() == 0U) ||
        (g_map_state != MAP_RUN_STATE_RUNNING)) {
        return;
    }

    /*
     * 短时丢球由控制器预测。超过容忍时间后只让摆杆回标定水平，底盘
     * 继续完成循迹；摄像头恢复有效位置后，控制器自动重新接管。
     */
    if ((BallBeamController_IsEnabled(
             &g_ball_beam_controller) == 0U) ||
        (BallBeamController_IsControlReady(
             &g_ball_beam_controller) == 0U)) {
        if (g_ball_lap_camera_fallback_active == 0U) {
            g_ball_lap_camera_fallback_active = 1U;
            (void) snprintf(
                response,
                sizeof(response),
                "[Q%u CAMERA LOST] Frame=%lu; chassis continues, beam level\r\n",
                (unsigned int) MapRun_GetQuestionNumber(),
                (unsigned long) CameraLink_GetFrameCount());
            MapRun_SendBluetoothText(response);
        }
        return;
    }

    if (g_ball_lap_camera_fallback_active != 0U) {
        g_ball_lap_camera_fallback_active = 0U;
        (void) snprintf(
            response,
            sizeof(response),
            "[Q%u CAMERA OK] Ball control resumed\r\n",
            (unsigned int) MapRun_GetQuestionNumber());
        MapRun_SendBluetoothText(response);
    }

    ball_error_mm = MapRun_AbsFloat(
        g_ball_beam_controller.raw_x_mm -
        g_ball_lap_active_target_mm);
    if (ball_error_mm > g_ball_lap_max_error_mm) {
        g_ball_lap_max_error_mm = ball_error_mm;
    }
    if (ball_error_mm > MAP_RUN_BALL_ALLOWED_ERROR_MM) {
        g_ball_lap_limit_exceeded = 1U;
    }
}

static void MapRun_UpdateBallBeam(void)
{
    MapRun_UpdateMovingBallFeedforward();
    BallBeamController_Update(
        &g_ball_beam_controller, (uint32_t) tick_ms);
    MapRun_UpdateQ4BallControl();
    MapRun_UpdateBallLapControl();
    MapRun_UpdateQ3AdaptiveHold();

    if ((g_ball_stepper_zeroed != 0U) &&
        (StepperMotor_IsPositionTrackingEnabled() != 0U)) {
        if (g_q3_manual_beam_active != 0U) {
            uint8_t camera_ready =
                BallBeamController_IsControlReady(
                    &g_ball_beam_controller);
            float manual_angle_deg = g_q3_manual_beam_angle_deg;

            /*
             * 数据失联时通常回软件水平，避免盲目保持强驱动角。Q3 负向
             * 制动是例外：此时负角正在阻止球继续冲向左端，清零会释放
             * 制动力。短时失联继续保持已经算出的负制动角，等待球回到
             * 视野；其它开环驱动阶段仍回零。
             */
            if (camera_ready == 0U) {
                if (g_q3_state == MAP_Q3_STATE_BRAKE_NEGATIVE) {
                    manual_angle_deg = g_q3_brake_angle_deg;
                } else if (g_q3_state == MAP_Q3_STATE_HOLD_NEGATIVE) {
                    manual_angle_deg = MapRun_ClampFloat(
                        g_q3_hold_bias_deg,
                        -MAP_RUN_Q3_HOLD_STALE_BIAS_DEG,
                        MAP_RUN_Q3_HOLD_STALE_BIAS_DEG);
                } else {
                    manual_angle_deg = 0.0f;
                }
            }
            (void) StepperMotor_SetTargetPositionPulses(
                BallBeamController_ConvertBeamAngleToPulses(
                    &g_ball_beam_controller,
                    manual_angle_deg));
        } else if (BallBeamController_IsEnabled(
                       &g_ball_beam_controller) != 0U) {
            if (((MapRun_IsQ4Mode() != 0U) &&
                 (g_q4_camera_fallback_active != 0U)) ||
                ((MapRun_IsBallMode() != 0U) &&
                 (g_ball_lap_camera_fallback_active != 0U))) {
                /*
                 * 无视觉时不能继续使用旧位置闭环，但底盘加减速方向和
                 * 前馈角仍然确定；保留该角可避免关键加减速段完全失去
                 * 补偿。普通巡航时该值为 0，即回到赛前标定水平。
                 */
                (void) StepperMotor_SetTargetPositionPulses(
                    BallBeamController_ConvertBeamAngleToPulses(
                        &g_ball_beam_controller,
                        g_q4_acceleration_feedforward_deg));
            } else {
                (void) StepperMotor_SetTargetPositionPulses(
                    BallBeamController_GetTargetStepperPulses(
                        &g_ball_beam_controller));
            }
        }
    }

    StepperMotor_Task();
}

static void MapRun_ForwardCameraData(void)
{
    char latest_line[CAMERA_LINK_LINE_BUFFER_SIZE];
    uint32_t frame_count = CameraLink_GetFrameCount();

    if (frame_count == g_camera_forwarded_frame_count) {
        return;
    }

    if (CameraLink_CopyLatestLine(latest_line,
                                  sizeof(latest_line)) != 0U) {
        MapRun_SendBluetoothText("[CAM RX] ");
        MapRun_SendBluetoothText(latest_line);
        MapRun_SendBluetoothText("\r\n");
    }

    g_camera_forwarded_frame_count = frame_count;
}

static void MapRun_HandleBluetoothCommand(const char *command)
{
    char response[128];
    const char *cursor;
    float left_target;
    float right_target;
    float kp;
    float ki;
    float kd;
    float ball_value;
    float ball_kp;
    float ball_ki;
    float ball_kd;

    if (strcmp(command, "STOP") == 0) {
        MapRun_StopMotors(MAP_FINISH_MANUAL);
        MapRun_SendBluetoothText(
            "[STOP OK] Motor output disabled\r\n");
        return;
    }

    if (strcmp(command, "TARGET?") == 0) {
        (void) snprintf(
            response,
            sizeof(response),
            "[TARGET] Left=%.1f Right=%.1f\r\n",
            g_speed_test_left_target_rpm,
            g_speed_test_right_target_rpm);
        MapRun_SendBluetoothText(response);
        return;
    }

    if (strncmp(command, "TARGET,", 7U) == 0) {
        if ((g_calibration_state != SENSOR_CALIBRATION_IDLE) ||
            (MapRun_ParseTargetCommand(command,
                                       &left_target,
                                       &right_target) == 0U)) {
            MapRun_SendBluetoothText(
                "[TARGET ERR] Range is -150..150 RPM\r\n");
            return;
        }

        MapRun_StopMotors(MAP_FINISH_MANUAL);
        g_control_mode = MAP_CONTROL_SPEED_TEST;
        g_speed_test_left_target_rpm = left_target;
        g_speed_test_right_target_rpm = right_target;
        (void) snprintf(
            response,
            sizeof(response),
            "[TARGET OK] Left=%.1f Right=%.1f; send START\r\n",
            left_target,
            right_target);
        MapRun_SendBluetoothText(response);

        if (((MapRun_AbsFloat(left_target) > 0.01f) &&
             (MapRun_AbsFloat(left_target) <= 30.0f)) ||
            ((MapRun_AbsFloat(right_target) > 0.01f) &&
             (MapRun_AbsFloat(right_target) <= 30.0f))) {
            MapRun_SendBluetoothText(
                "[TARGET WARN] <=30 RPM uses open-loop, not PI\r\n");
        }
        return;
    }

    if (strcmp(command, "PID?") == 0) {
        MotorSpeedLoop_GetLeftWheelPI(&kp, &ki);
        (void) snprintf(
            response,
            sizeof(response),
            "[PID] P=%.4f I=%.4f D=0.0000\r\n",
            kp,
            ki);
        MapRun_SendBluetoothText(response);
        return;
    }

    if (strncmp(command, "PID,", 4U) == 0) {
        if ((g_calibration_state != SENSOR_CALIBRATION_IDLE) ||
            (MapRun_ParsePidCommand(command, &kp, &ki, &kd) == 0U)) {
            MapRun_SendBluetoothText(
                "[PID ERR] Use PID,0.1,0.2,0\r\n");
            return;
        }

        MapRun_StopMotors(MAP_FINISH_MANUAL);
        g_control_mode = MAP_CONTROL_SPEED_TEST;
        MotorSpeedLoop_SetLeftWheelPI(kp, ki);
        MotorSpeedLoop_SetRightWheelPI(kp, ki);
        (void) snprintf(
            response,
            sizeof(response),
            "[PID OK] P=%.4f I=%.4f D ignored; send START\r\n",
            kp,
            ki);
        MapRun_SendBluetoothText(response);
        return;
    }

    if (strcmp(command, "START") == 0) {
        if (g_calibration_state != SENSOR_CALIBRATION_IDLE) {
            MapRun_SendBluetoothText(
                "[START ERR] Calibration is active\r\n");
            return;
        }

        if (g_control_mode == MAP_CONTROL_LINE_FOLLOW) {
            MapRun_Start();
            if (g_map_state == MAP_RUN_STATE_RUNNING) {
                if (MapRun_IsQ3Mode() != 0U) {
                    MapRun_SendBluetoothText(
                        "[START OK] Q3 ball sequence enabled\r\n");
                } else {
                    MapRun_SendBluetoothText(
                        "[START OK] Map planner and line control enabled\r\n");
                }
            }
        } else {
            MapRun_StartSpeedTest();
            MapRun_SendBluetoothText(
                "[START OK] Speed-loop test enabled\r\n");
        }
        return;
    }

    if (strcmp(command, "CAR?") == 0) {
        MapRun_SendCarStatus();
        return;
    }

    if (strcmp(command, "LINE?") == 0) {
        MapRun_SendLineStatus();
        return;
    }

    if (strcmp(command, "MAP?") == 0) {
        MapRun_SendMapStatus();
        return;
    }

    if (strcmp(command, "CAM?") == 0) {
        MapRun_SendCameraStatus();
        return;
    }

    if (strcmp(command, "BALL?") == 0) {
        MapRun_SendBallBeamStatus();
        return;
    }

    if (strcmp(command, "Q6?") == 0) {
        (void) snprintf(
            response,
            sizeof(response),
            "[Q6] Target=%+.1f mm Active=%+.1f mm\r\n",
            g_q6_target_position_mm,
            g_ball_lap_active_target_mm);
        MapRun_SendBluetoothText(response);
        return;
    }

    if (strncmp(command, "Q6,TARGET,", 10U) == 0) {
        cursor = command + 10U;
        if ((g_map_state == MAP_RUN_STATE_RUNNING) ||
            (MapRun_ParseFloat(&cursor, &ball_value) == 0U) ||
            (*cursor != '\0') ||
            (MapRun_AbsFloat(ball_value) >
             BALL_BEAM_TARGET_MAX_ABS_MM)) {
            MapRun_SendBluetoothText(
                "[Q6 TARGET ERR] Stop first; range -120..120 mm\r\n");
            return;
        }

        g_q6_target_position_mm = ball_value;
        if (g_question_mode == MAP_QUESTION_MODE_5_Q6) {
            g_ball_lap_active_target_mm =
                g_q6_target_position_mm;
            (void) BallBeamController_SetTargetPositionMm(
                &g_ball_beam_controller,
                g_q6_target_position_mm);
        }
        (void) snprintf(
            response,
            sizeof(response),
            "[Q6 TARGET OK] Set=%+.1f mm; place ball there before B21\r\n",
            g_q6_target_position_mm);
        MapRun_SendBluetoothText(response);
        return;
    }

    if (strcmp(command, "BALL,LOG?") == 0) {
        (void) snprintf(
            response,
            sizeof(response),
            "[BALL LOG] Enabled=%u Period=%lums\r\n",
            (unsigned int) g_ball_log_enabled,
            (unsigned long) MAP_RUN_BALL_LOG_PERIOD_MS);
        MapRun_SendBluetoothText(response);
        return;
    }

    if (strcmp(command, "BALL,TRIM?") == 0) {
        (void) snprintf(
            response,
            sizeof(response),
            "[BALL TRIM] Level=%+.2f deg\r\n",
            g_ball_beam_controller.level_trim_deg);
        MapRun_SendBluetoothText(response);
        return;
    }

    if (strcmp(command, "BALL,LOG,ON") == 0) {
        g_ball_log_enabled = 1U;
        g_last_ball_log_ms = tick_ms;
        MapRun_SendBluetoothText(
            "[BALL LOG OK] Diagnostic CSV enabled at 100 ms\r\n");
        MapRun_SendBallLogHeader();
        MapRun_SendBallLogSample();
        return;
    }

    if (strcmp(command, "BALL,LOG,OFF") == 0) {
        g_ball_log_enabled = 0U;
        MapRun_SendBluetoothText(
            "[BALL LOG OK] Diagnostic CSV disabled\r\n");
        return;
    }

    if (strcmp(command, "BALL,ZERO") == 0) {
        if ((g_map_state == MAP_RUN_STATE_RUNNING) ||
            (g_calibration_state != SENSOR_CALIBRATION_IDLE) ||
            (BallBeamController_IsEnabled(
                 &g_ball_beam_controller) != 0U) ||
            (StepperMotor_SetCurrentPositionZero() == 0U)) {
            MapRun_SendBluetoothText(
                "[BALL ZERO ERR] Stop chassis and stepper first\r\n");
            return;
        }

        g_ball_stepper_zeroed = 1U;
        (void) snprintf(
            response,
            sizeof(response),
            "[BALL ZERO OK] Current pose is software zero; "
            "Trim=%+.2f deg\r\n",
            g_ball_beam_controller.level_trim_deg);
        MapRun_SendBluetoothText(response);
        return;
    }

    if (strcmp(command, "BALL,ON") == 0) {
        if (g_calibration_state != SENSOR_CALIBRATION_IDLE) {
            MapRun_SendBluetoothText(
                "[BALL ON ERR] Calibration is active\r\n");
            return;
        }
        if (g_ball_stepper_zeroed == 0U) {
            MapRun_SendBluetoothText(
                "[BALL ON ERR] Re-zero level beam with BALL,ZERO\r\n");
            return;
        }
        if (BallBeamController_IsControlReady(
                &g_ball_beam_controller) == 0U) {
            MapRun_SendBluetoothText(
                "[BALL ON ERR] No fresh valid camera frame\r\n");
            return;
        }
        if (StepperMotor_EnablePositionTracking(1U) == 0U) {
            MapRun_SendBluetoothText(
                "[BALL ON ERR] Stepper is busy\r\n");
            return;
        }

        BallBeamController_SetEnabled(
            &g_ball_beam_controller, 1U);
        (void) StepperMotor_SetTargetPositionPulses(
            BallBeamController_GetTargetStepperPulses(
                &g_ball_beam_controller));
        MapRun_SendBluetoothText(
            "[BALL ON OK] Camera PD and stepper tracking enabled\r\n");
        return;
    }

    if (strcmp(command, "BALL,OFF") == 0) {
        MapRun_DisableBallBeam();
        MapRun_SendBluetoothText(
            "[BALL OFF OK] Stepper stopped at current position\r\n");
        return;
    }

    if (strncmp(command, "BALL,TARGET,", 12U) == 0) {
        cursor = command + 12U;
        if ((g_map_state == MAP_RUN_STATE_RUNNING) ||
            (MapRun_ParseFloat(&cursor, &ball_value) == 0U) ||
            (*cursor != '\0') ||
            (BallBeamController_SetTargetPositionMm(
                 &g_ball_beam_controller, ball_value) == 0U)) {
            MapRun_SendBluetoothText(
                "[BALL TARGET ERR] Stop first; range -120..120 mm\r\n");
            return;
        }

        if (g_question_mode == MAP_QUESTION_MODE_5_Q6) {
            g_q6_target_position_mm = ball_value;
            g_ball_lap_active_target_mm = ball_value;
        }

        (void) snprintf(
            response,
            sizeof(response),
            "[BALL TARGET OK] Set=%.1f mm\r\n",
            ball_value);
        MapRun_SendBluetoothText(response);
        return;
    }

    if (strncmp(command, "BALL,PD,", 8U) == 0) {
        cursor = command + 8U;
        if ((MapRun_ParseFloat(&cursor, &ball_kp) == 0U) ||
            (*cursor != ',')) {
            MapRun_SendBluetoothText(
                "[BALL PD ERR] Use BALL,PD,0.03,0.02\r\n");
            return;
        }
        cursor++;
        if ((MapRun_ParseFloat(&cursor, &ball_kd) == 0U) ||
            (*cursor != '\0') ||
            (BallBeamController_SetGains(
                 &g_ball_beam_controller,
                 ball_kp,
                 ball_kd) == 0U)) {
            MapRun_SendBluetoothText(
                "[BALL PD ERR] P 0..0.2, D 0..0.05\r\n");
            return;
        }

        (void) snprintf(
            response,
            sizeof(response),
            "[BALL PD OK] P=%.4f I=0.0000 D=%.4f\r\n",
            ball_kp,
            ball_kd);
        MapRun_SendBluetoothText(response);
        return;
    }

    if (strncmp(command, "BALL,PID,", 9U) == 0) {
        cursor = command + 9U;
        if ((MapRun_ParseFloat(&cursor, &ball_kp) == 0U) ||
            (*cursor != ',')) {
            MapRun_SendBluetoothText(
                "[BALL PID ERR] Use BALL,PID,0.0075,0.015,0.03\r\n");
            return;
        }
        cursor++;
        if ((MapRun_ParseFloat(&cursor, &ball_ki) == 0U) ||
            (*cursor != ',')) {
            MapRun_SendBluetoothText(
                "[BALL PID ERR] Use BALL,PID,0.0075,0.015,0.03\r\n");
            return;
        }
        cursor++;
        if ((MapRun_ParseFloat(&cursor, &ball_kd) == 0U) ||
            (*cursor != '\0') ||
            (BallBeamController_SetPIDGains(
                 &g_ball_beam_controller,
                 ball_kp,
                 ball_ki,
                 ball_kd) == 0U)) {
            MapRun_SendBluetoothText(
                "[BALL PID ERR] P/I/D range is 0..0.2/0.05/0.05\r\n");
            return;
        }

        (void) snprintf(
            response,
            sizeof(response),
            "[BALL PID OK] P=%.4f I=%.4f D=%.4f\r\n",
            ball_kp,
            ball_ki,
            ball_kd);
        MapRun_SendBluetoothText(response);
        return;
    }

    if (strncmp(command, "BALL,DELAY,", 11U) == 0) {
        uint32_t prediction_delay_ms;

        cursor = command + 11U;
        if ((MapRun_ParseFloat(&cursor, &ball_value) == 0U) ||
            (*cursor != '\0') ||
            (ball_value < 0.0f) ||
            (ball_value >
             (float) BALL_BEAM_MAX_PREDICTION_DELAY_MS)) {
            MapRun_SendBluetoothText(
                "[BALL DELAY ERR] Range is 0..200 ms\r\n");
            return;
        }

        prediction_delay_ms = (uint32_t) ball_value;
        if ((ball_value != (float) prediction_delay_ms) ||
            (BallBeamController_SetPredictionDelayMs(
                 &g_ball_beam_controller,
                 prediction_delay_ms) == 0U)) {
            MapRun_SendBluetoothText(
                "[BALL DELAY ERR] Use an integer 0..200 ms\r\n");
            return;
        }

        (void) snprintf(
            response,
            sizeof(response),
            "[BALL DELAY OK] Prediction=%lu ms\r\n",
            (unsigned long) prediction_delay_ms);
        MapRun_SendBluetoothText(response);
        return;
    }

    if (strncmp(command, "BALL,TRIM,", 10U) == 0) {
        cursor = command + 10U;
        if ((BallBeamController_IsEnabled(
                 &g_ball_beam_controller) != 0U) ||
            (MapRun_ParseFloat(&cursor, &ball_value) == 0U) ||
            (*cursor != '\0') ||
            (BallBeamController_SetLevelTrimDeg(
                 &g_ball_beam_controller, ball_value) == 0U)) {
            MapRun_SendBluetoothText(
                "[BALL TRIM ERR] Disable first; range -2..2 deg\r\n");
            return;
        }

        (void) snprintf(
            response,
            sizeof(response),
            "[BALL TRIM OK] Level=%+.2f deg\r\n",
            g_ball_beam_controller.level_trim_deg);
        MapRun_SendBluetoothText(response);
        return;
    }

    if (strncmp(command, "BALL,SIGN,", 10U) == 0) {
        cursor = command + 10U;
        if ((BallBeamController_IsEnabled(
                 &g_ball_beam_controller) != 0U) ||
            (MapRun_ParseFloat(&cursor, &ball_value) == 0U) ||
            (*cursor != '\0') ||
            ((ball_value != 1.0f) && (ball_value != -1.0f)) ||
            (BallBeamController_SetOutputSign(
                 &g_ball_beam_controller,
                 (int8_t) ball_value) == 0U)) {
            MapRun_SendBluetoothText(
                "[BALL SIGN ERR] Disable first; use +1 or -1\r\n");
            return;
        }

        (void) snprintf(
            response,
            sizeof(response),
            "[BALL SIGN OK] Sign=%d\r\n",
            (int) g_ball_beam_controller.output_sign);
        MapRun_SendBluetoothText(response);
        return;
    }

    if (strcmp(command, "MODE?") == 0) {
        MapRun_SendQuestionModeStatus();
        return;
    }

    if (strcmp(command, "MAP") == 0) {
        MapRun_StopMotors(MAP_FINISH_MANUAL);
        g_control_mode = MAP_CONTROL_LINE_FOLLOW;
        g_map_state = MAP_RUN_STATE_READY;
        g_finish_reason = MAP_FINISH_NONE;
        MapRun_SendBluetoothText(
            "[MAP OK] Press B21 to run map\r\n");
        MapRun_SendQuestionModeStatus();
        return;
    }

    if (strcmp(command, "HELP") == 0) {
        MapRun_SendBluetoothText(
            "[HELP] STOP | TARGET,n[,n] | TARGET? | "
            "PID,p,i,d | PID? | START | CAR? | LINE? | "
            "MAP? | CAM? | MODE? | MAP\r\n");
        MapRun_SendBluetoothText(
            "[HELP] BALL? | BALL,ZERO | BALL,ON | BALL,OFF | "
            "BALL,TARGET,mm | BALL,PD,p,d | BALL,PID,p,i,d | "
            "BALL,DELAY,ms | BALL,TRIM,deg | BALL,TRIM? | "
            "BALL,SIGN,+1|-1 | "
            "BALL,LOG,ON|OFF|?\r\n");
        MapRun_SendBluetoothText(
            "[HELP] Q6,TARGET,mm | Q6? (debug; B21 auto-captures)\r\n");
        return;
    }

    MapRun_SendBluetoothText(
        "[BT ERR] Unknown command; send HELP\r\n");
}

static void MapRun_UpdateBluetooth(void)
{
    uint8_t data;

    if (g_uart_rx_overflow != 0U) {
        g_uart_rx_overflow = 0U;
        MapRun_SendBluetoothText(
            "[BT ERR] RX buffer overflow\r\n");
    }

    while (MapRun_ReadBluetoothByte(&data) != 0U) {
        if ((data == '\r') || (data == '\n')) {
            if (g_uart_line_length != 0U) {
                g_uart_line_buffer[g_uart_line_length] = '\0';
                MapRun_HandleBluetoothCommand(g_uart_line_buffer);
                g_uart_line_length = 0U;
            }
            continue;
        }

        if ((data >= 'a') && (data <= 'z')) {
            data = (uint8_t) (data - ('a' - 'A'));
        }

        if (g_uart_line_length <
            (MAP_RUN_UART_LINE_BUFFER_SIZE - 1U)) {
            g_uart_line_buffer[g_uart_line_length++] = (char) data;
        } else {
            g_uart_line_length = 0U;
            MapRun_SendBluetoothText(
                "[BT ERR] Command too long\r\n");
        }
    }
}

static void MapRun_UpdateButtonState(MapRunButton *button,
                                     uint8_t pressed)
{
    if (button == NULL) {
        return;
    }

    button->pressed_event = 0U;
    button->short_release_event = 0U;
    button->long_release_event = 0U;

    if (pressed != button->raw_pressed) {
        button->raw_pressed = pressed;
        button->change_ms = tick_ms;
    }

    if ((pressed == button->stable_pressed) ||
        ((tick_ms - button->change_ms) < MAP_RUN_BUTTON_DEBOUNCE_MS)) {
        return;
    }

    button->stable_pressed = pressed;
    if (pressed != 0U) {
        button->press_start_ms = tick_ms;
        button->pressed_event = 1U;
        return;
    }

    if (button->suppress_release != 0U) {
        button->suppress_release = 0U;
        return;
    }

    if ((tick_ms - button->press_start_ms) >=
        MAP_RUN_BUTTON_LONG_PRESS_MS) {
        button->long_release_event = 1U;
    } else {
        button->short_release_event = 1U;
    }
}

static void MapRun_UpdateButtons(void)
{
    uint8_t b21_pressed =
        (DL_GPIO_readPins(GPIO_Button_PIN_Button_PORT,
                          GPIO_Button_PIN_Button_PIN) == 0U) ? 1U : 0U;
    uint8_t sw1_pressed =
        (DL_GPIO_readPins(GPIO_Button_PIN_Button_SW1_PORT,
                          GPIO_Button_PIN_Button_SW1_PIN) == 0U) ? 1U : 0U;
    uint8_t sw2_pressed =
        (DL_GPIO_readPins(GPIO_Button_PIN_Button_SW2_PORT,
                          GPIO_Button_PIN_Button_SW2_PIN) == 0U) ? 1U : 0U;

    MapRun_UpdateButtonState(&g_b21_button, b21_pressed);
    MapRun_UpdateButtonState(&g_sw1_button, sw1_pressed);
    MapRun_UpdateButtonState(&g_sw2_button, sw2_pressed);
}

static void MapRun_InitButtonState(MapRunButton *button,
                                   uint8_t pressed)
{
    if (button == NULL) {
        return;
    }

    button->raw_pressed = pressed;
    button->stable_pressed = pressed;
    button->change_ms = tick_ms;
    button->press_start_ms = tick_ms;
    button->pressed_event = 0U;
    button->short_release_event = 0U;
    button->long_release_event = 0U;
    button->suppress_release = 0U;
}

static void MapRun_SetCalibrationState(SensorCalibrationState state)
{
    g_calibration_state = state;
    g_calibration_state_start_ms = tick_ms;
}

static void MapRun_BeginCalibration(void)
{
    MapRun_StopMotors(MAP_FINISH_MANUAL);
    StepperMotor_RequestStop();
    Sensor_SetCalibrationKeyPressed(0U);
    MapRun_SetCalibrationState(SENSOR_CALIBRATION_WAIT_STEPPER);
}

static void MapRun_HandleCalibrationShortPress(void)
{
    if (g_calibration_state == SENSOR_CALIBRATION_WAIT_BLACK) {
        Sensor_SetCalibrationKeyPressed(1U);
        MapRun_SetCalibrationState(SENSOR_CALIBRATION_BLACK_HOLD);
    } else if (g_calibration_state == SENSOR_CALIBRATION_WAIT_WHITE) {
        Sensor_SetCalibrationKeyPressed(1U);
        MapRun_SetCalibrationState(SENSOR_CALIBRATION_WHITE_HOLD);
    }
}

static void MapRun_UpdateCalibration(void)
{
    switch (g_calibration_state) {
        case SENSOR_CALIBRATION_IDLE:
            break;

        case SENSOR_CALIBRATION_WAIT_STEPPER:
            if (StepperMotor_IsBusy() == 0U) {
                Sensor_SetCalibrationKeyPressed(1U);
                MapRun_SetCalibrationState(
                    SENSOR_CALIBRATION_ENTER_HOLD);
            }
            break;

        case SENSOR_CALIBRATION_ENTER_HOLD:
            if ((tick_ms - g_calibration_state_start_ms) >=
                SENSOR_CALIBRATION_ENTER_HOLD_MS) {
                Sensor_SetCalibrationKeyPressed(0U);
                MapRun_SetCalibrationState(
                    SENSOR_CALIBRATION_ENTER_SETTLE);
            }
            break;

        case SENSOR_CALIBRATION_ENTER_SETTLE:
            if ((tick_ms - g_calibration_state_start_ms) >=
                SENSOR_CALIBRATION_SETTLE_MS) {
                MapRun_SetCalibrationState(
                    SENSOR_CALIBRATION_WAIT_BLACK);
            }
            break;

        case SENSOR_CALIBRATION_WAIT_BLACK:
        case SENSOR_CALIBRATION_WAIT_WHITE:
            break;

        case SENSOR_CALIBRATION_BLACK_HOLD:
            if ((tick_ms - g_calibration_state_start_ms) >=
                SENSOR_CALIBRATION_SHORT_HOLD_MS) {
                Sensor_SetCalibrationKeyPressed(0U);
                MapRun_SetCalibrationState(
                    SENSOR_CALIBRATION_BLACK_SETTLE);
            }
            break;

        case SENSOR_CALIBRATION_BLACK_SETTLE:
            if ((tick_ms - g_calibration_state_start_ms) >=
                SENSOR_CALIBRATION_SETTLE_MS) {
                MapRun_SetCalibrationState(
                    SENSOR_CALIBRATION_WAIT_WHITE);
            }
            break;

        case SENSOR_CALIBRATION_WHITE_HOLD:
            if ((tick_ms - g_calibration_state_start_ms) >=
                SENSOR_CALIBRATION_SHORT_HOLD_MS) {
                Sensor_SetCalibrationKeyPressed(0U);
                MapRun_SetCalibrationState(SENSOR_CALIBRATION_DONE);
            }
            break;

        case SENSOR_CALIBRATION_DONE:
            if ((tick_ms - g_calibration_state_start_ms) >=
                SENSOR_CALIBRATION_DONE_HOLD_MS) {
                g_map_state = MAP_RUN_STATE_READY;
                g_finish_reason = MAP_FINISH_NONE;
                MapRun_SetCalibrationState(SENSOR_CALIBRATION_IDLE);
            }
            break;

        default:
            Sensor_SetCalibrationKeyPressed(0U);
            MapRun_SetCalibrationState(SENSOR_CALIBRATION_IDLE);
            break;
    }
}

static void MapRun_UpdateOneLapFinishDetection(void)
{
    if (g_ball_lap_final_stop_active != 0U) {
        return;
    }

    /*
     * Q5/Q6 直接使用实测写死终点：平均编码器达到缩短后的距离，并且
     * BNO085 处理过正负 180 度跨界后的累计转动量达到 360 度。
     * 正常时两个条件同时满足才缓停；IMU 异常时由最大距离强制兜底，
     * 避免停车条件永久无法成立而持续绕圈。
     */
    if (MapRun_IsBallMode() != 0U) {
        uint8_t imu_fresh;
        uint8_t imu_unavailable;
        uint8_t imu_stop_ready;

        imu_fresh =
            MapYawRateController_IsFresh(
                &g_yaw_rate_controller,
                (uint32_t) tick_ms);
        imu_stop_ready =
            ((g_ball_lap_yaw_valid != 0U) &&
             (g_ball_lap_right_turn_deg >=
              MAP_RUN_BALL_FIXED_STOP_TURN_DEG)) ? 1U : 0U;
        /*
         * 若 BNO085 未产生新样本、数据已经超时，或跑到终点仍几乎没有
         * 累计转角，则把 IMU 视为不可用，避免它阻塞实测编码器终点。
         */
        imu_unavailable =
            ((g_ball_lap_yaw_valid == 0U) ||
             (imu_fresh == 0U) ||
             (g_ball_lap_right_turn_deg <
              MAP_RUN_BALL_IMU_MIN_PLAUSIBLE_TURN_DEG)) ? 1U : 0U;

        if (((g_run_distance_counts >=
              MAP_RUN_BALL_FIXED_STOP_COUNTS) &&
             ((imu_stop_ready != 0U) ||
              (imu_unavailable != 0U))) ||
            (g_run_distance_counts >=
             MAP_RUN_BALL_FIXED_STOP_MAX_COUNTS)) {
            MapRun_BeginBallLapFinalStop();
        }
        return;
    }

    if (g_start_line_left == 0U) {
        if ((g_run_distance_counts >= MAP_RUN_START_LINE_LEAVE_COUNTS) &&
            (g_sensor_active_count <= MAP_RUN_NORMAL_LINE_ACTIVE_MAX)) {
            if (g_start_line_leave_count <
                MAP_RUN_START_LINE_LEAVE_CONFIRM) {
                g_start_line_leave_count++;
            }
        } else {
            g_start_line_leave_count = 0U;
        }

        if (g_start_line_leave_count >=
            MAP_RUN_START_LINE_LEAVE_CONFIRM) {
            g_start_line_left = 1U;
        }
    }

    if ((g_start_line_left != 0U) &&
        (g_run_distance_counts >= MAP_RUN_LAP_ARM_COUNTS)) {
        g_lap_finish_armed = 1U;
    }

    if ((g_lap_finish_armed != 0U) &&
        (g_sensor_active_count >= MAP_RUN_FINISH_LINE_ACTIVE_MIN)) {
        if (g_finish_line_count < MAP_RUN_FINISH_LINE_CONFIRM) {
            g_finish_line_count++;
        }
    } else {
        g_finish_line_count = 0U;
    }

    if (g_finish_line_count >= MAP_RUN_FINISH_LINE_CONFIRM) {
        MapRun_StopMotors(MAP_FINISH_ONE_LAP);
    }
}

static void MapRun_BeginQ4FinalStop(void)
{
    char response[112];

    if ((g_q4_final_stop_active != 0U) ||
        (g_map_state != MAP_RUN_STATE_RUNNING)) {
        return;
    }

    g_q4_final_stop_start_ms = tick_ms;
    g_q4_final_stop_start_rpm = MapRun_GetQ4BaseSpeedRpm();
    g_q4_final_stop_left_duty_percent = MapRun_AbsFloat(
        MotorSpeedLoop_GetLeftWheelDutyPercent());
    g_q4_final_stop_right_duty_percent = MapRun_AbsFloat(
        MotorSpeedLoop_GetRightWheelDutyPercent());
    g_q4_final_stop_active = 1U;
    g_q4_park_hold_active = 0U;
    g_q4_stop_feedforward_active = 0U;
    g_center_speed_rpm = g_q4_final_stop_start_rpm;
    g_base_speed_rpm = g_center_speed_rpm;
    g_correction_rpm = 0.0f;
    g_yaw_cascade_active = 0U;
    g_finish_heading_active = 0U;

    /*
     * 速度环低于 30 RPM 会切到带死区的开环映射，不能形成真实的
     * 35 -> 0 缓降。因此捕获当前实际占空比后关闭 PI，由停车阶段
     * 直接按同一 S 曲线衰减左右轮 PWM。
     */
    MotorSpeedLoop_EnableLeftWheel(0U);
    MotorSpeedLoop_EnableRightWheel(0U);
    MotorSpeedLoop_SetLeftWheelTargetRPM(g_q4_final_stop_start_rpm);
    MotorSpeedLoop_SetRightWheelTargetRPM(g_q4_final_stop_start_rpm);
    MotorPWM_SetLeftWheelSignedDutyPercent(
        g_q4_final_stop_left_duty_percent);
    MotorPWM_SetRightWheelSignedDutyPercent(
        g_q4_final_stop_right_duty_percent);
    MapRun_ResetEncoderFeedbackWatchdog();

    g_q4_feedforward_update_ms =
        tick_ms - MAP_RUN_CONTROL_PERIOD_MS;
    MapRun_UpdateMovingBallFeedforward();

    (void) snprintf(
        response,
        sizeof(response),
        "[Q4 STOP] PWM %.1f,%.1f -> 0 in %lu ms\r\n",
        g_q4_final_stop_left_duty_percent,
        g_q4_final_stop_right_duty_percent,
        (unsigned long) MAP_RUN_Q4_FINAL_STOP_RAMP_MS);
    MapRun_SendBluetoothText(response);
}

static void MapRun_UpdateQ4FinalStop(void)
{
    char response[128];
    unsigned long elapsed_ms;
    float progress;
    float smooth_progress;
    float remaining_ratio;

    if (g_q4_final_stop_active == 0U) {
        return;
    }

    elapsed_ms = tick_ms - g_q4_final_stop_start_ms;
    if (elapsed_ms >= MAP_RUN_Q4_FINAL_STOP_RAMP_MS) {
        MotorPWM_SetLeftWheelSignedDutyPercent(0.0f);
        MotorPWM_SetRightWheelSignedDutyPercent(0.0f);
        g_q4_final_stop_active = 0U;
        g_center_speed_rpm = 0.0f;
        g_base_speed_rpm = 0.0f;
        MapRun_StopMotors(MAP_FINISH_Q4_B_PASSED);

        (void) snprintf(
            response,
            sizeof(response),
            "[Q4 DONE] Time=%.2fs MaxBallErr=%.1f mm %s; hold=15/0.5\r\n",
            (float) g_run_elapsed_ms / 1000.0f,
            g_q4_max_ball_error_mm,
            (g_q4_ball_limit_exceeded == 0U) ? "PASS" : "OVER");
        MapRun_SendBluetoothText(response);
        return;
    }

    progress =
        (float) elapsed_ms /
        (float) MAP_RUN_Q4_FINAL_STOP_RAMP_MS;
    smooth_progress =
        progress * progress * (3.0f - (2.0f * progress));
    remaining_ratio = 1.0f - smooth_progress;

    g_center_speed_rpm =
        g_q4_final_stop_start_rpm * remaining_ratio;
    g_base_speed_rpm = g_center_speed_rpm;
    MotorSpeedLoop_SetLeftWheelTargetRPM(g_center_speed_rpm);
    MotorSpeedLoop_SetRightWheelTargetRPM(g_center_speed_rpm);
    MotorPWM_SetLeftWheelSignedDutyPercent(
        g_q4_final_stop_left_duty_percent * remaining_ratio);
    MotorPWM_SetRightWheelSignedDutyPercent(
        g_q4_final_stop_right_duty_percent * remaining_ratio);
}

static void MapRun_BeginBallLapFinalStop(void)
{
    char response[144];
    float left_target_rpm;
    float right_target_rpm;

    if ((g_ball_lap_final_stop_active != 0U) ||
        (g_map_state != MAP_RUN_STATE_RUNNING) ||
        (MapRun_IsBallMode() == 0U)) {
        return;
    }

    left_target_rpm = MotorSpeedLoop_GetLeftWheelTargetRPM();
    right_target_rpm = MotorSpeedLoop_GetRightWheelTargetRPM();
    g_ball_lap_final_stop_start_ms = tick_ms;
    g_ball_lap_final_stop_start_rpm =
        (MapRun_AbsFloat(left_target_rpm) +
         MapRun_AbsFloat(right_target_rpm)) * 0.5f;
    g_ball_lap_final_stop_left_duty_percent = MapRun_AbsFloat(
        MotorSpeedLoop_GetLeftWheelDutyPercent());
    g_ball_lap_final_stop_right_duty_percent = MapRun_AbsFloat(
        MotorSpeedLoop_GetRightWheelDutyPercent());
    g_ball_lap_final_stop_active = 1U;
    g_center_speed_rpm = g_ball_lap_final_stop_start_rpm;
    g_base_speed_rpm = g_center_speed_rpm;
    g_correction_rpm = 0.0f;
    g_yaw_cascade_active = 0U;
    g_finish_heading_active = 0U;

    /*
     * 终点时左右轮可能仍有巡线差速。捕获各自实际 PWM 后关闭速度环，
     * 再按同一 S 曲线同比例降到 0，既保留当前转向关系，也避免硬停。
     */
    MotorSpeedLoop_EnableLeftWheel(0U);
    MotorSpeedLoop_EnableRightWheel(0U);
    MotorSpeedLoop_SetLeftWheelTargetRPM(left_target_rpm);
    MotorSpeedLoop_SetRightWheelTargetRPM(right_target_rpm);
    MotorPWM_SetLeftWheelSignedDutyPercent(
        g_ball_lap_final_stop_left_duty_percent);
    MotorPWM_SetRightWheelSignedDutyPercent(
        g_ball_lap_final_stop_right_duty_percent);
    MapRun_ResetEncoderFeedbackWatchdog();

    g_q4_feedforward_update_ms =
        tick_ms - MAP_RUN_CONTROL_PERIOD_MS;
    MapRun_UpdateMovingBallFeedforward();

    (void) snprintf(
        response,
        sizeof(response),
        "[Q%u STOP] D=%lu Turn=%.1f PWM %.1f,%.1f -> 0 in %lu ms\r\n",
        (unsigned int) MapRun_GetQuestionNumber(),
        (unsigned long) g_run_distance_counts,
        g_ball_lap_right_turn_deg,
        g_ball_lap_final_stop_left_duty_percent,
        g_ball_lap_final_stop_right_duty_percent,
        (unsigned long) MAP_RUN_BALL_FINAL_STOP_RAMP_MS);
    MapRun_SendBluetoothText(response);
}

static void MapRun_UpdateBallLapFinalStop(void)
{
    unsigned long elapsed_ms;
    float progress;
    float smooth_progress;
    float remaining_ratio;

    if (g_ball_lap_final_stop_active == 0U) {
        return;
    }

    elapsed_ms = tick_ms - g_ball_lap_final_stop_start_ms;
    if (elapsed_ms >= MAP_RUN_BALL_FINAL_STOP_RAMP_MS) {
        MotorPWM_SetLeftWheelSignedDutyPercent(0.0f);
        MotorPWM_SetRightWheelSignedDutyPercent(0.0f);
        g_ball_lap_final_stop_active = 0U;
        g_center_speed_rpm = 0.0f;
        g_base_speed_rpm = 0.0f;
        MapRun_StopMotors(MAP_FINISH_ONE_LAP);
        return;
    }

    progress =
        (float) elapsed_ms /
        (float) MAP_RUN_BALL_FINAL_STOP_RAMP_MS;
    smooth_progress =
        progress * progress * (3.0f - (2.0f * progress));
    remaining_ratio = 1.0f - smooth_progress;

    g_center_speed_rpm =
        g_ball_lap_final_stop_start_rpm * remaining_ratio;
    g_base_speed_rpm = g_center_speed_rpm;
    MotorSpeedLoop_SetLeftWheelTargetRPM(g_center_speed_rpm);
    MotorSpeedLoop_SetRightWheelTargetRPM(g_center_speed_rpm);
    MotorPWM_SetLeftWheelSignedDutyPercent(
        g_ball_lap_final_stop_left_duty_percent * remaining_ratio);
    MotorPWM_SetRightWheelSignedDutyPercent(
        g_ball_lap_final_stop_right_duty_percent * remaining_ratio);
}

static void MapRun_UpdateQ4FinishDetection(void)
{
    if (g_q4_final_stop_active != 0U) {
        return;
    }

    /*
     * bit7 是最右探头，bit6 是右边第二个探头。4500 count 前忽略它，
     * 防止直线阶段偶发摆动或噪声提前锁存 B 点。
     */
    if ((g_run_distance_counts >= MAP_RUN_Q4_SENSOR_ARM_COUNTS) &&
        ((g_sensor_value & MAP_RUN_Q4_RIGHT_SECOND_MASK) != 0U)) {
        g_q4_right_second_seen = 1U;
    }

    /*
     * 正常条件：已经看到右二黑，并且车轴越过 B 点约 9.8 cm。
     * 兜底条件：传感器没有触发，但编码器已达到约 1.67 m。
     */
    if (((g_q4_right_second_seen != 0U) &&
         (g_run_distance_counts >= MAP_RUN_Q4_TARGET_STOP_COUNTS)) ||
        (g_run_distance_counts >= MAP_RUN_Q4_MAX_STOP_COUNTS)) {
        MapRun_BeginQ4FinalStop();
    }
}

static void MapRun_UpdateFinishDetection(void)
{
    if (g_question_mode == MAP_QUESTION_MODE_3_Q4) {
        MapRun_UpdateQ4FinishDetection();
    } else {
        MapRun_UpdateOneLapFinishDetection();
    }
}

static float MapRun_ApplyLineTargetMinimum(float target_rpm)
{
    /*
     * 第四至第六问都带球，边缘探头触发后不能把不足 30 RPM 的内轮
     * 直接截成 0，否则停车前可能形成单轮制动。25 RPM 是当前底盘
     * 实测可连续转动的最低目标；真正的 0 仍保留给起步等待和停机流程。
     */
    if ((MapRun_IsQ4Mode() != 0U) ||
        (MapRun_IsBallMode() != 0U)) {
        if (target_rpm <= 0.01f) {
            return 0.0f;
        }
        return (target_rpm < MAP_RUN_BALL_MIN_TARGET_RPM) ?
            MAP_RUN_BALL_MIN_TARGET_RPM : target_rpm;
    }

    if (target_rpm <= MAP_RUN_LOW_TARGET_STOP_RPM) {
        return 0.0f;
    }
    return target_rpm;
}

static float MapRun_ApplyBallTargetMaximum(float target_rpm)
{
    if ((MapRun_IsBallMode() != 0U) &&
        (target_rpm > MAP_RUN_BALL_MAX_OUTER_TARGET_RPM)) {
        return MAP_RUN_BALL_MAX_OUTER_TARGET_RPM;
    }

    return target_rpm;
}

static float MapRun_LimitBallTurnRpm(float turn_rpm,
                                     float base_speed_rpm)
{
    float turn_limit_rpm;

    if (MapRun_IsBallMode() == 0U) {
        return turn_rpm;
    }

    /*
     * 内轮仍按 base - 2 * |turn| 计算。
     * 将转向量限制为 (base - 25) / 2，可保证内轮不低于 25 RPM；
     * 外轮的额外提速由 MapRun_GetBallOuterBoostRpm() 单独计算。
     */
    turn_limit_rpm =
        (base_speed_rpm - MAP_RUN_BALL_MIN_TARGET_RPM) * 0.5f;
    if (turn_limit_rpm < 0.0f) {
        turn_limit_rpm = 0.0f;
    }

    if (turn_rpm > turn_limit_rpm) {
        return turn_limit_rpm;
    }
    if (turn_rpm < -turn_limit_rpm) {
        return -turn_limit_rpm;
    }
    return turn_rpm;
}

static float MapRun_GetBallOuterBoostRpm(float turn_rpm,
                                         float base_speed_rpm)
{
    float turn_limit_rpm;
    float boost_limit_rpm;
    float turn_ratio;

    if (MapRun_IsBallMode() == 0U) {
        return 0.0f;
    }

    turn_limit_rpm =
        (base_speed_rpm - MAP_RUN_BALL_MIN_TARGET_RPM) * 0.5f;
    if (turn_limit_rpm <= 0.0f) {
        return 0.0f;
    }

    /*
     * 稳态基础速度为 85 RPM，满纠偏时外轮最多增加 10 RPM。
     * 起步阶段仍只增加同样的 10 RPM，避免 30 RPM 起步时突然冲到 95。
     */
    boost_limit_rpm =
        MAP_RUN_BALL_MAX_OUTER_TARGET_RPM -
        MAP_RUN_BALL_BASE_SPEED_RPM;
    if ((base_speed_rpm + boost_limit_rpm) >
        MAP_RUN_BALL_MAX_OUTER_TARGET_RPM) {
        boost_limit_rpm =
            MAP_RUN_BALL_MAX_OUTER_TARGET_RPM - base_speed_rpm;
    }
    if (boost_limit_rpm <= 0.0f) {
        return 0.0f;
    }

    turn_ratio = MapRun_AbsFloat(turn_rpm) / turn_limit_rpm;
    if (turn_ratio > 1.0f) {
        turn_ratio = 1.0f;
    }

    return boost_limit_rpm * turn_ratio;
}

static float MapRun_LimitBallTargetRise(float target_rpm,
                                        float current_target_rpm)
{
    if ((MapRun_IsBallMode() != 0U) &&
        (target_rpm >
         (current_target_rpm +
          MAP_RUN_BALL_TARGET_RISE_RPM_PER_STEP))) {
        return current_target_rpm +
               MAP_RUN_BALL_TARGET_RISE_RPM_PER_STEP;
    }

    return target_rpm;
}

static void MapRun_SetLineWheelTargets(float left_target_rpm,
                                       float right_target_rpm)
{
    left_target_rpm =
        MapRun_LimitBallTargetRise(
            left_target_rpm,
            MotorSpeedLoop_GetLeftWheelTargetRPM());
    right_target_rpm =
        MapRun_LimitBallTargetRise(
            right_target_rpm,
            MotorSpeedLoop_GetRightWheelTargetRPM());

    MotorSpeedLoop_SetLeftWheelTargetRPM(left_target_rpm);
    MotorSpeedLoop_SetRightWheelTargetRPM(right_target_rpm);
}

static void MapRun_UpdateDirectLineTargets(void)
{
    float base_speed_rpm = MapRun_GetQuestionBaseSpeedRpm();
    float left_target_rpm;
    float right_target_rpm;
    float turn_rpm;
    float outer_boost_rpm;

    if (MapRun_IsBallMode() != 0U) {
        /*
         * BNO085 暂不可用时仍采用与串级控制一致的轮速分配：
         * 满纠偏时内轮降至 25 RPM，外轮随强度升至最高 95 RPM。
         */
        turn_rpm =
            MapRun_LimitBallTurnRpm(-g_correction_rpm,
                                    base_speed_rpm);
        outer_boost_rpm =
            MapRun_GetBallOuterBoostRpm(turn_rpm,
                                        base_speed_rpm);

        if (turn_rpm > 0.0f) {
            left_target_rpm =
                base_speed_rpm - (2.0f * turn_rpm);
            right_target_rpm =
                base_speed_rpm + outer_boost_rpm;
        } else if (turn_rpm < 0.0f) {
            left_target_rpm =
                base_speed_rpm + outer_boost_rpm;
            right_target_rpm =
                base_speed_rpm + (2.0f * turn_rpm);
        } else {
            left_target_rpm = base_speed_rpm;
            right_target_rpm = base_speed_rpm;
        }

        left_target_rpm =
            MapRun_ApplyLineTargetMinimum(left_target_rpm);
        right_target_rpm =
            MapRun_ApplyLineTargetMinimum(right_target_rpm);
        left_target_rpm =
            MapRun_ApplyBallTargetMaximum(left_target_rpm);
        right_target_rpm =
            MapRun_ApplyBallTargetMaximum(right_target_rpm);

        g_center_speed_rpm =
            (left_target_rpm + right_target_rpm) * 0.5f;
        g_base_speed_rpm = g_center_speed_rpm;
        MapRun_SetLineWheelTargets(left_target_rpm,
                                   right_target_rpm);
        return;
    }

    /*
     * BNO085 尚未形成有效角速度或数据超时时，保持原灰度 P 行为：
     * 只降低弯道内轮，不提高外轮，确保传感器异常不会让车辆失控。
     */
    if (g_correction_rpm > 0.0f) {
        left_target_rpm = base_speed_rpm;
        right_target_rpm =
            base_speed_rpm - (2.0f * g_correction_rpm);
        right_target_rpm =
            MapRun_ApplyLineTargetMinimum(right_target_rpm);
    } else if (g_correction_rpm < 0.0f) {
        left_target_rpm =
            base_speed_rpm + (2.0f * g_correction_rpm);
        right_target_rpm = base_speed_rpm;
        left_target_rpm =
            MapRun_ApplyLineTargetMinimum(left_target_rpm);
    } else {
        left_target_rpm = base_speed_rpm;
        right_target_rpm = base_speed_rpm;
    }

    g_center_speed_rpm =
        (left_target_rpm + right_target_rpm) * 0.5f;
    g_base_speed_rpm = g_center_speed_rpm;
    MapRun_SetLineWheelTargets(left_target_rpm,
                               right_target_rpm);
}

static void MapRun_UpdateYawCascadeTargets(void)
{
    float base_speed_rpm = MapRun_GetQuestionBaseSpeedRpm();
    float recovery_rpm_per_step =
        MapRun_GetCenterRecoveryRpmPerStep();
    float desired_center_rpm;
    float left_target_rpm;
    float right_target_rpm;
    float outer_boost_rpm;

    g_yaw_turn_rpm =
        MapYawRateController_GetTurnRpm(&g_yaw_rate_controller);
    g_yaw_turn_rpm =
        MapRun_LimitBallTurnRpm(g_yaw_turn_rpm,
                                base_speed_rpm);
    outer_boost_rpm =
        MapRun_GetBallOuterBoostRpm(g_yaw_turn_rpm,
                                    base_speed_rpm);

    /*
     * 第二问速度较高，外轮不能跟随中心速度恢复过程一起降速。
     * 外轮始终保持基础速度，只用降低内轮形成差速；这样强纠偏结束后
     * 外轮也不会从 120 RPM 突然掉到七八十，避免弯道后半段转不过去。
     */
    if (g_question_mode == MAP_QUESTION_MODE_1_Q2) {
        if (g_yaw_turn_rpm > 0.0f) {
            left_target_rpm =
                base_speed_rpm - (2.0f * g_yaw_turn_rpm);
            right_target_rpm = base_speed_rpm;
        } else if (g_yaw_turn_rpm < 0.0f) {
            left_target_rpm = base_speed_rpm;
            right_target_rpm =
                base_speed_rpm + (2.0f * g_yaw_turn_rpm);
        } else {
            left_target_rpm = base_speed_rpm;
            right_target_rpm = base_speed_rpm;
        }

        left_target_rpm =
            MapRun_ApplyLineTargetMinimum(left_target_rpm);
        right_target_rpm =
            MapRun_ApplyLineTargetMinimum(right_target_rpm);
        g_center_speed_rpm =
            (left_target_rpm + right_target_rpm) * 0.5f;
        g_base_speed_rpm = g_center_speed_rpm;
        MapRun_SetLineWheelTargets(left_target_rpm,
                                   right_target_rpm);
        return;
    }

    /*
     * 差速增大时立即降低中心速度，再按转向强度给外轮叠加最多 15 RPM；
     * 普通模式每 10 ms 恢复 2 RPM，带球模式降为 1 RPM，减小小球冲击。
     * M4/M5 还限制单轮目标的上升速度，减速保持立即生效。
     */
    desired_center_rpm =
        base_speed_rpm - MapRun_AbsFloat(g_yaw_turn_rpm);
    if (desired_center_rpm < g_center_speed_rpm) {
        g_center_speed_rpm = desired_center_rpm;
    } else if ((g_center_speed_rpm +
                recovery_rpm_per_step) <
               desired_center_rpm) {
        g_center_speed_rpm += recovery_rpm_per_step;
    } else {
        g_center_speed_rpm = desired_center_rpm;
    }

    left_target_rpm =
        g_center_speed_rpm - g_yaw_turn_rpm;
    right_target_rpm =
        g_center_speed_rpm + g_yaw_turn_rpm;
    if (g_yaw_turn_rpm > 0.0f) {
        right_target_rpm += outer_boost_rpm;
    } else if (g_yaw_turn_rpm < 0.0f) {
        left_target_rpm += outer_boost_rpm;
    }
    left_target_rpm =
        MapRun_ApplyLineTargetMinimum(left_target_rpm);
    right_target_rpm =
        MapRun_ApplyLineTargetMinimum(right_target_rpm);
    left_target_rpm =
        MapRun_ApplyBallTargetMaximum(left_target_rpm);
    right_target_rpm =
        MapRun_ApplyBallTargetMaximum(right_target_rpm);

    g_base_speed_rpm = g_center_speed_rpm;
    MapRun_SetLineWheelTargets(left_target_rpm,
                               right_target_rpm);
}

static float MapRun_ExpandBallOuterError(float line_error)
{
    float abs_error = MapRun_AbsFloat(line_error);
    float expanded_error;

    if ((MapRun_IsBallMode() == 0U) ||
        (abs_error <= MAP_RUN_BALL_OUTER_ERROR_START)) {
        return line_error;
    }

    /*
     * 中心误差保持原值；超过 2 后只放大超出的部分，函数在分界点连续。
     * 这样弯道刚开始偏离时更早建立角速度，不必等到最外侧才满纠偏。
     */
    expanded_error =
        MAP_RUN_BALL_OUTER_ERROR_START +
        ((abs_error - MAP_RUN_BALL_OUTER_ERROR_START) *
         MAP_RUN_BALL_OUTER_ERROR_GAIN);
    return (line_error < 0.0f) ?
        -expanded_error : expanded_error;
}

static void MapRun_UpdateLineTargets(void)
{
    uint8_t yaw_rate_fresh;
    float control_line_error;

    MapLineController_Update(&g_line_controller,
                             g_sensor_value,
                             &g_line_result);
    g_sensor_active_count = g_line_result.active_count;
    g_line_error = g_line_result.error;
    g_correction_rpm = g_line_result.correction_rpm;

    /* 预置摆杆的 200 ms 内底盘必须保持静止，不能被起点横线纠偏带动。 */
    if ((MapRun_IsBallMode() != 0U) &&
        ((tick_ms - g_run_start_ms) < MAP_RUN_BALL_START_HOLD_MS)) {
        g_center_speed_rpm = 0.0f;
        g_base_speed_rpm = 0.0f;
        g_yaw_cascade_active = 0U;
        g_target_yaw_rate_deg_s = 0.0f;
        g_yaw_turn_rpm = 0.0f;
        MotorSpeedLoop_SetLeftWheelTargetRPM(0.0f);
        MotorSpeedLoop_SetRightWheelTargetRPM(0.0f);
        return;
    }

    yaw_rate_fresh =
        MapYawRateController_IsFresh(
            &g_yaw_rate_controller,
            (uint32_t) tick_ms);

    /*
     * M3 的直线参数已经实车验证较稳，继续使用中心误差减半。M4/M5
     * 的速度更高且负载更大，中心两探头在 -2/+2 间跳变时改为四分之一，
     * 减少直线反复改变目标角速度；外侧误差仍保持原强度。
     * 带球模式按当前基础速度相对 80 RPM 的比例缩放误差；超过中心区
     * 后再放大偏差，提前建立转向，避免等到边缘探头才开始强纠偏。
     */
    if ((g_question_mode == MAP_QUESTION_MODE_3_Q4) ||
        (MapRun_IsBallMode() != 0U)) {
        if (MapRun_AbsFloat(g_line_error) <=
            MAP_RUN_STABLE_CENTER_ERROR_LIMIT) {
            if (MapRun_IsBallMode() != 0U) {
                g_line_error *= MAP_RUN_BALL_CENTER_ERROR_SCALE;
            } else {
                g_line_error *= MAP_RUN_STABLE_CENTER_ERROR_SCALE;
            }
        }
        if (MapRun_IsBallMode() != 0U) {
            g_line_error *=
                MapRun_GetQuestionBaseSpeedRpm() /
                MAP_RUN_BALL_LINE_REFERENCE_RPM;
            g_line_error =
                MapRun_ExpandBallOuterError(g_line_error);

            /*
             * 右侧外探头进入已知半圆后，按 BNO085 累计转角保持最低
             * 右转角速度；达到出弯角度并回到中线后再主动消除残余转动。
             */
            MapCurveHold_Update(
                &g_curve_hold,
                g_run_distance_counts,
                g_sensor_value,
                yaw,
                yaw_rate_fresh,
                MapYawRateController_GetMeasuredYawRate(
                    &g_yaw_rate_controller));
            g_line_error =
                MapCurveHold_ApplyLineError(
                    &g_curve_hold,
                    g_line_error);
        }
        g_correction_rpm =
            MapLineController_CalculateCorrectionRpm(
                g_line_error);
    }

    MapRun_UpdateFinishHeadingAssist(yaw_rate_fresh);
    control_line_error = g_line_error;

    /*
     * M4/M5 全白时保留最后有效灰度误差，但继续运行 BNO085 角速度环，
     * 避免弯道短暂丢线后冻结旧轮速。M1 最后一段允许仅靠起跑航向
     * 继续对正；其它普通模式保持原来的严格目标保持。
     */
    if ((g_line_result.line_found == 0U) &&
        (MapRun_IsBallMode() == 0U)) {
        if (g_finish_heading_active == 0U) {
            return;
        }
        control_line_error = 0.0f;
    }

    g_yaw_cascade_active =
        MapYawRateController_Update(&g_yaw_rate_controller,
                                    control_line_error,
                                    g_finish_heading_rate_offset_deg_s,
                                    (uint32_t) tick_ms,
                                    (g_question_mode ==
                                     MAP_QUESTION_MODE_1_Q2) ?
                                        0U : 1U);
    g_target_yaw_rate_deg_s =
        MapYawRateController_GetTargetYawRate(
            &g_yaw_rate_controller);
    g_measured_yaw_rate_deg_s =
        MapYawRateController_GetMeasuredYawRate(
            &g_yaw_rate_controller);
    g_yaw_turn_rpm =
        MapYawRateController_GetTurnRpm(
            &g_yaw_rate_controller);

    if (g_yaw_cascade_active != 0U) {
        MapRun_UpdateYawCascadeTargets();
    } else {
        MapRun_UpdateDirectLineTargets();
    }
}

static void MapRun_UpdateControl(void)
{
    if ((tick_ms - g_last_control_ms) < MAP_RUN_CONTROL_PERIOD_MS) {
        return;
    }

    g_last_control_ms = tick_ms;

    /*
     * 直接使用当前原始位图。逐位三帧多数表决会把连续经过相邻探头的
     * 真实黑线误判成全白；第五、六问沿用单帧误差，避免额外相位滞后。
     */
    g_sensor_value = Sensor_Read_Grayscale();
    g_sensor_active_count =
        MapLineController_CountActive(g_sensor_value);

    if (g_map_state != MAP_RUN_STATE_RUNNING) {
        MapLineController_Update(&g_line_controller,
                                 g_sensor_value,
                                 &g_line_result);
        g_sensor_active_count = g_line_result.active_count;
        g_line_error = g_line_result.error;
        g_correction_rpm = g_line_result.correction_rpm;
        return;
    }

    g_run_elapsed_ms = tick_ms - g_run_start_ms;

    if (MapRun_IsQ3Mode() != 0U) {
        MapRun_UpdateQ3Sequence();
        return;
    }

    g_run_distance_counts = MapRun_GetDistanceCounts();
    MapRun_UpdateEncoderFeedbackWatchdog();

    if (g_map_state != MAP_RUN_STATE_RUNNING) {
        return;
    }

    if (g_control_mode == MAP_CONTROL_SPEED_TEST) {
        return;
    }

    MapRun_UpdateFinishDetection();
    if (g_map_state == MAP_RUN_STATE_RUNNING) {
        if (g_q4_final_stop_active != 0U) {
            MapRun_UpdateQ4FinalStop();
        } else if (g_ball_lap_final_stop_active != 0U) {
            MapRun_UpdateBallLapFinalStop();
        } else {
            MapRun_UpdateLineTargets();
        }
    }
}

static void MapRun_UpdateSpeedLoop(void)
{
    uint32_t elapsed_ms;

    /* 主动刹车期间禁止已关闭的速度环把四路高电平重新拉低。 */
    if (g_motor_brake_active != 0U) {
        return;
    }

    if ((tick_ms - g_last_speed_ms) < MAP_RUN_SPEED_PERIOD_MS) {
        return;
    }

    elapsed_ms = (uint32_t) (tick_ms - g_last_speed_ms);
    g_last_speed_ms = tick_ms;

    if (MotorSpeed_Update(elapsed_ms) != 0U) {
        if ((g_q4_final_stop_active == 0U) &&
            (g_ball_lap_final_stop_active == 0U)) {
            (void) MotorSpeedLoop_UpdateLeftWheel(elapsed_ms);
            (void) MotorSpeedLoop_UpdateRightWheel(elapsed_ms);
        }
        if ((MAP_RUN_AUTO_SPEED_TELEMETRY_ENABLED != 0U) &&
            ((tick_ms - g_last_telemetry_ms) >=
                MAP_RUN_TELEMETRY_PERIOD_MS)) {
            g_last_telemetry_ms = tick_ms;
            MapRun_SendSpeedTelemetry();
        }
    }
}

static void MapRun_PadDisplayLine(void)
{
    uint8_t index = 0U;

    while ((index < 16U) && (g_display_buffer[index] != '\0')) {
        index++;
    }

    while (index < 16U) {
        g_display_buffer[index++] = ' ';
    }
    g_display_buffer[16] = '\0';
}

static void MapRun_FormatSensorBits(void)
{
    uint8_t index;

    g_display_buffer[0] = 'G';
    g_display_buffer[1] = ':';
    for (index = 0U; index < 8U; index++) {
        g_display_buffer[2U + index] =
            (((g_sensor_value >> (7U - index)) & 0x01U) != 0U) ?
            '1' : '0';
    }
    g_display_buffer[10] = ' ';
    g_display_buffer[11] = 'E';
    g_display_buffer[12] = ':';
    (void) snprintf((char *) &g_display_buffer[13],
                    sizeof(g_display_buffer) - 13U,
                    "%+3.0f",
                    g_line_error);
    MapRun_PadDisplayLine();
}

static void MapRun_UpdateCalibrationDisplay(void)
{
    const uint8_t *status_text;
    const uint8_t *action_text = (const uint8_t *) "                ";

    switch (g_calibration_state) {
        case SENSOR_CALIBRATION_WAIT_STEPPER:
            status_text = (const uint8_t *) "Wait stepper    ";
            break;
        case SENSOR_CALIBRATION_ENTER_HOLD:
            status_text = (const uint8_t *) "Entering 4s...  ";
            break;
        case SENSOR_CALIBRATION_ENTER_SETTLE:
            status_text = (const uint8_t *) "Entering...     ";
            break;
        case SENSOR_CALIBRATION_WAIT_BLACK:
            status_text = (const uint8_t *) "Place BLACK     ";
            action_text = (const uint8_t *) "Press B21       ";
            break;
        case SENSOR_CALIBRATION_BLACK_HOLD:
        case SENSOR_CALIBRATION_BLACK_SETTLE:
            status_text = (const uint8_t *) "Saving BLACK... ";
            break;
        case SENSOR_CALIBRATION_WAIT_WHITE:
            status_text = (const uint8_t *) "Place WHITE     ";
            action_text = (const uint8_t *) "Press B21       ";
            break;
        case SENSOR_CALIBRATION_WHITE_HOLD:
            status_text = (const uint8_t *) "Saving WHITE... ";
            break;
        case SENSOR_CALIBRATION_DONE:
            status_text = (const uint8_t *) "Calibration OK  ";
            break;
        default:
            status_text = (const uint8_t *) "Calibration ERR ";
            break;
    }

    OLED_ShowString(0, 0, (uint8_t *) "Gray calibrate  ", 8);
    OLED_ShowString(0, 2, (uint8_t *) status_text, 8);
    OLED_ShowString(0, 4, (uint8_t *) action_text, 8);
    OLED_ShowString(0, 6, (uint8_t *) "hold=enter      ", 8);
}

static void MapRun_UpdateQ3Display(void)
{
    if ((g_map_state == MAP_RUN_STATE_RUNNING) &&
        (g_q3_state == MAP_Q3_STATE_TO_POSITIVE)) {
        (void) snprintf((char *) g_display_buffer,
                        sizeof(g_display_buffer),
                        "M2 PUSH+  %5.2fs",
                        (float) g_run_elapsed_ms / 1000.0f);
    } else if ((g_map_state == MAP_RUN_STATE_RUNNING) &&
               (g_q3_state == MAP_Q3_STATE_DRIVE_NEGATIVE)) {
        (void) snprintf((char *) g_display_buffer,
                        sizeof(g_display_buffer),
                        "M2 RETURN %5.2fs",
                        (float) g_run_elapsed_ms / 1000.0f);
    } else if ((g_map_state == MAP_RUN_STATE_RUNNING) &&
               (g_q3_state == MAP_Q3_STATE_BRAKE_NEGATIVE)) {
        (void) snprintf((char *) g_display_buffer,
                        sizeof(g_display_buffer),
                        (g_q3_settle_capture_active != 0U) ?
                            "M2 PRECAP %5.2fs" :
                            "M2 VCATCH %5.2fs",
                        (float) g_run_elapsed_ms / 1000.0f);
    } else if ((g_map_state == MAP_RUN_STATE_RUNNING) &&
               (g_q3_state == MAP_Q3_STATE_LEVEL_SETTLE)) {
        (void) snprintf((char *) g_display_buffer,
                        sizeof(g_display_buffer),
                        (g_q3_settle_capture_active != 0U) ?
                            "M2 CAP160 %4.2fs" :
                        (g_q3_settle_lock_active != 0U) ?
                            "M2 VER150 %4.2fs" :
                            "M2 ADAPT  %4.2fs",
                        (float) g_run_elapsed_ms / 1000.0f);
    } else if ((g_map_state == MAP_RUN_STATE_FINISHED) &&
               (g_finish_reason == MAP_FINISH_Q3_COMPLETE)) {
        (void) snprintf((char *) g_display_buffer,
                        sizeof(g_display_buffer),
                        "M2 %s %5.2fs",
                        (g_q3_completed_within_time != 0U) ?
                            "PASS" : "LATE",
                        (float) g_run_elapsed_ms / 1000.0f);
    } else if ((g_map_state == MAP_RUN_STATE_FINISHED) &&
               (g_finish_reason == MAP_FINISH_MANUAL)) {
        (void) snprintf((char *) g_display_buffer,
                        sizeof(g_display_buffer),
                        "M2 MANUAL STOP");
    } else {
        (void) snprintf((char *) g_display_buffer,
                        sizeof(g_display_buffer),
                        "M2 Q3 READY B21");
    }
    MapRun_PadDisplayLine();
    OLED_ShowString(0, 0, g_display_buffer, 8);

    (void) snprintf((char *) g_display_buffer,
                    sizeof(g_display_buffer),
                    "X:%+5.1f T:%+4.0f",
                    g_ball_beam_controller.filtered_x_mm,
                    g_ball_beam_controller.target_x_mm);
    MapRun_PadDisplayLine();
    OLED_ShowString(0, 2, g_display_buffer, 8);

    (void) snprintf((char *) g_display_buffer,
                    sizeof(g_display_buffer),
                    "V:%+5.0f E:%+4.0f",
                    g_ball_beam_controller.ball_speed_mm_s,
                    g_ball_beam_controller.position_error_mm);
    MapRun_PadDisplayLine();
    OLED_ShowString(0, 4, g_display_buffer, 8);

    (void) snprintf((char *) g_display_buffer,
                    sizeof(g_display_buffer),
                    "A:%+4.1f I:%+3.1f",
                    g_ball_beam_controller.target_beam_angle_deg,
                    g_ball_beam_controller.integral_output_deg);
    MapRun_PadDisplayLine();
    OLED_ShowString(0, 6, g_display_buffer, 8);
}

static void MapRun_UpdateDisplay(void)
{
    float elapsed_seconds;
    float display_ball_target_mm = g_ball_lap_active_target_mm;

    if ((tick_ms - g_last_display_ms) < MAP_RUN_DISPLAY_PERIOD_MS) {
        return;
    }
    g_last_display_ms = tick_ms;

    if (g_calibration_state != SENSOR_CALIBRATION_IDLE) {
        MapRun_UpdateCalibrationDisplay();
        return;
    }

    if ((MapRun_IsQ3Mode() != 0U) &&
        (g_control_mode == MAP_CONTROL_LINE_FOLLOW)) {
        MapRun_UpdateQ3Display();
        return;
    }

    if ((g_map_state == MAP_RUN_STATE_RUNNING) &&
        (g_control_mode == MAP_CONTROL_SPEED_TEST)) {
        elapsed_seconds = (float) g_run_elapsed_ms / 1000.0f;
        (void) snprintf((char *) g_display_buffer,
                        sizeof(g_display_buffer),
                        "SPD %7.2fs",
                        elapsed_seconds);
    } else if (g_map_state == MAP_RUN_STATE_RUNNING) {
        elapsed_seconds = (float) g_run_elapsed_ms / 1000.0f;
        (void) snprintf((char *) g_display_buffer,
                        sizeof(g_display_buffer),
                        "M%u RUN %6.2fs",
                        (unsigned int) g_question_mode,
                        elapsed_seconds);
    } else if ((g_map_state == MAP_RUN_STATE_FINISHED) &&
               (g_finish_reason == MAP_FINISH_ONE_LAP)) {
        elapsed_seconds = (float) g_run_elapsed_ms / 1000.0f;
        if (MapRun_IsBallMode() != 0U) {
            (void) snprintf(
                (char *) g_display_buffer,
                sizeof(g_display_buffer),
                "M%u %s %5.2fs",
                (unsigned int) g_question_mode,
                (g_ball_lap_completed_within_time == 0U) ? "LATE" :
                ((g_ball_lap_limit_exceeded != 0U) ? "OVER" : "PASS"),
                elapsed_seconds);
        } else {
            (void) snprintf((char *) g_display_buffer,
                            sizeof(g_display_buffer),
                            "M%u DONE %5.2fs",
                            (unsigned int) g_question_mode,
                            elapsed_seconds);
        }
    } else if ((g_map_state == MAP_RUN_STATE_FINISHED) &&
               (g_finish_reason == MAP_FINISH_Q4_B_PASSED)) {
        elapsed_seconds = (float) g_run_elapsed_ms / 1000.0f;
        (void) snprintf((char *) g_display_buffer,
                        sizeof(g_display_buffer),
                        "M3 B PASS %4.2fs",
                        elapsed_seconds);
    } else if ((g_map_state == MAP_RUN_STATE_FINISHED) &&
               (g_finish_reason == MAP_FINISH_MANUAL)) {
        (void) snprintf((char *) g_display_buffer,
                        sizeof(g_display_buffer),
                        "MANUAL STOP");
    } else if ((g_map_state == MAP_RUN_STATE_FINISHED) &&
               (g_finish_reason == MAP_FINISH_ENCODER_FAULT)) {
        (void) snprintf((char *) g_display_buffer,
                        sizeof(g_display_buffer),
                        "ENCODER FAULT");
    } else if ((g_map_state == MAP_RUN_STATE_FINISHED) &&
               (g_finish_reason == MAP_FINISH_NO_START_LINE)) {
        (void) snprintf((char *) g_display_buffer,
                        sizeof(g_display_buffer),
                        "NO LINE START");
    } else if (MapRun_IsQuestionModeReady() != 0U) {
        (void) snprintf((char *) g_display_buffer,
                        sizeof(g_display_buffer),
                        "M%u Q%u READY B21",
                        (unsigned int) g_question_mode,
                        (unsigned int) MapRun_GetQuestionNumber());
    } else {
        (void) snprintf((char *) g_display_buffer,
                        sizeof(g_display_buffer),
                        "M%u Q%u NOT READY",
                        (unsigned int) g_question_mode,
                        (unsigned int) MapRun_GetQuestionNumber());
    }
    MapRun_PadDisplayLine();
    OLED_ShowString(0, 0, g_display_buffer, 8);

    MapRun_FormatSensorBits();
    OLED_ShowString(0, 2, g_display_buffer, 8);

    (void) snprintf((char *) g_display_buffer,
                    sizeof(g_display_buffer),
                    "L:%3.0f R:%3.0f A%u",
                    MotorSpeed_GetLeftWheelRPM(),
                    MotorSpeed_GetRightWheelRPM(),
                    (unsigned int) g_sensor_active_count);
    MapRun_PadDisplayLine();
    OLED_ShowString(0, 4, g_display_buffer, 8);

    if (MapRun_IsQ4Mode() != 0U) {
        (void) snprintf((char *) g_display_buffer,
                        sizeof(g_display_buffer),
                        "D:%5lu X:%+4.0f",
                        (unsigned long) g_run_distance_counts,
                        g_ball_beam_controller.filtered_x_mm);
    } else if (MapRun_IsBallMode() != 0U) {
        if ((g_map_state != MAP_RUN_STATE_RUNNING) &&
            (g_question_mode == MAP_QUESTION_MODE_5_Q6)) {
            display_ball_target_mm = g_q6_target_position_mm;
        }
        (void) snprintf((char *) g_display_buffer,
                        sizeof(g_display_buffer),
                        "X:%+4.0f T:%+4.0f",
                        g_ball_beam_controller.filtered_x_mm,
                        display_ball_target_mm);
    } else if (BNO085_IsReady() != 0U) {
        (void) snprintf((char *) g_display_buffer,
                        sizeof(g_display_buffer),
                        "D:%5lu Y:%+4.0f",
                        (unsigned long) g_run_distance_counts,
                        yaw);
    } else {
        (void) snprintf((char *) g_display_buffer,
                        sizeof(g_display_buffer),
                        "D:%5lu Y:----",
                        (unsigned long) g_run_distance_counts);
    }
    MapRun_PadDisplayLine();
    OLED_ShowString(0, 6, g_display_buffer, 8);
}

void MapRun_Init(void)
{
    uint8_t initial_b21_pressed;
    uint8_t initial_sw1_pressed;
    uint8_t initial_sw2_pressed;
    uint8_t initial_sensor;

    /*
     * 步进平台在地图底盘验证阶段只初始化，不输出脉冲。
     * 直流电机四路 PWM 先强制拉低，再启动 TIMA0 计数器。
     */
    StepperMotor_Init();
    BallBeamController_Init(&g_ball_beam_controller);
    /*
     * 机构没有零位开关，上电后只能把当前机械姿态作为软件零点。
     * 比赛前把摆杆放到固定、近似水平的起始姿态再上电，固定安装坡度
     * 由 level trim 补偿。调试时仍可使用 BALL,ZERO 重新确认该姿态。
     */
    g_ball_stepper_zeroed = 1U;
    MotorPWM_InitDefaults();
    DL_TimerA_startCounter(PWM_0_INST);

    SysTick_Init();
    __enable_irq();

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *) "MAP init...     ", 8);

    (void) Sensor_Init();
    initial_sensor = Sensor_Read_Grayscale();
    (void) BNO085_Init();

    /*
     * BNO085 初始化完成后再开启 GPIOB 组中断。
     * 同一个中断入口会同时处理 BNO085 INT 和左右编码器 A 相边沿。
     */
    Interrupt_Init();
    CameraLink_Init();
    g_camera_forwarded_frame_count = 0UL;
    g_camera_control_frame_count = 0UL;

    MotorSpeed_SetSampleTimeMs((uint32_t) MAP_RUN_SPEED_PERIOD_MS);
    MotorSpeedLoop_SetLeftWheelPI(0.1f, 0.2f);
    MotorSpeedLoop_SetRightWheelPI(0.1f, 0.2f);
    MotorSpeed_Reset();
    MotorSpeedLoop_EnableLeftWheel(0U);
    MotorSpeedLoop_EnableRightWheel(0U);

    g_map_state = MAP_RUN_STATE_READY;
    g_control_mode = MAP_CONTROL_LINE_FOLLOW;
    g_question_mode = MAP_QUESTION_MODE_1_Q2;
    g_finish_reason = MAP_FINISH_NONE;
    g_calibration_state = SENSOR_CALIBRATION_IDLE;
    g_run_start_ms = tick_ms;
    g_run_elapsed_ms = 0UL;
    g_last_control_ms = tick_ms;
    g_last_speed_ms = tick_ms;
    g_last_telemetry_ms = tick_ms;
    g_last_ball_log_ms = tick_ms;
    g_last_ball_lap_telemetry_ms = tick_ms;
    g_last_display_ms = tick_ms - MAP_RUN_DISPLAY_PERIOD_MS;
    g_calibration_state_start_ms = tick_ms;
    g_motor_brake_start_ms = tick_ms;
    g_run_distance_counts = 0U;
    g_finish_heading_reference_yaw_deg = 0.0f;
    g_finish_heading_error_deg = 0.0f;
    g_finish_heading_rate_offset_deg_s = 0.0f;
    g_finish_heading_reference_valid = 0U;
    g_finish_heading_active = 0U;
    MapRun_ResetBallLapTurnTracking(0U);
    g_q4_right_second_seen = 0U;
    g_q4_ball_limit_exceeded = 0U;
    g_q4_max_ball_error_mm = 0.0f;
    g_q4_acceleration_feedforward_deg = 0.0f;
    g_q4_position_guard_deg = 0.0f;
    g_q4_velocity_damping_deg = 0.0f;
    MapRun_ResetQ4VelocityObserver();
    g_q4_feedforward_update_ms = tick_ms;
    g_q4_stop_feedforward_start_ms = tick_ms;
    g_q4_stop_feedforward_active = 0U;
    g_q4_final_stop_start_ms = tick_ms;
    g_q4_final_stop_active = 0U;
    g_q4_park_hold_active = 0U;
    g_q4_final_stop_start_rpm = 0.0f;
    g_q4_final_stop_left_duty_percent = 0.0f;
    g_q4_final_stop_right_duty_percent = 0.0f;
    g_q4_camera_fallback_active = 0U;
    g_ball_lap_camera_fallback_active = 0U;
    g_ball_lap_limit_exceeded = 0U;
    g_ball_lap_completed_within_time = 0U;
    g_ball_lap_final_stop_start_ms = tick_ms;
    g_ball_lap_final_stop_active = 0U;
    g_ball_lap_final_stop_start_rpm = 0.0f;
    g_ball_lap_final_stop_left_duty_percent = 0.0f;
    g_ball_lap_final_stop_right_duty_percent = 0.0f;
    g_q6_target_position_mm = 0.0f;
    g_ball_lap_active_target_mm = 0.0f;
    g_ball_lap_max_error_mm = 0.0f;
    g_motor_brake_active = 0U;
    g_q3_state = MAP_Q3_STATE_IDLE;
    g_q3_completed_within_time = 0U;
    g_q3_time_warning_sent = 0U;
    g_q3_saved_pid_valid = 0U;
    g_q3_manual_beam_active = 0U;
    g_q3_hold_entry_seed_pending = 0U;
    g_q3_settle_lock_active = 0U;
    g_q3_settle_capture_active = 0U;
    g_q3_capture_pd_active = 0U;
    g_q3_brake_low_speed_active = 0U;
    g_q3_last_camera_frame_id = 0UL;
    g_q3_phase_start_ms = (uint32_t) tick_ms;
    g_q3_motion_last_ms = (uint32_t) tick_ms;
    g_q3_brake_low_speed_start_ms = (uint32_t) tick_ms;
    g_q3_window_start_ms = (uint32_t) tick_ms;
    g_q3_hold_last_update_ms = (uint32_t) tick_ms;
    g_q3_hold_last_camera_frame_id = 0UL;
    g_q3_hold_last_filtered_x_mm = 0.0f;
    g_q3_hold_frame_delta_mm = 0.0f;
    g_q3_hold_feedback_speed_mm_s = 0.0f;
    g_q3_hold_profile_target_speed_mm_s = 0.0f;
    g_q3_hold_profile_blend = 0.0f;
    MapRun_ResetQ3StictionAssist();
    g_q3_window_start_x_mm = 0.0f;
    g_q3_window_min_x_mm = 0.0f;
    g_q3_window_max_x_mm = 0.0f;
    g_q3_hold_bias_deg = 0.0f;
    g_q3_manual_beam_angle_deg = 0.0f;
    g_q3_settle_lock_angle_deg = 0.0f;
    g_q3_brake_planned_speed_mm_s = 0.0f;
    g_q3_brake_angle_deg = 0.0f;
    g_q3_motion_last_x_mm = 0.0f;
    g_q3_fresh_speed_mm_s = 0.0f;
    g_q3_saved_kp = 0.0f;
    g_q3_saved_ki = 0.0f;
    g_q3_saved_kd = 0.0f;
    g_sensor_value = initial_sensor;
    MapLineController_Reset(&g_line_controller);
    MapLineController_Update(&g_line_controller,
                             initial_sensor,
                             &g_line_result);
    g_sensor_active_count = g_line_result.active_count;
    g_line_error = g_line_result.error;
    g_correction_rpm = g_line_result.correction_rpm;
    MapRun_ResetYawCascade();
    g_speed_test_left_target_rpm = MAP_RUN_SPEED_TEST_DEFAULT_RPM;
    g_speed_test_right_target_rpm = MAP_RUN_SPEED_TEST_DEFAULT_RPM;
    g_uart_rx_write_index = 0U;
    g_uart_rx_read_index = 0U;
    g_uart_rx_overflow = 0U;
    g_uart_line_length = 0U;
    g_ball_log_enabled = 0U;

    initial_b21_pressed =
        (DL_GPIO_readPins(GPIO_Button_PIN_Button_PORT,
                          GPIO_Button_PIN_Button_PIN) == 0U) ? 1U : 0U;
    initial_sw1_pressed =
        (DL_GPIO_readPins(GPIO_Button_PIN_Button_SW1_PORT,
                          GPIO_Button_PIN_Button_SW1_PIN) == 0U) ? 1U : 0U;
    initial_sw2_pressed =
        (DL_GPIO_readPins(GPIO_Button_PIN_Button_SW2_PORT,
                          GPIO_Button_PIN_Button_SW2_PIN) == 0U) ? 1U : 0U;
    MapRun_InitButtonState(&g_b21_button, initial_b21_pressed);
    MapRun_InitButtonState(&g_sw1_button, initial_sw1_pressed);
    MapRun_InitButtonState(&g_sw2_button, initial_sw2_pressed);

    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    MapRun_UpdateDisplay();
}

void MapRun_RunStep(void)
{
    CameraLink_Task((uint32_t) tick_ms);
    MapRun_ProcessBallCameraData();
    if (MAP_RUN_AUTO_CAMERA_FORWARD_ENABLED != 0U) {
        MapRun_ForwardCameraData();
    }
    MapRun_UpdateBluetooth();
    MapRun_UpdateBallBeam();
    MapRun_UpdateBallLog();
    MapRun_UpdateBallLapTelemetry();
    if (BNO085_UpdateIfDataReady() != 0U) {
        MapYawRateController_AddYawSample(
            &g_yaw_rate_controller,
            yaw,
            (uint32_t) tick_ms);
        MapRun_AddBallLapYawSample(yaw);
        MapCurveHold_AddYawSample(&g_curve_hold, yaw);
        g_measured_yaw_rate_deg_s =
            MapYawRateController_GetMeasuredYawRate(
                &g_yaw_rate_controller);

        /* 起跑前样本不可用时，用起跑后最早的新姿态补记航向基准。 */
        if ((g_map_state == MAP_RUN_STATE_RUNNING) &&
            (g_question_mode == MAP_QUESTION_MODE_1_Q2) &&
            (g_finish_heading_reference_valid == 0U) &&
            (g_run_distance_counts < 500U)) {
            g_finish_heading_reference_yaw_deg = yaw;
            g_finish_heading_reference_valid = 1U;
        }
    }
    MapRun_UpdateButtons();

    /* 运行中按键一旦确认按下便立即停机，不等待松手。 */
    if ((g_map_state == MAP_RUN_STATE_RUNNING) &&
        (g_b21_button.pressed_event != 0U)) {
        MapRun_StopMotors(MAP_FINISH_MANUAL);
        g_b21_button.suppress_release = 1U;
    }

    if (g_calibration_state == SENSOR_CALIBRATION_IDLE) {
        if ((g_map_state != MAP_RUN_STATE_RUNNING) &&
            (g_b21_button.long_release_event != 0U)) {
            MapRun_BeginCalibration();
        } else if ((g_map_state != MAP_RUN_STATE_RUNNING) &&
                   (g_sw1_button.pressed_event != 0U) &&
                   (g_sw2_button.pressed_event == 0U)) {
            MapRun_ChangeQuestionMode(-1);
        } else if ((g_map_state != MAP_RUN_STATE_RUNNING) &&
                   (g_sw2_button.pressed_event != 0U) &&
                   (g_sw1_button.pressed_event == 0U)) {
            MapRun_ChangeQuestionMode(1);
        } else if ((g_map_state != MAP_RUN_STATE_RUNNING) &&
                   (g_b21_button.short_release_event != 0U)) {
            if (MapRun_CaptureQ6TargetFromCamera() != 0U) {
                MapRun_Start();
            }
        }
    } else if (g_b21_button.short_release_event != 0U) {
        MapRun_HandleCalibrationShortPress();
    }

    MapRun_UpdateCalibration();
    MapRun_UpdateControl();
    MapRun_UpdateMotorBrake();
    MapRun_UpdateSpeedLoop();
    MapRun_UpdateDisplay();
}
