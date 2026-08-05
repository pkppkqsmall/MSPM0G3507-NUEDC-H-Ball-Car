#include "ti_msp_dl_config.h"
#include "motor_pwm.h"
#include "clock.h"

extern volatile long encoder_left_count;
extern volatile long encoder_right_count;

/*
 * 转速计算使用的编码器和减速箱参数：
 * 1. 编码器线数为 13 PPR。
 * 2. 当前 GPIO 中断实际等效 2 倍频，电机轴每转一圈约 26 个计数。
 * 3. 减速比为 1:28，实测车轮输出轴每转一圈约 728 个计数。
 * 注意：编码器硬件可以按四倍频理解，但当前程序只对 A 相中断计数，
 * 实测一圈约 728，因此速度换算必须按 2 倍频等效值计算。
 */
#define MOTOR_ENCODER_PPR               (13.0f)
#define MOTOR_ENCODER_EFFECTIVE_FACTOR  (2.0f)
#define MOTOR_GEAR_RATIO                (28.0f)
#define MOTOR_WHEEL_COUNTS_PER_REV      (MOTOR_ENCODER_PPR * MOTOR_ENCODER_EFFECTIVE_FACTOR * MOTOR_GEAR_RATIO)
#define MOTOR_SPEED_FILTER_WINDOW_SIZE  (5U)
#define LEFT_WHEEL_AIN1_CHANNEL         (0U)
#define LEFT_WHEEL_AIN2_CHANNEL         (1U)
#define RIGHT_WHEEL_BIN1_CHANNEL        (2U)
#define RIGHT_WHEEL_BIN2_CHANNEL        (3U)
#define LEFT_WHEEL_DIRECTION_SIGN       (1.0f)
#define RIGHT_WHEEL_DIRECTION_SIGN      (-1.0f)
#define MOTOR_SPEED_LOOP_DUTY_LIMIT     (100.0f)
#define MOTOR_PWM_CHANNEL_COUNT         (4U)
#define MOTOR_PWM_TIMER_PERIOD_TICKS    (1000U)
#define MOTOR_PWM_PIN_MODE_GPIO_LOW     (0U)
#define MOTOR_PWM_PIN_MODE_PWM          (1U)
#define MOTOR_PWM_PIN_MODE_GPIO_HIGH    (2U)
#define MOTOR_PWM_PIN_MODE_UNKNOWN      (0xFFU)

/*
 * 低速段不再做闭环 PID，而是直接按目标 RPM 生成一条线性 PWM 曲线。
 * 公式：最终PWM = 52% + (目标RPM / 30RPM) * (58% - 52%)。
 * 超过 30RPM 后再进入 PI 速度环，避免低速测速量化和静摩擦导致抽搐。
 */
#define MOTOR_DEADZONE_EXTRA_DUTY_PERCENT      (1.0f)
#define MOTOR_SPEED_LOOP_OPEN_LOOP_MAX_RPM     (30.0f)
#define MOTOR_SPEED_LOOP_OPEN_LOOP_MIN_DUTY_PERCENT (52.0f)
#define MOTOR_SPEED_LOOP_OPEN_LOOP_MAX_DUTY_PERCENT (58.0f)

/*
 * 当前使用的是直径 65 mm 的橡胶轮胎。
 * 这个参数用于把 RPM 换算成线速度。
 */
#define MOTOR_WHEEL_DIAMETER_MM         (65.0f)
#define MOTOR_PI                        (3.1415926f)
#define MOTOR_WHEEL_CIRCUMFERENCE_MM    (MOTOR_WHEEL_DIAMETER_MM * MOTOR_PI)

/*
 * 默认速度采样周期，单位为 ms。
 * 如果主循环里用于测速的 (tick_ms - last_speed_tick) 周期发生变化，
 * 这里也要同步修改。
 */
volatile uint32_t g_motor_speed_sample_time_ms = 50U;

/*
 * 根据你实测“占空比 56 才刚开始转”的结果，
 * 这里把死区补偿默认值设为 56%。
 */
volatile float g_motor_deadzone_duty_percent = 56.0f;

/*
 * 启动脉冲默认再比死区略高一些，给电机一个更稳的拉起过程。
 */
volatile float g_motor_start_boost_duty_percent = 60.0f;
volatile uint32_t g_motor_start_boost_time_ms = 0U;

/* 保存上一次采样时的编码器计数，用于做差分测速 */
static long g_left_encoder_last_count = 0;
static long g_right_encoder_last_count = 0;

/* 保存最近一次计算得到的左右轮转速，单位为 RPM */
static float g_left_wheel_rpm = 0.0f;
static float g_right_wheel_rpm = 0.0f;
static float g_left_wheel_rpm_raw = 0.0f;
static float g_right_wheel_rpm_raw = 0.0f;

/* 保存最近一次计算得到的左右轮线速度，单位为 mm/s */
static float g_left_wheel_linear_speed_mm_per_sec = 0.0f;
static float g_right_wheel_linear_speed_mm_per_sec = 0.0f;

/* 用于对最近几次测速结果做滑动平均，减小低速时的跳动。 */
static float g_left_wheel_rpm_history[MOTOR_SPEED_FILTER_WINDOW_SIZE] = {0.0f};
static float g_right_wheel_rpm_history[MOTOR_SPEED_FILTER_WINDOW_SIZE] = {0.0f};
static float g_left_wheel_rpm_sum = 0.0f;
static float g_right_wheel_rpm_sum = 0.0f;
static uint8_t g_speed_filter_count = 0U;
static uint8_t g_speed_filter_index = 0U;

/* 单个车轮速度环的运行状态，左右轮共用同一套更新逻辑。 */
typedef struct {
    float target_rpm;
    float kp;
    float ki;
    float integral;
    float output_duty_percent;
    int8_t last_direction;
    uint8_t enabled;
    uint8_t start_boost_active;
    unsigned long start_boost_begin_ms;
} MotorSpeedLoopState;

typedef void (*MotorSpeedLoopDutyOutput)(float signed_duty_percent);
typedef void (*MotorSpeedLoopStopOutput)(void);

static MotorSpeedLoopState g_left_speed_loop = {
    0.0f, 0.1f, 0.2f, 0.0f, 0.0f, 0, 0U, 0U, 0UL
};
static MotorSpeedLoopState g_right_speed_loop = {
    0.0f, 0.1f, 0.2f, 0.0f, 0.0f, 0, 0U, 0U, 0UL
};

/* 记录每个 PWM 引脚当前复用状态，避免每次刷新占空比都重复初始化 GPIO。 */
static uint8_t g_motor_pwm_pin_mode[MOTOR_PWM_CHANNEL_COUNT] = {
    MOTOR_PWM_PIN_MODE_UNKNOWN,
    MOTOR_PWM_PIN_MODE_UNKNOWN,
    MOTOR_PWM_PIN_MODE_UNKNOWN,
    MOTOR_PWM_PIN_MODE_UNKNOWN
};

/*
 * 对占空比做统一限幅。
 * 这样外部无论传入过大、过小还是负数，最后都会落到安全范围内。
 */
static float MotorPWM_ClampDutyPercent(float duty_percent)
{
    if (duty_percent > 100.0f) {
        return 100.0f;
    }

    if (duty_percent < 0.01f) {
        return 0.0f;
    }

    return duty_percent;
}

/* 对带方向的占空比做统一限幅，允许范围为 -100% 到 100%。 */
static float MotorSpeedLoop_ClampSignedDuty(float duty_percent)
{
    float duty_limit = MOTOR_SPEED_LOOP_DUTY_LIMIT - MotorPWM_ClampDutyPercent(MotorPWM_GetDeadzoneDutyPercent());

    if (duty_limit < 0.0f) {
        duty_limit = 0.0f;
    }

    if (duty_percent > duty_limit) {
        return duty_limit;
    }

    if (duty_percent < (-duty_limit)) {
        return (-duty_limit);
    }

    return duty_percent;
}

/* 计算浮点数绝对值，避免额外引入数学库。 */
static float MotorSpeedLoop_AbsFloat(float value)
{
    return (value < 0.0f) ? (-value) : value;
}

/* 根据数值正负返回方向：正为 1，负为 -1，零为 0。 */
static int8_t MotorSpeedLoop_GetDirection(float value)
{
    if (value > 0.0f) {
        return 1;
    }

    if (value < 0.0f) {
        return -1;
    }

    return 0;
}

/* 0RPM 表示停机，其它目标保持原值，低速段交给开环线性 PWM 处理。 */
static float MotorSpeedLoop_ClampTargetRPM(float target_rpm)
{
    float abs_target_rpm = MotorSpeedLoop_AbsFloat(target_rpm);

    if (abs_target_rpm < 0.01f) {
        return 0.0f;
    }

    return target_rpm;
}

/* 低速 0~30RPM 使用开环 PWM，超过该范围才用闭环 PI。 */
static uint8_t MotorSpeedLoop_ShouldUseOpenLoop(float target_rpm)
{
    float abs_target_rpm = MotorSpeedLoop_AbsFloat(target_rpm);

    return ((abs_target_rpm >= 0.01f) &&
            (abs_target_rpm <= MOTOR_SPEED_LOOP_OPEN_LOOP_MAX_RPM)) ? 1U : 0U;
}

/*
 * 将一个采样周期内的编码器计数差值换算成车轮转速 RPM。
 * 这里已经把减速比计算进去了，因此结果是减速后输出轴的 RPM。
 */
static float MotorSpeed_CountDeltaToWheelRPM(long delta_count, uint32_t sample_time_ms)
{
    if (sample_time_ms == 0U) {
        return 0.0f;
    }

    return ((float) delta_count * 60000.0f) /
           (MOTOR_WHEEL_COUNTS_PER_REV * (float) sample_time_ms);
}

/*
 * 将车轮 RPM 换算成线速度，单位为 mm/s。
 * 公式：
 * 线速度 = 轮胎周长 * RPM / 60
 */
static float MotorSpeed_WheelRPMToLinearSpeedMmPerSec(float wheel_rpm)
{
    return (wheel_rpm * MOTOR_WHEEL_CIRCUMFERENCE_MM) / 60.0f;
}

/* 对带方向的占空比做死区补偿。
 * 正反方向共用同一个死区门槛，补偿完成后再把方向符号加回去。 */
static float MotorSpeedLoop_ApplySignedDeadzoneCompensation(float signed_duty_percent)
{
    float abs_duty = MotorSpeedLoop_AbsFloat(signed_duty_percent);
    float deadzone_duty = MotorPWM_ClampDutyPercent(MotorPWM_GetDeadzoneDutyPercent());
    float compensated_abs_duty;

    if (abs_duty < 0.01f) {
        return 0.0f;
    }

    compensated_abs_duty = deadzone_duty +
                           MOTOR_DEADZONE_EXTRA_DUTY_PERCENT +
                           abs_duty;
    if (compensated_abs_duty > MOTOR_SPEED_LOOP_DUTY_LIMIT) {
        compensated_abs_duty = MOTOR_SPEED_LOOP_DUTY_LIMIT;
    }

    if (signed_duty_percent > 0.0f) {
        return compensated_abs_duty;
    }

    if (signed_duty_percent < 0.0f) {
        return (-compensated_abs_duty);
    }

    return 0.0f;
}

/* 把低速目标 RPM 映射成最终带方向的 PWM，占空比曲线保持简单可调。 */
static float MotorSpeedLoop_GetOpenLoopDuty(float target_rpm)
{
    float abs_target_rpm = MotorSpeedLoop_AbsFloat(target_rpm);
    float duty_span;
    float output_duty;

    if (abs_target_rpm < 0.01f) {
        return 0.0f;
    }

    if (abs_target_rpm > MOTOR_SPEED_LOOP_OPEN_LOOP_MAX_RPM) {
        abs_target_rpm = MOTOR_SPEED_LOOP_OPEN_LOOP_MAX_RPM;
    }

    duty_span = MOTOR_SPEED_LOOP_OPEN_LOOP_MAX_DUTY_PERCENT -
                MOTOR_SPEED_LOOP_OPEN_LOOP_MIN_DUTY_PERCENT;
    output_duty = MOTOR_SPEED_LOOP_OPEN_LOOP_MIN_DUTY_PERCENT +
                  ((abs_target_rpm / MOTOR_SPEED_LOOP_OPEN_LOOP_MAX_RPM) * duty_span);

    if (target_rpm > 0.0f) {
        return output_duty;
    }

    return (-output_duty);
}

/* 将速度环输出限制在目标转向同一方向。
 * 例如目标是正转时，速度超调只能减小正向占空比，不能直接打成反转。 */
static float MotorSpeedLoop_ClampToTargetDirection(float target_rpm, float signed_output)
{
    if (target_rpm > 0.0f) {
        return (signed_output > 0.0f) ? signed_output : 0.0f;
    }

    if (target_rpm < 0.0f) {
        return (signed_output < 0.0f) ? signed_output : 0.0f;
    }

    return 0.0f;
}

static uint32_t MotorPWM_DutyPercentToCompareValue(float duty_percent)
{
    float compare_value;

    compare_value = (float) MOTOR_PWM_TIMER_PERIOD_TICKS -
                    ((duty_percent * (float) MOTOR_PWM_TIMER_PERIOD_TICKS) / 100.0f);

    if (compare_value < 0.0f) {
        return 0U;
    }

    if (compare_value > (float) MOTOR_PWM_TIMER_PERIOD_TICKS) {
        return MOTOR_PWM_TIMER_PERIOD_TICKS;
    }

    return (uint32_t) compare_value;
}

static void MotorPWM_SetChannelLow(uint8_t channel)
{
    if (channel >= MOTOR_PWM_CHANNEL_COUNT) {
        return;
    }

    if (g_motor_pwm_pin_mode[channel] != MOTOR_PWM_PIN_MODE_GPIO_LOW) {
        switch (channel) {
            case LEFT_WHEEL_AIN1_CHANNEL:
                DL_GPIO_initDigitalOutput(GPIO_PWM_0_C0_IOMUX);
                DL_GPIO_enableOutput(GPIO_PWM_0_C0_PORT, GPIO_PWM_0_C0_PIN);
                break;
            case LEFT_WHEEL_AIN2_CHANNEL:
                DL_GPIO_initDigitalOutput(GPIO_PWM_0_C1_IOMUX);
                DL_GPIO_enableOutput(GPIO_PWM_0_C1_PORT, GPIO_PWM_0_C1_PIN);
                break;
            case RIGHT_WHEEL_BIN1_CHANNEL:
                DL_GPIO_initDigitalOutput(GPIO_PWM_0_C2_IOMUX);
                DL_GPIO_enableOutput(GPIO_PWM_0_C2_PORT, GPIO_PWM_0_C2_PIN);
                break;
            case RIGHT_WHEEL_BIN2_CHANNEL:
                DL_GPIO_initDigitalOutput(GPIO_PWM_0_C3_IOMUX);
                DL_GPIO_enableOutput(GPIO_PWM_0_C3_PORT, GPIO_PWM_0_C3_PIN);
                break;
            default:
                return;
        }

        g_motor_pwm_pin_mode[channel] = MOTOR_PWM_PIN_MODE_GPIO_LOW;
    }

    switch (channel) {
        case LEFT_WHEEL_AIN1_CHANNEL:
            DL_GPIO_clearPins(GPIO_PWM_0_C0_PORT, GPIO_PWM_0_C0_PIN);
            break;
        case LEFT_WHEEL_AIN2_CHANNEL:
            DL_GPIO_clearPins(GPIO_PWM_0_C1_PORT, GPIO_PWM_0_C1_PIN);
            break;
        case RIGHT_WHEEL_BIN1_CHANNEL:
            DL_GPIO_clearPins(GPIO_PWM_0_C2_PORT, GPIO_PWM_0_C2_PIN);
            break;
        case RIGHT_WHEEL_BIN2_CHANNEL:
            DL_GPIO_clearPins(GPIO_PWM_0_C3_PORT, GPIO_PWM_0_C3_PIN);
            break;
        default:
            break;
    }
}

static void MotorPWM_SetChannelHigh(uint8_t channel)
{
    if (channel >= MOTOR_PWM_CHANNEL_COUNT) {
        return;
    }

    if (g_motor_pwm_pin_mode[channel] !=
        MOTOR_PWM_PIN_MODE_GPIO_HIGH) {
        switch (channel) {
            case LEFT_WHEEL_AIN1_CHANNEL:
                DL_GPIO_initDigitalOutput(GPIO_PWM_0_C0_IOMUX);
                DL_GPIO_enableOutput(GPIO_PWM_0_C0_PORT,
                                     GPIO_PWM_0_C0_PIN);
                break;
            case LEFT_WHEEL_AIN2_CHANNEL:
                DL_GPIO_initDigitalOutput(GPIO_PWM_0_C1_IOMUX);
                DL_GPIO_enableOutput(GPIO_PWM_0_C1_PORT,
                                     GPIO_PWM_0_C1_PIN);
                break;
            case RIGHT_WHEEL_BIN1_CHANNEL:
                DL_GPIO_initDigitalOutput(GPIO_PWM_0_C2_IOMUX);
                DL_GPIO_enableOutput(GPIO_PWM_0_C2_PORT,
                                     GPIO_PWM_0_C2_PIN);
                break;
            case RIGHT_WHEEL_BIN2_CHANNEL:
                DL_GPIO_initDigitalOutput(GPIO_PWM_0_C3_IOMUX);
                DL_GPIO_enableOutput(GPIO_PWM_0_C3_PORT,
                                     GPIO_PWM_0_C3_PIN);
                break;
            default:
                return;
        }

        g_motor_pwm_pin_mode[channel] =
            MOTOR_PWM_PIN_MODE_GPIO_HIGH;
    }

    switch (channel) {
        case LEFT_WHEEL_AIN1_CHANNEL:
            DL_GPIO_setPins(GPIO_PWM_0_C0_PORT, GPIO_PWM_0_C0_PIN);
            break;
        case LEFT_WHEEL_AIN2_CHANNEL:
            DL_GPIO_setPins(GPIO_PWM_0_C1_PORT, GPIO_PWM_0_C1_PIN);
            break;
        case RIGHT_WHEEL_BIN1_CHANNEL:
            DL_GPIO_setPins(GPIO_PWM_0_C2_PORT, GPIO_PWM_0_C2_PIN);
            break;
        case RIGHT_WHEEL_BIN2_CHANNEL:
            DL_GPIO_setPins(GPIO_PWM_0_C3_PORT, GPIO_PWM_0_C3_PIN);
            break;
        default:
            break;
    }
}

static void MotorPWM_SetChannelPwmMode(uint8_t channel)
{
    if (channel >= MOTOR_PWM_CHANNEL_COUNT) {
        return;
    }

    if (g_motor_pwm_pin_mode[channel] == MOTOR_PWM_PIN_MODE_PWM) {
        return;
    }

    switch (channel) {
        case LEFT_WHEEL_AIN1_CHANNEL:
            DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_0_C0_IOMUX, GPIO_PWM_0_C0_IOMUX_FUNC);
            DL_GPIO_enableOutput(GPIO_PWM_0_C0_PORT, GPIO_PWM_0_C0_PIN);
            break;
        case LEFT_WHEEL_AIN2_CHANNEL:
            DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_0_C1_IOMUX, GPIO_PWM_0_C1_IOMUX_FUNC);
            DL_GPIO_enableOutput(GPIO_PWM_0_C1_PORT, GPIO_PWM_0_C1_PIN);
            break;
        case RIGHT_WHEEL_BIN1_CHANNEL:
            DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_0_C2_IOMUX, GPIO_PWM_0_C2_IOMUX_FUNC);
            DL_GPIO_enableOutput(GPIO_PWM_0_C2_PORT, GPIO_PWM_0_C2_PIN);
            break;
        case RIGHT_WHEEL_BIN2_CHANNEL:
            DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_0_C3_IOMUX, GPIO_PWM_0_C3_IOMUX_FUNC);
            DL_GPIO_enableOutput(GPIO_PWM_0_C3_PORT, GPIO_PWM_0_C3_PIN);
            break;
        default:
            return;
    }

    g_motor_pwm_pin_mode[channel] = MOTOR_PWM_PIN_MODE_PWM;
}

static void MotorPWM_SetChannelCompareValue(uint8_t channel, uint32_t compare_value)
{
    switch (channel) {
        case LEFT_WHEEL_AIN1_CHANNEL:
            DL_TimerA_setCaptureCompareValue(PWM_0_INST, compare_value, DL_TIMER_CC_0_INDEX);
            break;
        case LEFT_WHEEL_AIN2_CHANNEL:
            DL_TimerA_setCaptureCompareValue(PWM_0_INST, compare_value, DL_TIMER_CC_1_INDEX);
            break;
        case RIGHT_WHEEL_BIN1_CHANNEL:
            DL_TimerA_setCaptureCompareValue(PWM_0_INST, compare_value, DL_TIMER_CC_2_INDEX);
            break;
        case RIGHT_WHEEL_BIN2_CHANNEL:
            DL_TimerA_setCaptureCompareValue(PWM_0_INST, compare_value, DL_TIMER_CC_3_INDEX);
            break;
        default:
            break;
    }
}

/* 将新一次测速结果放入滑动窗口，并返回平均后的速度值。 */
static float MotorSpeed_UpdateAverage(float new_rpm,
                                      float *history,
                                      float *sum,
                                      uint8_t index,
                                      uint8_t valid_count)
{
    *sum -= history[index];
    history[index] = new_rpm;
    *sum += new_rpm;

    if (valid_count == 0U) {
        return 0.0f;
    }

    return (*sum / (float) valid_count);
}

/* 停止左轮输出，只关闭左轮的 AIN1/AIN2。 */
static void MotorSpeedLoop_StopLeftWheel(void)
{
    Set_PWM_DutyCycle(LEFT_WHEEL_AIN1_CHANNEL, 0.0f);
    Set_PWM_DutyCycle(LEFT_WHEEL_AIN2_CHANNEL, 0.0f);
}

/* 停止右轮输出，只关闭右轮的 BIN1/BIN2。 */
static void MotorSpeedLoop_StopRightWheel(void)
{
    Set_PWM_DutyCycle(RIGHT_WHEEL_BIN1_CHANNEL, 0.0f);
    Set_PWM_DutyCycle(RIGHT_WHEEL_BIN2_CHANNEL, 0.0f);
}

/* 把带方向的占空比输出到左轮。
 * 正值对应正转，负值对应反转。 */
static void MotorSpeedLoop_ApplyLeftWheelDuty(float signed_duty_percent)
{
    float abs_duty = MotorPWM_ClampDutyPercent(MotorSpeedLoop_AbsFloat(signed_duty_percent));

    if (abs_duty < 0.01f) {
        MotorSpeedLoop_StopLeftWheel();
        return;
    }

    if (signed_duty_percent > 0.0f) {
        Set_PWM_DutyCycle(LEFT_WHEEL_AIN2_CHANNEL, 0.0f);
        Set_PWM_DutyCycle(LEFT_WHEEL_AIN1_CHANNEL, abs_duty);
    } else {
        Set_PWM_DutyCycle(LEFT_WHEEL_AIN1_CHANNEL, 0.0f);
        Set_PWM_DutyCycle(LEFT_WHEEL_AIN2_CHANNEL, abs_duty);
    }
}

/* 把带方向的占空比输出到右轮。
 * 正值对应正转，负值对应反转。 */
static void MotorSpeedLoop_ApplyRightWheelDuty(float signed_duty_percent)
{
    /*
     * 小车装配完成后，右轮硬件正向与整车前进方向相反。
     * 这里先把右轮控制量反相，再按驱动板引脚映射输出，
     * 这样外层依然可以保持“正速度 = 小车前进”的统一语义。
     */
    signed_duty_percent *= RIGHT_WHEEL_DIRECTION_SIGN;

    float abs_duty = MotorPWM_ClampDutyPercent(MotorSpeedLoop_AbsFloat(signed_duty_percent));

    if (abs_duty < 0.01f) {
        MotorSpeedLoop_StopRightWheel();
        return;
    }

    if (signed_duty_percent > 0.0f) {
        Set_PWM_DutyCycle(RIGHT_WHEEL_BIN2_CHANNEL, 0.0f);
        Set_PWM_DutyCycle(RIGHT_WHEEL_BIN1_CHANNEL, abs_duty);
    } else {
        Set_PWM_DutyCycle(RIGHT_WHEEL_BIN1_CHANNEL, 0.0f);
        Set_PWM_DutyCycle(RIGHT_WHEEL_BIN2_CHANNEL, abs_duty);
    }
}

void MotorPWM_SetLeftWheelSignedDutyPercent(float signed_duty_percent)
{
    float abs_duty = MotorPWM_ClampDutyPercent(
        MotorSpeedLoop_AbsFloat(signed_duty_percent));
    float clamped_duty = (signed_duty_percent < 0.0f) ?
        -abs_duty : abs_duty;

    g_left_speed_loop.output_duty_percent = clamped_duty;
    MotorSpeedLoop_ApplyLeftWheelDuty(clamped_duty);
}

void MotorPWM_SetRightWheelSignedDutyPercent(float signed_duty_percent)
{
    float abs_duty = MotorPWM_ClampDutyPercent(
        MotorSpeedLoop_AbsFloat(signed_duty_percent));
    float clamped_duty = (signed_duty_percent < 0.0f) ?
        -abs_duty : abs_duty;

    g_right_speed_loop.output_duty_percent = clamped_duty;
    MotorSpeedLoop_ApplyRightWheelDuty(clamped_duty);
}

/*
 * 设置指定 PWM 通道的占空比。
 * 当占空比为 0 时，直接切成 GPIO 低电平，避免残余 PWM 让电机误动作。
 */
void Set_PWM_DutyCycle(uint8_t channel, float duty_percent)
{
    uint32_t compare_value;

    duty_percent = MotorPWM_ClampDutyPercent(duty_percent);

    if (channel >= MOTOR_PWM_CHANNEL_COUNT) {
        return;
    }

    if (duty_percent < 0.01f) {
        MotorPWM_SetChannelLow(channel);
        return;
    }

    compare_value = MotorPWM_DutyPercentToCompareValue(duty_percent);
    MotorPWM_SetChannelPwmMode(channel);
    MotorPWM_SetChannelCompareValue(channel, compare_value);
}

void MotorPWM_InitDefaults(void)
{
    /*
     * 这些是电机和驱动板相关参数，不属于按键职责。
     * 默认值来自实测：死区约 56%，启动助推当前关闭。
     */
    MotorPWM_SetDeadzoneDutyPercent(56.0f);
    MotorPWM_SetStartBoostDutyPercent(60.0f);
    MotorPWM_SetStartBoostTimeMs(0U);
    MotorPWM_StopAllChannels();
}

void MotorPWM_StopAllChannels(void)
{
    /*
     * 这里把 PWM_0 的四路输出全部切成 GPIO 低电平。
     * 这样无论左轮还是右轮，在上电初始化和复位阶段都会先被明确关闭，
     * 能显著减少驱动模块因瞬态状态不确定而导致的“电机蹿一下”现象。
     */
    Set_PWM_DutyCycle(0U, 0.0f);
    Set_PWM_DutyCycle(1U, 0.0f);
    Set_PWM_DutyCycle(2U, 0.0f);
    Set_PWM_DutyCycle(3U, 0.0f);
}

void MotorPWM_BrakeAllChannels(void)
{
    /*
     * AT8236 的 IN1=IN2=1 对应低侧慢衰减刹车。
     * 该接口只应短时使用，静止后应调用 StopAllChannels 恢复全低。
     */
    MotorPWM_SetChannelHigh(LEFT_WHEEL_AIN1_CHANNEL);
    MotorPWM_SetChannelHigh(LEFT_WHEEL_AIN2_CHANNEL);
    MotorPWM_SetChannelHigh(RIGHT_WHEEL_BIN1_CHANNEL);
    MotorPWM_SetChannelHigh(RIGHT_WHEEL_BIN2_CHANNEL);
}

void MotorPWM_SetDeadzoneDutyPercent(float duty_percent)
{
    g_motor_deadzone_duty_percent = MotorPWM_ClampDutyPercent(duty_percent);
}

float MotorPWM_GetDeadzoneDutyPercent(void)
{
    return g_motor_deadzone_duty_percent;
}

void MotorPWM_SetStartBoostDutyPercent(float duty_percent)
{
    g_motor_start_boost_duty_percent = MotorPWM_ClampDutyPercent(duty_percent);
}

float MotorPWM_GetStartBoostDutyPercent(void)
{
    return g_motor_start_boost_duty_percent;
}

void MotorPWM_SetStartBoostTimeMs(uint32_t boost_time_ms)
{
    g_motor_start_boost_time_ms = boost_time_ms;
}

uint32_t MotorPWM_GetStartBoostTimeMs(void)
{
    return g_motor_start_boost_time_ms;
}

/*
 * 对目标占空比做死区补偿。
 * 当目标值已经高于死区，或者目标本身就是 0 时，会直接原样返回。
 */
float MotorPWM_ApplyDeadzoneCompensation(float duty_percent)
{
    float clamped_duty = MotorPWM_ClampDutyPercent(duty_percent);
    float deadzone_duty = MotorPWM_ClampDutyPercent(g_motor_deadzone_duty_percent);

    if ((clamped_duty > 0.0f) && (clamped_duty < deadzone_duty)) {
        return deadzone_duty;
    }

    return clamped_duty;
}

/*
 * 根据维持占空比给出建议的启动脉冲占空比。
 * 返回值至少不会低于维持占空比，也不会低于启动脉冲配置值。
 */
float MotorPWM_GetStartupDutyPercent(float hold_duty_percent)
{
    float compensated_hold_duty = MotorPWM_ApplyDeadzoneCompensation(hold_duty_percent);
    float boost_duty = MotorPWM_ApplyDeadzoneCompensation(g_motor_start_boost_duty_percent);

    if (compensated_hold_duty <= 0.0f) {
        return 0.0f;
    }

    if (boost_duty < compensated_hold_duty) {
        boost_duty = compensated_hold_duty;
    }

    return boost_duty;
}

void MotorSpeed_SetSampleTimeMs(uint32_t sample_time_ms)
{
    if (sample_time_ms == 0U) {
        return;
    }

    g_motor_speed_sample_time_ms = sample_time_ms;
}

uint32_t MotorSpeed_GetSampleTimeMs(void)
{
    return g_motor_speed_sample_time_ms;
}

void MotorSpeed_Reset(void)
{
    uint8_t index;

    g_left_encoder_last_count = encoder_left_count;
    g_right_encoder_last_count = encoder_right_count;
    g_left_wheel_rpm = 0.0f;
    g_right_wheel_rpm = 0.0f;
    g_left_wheel_rpm_raw = 0.0f;
    g_right_wheel_rpm_raw = 0.0f;
    g_left_wheel_linear_speed_mm_per_sec = 0.0f;
    g_right_wheel_linear_speed_mm_per_sec = 0.0f;
    g_left_wheel_rpm_sum = 0.0f;
    g_right_wheel_rpm_sum = 0.0f;
    g_speed_filter_count = 0U;
    g_speed_filter_index = 0U;

    for (index = 0U; index < MOTOR_SPEED_FILTER_WINDOW_SIZE; index++) {
        g_left_wheel_rpm_history[index] = 0.0f;
        g_right_wheel_rpm_history[index] = 0.0f;
    }
}

void MotorSpeed_GetWheelRPM(float *left_rpm, float *right_rpm)
{
    if (left_rpm != NULL) {
        *left_rpm = g_left_wheel_rpm;
    }

    if (right_rpm != NULL) {
        *right_rpm = g_right_wheel_rpm;
    }
}

/*
 * 根据编码器计数差值计算当前车轮转速。
 * elapsed_ms 传入本次实际经过的时间，通常就是 tick_ms - last_speed_tick。
 */
uint8_t MotorSpeed_Update(uint32_t elapsed_ms)
{
    long left_count_now;
    long right_count_now;
    long left_count_delta;
    long right_count_delta;
    float left_wheel_rpm_raw;
    float right_wheel_rpm_raw;
    uint32_t sample_time_ms = elapsed_ms;

    if (sample_time_ms == 0U) {
        sample_time_ms = g_motor_speed_sample_time_ms;
    }

    if (sample_time_ms == 0U) {
        return 0U;
    }

    left_count_now = encoder_left_count;
    right_count_now = encoder_right_count;

    left_count_delta = left_count_now - g_left_encoder_last_count;
    right_count_delta = right_count_now - g_right_encoder_last_count;

    g_left_encoder_last_count = left_count_now;
    g_right_encoder_last_count = right_count_now;

    left_wheel_rpm_raw = LEFT_WHEEL_DIRECTION_SIGN *
                         MotorSpeed_CountDeltaToWheelRPM(left_count_delta, sample_time_ms);
    right_wheel_rpm_raw = RIGHT_WHEEL_DIRECTION_SIGN *
                          MotorSpeed_CountDeltaToWheelRPM(right_count_delta, sample_time_ms);
    g_left_wheel_rpm_raw = left_wheel_rpm_raw;
    g_right_wheel_rpm_raw = right_wheel_rpm_raw;

    if (g_speed_filter_count < MOTOR_SPEED_FILTER_WINDOW_SIZE) {
        g_speed_filter_count++;
    }

    g_left_wheel_rpm = MotorSpeed_UpdateAverage(left_wheel_rpm_raw,
                                                g_left_wheel_rpm_history,
                                                &g_left_wheel_rpm_sum,
                                                g_speed_filter_index,
                                                g_speed_filter_count);
    g_right_wheel_rpm = MotorSpeed_UpdateAverage(right_wheel_rpm_raw,
                                                 g_right_wheel_rpm_history,
                                                 &g_right_wheel_rpm_sum,
                                                 g_speed_filter_index,
                                                 g_speed_filter_count);

    g_speed_filter_index++;
    if (g_speed_filter_index >= MOTOR_SPEED_FILTER_WINDOW_SIZE) {
        g_speed_filter_index = 0U;
    }

    g_left_wheel_linear_speed_mm_per_sec = MotorSpeed_WheelRPMToLinearSpeedMmPerSec(g_left_wheel_rpm);
    g_right_wheel_linear_speed_mm_per_sec = MotorSpeed_WheelRPMToLinearSpeedMmPerSec(g_right_wheel_rpm);

    return 1U;
}

float MotorSpeed_GetWheelDiameterMm(void)
{
    return MOTOR_WHEEL_DIAMETER_MM;
}

float MotorSpeed_GetWheelCountsPerRevolution(void)
{
    return MOTOR_WHEEL_COUNTS_PER_REV;
}

float MotorSpeed_GetLeftWheelRPM(void)
{
    return g_left_wheel_rpm;
}

float MotorSpeed_GetRightWheelRPM(void)
{
    return g_right_wheel_rpm;
}

float MotorSpeed_GetLeftWheelLinearSpeedMmPerSec(void)
{
    return g_left_wheel_linear_speed_mm_per_sec;
}

float MotorSpeed_GetRightWheelLinearSpeedMmPerSec(void)
{
    return g_right_wheel_linear_speed_mm_per_sec;
}

void MotorSpeed_GetWheelLinearSpeedMmPerSec(float *left_mm_per_sec, float *right_mm_per_sec)
{
    if (left_mm_per_sec != NULL) {
        *left_mm_per_sec = g_left_wheel_linear_speed_mm_per_sec;
    }

    if (right_mm_per_sec != NULL) {
        *right_mm_per_sec = g_right_wheel_linear_speed_mm_per_sec;
    }
}

int32_t MotorSpeed_GetLeftEncoderTotalCount(void)
{
    return (int32_t) encoder_left_count;
}

int32_t MotorSpeed_GetRightEncoderTotalCount(void)
{
    return (int32_t) encoder_right_count;
}

int32_t MotorSpeed_GetLeftEncoderForwardCount(void)
{
    return (int32_t) (LEFT_WHEEL_DIRECTION_SIGN * (float) encoder_left_count);
}

int32_t MotorSpeed_GetRightEncoderForwardCount(void)
{
    return (int32_t) (RIGHT_WHEEL_DIRECTION_SIGN * (float) encoder_right_count);
}

void MotorSpeedLoop_SetLeftWheelTargetRPM(float target_rpm)
{
    g_left_speed_loop.target_rpm = MotorSpeedLoop_ClampTargetRPM(target_rpm);

    if (MotorSpeedLoop_AbsFloat(g_left_speed_loop.target_rpm) < 0.01f) {
        MotorSpeedLoop_ResetLeftWheelController();
        MotorSpeedLoop_StopLeftWheel();
    }
}

void MotorSpeedLoop_SetRightWheelTargetRPM(float target_rpm)
{
    g_right_speed_loop.target_rpm = MotorSpeedLoop_ClampTargetRPM(target_rpm);

    if (MotorSpeedLoop_AbsFloat(g_right_speed_loop.target_rpm) < 0.01f) {
        MotorSpeedLoop_ResetRightWheelController();
        MotorSpeedLoop_StopRightWheel();
    }
}

float MotorSpeedLoop_GetLeftWheelTargetRPM(void)
{
    return g_left_speed_loop.target_rpm;
}

float MotorSpeedLoop_GetRightWheelTargetRPM(void)
{
    return g_right_speed_loop.target_rpm;
}

void MotorSpeedLoop_SetLeftWheelPI(float kp, float ki)
{
    g_left_speed_loop.kp = kp;
    g_left_speed_loop.ki = ki;
}

void MotorSpeedLoop_SetRightWheelPI(float kp, float ki)
{
    g_right_speed_loop.kp = kp;
    g_right_speed_loop.ki = ki;
}

void MotorSpeedLoop_GetLeftWheelPI(float *kp, float *ki)
{
    if (kp != NULL) {
        *kp = g_left_speed_loop.kp;
    }

    if (ki != NULL) {
        *ki = g_left_speed_loop.ki;
    }
}

void MotorSpeedLoop_GetRightWheelPI(float *kp, float *ki)
{
    if (kp != NULL) {
        *kp = g_right_speed_loop.kp;
    }

    if (ki != NULL) {
        *ki = g_right_speed_loop.ki;
    }
}

static void MotorSpeedLoop_ResetController(MotorSpeedLoopState *loop)
{
    if (loop == NULL) {
        return;
    }

    loop->integral = 0.0f;
    loop->output_duty_percent = 0.0f;
    loop->last_direction = 0;
    loop->start_boost_active = 0U;
    loop->start_boost_begin_ms = 0UL;
}

static void MotorSpeedLoop_EnableWheel(MotorSpeedLoopState *loop,
                                       uint8_t enable,
                                       MotorSpeedLoopStopOutput stop_output)
{
    if (loop == NULL) {
        return;
    }

    loop->enabled = (enable != 0U) ? 1U : 0U;
    MotorSpeedLoop_ResetController(loop);

    if ((loop->enabled == 0U) && (stop_output != NULL)) {
        stop_output();
    }
}

void MotorSpeedLoop_EnableLeftWheel(uint8_t enable)
{
    MotorSpeedLoop_EnableWheel(&g_left_speed_loop, enable, MotorSpeedLoop_StopLeftWheel);
}

void MotorSpeedLoop_EnableRightWheel(uint8_t enable)
{
    MotorSpeedLoop_EnableWheel(&g_right_speed_loop, enable, MotorSpeedLoop_StopRightWheel);
}

uint8_t MotorSpeedLoop_IsLeftWheelEnabled(void)
{
    return g_left_speed_loop.enabled;
}

uint8_t MotorSpeedLoop_IsRightWheelEnabled(void)
{
    return g_right_speed_loop.enabled;
}

void MotorSpeedLoop_ResetLeftWheelController(void)
{
    MotorSpeedLoop_ResetController(&g_left_speed_loop);
}

void MotorSpeedLoop_ResetRightWheelController(void)
{
    MotorSpeedLoop_ResetController(&g_right_speed_loop);
}

static uint8_t MotorSpeedLoop_UpdateWheel(MotorSpeedLoopState *loop,
                                          float feedback_rpm,
                                          uint32_t elapsed_ms,
                                          MotorSpeedLoopDutyOutput apply_output,
                                          MotorSpeedLoopStopOutput stop_output)
{
    float error_rpm;
    float proportional_output;
    float controller_output;
    float compensated_output;
    float sample_time_sec;
    float target_direction_float;
    int8_t controller_direction;
    int8_t compensated_direction;

    if ((loop == NULL) || (apply_output == NULL) || (elapsed_ms == 0U)) {
        return 0U;
    }

    if ((loop->enabled == 0U) ||
        (MotorSpeedLoop_AbsFloat(loop->target_rpm) < 0.01f)) {
        MotorSpeedLoop_ResetController(loop);
        if (stop_output != NULL) {
            stop_output();
        }
        return 0U;
    }

    if (MotorSpeedLoop_ShouldUseOpenLoop(loop->target_rpm) != 0U) {
        loop->integral = 0.0f;
        loop->start_boost_active = 0U;
        compensated_output = MotorSpeedLoop_GetOpenLoopDuty(loop->target_rpm);
        loop->output_duty_percent = compensated_output;
        loop->last_direction = MotorSpeedLoop_GetDirection(compensated_output);
        apply_output(loop->output_duty_percent);
        return 1U;
    }

    sample_time_sec = (float) elapsed_ms / 1000.0f;
    error_rpm = loop->target_rpm - feedback_rpm;
    target_direction_float = (float) MotorSpeedLoop_GetDirection(loop->target_rpm);

    /*
     * 积分项允许正负累加：
     * 1. 实际速度低于目标时，积分沿目标方向增加，补足输出不足。
     * 2. 实际速度高于目标时，积分反向累加，降低输出占空比。
     * 最终 PWM 输出仍会按目标方向限幅，避免目标正转时直接打反转。
     */
    if (((target_direction_float * error_rpm) > 1.0f) ||
        ((target_direction_float * error_rpm) < -1.0f)) {
        loop->integral += (loop->ki * error_rpm * sample_time_sec);
    } else {
        loop->integral *= 0.95f;
    }

    loop->integral = MotorSpeedLoop_ClampSignedDuty(loop->integral);

    proportional_output = loop->kp * error_rpm;
    /* 高速段总输出 = PI 修正量，死区和固定底座在最终占空比阶段统一补偿。 */
    controller_output = proportional_output + loop->integral;
    controller_output = MotorSpeedLoop_ClampSignedDuty(controller_output);
    controller_output = MotorSpeedLoop_ClampToTargetDirection(loop->target_rpm, controller_output);
    controller_direction = MotorSpeedLoop_GetDirection(controller_output);

    if ((controller_direction != 0) &&
        (controller_direction != loop->last_direction) &&
        (MotorPWM_GetStartBoostTimeMs() > 0U)) {
        loop->start_boost_active = 1U;
        loop->start_boost_begin_ms = tick_ms;
    }

    compensated_output = MotorSpeedLoop_ApplySignedDeadzoneCompensation(controller_output);
    compensated_direction = MotorSpeedLoop_GetDirection(compensated_output);

    if (loop->start_boost_active != 0U) {
        if ((tick_ms - loop->start_boost_begin_ms) < MotorPWM_GetStartBoostTimeMs()) {
            float boost_duty = MotorPWM_GetStartupDutyPercent(MotorSpeedLoop_AbsFloat(compensated_output));

            if (compensated_direction > 0) {
                compensated_output = boost_duty;
            } else if (compensated_direction < 0) {
                compensated_output = (-boost_duty);
            }
        } else {
            loop->start_boost_active = 0U;
        }
    }

    loop->output_duty_percent = compensated_output;
    loop->last_direction = MotorSpeedLoop_GetDirection(compensated_output);
    apply_output(loop->output_duty_percent);
    return 1U;
}

uint8_t MotorSpeedLoop_UpdateLeftWheel(uint32_t elapsed_ms)
{
    return MotorSpeedLoop_UpdateWheel(&g_left_speed_loop,
                                      g_left_wheel_rpm_raw,
                                      elapsed_ms,
                                      MotorSpeedLoop_ApplyLeftWheelDuty,
                                      MotorSpeedLoop_StopLeftWheel);
}

uint8_t MotorSpeedLoop_UpdateRightWheel(uint32_t elapsed_ms)
{
    return MotorSpeedLoop_UpdateWheel(&g_right_speed_loop,
                                      g_right_wheel_rpm_raw,
                                      elapsed_ms,
                                      MotorSpeedLoop_ApplyRightWheelDuty,
                                      MotorSpeedLoop_StopRightWheel);
}

float MotorSpeedLoop_GetLeftWheelDutyPercent(void)
{
    return g_left_speed_loop.output_duty_percent;
}

float MotorSpeedLoop_GetRightWheelDutyPercent(void)
{
    return g_right_speed_loop.output_duty_percent;
}
