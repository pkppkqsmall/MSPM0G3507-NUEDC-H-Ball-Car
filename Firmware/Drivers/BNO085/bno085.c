#include "bno085.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "bno085_i2c.h"
#include "clock.h"
#include "sh2/sh2.h"
#include "sh2/sh2_SensorValue.h"
#include "sh2/sh2_err.h"

#define BNO085_REPORT_INTERVAL_US       (20000U)
#define BNO085_SERVICE_LIMIT            (4U)
#define BNO085_RAD_TO_DEG               (57.2957795f)

/* 若模块安装方向与期望相反，只需把该系数改为 -1.0f。 */
#define BNO085_YAW_DIRECTION            (1.0f)

volatile float pitch;
volatile float roll;
volatile float yaw;

static uint8_t g_bno085_ready;
static uint8_t g_bno085_accuracy;
static uint8_t g_bno085_sample_updated;
static uint8_t g_bno085_reconfigure_requested;
static int g_bno085_last_error;

static float BNO085_NormalizeAngle(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static void BNO085_UpdateEuler(float q_real,
                               float q_i,
                               float q_j,
                               float q_k)
{
    float roll_numerator;
    float roll_denominator;
    float pitch_sine;
    float yaw_numerator;
    float yaw_denominator;
    float next_pitch;
    float next_roll;
    float next_yaw;

    roll_numerator = 2.0f * ((q_real * q_i) + (q_j * q_k));
    roll_denominator = 1.0f - (2.0f * ((q_i * q_i) + (q_j * q_j)));
    next_roll = atan2f(roll_numerator, roll_denominator) * BNO085_RAD_TO_DEG;

    pitch_sine = 2.0f * ((q_real * q_j) - (q_k * q_i));
    if (pitch_sine > 1.0f) {
        pitch_sine = 1.0f;
    } else if (pitch_sine < -1.0f) {
        pitch_sine = -1.0f;
    }
    next_pitch = asinf(pitch_sine) * BNO085_RAD_TO_DEG;

    yaw_numerator = 2.0f * ((q_real * q_k) + (q_i * q_j));
    yaw_denominator = 1.0f - (2.0f * ((q_j * q_j) + (q_k * q_k)));
    next_yaw = atan2f(yaw_numerator, yaw_denominator) * BNO085_RAD_TO_DEG;

    roll = next_roll;
    pitch = next_pitch;
    yaw = BNO085_NormalizeAngle(next_yaw * BNO085_YAW_DIRECTION);
}

static void BNO085_EventHandler(void *cookie, sh2_AsyncEvent_t *event)
{
    (void) cookie;

    if ((event != NULL) && (event->eventId == SH2_RESET)) {
        /* BNO085 复位后会丢失传感器报告配置，留给主循环重新配置。 */
        g_bno085_reconfigure_requested = 1U;
    }
}

static void BNO085_SensorHandler(void *cookie, sh2_SensorEvent_t *event)
{
    sh2_SensorValue_t value;

    (void) cookie;

    if ((event == NULL) ||
        (event->reportId != SH2_GAME_ROTATION_VECTOR) ||
        (sh2_decodeSensorEvent(&value, event) != SH2_OK)) {
        return;
    }

    g_bno085_accuracy = value.status & 0x03U;
    BNO085_UpdateEuler(value.un.gameRotationVector.real,
                       value.un.gameRotationVector.i,
                       value.un.gameRotationVector.j,
                       value.un.gameRotationVector.k);
    g_bno085_sample_updated = 1U;
}

static int BNO085_EnableGameRotationVector(void)
{
    sh2_SensorConfig_t config;

    memset(&config, 0, sizeof(config));
    config.reportInterval_us = BNO085_REPORT_INTERVAL_US;
    return sh2_setSensorConfig(SH2_GAME_ROTATION_VECTOR, &config);
}

static uint8_t BNO085_TryAddress(uint8_t address)
{
    int result;

    result = sh2_open(BNO085_I2cHal_Get(address),
                      BNO085_EventHandler,
                      NULL);
    if (result != SH2_OK) {
        g_bno085_last_error = result;
        return 0U;
    }

    result = sh2_setSensorCallback(BNO085_SensorHandler, NULL);
    if (result == SH2_OK) {
        result = BNO085_EnableGameRotationVector();
    }

    if (result != SH2_OK) {
        g_bno085_last_error = result;
        sh2_close();
        return 0U;
    }

    g_bno085_last_error = SH2_OK;
    g_bno085_reconfigure_requested = 0U;
    return 1U;
}

uint8_t BNO085_Init(void)
{
    static const uint8_t addresses[] = {
        BNO085_I2C_ADDRESS_DEFAULT,
        BNO085_I2C_ADDRESS_ALTERNATE
    };
    uint8_t attempt;

    pitch = 0.0f;
    roll = 0.0f;
    yaw = 0.0f;
    g_bno085_ready = 0U;
    g_bno085_accuracy = 0U;
    g_bno085_sample_updated = 0U;
    g_bno085_reconfigure_requested = 0U;
    g_bno085_last_error = SH2_ERR;

    /*
     * BNO085 与 MCU 同时上电时需要一点启动时间。此时 SysTick 已运行，
     * 延时不会影响尚未启动的电机控制。
     */
    mspm0_delay_ms(100U);

    /*
     * 每个地址最多尝试两轮。这样 BNO085 与 MCU 同时上电、第一轮尚未
     * 完成启动时，后续仍会回到正确地址，不会因为只尝试一次而误报失败。
     */
    for (attempt = 0U; attempt < 4U; attempt++) {
        if (BNO085_TryAddress(addresses[attempt % 2U]) != 0U) {
            g_bno085_ready = 1U;
            return 1U;
        }
        mspm0_delay_ms(50U);
    }

    return 0U;
}

void BNO085_NotifyDataReadyFromIsr(void)
{
    BNO085_I2cHal_NotifyDataReadyFromIsr();
}

uint8_t BNO085_UpdateIfDataReady(void)
{
    uint8_t service_count;
    uint8_t sample_updated;

    if (g_bno085_ready == 0U) {
        return 0U;
    }

    g_bno085_sample_updated = 0U;
    for (service_count = 0U;
         (service_count < BNO085_SERVICE_LIMIT) &&
             (BNO085_I2cHal_HasPendingRead() != 0U);
         service_count++) {
        sh2_service();
    }

    if (g_bno085_reconfigure_requested != 0U) {
        g_bno085_reconfigure_requested = 0U;
        g_bno085_last_error = BNO085_EnableGameRotationVector();
        if (g_bno085_last_error != SH2_OK) {
            g_bno085_ready = 0U;
        }
    }

    sample_updated = g_bno085_sample_updated;
    return sample_updated;
}

uint8_t BNO085_IsReady(void)
{
    return g_bno085_ready;
}

uint8_t BNO085_GetAccuracy(void)
{
    return g_bno085_accuracy;
}

int BNO085_GetLastError(void)
{
    return g_bno085_last_error;
}
