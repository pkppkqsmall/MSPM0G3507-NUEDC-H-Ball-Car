#include "Sensor.h"

#include <stddef.h>

#include "ti_msp_dl_config.h"

#define SENSOR_SERIAL_CLK_LOW_US     (2U)
#define SENSOR_SERIAL_CLK_HIGH_US    (5U)
#define SENSOR_CYCLES_PER_US         (CPUCLK_FREQ / 1000000U)

static uint8_t g_sensor_initialized;
static uint8_t g_sensor_last_value;
static SensorError g_sensor_last_error = SENSOR_ERROR_NOT_INITIALIZED;

static void Sensor_DelayUs(uint32_t delay_us)
{
    delay_cycles(SENSOR_CYCLES_PER_US * delay_us);
}

/*
 * 灰度 KEY 等效为一个接地按键。GPIO 只在“按下”时输出低电平，
 * “松开”时切回高阻输入，绝不主动向模块 KEY 输出高电平。
 */
uint8_t Sensor_Init(void)
{
    /*
     * CLK 为主控推挽输出，DAT 为 3.3V 上拉输入。
     * KEY 的输出锁存值先清零，再保持高阻释放状态。
     */
    DL_GPIO_clearPins(Sensor_PORT, Sensor_CLK_PIN | Sensor_KEY_PIN);
    DL_GPIO_enableOutput(Sensor_PORT, Sensor_CLK_PIN);
    DL_GPIO_disableOutput(Sensor_PORT, Sensor_DAT_PIN | Sensor_KEY_PIN);

    g_sensor_initialized = 1U;
    g_sensor_last_value = 0U;
    g_sensor_last_error = SENSOR_ERROR_NONE;
    return 1U;
}

uint8_t Sensor_Read_Grayscale(void)
{
    uint8_t bit_index;
    uint8_t raw_value = 0U;

    if (g_sensor_initialized == 0U) {
        g_sensor_last_error = SENSOR_ERROR_NOT_INITIALIZED;
        return g_sensor_last_value;
    }

    /*
     * 官方串行时序为低电平读取、高电平更新下一位。
     * 每个高电平至少保持 5 us，8 个时钟需在 1 ms 内完成。
     */
    for (bit_index = 0U; bit_index < SENSOR_CHANNEL_COUNT; bit_index++) {
        DL_GPIO_clearPins(Sensor_PORT, Sensor_CLK_PIN);
        Sensor_DelayUs(SENSOR_SERIAL_CLK_LOW_US);

        if (DL_GPIO_readPins(Sensor_PORT, Sensor_DAT_PIN) != 0U) {
            raw_value |= (uint8_t) (1U << bit_index);
        }

        DL_GPIO_setPins(Sensor_PORT, Sensor_CLK_PIN);
        Sensor_DelayUs(SENSOR_SERIAL_CLK_HIGH_US);
    }

    /* 模块输出白底=1、黑线=0，应用层继续保持黑线=1、白底=0。 */
    g_sensor_last_value = (uint8_t) (~raw_value);
    g_sensor_last_error = SENSOR_ERROR_NONE;
    return g_sensor_last_value;
}

uint8_t Sensor_Read_Analog(uint8_t values[SENSOR_CHANNEL_COUNT])
{
    if (values == NULL) {
        g_sensor_last_error = SENSOR_ERROR_INVALID_ARGUMENT;
        return 0U;
    }

    g_sensor_last_error = SENSOR_ERROR_UNSUPPORTED;
    return 0U;
}

uint8_t Sensor_Write_Calibration(
    const uint8_t black_values[SENSOR_CHANNEL_COUNT],
    const uint8_t white_values[SENSOR_CHANNEL_COUNT])
{
    if ((black_values == NULL) || (white_values == NULL)) {
        g_sensor_last_error = SENSOR_ERROR_INVALID_ARGUMENT;
        return 0U;
    }

    g_sensor_last_error = SENSOR_ERROR_UNSUPPORTED;
    return 0U;
}

void Sensor_SetCalibrationKeyPressed(uint8_t pressed)
{
    if (pressed != 0U) {
        DL_GPIO_enableOutput(Sensor_PORT, Sensor_KEY_PIN);
    } else {
        DL_GPIO_disableOutput(Sensor_PORT, Sensor_KEY_PIN);
    }
}

uint8_t Sensor_IsReady(void)
{
    return g_sensor_initialized;
}

SensorError Sensor_GetLastError(void)
{
    return g_sensor_last_error;
}
