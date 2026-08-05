#ifndef SENSOR_H_
#define SENSOR_H_

#include <stdint.h>

#define SENSOR_CHANNEL_COUNT (8U)

typedef enum {
    SENSOR_ERROR_NONE = 0,
    SENSOR_ERROR_NOT_INITIALIZED,
    SENSOR_ERROR_BUS_STUCK,
    SENSOR_ERROR_NO_ACK,
    SENSOR_ERROR_BAD_PING,
    SENSOR_ERROR_INVALID_ARGUMENT,
    SENSOR_ERROR_UNSUPPORTED
} SensorError;

/* 初始化 PA15/PA16 串行 CLK/DAT 与 PA13 灰度校准 KEY。 */
uint8_t Sensor_Init(void);

/* 通过串行 CLK/DAT 读取 8 路数字量，保持“1=黑线，0=白底”。 */
uint8_t Sensor_Read_Grayscale(void);

/* 串行模式不支持模拟量，调用后返回 0。 */
uint8_t Sensor_Read_Analog(uint8_t values[SENSOR_CHANNEL_COUNT]);

/* 串行模式不能写校准值，调用后返回 0。 */
uint8_t Sensor_Write_Calibration(
    const uint8_t black_values[SENSOR_CHANNEL_COUNT],
    const uint8_t white_values[SENSOR_CHANNEL_COUNT]);

/* 模拟灰度模块 KEY：非零表示按下接地，0 表示高阻释放。 */
void Sensor_SetCalibrationKeyPressed(uint8_t pressed);

/* 查询串行接口是否已经初始化。 */
uint8_t Sensor_IsReady(void);

/* 获取最近一次驱动错误。 */
SensorError Sensor_GetLastError(void);

#endif /* SENSOR_H_ */
