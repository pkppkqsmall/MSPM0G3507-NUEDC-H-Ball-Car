#ifndef UART_H_
#define UART_H_

#include "ti_msp_dl_config.h"

/*
 * 保留原来的串口环形缓冲，并为蓝牙测试新增一份“按行接收”缓冲区。
 * 作用：
 * 1. 中断里先收字节，主循环里再处理，避免阻塞中断。
 * 2. 后续可直接基于一整行字符串扩展 PID、目标速度、STOP 等命令解析。
 */
#define UART_RX_BUFFER_SIZE          128U
#define UART_BT_LINE_BUFFER_SIZE      64U

/* 蓝牙解析成功后，PID 参数会保存在这个结构体里。 */
typedef struct {
    float kp;
    float ki;
    float kd;
} PID_Params;

extern volatile PID_Params g_pid_params;

void UART_InitInterrupt(void);
void UART_IRQHandler(void);

void UART_SendByte(uint8_t data);
void UART_SendBuffer(const uint8_t *data, uint16_t length);
void UART_SendString(const char *str);
uint8_t UART_ReadByte(uint8_t *data);

uint16_t UART_GetRxCount(void);
uint8_t UART_HasRxOverflow(void);
void UART_ClearRxOverflow(void);

void UART_Printf(const char *format, ...);

/*
 * 蓝牙收发接口。
 * 作用：
 * 1. 初始化蓝牙串口测试环境
 * 2. 在主循环中处理接收数据
 * 3. 发送测速数据到 VOFA+ FireWater 上位机
 * 4. 为后续按行协议解析提供统一入口
 */
void UART_BluetoothInit(void);
void UART_BluetoothTask(void);
/*
 * 普通上位机显示用：按 VOFA+ FireWater 格式发送左右轮速度。
 *   speed:left_rpm,right_rpm\n
 * 其中 speed 是通道名前缀，换行符用于告诉 VOFA 当前帧结束。
 */
void UART_BluetoothSendVofaWheelSpeed(float left_speed, float right_speed);
void UART_BluetoothSendWheelSpeed(float left_speed, float right_speed);
uint8_t UART_BluetoothReadLine(char *buffer, uint16_t max_length);
uint8_t UART_BluetoothHasNewLine(void);
uint16_t UART_BluetoothGetLastLineLength(void);
uint32_t UART_BluetoothGetRxByteCount(void);
uint32_t UART_BluetoothGetRxFrameCount(void);

void PID_SetParameters(float kp, float ki, float kd);
void PID_GetParameters(PID_Params *params);
uint8_t UART_BluetoothHandlePidCommand(const char *command);

/*
 * 处理蓝牙下发的目标速度命令。
 * 支持格式：
 * 1. TARGET,24        -> 左右轮都设为 24RPM
 * 2. TARGET,24,22     -> 左轮 24RPM，右轮 22RPM
 * 3. TARGET?          -> 查询当前保存的目标速度
 */
uint8_t UART_BluetoothHandleTargetCommand(const char *command);

/*
 * 处理左右轮分开调参命令。
 * 支持格式：
 * 1. LPID,0.12,0.05   -> 单独设置左轮速度环 PI
 * 2. RPID,0.12,0.06   -> 单独设置右轮速度环 PI
 * 3. LPID? / RPID?    -> 查询当前参数
 */
uint8_t UART_BluetoothHandleWheelTuneCommand(const char *command);

/*
 * 处理蓝牙 STOP 命令。
 * 支持格式：
 * 1. STOP -> 立即停止小车运行
 */
uint8_t UART_BluetoothHandleStopCommand(const char *command);
uint8_t UART_BluetoothHandleStartCommand(const char *command);

/*
 * 查询编码器累计计数。
 * 支持格式：
 * 1. ENC? -> 返回 RawL/RawR 原始计数，以及 FwdL/FwdR 按前进方向修正后的计数。
 */
uint8_t UART_BluetoothHandleEncoderQueryCommand(const char *command);

/*
 * 查询小车当前调试状态。
 * 支持格式：
 * 1. CAR? -> 返回运行状态、目标速度、实际速度、PI 和当前占空比。
 */
uint8_t UART_BluetoothHandleCarStatusCommand(const char *command);

/*
 * 按 llm-pid-tuner 要求的 CSV 格式上报左轮速度环数据。
 * 格式: timestamp_ms,setpoint,input,pwm,error,p,i,d
 */
uint8_t UART_LlmTunerIsEnabled(void);
void UART_LlmTunerSetEnabled(uint8_t enable);
/*
 * AI 自动调参用：按 llm-pid-tuner 需要的 CSV 格式发送一帧左轮速度环数据。
 *   timestamp_ms,setpoint,input,pwm,error,p,i,d
 * 这个格式不要给 VOFA 画图用，主要给 Python 调参脚本解析。
 */
void UART_BluetoothSendAiTuneSpeedCsv(void);
void UART_LlmTunerSendCsv(void);
uint8_t UART_BluetoothHandleLlmTunerCommand(const char *command);

/*
 * 处理 llm-pid-tuner 下发的 SET 命令。
 * 支持格式: SET P:kp I:ki D:kd
 */
uint8_t UART_BluetoothHandleSetCommand(const char *command);

#endif
