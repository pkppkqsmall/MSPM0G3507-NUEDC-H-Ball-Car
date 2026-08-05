#ifndef CAMERA_LINK_H_
#define CAMERA_LINK_H_

#include <stdint.h>

#define CAMERA_LINK_LINE_BUFFER_SIZE    (128U)

/* 初始化摄像头 BLE 从机使用的 UART2 接收链路。 */
void CameraLink_Init(void);

/* 在主循环中组装 ASCII 行，now_ms 用于记录最新一帧到达时间。 */
void CameraLink_Task(uint32_t now_ms);

/* 复制最近一条完整消息；尚未收到完整行时返回 0。 */
uint8_t CameraLink_CopyLatestLine(char *buffer, uint16_t buffer_size);

uint8_t CameraLink_HasFrame(void);
uint32_t CameraLink_GetRxByteCount(void);
uint32_t CameraLink_GetFrameCount(void);
uint32_t CameraLink_GetOverflowCount(void);
uint32_t CameraLink_GetLastFrameMs(void);

#endif /* CAMERA_LINK_H_ */
