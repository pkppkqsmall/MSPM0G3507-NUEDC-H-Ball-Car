#ifndef BNO085_H_
#define BNO085_H_

#include <stdint.h>

/* 与原 MPU6050 接口保持一致，减少 OLED 和角度测试模块的改动。 */
extern volatile float pitch;
extern volatile float roll;
extern volatile float yaw;

uint8_t BNO085_Init(void);
void BNO085_NotifyDataReadyFromIsr(void);
uint8_t BNO085_UpdateIfDataReady(void);
uint8_t BNO085_IsReady(void);
uint8_t BNO085_GetAccuracy(void);
int BNO085_GetLastError(void);

#endif /* BNO085_H_ */
