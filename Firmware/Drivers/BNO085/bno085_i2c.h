#ifndef BNO085_I2C_H_
#define BNO085_I2C_H_

#include <stdint.h>

#include "sh2/sh2_hal.h"

/*
 * BNO085 默认 I2C 地址是 0x4A，SA0 拉高时通常为 0x4B。
 * 上层初始化会依次尝试两个地址。
 */
#define BNO085_I2C_ADDRESS_DEFAULT      (0x4AU)
#define BNO085_I2C_ADDRESS_ALTERNATE    (0x4BU)

sh2_Hal_t *BNO085_I2cHal_Get(uint8_t address);
void BNO085_I2cHal_NotifyDataReadyFromIsr(void);
uint8_t BNO085_I2cHal_HasPendingRead(void);

#endif /* BNO085_I2C_H_ */
