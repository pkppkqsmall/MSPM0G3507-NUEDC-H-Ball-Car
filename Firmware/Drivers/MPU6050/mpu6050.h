/*
 * SysConfig Configuration Steps:
 *   I2C:
 *     1. Add an I2C module.
 *     2. Name it as "I2C_MPU6050".
 *     3. Check the box "Enable Controller Mode".
 *     4. Set Standard Bus Speed to "Fast Mode (400kHz)". (optional)
 *     5. Set the pins according to your needs.
 *   GPIO:
 *     1. Add a GPIO module.
 *     2. Name the group as "GPIO_MPU6050".
 *     3. Name the pin as "PIN_MPU6050_INT".
 *     4. Set Direction to "Input".
 *     5. Set "Internal Resistor" to "Pull-Up Resistor".
 *     6. Check the box "Enable Interrupts".
 *     7. Set "Interrupt Priority" to "Level 3 - Lowest".
 *     8. Set "Trigger Polarity" to "Trigger on Falling Edge".
 *     9. Set the pin according to your needs.
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdint.h>

extern short gyro[3], accel[3];
/* pitch/roll/yaw 在主循环中更新，OLED/状态机读取，因此用 volatile 表明它们会异步变化。 */
extern volatile float pitch, roll, yaw;

uint8_t MPU6050_Init(void);
int Read_Quad(void);
/*
 * GPIO 中断里只调用这个函数置位标志。
 * 不在中断里读 I2C/DMP，避免长时间阻塞编码器中断。
 */
void MPU6050_NotifyDataReadyFromIsr(void);
/*
 * 主循环调用：如果 MPU6050 产生新数据，就读取 DMP FIFO 并更新 pitch/roll/yaw。
 */
uint8_t MPU6050_UpdateIfDataReady(void);

#endif  /* MPU6050_H_ */
