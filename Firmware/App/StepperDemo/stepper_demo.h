#ifndef APP_STEPPER_DEMO_H_
#define APP_STEPPER_DEMO_H_

/* 初始化步进电机、OLED、BNO085、灰度串行接口和 B21 校准。 */
void StepperDemo_Init(void);

/* 主循环反复调用：更新传感器、校准流程、步进动作阶段和 OLED。 */
void StepperDemo_RunStep(void);

#endif /* APP_STEPPER_DEMO_H_ */
