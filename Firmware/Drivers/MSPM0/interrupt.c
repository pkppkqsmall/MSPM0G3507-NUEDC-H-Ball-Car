#include "ti_msp_dl_config.h"
#include "interrupt.h"
#include "clock.h"
#include "bno085.h"

void Interrupt_Init(void)
{
    /*
     * BNO085 与左右编码器都位于 GPIOB 时，SysConfig 会把它们合并为
     * GPIO_MULTIPLE_GPIOB_INT_IRQN，而不会再生成各自独立的 IRQ 宏。
     */
#if defined GPIO_MULTIPLE_GPIOB_INT_IRQN
    NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
#else
#if defined GPIO_BNO085_INT_IRQN
    NVIC_ClearPendingIRQ(GPIO_BNO085_INT_IRQN);
    NVIC_EnableIRQ(GPIO_BNO085_INT_IRQN);
#endif

#if defined Encoder_INT_IRQN
    NVIC_ClearPendingIRQ(Encoder_INT_IRQN);
    NVIC_EnableIRQ(Encoder_INT_IRQN);
#endif
#endif
}

void SysTick_Handler(void)
{
    tick_ms++;
}

#if defined UART_BNO08X_INST_IRQHandler
void UART_BNO08X_INST_IRQHandler(void)
{
    uint8_t checkSum = 0;
    extern uint8_t bno08x_dmaBuffer[19];

    DL_DMA_disableChannel(DMA, DMA_BNO08X_CHAN_ID);
    uint8_t rxSize = 18 - DL_DMA_getTransferSize(DMA, DMA_BNO08X_CHAN_ID);

    if(DL_UART_isRXFIFOEmpty(UART_BNO08X_INST) == false)
        bno08x_dmaBuffer[rxSize++] = DL_UART_receiveData(UART_BNO08X_INST);

    for(int i=2; i<=14; i++)
        checkSum += bno08x_dmaBuffer[i];

    if((rxSize == 19) && (bno08x_dmaBuffer[0] == 0xAA) && (bno08x_dmaBuffer[1] == 0xAA) && (checkSum == bno08x_dmaBuffer[18]))
    {
        bno08x_data.index = bno08x_dmaBuffer[2];
        bno08x_data.yaw = (int16_t)((bno08x_dmaBuffer[4]<<8)|bno08x_dmaBuffer[3]) / 100.0;
        bno08x_data.pitch = (int16_t)((bno08x_dmaBuffer[6]<<8)|bno08x_dmaBuffer[5]) / 100.0;
        bno08x_data.roll = (int16_t)((bno08x_dmaBuffer[8]<<8)|bno08x_dmaBuffer[7]) / 100.0;
        bno08x_data.ax = (bno08x_dmaBuffer[10]<<8)|bno08x_dmaBuffer[9];
        bno08x_data.ay = (bno08x_dmaBuffer[12]<<8)|bno08x_dmaBuffer[11];
        bno08x_data.az = (bno08x_dmaBuffer[14]<<8)|bno08x_dmaBuffer[13];
    }
    
    uint8_t dummy[4];
    DL_UART_drainRXFIFO(UART_BNO08X_INST, dummy, 4);

    DL_DMA_setDestAddr(DMA, DMA_BNO08X_CHAN_ID, (uint32_t) &bno08x_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_BNO08X_CHAN_ID, 18);
    DL_DMA_enableChannel(DMA, DMA_BNO08X_CHAN_ID);
}
#endif

#if defined UART_WIT_INST_IRQHandler
void UART_WIT_INST_IRQHandler(void)
{
    uint8_t checkSum, packCnt = 0;
    extern uint8_t wit_dmaBuffer[33];

    DL_DMA_disableChannel(DMA, DMA_WIT_CHAN_ID);
    uint8_t rxSize = 32 - DL_DMA_getTransferSize(DMA, DMA_WIT_CHAN_ID);

    if(DL_UART_isRXFIFOEmpty(UART_WIT_INST) == false)
        wit_dmaBuffer[rxSize++] = DL_UART_receiveData(UART_WIT_INST);

    while(rxSize >= 11)
    {
        checkSum=0;
        for(int i=packCnt*11; i<(packCnt+1)*11-1; i++)
            checkSum += wit_dmaBuffer[i];

        if((wit_dmaBuffer[packCnt*11] == 0x55) && (checkSum == wit_dmaBuffer[packCnt*11+10]))
        {
            if(wit_dmaBuffer[packCnt*11+1] == 0x51)
            {
                wit_data.ax = (int16_t)((wit_dmaBuffer[packCnt*11+3]<<8)|wit_dmaBuffer[packCnt*11+2]) / 2.048; //mg
                wit_data.ay = (int16_t)((wit_dmaBuffer[packCnt*11+5]<<8)|wit_dmaBuffer[packCnt*11+4]) / 2.048; //mg
                wit_data.az = (int16_t)((wit_dmaBuffer[packCnt*11+7]<<8)|wit_dmaBuffer[packCnt*11+6]) / 2.048; //mg
                wit_data.temperature =  (int16_t)((wit_dmaBuffer[packCnt*11+9]<<8)|wit_dmaBuffer[packCnt*11+8]) / 100.0; //°C
            }
            else if(wit_dmaBuffer[packCnt*11+1] == 0x52)
            {
                wit_data.gx = (int16_t)((wit_dmaBuffer[packCnt*11+3]<<8)|wit_dmaBuffer[packCnt*11+2]) / 16.384; //°/S
                wit_data.gy = (int16_t)((wit_dmaBuffer[packCnt*11+5]<<8)|wit_dmaBuffer[packCnt*11+4]) / 16.384; //°/S
                wit_data.gz = (int16_t)((wit_dmaBuffer[packCnt*11+7]<<8)|wit_dmaBuffer[packCnt*11+6]) / 16.384; //°/S
            }
            else if(wit_dmaBuffer[packCnt*11+1] == 0x53)
            {
                wit_data.roll  = (int16_t)((wit_dmaBuffer[packCnt*11+3]<<8)|wit_dmaBuffer[packCnt*11+2]) / 32768.0 * 180.0; //°
                wit_data.pitch = (int16_t)((wit_dmaBuffer[packCnt*11+5]<<8)|wit_dmaBuffer[packCnt*11+4]) / 32768.0 * 180.0; //°
                wit_data.yaw   = (int16_t)((wit_dmaBuffer[packCnt*11+7]<<8)|wit_dmaBuffer[packCnt*11+6]) / 32768.0 * 180.0; //°
                wit_data.version = (int16_t)((wit_dmaBuffer[packCnt*11+9]<<8)|wit_dmaBuffer[packCnt*11+8]);
            }
        }

        rxSize -= 11;
        packCnt++;
    }
    
    uint8_t dummy[4];
    DL_UART_drainRXFIFO(UART_WIT_INST, dummy, 4);

    DL_DMA_setDestAddr(DMA, DMA_WIT_CHAN_ID, (uint32_t) &wit_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_WIT_CHAN_ID, 32);
    DL_DMA_enableChannel(DMA, DMA_WIT_CHAN_ID);
}
#endif
#if defined Encoder_PORT
volatile long encoder_left_count = 0;
volatile long encoder_right_count = 0;

static void Encoder_UpdateCount(volatile long *count, uint32_t phase_a_pin, uint32_t phase_b_pin)
{
    uint8_t phase_a_is_high;
    uint8_t phase_b_is_high;

    if (count == 0) {
        return;
    }

    phase_a_is_high = (DL_GPIO_readPins(Encoder_PORT, phase_a_pin) != 0U) ? 1U : 0U;
    phase_b_is_high = (DL_GPIO_readPins(Encoder_PORT, phase_b_pin) != 0U) ? 1U : 0U;

    /*
     * 当前只在 A 相边沿进中断，方向由 A/B 当前电平关系判断。
     * 这与原来的 sum 判定等价，但去掉了 x=1 导致的死分支，更容易看懂。
     */
    if (phase_a_is_high != phase_b_is_high) {
        (*count)++;
    } else {
        (*count)--;
    }
}
#endif

void GROUP1_IRQHandler(void)
{
    // 【第一步】：拿到 GROUP1 的触发来源
    uint32_t group_status = DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1);

    switch (group_status) 
    {
        // ==========================================
        // 核心处理区：BNO085 当前连接在 GPIOB。
        // ==========================================
        case DL_INTERRUPT_GROUP1_IIDX_GPIOB:
        {
            uint32_t gpio_iidx;
            // 【第二步】：循环读取 GPIOB 具体是哪根针触发的中断
            while ((gpio_iidx = DL_GPIO_getPendingInterrupt(GPIOB)) != 0)
            {
                switch (gpio_iidx)
                {
                    // === 1. 处理 BNO085 中断 ===
                    #if (defined GPIO_BNO085_PORT) && (GPIO_BNO085_PORT == GPIOB)
                    case GPIO_BNO085_PIN_BNO085_INT_IIDX:
                        /*
                         * 中断里只置位标志，I2C/SH-2 读取和姿态解算
                         * 全部放在主循环中执行。
                         */
                        BNO085_NotifyDataReadyFromIsr();
                        break;
                    #endif

                    // === 2. 处理左轮编码器 (Left_A) ===
                    #if defined Encoder_PORT
                    case Encoder_Left_A_IIDX:
                        Encoder_UpdateCount(&encoder_left_count,
                                            Encoder_Left_A_PIN,
                                            Encoder_Left_B_PIN);
                        break;

                    // === 3. 处理右轮编码器 (Right_A) ===
                    case Encoder_Right_A_IIDX:
                        Encoder_UpdateCount(&encoder_right_count,
                                            Encoder_Right_A_PIN,
                                            Encoder_Right_B_PIN);
                        break;
                    #endif

                    // === 4. 处理可能存在的其他传感器 ===
                    #if (defined GPIO_LSM6DSV16X_PORT) && (GPIO_LSM6DSV16X_PORT == GPIOB)
                    case GPIO_LSM6DSV16X_PIN_LSM6DSV16X_INT_IIDX:
                        Read_LSM6DSV16X();
                        break;
                    #endif

                    #if (defined GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT) && (GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT == GPIOB)
                    case GPIO_VL53L0X_PIN_VL53L0X_GPIO1_IIDX:
                        Read_VL53L0X();
                        break;
                    #endif
                }
            }
            break; // 结束 GPIOB 的处理
        }

        // ==========================================
        // 预留区：如果以后有传感器接在 GPIOA 上，在这里处理
        // ==========================================
        case DL_INTERRUPT_GROUP1_IIDX_GPIOA:
        {
            uint32_t gpioA_iidx;
            while ((gpioA_iidx = DL_GPIO_getPendingInterrupt(GPIOA)) != 0)
            {
                switch(gpioA_iidx)
                {
                    #if (defined GPIO_BNO085_PORT) && (GPIO_BNO085_PORT == GPIOA)
                    case GPIO_BNO085_PIN_BNO085_INT_IIDX:
                        /*
                         * 保留 GPIOA 兼容分支，方便以后调整 BNO085 INT 引脚。
                         */
                        BNO085_NotifyDataReadyFromIsr();
                        break;
                    #endif

                    // ... 这里可以预留其他 GPIOA 的传感器处理 ...
                }
            }
            break;
        }
    }
}
