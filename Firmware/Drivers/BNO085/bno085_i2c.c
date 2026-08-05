#include "bno085_i2c.h"

#include <stddef.h>

#include "clock.h"
#include "sh2/sh2_err.h"
#include "ti_msp_dl_config.h"

#define BNO085_I2C_TIMEOUT_MS           (10UL)
#define BNO085_SHTP_HEADER_LENGTH       (4U)

typedef enum {
    BNO085_HAL_READ_IDLE = 0,
    BNO085_HAL_READ_FULL_TRANSFER
} BNO085_HalReadState;

static sh2_Hal_t g_bno085_hal;
static uint8_t g_bno085_address = BNO085_I2C_ADDRESS_DEFAULT;
static volatile uint8_t g_bno085_data_ready;
static BNO085_HalReadState g_bno085_read_state;
static uint16_t g_bno085_transfer_length;
static uint32_t g_bno085_timestamp_us;

static uint8_t BNO085_I2cHasTimedOut(unsigned long start_ms)
{
    return (uint8_t) (((tick_ms - start_ms) >= BNO085_I2C_TIMEOUT_MS) ? 1U : 0U);
}

static void BNO085_I2cRestoreBus(void)
{
    uint8_t cycle_count;

    /*
     * 总线异常时临时把 SCL 改为 GPIO，输出 9 个时钟释放可能卡住 SDA
     * 的从机，然后重新调用 SysConfig 生成的 I2C 初始化函数。
     */
    DL_I2C_reset(I2C_BNO085_INST);
    DL_GPIO_initDigitalOutput(GPIO_I2C_BNO085_IOMUX_SCL);
    DL_GPIO_initDigitalInputFeatures(GPIO_I2C_BNO085_IOMUX_SDA,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableOutput(GPIO_I2C_BNO085_SCL_PORT, GPIO_I2C_BNO085_SCL_PIN);

    for (cycle_count = 0U; cycle_count < 9U; cycle_count++) {
        DL_GPIO_clearPins(GPIO_I2C_BNO085_SCL_PORT, GPIO_I2C_BNO085_SCL_PIN);
        delay_cycles(CPUCLK_FREQ / 200000U);
        DL_GPIO_setPins(GPIO_I2C_BNO085_SCL_PORT, GPIO_I2C_BNO085_SCL_PIN);
        delay_cycles(CPUCLK_FREQ / 200000U);

        if (DL_GPIO_readPins(GPIO_I2C_BNO085_SDA_PORT,
                             GPIO_I2C_BNO085_SDA_PIN) != 0U) {
            break;
        }
    }

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_BNO085_IOMUX_SDA,
        GPIO_I2C_BNO085_IOMUX_SDA_FUNC,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_BNO085_IOMUX_SCL,
        GPIO_I2C_BNO085_IOMUX_SCL_FUNC,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_BNO085_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_BNO085_IOMUX_SCL);
    DL_I2C_enablePower(I2C_BNO085_INST);
    SYSCFG_DL_I2C_BNO085_init();
}

static int BNO085_I2cWaitControllerIdle(void)
{
    unsigned long start_ms = tick_ms;

    while ((DL_I2C_getControllerStatus(I2C_BNO085_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (BNO085_I2cHasTimedOut(start_ms) != 0U) {
            BNO085_I2cRestoreBus();
            return SH2_ERR_TIMEOUT;
        }
    }

    return SH2_OK;
}

static int BNO085_I2cReadRaw(uint8_t *buffer, uint16_t length)
{
    uint16_t received = 0U;
    unsigned long start_ms;
    uint32_t status;

    if ((buffer == NULL) || (length == 0U)) {
        return SH2_ERR_BAD_PARAM;
    }

    if (BNO085_I2cWaitControllerIdle() != SH2_OK) {
        return SH2_ERR_TIMEOUT;
    }

    DL_I2C_flushControllerRXFIFO(I2C_BNO085_INST);
    DL_I2C_clearInterruptStatus(
        I2C_BNO085_INST,
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
            DL_I2C_INTERRUPT_CONTROLLER_NACK);
    DL_I2C_startControllerTransfer(I2C_BNO085_INST,
                                   g_bno085_address,
                                   DL_I2C_CONTROLLER_DIRECTION_RX,
                                   length);

    start_ms = tick_ms;
    for (;;) {
        while ((received < length) &&
               !DL_I2C_isControllerRXFIFOEmpty(I2C_BNO085_INST)) {
            buffer[received++] =
                DL_I2C_receiveControllerData(I2C_BNO085_INST);
        }

        status = DL_I2C_getRawInterruptStatus(
            I2C_BNO085_INST,
            DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
                DL_I2C_INTERRUPT_CONTROLLER_NACK);

        if ((status & DL_I2C_INTERRUPT_CONTROLLER_NACK) != 0U) {
            DL_I2C_resetControllerTransfer(I2C_BNO085_INST);
            DL_I2C_flushControllerRXFIFO(I2C_BNO085_INST);
            return SH2_ERR_IO;
        }

        if ((status & DL_I2C_INTERRUPT_CONTROLLER_RX_DONE) != 0U) {
            while ((received < length) &&
                   !DL_I2C_isControllerRXFIFOEmpty(I2C_BNO085_INST)) {
                buffer[received++] =
                    DL_I2C_receiveControllerData(I2C_BNO085_INST);
            }
            break;
        }

        if (BNO085_I2cHasTimedOut(start_ms) != 0U) {
            DL_I2C_resetControllerTransfer(I2C_BNO085_INST);
            BNO085_I2cRestoreBus();
            return SH2_ERR_TIMEOUT;
        }
    }

    return (received == length) ? SH2_OK : SH2_ERR_IO;
}

static int BNO085_I2cWriteRaw(const uint8_t *buffer, uint16_t length)
{
    const uint8_t *cursor = buffer;
    uint16_t remaining = length;
    unsigned long start_ms;
    uint32_t status;

    if ((buffer == NULL) || (length == 0U)) {
        return SH2_ERR_BAD_PARAM;
    }

    if (BNO085_I2cWaitControllerIdle() != SH2_OK) {
        return SH2_ERR_TIMEOUT;
    }

    DL_I2C_flushControllerTXFIFO(I2C_BNO085_INST);
    DL_I2C_clearInterruptStatus(
        I2C_BNO085_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
            DL_I2C_INTERRUPT_CONTROLLER_NACK);

    {
        uint16_t filled =
            DL_I2C_fillControllerTXFIFO(I2C_BNO085_INST, cursor, remaining);
        cursor += filled;
        remaining -= filled;
    }

    DL_I2C_startControllerTransfer(I2C_BNO085_INST,
                                   g_bno085_address,
                                   DL_I2C_CONTROLLER_DIRECTION_TX,
                                   length);

    start_ms = tick_ms;
    for (;;) {
        if (remaining != 0U) {
            uint16_t filled =
                DL_I2C_fillControllerTXFIFO(I2C_BNO085_INST,
                                            cursor,
                                            remaining);
            cursor += filled;
            remaining -= filled;
        }

        status = DL_I2C_getRawInterruptStatus(
            I2C_BNO085_INST,
            DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
                DL_I2C_INTERRUPT_CONTROLLER_NACK);

        if ((status & DL_I2C_INTERRUPT_CONTROLLER_NACK) != 0U) {
            DL_I2C_resetControllerTransfer(I2C_BNO085_INST);
            DL_I2C_flushControllerTXFIFO(I2C_BNO085_INST);
            return SH2_ERR_IO;
        }

        if ((status & DL_I2C_INTERRUPT_CONTROLLER_TX_DONE) != 0U) {
            break;
        }

        if (BNO085_I2cHasTimedOut(start_ms) != 0U) {
            DL_I2C_resetControllerTransfer(I2C_BNO085_INST);
            BNO085_I2cRestoreBus();
            return SH2_ERR_TIMEOUT;
        }
    }

    return (remaining == 0U) ? SH2_OK : SH2_ERR_IO;
}

static uint8_t BNO085_I2cIntIsAsserted(void)
{
    return (uint8_t)
        ((DL_GPIO_readPins(GPIO_BNO085_PORT,
                           GPIO_BNO085_PIN_BNO085_INT_PIN) == 0U) ? 1U : 0U);
}

static int BNO085_HalOpen(sh2_Hal_t *self)
{
    (void) self;
    g_bno085_data_ready = 0U;
    g_bno085_read_state = BNO085_HAL_READ_IDLE;
    g_bno085_transfer_length = 0U;
    return SH2_OK;
}

static void BNO085_HalClose(sh2_Hal_t *self)
{
    (void) self;
    g_bno085_data_ready = 0U;
    g_bno085_read_state = BNO085_HAL_READ_IDLE;
    g_bno085_transfer_length = 0U;
}

static int BNO085_HalRead(sh2_Hal_t *self,
                          uint8_t *buffer,
                          unsigned int buffer_length,
                          uint32_t *timestamp_us)
{
    uint16_t transfer_length;
    int result;

    (void) self;

    if ((buffer == NULL) || (timestamp_us == NULL)) {
        return SH2_ERR_BAD_PARAM;
    }

    if (g_bno085_read_state == BNO085_HAL_READ_FULL_TRANSFER) {
        if (buffer_length < g_bno085_transfer_length) {
            g_bno085_read_state = BNO085_HAL_READ_IDLE;
            return SH2_ERR_BAD_PARAM;
        }

        result = BNO085_I2cReadRaw(buffer, g_bno085_transfer_length);
        transfer_length = g_bno085_transfer_length;
        g_bno085_read_state = BNO085_HAL_READ_IDLE;
        g_bno085_transfer_length = 0U;
        if (result != SH2_OK) {
            return result;
        }

        *timestamp_us = g_bno085_timestamp_us;
        return (int) transfer_length;
    }

    if ((g_bno085_data_ready == 0U) &&
        (BNO085_I2cIntIsAsserted() == 0U)) {
        return 0;
    }

    if (buffer_length < BNO085_SHTP_HEADER_LENGTH) {
        return SH2_ERR_BAD_PARAM;
    }

    result = BNO085_I2cReadRaw(buffer, BNO085_SHTP_HEADER_LENGTH);
    if (result != SH2_OK) {
        g_bno085_data_ready = 0U;
        return result;
    }

    transfer_length =
        (uint16_t) ((((uint16_t) buffer[1] << 8U) | buffer[0]) & 0x7FFFU);
    if ((transfer_length < BNO085_SHTP_HEADER_LENGTH) ||
        (transfer_length > SH2_HAL_MAX_TRANSFER_IN)) {
        g_bno085_data_ready = 0U;
        return SH2_ERR_IO;
    }

    /*
     * BNO085 的 I2C 协议需要先读 4 字节 SHTP 头获得长度，再发起一次
     * 新的读事务取得完整传输。这里沿用 CEVA 官方 I2C HAL 的两阶段方式。
     */
    g_bno085_timestamp_us = (uint32_t) tick_ms * 1000U;
    g_bno085_transfer_length = transfer_length;
    g_bno085_read_state = BNO085_HAL_READ_FULL_TRANSFER;
    g_bno085_data_ready = 0U;
    *timestamp_us = g_bno085_timestamp_us;
    return (int) BNO085_SHTP_HEADER_LENGTH;
}

static int BNO085_HalWrite(sh2_Hal_t *self,
                           uint8_t *buffer,
                           unsigned int length)
{
    int result;

    (void) self;

    if ((buffer == NULL) || (length == 0U) ||
        (length > SH2_HAL_MAX_TRANSFER_OUT)) {
        return SH2_ERR_BAD_PARAM;
    }

    result = BNO085_I2cWriteRaw(buffer, (uint16_t) length);
    return (result == SH2_OK) ? (int) length : result;
}

static uint32_t BNO085_HalGetTimeUs(sh2_Hal_t *self)
{
    (void) self;
    return (uint32_t) tick_ms * 1000U;
}

sh2_Hal_t *BNO085_I2cHal_Get(uint8_t address)
{
    g_bno085_address = address;
    g_bno085_hal.open = BNO085_HalOpen;
    g_bno085_hal.close = BNO085_HalClose;
    g_bno085_hal.read = BNO085_HalRead;
    g_bno085_hal.write = BNO085_HalWrite;
    g_bno085_hal.getTimeUs = BNO085_HalGetTimeUs;
    return &g_bno085_hal;
}

void BNO085_I2cHal_NotifyDataReadyFromIsr(void)
{
    g_bno085_data_ready = 1U;
}

uint8_t BNO085_I2cHal_HasPendingRead(void)
{
    return (uint8_t) (((g_bno085_read_state != BNO085_HAL_READ_IDLE) ||
                       (g_bno085_data_ready != 0U) ||
                       (BNO085_I2cIntIsAsserted() != 0U)) ? 1U : 0U);
}
