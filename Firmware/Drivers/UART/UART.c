#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "UART.h"
#include "clock.h"

/* UART 中断接收环形缓冲：
 * 蓝牙收到的字节先进入这里，主循环再统一取出处理。 */
static volatile uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t uart_rx_write_index = 0U;
static volatile uint16_t uart_rx_read_index = 0U;
static volatile uint8_t uart_rx_overflow = 0U;
static volatile uint8_t uart_tx_timeout = 0U;

/* 蓝牙命令缓存：
 * 将收到的内容按一行拼接，行结束符为 CR/LF。
 * 这样后面可以直接解析类似 PID,1.2,0.3,0.05 的命令。 */
static char uart_bt_line_buffer[UART_BT_LINE_BUFFER_SIZE];
static uint16_t uart_bt_line_index = 0U;
static volatile uint8_t uart_bt_line_overflow = 0U;
static volatile uint32_t uart_bt_rx_byte_count = 0UL;
static volatile uint32_t uart_bt_rx_frame_count = 0UL;

#define UART_BT_LINE_QUEUE_SIZE    (4U)

static char uart_bt_line_queue[UART_BT_LINE_QUEUE_SIZE][UART_BT_LINE_BUFFER_SIZE];
static volatile uint16_t uart_bt_line_queue_length[UART_BT_LINE_QUEUE_SIZE];
static volatile uint8_t uart_bt_line_queue_read_index = 0U;
static volatile uint8_t uart_bt_line_queue_write_index = 0U;
static volatile uint8_t uart_bt_line_queue_count = 0U;

/*
 * UART 发送保护：
 * 正常情况下 TX FIFO 很快会空出来；如果蓝牙模块/串口硬件异常，
 * 这里用时间 + 循环次数双保险退出，避免主循环被 while 永久卡住。
 */
#define UART_TX_FIFO_WAIT_TIMEOUT_MS    (20UL)
#define UART_TX_BUSY_WAIT_TIMEOUT_MS    (50UL)
#define UART_TX_WAIT_LOOP_LIMIT         (1000000UL)

static uint8_t UART_HasWaitTimedOut(unsigned long start_time_ms,
                                    unsigned long timeout_ms,
                                    unsigned long loop_count)
{
    if ((tick_ms - start_time_ms) >= timeout_ms) {
        return 1U;
    }

    if (loop_count >= UART_TX_WAIT_LOOP_LIMIT) {
        return 1U;
    }

    return 0U;
}

static uint8_t UART_WaitTxFifoReady(void)
{
    unsigned long start_time_ms = tick_ms;
    unsigned long loop_count = 0UL;

    while (DL_UART_Main_isTXFIFOFull(UART_0_INST)) {
        if (UART_HasWaitTimedOut(start_time_ms,
                                 UART_TX_FIFO_WAIT_TIMEOUT_MS,
                                 loop_count) != 0U) {
            uart_tx_timeout = 1U;
            return 0U;
        }
        loop_count++;
    }

    return 1U;
}

static uint8_t UART_WaitTxIdle(void)
{
    unsigned long start_time_ms = tick_ms;
    unsigned long loop_count = 0UL;

    while (DL_UART_Main_isBusy(UART_0_INST)) {
        if (UART_HasWaitTimedOut(start_time_ms,
                                 UART_TX_BUSY_WAIT_TIMEOUT_MS,
                                 loop_count) != 0U) {
            uart_tx_timeout = 1U;
            return 0U;
        }
        loop_count++;
    }

    return 1U;
}

static uint8_t UART_SendByteChecked(uint8_t data)
{
    if (UART_WaitTxFifoReady() == 0U) {
        return 0U;
    }

    DL_UART_Main_transmitData(UART_0_INST, data);
    return 1U;
}

/* 清空 UART 接收状态，避免残留数据影响本次蓝牙通信。 */
static void UART_ResetRxState(void)
{
    uart_rx_write_index = 0U;
    uart_rx_read_index = 0U;
    uart_rx_overflow = 0U;
}

/* 清空蓝牙命令缓存和统计信息。 */
static void UART_ResetBluetoothState(void)
{
    uint8_t index;

    uart_bt_line_index = 0U;
    uart_bt_line_overflow = 0U;
    uart_bt_rx_byte_count = 0UL;
    uart_bt_rx_frame_count = 0UL;
    UART_LlmTunerSetEnabled(0U);
    uart_bt_line_queue_read_index = 0U;
    uart_bt_line_queue_write_index = 0U;
    uart_bt_line_queue_count = 0U;
    uart_bt_line_buffer[0] = '\0';

    for (index = 0U; index < UART_BT_LINE_QUEUE_SIZE; index++) {
        uart_bt_line_queue[index][0] = '\0';
        uart_bt_line_queue_length[index] = 0U;
    }
}

static uint8_t UART_BluetoothQueueLine(const char *line, uint16_t length)
{
    if ((line == NULL) || (length == 0U)) {
        return 0U;
    }

    if (uart_bt_line_queue_count >= UART_BT_LINE_QUEUE_SIZE) {
        UART_SendString("[BT] RX line queue full\r\n");
        return 0U;
    }

    memcpy(uart_bt_line_queue[uart_bt_line_queue_write_index], line, (size_t) length + 1U);
    uart_bt_line_queue_length[uart_bt_line_queue_write_index] = length;
    uart_bt_line_queue_write_index =
        (uint8_t) ((uart_bt_line_queue_write_index + 1U) % UART_BT_LINE_QUEUE_SIZE);
    uart_bt_line_queue_count++;
    return 1U;
}

/* 将蓝牙收到的字节拼成一整行命令：
 * 1. 遇到 CR/LF 视为一条命令结束。
 * 2. 收到完整一行后先回显，方便确认收发链路正常。
 * 3. 如果一行过长，直接报错，避免越界。 */
static void UART_BluetoothPushByte(uint8_t data)
{
    if ((data == '\r') || (data == '\n')) {
        if (uart_bt_line_overflow != 0U) {
            UART_SendString("[BT] RX line too long\r\n");
            uart_bt_line_index = 0U;
            uart_bt_line_overflow = 0U;
            uart_bt_line_buffer[0] = '\0';
            return;
        }

        if (uart_bt_line_index == 0U) {
            return;
        }

        uart_bt_line_buffer[uart_bt_line_index] = '\0';
        (void) UART_BluetoothQueueLine(uart_bt_line_buffer, uart_bt_line_index);
        uart_bt_rx_frame_count++;
        UART_Printf("[BT RX OK] %s\r\n", uart_bt_line_buffer);

        uart_bt_line_index = 0U;
        uart_bt_line_buffer[0] = '\0';
        return;
    }

    if (uart_bt_line_overflow != 0U) {
        return;
    }

    if (uart_bt_line_index < (UART_BT_LINE_BUFFER_SIZE - 1U)) {
        uart_bt_line_buffer[uart_bt_line_index] = (char) data;
        uart_bt_line_index++;
        uart_bt_line_buffer[uart_bt_line_index] = '\0';
    } else {
        uart_bt_line_overflow = 1U;
    }
}

/* 使能 UART0 中断。
 * 蓝牙模块当前复用 UART0，这一步是接收中断生效的前提。 */
void UART_InitInterrupt(void)
{
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

/* UART 中断服务函数：
 * 仅负责把 RX FIFO 中的数据搬运到环形缓冲。 */
void UART_IRQHandler(void)
{
    while (DL_UART_isRXFIFOEmpty(UART_0_INST) == false) {
        uint8_t data = DL_UART_receiveData(UART_0_INST);
        uint16_t next_index = (uint16_t) ((uart_rx_write_index + 1U) % UART_RX_BUFFER_SIZE);

        if (next_index == uart_rx_read_index) {
            uart_rx_overflow = 1U;
        } else {
            uart_rx_buffer[uart_rx_write_index] = data;
            uart_rx_write_index = next_index;
        }
    }
}

void UART_0_INST_IRQHandler(void)
{
    UART_IRQHandler();
}

void UART_SendByte(uint8_t data)
{
    (void) UART_SendByteChecked(data);
}

void UART_SendBuffer(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    if (data == NULL) {
        return;
    }

    for (index = 0U; index < length; index++) {
        if (UART_SendByteChecked(data[index]) == 0U) {
            return;
        }
    }

    (void) UART_WaitTxIdle();
}

void UART_SendString(const char *str)
{
    if (str == NULL) {
        return;
    }

    while (*str != '\0') {
        if (UART_SendByteChecked((uint8_t) (*str)) == 0U) {
            return;
        }
        str++;
    }

    (void) UART_WaitTxIdle();
}

uint8_t UART_ReadByte(uint8_t *data)
{
    if (data == NULL) {
        return 0U;
    }

    if (uart_rx_read_index == uart_rx_write_index) {
        return 0U;
    }

    *data = uart_rx_buffer[uart_rx_read_index];
    uart_rx_read_index = (uint16_t) ((uart_rx_read_index + 1U) % UART_RX_BUFFER_SIZE);
    return 1U;
}

uint16_t UART_GetRxCount(void)
{
    if (uart_rx_write_index >= uart_rx_read_index) {
        return (uint16_t) (uart_rx_write_index - uart_rx_read_index);
    }

    return (uint16_t) (UART_RX_BUFFER_SIZE - uart_rx_read_index + uart_rx_write_index);
}

uint8_t UART_HasRxOverflow(void)
{
    return uart_rx_overflow;
}

void UART_ClearRxOverflow(void)
{
    uart_rx_overflow = 0U;
}

void UART_Printf(const char *format, ...)
{
    char buffer[128];
    va_list args;

    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    UART_SendString(buffer);
}

/* 蓝牙测试初始化：
 * 清空缓存、打开中断，并主动发送一条提示信息。 */
void UART_BluetoothInit(void)
{
    UART_ResetRxState();
    UART_ResetBluetoothState();
    UART_InitInterrupt();
    UART_SendString("\r\n[BT] Test mode ready. Send text ending with CR/LF.\r\n");
}

/* 蓝牙主循环任务：
 * 主循环中持续调用它，把中断收到的字节进一步整理成命令。 */
void UART_BluetoothTask(void)
{
    uint8_t data;

    while (UART_ReadByte(&data) != 0U) {
        uart_bt_rx_byte_count++;
        UART_BluetoothPushByte(data);
    }

    if (UART_HasRxOverflow() != 0U) {
        UART_ClearRxOverflow();
        UART_SendString("[BT] RX ring buffer overflow\r\n");
    }
}

uint8_t UART_BluetoothReadLine(char *buffer, uint16_t max_length)
{
    uint16_t copy_length;
    uint8_t read_index;

    if ((buffer == NULL) || (max_length == 0U) || (uart_bt_line_queue_count == 0U)) {
        return 0U;
    }

    read_index = uart_bt_line_queue_read_index;
    copy_length = uart_bt_line_queue_length[read_index];
    if (copy_length >= max_length) {
        copy_length = (uint16_t) (max_length - 1U);
    }

    memcpy(buffer, uart_bt_line_queue[read_index], copy_length);
    buffer[copy_length] = '\0';

    uart_bt_line_queue[read_index][0] = '\0';
    uart_bt_line_queue_length[read_index] = 0U;
    uart_bt_line_queue_read_index =
        (uint8_t) ((uart_bt_line_queue_read_index + 1U) % UART_BT_LINE_QUEUE_SIZE);
    uart_bt_line_queue_count--;
    return 1U;
}

uint8_t UART_BluetoothHasNewLine(void)
{
    return (uint8_t) ((uart_bt_line_queue_count != 0U) ? 1U : 0U);
}

uint16_t UART_BluetoothGetLastLineLength(void)
{
    if (uart_bt_line_queue_count == 0U) {
        return 0U;
    }

    return uart_bt_line_queue_length[uart_bt_line_queue_read_index];
}

uint32_t UART_BluetoothGetRxByteCount(void)
{
    return uart_bt_rx_byte_count;
}

uint32_t UART_BluetoothGetRxFrameCount(void)
{
    return uart_bt_rx_frame_count;
}
