#include "camera_link.h"

#include <stddef.h>
#include <string.h>

#include "ti_msp_dl_config.h"

#define CAMERA_LINK_RX_BUFFER_SIZE      (512U)

static volatile uint8_t g_camera_rx_buffer[CAMERA_LINK_RX_BUFFER_SIZE];
static volatile uint16_t g_camera_rx_write_index;
static volatile uint16_t g_camera_rx_read_index;
static volatile uint32_t g_camera_rx_byte_count;
static volatile uint32_t g_camera_overflow_count;

static char g_camera_line_buffer[CAMERA_LINK_LINE_BUFFER_SIZE];
static uint16_t g_camera_line_length;
static uint8_t g_camera_discard_line;

static char g_camera_latest_line[CAMERA_LINK_LINE_BUFFER_SIZE];
static uint16_t g_camera_latest_length;
static uint32_t g_camera_frame_count;
static uint32_t g_camera_last_frame_ms;
static uint8_t g_camera_has_frame;

static uint8_t CameraLink_ReadByte(uint8_t *data)
{
    if ((data == NULL) ||
        (g_camera_rx_read_index == g_camera_rx_write_index)) {
        return 0U;
    }

    *data = g_camera_rx_buffer[g_camera_rx_read_index];
    g_camera_rx_read_index =
        (uint16_t) ((g_camera_rx_read_index + 1U) %
                    CAMERA_LINK_RX_BUFFER_SIZE);
    return 1U;
}

static uint8_t CameraLink_StoreCurrentLine(uint32_t now_ms)
{
    if ((g_camera_line_length == 0U) ||
        (g_camera_discard_line != 0U)) {
        return 0U;
    }

    g_camera_line_buffer[g_camera_line_length] = '\0';
    memcpy(g_camera_latest_line,
           g_camera_line_buffer,
           (size_t) g_camera_line_length + 1U);
    g_camera_latest_length = g_camera_line_length;
    g_camera_frame_count++;
    g_camera_last_frame_ms = now_ms;
    g_camera_has_frame = 1U;
    return 1U;
}

void CameraLink_Init(void)
{
    g_camera_rx_write_index = 0U;
    g_camera_rx_read_index = 0U;
    g_camera_rx_byte_count = 0UL;
    g_camera_overflow_count = 0UL;
    g_camera_line_length = 0U;
    g_camera_discard_line = 0U;
    g_camera_latest_length = 0U;
    g_camera_frame_count = 0UL;
    g_camera_last_frame_ms = 0UL;
    g_camera_has_frame = 0U;
    g_camera_line_buffer[0] = '\0';
    g_camera_latest_line[0] = '\0';

    /* 丢弃上电阶段可能残留在 FIFO 中的不完整数据。 */
    while (DL_UART_isRXFIFOEmpty(UART_CAMERA_INST) == false) {
        (void) DL_UART_receiveData(UART_CAMERA_INST);
    }

    NVIC_ClearPendingIRQ(UART_CAMERA_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_CAMERA_INST_INT_IRQN);
}

/*
 * UART2 中断只搬运字节到环形缓冲。ASCII 组帧和后续解析都留在主循环，
 * 避免摄像头持续发送时阻塞编码器和控制任务。
 */
void UART_CAMERA_INST_IRQHandler(void)
{
    while (DL_UART_isRXFIFOEmpty(UART_CAMERA_INST) == false) {
        uint8_t data = DL_UART_receiveData(UART_CAMERA_INST);
        uint16_t next_index =
            (uint16_t) ((g_camera_rx_write_index + 1U) %
                        CAMERA_LINK_RX_BUFFER_SIZE);

        g_camera_rx_byte_count++;
        if (next_index == g_camera_rx_read_index) {
            g_camera_overflow_count++;
        } else {
            g_camera_rx_buffer[g_camera_rx_write_index] = data;
            g_camera_rx_write_index = next_index;
        }
    }
}

void CameraLink_Task(uint32_t now_ms)
{
    uint8_t data;

    while (CameraLink_ReadByte(&data) != 0U) {
        if ((data == '\r') || (data == '\n')) {
            (void) CameraLink_StoreCurrentLine(now_ms);

            g_camera_line_length = 0U;
            g_camera_discard_line = 0U;
            g_camera_line_buffer[0] = '\0';
            continue;
        }

        if (g_camera_discard_line != 0U) {
            continue;
        }

        if (g_camera_line_length <
            (CAMERA_LINK_LINE_BUFFER_SIZE - 1U)) {
            g_camera_line_buffer[g_camera_line_length++] = (char) data;
        } else {
            /* 超长帧整行丢弃，等待下一个 CR/LF 后重新同步。 */
            g_camera_overflow_count++;
            g_camera_line_length = 0U;
            g_camera_discard_line = 1U;
            g_camera_line_buffer[0] = '\0';
        }
    }
}

uint8_t CameraLink_CopyLatestLine(char *buffer, uint16_t buffer_size)
{
    uint16_t copy_length;

    if ((buffer == NULL) || (buffer_size == 0U) ||
        (g_camera_has_frame == 0U)) {
        return 0U;
    }

    copy_length = g_camera_latest_length;
    if (copy_length >= buffer_size) {
        copy_length = (uint16_t) (buffer_size - 1U);
    }

    memcpy(buffer, g_camera_latest_line, copy_length);
    buffer[copy_length] = '\0';
    return 1U;
}

uint8_t CameraLink_HasFrame(void)
{
    return g_camera_has_frame;
}

uint32_t CameraLink_GetRxByteCount(void)
{
    return g_camera_rx_byte_count;
}

uint32_t CameraLink_GetFrameCount(void)
{
    return g_camera_frame_count;
}

uint32_t CameraLink_GetOverflowCount(void)
{
    return g_camera_overflow_count;
}

uint32_t CameraLink_GetLastFrameMs(void)
{
    return g_camera_last_frame_ms;
}
