#include "vofa.h"
#include "control.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* RX由中断写入、主循环读取；TX由主循环排队、中断发送。
 * 不在中断中解析命令，也不阻塞控制周期等待串口。 */
#define RX_SIZE 128U
#define TX_SIZE 1024U
static UART_HandleTypeDef *serial;
static uint8_t rx_byte, rx[RX_SIZE], tx[TX_SIZE], tx_chunk[128];
static volatile unsigned int rx_head, rx_tail;
static volatile int tx_busy, uart_fault;
static unsigned int tx_head, tx_tail;
static char command[96];
static unsigned int length;
static int discard_line;
static uint32_t last_rx_tick;

void VOFA_Print(const char *text)
{
    unsigned int n = (unsigned int)strlen(text);
    unsigned int free_bytes = (tx_tail - tx_head - 1U) & (TX_SIZE - 1U);
    if (n > free_bytes) return; /* 队列满时丢整帧，不影响控制。 */
    while (*text) {
        tx[tx_head] = (uint8_t)*text++;
        tx_head = (tx_head + 1U) & (TX_SIZE - 1U);
    }
}

void VOFA_Frame(const float *values, unsigned int count)
{
    char frame[480];
    unsigned int used = 4;
    memcpy(frame, "imu:", 4);
    for (unsigned int i = 0; i < count; ++i) {
        long scaled = lroundf(values[i] * 1000.0f);
        unsigned long absolute = (unsigned long)(scaled < 0 ? -scaled : scaled);
        int n = snprintf(frame+used, sizeof(frame)-used, "%s%lu.%03lu%s",
                         scaled < 0 ? "-" : "", absolute/1000, absolute%1000,
                         i+1 == count ? "\r\n" : ",");
        if (n < 0 || (unsigned int)n >= sizeof(frame)-used) return;
        used += (unsigned int)n;
    }
    VOFA_Print(frame); /* FireWater带标签CSV，不需要浮点printf链接选项。 */
}

void VOFA_Start(UART_HandleTypeDef *uart)
{
    serial = uart;
    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    if (HAL_UART_Receive_IT(serial, &rx_byte, 1) != HAL_OK) uart_fault = 1;
}

void VOFA_Task(void)
{
    if (uart_fault) {
        uart_fault = 0;
        rx_tail = rx_head;
        length = 0;
        discard_line = 0;
        Control_Stop("UART ERROR / RX OVERFLOW");
    }
    while (rx_tail != rx_head) {
        char ch = (char)rx[rx_tail];
        last_rx_tick=HAL_GetTick();
        rx_tail = (rx_tail+1U) & (RX_SIZE-1U);
        if (ch == 'q' || ch == 'Q') { /* 不完整输入行中也可以急停。 */
            length = 0; discard_line = 0;
            Control_Stop("USER STOP");
        } else if (ch == '\r' || ch == '\n') {
            if (length && !discard_line) {
                command[length] = '\0';
                Control_Command(command);
            }
            length = 0; discard_line = 0;
        } else if (!discard_line) {
            if (length < sizeof(command)-1) command[length++] = ch;
            else {
                length = 0; discard_line = 1;
                VOFA_Print("COMMAND TOO LONG; LINE DISCARDED\r\n");
            }
        }
    }
    /* 裸单字符允许不带换行；等30ms接收空闲再解析，避免把pwm/cpr
     * 的首字母误当作p/c命令。完整参数命令必须以换行结束。 */
    if (length==1 && !discard_line && HAL_GetTick()-last_rx_tick>=30U &&
        strchr("0zZhHpPvVcCdD?",command[0])) {
        command[1]='\0'; Control_Command(command); length=0;
    }
    if (!tx_busy && tx_tail != tx_head) {
        unsigned int n = 0, cursor = tx_tail;
        while (cursor != tx_head && n < sizeof(tx_chunk)) {
            tx_chunk[n++] = tx[cursor];
            cursor = (cursor+1U) & (TX_SIZE-1U);
        }
        tx_busy = 1;
        if (HAL_UART_Transmit_IT(serial, tx_chunk, (uint16_t)n) == HAL_OK)
            tx_tail = cursor;
        else tx_busy = 0;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart != serial) return;
    unsigned int next = (rx_head+1U) & (RX_SIZE-1U);
    if (next == rx_tail) uart_fault = 1;
    else { rx[rx_head] = rx_byte; __DMB(); rx_head = next; }
    if (HAL_UART_Receive_IT(serial, &rx_byte, 1) != HAL_OK) uart_fault = 1;
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == serial) tx_busy = 0;
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart != serial) return;
    uart_fault = 1;
    HAL_UART_AbortTransmit(serial);
    tx_busy = 0;
    HAL_UART_AbortReceive(serial);
    if (HAL_UART_Receive_IT(serial, &rx_byte, 1) != HAL_OK) uart_fault = 1;
}
