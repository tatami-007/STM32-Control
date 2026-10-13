#include "serial.h"
#include <string.h>

/* 中断只收字节，主循环解析；避免在中断里做浮点、PID或长时间发送。 */
static UART_HandleTypeDef *port;
static uint8_t received, queue[128];
static volatile unsigned int head, tail;
static volatile int busy, error;
static char line[64];
static unsigned int length;
static int discard;
static uint32_t last_rx;

void Serial_Init(UART_HandleTypeDef *uart) {
    port = uart;
    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    if (HAL_UART_Receive_IT(port, &received, 1) != HAL_OK) error = 1;
}
int Serial_Busy(void) {
    return busy;
}
int Serial_Send(const char *text) {
    if (busy) return 0;
    busy = 1;
    if (HAL_UART_Transmit_IT(port, (uint8_t *)text, (uint16_t)strlen(text)) == HAL_OK) return 1;
    busy = 0;
    return 0;
}
const char *Serial_ReadCommand(void) {
    if (error) {
        error = 0;
        tail = head;
        length = 0;
        discard = 0;
        return "q";
    }
    while (tail != head) {
        char ch = (char)queue[tail];
        tail = (tail + 1U) & 127U;
        last_rx = HAL_GetTick();
        if (ch == 'q' || ch == 'Q') {
            length = 0;
            discard = 0;
            return "q";
        } else if (ch == '\n' || ch == '\r') {
            if (length && !discard) {
                line[length] = 0;
                length = 0;
                return line;
            }
            length = 0;
            discard = 0;
        } else if (!discard) {
            if (length < sizeof(line) - 1)
                line[length++] = ch;
            else {
                length = 0;
                discard = 1;
            } // 超长行整行丢弃，防止截断后误执行
        }
    }
    // 单字符z/p/v/0无需换行；参数命令要换行，不能把pwm的p误当暂停。
    if (length == 1 && HAL_GetTick() - last_rx >= 30U && strchr("zpvh0", line[0])) {
        line[1] = 0;
        length = 0;
        return line;
    }
    return NULL;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart) {
    if (uart != port) return;
    unsigned int next = (head + 1U) & 127U;
    if (next == tail)
        error = 1;
    else {
        queue[head] = received;
        __DMB();
        head = next;
    }
    if (HAL_UART_Receive_IT(port, &received, 1) != HAL_OK) error = 1;
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart) {
    if (uart == port) busy = 0;
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart) {
    if (uart != port) return;
    error = 1;
    HAL_UART_AbortTransmit(port);
    busy = 0;
    HAL_UART_AbortReceive(port);
    if (HAL_UART_Receive_IT(port, &received, 1) != HAL_OK) error = 1;
}
