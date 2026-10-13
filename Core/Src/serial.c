#include "serial.h"
#include <string.h>

/* 收到一行再交给main处理；中断中不做PID或浮点运算。 */
static UART_HandleTypeDef *port;
static uint8_t byte;
static char input[32], completed[32], command[32];
static unsigned int length;
static volatile int ready, busy, stop_requested;
static int discard;

void Serial_Init(UART_HandleTypeDef *uart)
{
    port = uart;
    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    if (HAL_UART_Receive_IT(port, &byte, 1) != HAL_OK) stop_requested = 1;
}
int Serial_Busy(void) { return busy; }
int Serial_Send(const char *text)
{
    if (busy) return 0;
    busy = 1;
    if (HAL_UART_Transmit_IT(port, (uint8_t *)text, (uint16_t)strlen(text)) == HAL_OK) return 1;
    busy = 0;
    return 0;
}
const char *Serial_ReadCommand(void)
{
    if (stop_requested) { stop_requested = 0; ready = 0; return "q"; }
    if (!ready) return NULL;
    strcpy(command, completed); // ready清除前，中断不会覆盖completed
    __DMB(); ready = 0;
    return command;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart != port) return;
    if (byte == 'q' || byte == 'Q') { stop_requested = 1; length = 0; discard = 0; }
    else if (byte == '\n' || byte == '\r') {
        if (length && !discard) {
            input[length] = 0;
            if (ready) stop_requested = 1; // 上一条未处理，停机而不是执行残缺命令
            else { strcpy(completed, input); __DMB(); ready = 1; }
        }
        length = 0; discard = 0;
    } else if (!discard) {
        if (length < sizeof(input) - 1) input[length++] = (char)byte;
        else { discard = 1; stop_requested = 1; }
    }
    if (HAL_UART_Receive_IT(port, &byte, 1) != HAL_OK) stop_requested = 1;
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart) { if (uart == port) busy = 0; }
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart != port) return;
    stop_requested = 1;
    HAL_UART_AbortTransmit(port); busy = 0;
    HAL_UART_AbortReceive(port);
    HAL_UART_Receive_IT(port, &byte, 1);
}
