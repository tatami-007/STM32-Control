#ifndef SERIAL_H
#define SERIAL_H
#include "stm32f1xx_hal.h"
void Serial_Init(UART_HandleTypeDef *uart);
void Serial_Task(void);
int Serial_Busy(void);
/* 异步发送：字符串必须一直有效到发送结束（用静态数组或字符串常量）。 */
int Serial_Send(const char *text);
#endif
