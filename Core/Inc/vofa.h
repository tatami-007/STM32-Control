#ifndef VOFA_H
#define VOFA_H
#include "stm32f1xx_hal.h"
void VOFA_Start(UART_HandleTypeDef *uart);
void VOFA_Task(void);
void VOFA_Print(const char *text);
void VOFA_Frame(const float *values, unsigned int count);
#endif
