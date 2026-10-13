#ifndef APP_H
#define APP_H
#include "stm32f1xx_hal.h"
void App_Init(TIM_HandleTypeDef *pwm, TIM_HandleTypeDef *encoder, UART_HandleTypeDef *uart);
void App_Task(void);
void App_Command(const char *command);
#endif
