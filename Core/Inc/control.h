#ifndef CONTROL_H
#define CONTROL_H
#include "stm32f1xx_hal.h"
typedef enum { CONTROL_STOPPED, CONTROL_PWM, CONTROL_SPEED, CONTROL_ANGLE, CONTROL_ZERO } ControlMode;
void Control_Start(TIM_HandleTypeDef *pwm, TIM_HandleTypeDef *encoder, UART_HandleTypeDef *uart);
void Control_Task(void);
void Control_Command(const char *command);
void Control_Stop(const char *reason);
void Control_SensorFault(void);
void Control_SetStream(int enabled);
void Control_Values(float out[7]);
#endif
