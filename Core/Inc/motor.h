#ifndef MOTOR_H
#define MOTOR_H
#include "stm32f1xx_hal.h"
#include "config.h"
void Motor_Start(TIM_HandleTypeDef *timer);
void Motor_Set(int pwm);
void Motor_Stop(void);
int Motor_Output(void);
#endif
