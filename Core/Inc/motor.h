#ifndef MOTOR_H
#define MOTOR_H
#include "stm32f1xx_hal.h"
#include "app_config.h"
#define MOTOR_PWM_LIMIT APP_PWM_LIMIT
void Motor_Start(TIM_HandleTypeDef *timer);
void Motor_Set(int pwm);
void Motor_Task(void);
void Motor_Stop(void);
int Motor_Output(void);
#endif
