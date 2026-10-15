/**
 * @file motor.h
 * @brief TB6612 电机控制接口。
 */
#ifndef MOTOR_H
#define MOTOR_H
#include "stm32f1xx_hal.h"
#include "config.h"

/**
 * @brief 绑定 PWM 定时器并启动通道 1，初始输出为 0。
 * @param timer 已完成初始化的 PWM 定时器句柄。
 */
void Motor_Start(TIM_HandleTypeDef *timer);

/**
 * @brief 设置有符号 PWM 输出，自动限幅并处理换向等待。
 * @param pwm 比较寄存器目标值，正负号表示方向。
 * @note 调用前须执行 Motor_Start()；换向等待期间须周期性调用。
 */
void Motor_Set(int pwm);

/**
 * @brief 将 PWM 和方向引脚清零。
 * @note 调用前须执行 Motor_Start()。
 */
void Motor_Stop(void);

/**
 * @brief 获取当前有符号 PWM 输出。
 * @return 实际写入的 PWM 输出值，停机时为 0。
 */
int Motor_Output(void);
#endif
