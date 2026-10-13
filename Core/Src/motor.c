#include "motor.h"
#include "main.h"

static TIM_HandleTypeDef *timer;
static int output, last_direction;
static uint32_t stopped_at;

void Motor_Stop(void)
{
    if (output) stopped_at = HAL_GetTick();
    output = 0;
    __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_1, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
}
void Motor_Start(TIM_HandleTypeDef *pwm)
{
    timer = pwm;
    Motor_Stop();
    if (HAL_TIM_PWM_Start(timer, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
}
void Motor_Set(int pwm)
{
    if (pwm > MOTOR_PWM_LIMIT) pwm = MOTOR_PWM_LIMIT;
    if (pwm < -MOTOR_PWM_LIMIT) pwm = -MOTOR_PWM_LIMIT;
    if (pwm == 0) {
        Motor_Stop();
        return;
    }
    int direction = 1; // 正转为1，反转为-1
    if (pwm < 0) direction = -1;
    if (output && direction != last_direction) Motor_Stop();
    if (last_direction && direction != last_direction && HAL_GetTick() - stopped_at < 100U) return;
    // 两个方向引脚一高一低；PWM占空比始终取正数。
    int duty = pwm;
    if (direction == 1) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
        duty = -pwm;
    }
    __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_1, duty);
    output = pwm;
    last_direction = direction;
}
int Motor_Output(void) { return output; }
