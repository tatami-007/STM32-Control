#include "motor.h"
#include "main.h"
static TIM_HandleTypeDef *timer;
static int requested,output,last_direction;
static uint32_t stopped_at;
void Motor_Stop(void)
{
  requested=0;
  if(output) stopped_at=HAL_GetTick();
  output=0;
  __HAL_TIM_SET_COMPARE(timer,TIM_CHANNEL_1,0);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0|GPIO_PIN_1,GPIO_PIN_RESET);
}
void Motor_Start(TIM_HandleTypeDef *p)
{
  timer=p; stopped_at=HAL_GetTick(); last_direction=0; Motor_Stop();
  if(HAL_TIM_PWM_Start(timer,TIM_CHANNEL_1)!=HAL_OK) Error_Handler();
}
void Motor_Set(int pwm)
{
  if(pwm>MOTOR_PWM_LIMIT)pwm=MOTOR_PWM_LIMIT;
  if(pwm< -MOTOR_PWM_LIMIT)pwm= -MOTOR_PWM_LIMIT;
  requested=pwm;
}
void Motor_Task(void)
{
  if(!requested)
  {
    if(output) Motor_Stop();
    return;
  }
  int direction=requested>0 ? 1 : -1;
  if(output && direction!=last_direction)
  {
    __HAL_TIM_SET_COMPARE(timer,TIM_CHANNEL_1,0);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0|GPIO_PIN_1,GPIO_PIN_RESET);
    output=0; stopped_at=HAL_GetTick();
  }
  if(direction!=last_direction && last_direction &&
      HAL_GetTick()-stopped_at<APP_REVERSAL_DELAY_MS) return; /* 换向先关PWM */
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,direction>0?GPIO_PIN_SET:GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,direction>0?GPIO_PIN_RESET:GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(timer,TIM_CHANNEL_1,requested>0?requested:-requested);
  output=requested; last_direction=direction;
}
int Motor_Output(void) { return output; }
