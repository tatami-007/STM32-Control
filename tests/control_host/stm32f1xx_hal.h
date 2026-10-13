#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
typedef struct { uint16_t Counter; int Compare; } TIM_HandleTypeDef;
typedef struct {int unused;} UART_HandleTypeDef;
typedef struct {int unused;} I2C_HandleTypeDef;
typedef enum {HAL_OK,HAL_ERROR,HAL_BUSY,HAL_TIMEOUT} HAL_StatusTypeDef;
#define TIM_CHANNEL_1 1
#define TIM_CHANNEL_ALL 3
#define GPIOB 0
#define GPIOA 1
#define GPIO_PIN_6 64
#define GPIO_PIN_7 128
#define GPIO_PIN_0 1
#define GPIO_PIN_1 2
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define USART1_IRQn 0
#define I2C_MEMADD_SIZE_8BIT 1
#define __DMB() ((void)0)
#define __HAL_TIM_GET_COUNTER(t) ((t)->Counter)
#define __HAL_TIM_SET_COMPARE(t,c,v) ((t)->Compare=(v))
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef*,uint16_t,uint16_t,uint16_t,uint8_t*,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef*,uint16_t,uint16_t,uint16_t,uint8_t*,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef*,int);
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef*,int);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef*,uint8_t*,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef*,uint8_t*,uint16_t);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef*,uint8_t*,uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef*);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef*);
void HAL_NVIC_SetPriority(int,int,int);
void HAL_NVIC_EnableIRQ(int);
void HAL_GPIO_WritePin(int,int,int);
int HAL_GPIO_ReadPin(int,int);
#endif
