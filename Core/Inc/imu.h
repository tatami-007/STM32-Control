#ifndef IMU_H
#define IMU_H
#include "stm32f1xx_hal.h"
#include "ins.h"
typedef struct {
  float accel[3],gyro[3];
  INS attitude;
  uint32_t tick;
  int valid;
} IMU_Data;
void IMU_Start(I2C_HandleTypeDef *i2c,UART_HandleTypeDef *uart);
void IMU_Task(void);
const IMU_Data *IMU_GetData(void);
void IMU_Recalibrate(void);
#endif
