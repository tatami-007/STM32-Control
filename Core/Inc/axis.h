#ifndef AXIS_H
#define AXIS_H
/* 单电机只能补偿绕输出轴的一个自由度。 */
int Axis_SetDirection(const float direction[3]);
int Axis_Zero(const float gravity[3]);
void Axis_Update(const float gyro[3], const float accel[3], float dt);
void Axis_Cancel(void);
float Axis_Angle(void);
float Axis_Rate(void);
int Axis_Ready(void);
#endif
