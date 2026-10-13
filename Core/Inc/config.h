#ifndef CONFIG_H
#define CONFIG_H
/* 仅需确认这些电机/安装参数，其他代码不需要改。 */
#define ENCODER_CPR 2112.0f   /* 输出轴一圈四倍频计数，尚未核实 */
#define ENCODER_SIGN (-1)     /* 正PWM时编码器计数的方向 */
#define ANGLE_USE_ROLL 0      /* 0用pitch，1用roll；电机轴应平行所选IMU轴 */
#define ANGLE_SIGN (-1)       /* 正PWM时所选姿态角的方向 */
#define CONTROL_MS 20U        /* 每20ms计算一次PID，50Hz控制 */
#define PWM_LIMIT 500         /* TIM2 ARR=999，最大50%占空比 */
#define SPEED_LIMIT 30.0f     /* 串口目标速度范围，RPM */
#define ANGLE_LIMIT 60.0f     /* 串口目标角度范围，°；不是20°自动停机 */
#define ANGLE_SPEED_MAX 12.0f /* 角度外环输出速度上限，RPM */
#endif
