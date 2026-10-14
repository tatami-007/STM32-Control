#ifndef CONFIG_H
#define CONFIG_H
/* 电机参数、安装方向和控制范围。 */
#define ENCODER_CPR 2112.0f   /* 1:48商品页528线按单相脉冲数计算：528×4，待实测 */
#define ENCODER_SIGN (-1)     /* 正PWM时编码器计数的方向 */
#define ANGLE_SIGN (-1)       /* 正PWM时所选姿态角的方向 */
#define CONTROL_MS 20U        /* 每20ms计算一次PID，50Hz控制 */
#define PWM_LIMIT 500         /* TIM2 ARR=999，最大50%占空比 */
#define SPEED_LIMIT 30.0f     /* 串口目标速度范围，RPM */
#define ANGLE_LIMIT 60.0f     /* 串口目标角度范围，°；不是20°自动停机 */
#define ANGLE_SPEED_MAX 12.0f /* 角度外环输出速度上限，RPM */
#endif
