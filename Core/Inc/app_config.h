#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 所有需要实机确认/整定的参数集中在此处。 */
#define APP_CONTROL_PERIOD_MS       10U
#define APP_TELEMETRY_PERIOD_MS     50U
#define APP_ENCODER_CPR             2112.0f /* 输出轴一圈的四倍频计数，尚待确认 */
#define APP_ENCODER_SIGN            (-1)   /* 正PWM对应编码器增量的符号 */
#define APP_MAX_SPEED_RPM           30.0f
#define APP_MAX_TARGET_ANGLE        90.0f
#define APP_PWM_LIMIT               500    /* ARR=999，对应50%占空比 */
#define APP_REVERSAL_DELAY_MS       100U
#define APP_STALL_TIMEOUT_MS        2500U
#define APP_ANGLE_DEADBAND          0.5f
#define APP_SPEED_KP                15.0f
#define APP_SPEED_KI                40.0f
#define APP_SPEED_KD                0.0f
#define APP_ANGLE_KP                1.0f
#define APP_ANGLE_KI                0.0f
#define APP_ANGLE_KD                0.06f
#define APP_ANGLE_SPEED_LIMIT       12.0f
/* 当前IMU安装对应的轴向：正PWM试转时陀螺仪的单位方向。
 * 换安装后需重新测定；不是“模块Y轴天然就是电机轴”。 */
#define APP_SHAFT_AXIS              {-0.05099511f, -0.9959804f, -0.07363743f}

#endif
