/**
 * @file config.h
 * @brief 电机参数、安装方向与控制范围。
 */
#ifndef CONFIG_H
#define CONFIG_H

#define ENCODER_CPR 2112.0f   /* 输出轴每圈计数；按 1:48、单相 528 脉冲及四倍频推算，待实测 */
#define ENCODER_SIGN (-1)     /* 编码器方向修正系数，使正向驱动对应正转速 */
#define ANGLE_SIGN (-1)       /* 俯仰角到控制角度的方向修正系数 */
#define CONTROL_MS 20U        /* 控制周期，单位 ms */
#define PWM_LIMIT 500         /* PWM 幅值上限；ARR = 999，对应 50% 占空比 */
#define SPEED_LIMIT 30.0f     /* 目标转速绝对值上限，单位 RPM */
#define ANGLE_LIMIT 60.0f     /* 目标角度绝对值上限，单位 ° */
#define ANGLE_SPEED_MAX 12.0f /* 角度外环输出转速绝对值上限，单位 RPM */
#endif
