/**
 * @file pid.h
 * @brief 位置式 PID 参数与计算接口。
 */
#ifndef PID_H
#define PID_H

/** @brief 单个 PID 控制器的参数与历史状态。 */
typedef struct {
    float kp, ki, kd;       /**< 比例、积分、微分增益 */
    float limit;            /**< 积分项与总输出的绝对值上限 */
    float integral;         /**< 积分项累计值 */
    float last_error;       /**< 上次误差，用于计算微分项 */
} PID;

/**
 * @brief 清除积分项和历史误差，保留控制参数。
 * @param p PID 控制器。
 */
void PID_Reset(PID *p);

/**
 * @brief 根据目标值、反馈值和采样周期计算 PID 输出。
 * @param p PID 控制器，首次使用前须将历史状态清零。
 * @param target 目标值，与 actual 使用相同单位。
 * @param actual 实际反馈值。
 * @param dt 采样周期，单位 s，有效范围为 (0, 0.2)。
 * @return 限幅后的输出；采样周期或输入无效时返回 0。
 */
float PID_Update(PID *p, float target, float actual, float dt);
#endif
