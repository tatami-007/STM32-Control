/**
 * @file pid.c
 * @brief 位置式 PID 计算与状态复位。
 */
#include "pid.h"
#include <math.h>
void PID_Reset(PID *p)
{
    p->integral = 0;
    p->last_error = 0;
}
float PID_Update(PID *p, float target, float actual, float dt)
{
    /* 采样周期或输入无效时返回零输出。 */
    if (dt <= 0 || dt >= 0.2f || !isfinite(dt) || !isfinite(target) || !isfinite(actual)) return 0;

    /* 分别计算比例项、积分项和误差微分项。 */
    float error = target - actual;
    float p_output = p->kp * error;
    p->integral += p->ki * error * dt;
    float d_output = p->kd * (error - p->last_error) / dt;
    p->last_error = error;

    /* 对积分项和总输出分别限幅。 */
    if (p->integral > p->limit) p->integral = p->limit;
    if (p->integral < -p->limit) p->integral = -p->limit;
    float output = p_output + p->integral + d_output;
    if (output > p->limit) output = p->limit;
    if (output < -p->limit) output = -p->limit;
    return output;
}
