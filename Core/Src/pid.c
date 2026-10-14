#include "pid.h"
#include <math.h>
void PID_Reset(PID *p)
{
    p->integral = 0;
    p->last_error = 0;
}
float PID_Update(PID *p, float target, float actual, float dt)
{
    // 时间或数据无效时输出0。
    if (dt <= 0 || dt >= 0.2f || !isfinite(dt) || !isfinite(target) || !isfinite(actual)) return 0;

    // P看当前误差，I累加误差，D看误差变化。
    float error = target - actual;
    float p_output = p->kp * error;
    p->integral += p->ki * error * dt;
    float d_output = p->kd * (error - p->last_error) / dt;
    p->last_error = error;

    // 积分和最终输出都限制在允许范围内。
    if (p->integral > p->limit) p->integral = p->limit;
    if (p->integral < -p->limit) p->integral = -p->limit;
    float output = p_output + p->integral + d_output;
    if (output > p->limit) output = p->limit;
    if (output < -p->limit) output = -p->limit;
    return output;
}
