#include "pid.h"
#include <math.h>
void PID_Init(PID *p, float kp, float ki, float kd, float limit)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->limit = fabsf(limit);
    PID_Reset(p);
}
void PID_Reset(PID *p)
{
    p->integral = 0;
    p->last_actual = 0;
    p->has_last_sample = 0;
}
float PID_Update(PID *p, float target, float actual, float dt)
{
    // 时间或数据无效时输出0。
    if (dt <= 0 || dt >= 0.2f || !isfinite(dt) || !isfinite(target) || !isfinite(actual)) return 0;

    // 1. P：目标与实际差得越多，输出越大。
    float error = target - actual;
    float p_output = p->kp * error;

    // 2. I：累加误差，补偿一直差一点的情况。
    float next_integral = p->integral + p->ki * error * dt;

    // 3. D：实际值变化越快，阻尼越大，减小来回摆动。
    float d_output = 0;
    if (p->has_last_sample) {
        d_output = -p->kd * (actual - p->last_actual) / dt;
    }
    p->last_actual = actual;
    p->has_last_sample = 1;

    // 输出已到上限且误差还在往外推时，保持原来的积分。
    float output = p_output + next_integral + d_output;
    int keep_integral = 0;
    if (output > p->limit && error > 0) keep_integral = 1;
    if (output < -p->limit && error < 0) keep_integral = 1;
    if (keep_integral == 0) {
        p->integral = next_integral;
        if (p->integral > p->limit) p->integral = p->limit;
        if (p->integral < -p->limit) p->integral = -p->limit;
    }

    // 最后把P、I、D相加，并限制输出范围。
    output = p_output + p->integral + d_output;
    if (output > p->limit) output = p->limit;
    if (output < -p->limit) output = -p->limit;
    return output;
}
