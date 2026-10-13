#include "pid.h"
#include <math.h>
void PID_Init(PID *p, float kp, float ki, float kd, float limit) {
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->limit = fabsf(limit);
    PID_Reset(p);
}
void PID_Reset(PID *p) {
    p->integral = 0;
    p->previous = 0;
    p->primed = 0;
}
float PID_Update(PID *p, float target, float actual, float dt) {
    if (!(dt > 0 && dt < 0.2f) || !isfinite(target) || !isfinite(actual)) return 0;
    float error = target - actual;
    float derivative = p->primed ? -(actual - p->previous) / dt : 0;
    p->previous = actual;
    p->primed = 1;
    float candidate = p->integral + p->ki * error * dt;
    float output = p->kp * error + candidate + p->kd * derivative;
    /* 饱和且误差继续推动饱和时，不累积积分。 */
    if (!((output > p->limit && error > 0) || (output < -p->limit && error < 0)))
        p->integral = fmaxf(-p->limit, fminf(p->limit, candidate));
    output = p->kp * error + p->integral + p->kd * derivative;
    return fmaxf(-p->limit, fminf(p->limit, output));
}