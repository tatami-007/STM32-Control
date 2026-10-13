#include "ins.h"
#include <math.h>
#include <string.h>

#define RAD_TO_DEG 57.29577951f
#define DEG_TO_RAD 0.01745329252f

/* 用静止加速度初始化roll/pitch，避免从单位四元数缓慢收敛。 */
void INS_Init(INS *ins, const float accel[3])
{
    memset(ins, 0, sizeof(*ins));
    float roll = atan2f(accel[1], accel[2]);
    float pitch = atan2f(-accel[0], hypotf(accel[1], accel[2]));
    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    ins->q[0] = cr * cp;
    ins->q[1] = sr * cp;
    ins->q[2] = cr * sp;
    ins->q[3] = -sr * sp;
    ins->roll = roll * RAD_TO_DEG;
    ins->pitch = pitch * RAD_TO_DEG;
}

void INS_Update(INS *ins, const float accel[3], const float gyro[3], float dt)
{
    if (!(dt > 0.0f && dt <= 0.1f)) return;
    float q0 = ins->q[0], q1 = ins->q[1], q2 = ins->q[2], q3 = ins->q[3];
    float w[3];
    for (int i = 0; i < 3; ++i) {
        w[i] = gyro[i] * DEG_TO_RAD;
        float derivative = (gyro[i] - ins->previous_gyro[i]) / dt;
        ins->angular_accel[i] += dt / (0.1f + dt) * (derivative - ins->angular_accel[i]);
        ins->previous_gyro[i] = gyro[i];
    }
    float norm = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    /* 平动或冲击时不把加速度当重力；正常时用叉积误差修正积分。 */
    if (norm > 0.8f && norm < 1.2f) {
        float a[3] = {accel[0]/norm, accel[1]/norm, accel[2]/norm};
        float v[3] = {q1*q3-q0*q2, q0*q1+q2*q3, q0*q0-0.5f+q3*q3};
        w[0] += 1.6f * (a[1]*v[2] - a[2]*v[1]);
        w[1] += 1.6f * (a[2]*v[0] - a[0]*v[2]);
        w[2] += 1.6f * (a[0]*v[1] - a[1]*v[0]);
    }
    float half_dt = 0.5f * dt;
    ins->q[0] += (-q1*w[0] - q2*w[1] - q3*w[2]) * half_dt;
    ins->q[1] += ( q0*w[0] + q2*w[2] - q3*w[1]) * half_dt;
    ins->q[2] += ( q0*w[1] - q1*w[2] + q3*w[0]) * half_dt;
    ins->q[3] += ( q0*w[2] + q1*w[1] - q2*w[0]) * half_dt;
    float n = sqrtf(ins->q[0]*ins->q[0] + ins->q[1]*ins->q[1] +
                    ins->q[2]*ins->q[2] + ins->q[3]*ins->q[3]);
    for (int i = 0; i < 4; ++i) ins->q[i] /= n;
    q0=ins->q[0]; q1=ins->q[1]; q2=ins->q[2]; q3=ins->q[3];
    ins->roll = atan2f(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2)) * RAD_TO_DEG;
    ins->pitch = asinf(fmaxf(-1, fminf(1, 2*(q0*q2-q3*q1)))) * RAD_TO_DEG;
    ins->yaw = atan2f(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3)) * RAD_TO_DEG;
}
