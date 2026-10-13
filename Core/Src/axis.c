#include "axis.h"
#include "app_config.h"
#include <math.h>
#include <string.h>

/* 将重力投影到输出轴的法平面，避免斜装时多个欧拉角同时变化。
 * 正角对应正PWM；水平零点必须由实物摆平后建立。 */
static float shaft[3] = APP_SHAFT_AXIS;
static float reference[3], angle, rate;
static int valid;
static float dot(const float a[3], const float b[3])
{ return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
static int normalize(float v[3])
{
    float norm = sqrtf(dot(v,v));
    if (!(norm > 0.001f) || !isfinite(norm)) return 0;
    for (int i=0; i<3; ++i) v[i] /= norm;
    return 1;
}
static float wrap(float value)
{
    while (value > 180) value -= 360;
    while (value < -180) value += 360;
    return value;
}
static int project(const float gravity[3], float out[3])
{
    float along = dot(gravity,shaft);
    for (int i=0; i<3; ++i) out[i] = gravity[i] - along*shaft[i];
    return sqrtf(dot(out,out)) > 0.25f && normalize(out);
}
void Axis_Cancel(void) { valid=0; angle=rate=0; }
int Axis_SetDirection(const float direction[3])
{
    float next[3]; memcpy(next,direction,sizeof(next));
    if (!normalize(next)) return 0;
    memcpy(shaft,next,sizeof(shaft));
    Axis_Cancel();
    return 1;
}
int Axis_Zero(const float gravity[3])
{
    if (!normalize(shaft) || !project(gravity,reference)) return 0;
    valid=1; angle=rate=0;
    return 1;
}
void Axis_Update(const float gyro[3], const float accel[3], float dt)
{
    if (!valid) return;
    rate=dot(gyro,shaft);
    angle=wrap(angle + rate*dt);
    float norm=sqrtf(dot(accel,accel)), current[3];
    if (norm > 0.8f && norm < 1.2f && project(accel,current)) {
        float cross[3]={current[1]*reference[2]-current[2]*reference[1],
                        current[2]*reference[0]-current[0]*reference[2],
                        current[0]*reference[1]-current[1]*reference[0]};
        float measured=atan2f(dot(shaft,cross),dot(current,reference))*57.29577951f;
        angle=wrap(angle + dt/(0.25f+dt)*wrap(measured-angle));
    }
}
float Axis_Angle(void) { return angle; }
float Axis_Rate(void) { return rate; }
int Axis_Ready(void) { return valid; }
