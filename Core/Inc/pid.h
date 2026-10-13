#ifndef PID_H
#define PID_H
/* 输出限幅 + 条件积分抗饱和；D对测量值求导，避免目标阶跃冲击。 */
typedef struct {
    float kp, ki, kd, limit;
    float integral, previous;
    int primed;
} PID;
void PID_Init(PID *p, float kp, float ki, float kd, float limit);
void PID_Reset(PID *p);
float PID_Update(PID *p, float target, float actual, float dt);
#endif
