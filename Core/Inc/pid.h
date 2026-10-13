#ifndef PID_H
#define PID_H
/* 一个PID保存自己的一份参数和历史数据，速度环、角度环各用一份。 */
typedef struct {
    float kp, ki, kd;       // P、I、D三个参数
    float limit;            // 输出允许的最大绝对值
    float integral;         // 已累加的I部分
    float last_error;       // 上一次的误差，用于D部分
} PID;
void PID_Init(PID *p, float kp, float ki, float kd, float limit);
void PID_Reset(PID *p);
float PID_Update(PID *p, float target, float actual, float dt);
#endif
