#ifndef INS_H
#define INS_H

/* 参考实验室仓库mpu6050/ins.c：Mahony比例反馈 + 四元数积分。
 * 输入加速度[g]、校准角速度[deg/s]，输出欧拉角[deg]。
 * 六轴IMU无磁力计，yaw只能作为相对角，不能当绝对航向。 */
typedef struct {
    float q[4];
    float roll, pitch, yaw;
    float angular_accel[3];
    float previous_gyro[3];
} INS;

void INS_Init(INS *ins, const float accel[3]);
void INS_Update(INS *ins, const float accel[3], const float gyro[3], float dt);
#endif
