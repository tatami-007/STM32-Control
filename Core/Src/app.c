#include "app.h"
#include "main.h"
#include "config.h"
#include "motor.h"
#include "pid.h"
#include "serial.h"
#include "ins.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 仅四种模式，没有自动试转、辨轴、启动脉冲或计时停机流程。 */
enum { STOP, PWM, SPEED, ANGLE };
static int mode, pwm_target, stream = 1, zero_valid;
static TIM_HandleTypeDef *encoder;
static PID speed_pid, angle_pid;
static uint16_t last_count;
static int32_t total_count;
static uint32_t last_control, last_send, last_status;
static float cpr = ENCODER_CPR, speed, target_speed, target_angle, angle_zero;
static const char *message = "BOOT: MOTOR OFF; KEEP IMU STILL\r\n";

static float sensor_angle(void) {
    const ins_data_t *data = ins_data();
    return ANGLE_SIGN * (ANGLE_USE_ROLL ? data->roll : data->pitch);
}
static float angle(void) {
    return sensor_angle() - angle_zero;
}
static void stop(void) {
    Motor_Stop();
    mode = STOP;
    pwm_target = 0;
    target_speed = 0;
    PID_Reset(&speed_pid);
    PID_Reset(&angle_pid);
}
void App_Init(TIM_HandleTypeDef *pwm, TIM_HandleTypeDef *enc, UART_HandleTypeDef *uart) {
    encoder = enc;
    Serial_Init(uart);
    Motor_Start(pwm);
    if (HAL_TIM_Encoder_Start(enc, TIM_CHANNEL_ALL) != HAL_OK) Error_Handler();
    // Gitee封装：初始化量程/采样率，静止采集零偏，再解算姿态。
    message = ins_init() ? "IMU READY; z=ZERO; speed:N; angle:N; pwm:N; q=STOP\r\n"
                         : "IMU INIT FAILED; CHECK POWER/I2C; MOTOR OFF\r\n";
    PID_Init(&speed_pid, 25, 80, 0, PWM_LIMIT);
    PID_Init(&angle_pid, 1.5f, 0, 0.05f, ANGLE_SPEED_MAX);
    last_count = (uint16_t)__HAL_TIM_GET_COUNTER(enc);
    last_control = HAL_GetTick();
}
void App_Command(const char *command) {
    if (!strcmp(command, "q") || !strcmp(command, "0")) {
        stop();
        message = "STOP: USER\r\n";
        return;
    }
    if (!strcmp(command, "p")) {
        stream = 0;
        message = "VALUES PAUSED\r\n";
        return;
    }
    if (!strcmp(command, "v")) {
        stream = 1;
        message = "VALUES RESUMED\r\n";
        return;
    }
    if (!strcmp(command, "h")) {
        stop();
        message = "PLACE LEVEL, THEN z\r\n";
        return;
    }
    if (!strcmp(command, "z")) {
        stop();
        if (!ins_data()->raw.valid || HAL_GetTick() - ins_data()->timestamp > 100U) {
            message = "ZERO FAILED: IMU INVALID\r\n";
            return;
        }
        angle_zero = sensor_angle();
        zero_valid = 1;
        target_angle = 0;
        mode = ANGLE;
        message = "ZERO SET; HOLDING THIS POSITION\r\n";
        return;
    }
    const char *colon = strchr(command, ':');
    if (!colon) {
        message = "USE pwm:300 / speed:3 / angle:5 + NEWLINE\r\n";
        return;
    }
    char *end;
    float value = strtof(colon + 1, &end);
    while (*end == ' ')
        ++end;
    if (end == colon + 1 || *end || !isfinite(value)) {
        message = "INVALID NUMBER\r\n";
        return;
    }
    if (!strncmp(command, "pwm:", 4) && fabsf(value) <= PWM_LIMIT) {
        stop();
        pwm_target = (int)lroundf(value);
        mode = PWM;
    } else if (!strncmp(command, "speed:", 6) && fabsf(value) <= SPEED_LIMIT) {
        stop();
        target_speed = value;
        mode = SPEED;
    } else if (!strncmp(command, "angle:", 6) && fabsf(value) <= ANGLE_LIMIT && zero_valid &&
               ins_data()->raw.valid) {
        stop();
        target_angle = value;
        mode = ANGLE;
    } else if (!strncmp(command, "cpr:", 4) && value >= 4 && value <= 1000000) {
        stop();
        cpr = value;
        speed = 0;
    } else {
        message = "REJECTED: RANGE / NEED z FIRST\r\n";
        return;
    }
    message = "TARGET UPDATED\r\n";
}
void App_Task(void) {
    Serial_Task();
    Motor_Task();
    uint32_t now = HAL_GetTick();
    if (now - last_control >= CONTROL_MS) {
        float dt = (now - last_control) * 0.001f;
        last_control = now;
        ins_update();        // Gitee原代码：I2C读取、Mahony姿态、角加速度
        now = HAL_GetTick(); // I2C结束后的时间，避免比INS时间戳早导致无符号下溢
        uint16_t count = (uint16_t)__HAL_TIM_GET_COUNTER(encoder);
        int32_t delta = ((uint32_t)count - last_count) & 0xffffU;
        if (delta >= 32768) delta -= 65536; // 16位计数溢出也能正确计算正反转
        last_count = count;
        total_count += delta * ENCODER_SIGN;
        float measured = delta * ENCODER_SIGN * 60.0f / (cpr * dt);
        speed += dt / (0.04f + dt) * (measured - speed); // 编码器测速低通

        if (mode == ANGLE && (!ins_data()->raw.valid || now - ins_data()->timestamp > 100U)) {
            stop();
            zero_valid = 0;
            message = "STOP: IMU READ FAILED\r\n";
        }
        if (mode == PWM) Motor_Set(pwm_target);
        if (mode == ANGLE) {
            // 角度外环输出目标RPM；速度内环输出PWM。
            target_speed = PID_Update(&angle_pid, target_angle, angle(), dt);
            if (fabsf(target_angle - angle()) < 0.5f && fabsf(speed) < 0.5f) {
                target_speed = 0;
                PID_Reset(&speed_pid);
                PID_Reset(&angle_pid);
                Motor_Stop();
            }
        }
        if (mode == SPEED || (mode == ANGLE && target_speed != 0))
            Motor_Set((int)lroundf(PID_Update(&speed_pid, target_speed, speed, dt)));
        else if (mode == ANGLE)
            Motor_Stop();
    }
    // 非阻塞发送：状态优先；波形20Hz，控制100Hz，不互相等待。
    if (!Serial_Busy()) {
        if (message) {
            if (Serial_Send(message)) message = 0;
        } else if (stream && now - last_send >= 50U) {
            static char frame[320];
            last_send = now;
            const ins_data_t *data = ins_data();
            snprintf(frame, sizeof(frame),
                     "imu:%.2f,%.2f,%.2f,%.2f,%ld,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%"
                     "d,%u\r\n",
                     target_angle, angle(), target_speed, speed, (long)total_count, Motor_Output(),
                     data->gyro[0], data->gyro[1], data->gyro[2], data->gyro_accel[0],
                     data->gyro_accel[1], data->gyro_accel[2], data->roll, data->pitch, data->yaw,
                     mode, (unsigned int)data->raw.valid);
            Serial_Send(frame);
        } else if (!stream && now - last_status >= 1000U) {
            static char status[100];
            last_status = now;
            snprintf(status, sizeof(status),
                     "CTRL mode=%d target=%.2f angle=%.2f pwm=%d imu=%u\r\n", mode, target_angle,
                     angle(), Motor_Output(), (unsigned int)ins_data()->raw.valid);
            Serial_Send(status);
        }
    }
}
