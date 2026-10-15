/**
 * @file mpu6050.h
 * @brief MPU6050 数据类型与驱动接口。
 */

#pragma once

#include <stdint.h>

/** @brief 换算为物理单位的传感器数据，各轴按 X、Y、Z 排列。 */
typedef struct {
    uint8_t valid;    /**< 本次读取是否成功，1 为有效，0 为无效 */
    float accel[3];  /**< 加速度，单位 g */
    float gyro[3];   /**< 角速度，单位 °/s（已去除零偏） */
    float temp;      /**< 温度，单位 ℃ */
} bsp_mpu6050_data_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化传感器，配置 ±2 g、±250 °/s 量程及数字低通滤波。
 * @return 成功返回 1，设备标识不支持或通信失败返回 0。
 */
uint8_t bsp_mpu6050_init(void);

/**
 * @brief 读取 MPU6050 数据。
 * @return 加速度、去零偏角速度与温度；通信失败时 valid 为 0。
 */
bsp_mpu6050_data_t bsp_mpu6050_read(void);

/**
 * @brief 采样求取陀螺仪零偏，调用期间须保持传感器静止。
 * @param samples 采样次数，须大于 0，每次采样间隔 10 ms。
 * @return 成功返回 1；采样次数为 0 或通信失败返回 0，保留原零偏。
 */
uint8_t bsp_mpu6050_calibrate(uint16_t samples);

#ifdef __cplusplus
}
#endif
