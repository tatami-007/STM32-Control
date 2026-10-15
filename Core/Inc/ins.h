/**
 * @file ins.h
 * @brief 姿态解算数据与调用接口。
 */

#pragma once

#include <stdint.h>

#include "mpu6050.h"

/** @brief 姿态解算结果，三轴数组按 X、Y、Z 排列。 */
typedef struct {
    float q[4];              /**< 姿态四元数 [w, x, y, z] */
    float roll, pitch, yaw;  /**< 姿态角，单位 ° */
    float yaw_total_angle;   /**< 累计偏航角，单位 °，可超过 ±180 */
    float gyro[3];           /**< 角速度，单位 °/s */
    float gyro_accel[3];     /**< 角加速度，单位 °/s² */
    uint32_t timestamp;      /**< 最近一次成功读取传感器的时间，单位 ms */
    bsp_mpu6050_data_t raw;  /**< 本次传感器数据，valid 为 0 时解算结果不更新 */
} ins_data_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化姿态解算并校准陀螺仪零偏，期间须保持静止。
 * @note 初始化失败时，ins_data()->raw.valid 为 0，须重新初始化。
 */
void ins_init(void);

/**
 * @brief 更新姿态与角加速度，需要周期性调用。
 * @note 读取失败或采样周期无效时将 raw.valid 清零，保留上次解算结果。
 */
void ins_update(void);

/**
 * @brief 获取内部姿态数据。
 * @return 内部数据地址，不得释放；使用前须检查 raw.valid。
 */
ins_data_t *ins_data(void);

/**
 * @brief 获取去零偏角速度。
 * @return 内部三轴数组，依次为 X、Y、Z，单位 °/s；有效性由 raw.valid 标识。
 */
const float *ins_get_gyro(void);

/**
 * @brief 获取滤波后的角加速度。
 * @return 内部三轴数组，依次为 X、Y、Z，单位 °/s²；有效性由 raw.valid 标识。
 */
const float *ins_get_gyro_accel(void);

#ifdef __cplusplus
}
#endif
