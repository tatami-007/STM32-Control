/* 参考：jiangjiaxu222/mpu6050-code 的mpu6050.c/ins.c。
 * 移植变更：显式错误返回、有效样本窗口校准、兼容WHO_AM_I=0x70，
 * UART1非阻塞输出由vofa.c负责，避免通信失败被当成零数据。 */
#include "imu.h"
#include "axis.h"
#include "control.h"
#include "imu_calibration.h"
#include "vofa.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define ADDRESS (0x68U << 1) /* AD0接地；HAL要求7位地址左移一位 */
#define PERIOD_MS 5U
static I2C_HandleTypeDef *bus;
static IMU_Data measurement;
static IMU_Calibration calibration;
static float bias[3];
static int initialized, calibrated, init_phase;
static uint32_t next_tick, previous_tick, report_tick;
static uint8_t sensor_id, last_register;
static HAL_StatusTypeDef last_status;

static int write_reg(uint8_t reg, uint8_t value)
{
    last_register=reg;
    last_status=HAL_I2C_Mem_Write(bus,ADDRESS,reg,I2C_MEMADD_SIZE_8BIT,&value,1,20);
    return last_status==HAL_OK;
}
static int read_reg(uint8_t reg, uint8_t *bytes, uint16_t count)
{
    last_register=reg;
    last_status=HAL_I2C_Mem_Read(bus,ADDRESS,reg,I2C_MEMADD_SIZE_8BIT,bytes,count,20);
    return last_status==HAL_OK;
}
static int16_t decode(const uint8_t *p)
{
    int32_t value=((uint32_t)p[0]<<8)|p[1];
    return (int16_t)(value>=32768?value-65536:value);
}
void IMU_Recalibrate(void)
{
    measurement.valid=0; calibrated=0;
    IMU_CalibrationReset(&calibration);
    Axis_Cancel();
    VOFA_Print("IMU CALIBRATING: KEEP STILL\r\n");
}
static int initialize(void)
{
    /* 复位等待拆成状态步骤，不用HAL_Delay阻塞正在运行的速度环。 */
    if (init_phase==0) {
        if (!read_reg(0x75,&sensor_id,1)) return -1;
        if (sensor_id!=0x68 && sensor_id!=0x70) {
            VOFA_Print("IMU UNSUPPORTED ID\r\n"); return -1;
        }
        if (!write_reg(0x6B,0x80)) return -1;
        init_phase=1; next_tick=HAL_GetTick()+100U;
        return 0;
    }
    /* PLL时钟、所有轴开启；±2g/±250deg/s；DLPF；1kHz/(1+4)=200Hz。 */
    static const uint8_t registers[]={0x6B,0x6C,0x1A,0x1B,0x1C,0x19};
    static const uint8_t values[]={1,0,3,0,0,4};
    if (init_phase==1) {
        for (unsigned int i=0; i<sizeof(registers); ++i)
            if (!write_reg(registers[i],values[i])) return -1;
        if (sensor_id==0x70 && !write_reg(0x1D,3)) return -1;
        init_phase=2; next_tick=HAL_GetTick()+100U;
        return 0;
    }
    /* 读回量程/采样率，防止写入失败或量程不符导致软件换算错误。 */
    for (unsigned int i=0; i<sizeof(registers); ++i) {
        uint8_t actual;
        if (!read_reg(registers[i],&actual,1)) return -1;
        if (actual!=values[i]) {
            VOFA_Print("IMU REGISTER VERIFY FAILED\r\n"); return -1;
        }
    }
    char message[80];
    snprintf(message,sizeof(message),"IMU ID=0x%02X CONFIG VERIFIED; 200Hz\r\n",sensor_id);
    VOFA_Print(message);
    IMU_Recalibrate();
    init_phase=0;
    return 1;
}
void IMU_Start(I2C_HandleTypeDef *i2c, UART_HandleTypeDef *uart)
{
    (void)uart; bus=i2c; next_tick=HAL_GetTick();
}
void IMU_Task(void)
{
    uint32_t now=HAL_GetTick();
    if ((int32_t)(now-next_tick)<0) return;
    next_tick=now+PERIOD_MS;
    if (!initialized) {
        int result=initialize();
        if (result<0) goto failure;
        if (!result) return;
        initialized=1; previous_tick=HAL_GetTick();
        next_tick=previous_tick+PERIOD_MS;
        return;
    }
    uint8_t bytes[14]; /* 连续读取加速度6字节、温度2字节、角速度6字节 */
    if (!read_reg(0x3B,bytes,sizeof(bytes))) goto failure;
    now=HAL_GetTick();
    float dt=(now-previous_tick)*0.001f;
    previous_tick=now;
    if (!(dt>0 && dt<=0.1f)) {
        Control_SensorFault(); IMU_Recalibrate(); return;
    }
    for (int i=0; i<3; ++i) {
        measurement.accel[i]=decode(bytes+i*2)/16384.0f;
        measurement.gyro[i]=decode(bytes+8+i*2)/131.0f;
    }
    measurement.tick=now;
    if (!calibrated) {
        const char *result=IMU_CalibrationPush(&calibration,measurement.accel,measurement.gyro);
        if (!strcmp(result,"OK")) {
            memcpy(bias,calibration.gyro_mean,sizeof(bias));
            INS_Init(&measurement.attitude,calibration.accel_mean);
            calibrated=1;
            VOFA_Print("IMU CALIBRATION OK\r\n");
        } else {
            if (now-report_tick>=1000U) {
                char message[140]; report_tick=now;
                snprintf(message,sizeof(message),"IMU CAL %u/400 %s gyro_std_mdps=%ld gyro_drift_mdps=%ld\r\n",
                         calibration.count,result,(long)lroundf(calibration.gyro_std*1000),
                         (long)lroundf(calibration.gyro_drift*1000));
                VOFA_Print(message);
            }
            if (strcmp(result,"COLLECTING")) IMU_CalibrationReset(&calibration);
            return;
        }
    }
    for (int i=0; i<3; ++i) measurement.gyro[i]-=bias[i];
    measurement.valid=1;
    INS_Update(&measurement.attitude,measurement.accel,measurement.gyro,dt);
    Axis_Update(measurement.gyro,measurement.accel,dt);
    return;
failure:
    /* 传感器故障不提供伪造零姿态，角度模式停机后须手动重新启动。 */
    Control_SensorFault(); measurement.valid=0; initialized=calibrated=init_phase=0;
    char message[120];
    snprintf(message,sizeof(message),"IMU READ/WRITE FAILED reg=0x%02X status=%u error=0x%08lX\r\n",
             last_register,(unsigned int)last_status,(unsigned long)HAL_I2C_GetError(bus));
    VOFA_Print(message);
    HAL_I2C_DeInit(bus);
    __HAL_RCC_I2C1_FORCE_RESET(); __HAL_RCC_I2C1_RELEASE_RESET();
    if (HAL_I2C_Init(bus)!=HAL_OK) VOFA_Print("I2C REINIT FAILED\r\n");
    IMU_CalibrationReset(&calibration);
    next_tick=HAL_GetTick()+1000U;
}
const IMU_Data *IMU_GetData(void) { return &measurement; }
