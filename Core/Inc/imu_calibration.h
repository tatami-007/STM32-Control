#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H
#include <math.h>
#include <string.h>

#define IMU_CALIBRATION_SAMPLES 400U
typedef struct {
  unsigned int count;
  float accel_mean[3], gyro_mean[3], accel_m2[3], gyro_m2[3];
  float accel_first_half[3], gyro_first_half[3];
  float accel_std, gyro_std, accel_drift, gyro_drift, mean_norm;
} IMU_Calibration;

static inline void IMU_CalibrationReset(IMU_Calibration *c)
{
  memset(c, 0, sizeof(*c));
}

/* 400样本窗口（200Hz约2秒）：允许零均值噪声，拒绝运动、漂移及异常幅值。
 * 单独的IMU无法区分恒定低速转动和零偏；必须在静止状态启动。
 * 返回COLLECTING、OK或失败原因；失败后由调用者清空窗口。 */
static inline const char *IMU_CalibrationPush(IMU_Calibration *c,
    const float a[3], const float w[3])
{
  float norm2=0.0f;
  for (int i=0; i<3; ++i) {
    if (!isfinite(a[i]) || !isfinite(w[i])) return "INVALID_SAMPLE";
    norm2+=a[i]*a[i];
    if (fabsf(w[i])>15.0f) return "MOTION";
  }
  float norm=sqrtf(norm2);
  if (norm<0.6f || norm>1.4f) return "ACCEL_OUTLIER";
  ++c->count;
  c->accel_std=c->gyro_std=0.0f;
  for (int i=0; i<3; ++i) {
    float da=a[i]-c->accel_mean[i], dw=w[i]-c->gyro_mean[i];
    c->accel_mean[i]+=da/c->count;
    c->gyro_mean[i]+=dw/c->count;
    c->accel_m2[i]+=da*(a[i]-c->accel_mean[i]);
    c->gyro_m2[i]+=dw*(w[i]-c->gyro_mean[i]);
    if (c->count<=IMU_CALIBRATION_SAMPLES/2) {
      c->accel_first_half[i]+=a[i];
      c->gyro_first_half[i]+=w[i];
    }
    if (c->count>1) {
      c->accel_std=fmaxf(c->accel_std,sqrtf(fmaxf(0.0f,c->accel_m2[i]/(c->count-1))));
      c->gyro_std=fmaxf(c->gyro_std,sqrtf(fmaxf(0.0f,c->gyro_m2[i]/(c->count-1))));
    }
  }
  c->mean_norm=sqrtf(c->accel_mean[0]*c->accel_mean[0]+
      c->accel_mean[1]*c->accel_mean[1]+c->accel_mean[2]*c->accel_mean[2]);
  if (c->count<IMU_CALIBRATION_SAMPLES) return "COLLECTING";
  float accel_drift2=0.0f, gyro_mean_peak=0.0f;
  for (int i=0; i<3; ++i) {
    /* 全窗平均=(前半窗平均+后半窗平均)/2。 */
    float da=2.0f*(c->accel_mean[i]-c->accel_first_half[i]/(IMU_CALIBRATION_SAMPLES/2));
    float dw=2.0f*(c->gyro_mean[i]-c->gyro_first_half[i]/(IMU_CALIBRATION_SAMPLES/2));
    accel_drift2+=da*da;
    c->gyro_drift=fmaxf(c->gyro_drift,fabsf(dw));
    gyro_mean_peak=fmaxf(gyro_mean_peak,fabsf(c->gyro_mean[i]));
  }
  c->accel_drift=sqrtf(accel_drift2);
  if (c->mean_norm<0.8f || c->mean_norm>1.2f) return "ACCEL_NORM";
  if (gyro_mean_peak>5.0f) return "GYRO_MEAN_HIGH";
  if (c->gyro_std>2.0f) return "GYRO_NOISY";
  if (c->accel_std>0.025f) return "ACCEL_NOISY";
  if (c->gyro_drift>1.0f) return "GYRO_DRIFT";
  if (c->accel_drift>0.04f) return "ACCEL_DRIFT";
  return "OK";
}
#endif
