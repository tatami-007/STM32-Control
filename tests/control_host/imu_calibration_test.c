#include "imu_calibration.h"
#include <stdio.h>
#include <stdlib.h>

static void check(int condition, const char *name)
{
  if (!condition) { printf("FAIL: %s\n",name); exit(1); }
}
static const char *window(int mode)
{
  static IMU_Calibration c;
  IMU_CalibrationReset(&c);
  const char *result="";
  for (unsigned int n=0;n<IMU_CALIBRATION_SAMPLES;++n) {
    float a[3]={0,0,1.0f},w[3]={0,0,0};
    if (mode==0) {
      a[2]=1.15f+(n%2?0.014f:-0.014f);
      w[0]=1.5f+(n%2?1.5f:-1.5f);
      if (n==30) { a[2]=1.205f; w[0]=5.3f; }
    }
    if (mode==1) w[0]=6.0f; /* 持续转动 */
    if (mode==2) w[0]=n%2?3.0f:-3.0f; /* 大噪声 */
    if (mode==3) w[0]=n<IMU_CALIBRATION_SAMPLES/2?-0.6f:0.6f; /* 前后均值漂移 */
    if (mode==4) a[2]=1.25f; /* 不合理的平均重力幅值 */
    if (mode==5) a[0]=n%2?0.04f:-0.04f; /* 加速度振动 */
    if (mode==6) a[0]=n<IMU_CALIBRATION_SAMPLES/2?-0.023f:0.023f; /* 缓慢倾斜 */
    result=IMU_CalibrationPush(&c,a,w);
    if (n<IMU_CALIBRATION_SAMPLES-1) check(strcmp(result,"COLLECTING")==0,"requires complete window");
  }
  if (mode==0) check(fabsf(c.gyro_mean[0]-(1.5f+5.3f/IMU_CALIBRATION_SAMPLES))<0.002f,"bias is window mean");
  return result;
}
int main(void)
{
  check(strcmp(window(0),"OK")==0,"stationary noise and isolated spikes accepted");
  check(strcmp(window(1),"GYRO_MEAN_HIGH")==0,"sustained rotation rejected");
  check(strcmp(window(2),"GYRO_NOISY")==0,"excessive gyro noise rejected");
  check(strcmp(window(3),"GYRO_DRIFT")==0,"gyro drift rejected");
  check(strcmp(window(4),"ACCEL_NORM")==0,"bad mean gravity rejected");
  check(strcmp(window(5),"ACCEL_NOISY")==0,"shaking rejected");
  check(strcmp(window(6),"ACCEL_DRIFT")==0,"slow tilt rejected");
  IMU_Calibration c;
  IMU_CalibrationReset(&c);
  float a[3]={0,0,1},w[3]={16,0,0};
  check(strcmp(IMU_CalibrationPush(&c,a,w),"MOTION")==0 && c.count==0,"gross motion rejected immediately");
  w[0]=0; a[0]=NAN;
  check(strcmp(IMU_CalibrationPush(&c,a,w),"INVALID_SAMPLE")==0,"nonfinite sample rejected");
  IMU_CalibrationReset(&c);
  a[0]=0;
  check(strcmp(IMU_CalibrationPush(&c,a,w),"COLLECTING")==0 && c.count==1,"reset allows fresh window");
  puts("PASS: calibration accepts noisy rest; rejects motion, drift, bad gravity and invalid samples");
  return 0;
}
