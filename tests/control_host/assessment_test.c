#include "control.h"
#include "imu.h"
#include "axis.h"
#include "motor.h"
#include "vofa.h"
#include "app_config.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t tick;
static IMU_Data imu;
static TIM_HandleTypeDef pwm, encoder;
static UART_HandleTypeDef uart;
static uint8_t *rx_byte;
static float rpm, shaft_angle, base_angle, previous_base, values[7];
static int no_encoder;
static char output[8192];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *p,int ch) { (void)p;(void)ch;return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *p,int ch) { (void)p;(void)ch;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *p,uint8_t *s,uint16_t n)
{
    size_t used=strlen(output);
    if (used+n<sizeof(output)) { memcpy(output+used,s,n);output[used+n]=0; }
    HAL_UART_TxCpltCallback(p);return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *p,uint8_t *b,uint16_t n)
{ (void)p;(void)n;rx_byte=b;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *p) { (void)p;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *p) { (void)p;return HAL_OK; }
void HAL_NVIC_SetPriority(int a,int b,int c) { (void)a;(void)b;(void)c; }
void HAL_NVIC_EnableIRQ(int a) { (void)a; }
void HAL_GPIO_WritePin(int port,int pin,int value) { (void)port;(void)pin;(void)value; }
void Error_Handler(void) { abort(); }
const IMU_Data *IMU_GetData(void) { return &imu; }
void IMU_Recalibrate(void) { imu.valid=0;Axis_Cancel(); }
static void check(int ok,const char *message)
{ if (!ok) { printf("FAIL %s t=%lu rpm=%.2f angle=%.2f mode=%.0f\n",message,(unsigned long)tick,rpm,Axis_Angle(),values[5]);exit(1); } }
static void send(const char *command)
{
    while (*command) { *rx_byte=(uint8_t)*command++;HAL_UART_RxCpltCallback(&uart); }
}
static void run(unsigned int milliseconds)
{
    for (unsigned int n=0;n<milliseconds;++n,++tick) {
        rpm+=(Motor_Output()*0.05f-rpm)*0.01f;
        shaft_angle+=rpm*6*0.001f;
        int32_t counts=(int32_t)lroundf(shaft_angle/360*APP_ENCODER_CPR);
        encoder.Counter=(uint16_t)(65530-(no_encoder?0:counts));
        if (tick%5==0) {
            const float axis[3]=APP_SHAFT_AXIS;
            float theta=(shaft_angle+base_angle)*0.01745329252f;
            float w=rpm*6+(base_angle-previous_base)/0.005f;
            previous_base=base_angle;
            for (int i=0;i<3;++i) imu.gyro[i]=axis[i]*w;
            float c=cosf(theta),s=sinf(theta);
            imu.accel[0]=-s*axis[1]+axis[0]*axis[2]*(1-c);
            imu.accel[1]= s*axis[0]+axis[1]*axis[2]*(1-c);
            imu.accel[2]=c+axis[2]*axis[2]*(1-c);
            imu.tick=tick;
            Axis_Update(imu.gyro,imu.accel,0.005f);
        }
        VOFA_Task();Control_Task();Control_Values(values);
        check(abs(Motor_Output())<=APP_PWM_LIMIT,"PWM bounded");
    }
}
int main(int argc,char **argv)
{
    check(argc==2,"scenario argument");
    encoder.Counter=65530;imu.valid=1;imu.accel[2]=1;
    Control_Start(&pwm,&encoder,&uart);
    send("p\n");run(100);
    check(Motor_Output()==0,"no automatic motion");
    if (!strcmp(argv[1],"speed")) {
        send("speed:3\n");run(8000);check(fabsf(rpm-3)<0.6f,"+speed tracking");
        check(fabsf(values[3]-rpm)<0.3f,"encoder speed including wrap");
        send("speed:-3\n");run(8000);check(fabsf(rpm+3)<0.6f,"-speed tracking");
        run(25000);check((int)values[5]==CONTROL_SPEED,"no arbitrary speed timeout/travel stop");
        send("speed:nan\n");run(50);check((int)values[5]==CONTROL_SPEED,"reject NaN without changing mode");
    } else if (!strcmp(argv[1],"pwm")) {
        send("pwm:300\n");run(500);check(Motor_Output()==300,"PWM command prefix not consumed as p");
        send("pwm:-300\n");run(30);check(Motor_Output()==0,"reversal dead time");
        run(200);check(Motor_Output()==-300,"reverse PWM");
        send("angle:10\n");run(30);check((int)values[5]==CONTROL_PWM,"angle requires zero reference");
    } else if (!strcmp(argv[1],"angle")) {
        send("z");run(2400);check((int)values[5]==CONTROL_ANGLE,"bare z and stable zero window");
        send("angle:5\n");run(8000);check(fabsf(Axis_Angle()-5)<1,"+angle tracking");
        send("angle:-5\n");run(8000);check(fabsf(Axis_Angle()+5)<1,"-angle tracking");
        send("angle:0\n");run(8000);
        for (int i=0;i<1000;++i) { base_angle=5*(i+1)/1000.0f;run(1); }
        run(10000);check(fabsf(Axis_Angle())<1,"base tilt compensation");
    } else if (!strcmp(argv[1],"fault")) {
        send("z\n");run(2200);send("angle:5\n");run(1000);
        imu.valid=0;run(50);check((int)values[5]==CONTROL_STOPPED && Motor_Output()==0,"IMU loss stops angle");
        imu.valid=1;run(1000);check((int)values[5]==CONTROL_STOPPED,"no auto restart after fault");
    } else if (!strcmp(argv[1],"stall")) {
        no_encoder=1;send("speed:3\n");run(3500);
        check((int)values[5]==CONTROL_STOPPED && Motor_Output()==0,"stall with absent feedback");
    } else if (!strcmp(argv[1],"serial")) {
        send("cpr:4096\n");run(50);check(strstr(output,"CPR UPDATED")!=0,"cpr prefix not consumed as c");
        send("speed:3\n");run(100);send("angle:999\n");run(50);
        check((int)values[5]==CONTROL_SPEED,"invalid command preserves active mode");
        send("unfinishedq");run(5);check(Motor_Output()==0 && (int)values[5]==CONTROL_STOPPED,"q interrupts unfinished line");
    } else check(0,"unknown scenario");
    send("q");run(20);check(Motor_Output()==0,"emergency stop");
    printf("PASS %s\n",argv[1]);
}
