#include "control.h"
#include "main.h"
#include "app_config.h"
#include "imu.h"
#include "imu_calibration.h"
#include "axis.h"
#include "motor.h"
#include "pid.h"
#include "vofa.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static TIM_HandleTypeDef *encoder_timer;
static ControlMode mode;
static PID speed_pid, angle_pid;
static float target_speed, target_angle, actual_speed;
static float encoder_cpr=APP_ENCODER_CPR;
static int encoder_sign=APP_ENCODER_SIGN, manual_pwm, speed_direction;
static int32_t encoder_total, motion_mark;
static uint16_t previous_count;
static uint32_t control_tick, motion_tick, report_tick, telemetry_tick;
static const char *stop_reason="USER START REQUIRED";
static int stream_enabled=1;
static IMU_Calibration zero_window;
static uint32_t zero_tick, zero_sample_tick;
/* 低速静摩擦助力：真正输出PWM后才计时，不能把换向等待算进脉冲。 */
static int boost_active, boost_started, boost_sign;
static unsigned int boost_ms=10;
static uint32_t boost_until;
static uint16_t boost_count;

static int imu_good(uint32_t now)
{
    const IMU_Data *data=IMU_GetData();
    return data->valid && now-data->tick<=100U;
}
static void reset_loops(void)
{
    PID_Reset(&speed_pid); PID_Reset(&angle_pid);
    speed_direction=0;
    boost_active=boost_started=0; boost_ms=10;
    motion_mark=encoder_total; motion_tick=HAL_GetTick();
}
void Control_Stop(const char *reason)
{
    Motor_Stop();
    mode=CONTROL_STOPPED; stop_reason=reason;
    target_speed=0; manual_pwm=0;
    reset_loops();
    VOFA_Print("STOP: "); VOFA_Print(reason); VOFA_Print("\r\n");
}
void Control_SensorFault(void)
{
    /* 速度环依赖编码器；姿态故障仅禁止需要IMU的角度模式。 */
    if (mode==CONTROL_ANGLE || mode==CONTROL_ZERO) Control_Stop("IMU LOST");
    Axis_Cancel();
}
void Control_SetStream(int enabled) { stream_enabled=enabled!=0; }
void Control_Values(float out[7])
{
    out[0]=target_angle; out[1]=Axis_Angle();
    out[2]=target_speed; out[3]=actual_speed;
    out[4]=Motor_Output()/10.0f; out[5]=(float)mode;
    out[6]=(float)encoder_total*encoder_sign;
}
static void start_mode(ControlMode next, float value)
{
    if (next==CONTROL_ANGLE && (!Axis_Ready() || !imu_good(HAL_GetTick()))) {
        VOFA_Print("REJECTED: PLACE LEVEL AND SEND z FIRST\r\n"); return;
    }
    Motor_Stop(); reset_loops();
    mode=next;
    if (next==CONTROL_ANGLE) target_angle=value;
    if (next==CONTROL_SPEED) target_speed=value;
    if (next==CONTROL_PWM) manual_pwm=(int)lroundf(value);
    VOFA_Print("CONTROL STARTED\r\n");
}
/* 完整数字才接受，拒绝NaN、Inf和带垃圾后缀的指令。 */
static int numbers(const char *input, float *out, int count)
{
    for (int i=0; i<count; ++i) {
        char *end;
        out[i]=strtof(input,&end);
        if (end==input || !isfinite(out[i])) return 0;
        input=end;
        while (*input==' ') ++input;
        if (i+1<count) { if (*input++!=',') return 0; }
    }
    return *input=='\0';
}
void Control_Command(const char *command)
{
    if (!strcmp(command,"0") || !strcmp(command,"q") || !strcmp(command,"stop")) {
        Control_Stop("USER STOP"); return;
    }
    if (!strcmp(command,"p") || !strcmp(command,"P")) {
        stream_enabled=0; VOFA_Print("VALUES PAUSED\r\n"); return;
    }
    if (!strcmp(command,"v") || !strcmp(command,"V")) {
        stream_enabled=1; VOFA_Print("VALUES RESUMED\r\n"); return;
    }
    if (!strcmp(command,"h") || !strcmp(command,"H")) {
        Control_Stop("PLACE LEVEL, THEN z"); return;
    }
    if (!strcmp(command,"c") || !strcmp(command,"C")) {
        Control_Stop("IMU RECALIBRATION"); IMU_Recalibrate(); return;
    }
    if (!strcmp(command,"z") || !strcmp(command,"Z")) {
        Control_Stop("COLLECTING HORIZONTAL ZERO");
        if (!imu_good(HAL_GetTick())) { VOFA_Print("REJECTED: WAIT IMU READY\r\n"); return; }
        IMU_CalibrationReset(&zero_window);
        mode=CONTROL_ZERO; zero_tick=HAL_GetTick(); zero_sample_tick=IMU_GetData()->tick;
        VOFA_Print("ZERO: KEEP LEVEL AND STILL FOR 2 SECONDS\r\n"); return;
    }
    if (!strcmp(command,"d") || !strcmp(command,"D") || !strcmp(command,"hold")) {
        start_mode(CONTROL_ANGLE,0); return; /* 恢复原零点，不擅自重新设零。 */
    }
    if (!strcmp(command,"enczero")) {
        Control_Stop("ENCODER ORIGIN RESET"); encoder_total=0;
        previous_count=(uint16_t)__HAL_TIM_GET_COUNTER(encoder_timer);
        actual_speed=0; return;
    }
    float value[3];
    const char *colon=strchr(command,':');
    if (!colon) { VOFA_Print("COMMANDS: z q p v c pwm:300 speed:3 angle:5 cpr:2112\r\n"); return; }
    if (!strncmp(command,"speedpid:",9) || !strncmp(command,"anglepid:",9)) {
        if (!numbers(colon+1,value,3) || value[0]<0 || value[1]<0 || value[2]<0 ||
            value[0]>1000 || value[1]>1000 || value[2]>1000) goto invalid;
        Control_Stop("PID UPDATED");
        int is_speed=command[0]=='s';
        PID_Init(is_speed?&speed_pid:&angle_pid,value[0],value[1],value[2],
                 is_speed?APP_PWM_LIMIT:APP_ANGLE_SPEED_LIMIT);
        return;
    }
    if (!strncmp(command,"axis:",5)) {
        if (!numbers(colon+1,value,3)) goto invalid;
        Control_Stop("AXIS UPDATED; LEVEL AND z AGAIN");
        if (!Axis_SetDirection(value)) goto invalid;
        return;
    }
    if (!numbers(colon+1,value,1)) goto invalid;
    if (!strncmp(command,"pwm:",4) && fabsf(value[0])<=APP_PWM_LIMIT)
        start_mode(CONTROL_PWM,value[0]);
    else if (!strncmp(command,"speed:",6) && fabsf(value[0])<=APP_MAX_SPEED_RPM)
        start_mode(CONTROL_SPEED,value[0]);
    else if (!strncmp(command,"angle:",6) && fabsf(value[0])<=APP_MAX_TARGET_ANGLE)
        start_mode(CONTROL_ANGLE,value[0]);
    else if (!strncmp(command,"cpr:",4) && value[0]>=4 && value[0]<=1000000) {
        Control_Stop("CPR UPDATED"); encoder_cpr=value[0]; actual_speed=0;
    } else if (!strncmp(command,"sign:",5) && (value[0]==1 || value[0]==-1)) {
        Control_Stop("ENCODER SIGN UPDATED"); encoder_sign=(int)value[0]; actual_speed=0;
    } else goto invalid;
    return;
invalid:
    VOFA_Print("INVALID COMMAND / VALUE OUT OF RANGE\r\n");
}
void Control_Start(TIM_HandleTypeDef *pwm, TIM_HandleTypeDef *enc, UART_HandleTypeDef *uart)
{
    encoder_timer=enc;
    VOFA_Start(uart); Motor_Start(pwm);
    if (HAL_TIM_Encoder_Start(enc,TIM_CHANNEL_ALL)!=HAL_OK) Error_Handler();
    previous_count=(uint16_t)__HAL_TIM_GET_COUNTER(enc);
    PID_Init(&speed_pid,APP_SPEED_KP,APP_SPEED_KI,APP_SPEED_KD,APP_PWM_LIMIT);
    PID_Init(&angle_pid,APP_ANGLE_KP,APP_ANGLE_KI,APP_ANGLE_KD,APP_ANGLE_SPEED_LIMIT);
    control_tick=HAL_GetTick(); motion_tick=control_tick;
    VOFA_Print("ASSESSMENT BUILD; MOTOR OFF; z=LEVEL ZERO, speed:N, angle:N, pwm:N, q=STOP\r\n");
}
static void zero_task(uint32_t now)
{
    const IMU_Data *data=IMU_GetData();
    if (!imu_good(now) || now-zero_tick>8000U) { Control_Stop("ZERO NOT STABLE / IMU LOST"); return; }
    if (data->tick==zero_sample_tick) return;
    zero_sample_tick=data->tick;
    const char *result=IMU_CalibrationPush(&zero_window,data->accel,data->gyro);
    if (!strcmp(result,"COLLECTING")) return;
    float *w=zero_window.gyro_mean;
    if (!strcmp(result,"OK") && sqrtf(w[0]*w[0]+w[1]*w[1]+w[2]*w[2])<1.5f) {
        if (!Axis_Zero(zero_window.accel_mean)) { Control_Stop("SHAFT TOO VERTICAL"); return; }
        start_mode(CONTROL_ANGLE,0);
        VOFA_Print("HORIZONTAL ZERO SET; HOLD ACTIVE\r\n");
    } else IMU_CalibrationReset(&zero_window);
}
static void telemetry(uint32_t now)
{
    if (now-report_tick>=1000U) {
        report_tick=now;
        static const char *names[]={"STOPPED","PWM","SPEED","ANGLE","ZERO"};
        char line[200];
        snprintf(line,sizeof(line),"CTRL state=%s imu=%d axis=%d target_mdeg=%ld angle_mdeg=%ld pwm=%d counts=%ld reason=%s\r\n",
                 names[mode],imu_good(now),Axis_Ready(),(long)lroundf(target_angle*1000),
                 (long)lroundf(Axis_Angle()*1000),Motor_Output(),
                 (long)encoder_total*encoder_sign,mode==CONTROL_STOPPED?stop_reason:"NONE");
        VOFA_Print(line);
    }
    if (stream_enabled && now-telemetry_tick>=APP_TELEMETRY_PERIOD_MS) {
        telemetry_tick=now;
        const IMU_Data *data=IMU_GetData();
        float frame[22]={0};
        for (int i=0; i<3; ++i) {
            frame[i]=data->accel[i]; frame[3+i]=data->gyro[i];
            frame[18+i]=data->attitude.angular_accel[i];
        }
        frame[6]=data->attitude.roll; frame[7]=data->attitude.pitch;
        frame[8]=Axis_Angle(); frame[9]=Axis_Rate(); frame[10]=(float)Axis_Ready();
        Control_Values(frame+11); frame[21]=data->attitude.yaw;
        VOFA_Frame(frame,22);
    }
}
void Control_Task(void)
{
    uint32_t now=HAL_GetTick();
    if (boost_active && boost_started && (int32_t)(now-boost_until)>=0) {
        uint16_t current=(uint16_t)__HAL_TIM_GET_COUNTER(encoder_timer);
        boost_ms=current==boost_count ? (boost_ms<40?boost_ms+10:40) : 10;
        boost_active=boost_started=0; Motor_Stop();
    }
    Motor_Task();
    if (boost_active && !boost_started && Motor_Output()*boost_sign>0) {
        boost_started=1; boost_until=now+boost_ms;
        boost_count=(uint16_t)__HAL_TIM_GET_COUNTER(encoder_timer);
    }
    if (mode==CONTROL_ZERO) zero_task(now);
    if (mode==CONTROL_ANGLE && !imu_good(now)) Control_Stop("IMU INVALID / STALE");
    uint32_t elapsed=now-control_tick;
    if (elapsed>=APP_CONTROL_PERIOD_MS) {
        control_tick=now;
        float dt=elapsed*0.001f;
        uint16_t count=(uint16_t)__HAL_TIM_GET_COUNTER(encoder_timer);
        int32_t delta=((uint32_t)count-previous_count)&0xffffU;
        if (delta>=32768) delta-=65536;
        previous_count=count;
        encoder_total+=delta;
        float instant=delta*encoder_sign*60.0f/(encoder_cpr*dt);
        actual_speed+=dt/(0.04f+dt)*(instant-actual_speed);
        if (mode!=CONTROL_STOPPED && mode!=CONTROL_ZERO && elapsed>100U)
            Control_Stop("CONTROL TIMING GAP");
        if (abs(delta)>1000) Control_Stop("ENCODER JUMP");
        if (mode==CONTROL_PWM) Motor_Set(manual_pwm);
        if (mode==CONTROL_SPEED || mode==CONTROL_ANGLE) {
            if (mode==CONTROL_ANGLE) {
                target_speed=PID_Update(&angle_pid,target_angle,Axis_Angle(),dt);
                if (fabsf(target_angle-Axis_Angle())<APP_ANGLE_DEADBAND && fabsf(actual_speed)<0.5f) {
                    Motor_Stop(); reset_loops(); target_speed=0; goto done;
                }
            }
            int direction=target_speed>0.3f?1:target_speed< -0.3f?-1:0;
            if (direction!=speed_direction) {
                PID_Reset(&speed_pid); speed_direction=direction;
                boost_active=boost_started=0;
            }
            int pwm=(int)lroundf(PID_Update(&speed_pid,target_speed,actual_speed,dt));
            if (boost_active) pwm=boost_sign*APP_PWM_LIMIT;
            else if (direction && fabsf(actual_speed)<0.3f) {
                boost_active=1; boost_started=0; boost_sign=direction;
                pwm=direction*APP_PWM_LIMIT;
            }
            if (!direction || abs(encoder_total-motion_mark)>=2 || abs(pwm)<APP_PWM_LIMIT) {
                motion_mark=encoder_total; motion_tick=now;
            }
            /* 仅在持续最高输出且编码器确实没有进展时判堵转，
             * 低占空比尚未克服静摩擦时不制造1.2秒误停。 */
            if (direction && now-motion_tick>APP_STALL_TIMEOUT_MS)
                Control_Stop("STALL AT PWM LIMIT / CHECK POWER AND ENCODER");
            else Motor_Set(pwm);
        }
    }
done:
    telemetry(now);
}
