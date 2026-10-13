#include "test_main.h"
#include "motor.h"
#include "ins.h"
#include "config.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static uint32_t tick;
static TIM_HandleTypeDef pwm,encoder;
static UART_HandleTypeDef uart;
I2C_HandleTypeDef hi2c1;
static uint8_t *rx;
static float rpm,rotation,base;
static int read_fault, fail_reg = -1;
static char received[4096];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef*);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef*);
uint32_t HAL_GetTick(void){return tick;}
void HAL_Delay(uint32_t n){tick+=n;}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef*p,int c){(void)p;(void)c;return HAL_OK;}
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef*p,int c){(void)p;(void)c;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef*p,uint8_t*b,uint16_t n){(void)p;(void)n;rx=b;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef*p,uint8_t*b,uint16_t n)
{if(strlen(received)+n<sizeof(received))strncat(received,(char*)b,n);HAL_UART_TxCpltCallback(p);return HAL_OK;}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef*p){(void)p;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef*p){(void)p;return HAL_OK;}
void HAL_NVIC_SetPriority(int a,int b,int c){(void)a;(void)b;(void)c;}
void HAL_NVIC_EnableIRQ(int a){(void)a;}
void HAL_GPIO_WritePin(int a,int b,int c){(void)a;(void)b;(void)c;}
void Error_Handler(void){abort();}
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef*p){(void)p;return read_fault||fail_reg>=0?4U:0U;}
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef*p){(void)p;return HAL_OK;}
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef*p){(void)p;return HAL_OK;}
static void encode(uint8_t*p,float value){int16_t n=(int16_t)lroundf(value);p[0]=(uint8_t)((uint16_t)n>>8);p[1]=(uint8_t)n;}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef*p,uint16_t a,uint16_t r,uint16_t s,uint8_t*b,uint16_t n,uint32_t t)
{(void)p;(void)a;(void)r;(void)s;(void)b;(void)n;(void)t;return HAL_OK;}
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef*p,uint16_t a,uint16_t r,uint16_t s,uint8_t*b,uint16_t n,uint32_t t)
{
 (void)p;(void)a;(void)s;(void)t;
 if(read_fault || r==fail_reg)return HAL_ERROR;
 if(r==0x75){b[0]=0x70;return HAL_OK;}
 memset(b,0,n);
 if(r==0x43)return HAL_OK;
 float theta=(rotation+base)*0.01745329252f;
 if(r==0x3b&&n==14){encode(b,sinf(theta)*16384);encode(b+4,cosf(theta)*16384);encode(b+10,-rpm*6*131);}
 ++tick; // 读取结束晚于主循环入口，覆盖时间戳下溢回归
 return HAL_OK;
}
static void check(int ok,const char*s){if(!ok){printf("FAIL %s rpm=%.2f rotation=%.2f pitch=%.2f\n",s,rpm,rotation,ins_data()->pitch);exit(1);}}
static void send(const char*s){while(*s){*rx=(uint8_t)*s++;HAL_UART_RxCpltCallback(&uart);}}
static void run(unsigned int ms)
{
 for(unsigned int i=0;i<ms;++i,++tick){rpm+=(Motor_Output()*0.05f-rpm)*0.01f;rotation+=rpm*6*0.001f;encoder.Counter=(uint16_t)(65530-lroundf(rotation/360*ENCODER_CPR));Test_Loop();check(abs(Motor_Output())<=PWM_LIMIT,"PWM bound");}
}
int main(int argc,char**argv)
{
 check(argc==2,"scenario");
 if(!strcmp(argv[1],"retry"))read_fault=1; // 上电I2C失败，不能假装有姿态
 encoder.Counter=65530;Test_Init(&pwm,&encoder,&uart);send("p\n");run(100);check(Motor_Output()==0,"manual start only");
 if(!strcmp(argv[1],"pwm")){send("pwm:300\n");run(300);check(Motor_Output()==300,"pwm prefix");send("pwm:-300\n");run(30);check(!Motor_Output(),"reverse wait");run(200);check(Motor_Output()==-300,"reverse output");}
 else if(!strcmp(argv[1],"speed")){send("speed:3\n");run(8000);check(fabsf(rpm-3)<0.5f,"forward speed");send("speed:-3\n");run(8000);check(fabsf(rpm+3)<0.5f,"reverse speed");run(35000);check(fabsf(rpm+3)<0.5f,"continuous speed");}
 else if(!strcmp(argv[1],"angle")){send("z\n");run(100);send("angle:5\n");run(8000);check(fabsf(rotation-5)<1,"angle forward");send("angle:-5\n");run(8000);check(fabsf(rotation+5)<1,"angle reverse");send("angle:0\n");run(8000);base=5;run(10000);check(fabsf(rotation+base)<1,"tilt compensation");}
 else if(!strcmp(argv[1],"fault")){send("z\n");run(50);send("angle:5\n");run(1000);read_fault=1;run(50);check(Motor_Output()==0,"read failure stops angle");read_fault=0;run(1000);check(Motor_Output()==0,"does not auto resume");}
 else if(!strcmp(argv[1],"serial")){send("pwm:300\n");run(200);check(strstr(received,"TARGET SET")!=0,"command received");send("speed:nan\n");run(30);check(Motor_Output()==300,"reject NaN");send("unfinishedq");run(5);check(Motor_Output()==0,"q in unfinished line");}
 else if(!strcmp(argv[1],"retry")){
   send("z\n");run(20);check(strstr(received,"reg=0x75 status=1 error=0x00000004")!=0,"failure diagnostic");
   send("angle:5\n");run(20);check(!Motor_Output()&&strstr(received,"ZERO NOT SET")!=0,"reject angle before zero");
   read_fault=0;fail_reg=0x43;send("r\n");run(20);check(!ins_data()->raw.valid&&!Motor_Output(),"calibration read failure keeps stopped");
   fail_reg=-1;send("r\n");run(30);check(ins_data()->raw.valid&&!Motor_Output(),"manual retry recovers without starting");
   send("z\n");run(20);send("angle:5\n");run(8000);check(fabsf(rotation-5)<1,"angle works after retry and zero");
 }
 else check(0,"unknown scenario");
 send("q");run(20);check(!Motor_Output(),"stop");printf("PASS %s\n",argv[1]);
}
