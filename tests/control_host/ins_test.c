#include "ins.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
static void check(int ok,const char *s) { if(!ok){puts(s);exit(1);} }
int main(void)
{
    INS ins;float g[3]={0,0,1},w[3]={0,0,0};
    INS_Init(&ins,g);
    for(int i=0;i<1000;++i)INS_Update(&ins,g,w,0.005f);
    check(fabsf(ins.roll)<0.01f && fabsf(ins.pitch)<0.01f,"rest attitude");
    w[1]=30;
    for(int i=1;i<=200;++i){float r=i*0.005f*30*0.01745329252f;g[0]=-sinf(r);g[2]=cosf(r);INS_Update(&ins,g,w,0.005f);}
    check(fabsf(ins.pitch-30)<0.2f,"pitch gyro/accel fusion");
    w[1]=0;
    for(int i=0;i<1000;++i)INS_Update(&ins,g,w,0.005f);
    check(fabsf(ins.pitch-30)<0.1f,"static pitch convergence");
    check(fabsf(ins.angular_accel[1])<0.001f,"angular acceleration settles");
    float sum=0;for(int i=0;i<4;++i)sum+=ins.q[i]*ins.q[i];
    check(fabsf(sum-1)<0.0001f,"unit quaternion");
    puts("PASS Mahony attitude and angular acceleration");
}
