#ifndef _PLL_H
#define _PLL_H

#include "main.h"
#include "tim.h"
#include "adc.h"

#include "dac.h"


#define PERIOD (8500)
#define EXTERN extern
typedef struct PARK_  //PARK变换结构体
{
	float  f32Alpha;
	float  f32Beta;
	float  f32Angle;
	float  f32Ds;
	float  f32Qs;
	float  f32Sine;
	float  f32Cosine;
} PARK;
	
typedef struct IPARK_  //反park变换结构体
{
	float  f32Alpha;
	float  f32Beta;
	float  f32Angle;
	float  f32Ds;
	float  f32Qs;
	float  f32Sine;
	float  f32Cosine;
} IPARK;

typedef  struct _pid_ {
    float SetValue;            //定义设定值
    float ActualValue;        //定义实际值
    float err;                //定义偏差值
    float err_next;
    float err_last;            //定义上一个偏差值
    float Kp,Ki,Kd;            //定义比例、积分、微分系数
//    float voltage;          //定义电压值（控制执行器的变量值)
//    float integral;            //定义积分值
	float v1;
    float out_last;
} _pid;

extern PARK Upark;
extern PARK Ipark;
extern IPARK Uipark;
extern _pid Iac_q,Iac_d;


float PLL_SOGI(float Eac,float ts);
float SOGI(float Eac,float ts);
float PID_realize(struct _pid_* pid,float SetValue,float ActualValue,float min_out ,float  max_out);
void PLL_pid_init(void);
void PARK_MACRO(PARK* v);
void IPARK_MACRO(IPARK* v);
#endif