/*
 * PLL_SOGI.c
 *
 *  Created on: 2023年5月30日
 *      Author: Portion^zjg
 */

#include "PLL.h"
#include "arm_math.h"


#define pi  3.14159265358979f
#define Ts   0.00005f
static float k = 1.414;
static float w = 100 * pi; // sogi的开环频率-50hz
// static float phase_angle=0;//PLL输出的相角
float error_z = 0;	// 误差
float error_z1 = 0; // 上一时刻的误差
static float PLL_kp = 0.571;
static float PLL_ki = 50.766;

uint16_t buff[3];  //存放adc电压
float f32sin;
float f32cos;
float Uac;
float Uqac;
float b; //Uac
float Iac,Iqac;
float Iref=0.2f;
volatile	int i=0;
/*锁相专用变量*/
float Udc=0;
float f32vout=0.0f;
float 	f32vout1; 
	float f32qvout;
		float f32k = 1.414f;
float f32w_sogi = 1.0f;

float f32iout;
float	f32iout1;
float f32qiout;
float f32Iac_sogi = 0;

/*锁相变量结束*/
float f32Uout = 0; //

 PARK Upark;
 PARK Ipark;
 IPARK Uipark;
 _pid Iac_q,Iac_d,Udc_d;


///*幅度相位裕度法*/
///*-----------外部变量-------------*/
//float RefIp=3.5;//参考电流峰值
//float Phi=0;//电流相位角，单位为度
//float Vdc=25.0;//直流侧电压,逆变的输入电压
//volatile float LastVL=0;//上一时刻的电感电压
///*-----------end-------------*/


//float LastIacError=0;//上一时刻的误差
//float wcr=2*pi*500;//穿越频率
//float L=0.001;//电感感值，需修改。
//float Lr=0.01;//电路中的电阻,需修改
//float kp;//p值
//float ki;
////float kza=kp+ki*ts;//差分方程kza
//float kza;//差分方程kza
////float kzb=-kp;//差分方程kzb
//float kzb;//差分方程kzb
//float RefIac;
//	float Vac=0;
//void param_set()
//{
//	kp=wcr*L;
//	ki=wcr*Lr;
//	kza=kp+ki*Ts;
//	kzb=-kp;
//}
///*变量结束*/
/*幅度相位阈值*/

/**/
// sogi实现的单相锁相环
float PLL_SOGI(float Eac,float ts)
{
	static float va = 0; //
	static float last_angle = 0;
	static float vq_temp = 0; // 算出的w增量
	static float vb = 0;	  // 要生成的正交分量,滞后90度
	float vq;				  // q轴分量
	float phase_angle = 0;
	va += ts * w * ((Eac - va) * k - vb);
	vb += w * ts * va;
	// park变换，将va,vb转化到vd,vq。
	vq = va * arm_sin_f32(last_angle) * (-1) + vb * arm_cos_f32(last_angle);


	
	error_z = vq - 0;											 // 误差
	vq_temp += PLL_kp * (error_z - error_z1) + PLL_ki * error_z; // pi控制,计算出w增量
	error_z1 = error_z;											 // 保存上一次误差值
	if (vq_temp > 20.f * pi)									 // 限幅
	{
		vq_temp = 20.f * pi;
	}
	if (vq_temp < -20.f * pi)
	{
		vq_temp = -20.f * pi;
	}
	phase_angle = last_angle + ts * (100 * pi + vq_temp); // 输出信号当前角度
	if (phase_angle > 2 * pi)							  // 对2pi取余
	{
		phase_angle -= 2 * pi;
	}
	if (phase_angle < -2 * pi)
	{
		phase_angle += 2 * pi;
	}
	last_angle = phase_angle;
	return phase_angle;
}


/**/

//void single_grid(float Eac,float Iac)//传入参数为采集到的电压和电流
//{
//float phase_angle=0;
//float d;
//float IacError;

//float VL;//电感电压
////逆变输出电压
//phase_angle=PLL_SOGI(Eac,Ts);//得到当前信号当前时刻的相位角

//RefIac=RefIp*arm_sin_f32(phase_angle+Phi/180.0*pi);//生成此刻的参考电流，引入电流相位角
//IacError=RefIac-Iac;//计算电流误差
//VL=LastVL+kza*IacError+kzb*LastIacError;//pi控制计算出电感电压
//LastIacError=IacError;//将此刻误差保存为上一次误差
//Vac=VL+Eac;//加入前馈，计算得到逆变的输出电压
//LastVL=VL;
//if(Vac>Vdc)
//{
//	Vac=Vdc;
//}
//if(Vac<-Vdc)
//{
//	Vac=-Vdc;
//}
//	if(Vac>0)
//	{
//		d=Vac;
//		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
//		TIM1->CCR1=d*4200+0.5;
//		TIM1->CCR2=d*4200-0.5;
//	}
//	else
//	{
//		d=1+Vac;
//		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);
//		TIM1->CCR1=d*4200+0.5;
//	}
//TIM1->CNT=0;
//			TIM1->CCR1=Vac/Vdc*4200.0f+4200.0f;
//			TIM1->CCR2=4200.0f-Vac/Vdc*4200.f;
//			TIM1->CNT=0;
////	return Vac;
//		
//}
/**/
//SOGI输出正交分量
float SOGI(float Eac,float ts)
{
static float f32vout1=0.0f;
static float f32vout=0.0f;
static float f32k=1.414f;
static float f32w_sogi=1.0f;
static float f32qvout=0.0f;
      f32vout1 = ((Eac - f32vout) * f32k - f32qvout) * f32w_sogi * w;
      f32vout+= f32vout1 * ts;
      //Vq
      f32qvout += f32vout * f32w_sogi * w * ts;
	return f32qvout;
}

/**
 * @description: 增量式PID的实现
 * @param {_pid*} pid  传入结构体 
 * @param {float} SetValue  设定值
 * @param {float} ActualValue  实际值
 * @param {int} min_out  最小增量限幅
 * @param {int} max_out  最大增量限幅
 * @return {*}  返回增量值
 */
float PID_realize(struct _pid_* pid,float SetValue,float ActualValue,float min_out ,float  max_out)
{
    float OutValue=0;
    pid->SetValue=SetValue;
    pid->ActualValue=ActualValue;
    pid->err=pid->SetValue-ActualValue;
    //OutValue=pid->Kp*(pid->err-pid->err_next)+pid->Ki*pid->err+pid->Kd*(pid->err-2*pid->err_next+pid->err_last);
    OutValue=pid->Kp*pid->err+pid->out_last;
    if(OutValue<min_out)OutValue=min_out;
    if(OutValue>max_out)OutValue=max_out;
    pid->v1=pid->out_last-OutValue;
    pid->err_last=pid->err_next;
    pid->err_next=pid->err;
    pid->out_last+=pid->Ki*pid->err-pid->v1;
    //pid->out_last=OutValue;
    return OutValue;
}
void PLL_pid_init(void)
{

    Iac_q.SetValue=0;            //定义设定值
    Iac_q.ActualValue=0;        //定义实际值
     Iac_q.err=Iac_q.err_last=0.0;                //定义偏差值
          
   Iac_q.Kp=0.5f;
	Iac_q.Ki=0.005f;
	Iac_q.Kd=0.0f;            //定义比例、积分、微分系数
	
	
   Iac_d.Kp=0.5f;
	Iac_d.Ki=0.005f;
	Iac_d.Kd=0.0f;  
 Iac_d.err=Iac_q.err_last=0.0;
	Uipark.f32Ds=0.0f;
	Uipark.f32Qs=0.0f;
	
	
	  Udc_d.Kp=0.1f;
	Udc_d.Ki=0.0001f;
	Udc_d.Kd=0.0f; 
	//param_set();
}
//park 变换
void PARK_MACRO(PARK* v)
{
	v->f32Ds = v->f32Alpha*v->f32Cosine + v->f32Beta*v->f32Sine;
	v->f32Qs = v->f32Beta*v->f32Cosine - v->f32Alpha*v->f32Sine;
}
//反park变换
void IPARK_MACRO(IPARK* v)
{
	v->f32Alpha = v->f32Ds*v->f32Cosine - v->f32Qs*v->f32Sine;
	v->f32Beta = v->f32Qs*v->f32Cosine + v->f32Ds*v->f32Sine;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	

    if(htim==(&htim3))
    { 	
			//HAL_TIM_Base_Start_IT(&htim3);
				i++;
			//对Us采样，锁相
			Uac=(buff[0]/4096.0f*3.3f-1.65f)*62.0f;
			Iac=((buff[1]/4096.0f*3.3f)-1.65f)*0.831f;
			Udc=(buff[2]/4096.0f*3.3f*26.0f);
		// single_grid(Uac,Iac);
			b=PLL_SOGI(Uac,Ts); 
			f32sin = arm_sin_f32(b);
			f32cos = arm_cos_f32(b);
//		//	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,Uac*1500.0f+2000.0f);
//			//提取正交向量
		//	Uqac=f32sin;
      f32vout1 = ((Uac - f32vout) * f32k - f32qvout) * f32w_sogi * w;
      f32vout+= f32vout1 * Ts;
      //Vq
      f32qvout += f32vout * f32w_sogi * w * Ts;
		Uac=f32vout;
			//
		Uqac=f32qvout;

		//	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,f32cos*1500.0f+2000.0f);
			Upark.f32Alpha = Uac;
      Upark.f32Beta = Uqac;
      Upark.f32Sine = f32sin;
      Upark.f32Cosine = f32cos;
			PARK_MACRO(&Upark);
			
		
		  f32iout1 = ((Iac - f32iout) * f32k - f32qiout) * f32w_sogi * w;
      f32iout += f32iout1 * Ts;
      //Iq
      f32qiout += f32iout * f32w_sogi * w * Ts;
      Iqac = f32qiout;
			//HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,Iac*1500.0f+2000.0f);
			Ipark.f32Alpha =Iac;
      Ipark.f32Beta = Iqac;
      Ipark.f32Sine = f32sin;
      Ipark.f32Cosine = f32cos;
			//HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,f32cos*1500.0f+2000.0f);
      PARK_MACRO(&Ipark);
			Iac_q.ActualValue=Ipark.f32Qs;
			Iac_d.ActualValue=Ipark.f32Ds;

			Uipark.f32Ds=PID_realize(&Iac_d,Iref,Ipark.f32Ds,-0.8f,0.8f);
			Uipark.f32Qs=PID_realize(&Iac_q,0.0f,Ipark.f32Qs,-0.8f,0.8f);
		
//			if(Uipark.f32Ds>0.707f)
//			{
//			Uipark.f32Ds=0.707f;
//			}
//			if(Uipark.f32Ds<-0.707f)
//			{
//			Uipark.f32Ds=-0.707f;
//			}
//			if(Uipark.f32Qs>0.707f)
//			{
//			Uipark.f32Qs=0.707f;
//			}
//			if(Uipark.f32Qs<-0.707f)
//			{
//			Uipark.f32Qs=-0.707f;
//			}
			Uipark.f32Sine = f32sin;
			Uipark.f32Cosine = f32cos;
			IPARK_MACRO(&Uipark);
			f32Uout = Uipark.f32Alpha;
		TIM1->CCR1 = PERIOD * 0.5f-(f32Uout) * PERIOD * 0.5f;
			
		TIM1->CCR2 =PERIOD * 0.5f +(f32Uout) * PERIOD * 0.5f;
//		if(Uac>0)
//		{
//			TIM1->CNT=0;
//			TIM1->CCR1 = PERIOD * 0.5f-(f32Uout) * PERIOD * 0.5f;
//			TIM1->CCR2 =0;
//		//TIM1->CCR2 =PERIOD * 0.5f + (f32Uout) * PERIOD * 0.5f;
//		}
//				if(Uac<0)
//		{
//			TIM1->CNT=0;
//			TIM1->CCR1 = PERIOD * 0.5f - (f32Uout) * PERIOD * 0.5f;
//			TIM1->CCR2 =8500;
//		//TIM1->CCR2 = PERIOD * 0.5f + (f32Uout) * PERIOD * 0.5f;
//		}
//		if(i<20000)
//		{
//		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//		}
//				if(i>20000)
//		{
//		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
//		}
		if(i==400)
		{
		Iref=0.5f*PID_realize(&Udc_d,40.0f,Udc,-1.5f,1.5f);
		i=0;
		}
		//	TIM1->CCR1=4200;
	}
	

}
