#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "lcd.h"
#include "GPIO.h"
#include "MPU6050.h"
#include "remote.h"
#include "timer.h"
#include <math.h>

#define PI 		3.141592654f

#define ENCODER_FD 	4.0f			//编码器分频
#define ENCODER_MAX 500.0f		//编码器编码数
#define ENCODER_T 	0.01f   		//编码器计算当前速度时间
#define ENCODER_L 	0.2006f		//编码器轮到机器人中心的距离
#define ENCODER_R 	0.0275f		//编码器轮子半径
#define RAD	 0.1570796327f		//编码器一个脉冲对应的角度 pi/500/4/0.01

#define MOTOR_L 0.2013f		//轮到机器人中心的距离
#define MOTOR_R 0.0508f		//轮子的半径

#define MOTOR_STATIC_1 4000		//TIM9 CH1 PE5
#define MOTOR_STATIC_2 4000  		//TIM9 CH2 PE6

#define RADAR_MID 268	//雷达定位中心
#define VISION_MID 320	//视觉定位中心
#define DIS_RADAR 2500	//篮筐雷达定位距离
#define DIS_VISION 280	//篮筐视觉定位距离


struct ROBOT
{
	float X;		//机器人在坐标系中x坐标
	float Y;		//机器人在坐标系中y坐标
	float x;		//机器人在坐标系中x坐标
	float y;		//机器人在坐标系中y坐标
	float ThetaR;	//机器人正方向和y轴夹角 弧度
	float ThetaD;	//机器人正方向和y轴夹角 角度
	float Vx;		//机器人在坐标系x方向速度
	float Vy;		//机器人在坐标系y方向速度
	
	float W;		//机器人角速度，顺时针正方向
	float w[3];		//编码器的实际计数/4
	float v[3];		//编码器所得速度
	float Velocity[3];	//轮子的速度
	float LastTheta;	//上一时刻，机器人theta角
	float theta_offset;	//角度偏差矫正
};

//接收雷达数据，极坐标
struct RADAR
{
	u32 Distance;  //距离
	
	u32 Angle;	//角度
};	

//接收视觉数据
struct VISION
{
	u32 Depth;	//深度，纵轴
	
	u32 X;		//X位置，横轴
};


extern struct ROBOT BasketballRobot;

extern struct RADAR Radar;
extern struct VISION Vision;


void Control_Init(void);		//机器人初始化


static void Velocity2PWM(float *V);		//电机速度转换成PWM数值，原理看电机驱动板手册
void SetPWM(float V1,float V2,float V3); 	//设置三个轮子PWM

void GetMotorVelocity(float vx,float vy,float w);		//给定球场坐标速度求得轮子的速度
void GetMotorVelocity_Self(float vx,float vy,float w);	//给自身坐标系速度求得轮子的速度


void GetInfraredState(void);	//获取红外开关状态
void Robot_armDown(void);	//机械臂下降
void Robot_armUp(void);		//机械臂上升

u8 GetVisionData(void);		//视觉数据处理
u8 GetRadarData(void);		//激光处理数据

void GetPosition(void);		//坐标转换
void GetPosition2(void);		//坐标转换,两个里程计定位


static float AdjustAngleV(float D_Theta);		//根据偏差大小调整角速度
static float AdjustVy(float D_Y);			//根据偏差大小调整Y轴速度
static float AdjustVx(float D_X);			//根据偏差大小调整X轴速度


void RobotRotate(float theta);	//自旋运动，根据误差角度，自动调节

void RobotGoTo(float X_I,float Y_I,float Theta_I);	//行至指定坐标
#endif
