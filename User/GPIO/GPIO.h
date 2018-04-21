#ifndef __GPIO_H
#define __GPIO_H	 
#include "sys.h" 


//Beep端口定义
#define BEEP PFout(8)	// 蜂鸣器控制IO 
//蜂鸣器初始化	
void BEEP_Init(void);


//LED端口定义
#define LED0 PFout(9)	
#define LED1 PFout(10)		 
//LED初始化
void LED_Init(void);	


//按键初始化函数
void KEY_Init(void);

//弹射充电开关端口定义
#define CHARGE PGout(6)	
//弹射充电开关初始化
void Charge_Init(void);

//弹射开关端口定义
#define SHOT PGout(7)	
//弹射初始化,GPIOG7
void ShotSwitch_Init(void);

//限位开关端口定义	
#define LimitSwitchDowm 	PCin(6)	
#define LimitSwitchUp   	PCin(7)
//限位开关初始化
void LimitSwitch_init(void);


//红外开关端口定义
#define INFRARED PFin(9)
//红外开关初始化
void InfraredSwitch_Init(void);
	
#endif

















