#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"
#include "hctl2020.h"
#include "lcd.h"
//#include "control.h"
extern u8 time_out_flag;



void TIM_Init(u32 arr,u32 psc);	//主定时器初始化，TIM2，获取里程计信息用


void TIM7_Int_Init(u32 arr,u32 psc);//TIM7定时器初始化
void TIM7_IRQHandler(void); //TIM7中断函
	

#endif
