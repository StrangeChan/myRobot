#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"
#include "hctl2020.h"
#include "lcd.h"
//#include "control.h"
extern u8 time_out_flag;



void TIM_Init(u32 arr,u32 psc);	//����ʱ����ʼ����TIM2����ȡ��̼���Ϣ��


void TIM7_Int_Init(u32 arr,u32 psc);//TIM7��ʱ����ʼ��
void TIM7_IRQHandler(void); //TIM7�жϺ�
	

#endif
