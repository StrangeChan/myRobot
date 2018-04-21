#ifndef __GPIO_H
#define __GPIO_H	 
#include "sys.h" 


//Beep�˿ڶ���
#define BEEP PFout(8)	// ����������IO 
//��������ʼ��	
void BEEP_Init(void);


//LED�˿ڶ���
#define LED0 PFout(9)	
#define LED1 PFout(10)		 
//LED��ʼ��
void LED_Init(void);	


//������ʼ������
void KEY_Init(void);

//�����翪�ض˿ڶ���
#define CHARGE PGout(6)	
//�����翪�س�ʼ��
void Charge_Init(void);

//���俪�ض˿ڶ���
#define SHOT PGout(7)	
//�����ʼ��,GPIOG7
void ShotSwitch_Init(void);

//��λ���ض˿ڶ���	
#define LimitSwitchDowm 	PCin(6)	
#define LimitSwitchUp   	PCin(7)
//��λ���س�ʼ��
void LimitSwitch_init(void);


//���⿪�ض˿ڶ���
#define INFRARED PFin(9)
//���⿪�س�ʼ��
void InfraredSwitch_Init(void);
	
#endif

















