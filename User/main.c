#include "stm32f4xx.h"
#include "HCTL2020.h"
#include "pwm.h"
#include "remote.h"
#include "EXIT.h"
#include "control.h"


u8 zhongquan_case;
u8 changdi;
u8 chengxu;
u8 sanfen_case;

int main(void)
{
	u8 key = 0;					//����ֵ
	//u8 chengxu = 0;				//����ѡ��
	u8 flag=0;
	u8 qiu = 0;				//����
	int16_t time = 0;			//��ʱ
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//����ϵͳ�ж����ȼ�����2   2λ��ռ���� 2λ��Ӧ����
	zhongquan_case=10;
	changdi=1;					//���ҳ�
	chengxu=0;
	delay_init(168);  			//��ʼ����ʱ����
	UART_Init(9600);	 		//���ڳ�ʼ��Ϊ9600
	PWM_Init();				//��ʼ��PWM������
 	LED_Init();			    	 //LED�˿ڳ�ʼ��
	LCD_Init();					//LCD��ʼ��
	LCD_Show_Title();			//Һ������ʾ���ݳ�ʼ��
	//KEY_Init();					//������ʼ��
	hctl2020_init();			//��ʼ��������
	Remote_Init();				//����ң�س�ʼ��
	Charge_Init();				//�����翪�س�ʼ��
	ShotSwitch_Init();				//�����ʼ��
	LimitSwitch_init();				//��λ���س�ʼ��
	BEEP_Init();
	InfraredSwitch_Init();				//���⿪�س�ʼ��
	//EXTIX_Init();


	//TIM_SetCompare2(TIM5,280);	//��HCTL2020�ṩʱ���źţ�PA0
	TIM_SetCompare2(TIM5,7);
	Control_Init();				//�����˳�ʼ��
	TIM_Init(100-1,8400-1);//��ʱ��ȡ��������ʱ��0.01f
	EXTIX_Init();
	
	while(1)
	{
		//SetPWM(10,3,30);
		RobotGoTo(0,10,0);
		//GetMotorVelocity(0,100,0);
		//RobotRotate(-99);
		//LCD_Show_pwm();
	}
	
	
}

