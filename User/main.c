#include "stm32f4xx.h"
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
	UART_Init(115200);	 		//���ڳ�ʼ��Ϊ9600

	ALL_GPIO_Init();
	PWM_Init();				//��ʼ��PWM������
	LCD_Init();					//LCD��ʼ��
	LCD_Show_Title();			//Һ������ʾ���ݳ�ʼ��

	Control_Init();				//�����˳�ʼ��
	
	TIM5_Init(10000,83);
	EXTIX_Init();
	
	while(1)
	{
		//SetPWM(10,3,30);
		//RobotGoTo(0,10,0);
		//GetMotorVelocity(0,100,0);
		RobotRotate(-99);
		//LCD_Show_pwm();
	}
	
	
}

