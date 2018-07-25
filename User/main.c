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
	u8 key = 0;					//按键值
	//u8 chengxu = 0;				//程序选择
	u8 flag=0;
	u8 qiu = 0;				//找球
	int16_t time = 0;			//延时
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置系统中断优先级分组2   2位抢占优先 2位响应优先
	zhongquan_case=10;
	changdi=1;					//左场右场
	chengxu=0;

	delay_init(168);  			//初始化延时函数
	UART_Init(115200);	 		//串口初始化为9600

	ALL_GPIO_Init();
	PWM_Init();				//初始化PWM发生器
	LCD_Init();					//LCD初始化
	LCD_Show_Title();			//液晶屏显示内容初始化

	Control_Init();				//机器人初始化
	
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

