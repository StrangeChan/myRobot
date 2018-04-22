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
	UART_Init(9600);	 		//串口初始化为9600
	PWM_Init();				//初始化PWM发生器
 	LED_Init();			    	 //LED端口初始化
	LCD_Init();					//LCD初始化
	LCD_Show_Title();			//液晶屏显示内容初始化
	//KEY_Init();					//按键初始化
	hctl2020_init();			//初始化解码器
	Remote_Init();				//红外遥控初始化
	Charge_Init();				//弹射充电开关初始化
	ShotSwitch_Init();				//弹射初始化
	LimitSwitch_init();				//限位开关初始化
	BEEP_Init();
	InfraredSwitch_Init();				//红外开关初始化
	//EXTIX_Init();


	//TIM_SetCompare2(TIM5,280);	//给HCTL2020提供时钟信号，PA0
	TIM_SetCompare2(TIM5,7);
	Control_Init();				//机器人初始化
	TIM_Init(100-1,8400-1);//定时读取解码器，时间0.01f
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

