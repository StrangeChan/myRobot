#include "timer.h"
#include "usart.h"
#include "GPIO.h"
#include "control.h"
#include "stm32f4xx.h"

u8 time_out_flag=0;
u8 count = 0;
u8 counter=0;
void LCD_Show_position2(void);
float ll1[4]={0,0,0,0};
float ll2[4]={0,0,0,0};
float ll3[4]={0,0,0,0};

/*
void TIM7_Int_Init(u32 arr,u32 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);  ///ʹ��TIM7ʱ��
	
	TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStructure);//��ʼ��TIM7
	
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE); //����ʱ��7�����ж�

	
	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn; //��ʱ��7�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; //��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_Cmd(TIM7,ENABLE); //ʹ�ܶ�ʱ��7
}
*/
//����ʱ����ʼ����TIM2����ȡ��̼���Ϣ��
void TIM_Init(u32 arr,u32 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///ʹ��TIM2ʱ��
	
	TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//��ʼ��TIM3
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //����ʱ��2�����ж�
	TIM_Cmd(TIM2,ENABLE); //ʹ�ܶ�ʱ��2
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; //��ʱ��2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


/*
//
ѹ��Ͷ��  ����֤

void TIM7_IRQHandler(void)
{
	
			
	if(TIM_GetITStatus(TIM7,TIM_IT_Update)==SET)
	{
		
		if(counter<23)
		{
			counter++;
		}
		else
		{
			time_out_flag=1;
			
			if(chengxu==4||chengxu==5||chengxu==6)
			{
				if(lankuang_state==2)
					{
						control1_W(0);
						control2_W(0);
						control3_W(0);
						down_shot();						
					}
			}
			

			BEEP=1;
				
		}
		

	}
		 TIM_ClearITPendingBit(TIM7,TIM_IT_Update);  //����жϱ�־�	
	
}
*/




//��ʱ��2�жϷ���������ʱ��ȡ����������ȡ������λ��
void TIM2_IRQHandler(void)
{
	int16_t l1=0,l2=0,l3=0,temp=0;

	//printf("�ж�");
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) 
	{	
		if(count%3 == 0)
		{
			l1= hctl2020_getdata_0();
			l2= hctl2020_getdata_1();
			l3= hctl2020_getdata_2();
		}
		else if(count%3 == 1)
		{
			l2= hctl2020_getdata_1();
			l3= hctl2020_getdata_2();
			l1= hctl2020_getdata_0();
		}
		else if(count%3 == 2)
		{
			l3= hctl2020_getdata_2();
			l1= hctl2020_getdata_0();
			l2= hctl2020_getdata_1();
		}
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //����жϱ�־λ
	
	
	
	BasketballRobot.w[2] += l3/4;
	BasketballRobot.w[1] += l2/4;
	BasketballRobot.w[0] += l1/4;    
	

	
	BasketballRobot.v[0] = l1*RAD*ENCODER_R;
	BasketballRobot.v[1] = l2*RAD*ENCODER_R;			//1.0233173f
	BasketballRobot.v[2] = l3*RAD*ENCODER_R;
	
	
	if(receive2)
	{
		
		//ƫ���ǣ�z �ᣩ Yaw=((YawH<<8)|YawL)/32768*180(��)
		if(USART2_RX_STA&0x8000)
		{
			temp = USART2_RX_BUF[7];
			BasketballRobot.ThetaD = ((float)((temp<<8)|USART2_RX_BUF[6]))/32768*180;
			receive2 = 0;
			USART2_RX_STA=0;
			
			BasketballRobot.ThetaR = BasketballRobot.ThetaD * PI / 180 + BasketballRobot.theta_offset;
			
			while(BasketballRobot.ThetaR < 0)
				BasketballRobot.ThetaR  = BasketballRobot.ThetaR + PI + PI;
			
			while (BasketballRobot.ThetaR > 2 * PI)
				BasketballRobot.ThetaR = BasketballRobot.ThetaR - PI - PI;
			
			while(BasketballRobot.ThetaD < 0)
				BasketballRobot.ThetaD  = BasketballRobot.ThetaD + 360;
			
			while (BasketballRobot.ThetaD >360)
				BasketballRobot.ThetaD = BasketballRobot.ThetaD - 360;
		}
	}
	
		
	GetPosition2();
	//eGetPosition();
	
	if(count++ == 0)
	{
		LCD_Show_lcj();
	}
	else if(count == 2)
	{
		LCD_Show_v();
	}
	else if(count == 3)
	{
		LCD_Show_V();
	}
	else if(count == 4)
	{
		LCD_Show_position();
		LCD_Show_position2();
	}
		
}

//��ʾX,Y,thetaλ��
void LCD_Show_position2(void)
{
	float tem;
	u16 integer=0,decimal=0;
	
	tem = BasketballRobot.x;
	if(tem<0)
	{
		tem = -tem;
		LCD_ShowChar(100,200+340,'-',16,0);
	}
	else{
		LCD_ShowChar(100,200+340,' ',16,0);
	}
	integer = (u32)tem;
	decimal = (u32)((tem-integer)*1000);
	LCD_ShowxNum(130,200+340,integer,3,16,0);
	LCD_ShowChar(130+25,250+340,'.',16,0);
	LCD_ShowxNum(130+36,250+340,decimal,3,16,0);
	
	tem = BasketballRobot.y;
	if(tem<0)
	{
		tem = -tem;
		LCD_ShowChar(100,250+360,'-',16,0);
	}
	else{
		LCD_ShowChar(100,250+360,' ',16,0);
	}
	integer = (u32)tem;
	decimal = (u32)((tem-integer)*1000);
	LCD_ShowxNum(130,250+360,integer,3,16,0);
	LCD_ShowChar(130+25,250+360,'.',16,0);
	LCD_ShowxNum(130+36,250+360,decimal,3,16,0);
	
	tem = BasketballRobot.ThetaR;
	if(tem<0)
	{
		tem = -tem;
		LCD_ShowChar(100,250+380,'-',16,0);
	}
	else
	{
		LCD_ShowChar(100,250+380,' ',16,0);
	}
	integer = (u32)tem;
	decimal = (u32)((tem-integer)*1000);
	LCD_ShowxNum(130,250+380,integer,3,16,0);
	LCD_ShowChar(130+25,250+380,'.',16,0);
	LCD_ShowxNum(130+36,250+380,decimal,3,16,0);
				
}


