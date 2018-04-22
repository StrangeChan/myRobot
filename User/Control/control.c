#include "control.h"

struct ROBOT BasketballRobot;

struct RADAR Radar;
struct VISION Vision;

void Control_Init(void)
{
	BasketballRobot.X = 0;		//������������ϵ��x����
	BasketballRobot.Y = 0;		//������������ϵ��y����
	BasketballRobot.ThetaR = 0;	//�������������y��н�
	BasketballRobot.ThetaR = 0;	//�������������y��н�
	BasketballRobot.Vx = 0;		//������������ϵx�����ٶ�
	BasketballRobot.Vy = 0;		//������������ϵy�����ٶ�
	BasketballRobot.W = 0;		//�����˽��ٶȣ�˳ʱ��������
	
	BasketballRobot.w[1] = 0;		//��һ��������ʵ�ʼ���
	BasketballRobot.w[2] = 0;		//�ڶ���������ʵ�ʼ���
	BasketballRobot.w[0] = 0;		//������������ʵ�ʼ���
	
	BasketballRobot.v[1] = 0;		//��һ�������������ٶ�
	BasketballRobot.v[2] = 0;		//�ڶ��������������ٶ�
	BasketballRobot.v[0] = 0;		//�����������������ٶ�
	
	BasketballRobot.LastTheta = 0;	//��һʱ�̣�������theta��
	BasketballRobot.theta_offset = 0;	//�Ƕ�ƫ�����
	
	//�״�Ӿ��������
	Radar.Angle = 0;
	Radar.Distance = 0;
	
	Vision.Depth = 0;
	Vision.X = 0;
	
	SetPWM(0,0,0);
	
	MPU_Init();			//MPU6050��ʼ��
	MPU_Init();
	MPU_Init();	
	
}



//����ٶ�ת����PWM��ֵ��ԭ������������ֲ�
//���㹫ʽ�� V = Vmax *��ռ�ձ�*100 �C 50�� /50
static void Velocity2PWM(float *V)
{
	*V=1000 - *V;//*V+=1000;
	if(*V>=1900)
		*V=1900;
	if(*V<=100)
		*V=100;
}

//������������PWM
//V1:	���1�ٶ�
//V2:	���2�ٶ�
//V3;	���3�ٶ�
void SetPWM(float V1,float V2,float V3)
{
	BasketballRobot.Velocity[0] = V1;
	BasketballRobot.Velocity[1] = V2;
	BasketballRobot.Velocity[2] = V3;
	
	//ת��
	Velocity2PWM(&V1);
	Velocity2PWM(&V2);
	Velocity2PWM(&V3);
	
	//��CCR1�Ĵ���дֵ   ����PWM��
 	TIM_SetCompare1(TIM3,(uint32_t)V1);
	
 	TIM_SetCompare2(TIM3,(uint32_t)V2);
	
 	TIM_SetCompare3(TIM3,(uint32_t)V3);	
}

//�����������ٶ�������ӵ��ٶ�
//vx���������x���ٶ�
//vy���������y���ٶ�
//w:������ԭ����ת�Ľ��ٶ�
//ע������ٶȣ�
void GetMotorVelocity(float vx,float vy,float w)
{
	u8 i,j,k;
	float L[3][3];
	float theta[3][3];
	float V[3];
	float tem[3][3];
					
	//v(PWM)=L*Theta*V
	//   cos(60)	sin(60)	-MOTOR_L
	//L= cos(180) 	sin(180)	-MOTOR_L
	//   cos(-60)	sin(-60)	-MOTOR_L
	L[0][0] =  0.5;					L[0][1] =  0.8660254037844386;		L[0][2] = -MOTOR_L;
	L[1][0] = -1;						L[1][1] =  0;						L[1][2] = -MOTOR_L;
	L[2][0] =  0.5;					L[2][1] = -0.8660254037844386;		L[2][2] = -MOTOR_L;
	//		cos(theta)	sin(theta)	0
	//theta= -sin(theta)	cos(theta) 	0
	//		 	0			0		1
	theta[0][0]= cos(BasketballRobot.ThetaR);	theta[0][1] = sin(BasketballRobot.ThetaR);	theta[0][2] = 0;
	theta[1][0]= -sin(BasketballRobot.ThetaR);	theta[1][1] = cos(BasketballRobot.ThetaR);	theta[1][2] = 0;
	theta[2][0]= 0;						theta[2][1] = 0;						theta[2][2] = 1;
	//V
	V[0] = -vx*10;
	V[1] = -vy;
	V[2] = -w;
	
	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			tem[i][j] = 0;
			for(k=0;k<3;k++)
				tem[i][j] += L[i][k] * theta[k][j];			
		}
	}
	
	for(i=0;i<3;i++)
	{
		BasketballRobot.Velocity[i] = 0;
		for(j=0;j<3;j++)
			BasketballRobot.Velocity[i] += tem[i][j]*V[j];
	}
	
	LCD_Show_pwm();
}

//����������ϵ�ٶ�������ӵ��ٶ�
//vx������������x���ٶ�
//vy�����������y���ٶ�
//w:  ������ԭ����ת�Ľ��ٶ�
//ע������ٶȣ�
void GetMotorVelocity_Self(float vx,float vy,float w)
{
	u8 i,j,k;
	float L[3][3];
	float theta[3][3];
	float V[3];
	float tem[3][3];
					
	//v(PWM)=L*Theta*V
	//   cos(60)	sin(60)	-MOTOR_L
	//L= cos(180) 	sin(180)	-MOTOR_L
	//   cos(-60)	sin(-60)	-MOTOR_L
	L[0][0] =  0.5;					L[0][1] =  0.8660254037844386;		L[0][2] = -MOTOR_L;
	L[1][0] = -1;						L[1][1] =  0;						L[1][2] = -MOTOR_L;
	L[2][0] =  0.5;					L[2][1] = -0.8660254037844386;		L[2][2] = -MOTOR_L;
	//		cos(0)	sin(0)	0
	//theta= -sin(0)	cos(0) 	0
	//		 0			0		1
	theta[0][0]= 1;						theta[0][1] = 0;					theta[0][2] = 0;
	theta[1][0]= 0;						theta[1][1] = 1;					theta[1][2] = 0;
	theta[2][0]= 0;						theta[2][1] = 0;					theta[2][2] = 1;
	//V
	V[0] = -vx*10;
	V[1] = -vy;
	V[2] = -w;
	
	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			tem[i][j] = 0;
			for(k=0;k<3;k++)
				tem[i][j] += L[i][k] * theta[k][j];			
		}
	}
	
	for(i=0;i<3;i++)
	{
		BasketballRobot.Velocity[i] = 0;
		for(j=0;j<3;j++)
			BasketballRobot.Velocity[i] += tem[i][j]*V[j];
	}
	
	//LCD_Show_pwm();
}


//��ȡ���⿪��״̬
void GetInfraredState(void)
{	// return GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_9);
	while(1)
	{
		if(!INFRARED)
			break;
	}
}


//��е���½�
void Robot_armDown(void)
{
	//ԭ�����ӣ�3000
	//V1.0��1750
	u16 i,t;
	u16 W=2700;
	u16 nms=2000;

	
	if(LimitSwitchDowm==1)
	{
		TIM_SetCompare2(TIM9,MOTOR_STATIC_2);
		TIM_SetCompare1(TIM9,MOTOR_STATIC_1);
		return;
	}
	//EXTIX_Enable(1);
	#ifdef ZQD_DEBUG
	BEEP = 1;
	#endif
	TIM_SetCompare2(TIM9,W);  			//PE6
	TIM_SetCompare1(TIM9,MOTOR_STATIC_1);			//PE5
	LED1 = 1;
	for(i=0;i<nms;i++)
	{	  
		if(LimitSwitchDowm == 1)
		{	
			for(t=0;t<0xff;t++);
			if(LimitSwitchDowm==1)
			{
				TIM_SetCompare2(TIM9,MOTOR_STATIC_2);
				TIM_SetCompare1(TIM9,3970);
				break;
			}
		}
		for(t=0;t<0x4fff;t++)
			if(LimitSwitchDowm == 1)
				break;
	}
	TIM_SetCompare2(TIM9,MOTOR_STATIC_2);
	TIM_SetCompare1(TIM9,MOTOR_STATIC_1);

	#ifdef ZQD_DEBUG
	BEEP = 0;
	#endif
}

//��е������
void Robot_armUp(void)
{
	//ԭ�����ӣ�1960
	//V1.0:550
	u16 i,t;
	u16 W=2700;
	u16 nms=2000;

	
	if(LimitSwitchUp==1)
	{
		TIM_SetCompare2(TIM9,MOTOR_STATIC_2);
		TIM_SetCompare1(TIM9,MOTOR_STATIC_1);
		return ;
	}
	//EXTIX_Enable(0);
	#ifdef ZQD_DEBUG
	BEEP = 1;
	#endif
	TIM_SetCompare1(TIM9,W);		//PE6
	TIM_SetCompare2(TIM9,MOTOR_STATIC_2);		//PE5
	for(i=0;i<nms;i++)
	{
		if(LimitSwitchUp == 1)
		{
			for(t=0;t<0xff;t++);
			if(LimitSwitchUp == 1)
			{
				TIM_SetCompare1(TIM9,MOTOR_STATIC_1);
				TIM_SetCompare2(TIM9,MOTOR_STATIC_2);
				break;
			}
		}
		for(t=0;t<0x4fff;t++)
			if(LimitSwitchUp == 1)
				break;
	}
	TIM_SetCompare1(TIM9,MOTOR_STATIC_1);
	TIM_SetCompare2(TIM9,MOTOR_STATIC_2);	

	#ifdef ZQD_DEBUG
	BEEP = 0;
	#endif
}


//�Ӿ����ݴ���
u8 GetVisionData(void)
{	
	if(USART_RX_STA&0x8000)
	{					   
		//����λ����Ϣ
		if(USART_RX_BUF[0]!=' ')
			Vision.X = (USART_RX_BUF[0]-'0')*100;
		else 
			Vision.X = 0;
		USART_RX_BUF[0] = ' ';
		
		if(USART_RX_BUF[1]!=' ')
			Vision.X += (USART_RX_BUF[1]-'0')*10;
		USART_RX_BUF[1] = ' ';
		
		if(USART_RX_BUF[2]!=' ')
			Vision.X += (USART_RX_BUF[2]-'0');
		USART_RX_BUF[2] = ' ';
		
		//�����Ϣ
		if(USART_RX_BUF[3]!=' ')
			Vision.Depth=(USART_RX_BUF[3]-'0')*1000;
		else 
			Vision.Depth=0;
		USART_RX_BUF[3] = ' ';
		
		if(USART_RX_BUF[4]!=' ')
			Vision.Depth+=(USART_RX_BUF[4]-'0')*100;
		USART_RX_BUF[4] = ' ';
		
		if(USART_RX_BUF[5]!=' ')
			Vision.Depth+=(USART_RX_BUF[5]-'0')*10;
		USART_RX_BUF[5] = ' ';
		
		if(USART_RX_BUF[6]!=' ')
			Vision.Depth+=(USART_RX_BUF[6]-'0');
		USART_RX_BUF[6] = ' ';

		LCD_ShowString(30+200,420,200,16,16,"View :pix");	
		LCD_ShowNum(30+200+48+8+45,420,LimitSwitchUp,4,16);		
		LCD_ShowString(30+200,440,200,16,16,"View :length");	
		LCD_ShowNum(30+200+48+8+45,440,Vision.Depth,4,16);	
		
		USART_RX_STA=0;
		
		receive=0;
	}
	
	if(Vision.X<10 ||Vision.X >630)
		return 0;
	if(Vision.Depth<500)
		return 0;
	
	return 1;
}

//���⴦������
u8 GetRadarData(void)
{
	
	if(USART3_RX_STA&0x8000)
	{					   
		//�õ��˴ν��յ������ݳ���
		//len=USART_RX_STA&0x3fff;

		
		//������Ϣ
		if(USART3_RX_BUF[0]!=' ')
			Radar.Distance=(USART3_RX_BUF[0]-'0')*1000;
		else 
			Vision.Depth=0;
		USART3_RX_BUF[0] = ' ';
		
		if(USART3_RX_BUF[1]!=' ')
			Vision.Depth+=(USART3_RX_BUF[1]-'0')*100;
		USART3_RX_BUF[1] = ' ';
		
		if(USART3_RX_BUF[2]!=' ')
			Vision.Depth+=(USART3_RX_BUF[2]-'0')*10;
		USART3_RX_BUF[2] = ' ';
		
		if(USART3_RX_BUF[3]!=' ')
			Vision.Depth +=(USART3_RX_BUF[3]-'0');
		USART3_RX_BUF[3] = ' ';
		
		//�Ƕ���Ϣ
		if(USART3_RX_BUF[4]!=' ')
			Radar.Angle=(USART3_RX_BUF[4]-'0')*100;
		else 
			Radar.Angle=0;
		USART3_RX_BUF[4] = ' ';
		if(USART3_RX_BUF[5]!=' ')
			Radar.Angle+=(USART3_RX_BUF[5]-'0')*10;
		USART3_RX_BUF[5] = ' ';
		if(USART3_RX_BUF[6]!=' ')
			Radar.Angle+=(USART3_RX_BUF[6]-'0');
		USART3_RX_BUF[6] = ' ';

		LCD_ShowString(30+200,460,200,16,16,"Radar:rad");	
		LCD_ShowNum(30+200+48+8+45,460,Radar.Angle,4,16);		
		LCD_ShowString(30+200,480,200,16,16,"Radar:length");	
		LCD_ShowNum(30+200+48+8+45,480,Vision.Depth,4,16);	
		
		USART3_RX_STA=0;
		receive3=0;
	}
	
	if(Radar.Angle<240 || Radar.Angle >300) //ԭ��&&
		return 0;
	if(Vision.Depth>4000)
		return 0;
	
	return 1;
}

//����ת��
void GetPosition(void)
{
	//�����ٶ�����������
	u8 i,j,k;
	
	float theta;
	
	float L_inv[3][3];
	float theta_inv[3][3];
	float tem[3][3];
	
	//ȡTheta������ֵ
	if(fabs(BasketballRobot.ThetaR - BasketballRobot.LastTheta) < PI)
		theta = (BasketballRobot.ThetaR + BasketballRobot.LastTheta) / 2.0f;
	else
		theta = (BasketballRobot.ThetaR + BasketballRobot.LastTheta) / 2.0f + PI;
	
	BasketballRobot.LastTheta = BasketballRobot.ThetaR;
	
	//v(encoder)=L * Theta * V
	//theta_inv
	theta_inv[0][0]= cos(theta);	theta_inv[0][1] = -sin(theta);		theta_inv[0][2] = 0;
	theta_inv[1][0]= sin(theta);	theta_inv[1][1] = cos(theta);			theta_inv[1][2] = 0;
	theta_inv[2][0]= 0;			theta_inv[2][1] = 0;				theta_inv[2][2] = 1;
	//		[-cos(0)		-sin(0)		ENCODER_L]-1
	//L_inv=	[-cos(120) 	-sin(120)		ENCODER_L]
	//		[-cos(-120)	-sin(-120)	ENCODER_L]
	L_inv[0][0] = -0.666666666666667;		L_inv[0][1] =  0.333333333333333;		L_inv[0][2] = 0.333333333333333;
	L_inv[1][0] =  0;					L_inv[1][1] = -0.577350269189626;		L_inv[1][2] = 0.577350269189626;
	L_inv[2][0] =  1.666666666666667;		L_inv[2][1] =  1.666666666666667;		L_inv[2][2] = 1.666666666666667;
	
	//�������
	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			tem[i][j] = 0;
			for(k=0;k<3;k++)
				tem[i][j] += theta_inv[i][k] * L_inv[k][j];
		}
	}
	
	BasketballRobot.Vx = 0;
	for(j=0;j<3;j++){
		BasketballRobot.Vx += tem[0][j] * BasketballRobot.v[j];
	}
	
	BasketballRobot.Vy = 0;
	for(j=0;j<3;j++){
		BasketballRobot.Vy += tem[1][j] * BasketballRobot.v[j];
	}
	
	BasketballRobot.W = 0;
	for(j=0;j<3;j++){
		BasketballRobot.W += tem[2][j] * BasketballRobot.v[j];
	}
	
	
	BasketballRobot.X += BasketballRobot.Vx*0.01f;
	BasketballRobot.Y += BasketballRobot.Vy*0.01f;
}

//����ת��,������̼ƶ�λ
void GetPosition2(void)
{
	//�����ٶ�����������

	
	float D_theta; //�ǶȲ�
	float cot_A;	//�ƶ������������ϵY��н�a ����
	
	float l1,l2;	//����̼Ƽ�ȥ����ƫ�����ֵ
	
	float theta_inv[2][2]; //�ǶȾ���
	
	
	//ȡ�ǶȲ�
	
	D_theta = BasketballRobot.ThetaR - BasketballRobot.LastTheta;
	BasketballRobot.LastTheta = BasketballRobot.ThetaR;
	
	//|xd yd|=|x0 y0| + |L1 L1/tan(a)|*Theta
	
	//theta_inv
	//theta_inv[0][0]= sin(BasketballRobot.ThetaR);	theta_inv[0][1] = -sin(BasketballRobot.ThetaR);		
	//theta_inv[1][0]= cos(BasketballRobot.ThetaR);	theta_inv[1][1] = cos(BasketballRobot.ThetaR);
	theta_inv[0][0]= sin(BasketballRobot.ThetaR);	theta_inv[0][1] = -theta_inv[0][0];		
	theta_inv[1][0]= cos(BasketballRobot.ThetaR);	theta_inv[1][1] = theta_inv[1][0];	
	
	//��ȥ�Դ�ƫ��
	l1 = BasketballRobot.v[0]*0.01f - ENCODER_L*D_theta;
	l1 = -l1;
	
	l2 = BasketballRobot.v[1]*0.01f - ENCODER_L*D_theta;
	l2 =-l2;
	
	if((l1-2*l2) == 0)
	{
		BasketballRobot.X += l1;
		BasketballRobot.Y += 0;
	}
	else if(l1 ==0)
	{
		BasketballRobot.X += 0;
		BasketballRobot.Y += l2/2*1.73205081f;
	}
	else
	{
		//1/tan(a) = 1.732*l1 /(l1-2*l2)
		cot_A = 1.73205081f*l1/(l1-2*l2);

		BasketballRobot.X += l1*theta_inv[0][1]+l1*cot_A*theta_inv[0][0];
		BasketballRobot.Y += l1*theta_inv[1][1]+l1*cot_A*theta_inv[1][0];
		
		BasketballRobot.W = D_theta*100;
	}		
	
	

}

//����ƫ���С�������ٶ�
static float AdjustAngleV(float D_Theta)
{
	float Vw = 0;
	
	//����30�����Կ���
	if(D_Theta>0&&(D_Theta<180))  
	{
		Vw=D_Theta;
	}
	else if(D_Theta>0&&(D_Theta>=180)) 
	{
		D_Theta = 360-D_Theta;
		Vw=-D_Theta;
	}
	else if(D_Theta<0&&(D_Theta>=-180)) 
	{
		D_Theta = -D_Theta;
		Vw=-D_Theta;
	}
	else if(D_Theta<0&&(D_Theta<-180)) 
	{
		D_Theta = 360+D_Theta;
		Vw=D_Theta;
	}
	else 
		//Vw=Vw;
	
	//С��60�����30������
	//ʵ��PWMΪ100
	if(D_Theta < 60)
	{
		if(Vw>0)
			Vw = 1000;
		else
			Vw = -1000;
	}
	
	//С��30�����5��
	if(D_Theta < 30)
	{
		if(Vw>30)
			Vw = 200;
		else
			Vw = -200;
	}
	//С��5��
	if(D_Theta < 5)
	{
		if(Vw>0)
			Vw = 40;
		else
			Vw = -40;
	}
	if(D_Theta == 0)
		Vw = 0;
	
	return Vw;
}


//����ƫ���С����Y���ٶ�
static float AdjustVy(float D_Y)
{
	float sy;
	
	if(D_Y > 0.05f)
	{
			
		if(D_Y >= 1.5f)
		{
			sy = 8;
			if(BasketballRobot.Vy < 0.3f)
				sy = 2;
			else if(BasketballRobot.Vy < 0.5f)
				sy = 4;
			else if(BasketballRobot.Vy < 0.9f)
				sy = 6;
		}
		if(D_Y < 1.5f&&D_Y > 0.2f)  
			sy = D_Y*10/2;
			
		if(D_Y < 0.2f)	
			sy = 0.25;			
		}
	else if(D_Y < -0.05f)
	{
		if(D_Y < -1.5f)
		{
			sy = -8;
			if(BasketballRobot.Vy > -0.3f)
				sy = -2;
			else if(BasketballRobot.Vy > -0.5f)
				sy = -4;
			else if(BasketballRobot.Vy > -0.9f)
				sy = -6;
		}
		
		if(D_Y > -1.5f&&D_Y < -0.2f)  
			sy = D_Y*10/2;
			
		if(D_Y > -0.2f)
			sy = -0.25;
		}
	else 
		sy = 0;
	
	return sy;
	
}
	

//����ƫ���С����X���ٶ�
static float AdjustVx(float D_X)
{
	float sx;
	
	if(D_X > 0.05f)
	{
		if(D_X > 1.5f)
		{
			sx = 8;
			if(BasketballRobot.Vx < 0.1f)
				sx = 0.5;
			else if(BasketballRobot.Vx < 0.2f)
				sx = 1;
			else if(BasketballRobot.Vx < 0.4f)
				sx = 2;
			else if(BasketballRobot.Vx < 0.58f)
				sx = 4;
			else if(BasketballRobot.Vx < 7.5f)
				sx = 6;
		}
		else if(D_X < 1.5f)
		{
			if(D_X > 0.2f)
				sx = 2;
				
			else if(D_X > 0.15f)
			sx = 1;
				
			else if(D_X > 0.1f)
				sx = 0.5f;
			
			else
				sx = 0.25f;	
		}
	}
	else if(D_X < -0.05f)
	{
		if(D_X < -1.5f)
		{
			sx = -8;
			if(BasketballRobot.Vx > -0.1f)
				sx = -0.5;
			else if(BasketballRobot.Vx > -0.2f)
				sx = -1;
			else if(BasketballRobot.Vx > -0.4f)
				sx = -2;
			else if(BasketballRobot.Vx > -0.58f)
				sx = -4;
			else if(BasketballRobot.Vx > -7.5f)
				sx = -6;
		}
		else if(D_X > -1.5f)
		{
			if(D_X < -0.2f)
				sx = -2;
				
			else if(D_X < -0.15f)
				sx = -1;

			else if(D_X < -0.1f)
				sx = -0.5f;
			else
				sx = -0.25f;
		}
	}
	else 
		sx = 0;
	
	return sx;
	
}


//�����˶����������Ƕȣ��Զ�����
void RobotRotate(float theta)
{
	float D_Theta;
	float Vw=0;        //W����0 ��ʱ��

	//D_Theta = theta-BasketballRobot.ThetaD;
	D_Theta = theta-0;
	Vw = AdjustAngleV(D_Theta);
	
	
	while(D_Theta>1||D_Theta < -1)
	{
		GetMotorVelocity(0,0,Vw);	
		
		SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
		
		D_Theta = theta-BasketballRobot.ThetaD;
		
		Vw = AdjustAngleV(D_Theta);
	}
	SetPWM(0,0,0);

	while(BasketballRobot.W);
}


//����ָ������
//X_I:Ŀ�������X
//Y_I:Ŀ�������Y
//Theta_I:Ŀ������ĽǶ�
void RobotGoTo(float X_I,float Y_I,float Theta_I)
{
	float D_Theta,D_X,D_Y,Vw=0,sx,sy=0;
	
	D_Theta =  Theta_I - BasketballRobot.ThetaD;	//�ǶȲ�
	D_X = X_I - BasketballRobot.X;				
	D_Y = Y_I - BasketballRobot.Y;
	
	while(fabs(D_Y) > 0.05f || fabs(D_X) > 0.05f)
	{
		sy = AdjustVy(D_Y);		
		
		sx = AdjustVx(D_X);
		
		Vw = AdjustAngleV(D_Theta)/2;
		
		GetMotorVelocity(sx*12,sy*100,Vw);
		
		SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
		
		D_Theta =  Theta_I - BasketballRobot.ThetaD;
		D_X = X_I - BasketballRobot.X;
		D_Y = Y_I - BasketballRobot.Y;
	}
	SetPWM(0,0,0);
	delay_ms(1000);
	RobotRotate(Theta_I);
}









