#include "HCTL2020.h"
#include "delay.h" 
#include "usart.h"
/*!
 *  @brief      hctl2020 �ӿڳ�ʼ��
 *  @since      v5.0
 *  Sample usage:            hctl2020_init();   //��ʼ�� hctl2020
 */
 
 #define time 1					//��ʱʱ��
  
void hctl2020_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//ʹ��GPIOFʱ��
		
	//F0~7���������ݶ���
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;			//��ͨ����ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//100M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//
	GPIO_Init(GPIOF, &GPIO_InitStructure);				//��ʼ��

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOCʱ��
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12; //10-SEL,11-OE,12-RST  ������0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ����ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_PuPd =  GPIO_PuPd_UP;//����
	GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIOC
	
	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOB
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;//������1   RST
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_PuPd =  GPIO_PuPd_UP;//����
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOB

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);//ʹ��GPIOB,RES1ʱ��
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15; //10-SEL,11-OE,12-RST  ������1  13-SEL,14-OE,15-RST  ������2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_PuPd =  GPIO_PuPd_UP;//����
	GPIO_Init(GPIOG, &GPIO_InitStructure);//��ʼ��GPIOG
}
/*!
 *  @brief      ��ȡ������
 *  @since      v5.0
 *  Sample usage:            hctl2020_getdata_l();   hctl2020_getdata_l
*/
uint8_t HCTL2020_DATA()
{
	uint8_t l,m;


	m=PFin(7);
	l=m;
	l<<=1;
	
	m=PFin(6);
	l=l|m;
	l<<=1;
	
	m=PFin(5);
	l=l|m;
	l<<=1;
	
	m=PFin(4);
	l=l|m;
	l<<=1;
	
	m=PFin(3);
	l=l|m;
	l<<=1;
	
	m=PFin(2);
	l=l|m;
	l<<=1;
	
	m=PFin(1);
	l=l|m;
	l<<=1;
	
	m=PFin(0);
	l=l|m;
	//printf("HCTL2020_DATA:%d  us\r\n",l);
	
	return l;
	
}




int16_t hctl2020_getdata_0()
{
	
	uint8_t l=0,h=0;

	//���߰�λ
	HCTL2020_SEL0=0;
	HCTL2020_OE0=0;
	delay_us(20);
	h=HCTL2020_DATA();
	
	//���ڰ�λ
	HCTL2020_SEL0=1;
	HCTL2020_OE0=0;
	delay_us(time);
	l=HCTL2020_DATA();
	
	//��λ �ظ�����
	HCTL2020_OE0=1;
	HCTL2020_RST0=0;
	delay_us(time);
	HCTL2020_RST0=1;	
	
	HCTL2020_OE0=1;
	HCTL2020_RST0=0;
	delay_us(time);
	HCTL2020_RST0=1;
	
	HCTL2020_OE0=1;
	HCTL2020_RST0=0;
	delay_us(time);
	HCTL2020_RST0=1;
	
	//�ߡ���8λ �ϲ�
	return (h<<8)+l; 
	
}

int16_t hctl2020_getdata_1()
{

	uint8_t l=0,h=0;

	HCTL2020_SEL1=0;
	HCTL2020_OE1=0;
	delay_us(20);
	h=HCTL2020_DATA();


	HCTL2020_SEL1=1;
	HCTL2020_OE1=0;
	delay_us(time);
	l=HCTL2020_DATA();
	//printf("one:       %d  us\r\n",h);
	
	HCTL2020_OE1=1;
	HCTL2020_RST1=0;
	delay_us(time);
	HCTL2020_RST1=1;
	
	
	HCTL2020_OE1=1;
	HCTL2020_RST1=0;
	delay_us(time);
	HCTL2020_RST1=1;
	
	HCTL2020_OE1=1;
	HCTL2020_RST1=0;
	delay_us(time);
	HCTL2020_RST1=1;
	
	
	
	return (h<<8)+l; 
}


   int16_t hctl2020_getdata_2()
{

	
	uint8_t l=0,h=0;
	//  delay_us(time);
	HCTL2020_SEL2=0;
	HCTL2020_OE2=0;
	delay_us(20);
	h=HCTL2020_DATA();
	//printf("one:%d  us\r\n",h);
	HCTL2020_SEL2=1;
	HCTL2020_OE2=0;
	delay_us(time);
	l=HCTL2020_DATA();
	//printf("one:       %d  us\r\n",l);
	HCTL2020_OE2=1;
	HCTL2020_RST2=0;
	delay_us(time);
	HCTL2020_RST2=1;
	
	
	HCTL2020_OE2=1;
	HCTL2020_RST2=0;
	delay_us(time);
	HCTL2020_RST2=1;
	HCTL2020_OE2=1;
	HCTL2020_RST2=0;
	delay_us(time);
	HCTL2020_RST2=1;


	return (h<<8)+l; 
		
}

//������0��λ
void hctl2020_RST0(void)
{    HCTL2020_RST0=0;
  delay_us(time);
  HCTL2020_RST0=1;
}
//������1��λ
void hctl2020_RST1(void)
{  HCTL2020_RST1=0;
  delay_us(time);
  HCTL2020_RST1=1;
}
//������2��λ
void hctl2020_RST2(void)
{  HCTL2020_RST2=0;
  delay_us(time);
  HCTL2020_RST2=1;
}
 
