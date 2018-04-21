#ifndef __HCTL2020_H
#define __HCTL2020_H
#include "sys.h"


//#define HCTL2020_DATA       PTC_B0_IN
#define HCTL2020_SEL0       PCout(10)
#define HCTL2020_OE0        PCout(11)
#define HCTL2020_RST0       PCout(12)
#define HCTL2020_SEL1       PGout(10)
#define HCTL2020_OE1        PGout(11)
#define HCTL2020_RST1       PBout(6)//PGout(12)
#define HCTL2020_SEL2       PGout(13)
#define HCTL2020_OE2        PGout(14)
#define HCTL2020_RST2       PGout(15)


void hctl2020_init(void);                        //��ʼ��HCTL2020

//��ȡ��������ֵ ��ȡ������
int16_t hctl2020_getdata_0(void);
int16_t hctl2020_getdata_1(void);
int16_t hctl2020_getdata_2(void);

#endif  //__FIRE_HCTL2020_H__
