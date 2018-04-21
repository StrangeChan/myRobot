#ifndef __FINDBALL_H__
#define __FINDBALL_H__

#include "control.h"





void FindBasketball(void);
void FindVolleyball(void);



void FindBall_vision(u8 ball);		//视觉找球,限制4米以内
void FindBall_radar(void);			//利用激光找球
void FindBall_VandR(u8 ball);			//视觉、雷达找球结合





#endif
