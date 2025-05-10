#ifndef _TIMER_H_
#define _TIMER_H_

#include "main.h"

extern uint8_t s_2msFalg;
extern uint8_t s_1sFalg;

typedef enum
{
  FALSE = 0,
  TURE
}E3_MODE;

uint8_t    Get2msFlag(void);     //获取2ms标志位的值
void  Clr2msFlag(void);     //清除2ms标志位

uint8_t    Get1SecFlag(void);    //获取1s标志位的值
void  Clr1SecFlag(void);    //清除1s标志位

#endif //_U_TIMER_H_
