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

uint8_t    Get2msFlag(void);     //��ȡ2ms��־λ��ֵ
void  Clr2msFlag(void);     //���2ms��־λ

uint8_t    Get1SecFlag(void);    //��ȡ1s��־λ��ֵ
void  Clr1SecFlag(void);    //���1s��־λ

#endif //_U_TIMER_H_
