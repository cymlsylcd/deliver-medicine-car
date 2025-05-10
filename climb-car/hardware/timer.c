#include "timer.h"

uint8_t s_2msFalg;
uint8_t s_1sFalg;

uint8_t Get2msFlag(void)	//获取2ms标志位的值
{
  return s_2msFalg;
}

void  Clr2msFlag(void)		//清除2ms标志位
{
  s_2msFalg = FALSE;
}

uint8_t Get1SecFlag(void)	//获取1s标志位的值
{
  return s_1sFalg;
}

void  Clr1SecFlag(void)		//清除1s标志位
{
  s_1sFalg = FALSE;
}

