#include "timer.h"

uint8_t s_2msFalg;
uint8_t s_1sFalg;

uint8_t Get2msFlag(void)	//��ȡ2ms��־λ��ֵ
{
  return s_2msFalg;
}

void  Clr2msFlag(void)		//���2ms��־λ
{
  s_2msFalg = FALSE;
}

uint8_t Get1SecFlag(void)	//��ȡ1s��־λ��ֵ
{
  return s_1sFalg;
}

void  Clr1SecFlag(void)		//���1s��־λ
{
  s_1sFalg = FALSE;
}

