#include "uprint.h"
#include "stdint.h"
#include "stdarg.h"
#include "stdio.h"
#include "usart.h"

#define CONSOLEBUF_SIZE 64

void PrintfDebug(const char *fmt, ...)
{
	char Uart_buf[CONSOLEBUF_SIZE];
  uint16_t len = 0;
  
	va_list args;
	va_start(args, fmt);
	len = vsprintf(Uart_buf, fmt, args);
	va_end(args);
	
	HAL_UART_Transmit(&huart1, (uint8_t  *)Uart_buf, len, 0xf);
}

