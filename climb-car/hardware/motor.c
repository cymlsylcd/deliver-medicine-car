#include "motor.h"

void go_ahead(int speedA,int speedB)
{
	HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_RESET);
	if(speedA<0)
		speedA=0;
	else if(speedA>1000)
		speedA=1000;	
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,speedA);
	if(speedB<0)
		speedB=0;
	else if(speedB>1000)
		speedB=1000;	
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,speedB);
}







