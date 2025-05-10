#include "main.h"
#include "track.h"
#include <stdint.h> // Include this for uint8_t



void Get_Sensor_Value(void)
{

    // 不使用滤波，直接读取当前值
    Sensor_Left = HAL_GPIO_ReadPin(SENSOR1_PORT, SENSOR1_PIN);        // 左
    Sensor_MLeft = HAL_GPIO_ReadPin(SENSOR2_PORT, SENSOR2_PIN);  // 中左
    Sensor_MMLeft = HAL_GPIO_ReadPin(SENSOR3_PORT, SENSOR3_PIN);
    Sensor_MMMLeft = HAL_GPIO_ReadPin(SENSOR4_PORT, SENSOR4_PIN);

    Sensor_MMMRight = HAL_GPIO_ReadPin(SENSOR5_PORT, SENSOR5_PIN);     // 中
    Sensor_MMRight = HAL_GPIO_ReadPin(SENSOR6_PORT, SENSOR6_PIN);
    Sensor_MRight = HAL_GPIO_ReadPin(SENSOR7_PORT, SENSOR7_PIN); // 中右
    Sensor_Right = HAL_GPIO_ReadPin(SENSOR8_PORT, SENSOR8_PIN);       // 右

}

int  track(void)
{
    static float sensor_bias = 0;     
    static float sensor_bias_last = 0; 

    float PID_value = 0;               
 
 
    // 读取传感器值
    Get_Sensor_Value();

    // 根据传感器组合判断偏差，采用新的循迹逻辑

    if (Sensor_Left == 1  && Sensor_MLeft == 1 && Sensor_MMLeft == 1 && Sensor_MMMLeft == 1 && Sensor_MMMRight == 1 && Sensor_MMRight == 1 && 
        Sensor_MRight == 1 && Sensor_Right == 1)
    {
        // 全黑状态，可能是十字路口
       sensor_bias= sensor_bias_last ;
    }
   





    else  if (Sensor_Left == 0  && Sensor_MLeft == 1 && Sensor_MMLeft == 1 && Sensor_MMMLeft == 1 && Sensor_MMMRight == 1 && Sensor_MMRight == 1 && 
        Sensor_MRight == 1 && Sensor_Right == 1)
    {
        sensor_bias = -60;
      //  decide = 2;
    }
    else  if (Sensor_Left == 0  && Sensor_MLeft == 0 && Sensor_MMLeft == 1 && Sensor_MMMLeft == 1 && Sensor_MMMRight == 1 && Sensor_MMRight == 1 && 
        Sensor_MRight == 1 && Sensor_Right == 1)
    {
        sensor_bias = -60;
      //  decide = 2;
    }

    else  if (Sensor_Left == 1  && Sensor_MLeft == 0 && Sensor_MMLeft == 1 && Sensor_MMMLeft == 1 && Sensor_MMMRight == 1 && Sensor_MMRight == 1 && 
        Sensor_MRight == 1 && Sensor_Right == 1)
    {
        sensor_bias = -40;
     //   decide = 2; 
    }
    else  if (Sensor_Left == 1  && Sensor_MLeft == 0 && Sensor_MMLeft == 0 && Sensor_MMMLeft == 1 && Sensor_MMMRight == 1 && Sensor_MMRight == 1 && 
        Sensor_MRight == 1 && Sensor_Right == 1)
    {
        sensor_bias = -40;
     //   decide = 2;
    }

    else  if (Sensor_Left == 1  && Sensor_MLeft == 1 && Sensor_MMLeft == 0 && Sensor_MMMLeft == 1 && Sensor_MMMRight == 1 && Sensor_MMRight == 1 && 
        Sensor_MRight == 1 && Sensor_Right == 1)
    {
        sensor_bias = -20;
     //   decide = 4;
    }
    else  if (Sensor_Left == 1  && Sensor_MLeft == 1 && Sensor_MMLeft == 0 && Sensor_MMMLeft == 0 && Sensor_MMMRight == 1 && Sensor_MMRight == 1 && 
        Sensor_MRight == 1 && Sensor_Right == 1)
    {
        sensor_bias = -15;
     //   decide = 4;
    }

    else  if (Sensor_Left == 1  && Sensor_MLeft == 1 && Sensor_MMLeft == 1 && Sensor_MMMLeft == 0 && Sensor_MMMRight == 1 && Sensor_MMRight == 1 && 
        Sensor_MRight == 1 && Sensor_Right == 1)
    {
        sensor_bias = -8;
     //   decide = 4;
    }  



    else  if (Sensor_Left == 1  && Sensor_MLeft == 1 && Sensor_MMLeft == 1 && Sensor_MMMLeft == 1 && Sensor_MMMRight == 0 && Sensor_MMRight == 1 && 
        Sensor_MRight == 1 && Sensor_Right == 1)
    {
        sensor_bias = 8;
  //   decide = 2;
    }

    else  if (Sensor_Left == 1  && Sensor_MLeft == 1 && Sensor_MMLeft == 1 && Sensor_MMMLeft == 1 && Sensor_MMMRight == 1 && Sensor_MMRight == 0 && 
        Sensor_MRight == 1 && Sensor_Right == 1)
    {
        sensor_bias = 20;
  //  decide = 2;
    }
    else  if (Sensor_Left == 1  && Sensor_MLeft == 1 && Sensor_MMLeft == 1 && Sensor_MMMLeft == 1 && Sensor_MMMRight == 0 && Sensor_MMRight == 0 && 
        Sensor_MRight == 1 && Sensor_Right == 1)
    {
        sensor_bias = 15;
  //  decide = 2;
    }
   
    else  if (Sensor_Left == 1  && Sensor_MLeft == 1 && Sensor_MMLeft == 1 && Sensor_MMMLeft == 1 && Sensor_MMMRight == 1 && Sensor_MMRight == 1 && 
        Sensor_MRight == 0 && Sensor_Right == 1)
    {
        sensor_bias = 40;
  //    decide = 4;
    }
    else  if (Sensor_Left == 1  && Sensor_MLeft == 1 && Sensor_MMLeft == 1 && Sensor_MMMLeft == 1 && Sensor_MMMRight == 1 && Sensor_MMRight == 0 && 
        Sensor_MRight == 0 && Sensor_Right == 1)
    {
        sensor_bias = 40;
  //    decide = 4;
    }

    else  if (Sensor_Left == 1  && Sensor_MLeft == 1 && Sensor_MMLeft == 1 && Sensor_MMMLeft == 1 && Sensor_MMMRight == 1 && Sensor_MMRight == 1 && 
        Sensor_MRight == 1 && Sensor_Right == 0)
    {
        sensor_bias = 60;
//      decide = 4;
    }
    else  if (Sensor_Left == 1  && Sensor_MLeft == 1 && Sensor_MMLeft == 1 && Sensor_MMMLeft == 1 && Sensor_MMMRight == 1 && Sensor_MMRight == 1 && 
        Sensor_MRight == 0 && Sensor_Right == 0)
    {
        sensor_bias = 60;
//      decide = 4;
    }


    else if ((Sensor_Left + Sensor_MLeft + Sensor_MMLeft + Sensor_MMMLeft + Sensor_MMMRight + Sensor_MMRight + 
        Sensor_MRight + Sensor_Right)<=5)
    {
        // 如果有多个传感器检测到黑线，停下
        sensor_bias = 1;
        
        
    }

    else
    {
        // 其他情况，保持上一次偏差
        sensor_bias = 0;
    }   
   
        sensor_bias_last = sensor_bias;
        PID_value = sensor_bias;
        return (float)PID_value;

}

/**
 * @brief  初始化红外传感器引脚
 * @param  无
 * @retval 无
 */
void Init_Sensor_Pins(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 启用GPIO时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* 配置传感器1引脚 - 左传感器 */
    GPIO_InitStruct.Pin = SENSOR1_PIN;
    HAL_GPIO_Init(SENSOR1_PORT, &GPIO_InitStruct);

    /* 配置传感器2引脚 - 中左传感器 */
    GPIO_InitStruct.Pin = SENSOR2_PIN;
    HAL_GPIO_Init(SENSOR2_PORT, &GPIO_InitStruct);

    /* 配置传感器3引脚 - 中间传感器 */
    GPIO_InitStruct.Pin = SENSOR3_PIN;
    HAL_GPIO_Init(SENSOR3_PORT, &GPIO_InitStruct);

    /* 配置传感器4引脚 - 中右传感器 */
    GPIO_InitStruct.Pin = SENSOR4_PIN;
    HAL_GPIO_Init(SENSOR4_PORT, &GPIO_InitStruct);

    /* 配置传感器5引脚 - 右传感器 */
    GPIO_InitStruct.Pin = SENSOR5_PIN;
    HAL_GPIO_Init(SENSOR5_PORT, &GPIO_InitStruct);

    /* 配置传感器6引脚 - 中右传感器 */
    GPIO_InitStruct.Pin = SENSOR5_PIN;
    HAL_GPIO_Init(SENSOR6_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SENSOR7_PIN;
    HAL_GPIO_Init(SENSOR7_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SENSOR8_PIN;
    HAL_GPIO_Init(SENSOR8_PORT, &GPIO_InitStruct);


/*
#if FILTER_SAMPLES > 0
    for (int i = 0; i < 5; i++)
    {
        for (int j = 0; j < FILTER_SAMPLES; j++)
        {
            sensor_history[i][j] = 0;
        }
    }
    filter_index = 0;
#endif
*/
} 

