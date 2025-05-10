/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "track.h"
#include <stdbool.h>
#include "oled.h"
#include "rocker.h"
#include <math.h>

/*********************** 宏定义 ***********************/
#define k1 HAL_GPIO_ReadPin(k1_GPIO_Port, k1_Pin) // 读取PA15引脚的状态
#define k2 HAL_GPIO_ReadPin(k2_GPIO_Port, k2_Pin) // 读取PD8引脚的状态 
#define k3 HAL_GPIO_ReadPin(k3_GPIO_Port, k3_Pin) // 读取PD9引脚的状态

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*********************** 全局变量 ***********************/
int VL, VR;                      // 左右轮速度
int counter;                     // 计数器
volatile int EL0=0, EL1=0;      // 左编码器值
volatile int ER0=0, ER1=0;      // 右编码器值
volatile int NL=0, NR=0;        // 编码器增量
volatile int flag=0, time=0;     // 标志位和时间
volatile int slope=10, s=10;     // 斜坡角度和距离
float targetspeed=70, error;     // 目标速度和误差

/*********************** PID控制器结构体 ***********************/
typedef struct {
    float Kp;            // 比例系数
    float Ki;            // 积分系数
    float Kd;           // 微分系数
    float integral;      // 积分项
    float prev_error;    // 上次误差
    float output;        // 输出值
    float max_output;    // 最大输出
    float min_output;    // 最小输出
} PID_Controller;

PID_Controller pidL, pidR;  // 左右轮PID控制器

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*********************** 主循环状态定义 ***********************/
#define STATE_INIT    0  // 初始化状态
#define STATE_CONFIG  1  // 配置状态  
#define STATE_RUNNING 2  // 运行状态

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int V(void); // Forward declaration of the V function
float V_fit(int slope, int s); // Forward declaration of the V_fit function
int16_t get_speed(uint8_t slope_idx, uint8_t s_idx); // Forward declaration of the get_speed function
void Update_TimeDisplay(int time); // Forward declaration of the Update_TimeDisplay function
void Stop_Robot(void); // Forward declaration of the Stop_Robot function
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*********************** 函数定义 ***********************/
void PID_Init(PID_Controller* pid, float Kp, float Ki, float Kd, float min_out, float max_out) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->output = 0;
    pid->min_output = min_out;
    pid->max_output = max_out;
}

void PID_Update(PID_Controller* pid, float error) {
    // 更新积分项
    pid->integral += error;
    
    // 计算微分项
    float derivative = error - pid->prev_error;
    
    // 计算PID输出
    pid->output = pid->Kp * error + 
                 pid->Ki * pid->integral + 
                 pid->Kd * derivative;
    
    // 输出限幅
    if(pid->output > pid->max_output) pid->output = pid->max_output;
    if(pid->output < pid->min_output) pid->output = pid->min_output;
    
    // 保存当前误差
    pid->prev_error = error;
}

/*********************** 舵机控制函数 ***********************/
void servo(int angle) {
    int duty;
    
    // 角度限幅
    if(angle > 161)
        angle = 161;
    else if(angle <= 31)
        angle = 31;
        
    // 计算PWM占空比
    duty = angle * 200 / 180 + 50;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
}

/*********************** 速度计算函数 ***********************/
float speed(int num) {
    float v;
    v = num * 16.2;
    
    if(num > 600) {
        v = 0;
    }
    return v;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==(&htim3))
	{
	
		EL1=__HAL_TIM_GET_COUNTER(&htim4);
		NL=EL1-EL0;
		if(EL1<EL0)
			NL=EL1+65535-EL0;
		EL0=EL1;
		VL=speed(NL);
    error=targetspeed-VL;
    if (flag == 1)
    {
    time = time + 1;
    PID_Update(&pidL, error);
    }

		ER1=__HAL_TIM_GET_COUNTER(&htim5);
		NR=ER1-ER0;
		if(ER1<ER0)
			NR=ER1+65535-ER0;
		ER0=ER1;
    VR=speed(NR);
    error=targetspeed-VR;
    if (flag==1)
    {
    PID_Update(&pidR,error);
    } 

		
	}
}

/*********************** 辅助函数定义 ***********************/
// 更新时间显示
void Update_TimeDisplay(int time) {
    OLED_ShowNum(0, 4, time / 50, 3, 16);
    OLED_ShowNum(0, 6, time * 2 % 100, 3, 16);
}

// 停止机器人
void Stop_Robot(void) {
    flag = 0;
    go_ahead(0, 0);  // 停止电机
    
    // 控制指示灯
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{	

  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();
  OLED_Clear();
	
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim3); 
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);
  
	Get_KeyState();
    
	PID_Init(&pidL, 0.15, 0.1, 2, 0, 1000);
  PID_Init(&pidR, 0.15, 0.1, 2, 0, 1000);
	pidL.output = 0;
	pidR.output = 0; 
	Init_Sensor_Pins();
	float PID_value;
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	servo(90);
	OLED_ShowString(1,2, "target V: ",8);
	OLED_ShowString(1,3, "target time:", 8);
	OLED_ShowString(40,4,"slope",8);
  
  while (1) {
    // 参数配置循环
    while (1) {
        HAL_Delay(120);  // 按键消抖延时
        
        // 检测K1按键 - 开始/确认
        if (k1 == RESET) {
            // 重置计时并显示
            time = 0;
            Update_TimeDisplay(time);
            
            while (k1 == RESET);  // 等待按键释放
            
            // 进入斜坡角度设置
            while (1) {
                HAL_Delay(120);
                
                if (k1 == RESET) break;  // 确认设置
                
                // 斜坡角度增减
                if (k2 == RESET) slope++;
                if (k3 == RESET) slope--;
                
                // 更新显示
                OLED_ShowNum(80, 4, slope, 2, 16);
                
                // 计算并显示目标速度
                targetspeed = V();
                OLED_ShowNum(100, 1, targetspeed, 3, 16);
            }
            break;
        }
        
        // 距离参数调整
        if (k2 == RESET) s++;
        if (k3 == RESET) s--;
        
        // 更新距离显示
        OLED_ShowNum(100, 3, s, 2, 16);
    }

    // 运行控制循环
    while (1) {
        flag = 1;  // 启动标志
        
        // 更新时间显示
        Update_TimeDisplay(time);
        
        // PID控制和舵机输出
        PID_value = 90 + track();
        servo(PID_value);

        // 检测是否需要停止
        if (PID_value == 91) {
            Stop_Robot();
            break;
        } 
        // 正常运行
        else {
            // 发送速度数据并控制电机
            USART1_Send(VL, VR);
            go_ahead(pidL.output, pidR.output);
        }
    }
  }

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitStructTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitStructTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitStruct structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLLM = 4;
  RCC_OscInitStruct.PLLN = 168;
  RCC_OscInitStruct.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}
/* USER CODE BEGIN 4 */

int V(void)
{
    /************************ 速度查找表 ************************/
    /* 基础表(slope = 0) */
    static const int table_0[] = {69, 64, 57, 52, 48, 45, 42, 39, 37, 35, 34};

    /* 10-19度斜坡速度表 */  
    static const int table_10[] = {70, 64, 58, 53, 49, 46, 43, 40, 38, 36, 35};
    static const int table_11[] = {70, 64, 58, 53, 49, 46, 43, 40, 38, 36, 35};
    static const int table_12[] = {71, 65, 59, 54, 50, 47, 44, 41, 39, 37, 35};
    static const int table_13[] = {71, 65, 59, 54, 50, 47, 44, 41, 39, 37, 35};
    static const int table_14[] = {74, 66, 60, 55, 51, 48, 45, 42, 40, 38, 36};
    static const int table_15[] = {74, 66, 60, 55, 51, 48, 45, 42, 40, 38, 36};
    static const int table_16[] = {74, 66, 60, 55, 51, 48, 45, 42, 40, 38, 36};
    static const int table_17[] = {75, 67, 61, 56, 52, 48, 45, 43, 40, 38, 36};
    static const int table_18[] = {76, 68, 62, 57, 52, 49, 46, 43, 41, 39, 37};
    static const int table_19[] = {77, 68, 63, 57, 53, 49, 46, 44, 41, 39, 37};

    /* 20-29度斜坡速度表 */
    static const int table_20[] = {77, 68, 63, 57, 53, 49, 46, 44, 41, 39, 37};
    static const int table_21[] = {76, 68, 63, 57, 53, 49, 46, 44, 41, 39, 37}; 
    static const int table_22[] = {75, 68, 63, 57, 53, 50, 46, 44, 41, 39, 37};
    static const int table_23[] = {74, 69, 64, 58, 54, 51, 47, 45, 41, 39, 37};
    static const int table_24[] = {73, 69, 64, 58, 54, 52, 47, 45, 41, 39, 37};
    static const int table_25[] = {72, 69, 64, 58, 54, 52, 47, 45, 41, 39, 37};
    static const int table_26[] = {72, 69, 64, 58, 54, 52, 47, 45, 41, 39, 37};
    static const int table_27[] = {73, 69, 64, 59, 55, 52, 48, 45, 42, 39, 37};
    static const int table_28[] = {73, 70, 65, 60, 55, 52, 49, 46, 42, 40, 38};
    static const int table_29[] = {74, 70, 65, 61, 56, 52, 50, 46, 43, 40, 38};

    /* 30-35度斜坡速度表 */
    static const int table_30[] = {74, 70, 65, 61, 56, 52, 50, 46, 43, 40, 38};
    static const int table_31[] = {75, 70, 65, 61, 56, 52, 50, 46, 43, 40, 38};
    static const int table_32[] = {76, 71, 66, 61, 57, 53, 50, 47, 44, 41, 39};
    static const int table_33[] = {78, 71, 67, 61, 57, 53, 51, 47, 44, 41, 39};
    static const int table_34[] = {79, 72, 68, 62, 58, 54, 51, 48, 45, 42, 40};
    static const int table_35[] = {80, 72, 68, 62, 58, 54, 51, 48, 45, 42, 40};

    /* 计算索引并确保在合理范围内 */
    int idx = s - 10;
    idx = (idx < 0) ? 0 : ((idx > 10) ? 10 : idx);

    /* 根据斜坡角度选择对应速度表 */
    switch(slope) {
        /* 基础速度(0度) */
        case 0:  targetspeed = table_0[idx];  break;
        
        /* 10-19度速度 */ 
        case 10: targetspeed = table_10[idx]; break;
        case 11: targetspeed = table_11[idx]; break;
        case 12: targetspeed = table_12[idx]; break;
        case 13: targetspeed = table_13[idx]; break;
        case 14: targetspeed = table_14[idx]; break;
        case 15: targetspeed = table_15[idx]; break;
        case 16: targetspeed = table_16[idx]; break;
        case 17: targetspeed = table_17[idx]; break;
        case 18: targetspeed = table_18[idx]; break;
        case 19: targetspeed = table_19[idx]; break;

        /* 20-29度速度 */
        case 20: targetspeed = table_20[idx]; break;
        case 21: targetspeed = table_21[idx]; break;
        case 22: targetspeed = table_22[idx]; break;
        case 23: targetspeed = table_23[idx]; break;
        case 24: targetspeed = table_24[idx]; break;
        case 25: targetspeed = table_25[idx]; break;
        case 26: targetspeed = table_26[idx]; break;
        case 27: targetspeed = table_27[idx]; break;
        case 28: targetspeed = table_28[idx]; break;
        case 29: targetspeed = table_29[idx]; break;

        /* 30-35度速度 */
        case 30: targetspeed = table_30[idx]; break;
        case 31: targetspeed = table_31[idx]; break;
        case 32: targetspeed = table_32[idx]; break;
        case 33: targetspeed = table_33[idx]; break;
        case 34: targetspeed = table_34[idx]; break;
        case 35: targetspeed = table_35[idx]; break;

        default: break;
    }

    return targetspeed;
}

#include <math.h>
float V_fit(int slope, int s) 
{
  if (s < 10) s = 10;
    if (s > 20) s = 20;
    if (slope < 10) slope = 10;
    if (slope > 35) slope = 35;

    // Core reference parameters
    struct {
        int slope;  // Slope reference point
        float k;    // Proportional coefficient k = targetspeed * s
    } BASE[] = {
        {10, 700.0f},  // 70*10=700
        {15, 740.0f},  // 74*10=740
        {20, 770.0f},  // 77*10=770
        {25, 720.0f},  // 79*10=790
        {30, 740.0f},   // 90*10=900
        {35, 800.0f} 
    };

    // Find the nearest adjacent reference points
    int i;
    for (i = 0; i < 6; i++) {
        if (BASE[i].slope >= slope) break;
    }

    // Handle boundary cases
    if (i == 0) return BASE[0].k / s;
   
    if (i == 6) return BASE[5].k / s;
    

    // Get adjacent reference points
    float high_k = BASE[i].k;
    float low_k = BASE[i - 1].k;
    int high_slope = BASE[i].slope;
    int low_slope = BASE[i - 1].slope;

    // Calculate interpolation weight
    float t = (float)(slope - low_slope) / (high_slope - low_slope);

    // Interpolate and return the final speed
    float k = low_k * (1 - t) + high_k * t;
    return k / s;

}


/*
 // 定义坡度表结构
    static const int slopes[] = {0, 10, 15, 20, 25, 30, 35};
    static const int* tables[] = {
        table_0, table_10, table_15, 
        table_20, table_25, table_30, table_35
    };
    const int num_slopes = sizeof(slopes) / sizeof(slopes[0]);

    // 计算索引idx并确保在[0,10]范围内
    int idx = s - 10;
    if (idx < 0) idx = 0;
    else if (idx > 10) idx = 10;

    // 处理边界情况（slope超出最大/最小值）
    if (slope <= slopes[0]) {
        return tables[0][idx];
    } else if (slope >= slopes[num_slopes-1]) {
        return tables[num_slopes-1][idx];
    }

    // 查找相邻的坡度区间
    int i = 0;
    while (i < num_slopes-1 && slopes[i+1] < slope) {
        i++;
    }

    // 计算插值比例
    int slope_low = slopes[i];
    int slope_high = slopes[i+1];
    float ratio = (float)(slope - slope_low) / (slope_high - low_slope);

    // 获取相邻坡度表的值并插值
    int value_low = tables[i][idx];
    int value_high = tables[i+1][idx];
    return (int)(value_low + ratio * (value_high - value_low) + 0.5); 




if (s < 10) s = 10;
    if (s > 20) s = 20;
    if (slope < 10) slope = 10;
    if (slope > 30) slope = 30;

    // Core reference parameters
    struct {
        int slope;  // Slope reference point
        float k;    // Proportional coefficient k = targetspeed * s
    } BASE[] = {
        {10, 700.0f},  // 70*10=700
        {15, 740.0f},  // 74*10=740
        {20, 770.0f},  // 77*10=770
        {25, 790.0f},  // 79*10=790
        {30, 900.0f}   // 90*10=900
    };

    // Find the nearest adjacent reference points
    int i;
    for (i = 0; i < 5; i++) {
        if (BASE[i].slope >= slope) break;
    }

    // Handle boundary cases
    if (i == 0) return BASE[0].k / s;
    if (i == 5) return BASE[4].k / s;

    // Get adjacent reference points
    float high_k = BASE[i].k;
    float low_k = BASE[i - 1].k;
    int high_slope = BASE[i].slope;
    int low_slope = BASE[i - 1].slope;

    // Calculate interpolation weight
    float t = (float)(slope - low_slope) / (high_slope - low_slope);

    // Interpolate and return the final speed
    float k = low_k * (1 - t) + high_k * t;
    return k / s;


while (1)
  {	  
	 if(u==1)
		 {break;}	  
      
		E_RKEYState keystate = Get_KeyState(); // 获取摇杆状态
      
      // 根据状态执行相应操作
      switch(keystate)
      {
          case KEYSUP:
              // 向上操作
            s=s+1;
          
              break;
          case KEYSDOWN:
              // 向下操作
            s=s-1;
              break;
          case KEYSLEFT:
              // 向左操作
              targetspeed=targetspeed-10;
              break;
          case KEYSRIGHT:
              // 向右操
              targetspeed=targetspeed+10;
              break;
              case KEYPRESS:
              u=1;

              
          // 可以添加其他状态的处理...
			}
      
     
      OLED_ShowNum(108, 3, s, 2, 16);
      OLED_ShowNum(108, 2, targetspeed, 3, 8);
      //HAL_Delay(100); // 适当的延时，防止读取过快
  } */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
