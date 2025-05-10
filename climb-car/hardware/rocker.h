#ifndef __ROCKER_H__
#define __ROCKER_H__
#include "main.h"

/*************************************/
// 说明 
// Pc0用于ADC1_IN10,连接摇杆VRx
// Pc1用于ADC2_IN11,连接摇杆VRy
// Pc3连接遥感SW
// 实现18种按键状态
/*************************************/

typedef struct
{
  uint32_t Y;
  uint32_t X;
  uint32_t Z;
}xyzVal_struct;  //xyz轴值结构体

typedef enum
{
  KEYSTOP = 0,  //摇杆不动    /* 0 */
  KEYUP,        //摇杆上推    /* 1 */
  KEYSUP,       //摇杆慢上推  /* 2 */
  KEYDOWN,      //摇杆下推    /* 3 */
  KEYSDOWN,     //摇杆慢下推  /* 4 */
  KEYLEFT,      //摇杆左推    /* 5 */
  KEYSLEFT,     //摇杆慢左推  /* 6 */
  KEYRIGHT,     //摇杆右推    /* 7 */
  KEYSRIGHT,    //摇杆慢右推  /* 8 */
  KEYPRESS,     //摇杆按下    /* 9 */
  KEYUPLF,      //摇杆上左推      /* 10 */
  KEYSUPLF,     //摇杆慢上左推    /* 11 */
  KEYUPRI,      //摇杆上右推      /* 12 */
  KEYSUPRI,     //摇杆慢上左推    /* 13 */
  KEYDOWNLF,    //摇杆下左推      /* 14 */
  KEYSDOWNLF,   //摇杆慢下左推    /* 15 */
  KEYDOWNRI,    //摇杆下右推      /* 16 */
  KEYSDOWNRI,   //摇杆慢下右推    /* 17 */
  KEYState_MAX                /* 18 */
}E_RKEYState;

typedef enum
{
  Yup = 0,    //y轴上位    /* y0 */
  Ysup,       //y轴慢上位  /* y1 */
  Ysp,        //y轴中位    /* y2 */
  Ydn,        //y轴下位    /* y4 */
  Ysdn        //y轴慢下位  /* y3 */
}E_YaxleState;

typedef enum
{
  Xlf = 0,     //x轴左位   /* x0 */
  Xslf,        //x轴慢左位 /* x1 */ 
  Xsp,         //x轴中位   /* x2 */
  Xri,         //x轴右位   /* x4 */
  Xsri         //x轴慢右位 /* x3 */ 
}E_XaxleState;

typedef enum
{
  Zup = 0,    //z轴上位    /* z0 */
  Zdn         //z轴下位    /* z1 */ 
}E_ZaxleState;

void Get_xyzVal(void);           //获取xy轴的值
E_RKEYState Get_KeyState(void); //获取摇杆的状态
void Test(E_RKEYState state);   //测试摇杆功能

#endif 

