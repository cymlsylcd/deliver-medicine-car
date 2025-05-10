#include "rocker.h"
#include "adc.h"
#include "uprint.h"
#include "dma.h"

xyzVal_struct xyzVal;
uint16_t g_iAdcx[2];


/*函数名称:获取xyz轴的值*/
/*形参:无              */
/*返回值:无            */
void Get_xyzVal(void)
{ 
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_iAdcx, sizeof(g_iAdcx) / sizeof(g_iAdcx[0]));
//  PrintfDebug("Y = %d\r\nX = %d\r\n", g_iAdcx[0], g_iAdcx[1]);
  if(g_iAdcx[0]<1000 && g_iAdcx[0]>0)          //y0
    xyzVal.Y = Yup;
  else if(g_iAdcx[0]<1800 && g_iAdcx[0]>1000)  //y1
    xyzVal.Y = Ysup;                           
  else if(g_iAdcx[0]<2200 && g_iAdcx[0]>1800)  //y2
    xyzVal.Y = Ysp;                             
  else if(g_iAdcx[0]<3000 && g_iAdcx[0]>2200)  //y3
    xyzVal.Y = Ysdn;                           
  else if(g_iAdcx[0]<4200 && g_iAdcx[0]>3000)  //y4
    xyzVal.Y = Ydn;
  
  
  if(g_iAdcx[1]<1000 && g_iAdcx[1]>0)          //x0
    xyzVal.X = Xlf;                             
  else if(g_iAdcx[1]<1800 && g_iAdcx[1]>1000)  //x1
    xyzVal.X = Xslf;                            
  else if(g_iAdcx[1]<2200 && g_iAdcx[1]>1800)  //x2
    xyzVal.X = Xsp;                             
  else if(g_iAdcx[1]<3000 && g_iAdcx[1]>2200)  //x3
    xyzVal.X = Xsri;                            
  else if(g_iAdcx[1]<4200 && g_iAdcx[1]>3000)  //x4
    xyzVal.X = Xri; 
  
  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_RESET)
    xyzVal.Z = Zdn;    
  else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_SET)
    xyzVal.Z = Zup;
//  PrintfDebug("xyzVal.X = %d\r\nxyzVal.Y = %d\r\nxyzVal.Z = %d\r\n", xyzVal.X, xyzVal.Y, xyzVal.Z);
}

/*函数名称:获取摇杆的状态 */
/*形参:无                */
/*返回值:keystate        */
/*说明:keystate为E_RKEYState的变量，返回按键状态值*/
E_RKEYState Get_KeyState(void)
{
  E_RKEYState keystate;
  Get_xyzVal(); 
//  PrintfDebug("%d %d\r\n", xyzVal.X, xyzVal.Y);                 //摇杆状态编码
  if((xyzVal.Y == Ysp) && (xyzVal.X == Xsp) && (xyzVal.Z == Zup)) //x2 y2 z0 0
    keystate = KEYSTOP;                                        
  else if((xyzVal.Y == Yup) && (xyzVal.X == Xsp))                 //x2 y0 z0 1
    keystate = KEYUP;                                          
  else if((xyzVal.Y == Ysup) && (xyzVal.X == Xsp))                //x2 y1 z0 2
    keystate = KEYSUP;                                        
  else if((xyzVal.Y == Ydn) && (xyzVal.X == Xsp))                 //x2 y4 z0 3
    keystate = KEYDOWN;                                        
  else if((xyzVal.Y == Ysdn) && (xyzVal.X == Xsp))                //x2 y3 z0 4
    keystate = KEYSDOWN;                                       
  else if((xyzVal.Y == Ysp) && (xyzVal.X == Xlf))                 //x0 y2 z0 5
    keystate = KEYLEFT;                                        
  else if((xyzVal.Y == Ysp) && (xyzVal.X == Xslf))                //x1 y2 z0 6
    keystate = KEYSLEFT;                                       
  else if((xyzVal.Y == Ysp) && (xyzVal.X == Xri))                 //x4 y2 z0 7
    keystate = KEYRIGHT;                                      
  else if((xyzVal.Y == Ysp) && (xyzVal.X == Xsri))                //x3 y2 z0 8
    keystate = KEYSRIGHT;                                    
  else if(xyzVal.Z == Zdn)                                        //x2 y2 z1 9
    keystate = KEYPRESS;
  else if(((xyzVal.Y==Yup)&&(xyzVal.X==Xlf))||((xyzVal.Y==Yup)&&(xyzVal.X==Xslf))||((xyzVal.Y==Ysup)&&(xyzVal.X==Xlf)))                 //x01 y01 z0 10
    keystate = KEYUPLF;
  else if((xyzVal.Y == Ysup) && (xyzVal.X == Xslf))               //x1 y1 z0 11
    keystate = KEYSUPLF;
  else if(((xyzVal.Y==Yup)&&(xyzVal.X==Xri))||((xyzVal.Y==Yup)&&(xyzVal.X==Xsri))||((xyzVal.Y==Ysup)&&(xyzVal.X==Xri)))                 //x34 y01 z0 12
    keystate = KEYUPRI;
  else if((xyzVal.Y == Ysup) && (xyzVal.X == Xsri))               //x3 y1 z0 13
    keystate = KEYSUPRI;
  else if(((xyzVal.Y==Ydn)&&(xyzVal.X==Xlf))||((xyzVal.Y==Ydn)&&(xyzVal.X==Xslf))||((xyzVal.Y==Ysdn)&&(xyzVal.X==Xlf)))                 //x01 y34 z0 14
    keystate = KEYDOWNLF;
  else if((xyzVal.Y == Ysdn) && (xyzVal.X == Xslf))               //x1 y3 z0 15
    keystate = KEYSDOWNLF;
  else if(((xyzVal.Y==Ydn)&&(xyzVal.X==Xri))||((xyzVal.Y==Ydn)&&(xyzVal.X==Xsri))||((xyzVal.Y==Ysdn)&&(xyzVal.X==Xri)))                 //x34 y34 z0 16
    keystate = KEYDOWNRI;
  else if((xyzVal.Y == Ysdn) && (xyzVal.X == Xsri))               //x3 y3 z0 17
    keystate = KEYSDOWNRI;
  else
    keystate = KEYState_MAX;
//  PrintfDebug("keystate = %d\r\n", keystate);
  return keystate;
}

/*函数名称:测试摇杆功能   */
/*形参:E_RKEYState state */
/*返回值:无              */
void Test(E_RKEYState state)
{ 
  switch(state)
  {
    case KEYSTOP:PrintfDebug("No State\r\n");   
      break;
    case KEYUP:PrintfDebug("Key Up\r\n"); 
      break;
    case KEYSUP:PrintfDebug("Key Slowly Up\r\n"); 
      break;
    case KEYDOWN:PrintfDebug("Key Down\r\n"); 
      break;
    case KEYSDOWN:PrintfDebug("Key Slowly Down\r\n"); 
      break;
    case KEYLEFT:PrintfDebug("Key Left\r\n"); 
      break;
    case KEYSLEFT:PrintfDebug("Key Slowly Left\r\n"); 
      break;
    case KEYRIGHT:PrintfDebug("Key Right\r\n");
      break;
    case KEYSRIGHT:PrintfDebug("Key Slowly Right\r\n");
      break;
    case KEYPRESS:PrintfDebug("Press Key\r\n");
      break;
    
    case KEYUPLF:PrintfDebug("Key Up Left\r\n");
      break;
    case KEYSUPLF:PrintfDebug("Key Slowly Up Left\r\n");
      break;
    case KEYUPRI:PrintfDebug("Key Up Right\r\n");
      break;
    case KEYSUPRI:PrintfDebug("Key Slowly Up Right\r\n");
      break;
    case KEYDOWNLF:PrintfDebug("Key Down Left\r\n");
      break;
    case KEYSDOWNLF:PrintfDebug("Key Slowly Down Left\r\n");
      break;
    case KEYDOWNRI:PrintfDebug("Key Down Right\r\n");
      break;
    case KEYSDOWNRI:PrintfDebug("Key Slowly Down Right\r\n");
      break;
    case KEYState_MAX:PrintfDebug("State\r\n");
      break;
    default:
      break;
  }
}

