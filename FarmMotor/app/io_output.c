/*
********************************************************************************
                                UART串口通信

文件名    ：io_output.c
版本      ：
程序员    ：Tank_CG
********************************************************************************
说明：基于Tank交通机器人控制板
********************************************************************************
*/


/*
********************************************************************************
                               头文件        
********************************************************************************
*/
#include "io_output.h"

/*
********************************************************************************
                               函数申明       
********************************************************************************
*/


/*
********************************************************************************
                         void OutPutInit(void)

描述： DO输出初始化
参数：  
返回值：无
********************************************************************************
*/
void OutPutInit(void)
{
  
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOC, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  

  GPIO_ResetBits(GPIOE,GPIO_Pin_0);
  GPIO_ResetBits(GPIOE,GPIO_Pin_1);
   
  GPIO_ResetBits(GPIOE,GPIO_Pin_12);
  GPIO_ResetBits(GPIOE,GPIO_Pin_13);
  GPIO_ResetBits(GPIOE,GPIO_Pin_14);
  GPIO_ResetBits(GPIOE,GPIO_Pin_15);
  
  
  //发动机启停输入
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
 
 
}


/*
********************************************************************************
                 void MotorDirection(uint8_t dir)

描述：     电机控制，编号左1，右2
参数：     
           dir 车行驶方向 1前进，0后退 2逆时针 3顺时针
返回值：   无
********************************************************************************
*/
void DriveDirection(uint8_t dir)
{
  
 // printf("调用:%d",dir);
  switch(dir)
  {
  case 0:
    
     MOTOR_control.motor_dirL = 0;   
     MOTOR_control.motor_dirR = 1;      
    break;
  case 1:
     MOTOR_control.motor_dirL = 1;   
     MOTOR_control.motor_dirR = 0;   
    break;
  case 2:
     MOTOR_control.motor_dirL = 1;   
     MOTOR_control.motor_dirR = 1;   
  
    break;
  case 3:
    
     MOTOR_control.motor_dirL = 0;   
     MOTOR_control.motor_dirR = 0; 

    break;
  }

}
/*
********************************************************************************
                  EngineRelay(SwitchState state)

描述：     继电器控制
参数：    state:
          =TURN_ON                  1   //开
          =TURN_OFF                 0   //关
返回值：   无
********************************************************************************
*/
void EngineRelay(SwitchState state)
{ 
  
  if(state == TURN_ON)
  {
    GPIO_SetBits(GPIOE,GPIO_Pin_0);
  }else
  {
    GPIO_ResetBits(GPIOE,GPIO_Pin_0);
  }
  
}


/*
********************************************************************************
               void  RunMessage(uint8_t whatMsg,SwitchState state)

描述：     运行提示LED
参数：         
whatMsg :指示信息
LED_state=TURN_ON                  1   //开
LED_state=TURN_OFF                 0   //关
返回值：   无
********************************************************************************
*/
void  RunMessage(uint8_t whatMsg,SwitchState state)
{
  
  switch(whatMsg)
  {
  case RUN_LED:
    if(state == TURN_ON)
    {
      GPIO_SetBits(GPIOE,GPIO_Pin_12);
    }else
    {
      GPIO_ResetBits(GPIOE,GPIO_Pin_12);
    }
    break;
  case ENGINE_LED:
    if(state == TURN_ON)
    {
      GPIO_SetBits(GPIOE,GPIO_Pin_13);
    }else
    {
      GPIO_ResetBits(GPIOE,GPIO_Pin_13);
    }
    break;
  case NO_LEVEL_LED:
    if(state == TURN_ON)
    {
      GPIO_SetBits(GPIOE,GPIO_Pin_14);
    }else
    {
      GPIO_ResetBits(GPIOE,GPIO_Pin_14);
    }
    break;
  case NO_POWER_LED:
    if(state == TURN_ON)
    {
      GPIO_SetBits(GPIOE,GPIO_Pin_15);
    }else
    {
      GPIO_ResetBits(GPIOE,GPIO_Pin_15);
    }
    break;
  case STOP_LED:
    if(state == TURN_ON)
    {
      GPIO_SetBits(GPIOE,GPIO_Pin_1);
    }else
    {
      GPIO_ResetBits(GPIOE,GPIO_Pin_1);
    }
    break;
    
  }
}

