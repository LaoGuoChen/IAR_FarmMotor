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
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 
    | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  

  GPIO_ResetBits(GPIOE,GPIO_Pin_0);
  GPIO_ResetBits(GPIOE,GPIO_Pin_1);
  GPIO_ResetBits(GPIOE,GPIO_Pin_2);
  GPIO_ResetBits(GPIOE,GPIO_Pin_3);
  GPIO_ResetBits(GPIOE,GPIO_Pin_4);
  GPIO_ResetBits(GPIOE,GPIO_Pin_5);
  GPIO_ResetBits(GPIOE,GPIO_Pin_6);
  GPIO_ResetBits(GPIOE,GPIO_Pin_7);
  GPIO_ResetBits(GPIOE,GPIO_Pin_8);
  
  
}

/*
********************************************************************************
                 void MotorEnable(uint8_t enable)

描述：     电机控制
参数：     
           enable 伺服使能 1开启，0关闭
返回值：   无
********************************************************************************
*/

void MotorEnable(uint8_t enable)
{
  //伺服使能
  if(1 == enable)
  {
    GPIO_SetBits(GPIOE,GPIO_Pin_1);
    GPIO_SetBits(GPIOE,GPIO_Pin_3);
  }else
  {
    GPIO_ResetBits(GPIOE,GPIO_Pin_1);
    GPIO_ResetBits(GPIOE,GPIO_Pin_3);
  }
  

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
                  void relayControl(uint8 num, uint8 val)

描述：     继电器控制
参数：     num编号，
        state:
        =TURN_ON                  1   //开
        =TURN_OFF                 0   //关
返回值：   无
********************************************************************************
*/
void relayControl(uint8_t num, SwitchState state)
{
  
  if(num == RELAY_1)
  {
    if(state == TURN_ON)
    {
      GPIO_SetBits(GPIOE,GPIO_Pin_7);
    }else
    {
      GPIO_ResetBits(GPIOE,GPIO_Pin_7);
    }
  }if(num == RELAY_2)
  {
    if(state == TURN_ON)
    {
      GPIO_SetBits(GPIOE,GPIO_Pin_8);
    }else
    {
      GPIO_ResetBits(GPIOE,GPIO_Pin_8);
    }
  }
}

/*
********************************************************************************
                void ledControl(uint8 num, uint8 val)

描述：     LED控制
参数：     
num :      LED_ALARM 报警信号       

val :
LED_state=TURN_ON                  1   //开
LED_state=TURN_OFF                 0   //关
返回值：   无
********************************************************************************
*/
void ledControl(uint8_t num, SwitchState state)
{	
  switch(num)
  {
  case LED_ALARM:
    if(state == TURN_ON)
    {
      GPIO_SetBits(GPIOE,GPIO_Pin_6);
    }else
    {
      GPIO_ResetBits(GPIOE,GPIO_Pin_6);
    }

    break;
  }
}



