/**
******************************************************************************
                              main.c

                        开发环境：STM32F103VC
文件名：    main.c
固件版本：  3.5
程序员：    TK_CG
描述：      程序入口
******************************************************************************
*/

/*
********************************************************************************
                                   头文件
********************************************************************************
*/


#include <stdio.h>
#include "stm32f10x.h"
#include "delay.h"
#include "bsp_timer.h"
#include "app_conf.h"
#include "bsp_inputCapture.h"
#include "io_output.h"
#include "bsp_uart3.h"
#include "bsp_uart4.h"
#include "crc16_modbus.h"
#include "bsp_adc.h"
#include "bsp_can1.h"


/*
********************************************************************************
                                   printf（）打印字符到USART
********************************************************************************
*/

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch) 1
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif 

PUTCHAR_PROTOTYPE
{

  ITM_SendChar(ch);
 // USART_SendData(UART4,ch);
 // while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET){}

return ch;

}



/*
********************************************************************************
                                  数据初始化
********************************************************************************
*/

void Data_Init(void);




/*
********************************************************************************
                               全局变量      
********************************************************************************
*/
MSG_Group       MSG_Event;  

SwitchState     LED_state;
SwitchState     MOTOR_state;
SwitchState     ENGINE_relayState;
uint8_t         ENGINE_state;




StateMachine    STATE_machine;
  
UartGroup       UART3_group;

ADC_Sampling    POWER_val;

MotorCtrGroup MOTOR_control;

StateCondition  WORK_condition;

uint8_t DEBUG_err=0;
 

/*
********************************************************************************
                               局部变量     
********************************************************************************
*/

/*
********************************************************************************
                                   mian函数
********************************************************************************
*/
int main()
{ 
  Data_Init();   
  BSP_ADCInit();
  OutPutInit();
 

  Delay_Ms(300);
  UART3_Init();
  //UART4_Init();
  BSP_CAN1_Init();

  BSP_InputCatrueInit();

  Timer_Init();
 
  //测试模式
  MSG_Event.event_orderHandle = 1;
  while(1)
  { 
 
    //*****************进入状态机处理start*******************//
    switch(STATE_machine)
    {
    case busControl:
  
      STATE_machine = EventProcessing_e0(STATE_machine);
      break;
    case handleControl:
      STATE_machine = EventProcessing_e1(STATE_machine);
      break;
    case urgentStop:  //急停状态下不处理任何事件
      if( MSG_Event.event_motorAlarm || MSG_Event.event_noPower || MSG_Event.event_orderCAN ||
          MSG_Event.event_orderHandle || MSG_Event.event_orderSpeed ||MSG_Event.event_orderStop ||
          MSG_Event.event_engineOFF)
      {
     //   printf("急停状态，不处理%d %d %d %d %d %d %d\n",MSG_Event.event_motorAlarm,MSG_Event.event_noPower,
    //           MSG_Event.event_orderCAN,MSG_Event.event_orderHandle,MSG_Event.event_orderSpeed,
    //           MSG_Event.event_orderStop,MSG_Event.event_engineOFF);
        
        MSG_Event.event_motorAlarm=0;
        MSG_Event.event_noPower=0;
        MSG_Event.event_orderCAN=0;
        MSG_Event.event_orderHandle=0;
        MSG_Event.event_orderSpeed=0;
        MSG_Event.event_orderStop=0;
        MSG_Event.event_engineOFF=0;
        MSG_Event.event_orderCruise=0;
        
        MOTOR_control.cruise_leftSpeed=0;
        MOTOR_control.cruise_rightSpeed=0;
        MOTOR_control.cruise_orderSpeed=0;
        
      }
    case cruiseControl: //定速巡航
      STATE_machine = EventProcessing_e2(STATE_machine);
     //  printf("--进入定速巡航--%d\n",STATE_machine);
      break;
    default:
      break;
    }

   //*****************进入状态机处理end*******************//

    //byte0帧头，byte1帧长（除去帧头校验） byte2控制0结束1开始，byte3速度值 byte4校验
   if(1 == UART3_group.revFlag && STATE_machine != cruiseControl)
   {
     uint8_t check=0;
     for(uint8_t i=1;i<=UART3_group.revBuf[1];i++)//数据校验
     {
       check += UART3_group.revBuf[i];
     }
     
     if(check == UART3_group.revBuf[UART3_group.revBuf[1]+1] && 1 == UART3_group.revBuf[2])
     {
        //float right_v=((Vr/16.0)*MATH_PI*2.0*CAR_WHEEL_RADIUM)/60.0;
       
        MOTOR_control.cruise_orderSpeed = (double)UART3_group.revBuf[3]/100;//单转化成 米/秒
       
        MSG_Event.event_orderCruise = 1;
        
     }else
     {
       printf("--接收定速巡航数据校验失败--");
     }
     UART3_group.revFlag =0;
   }
  }

}

/*
********************************************************************************
                       void Data_Init(void)

描述：     数据初始化
参数：     无
返回值：   无
********************************************************************************
*/

void Data_Init(void)
{
  
  WORK_condition.vel_flag = 0;
  WORK_condition.angle_flag = 0;
  WORK_condition.online_flag = 0;
  WORK_condition.state = 0;
  WORK_condition.curise_start =0;
  

  POWER_val.powerVal1 =0;
  POWER_val.powerVal2 = 0;
  POWER_val.waterLevel = 0;

  MOTOR_control.leftSpeed = 0;
  MOTOR_control.rightSpeed = 0;
  MOTOR_control.can_leftSpeed=0;
  MOTOR_control.can_rightSpeed=0;
  MOTOR_control.cruise_leftSpeed=0;
  MOTOR_control.cruise_rightSpeed=0;
  MOTOR_control.cruise_orderSpeed=0;
  

  MOTOR_control.motor_dirL = 1;   
  MOTOR_control.motor_dirR = 0;  
  
  ENGINE_relayState = TURN_OFF;
  ENGINE_state = 0;
  LED_state = TURN_OFF;
  MOTOR_state = TURN_OFF;
  
  UART3_group.revFlag =0;
  
  //事件标志初始化
  MSG_Event.event_motorAlarm=0;
  MSG_Event.event_orderStop=0;
  MSG_Event.event_noPower=0;
  MSG_Event.event_orderHandle=0;
  MSG_Event.event_orderSpeed=0;
  MSG_Event.event_orderCAN=0;
  MSG_Event.event_engineOFF=0;
  MSG_Event.event_orderCruise=0;
 
  
  STATE_machine = busControl;

  
}










