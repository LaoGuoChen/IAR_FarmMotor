/**
******************************************************************************
                              main.c

                        ����������STM32F103VC
�ļ�����    main.c
�̼��汾��  3.5
����Ա��    TK_CG
������      �������
******************************************************************************
*/

/*
********************************************************************************
                                   ͷ�ļ�
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
                                   printf������ӡ�ַ���USART
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
                                  ���ݳ�ʼ��
********************************************************************************
*/

void Data_Init(void);


/*
********************************************************************************
                               ȫ�ֱ���      
********************************************************************************
*/
MSG_Group       MSG_Event;  

SwitchState     LED_state;
SwitchState     MOTOR_state;
SwitchState     ENGINE_relayState;
uint8_t         ENGINE_state;

float  ADC_powerVal[2]; 


StateMachine    STATE_machine;
  
UartGroup       UART3_group;
UartGroup       UART4_group;
ADC_Sampling    POWER_val;

MotorCtrGroup MOTOR_control;

StateCondition  WORK_condition;

uint8_t DEBUG_err=0;
 

/*
********************************************************************************
                               �ֲ�����     
********************************************************************************
*/

/*
********************************************************************************
                                   mian����
********************************************************************************
*/
int main()
{ 
  Data_Init();   
  BSP_ADCInit();
  OutPutInit();
  
  Delay_Ms(3000);//�ȴ���ѹ�ȶ���//��ʱ���ȴ���������ʼ�����
  //�ϵ�����������Դ��ѹ���������ϵ��Ž��빤��ģʽ���ŷ�ʹ�ܡ�
  while(POWER_val.powerVal2 < PWOER_DEFAULT)
  {  
     ADC_Cmd(ADC1, ENABLE); 
     ADC_SoftwareStartConvCmd(ADC1, ENABLE); 
     Delay_Ms(20);//�ϵ�ʱ20ms��һ�� ��ADC_DMA_LEN�μ���ƽ��ֵ
  }


 
  UART3_Init();
  UART4_Init();
  BSP_CAN1_Init();

  BSP_InputCatrueInit();

  Timer_Init();
 
  while(1)
  { 
    
    //*****************����״̬������start*******************//
    switch(STATE_machine)
    {
    case busControl:
  
      STATE_machine = EventProcessing_e0(STATE_machine);
      break;
    case handleControl:
      STATE_machine = EventProcessing_e1(STATE_machine);
      break;
    case urgentStop:  //��ͣ״̬�²������κ��¼�
      if( MSG_Event.event_motorAlarm || MSG_Event.event_noPower || MSG_Event.event_orderCAN ||
          MSG_Event.event_orderHandle || MSG_Event.event_orderSpeed ||MSG_Event.event_orderStop ||
          MSG_Event.event_engineOFF)
      {
     //   printf("��ͣ״̬��������%d %d %d %d %d %d %d\n",MSG_Event.event_motorAlarm,MSG_Event.event_noPower,
    //           MSG_Event.event_orderCAN,MSG_Event.event_orderHandle,MSG_Event.event_orderSpeed,
    //           MSG_Event.event_orderStop,MSG_Event.event_engineOFF);
        
        MSG_Event.event_motorAlarm=0;
        MSG_Event.event_noPower=0;
        MSG_Event.event_orderCAN=0;
        MSG_Event.event_orderHandle=0;
        MSG_Event.event_orderSpeed=0;
        MSG_Event.event_orderStop=0;
        MSG_Event.event_engineOFF=0;
        
      }
      
    default:
      break;
    }
   //*****************����״̬������end*******************//
    
 


  }

}

/*
********************************************************************************
                       void Data_Init(void)

������     ���ݳ�ʼ��
������     ��
����ֵ��   ��
********************************************************************************
*/

void Data_Init(void)
{
  
  WORK_condition.vel_flag = 0;
  WORK_condition.angle_flag = 0;
  WORK_condition.online_flag = 0;
  WORK_condition.state = 0;
  
  ADC_powerVal[0] = 0;
  ADC_powerVal[1] = 0;

  MOTOR_control.leftSpeed = 0;
  MOTOR_control.rightSpeed = 0;
  MOTOR_control.can_leftSpeed=0;
  MOTOR_control.can_rightSpeed=0;

  MOTOR_control.motor_dirL = 1;   
  MOTOR_control.motor_dirR = 0;  
  
  ENGINE_relayState = TURN_OFF;
  ENGINE_state = 0;
  LED_state = TURN_OFF;
  MOTOR_state = TURN_OFF;
  
  //�¼���־��ʼ��
  MSG_Event.event_motorAlarm=0;
  MSG_Event.event_orderStop=0;
  MSG_Event.event_noPower=0;
  MSG_Event.event_orderHandle=0;
  MSG_Event.event_orderSpeed=0;
  MSG_Event.event_orderCAN=0;
  MSG_Event.event_engineOFF=0;
 
  
  STATE_machine = busControl;

  
}










