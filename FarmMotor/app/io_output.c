/*
********************************************************************************
                                UART����ͨ��

�ļ���    ��io_output.c
�汾      ��
����Ա    ��Tank_CG
********************************************************************************
˵��������Tank��ͨ�����˿��ư�
********************************************************************************
*/


/*
********************************************************************************
                               ͷ�ļ�        
********************************************************************************
*/
#include "io_output.h"

/*
********************************************************************************
                               ��������       
********************************************************************************
*/


/*
********************************************************************************
                         void OutPutInit(void)

������ DO�����ʼ��
������  
����ֵ����
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
  
  
  //��������ͣ����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
 
 
}


/*
********************************************************************************
                 void MotorDirection(uint8_t dir)

������     ������ƣ������1����2
������     
           dir ����ʻ���� 1ǰ����0���� 2��ʱ�� 3˳ʱ��
����ֵ��   ��
********************************************************************************
*/
void DriveDirection(uint8_t dir)
{
  
 // printf("����:%d",dir);
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

������     �̵�������
������    state:
          =TURN_ON                  1   //��
          =TURN_OFF                 0   //��
����ֵ��   ��
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

������     ������ʾLED
������         
whatMsg :ָʾ��Ϣ
LED_state=TURN_ON                  1   //��
LED_state=TURN_OFF                 0   //��
����ֵ��   ��
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

