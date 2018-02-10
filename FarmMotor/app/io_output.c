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

������     �������
������     
           enable �ŷ�ʹ�� 1������0�ر�
����ֵ��   ��
********************************************************************************
*/

void MotorEnable(uint8_t enable)
{
  //�ŷ�ʹ��
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
                  void relayControl(uint8 num, uint8 val)

������     �̵�������
������     num��ţ�
        state:
        =TURN_ON                  1   //��
        =TURN_OFF                 0   //��
����ֵ��   ��
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

������     LED����
������     
num :      LED_ALARM �����ź�       

val :
LED_state=TURN_ON                  1   //��
LED_state=TURN_OFF                 0   //��
����ֵ��   ��
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



