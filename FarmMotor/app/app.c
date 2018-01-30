/*
********************************************************************************
                                �û�����
�ļ�����app.c
�汾��
����Ա��Tank_CG
********************************************************************************
*/

/*
********************************************************************************
                               ͷ�ļ�       
********************************************************************************
*/
#include <math.h>
#include <stdio.h>
#include "app_conf.h"

#include "io_output.h"
#include "bsp_can1.h"
#include "delay.h"
#include "bsp_uart1.h"
#include "bsp_uart4.h"
#include "crc16_modbus.h"


/*
********************************************************************************
                               ȫ�ֱ���      
********************************************************************************
*/
extern uint8_t HeartTime;
/*
********************************************************************************
                               �ֲ�����      
********************************************************************************
*/



//���������ݵ�ַ
static uint8_t   sendData1[11] = {0x00,0x10,0x44,0x20,0x00,0x01,0x02,0x00,0x00,0x00,0x00};
static uint8_t   sendData2[13] = {0x00,0x10,0x44,0x20,0x00,0x02,0x04,0x00,0x00,0xFF,0xFF,0x00,0x00};
static uint8_t   sendData3[8] =  {0x00,0x03,0x44,0x71,0x00,0x01,0x00,0x00};//��ȡ����ֵ��
static uint8_t   sendData4[8] =  {0x00,0x03,0x46,0x6E,0x00,0x04,0x00,0x00};//��ȡ�������ݣ�
static uint16_t  check_val=0;



/*
********************************************************************************
                  void CAN_DataCommunication (void)

������CAN��ʱ�������ݣ�ʱ���ǻ��ڶ�ʱ��5�ж�
������
     
����ֵ����������״̬
********************************************************************************
*/
void CAN_DataCommunication (void)
{
   static CanTxMsg TxMessage;
   //����״̬��
   if(HEART_TIME1 == HeartTime || 2*HEART_TIME1 == HeartTime || HEART_TIME2 - 3 == HeartTime)
   {
     TxMessage.StdId = USER_STDID_ADR4;             
     TxMessage.IDE=CAN_ID_STD;           //��׼��ʶ��
     TxMessage.RTR=CAN_RTR_DATA;
     TxMessage.DLC=1;

     TxMessage.Data[0]=STATE_machine;
 
     CAN_Transmit(CAN1,&TxMessage);

   }
   //�����������̵���
   if(HEART_TIME2 - 2 == HeartTime)
   {  
     TxMessage.StdId = USER_STDID_ADR5;             
     TxMessage.IDE=CAN_ID_STD;           //��׼��ʶ��
     TxMessage.RTR=CAN_RTR_DATA;
     TxMessage.DLC=1;

     TxMessage.Data[0]=ENGINE_state;

     CAN_Transmit(CAN1,&TxMessage);

   }
   //����ˮλ
   if(HEART_TIME2 - 1 == HeartTime)
   {   
     TxMessage.StdId = USER_STDID_ADR7;             
     TxMessage.IDE=CAN_ID_STD;           //��׼��ʶ��
     TxMessage.RTR=CAN_RTR_DATA;
     TxMessage.DLC=1;
     TxMessage.Data[0]=(uint8_t)(POWER_val.waterLevel);

     CAN_Transmit(CAN1,&TxMessage);
   }
   //������ص���
   if(HEART_TIME2 == HeartTime)
   {
     static  uint16_t power_val =0;
     power_val = (uint16_t)(POWER_val.powerVal2*PWOER_MSG);
     
     TxMessage.StdId = USER_STDID_ADR8;             
     TxMessage.IDE=CAN_ID_STD;           //��׼��ʶ��
     TxMessage.RTR=CAN_RTR_DATA;
     TxMessage.DLC=2;
     
     TxMessage.Data[0]=power_val & 0xff;
     TxMessage.Data[1]=(power_val>>8) & 0xff;
  
     CAN_Transmit(CAN1,&TxMessage);
     
          
    
   }
}
/*
********************************************************************************
                void DataCommunication(void)

����������������߶�ʱ�������ݣ�ʱ���ǻ��ڶ�ʱ��5�ж�
������
     
����ֵ����������״̬
********************************************************************************
*/

void UART_DataCommunication(void)
{
    static uint8_t timeFilm = 0;

    switch(timeFilm)
    {
    case 0:
      timeFilm = 1;
 
      if(0 == MOTOR_control.rightSpeed)
      {
        MOTOR_control.motor_dir2 = 1;
      }
      if(1 == MOTOR_control.motor_dir2)
      {
        
        sendData1[7]= (uint8_t)(MOTOR_control.rightSpeed>>8);
        sendData1[8]= (uint8_t)(MOTOR_control.rightSpeed);
        check_val= CRC16_MODBUS(sendData1,9);
        sendData1[9] = (uint8_t)check_val;
        sendData1[10] = (uint8_t)(check_val>>8);
        UART4_sendData(11,sendData1);

        UART4_group.revFlag = 0;
        
        
      }else if(0 == MOTOR_control.motor_dir2)
      {
        sendData2[7]= (uint8_t)((~MOTOR_control.rightSpeed+1)>>8);
        sendData2[8]= (uint8_t)(~MOTOR_control.rightSpeed+1);
        
        check_val= CRC16_MODBUS(sendData2,11);
        
        sendData2[11] = (uint8_t)check_val;
        sendData2[12] = (uint8_t)(check_val>>8);
        UART4_sendData(13,sendData2);

        UART4_group.revFlag = 0;
      }
      
      if(0 == MOTOR_control.leftSpeed)
      {
        MOTOR_control.motor_dir1 = 1;
      }
      if(1 == MOTOR_control.motor_dir1)
      {
        
        sendData1[7]= (uint8_t)(MOTOR_control.leftSpeed>>8);
        sendData1[8]= (uint8_t)(MOTOR_control.leftSpeed);
        check_val= CRC16_MODBUS(sendData1,9);
        sendData1[9] = (uint8_t)check_val;
        sendData1[10] = (uint8_t)(check_val>>8);
        UART1_sendData(11,sendData1);
        UART1_group.revFlag = 0;
        
        
      }else if(0 == MOTOR_control.motor_dir1)
      {
        
        sendData2[7]= (uint8_t)((~MOTOR_control.leftSpeed+1)>>8);
        sendData2[8]= (uint8_t)(~MOTOR_control.leftSpeed+1);
        
        check_val= CRC16_MODBUS(sendData2,11);
        
        sendData2[11] = (uint8_t)check_val;
        sendData2[12] = (uint8_t)(check_val>>8);      
        UART1_sendData(13,sendData2);
        UART1_group.revFlag = 0;

        
      }
   //   printf("�����ٶȣ�%d %d\n",MOTOR_control.leftSpeed,MOTOR_control.rightSpeed);

      break;
      
    case 1:
    
        timeFilm = 0;
        check_val= CRC16_MODBUS(sendData3,6);
        
        sendData3[6] = (uint8_t)check_val;
        sendData3[7] = (uint8_t)(check_val>>8);  
        
        UART1_sendData(8,sendData3);
        UART1_group.revFlag = 0;
        UART4_sendData(8,sendData3);   

        UART4_group.revFlag = 0;
   
  
      break;
    }
    

}
/*
********************************************************************************
                StateMachine EventProcessing_e1(void)

���������߿���״̬���¼�����
������
     
����ֵ����������״̬
********************************************************************************
*/
StateMachine EventProcessing_e0(StateMachine state)
{
  StateMachine next_state;
  next_state= state;
  

  //���뼱ͣ״̬
  if(MSG_Event.event_motorAlarm || MSG_Event.event_orderStop || MSG_Event.event_noPower)
  {
    
 //  printf("���߿��Ƽ�ͣ������%d ָ��%d ������%d\n",MSG_Event.event_motorAlarm ,MSG_Event.event_orderStop,MSG_Event.event_noPower);
    
    MOTOR_control.leftSpeed  = MOTOR_control.rightSpeed = 0;
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3, DISABLE);
    relayControl(RELAY_1, ENGINE_state=TURN_OFF);  
    
    //***********����������뼱ͣstart*************//
    TIM_Cmd(TIM5, DISABLE);  //�رն�ʱ����
    
    check_val= CRC16_MODBUS(sendData4,6);
    
    sendData4[6] = (uint8_t)check_val;
    sendData4[7] = (uint8_t)(check_val>>8); 
    
    if(1 == MSG_Event.event_motorAlarm) //����ǵ���������ȡ������Ϣ
    {
         
        
        UART1_sendData(8,sendData4);
        UART1_group.revFlag = 0;
     // printf("��������\n");
      
    }
    if(2 == MSG_Event.event_motorAlarm)
    {
      UART4_sendData(8,sendData4);
      UART4_group.revFlag = 0;
    //  printf("�ҵ������\n");
    }
    
    TIM_Cmd(TIM5, ENABLE);  //������ʱ��
    //***********����������뼱ͣend*************//
    
    Delay_Ms(500);  //��ʱ�ȴ����������0
    MotorEnable(MOTOR_state = TURN_OFF);
    
    
    MSG_Event.event_orderStop =0; //�¼���־���
    MSG_Event.event_motorAlarm=0;
    MSG_Event.event_noPower=0;
    next_state = urgentStop;  //��һ��״̬
 
  }else if(MSG_Event.event_orderHandle)//����ң��������
  {
    
    
    MOTOR_control.leftSpeed  = MOTOR_control.rightSpeed = 0;
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3, ENABLE);
    
    MSG_Event.event_orderHandle=0;
    next_state = handleControl;    //��һ��״̬
    
  //  printf("���߿��ƽ���ң��������%d\n",next_state);
     
  }else if(MSG_Event.event_orderCAN) //�������߿���
  {
    MOTOR_control.leftSpeed  = MOTOR_control.rightSpeed= 0;
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3, DISABLE);
    
    MSG_Event.event_orderCAN=0;
    next_state = busControl;       //��һ��״̬
    
  //  printf("���߿��ƽ������߿���%d\n",next_state);
  }
  else if(MSG_Event.event_engineOFF)  //����������ָ��
  {
    
    if(1 == MSG_Event.event_engineOFF) //�ط�����
    {
      relayControl(RELAY_1, ENGINE_state=TURN_OFF);
      
    }else if(2 == MSG_Event.event_engineOFF)              //���������̵���
    {
      relayControl(RELAY_1, ENGINE_state=TURN_ON);

    }
    
    MSG_Event.event_engineOFF = 0;
    
  }else if(MSG_Event.event_orderSpeed)
  {
      TIM_Cmd(TIM5, DISABLE);  //�رն�ʱ������������
      
   //   printf("���߿����ٶ�ָ�� ����%d �ҵ��%d \n",MOTOR_control.can_leftSpeed,MOTOR_control.can_rightSpeed);
       
      if(MOTOR_control.can_leftSpeed >=0)
      {
        MOTOR_control.leftSpeed = MOTOR_control.can_leftSpeed*SPEED_RATE;
      }else
      {
        MOTOR_control.leftSpeed = (unsigned)-MOTOR_control.can_leftSpeed*SPEED_RATE;
      }
      
      if(MOTOR_control.can_rightSpeed >= 0)
      {
        MOTOR_control.rightSpeed = MOTOR_control.can_rightSpeed*SPEED_RATE; 
      }
      else
      {
        MOTOR_control.rightSpeed = (unsigned)-MOTOR_control.can_rightSpeed*SPEED_RATE;
      }
       
      //ǰ��
     if(MOTOR_control.can_leftSpeed < 0 && MOTOR_control.can_rightSpeed > 0)
     {
       MOTOR_control.motor_dir1 = 0;   
       MOTOR_control.motor_dir2 = 1;  
   
   //    printf("��ǰ��%d %d\n",MOTOR_control.can_leftSpeed,MOTOR_control.can_rightSpeed);
 
     }else if(MOTOR_control.can_leftSpeed > 0 && MOTOR_control.can_rightSpeed <0)
     { //����
       MOTOR_control.motor_dir1 = 1;   
       MOTOR_control.motor_dir2 = 0; 
      
   //    printf("������%d %d\n",MOTOR_control.can_leftSpeed,MOTOR_control.can_rightSpeed);
       
     }else if(MOTOR_control.can_leftSpeed > 0 && MOTOR_control.can_rightSpeed >0)
     {//��ʱ��
       MOTOR_control.motor_dir1 = 1;
       MOTOR_control.motor_dir2 = 1;
   //    printf("����ʱ��%d %d\n",MOTOR_control.can_leftSpeed,MOTOR_control.can_rightSpeed);
       
     }else if(MOTOR_control.can_leftSpeed < 0 && MOTOR_control.can_rightSpeed <0)
     {//˳ʱ��
       MOTOR_control.motor_dir1 = 0;
       MOTOR_control.motor_dir2 = 0;

  //     printf("��˳ʱ�� %d %d\n",MOTOR_control.can_leftSpeed,MOTOR_control.can_rightSpeed);
     }
     
  //   printf("ʵ���ٶ�%d %d\n",MOTOR_control.leftSpeed,MOTOR_control.rightSpeed);
     //�ٶ�����
     if(MOTOR_control.leftSpeed > SPEED_MAX){MOTOR_control.leftSpeed = SPEED_MAX;}
     if(MOTOR_control.rightSpeed > SPEED_MAX){MOTOR_control.rightSpeed = SPEED_MAX;}
     if(MOTOR_control.leftSpeed < 0){MOTOR_control.leftSpeed = 0;}
     if(MOTOR_control.rightSpeed < 0){MOTOR_control.rightSpeed = 0;}
     
     TIM_Cmd(TIM5, ENABLE);  //����ʱ��
     MSG_Event.event_orderSpeed=0;
  }
  
  return next_state;
}

/*
********************************************************************************
                 StateMachine EventProcessing_e1void)

������ң����ģʽ���¼�����
������
     
����ֵ����������״̬
********************************************************************************
*/
StateMachine EventProcessing_e1(StateMachine state)
{

  StateMachine next_state;
  next_state= state;
  
  //���뼱ͣ״̬
  if(MSG_Event.event_motorAlarm || MSG_Event.event_orderStop || MSG_Event.event_noPower)
  {
    
    MOTOR_control.leftSpeed  = MOTOR_control.rightSpeed = 0;
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3, DISABLE);
    relayControl(RELAY_1, ENGINE_state=TURN_OFF);
    
 //   printf("ң������ͣ������%d ָ��%d ������%d\n",MSG_Event.event_motorAlarm ,MSG_Event.event_orderStop,MSG_Event.event_noPower);
      //***********����������뼱ͣstart*************//
    TIM_Cmd(TIM5, DISABLE);  //�رն�ʱ����

    check_val= CRC16_MODBUS(sendData4,6);
    
    sendData4[6] = (uint8_t)check_val;
    sendData4[7] = (uint8_t)(check_val>>8); 
    if(1 == MSG_Event.event_motorAlarm) //����ǵ���������ȡ������Ϣ
    {
      UART1_sendData(8,sendData4);
      UART1_group.revFlag = 0;
    //  printf("��������\n");
      
    }
    if(2 == MSG_Event.event_motorAlarm)
    {
      UART4_sendData(8,sendData4);
      UART4_group.revFlag = 0;
    //  printf("�ҵ������\n");
    }
    TIM_Cmd(TIM5, ENABLE);  //����ʱ��
    //***********����������뼱ͣend*************//
    
    Delay_Ms(500);
    MotorEnable(MOTOR_state = TURN_OFF);

    MSG_Event.event_orderStop =0; //�¼���־���
    MSG_Event.event_motorAlarm=0;
    MSG_Event.event_noPower=0;
    next_state = urgentStop;  //��һ��״̬
   
 
  }else if(MSG_Event.event_orderHandle)//�����ֶ�����
  {
   
    MOTOR_control.leftSpeed  = MOTOR_control.rightSpeed = 0;
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3, ENABLE);
    
    MSG_Event.event_orderHandle=0;
    next_state = handleControl;    //��һ��״̬
    
    WORK_condition.state = 0;   //ң�����л�����ͨģʽ
 //   printf("ң�������ƽ���ң��������%d\n",next_state);
     
  }else if(MSG_Event.event_orderCAN) //�������߿���
  {
    MOTOR_control.leftSpeed  = MOTOR_control.rightSpeed= 0;
    MSG_Event.event_orderCAN=0;
    next_state = busControl;       //��һ��״̬
 //   printf("ң�����������߿���%d\n",next_state);
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3, DISABLE);
  }
  else if(MSG_Event.event_engineOFF)  //����������ָ��
  {
      
      if(1 == MSG_Event.event_engineOFF) //�ط�����
      {
        relayControl(RELAY_1, ENGINE_state=TURN_OFF);
        
      }else if(2 == MSG_Event.event_engineOFF)              //���������̵���
      {
        relayControl(RELAY_1, ENGINE_state=TURN_ON);
      }
      
      MSG_Event.event_engineOFF = 0;
      
  }else if(MSG_Event.event_orderSpeed)//ң����ģʽ�·���������Ч
  {

     MSG_Event.event_orderSpeed=0;
  //   printf("ң���������ٶ�ָ����Ч\n");
  }
  return next_state;
}

/*
********************************************************************************
                        void  SetCarSpeed (float angle,uint8_t dir)

���������ó��ٶ�
������angle ת�� 0-90
      dir   ���� 0 ֱ�� 1 ��ת��2��ת,3˳ʱ�룬4��ʱ��
     vel    �ٶ�
����ֵ����
********************************************************************************
*/

void SetCarSpeed(double angle,uint8_t dir,uint32_t vel)
{
  
  double left_vel,right_vel,angle_t;
  //ת��Ƕ�����
  angle_t = atan((double)LENGTH_B*2/LENGTH_L);
  
  if(angle > angle_t){angle = angle_t;}
  //����ת��Ƕ������ٶ�
  vel = (uint32_t)(vel * cos(angle));
  
  right_vel = 0;
  left_vel = 0;
 
  if(1 == dir)
  {       
      left_vel = vel*(1 - ((double)LENGTH_L/(2*LENGTH_B)) * tan(angle));
      right_vel = vel*(1 + ((double)LENGTH_L/(2*LENGTH_B)) * tan(angle)); 
      
      
  }else if(2 == dir){
     
      left_vel = vel*(1 + ((double)LENGTH_L/(2*LENGTH_B)) * tan(angle));
      right_vel = vel*(1 - ((double)LENGTH_L/(2*LENGTH_B)) * tan(angle)); 
    
  }else 
  {

     right_vel = vel;
     left_vel = vel;
  }
  
   MOTOR_control.leftSpeed = (uint32_t)left_vel;
   MOTOR_control.rightSpeed = (uint32_t)right_vel;
   //���ֵ����  
   if(MOTOR_control.leftSpeed > SPEED_MAX)
   {
     MOTOR_control.leftSpeed = SPEED_MAX;
   }
   if(MOTOR_control.rightSpeed > SPEED_MAX)
   {
     MOTOR_control.rightSpeed = SPEED_MAX;
   }
   
   if(MOTOR_control.leftSpeed < 0)
   {
     MOTOR_control.leftSpeed = 0;
   }
   if(MOTOR_control.rightSpeed < 0)
   {
     MOTOR_control.rightSpeed = 0;
   }
   
  // printf(" 2�ٶ�%d ",motor_vel);
}

/*
********************************************************************************
                        void  StateTransformation()

������       ״̬ת��
������     
����ֵ��    ��
********************************************************************************
*/
void  StateTransformation(void)
{
 
  if(0 == WORK_condition.state)
  {
    if(0 == WORK_condition.vel_flag && 0 == WORK_condition.angle_flag)
    {
      WORK_condition.state = 1;
    }
  }else
  { 
      WORK_condition.state = 0;  
  }
  // printf("״̬��%d ",WORK_condition.state);
}
