
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
#include "bsp_uart3.h"
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
/*
********************************************************************************
                               ��������     
********************************************************************************
*/
void setCruiseSpeed(double angle,uint8_t dir);
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
   
    //�����ٶ�ָ��
    SendSpeedValue();
    
  
   if(HEART_TIME1 == HeartTime || 2*HEART_TIME1 == HeartTime || HEART_TIME2 - 3 == HeartTime)
   {
      //����״̬��
     TxMessage.StdId = USER_STDID_ADR4;          
     TxMessage.IDE=CAN_ID_STD;           //��׼��ʶ��
     TxMessage.RTR=CAN_RTR_DATA;
     TxMessage.DLC=1;

     TxMessage.Data[0]=STATE_machine;
 
     CAN_Transmit(CAN1,&TxMessage);
     
     Delay_Ms(1);
     //������Ϣ���
     TxMessage.StdId = USER_STDID_ADR10;             
     TxMessage.Data[0]=DEBUG_err;
 
     CAN_Transmit(CAN1,&TxMessage);
     
  

   }
  
   if(HEART_TIME2 - 2 == HeartTime)
   { 
      //�����������̵���
     TxMessage.StdId = USER_STDID_ADR5;             
     TxMessage.IDE=CAN_ID_STD;           //��׼��ʶ��
     TxMessage.RTR=CAN_RTR_DATA;
     TxMessage.DLC=1;

     TxMessage.Data[0]=ENGINE_relayState;

     CAN_Transmit(CAN1,&TxMessage);
     //����������״̬
     Delay_Ms(1);
     TxMessage.StdId = USER_STDID_ADR11;  
     TxMessage.Data[0] = ENGINE_state;
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
                void SendSpeedValue(void)

����������������߶�ʱ�������ݣ�ʱ���ǻ��ڶ�ʱ��5�ж�
������

����ֵ����������״̬
********************************************************************************
*/

void SendSpeedValue(void)
{
  
  static CanTxMsg TxMessage;
  static short int can_leftSpeed,can_rightSpeed; 
  
 
  if(STATE_machine == handleControl)
  {
    //��ң��������ģʽ�·��ٶ�ָ��
    if(0 == MOTOR_control.rightSpeed)
    {
      MOTOR_control.motor_dirR = 1;
    }
    if(1 == MOTOR_control.motor_dirR)
    {
      
      can_rightSpeed = MOTOR_control.rightSpeed/SPEED_RATE;
      
      
    }else if(0 == MOTOR_control.motor_dirR)
    {
      
      can_rightSpeed = ~(MOTOR_control.rightSpeed/SPEED_RATE)+1;
    }
    
    if(0 == MOTOR_control.leftSpeed)
    {
      MOTOR_control.motor_dirL = 1;
    }
    if(1 == MOTOR_control.motor_dirL)
    {
      
      can_leftSpeed = MOTOR_control.leftSpeed/SPEED_RATE;
      
    }else if(0 == MOTOR_control.motor_dirL)
    {
      
      can_leftSpeed = ~(MOTOR_control.leftSpeed/SPEED_RATE)+1;
    }
    
    TxMessage.StdId = USER_STDID_ADR3;             
    TxMessage.IDE=CAN_ID_STD;           //��׼��ʶ��
    TxMessage.RTR=CAN_RTR_DATA;
    TxMessage.DLC=4;
    
    TxMessage.Data[0]=(uint8_t)can_leftSpeed;
    TxMessage.Data[1]=(uint8_t)(can_leftSpeed>>8);
    TxMessage.Data[2]=(uint8_t)can_rightSpeed;
    TxMessage.Data[3]=(uint8_t)(can_rightSpeed>>8);
    
    CAN_Transmit(CAN1,&TxMessage);
    //  printf("�����ٶȣ�%d %d\n",MOTOR_control.leftSpeed,MOTOR_control.rightSpeed);
    
  }else if(STATE_machine == cruiseControl && 1 == WORK_condition.curise_start)
  {
    
    //����Ѳ�����ٶ�ָ��
    TxMessage.StdId = USER_STDID_ADR3;             
    TxMessage.IDE=CAN_ID_STD;           //��׼��ʶ��
    TxMessage.RTR=CAN_RTR_DATA;
    TxMessage.DLC=4;
    
    TxMessage.Data[0]=(uint8_t) MOTOR_control.cruise_leftSpeed;
    TxMessage.Data[1]=(uint8_t)(MOTOR_control.cruise_leftSpeed>>8);
    TxMessage.Data[2]=(uint8_t) MOTOR_control.cruise_rightSpeed;
    TxMessage.Data[3]=(uint8_t)(MOTOR_control.cruise_rightSpeed>>8);
    
     printf("�����ٶȣ�%d %d\n",MOTOR_control.cruise_leftSpeed,MOTOR_control.cruise_rightSpeed);
    
    CAN_Transmit(CAN1,&TxMessage);
  }
  
  
  
}

/*
********************************************************************************
                void CruiseControlAsk(void)

���������߿���״̬���¼�����
������
     
����ֵ����������״̬
********************************************************************************
*/
void CruiseControlAsk(void)
{
  uint8_t data[8];
  data[0]=0xC3;
  data[1]=0x50;
  data[2]=0x00;   
  data[3]=0x55;
  data[4]=3;
  data[5]=STATE_machine;
  data[6]=(uint8_t)(MOTOR_control.cruise_orderSpeed*100);

  data[7]=0;      
  for(uint8_t i=4;i<7;i++)
  {
    data[7]+=data[i];
  }
  
  UART3_sendData(8,data);
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
    EngineRelay(ENGINE_relayState=TURN_OFF);  
    
    //***********����������뼱ͣstart*************//
  //  TIM_Cmd(TIM5, DISABLE);  //�رն�ʱ����

    
    if(1 == MSG_Event.event_motorAlarm) //����ǵ���������ȡ������Ϣ
    {
         DEBUG_err |= 0x04;
  //   printf("��������\n");
         
    }
    if(2 == MSG_Event.event_motorAlarm)
    {
    
      DEBUG_err |= 0x08;
    }
    
    if(MSG_Event.event_noPower)
    {
      DEBUG_err |= 0x10;
    }
    
    if(MSG_Event.event_orderStop)
    {
      DEBUG_err |= 0x20;
    }
 //   TIM_Cmd(TIM5, ENABLE);  //������ʱ��
    //***********����������뼱ͣend*************//
    
    Delay_Ms(500);  //��ʱ�ȴ����������0
  
    MSG_Event.event_orderStop =0; //�¼���־���
    MSG_Event.event_motorAlarm=0;
    MSG_Event.event_noPower=0;
    next_state = urgentStop;  //��һ��״̬
 //   printf("״̬��%d\n",next_state);
 
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
  }else if(MSG_Event.event_orderCruise)//���붨��Ѳ��
  {
    
    MOTOR_control.leftSpeed  = MOTOR_control.rightSpeed = 0;
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3, ENABLE);
    MSG_Event.event_orderCruise=0;
    next_state = cruiseControl;   
    WORK_condition.state = 0;   //ң�����л�����ͨģʽ
    
  }else if(MSG_Event.event_engineOFF)  //����������ָ��
  {
    
    if(1 == MSG_Event.event_engineOFF) //�ط�����
    {
      EngineRelay(ENGINE_relayState=TURN_OFF);
      
    }else if(2 == MSG_Event.event_engineOFF)              //���������̵���
    {
      EngineRelay(ENGINE_relayState=TURN_ON);

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
       MOTOR_control.motor_dirL = 0;   
       MOTOR_control.motor_dirR = 1;  
   
   //    printf("��ǰ��%d %d\n",MOTOR_control.can_leftSpeed,MOTOR_control.can_rightSpeed);
 
     }else if(MOTOR_control.can_leftSpeed > 0 && MOTOR_control.can_rightSpeed <0)
     { //����
       MOTOR_control.motor_dirL = 1;   
       MOTOR_control.motor_dirR = 0; 
      
   //    printf("������%d %d\n",MOTOR_control.can_leftSpeed,MOTOR_control.can_rightSpeed);
       
     }else if(MOTOR_control.can_leftSpeed > 0 && MOTOR_control.can_rightSpeed >0)
     {//��ʱ��
       MOTOR_control.motor_dirL = 1;
       MOTOR_control.motor_dirR = 1;
   //    printf("����ʱ��%d %d\n",MOTOR_control.can_leftSpeed,MOTOR_control.can_rightSpeed);
       
     }else if(MOTOR_control.can_leftSpeed < 0 && MOTOR_control.can_rightSpeed <0)
     {//˳ʱ��
       MOTOR_control.motor_dirL = 0;
       MOTOR_control.motor_dirR = 0;

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
    
    EngineRelay(ENGINE_relayState=TURN_OFF);
    
 //   printf("ң������ͣ������%d ָ��%d ������%d\n",MSG_Event.event_motorAlarm ,MSG_Event.event_orderStop,MSG_Event.event_noPower);
      //***********����������뼱ͣstart*************//

    if(1 == MSG_Event.event_motorAlarm) //����ǵ���������ȡ������Ϣ
    {
 
      DEBUG_err |= 0x04;
      
    }
    if(2 == MSG_Event.event_motorAlarm)
    {
      DEBUG_err |= 0x08;
      
    }
    //***********����������뼱ͣend*************//
    
    
    if(MSG_Event.event_noPower)
    {
      DEBUG_err |= 0x10;
    }
    
    if(MSG_Event.event_orderStop)
    {
      DEBUG_err |= 0x20;
    }
    
    
    Delay_Ms(500);


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
    
  }else if(MSG_Event.event_orderCruise)//���붨��Ѳ��
  {
    
    MOTOR_control.leftSpeed  = MOTOR_control.rightSpeed = 0;
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3, ENABLE);
    MSG_Event.event_orderCruise=0;
    next_state = cruiseControl;   
    WORK_condition.state = 0;   //ң�����л�����ͨģʽ
    
//    printf("--���붨��Ѳ��--%d",next_state);
    
  }
  else if(MSG_Event.event_engineOFF)  //����������ָ��
  {
      
      if(1 == MSG_Event.event_engineOFF) //�ط�����
      {
        EngineRelay(ENGINE_relayState=TURN_OFF);
        
      }else if(2 == MSG_Event.event_engineOFF)              //���������̵���
      {
        EngineRelay(ENGINE_relayState=TURN_ON);
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
                 StateMachine EventProcessing_e1void)

������ң��������Ѳ��ģʽ���¼�����
������
     
����ֵ����������״̬
********************************************************************************
*/
StateMachine EventProcessing_e2(StateMachine state)
{

  StateMachine next_state;
  next_state= state;
  
  //���뼱ͣ״̬
  if(MSG_Event.event_motorAlarm || MSG_Event.event_orderStop || MSG_Event.event_noPower)
  {
    
    MOTOR_control.leftSpeed  = MOTOR_control.rightSpeed = 0;
    
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3, DISABLE);
    
    EngineRelay(ENGINE_relayState=TURN_OFF);
    
 //   printf("ң������ͣ������%d ָ��%d ������%d\n",MSG_Event.event_motorAlarm ,MSG_Event.event_orderStop,MSG_Event.event_noPower);
      //***********����������뼱ͣstart*************//

    if(1 == MSG_Event.event_motorAlarm) //����ǵ���������ȡ������Ϣ
    {
 
      DEBUG_err |= 0x04;
      
    }
    if(2 == MSG_Event.event_motorAlarm)
    {
      DEBUG_err |= 0x08;
      
    }
    //***********����������뼱ͣend*************//
    
    
    if(MSG_Event.event_noPower)
    {
      DEBUG_err |= 0x10;
    }
    
    if(MSG_Event.event_orderStop)
    {
      DEBUG_err |= 0x20;
    }
    
    
    Delay_Ms(500);


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
    
    
    MOTOR_control.cruise_leftSpeed=0;
    MOTOR_control.cruise_rightSpeed=0;
    MOTOR_control.cruise_orderSpeed=0;
 //   printf("ң�������ƽ���ң��������%d\n",next_state);
     
  }else if(MSG_Event.event_orderCAN) //�������߿���
  {
    MOTOR_control.leftSpeed  = MOTOR_control.rightSpeed= 0;
    MSG_Event.event_orderCAN=0;
    next_state = busControl;       //��һ��״̬
    
    MOTOR_control.cruise_leftSpeed=0;
    MOTOR_control.cruise_rightSpeed=0;
    MOTOR_control.cruise_orderSpeed=0;
    
 //   printf("ң�����������߿���%d\n",next_state);
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3, DISABLE);
    
  }else if(MSG_Event.event_orderCruise)//���붨��Ѳ��
  {
    
    MOTOR_control.leftSpeed  = MOTOR_control.rightSpeed = 0;
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3, ENABLE);
    MSG_Event.event_orderCruise=0;
    next_state = cruiseControl;   
    WORK_condition.state = 0;   //ң�����л�����ͨģʽ
    
  }
  else if(MSG_Event.event_engineOFF)  //����������ָ��
  {
      
      if(1 == MSG_Event.event_engineOFF) //�ط�����
      {
        EngineRelay(ENGINE_relayState=TURN_OFF);
        
      }else if(2 == MSG_Event.event_engineOFF)              //���������̵���
      {
        EngineRelay(ENGINE_relayState=TURN_ON);
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
   
 //  printf("�ٶ���=%d ��=%d \n",MOTOR_control.leftSpeed,MOTOR_control.rightSpeed);
   
   if(cruiseControl == STATE_machine ){
     setCruiseSpeed(angle,dir);
   }
}

void setCruiseSpeed(double angle,uint8_t dir)
{
  double left_vel,right_vel,angle_t;
  //����Ѳ�������ٶ�
  double vel = (int)(((MOTOR_control.cruise_orderSpeed*60.0)/(PI*2.0*CAR_WHEEL_RADIUM))*16.0);
  if(vel > SPEED_MAX/SPEED_RATE)
  {
    vel = SPEED_MAX/SPEED_RATE;
  }

  //�ٶ����ƣ���ֹ�����������ٶȷ�Χ������
  //if(MOTOR_control.cruise_rightSpeed > (SPEED_MAX/SPEED_RATE))
 // {
 //   MOTOR_control.cruise_rightSpeed = SPEED_MAX/SPEED_RATE;
 // }else if(MOTOR_control.cruise_rightSpeed<0)
 // {
 //   MOTOR_control.cruise_rightSpeed=0;
 // }
  
  //    printf("APP�����ٶ�=%d �����ٶ�=%d\n",UART3_group.revBuf[3],MOTOR_control.cruise_rightSpeed);
  //MOTOR_control.cruise_leftSpeed   = ~MOTOR_control.cruise_rightSpeed+1;
  
  

  //ת��Ƕ�����
  angle_t = atan((double)LENGTH_B*2/LENGTH_L);
  
  if(angle > angle_t){angle = angle_t;}
  //����ת��Ƕ������ٶ�
  vel = (uint32_t)(vel * cos(angle));
  
  right_vel = 0;
  left_vel = 0;
 
  
 // left_vel = vel*(1 - ((double)LENGTH_L/(2*LENGTH_B)) * tan(angle));
 // right_vel = vel*(1 + ((double)LENGTH_L/(2*LENGTH_B)) * tan(angle)); 
  
  
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
  
  
   //  printf("�����ٶȣ�%.3f %.3f %0.3f\n",left_vel,right_vel,angle);
     
   MOTOR_control.cruise_leftSpeed  = (uint32_t)left_vel;
   MOTOR_control.cruise_rightSpeed = (uint32_t)right_vel;
   //���ֵ����  
   if(MOTOR_control.cruise_leftSpeed > SPEED_MAX/SPEED_RATE)
   {
     MOTOR_control.cruise_leftSpeed = SPEED_MAX/SPEED_RATE;
   }
   if(MOTOR_control.cruise_rightSpeed > SPEED_MAX/SPEED_RATE)
   {
     MOTOR_control.cruise_rightSpeed = SPEED_MAX/SPEED_RATE;
   }
   
   if(MOTOR_control.cruise_leftSpeed < 0)
   {
     MOTOR_control.cruise_leftSpeed = 0;
   }
   if(MOTOR_control.cruise_rightSpeed < 0)
   {
     MOTOR_control.cruise_rightSpeed = 0;
   }
   
    //����Ѳ��ֻ��ǰ��
    if(MOTOR_control.cruise_leftSpeed > 0){
       MOTOR_control.cruise_leftSpeed   = ~MOTOR_control.cruise_leftSpeed+1;
    }

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
