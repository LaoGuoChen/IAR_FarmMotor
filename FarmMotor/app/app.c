/*
********************************************************************************
                                用户程序
文件名：app.c
版本：
程序员：Tank_CG
********************************************************************************
*/

/*
********************************************************************************
                               头文件       
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
                               全局变量      
********************************************************************************
*/
extern uint8_t HeartTime;
/*
********************************************************************************
                               局部变量      
********************************************************************************
*/



//驱动器数据地址
static uint8_t   sendData1[11] = {0x00,0x10,0x44,0x20,0x00,0x01,0x02,0x00,0x00,0x00,0x00};
static uint8_t   sendData2[13] = {0x00,0x10,0x44,0x20,0x00,0x02,0x04,0x00,0x00,0xFF,0xFF,0x00,0x00};
static uint8_t   sendData3[8] =  {0x00,0x03,0x44,0x71,0x00,0x01,0x00,0x00};//读取电流值，
static uint8_t   sendData4[8] =  {0x00,0x03,0x46,0x6E,0x00,0x04,0x00,0x00};//读取报警数据，
static uint16_t  check_val=0;



/*
********************************************************************************
                  void CAN_DataCommunication (void)

描述：CAN定时发送数据，时基是基于定时器5中断
参数：
     
返回值：最终所处状态
********************************************************************************
*/
void CAN_DataCommunication (void)
{
   static CanTxMsg TxMessage;
   //反馈状态机
   if(HEART_TIME1 == HeartTime || 2*HEART_TIME1 == HeartTime || HEART_TIME2 - 3 == HeartTime)
   {
     TxMessage.StdId = USER_STDID_ADR4;             
     TxMessage.IDE=CAN_ID_STD;           //标准标识符
     TxMessage.RTR=CAN_RTR_DATA;
     TxMessage.DLC=1;

     TxMessage.Data[0]=STATE_machine;
 
     CAN_Transmit(CAN1,&TxMessage);

   }
   //反馈发动机继电器
   if(HEART_TIME2 - 2 == HeartTime)
   {  
     TxMessage.StdId = USER_STDID_ADR5;             
     TxMessage.IDE=CAN_ID_STD;           //标准标识符
     TxMessage.RTR=CAN_RTR_DATA;
     TxMessage.DLC=1;

     TxMessage.Data[0]=ENGINE_state;

     CAN_Transmit(CAN1,&TxMessage);

   }
   //反馈水位
   if(HEART_TIME2 - 1 == HeartTime)
   {   
     TxMessage.StdId = USER_STDID_ADR7;             
     TxMessage.IDE=CAN_ID_STD;           //标准标识符
     TxMessage.RTR=CAN_RTR_DATA;
     TxMessage.DLC=1;
     TxMessage.Data[0]=(uint8_t)(POWER_val.waterLevel);

     CAN_Transmit(CAN1,&TxMessage);
   }
   //反馈电池电量
   if(HEART_TIME2 == HeartTime)
   {
     static  uint16_t power_val =0;
     power_val = (uint16_t)(POWER_val.powerVal2*PWOER_MSG);
     
     TxMessage.StdId = USER_STDID_ADR8;             
     TxMessage.IDE=CAN_ID_STD;           //标准标识符
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

描述：电机控制总线定时发送数据，时基是基于定时器5中断
参数：
     
返回值：最终所处状态
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
   //   printf("发送速度：%d %d\n",MOTOR_control.leftSpeed,MOTOR_control.rightSpeed);

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

描述：总线控制状态下事件处理
参数：
     
返回值：最终所处状态
********************************************************************************
*/
StateMachine EventProcessing_e0(StateMachine state)
{
  StateMachine next_state;
  next_state= state;
  

  //进入急停状态
  if(MSG_Event.event_motorAlarm || MSG_Event.event_orderStop || MSG_Event.event_noPower)
  {
    
 //  printf("总线控制急停：报警%d 指令%d 电量低%d\n",MSG_Event.event_motorAlarm ,MSG_Event.event_orderStop,MSG_Event.event_noPower);
    
    MOTOR_control.leftSpeed  = MOTOR_control.rightSpeed = 0;
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3, DISABLE);
    relayControl(RELAY_1, ENGINE_state=TURN_OFF);  
    
    //***********电机报警进入急停start*************//
    TIM_Cmd(TIM5, DISABLE);  //关闭定时器，
    
    check_val= CRC16_MODBUS(sendData4,6);
    
    sendData4[6] = (uint8_t)check_val;
    sendData4[7] = (uint8_t)(check_val>>8); 
    
    if(1 == MSG_Event.event_motorAlarm) //如果是电机报警测读取报警信息
    {
         
        
        UART1_sendData(8,sendData4);
        UART1_group.revFlag = 0;
     // printf("左电机报警\n");
      
    }
    if(2 == MSG_Event.event_motorAlarm)
    {
      UART4_sendData(8,sendData4);
      UART4_group.revFlag = 0;
    //  printf("右电机报警\n");
    }
    
    TIM_Cmd(TIM5, ENABLE);  //开启定时器
    //***********电机报警进入急停end*************//
    
    Delay_Ms(500);  //延时等待电机减速至0
    MotorEnable(MOTOR_state = TURN_OFF);
    
    
    MSG_Event.event_orderStop =0; //事件标志清除
    MSG_Event.event_motorAlarm=0;
    MSG_Event.event_noPower=0;
    next_state = urgentStop;  //下一个状态
 
  }else if(MSG_Event.event_orderHandle)//进入遥控器控制
  {
    
    
    MOTOR_control.leftSpeed  = MOTOR_control.rightSpeed = 0;
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3, ENABLE);
    
    MSG_Event.event_orderHandle=0;
    next_state = handleControl;    //下一个状态
    
  //  printf("总线控制进入遥控器控制%d\n",next_state);
     
  }else if(MSG_Event.event_orderCAN) //进入总线控制
  {
    MOTOR_control.leftSpeed  = MOTOR_control.rightSpeed= 0;
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3, DISABLE);
    
    MSG_Event.event_orderCAN=0;
    next_state = busControl;       //下一个状态
    
  //  printf("总线控制进入总线控制%d\n",next_state);
  }
  else if(MSG_Event.event_engineOFF)  //发动机开关指令
  {
    
    if(1 == MSG_Event.event_engineOFF) //关发动机
    {
      relayControl(RELAY_1, ENGINE_state=TURN_OFF);
      
    }else if(2 == MSG_Event.event_engineOFF)              //开发动机继电器
    {
      relayControl(RELAY_1, ENGINE_state=TURN_ON);

    }
    
    MSG_Event.event_engineOFF = 0;
    
  }else if(MSG_Event.event_orderSpeed)
  {
      TIM_Cmd(TIM5, DISABLE);  //关闭定时器，保护数据
      
   //   printf("总线控制速度指令 左电机%d 右电机%d \n",MOTOR_control.can_leftSpeed,MOTOR_control.can_rightSpeed);
       
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
       
      //前进
     if(MOTOR_control.can_leftSpeed < 0 && MOTOR_control.can_rightSpeed > 0)
     {
       MOTOR_control.motor_dir1 = 0;   
       MOTOR_control.motor_dir2 = 1;  
   
   //    printf("车前进%d %d\n",MOTOR_control.can_leftSpeed,MOTOR_control.can_rightSpeed);
 
     }else if(MOTOR_control.can_leftSpeed > 0 && MOTOR_control.can_rightSpeed <0)
     { //后退
       MOTOR_control.motor_dir1 = 1;   
       MOTOR_control.motor_dir2 = 0; 
      
   //    printf("车后退%d %d\n",MOTOR_control.can_leftSpeed,MOTOR_control.can_rightSpeed);
       
     }else if(MOTOR_control.can_leftSpeed > 0 && MOTOR_control.can_rightSpeed >0)
     {//逆时针
       MOTOR_control.motor_dir1 = 1;
       MOTOR_control.motor_dir2 = 1;
   //    printf("车逆时针%d %d\n",MOTOR_control.can_leftSpeed,MOTOR_control.can_rightSpeed);
       
     }else if(MOTOR_control.can_leftSpeed < 0 && MOTOR_control.can_rightSpeed <0)
     {//顺时针
       MOTOR_control.motor_dir1 = 0;
       MOTOR_control.motor_dir2 = 0;

  //     printf("车顺时针 %d %d\n",MOTOR_control.can_leftSpeed,MOTOR_control.can_rightSpeed);
     }
     
  //   printf("实际速度%d %d\n",MOTOR_control.leftSpeed,MOTOR_control.rightSpeed);
     //速度限制
     if(MOTOR_control.leftSpeed > SPEED_MAX){MOTOR_control.leftSpeed = SPEED_MAX;}
     if(MOTOR_control.rightSpeed > SPEED_MAX){MOTOR_control.rightSpeed = SPEED_MAX;}
     if(MOTOR_control.leftSpeed < 0){MOTOR_control.leftSpeed = 0;}
     if(MOTOR_control.rightSpeed < 0){MOTOR_control.rightSpeed = 0;}
     
     TIM_Cmd(TIM5, ENABLE);  //开定时器
     MSG_Event.event_orderSpeed=0;
  }
  
  return next_state;
}

/*
********************************************************************************
                 StateMachine EventProcessing_e1void)

描述：遥控器模式下事件处理
参数：
     
返回值：最终所处状态
********************************************************************************
*/
StateMachine EventProcessing_e1(StateMachine state)
{

  StateMachine next_state;
  next_state= state;
  
  //进入急停状态
  if(MSG_Event.event_motorAlarm || MSG_Event.event_orderStop || MSG_Event.event_noPower)
  {
    
    MOTOR_control.leftSpeed  = MOTOR_control.rightSpeed = 0;
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3, DISABLE);
    relayControl(RELAY_1, ENGINE_state=TURN_OFF);
    
 //   printf("遥控器急停：报警%d 指令%d 电量低%d\n",MSG_Event.event_motorAlarm ,MSG_Event.event_orderStop,MSG_Event.event_noPower);
      //***********电机报警进入急停start*************//
    TIM_Cmd(TIM5, DISABLE);  //关闭定时器，

    check_val= CRC16_MODBUS(sendData4,6);
    
    sendData4[6] = (uint8_t)check_val;
    sendData4[7] = (uint8_t)(check_val>>8); 
    if(1 == MSG_Event.event_motorAlarm) //如果是电机报警测读取报警信息
    {
      UART1_sendData(8,sendData4);
      UART1_group.revFlag = 0;
    //  printf("左电机报警\n");
      
    }
    if(2 == MSG_Event.event_motorAlarm)
    {
      UART4_sendData(8,sendData4);
      UART4_group.revFlag = 0;
    //  printf("右电机报警\n");
    }
    TIM_Cmd(TIM5, ENABLE);  //开定时器
    //***********电机报警进入急停end*************//
    
    Delay_Ms(500);
    MotorEnable(MOTOR_state = TURN_OFF);

    MSG_Event.event_orderStop =0; //事件标志清除
    MSG_Event.event_motorAlarm=0;
    MSG_Event.event_noPower=0;
    next_state = urgentStop;  //下一个状态
   
 
  }else if(MSG_Event.event_orderHandle)//进入手动控制
  {
   
    MOTOR_control.leftSpeed  = MOTOR_control.rightSpeed = 0;
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3, ENABLE);
    
    MSG_Event.event_orderHandle=0;
    next_state = handleControl;    //下一个状态
    
    WORK_condition.state = 0;   //遥控器切换到普通模式
 //   printf("遥控器控制进入遥控器控制%d\n",next_state);
     
  }else if(MSG_Event.event_orderCAN) //进入总线控制
  {
    MOTOR_control.leftSpeed  = MOTOR_control.rightSpeed= 0;
    MSG_Event.event_orderCAN=0;
    next_state = busControl;       //下一个状态
 //   printf("遥控器进入总线控制%d\n",next_state);
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3, DISABLE);
  }
  else if(MSG_Event.event_engineOFF)  //发动机开关指令
  {
      
      if(1 == MSG_Event.event_engineOFF) //关发动机
      {
        relayControl(RELAY_1, ENGINE_state=TURN_OFF);
        
      }else if(2 == MSG_Event.event_engineOFF)              //开发动机继电器
      {
        relayControl(RELAY_1, ENGINE_state=TURN_ON);
      }
      
      MSG_Event.event_engineOFF = 0;
      
  }else if(MSG_Event.event_orderSpeed)//遥控器模式下发送数据无效
  {

     MSG_Event.event_orderSpeed=0;
  //   printf("遥控器控制速度指令无效\n");
  }
  return next_state;
}

/*
********************************************************************************
                        void  SetCarSpeed (float angle,uint8_t dir)

描述：设置车速度
参数：angle 转角 0-90
      dir   方向 0 直行 1 左转，2右转,3顺时针，4逆时针
     vel    速度
返回值：无
********************************************************************************
*/

void SetCarSpeed(double angle,uint8_t dir,uint32_t vel)
{
  
  double left_vel,right_vel,angle_t;
  //转弯角度限制
  angle_t = atan((double)LENGTH_B*2/LENGTH_L);
  
  if(angle > angle_t){angle = angle_t;}
  //根据转弯角度限制速度
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
   //最大值限制  
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
   
  // printf(" 2速度%d ",motor_vel);
}

/*
********************************************************************************
                        void  StateTransformation()

描述：       状态转换
参数：     
返回值：    无
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
  // printf("状态：%d ",WORK_condition.state);
}
