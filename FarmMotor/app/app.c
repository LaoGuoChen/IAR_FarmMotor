
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
#include "bsp_uart3.h"
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
/*
********************************************************************************
                               函数声明     
********************************************************************************
*/
void setCruiseSpeed(double angle,uint8_t dir);
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
   
    //发送速度指令
    SendSpeedValue();
    
  
   if(HEART_TIME1 == HeartTime || 2*HEART_TIME1 == HeartTime || HEART_TIME2 - 3 == HeartTime)
   {
      //反馈状态机
     TxMessage.StdId = USER_STDID_ADR4;          
     TxMessage.IDE=CAN_ID_STD;           //标准标识符
     TxMessage.RTR=CAN_RTR_DATA;
     TxMessage.DLC=1;

     TxMessage.Data[0]=STATE_machine;
 
     CAN_Transmit(CAN1,&TxMessage);
     
     Delay_Ms(1);
     //报警信息输出
     TxMessage.StdId = USER_STDID_ADR10;             
     TxMessage.Data[0]=DEBUG_err;
 
     CAN_Transmit(CAN1,&TxMessage);
     
  

   }
  
   if(HEART_TIME2 - 2 == HeartTime)
   { 
      //反馈发动机继电器
     TxMessage.StdId = USER_STDID_ADR5;             
     TxMessage.IDE=CAN_ID_STD;           //标准标识符
     TxMessage.RTR=CAN_RTR_DATA;
     TxMessage.DLC=1;

     TxMessage.Data[0]=ENGINE_relayState;

     CAN_Transmit(CAN1,&TxMessage);
     //反馈发动机状态
     Delay_Ms(1);
     TxMessage.StdId = USER_STDID_ADR11;  
     TxMessage.Data[0] = ENGINE_state;
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
                void SendSpeedValue(void)

描述：电机控制总线定时发送数据，时基是基于定时器5中断
参数：

返回值：最终所处状态
********************************************************************************
*/

void SendSpeedValue(void)
{
  
  static CanTxMsg TxMessage;
  static short int can_leftSpeed,can_rightSpeed; 
  
 
  if(STATE_machine == handleControl)
  {
    //再遥控器控制模式下发速度指令
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
    TxMessage.IDE=CAN_ID_STD;           //标准标识符
    TxMessage.RTR=CAN_RTR_DATA;
    TxMessage.DLC=4;
    
    TxMessage.Data[0]=(uint8_t)can_leftSpeed;
    TxMessage.Data[1]=(uint8_t)(can_leftSpeed>>8);
    TxMessage.Data[2]=(uint8_t)can_rightSpeed;
    TxMessage.Data[3]=(uint8_t)(can_rightSpeed>>8);
    
    CAN_Transmit(CAN1,&TxMessage);
    //  printf("发送速度：%d %d\n",MOTOR_control.leftSpeed,MOTOR_control.rightSpeed);
    
  }else if(STATE_machine == cruiseControl && 1 == WORK_condition.curise_start)
  {
    
    //定速巡航发速度指令
    TxMessage.StdId = USER_STDID_ADR3;             
    TxMessage.IDE=CAN_ID_STD;           //标准标识符
    TxMessage.RTR=CAN_RTR_DATA;
    TxMessage.DLC=4;
    
    TxMessage.Data[0]=(uint8_t) MOTOR_control.cruise_leftSpeed;
    TxMessage.Data[1]=(uint8_t)(MOTOR_control.cruise_leftSpeed>>8);
    TxMessage.Data[2]=(uint8_t) MOTOR_control.cruise_rightSpeed;
    TxMessage.Data[3]=(uint8_t)(MOTOR_control.cruise_rightSpeed>>8);
    
     printf("发送速度：%d %d\n",MOTOR_control.cruise_leftSpeed,MOTOR_control.cruise_rightSpeed);
    
    CAN_Transmit(CAN1,&TxMessage);
  }
  
  
  
}

/*
********************************************************************************
                void CruiseControlAsk(void)

描述：总线控制状态下事件处理
参数：
     
返回值：最终所处状态
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
    EngineRelay(ENGINE_relayState=TURN_OFF);  
    
    //***********电机报警进入急停start*************//
  //  TIM_Cmd(TIM5, DISABLE);  //关闭定时器，

    
    if(1 == MSG_Event.event_motorAlarm) //如果是电机报警测读取报警信息
    {
         DEBUG_err |= 0x04;
  //   printf("左电机报警\n");
         
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
 //   TIM_Cmd(TIM5, ENABLE);  //开启定时器
    //***********电机报警进入急停end*************//
    
    Delay_Ms(500);  //延时等待电机减速至0
  
    MSG_Event.event_orderStop =0; //事件标志清除
    MSG_Event.event_motorAlarm=0;
    MSG_Event.event_noPower=0;
    next_state = urgentStop;  //下一个状态
 //   printf("状态：%d\n",next_state);
 
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
  }else if(MSG_Event.event_orderCruise)//进入定速巡航
  {
    
    MOTOR_control.leftSpeed  = MOTOR_control.rightSpeed = 0;
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3, ENABLE);
    MSG_Event.event_orderCruise=0;
    next_state = cruiseControl;   
    WORK_condition.state = 0;   //遥控器切换到普通模式
    
  }else if(MSG_Event.event_engineOFF)  //发动机开关指令
  {
    
    if(1 == MSG_Event.event_engineOFF) //关发动机
    {
      EngineRelay(ENGINE_relayState=TURN_OFF);
      
    }else if(2 == MSG_Event.event_engineOFF)              //开发动机继电器
    {
      EngineRelay(ENGINE_relayState=TURN_ON);

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
       MOTOR_control.motor_dirL = 0;   
       MOTOR_control.motor_dirR = 1;  
   
   //    printf("车前进%d %d\n",MOTOR_control.can_leftSpeed,MOTOR_control.can_rightSpeed);
 
     }else if(MOTOR_control.can_leftSpeed > 0 && MOTOR_control.can_rightSpeed <0)
     { //后退
       MOTOR_control.motor_dirL = 1;   
       MOTOR_control.motor_dirR = 0; 
      
   //    printf("车后退%d %d\n",MOTOR_control.can_leftSpeed,MOTOR_control.can_rightSpeed);
       
     }else if(MOTOR_control.can_leftSpeed > 0 && MOTOR_control.can_rightSpeed >0)
     {//逆时针
       MOTOR_control.motor_dirL = 1;
       MOTOR_control.motor_dirR = 1;
   //    printf("车逆时针%d %d\n",MOTOR_control.can_leftSpeed,MOTOR_control.can_rightSpeed);
       
     }else if(MOTOR_control.can_leftSpeed < 0 && MOTOR_control.can_rightSpeed <0)
     {//顺时针
       MOTOR_control.motor_dirL = 0;
       MOTOR_control.motor_dirR = 0;

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
    
    EngineRelay(ENGINE_relayState=TURN_OFF);
    
 //   printf("遥控器急停：报警%d 指令%d 电量低%d\n",MSG_Event.event_motorAlarm ,MSG_Event.event_orderStop,MSG_Event.event_noPower);
      //***********电机报警进入急停start*************//

    if(1 == MSG_Event.event_motorAlarm) //如果是电机报警测读取报警信息
    {
 
      DEBUG_err |= 0x04;
      
    }
    if(2 == MSG_Event.event_motorAlarm)
    {
      DEBUG_err |= 0x08;
      
    }
    //***********电机报警进入急停end*************//
    
    
    if(MSG_Event.event_noPower)
    {
      DEBUG_err |= 0x10;
    }
    
    if(MSG_Event.event_orderStop)
    {
      DEBUG_err |= 0x20;
    }
    
    
    Delay_Ms(500);


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
    
  }else if(MSG_Event.event_orderCruise)//进入定速巡航
  {
    
    MOTOR_control.leftSpeed  = MOTOR_control.rightSpeed = 0;
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3, ENABLE);
    MSG_Event.event_orderCruise=0;
    next_state = cruiseControl;   
    WORK_condition.state = 0;   //遥控器切换到普通模式
    
//    printf("--进入定速巡航--%d",next_state);
    
  }
  else if(MSG_Event.event_engineOFF)  //发动机开关指令
  {
      
      if(1 == MSG_Event.event_engineOFF) //关发动机
      {
        EngineRelay(ENGINE_relayState=TURN_OFF);
        
      }else if(2 == MSG_Event.event_engineOFF)              //开发动机继电器
      {
        EngineRelay(ENGINE_relayState=TURN_ON);
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
                 StateMachine EventProcessing_e1void)

描述：遥控器定速巡航模式下事件处理
参数：
     
返回值：最终所处状态
********************************************************************************
*/
StateMachine EventProcessing_e2(StateMachine state)
{

  StateMachine next_state;
  next_state= state;
  
  //进入急停状态
  if(MSG_Event.event_motorAlarm || MSG_Event.event_orderStop || MSG_Event.event_noPower)
  {
    
    MOTOR_control.leftSpeed  = MOTOR_control.rightSpeed = 0;
    
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3, DISABLE);
    
    EngineRelay(ENGINE_relayState=TURN_OFF);
    
 //   printf("遥控器急停：报警%d 指令%d 电量低%d\n",MSG_Event.event_motorAlarm ,MSG_Event.event_orderStop,MSG_Event.event_noPower);
      //***********电机报警进入急停start*************//

    if(1 == MSG_Event.event_motorAlarm) //如果是电机报警测读取报警信息
    {
 
      DEBUG_err |= 0x04;
      
    }
    if(2 == MSG_Event.event_motorAlarm)
    {
      DEBUG_err |= 0x08;
      
    }
    //***********电机报警进入急停end*************//
    
    
    if(MSG_Event.event_noPower)
    {
      DEBUG_err |= 0x10;
    }
    
    if(MSG_Event.event_orderStop)
    {
      DEBUG_err |= 0x20;
    }
    
    
    Delay_Ms(500);


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
    
    
    MOTOR_control.cruise_leftSpeed=0;
    MOTOR_control.cruise_rightSpeed=0;
    MOTOR_control.cruise_orderSpeed=0;
 //   printf("遥控器控制进入遥控器控制%d\n",next_state);
     
  }else if(MSG_Event.event_orderCAN) //进入总线控制
  {
    MOTOR_control.leftSpeed  = MOTOR_control.rightSpeed= 0;
    MSG_Event.event_orderCAN=0;
    next_state = busControl;       //下一个状态
    
    MOTOR_control.cruise_leftSpeed=0;
    MOTOR_control.cruise_rightSpeed=0;
    MOTOR_control.cruise_orderSpeed=0;
    
 //   printf("遥控器进入总线控制%d\n",next_state);
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3, DISABLE);
    
  }else if(MSG_Event.event_orderCruise)//进入定速巡航
  {
    
    MOTOR_control.leftSpeed  = MOTOR_control.rightSpeed = 0;
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3, ENABLE);
    MSG_Event.event_orderCruise=0;
    next_state = cruiseControl;   
    WORK_condition.state = 0;   //遥控器切换到普通模式
    
  }
  else if(MSG_Event.event_engineOFF)  //发动机开关指令
  {
      
      if(1 == MSG_Event.event_engineOFF) //关发动机
      {
        EngineRelay(ENGINE_relayState=TURN_OFF);
        
      }else if(2 == MSG_Event.event_engineOFF)              //开发动机继电器
      {
        EngineRelay(ENGINE_relayState=TURN_ON);
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
   
 //  printf("速度左=%d 右=%d \n",MOTOR_control.leftSpeed,MOTOR_control.rightSpeed);
   
   if(cruiseControl == STATE_machine ){
     setCruiseSpeed(angle,dir);
   }
}

void setCruiseSpeed(double angle,uint8_t dir)
{
  double left_vel,right_vel,angle_t;
  //定速巡航给定速度
  double vel = (int)(((MOTOR_control.cruise_orderSpeed*60.0)/(PI*2.0*CAR_WHEEL_RADIUM))*16.0);
  if(vel > SPEED_MAX/SPEED_RATE)
  {
    vel = SPEED_MAX/SPEED_RATE;
  }

  //速度限制，防止发送驱动器速度范围外数据
  //if(MOTOR_control.cruise_rightSpeed > (SPEED_MAX/SPEED_RATE))
 // {
 //   MOTOR_control.cruise_rightSpeed = SPEED_MAX/SPEED_RATE;
 // }else if(MOTOR_control.cruise_rightSpeed<0)
 // {
 //   MOTOR_control.cruise_rightSpeed=0;
 // }
  
  //    printf("APP给定速度=%d 计算速度=%d\n",UART3_group.revBuf[3],MOTOR_control.cruise_rightSpeed);
  //MOTOR_control.cruise_leftSpeed   = ~MOTOR_control.cruise_rightSpeed+1;
  
  

  //转弯角度限制
  angle_t = atan((double)LENGTH_B*2/LENGTH_L);
  
  if(angle > angle_t){angle = angle_t;}
  //根据转弯角度限制速度
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
  
  
   //  printf("计算速度：%.3f %.3f %0.3f\n",left_vel,right_vel,angle);
     
   MOTOR_control.cruise_leftSpeed  = (uint32_t)left_vel;
   MOTOR_control.cruise_rightSpeed = (uint32_t)right_vel;
   //最大值限制  
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
   
    //定速巡航只有前进
    if(MOTOR_control.cruise_leftSpeed > 0){
       MOTOR_control.cruise_leftSpeed   = ~MOTOR_control.cruise_leftSpeed+1;
    }

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
