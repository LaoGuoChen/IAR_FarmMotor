/*
********************************************************************************
                                
文件名：app_conf.h
版本：
程序员：Tank_CG

********************************************************************************
*/

#ifndef  __APP_CONF_H
#define  __APP_CONF_H


/*
********************************************************************************
                               头文件        
********************************************************************************
*/
#include "stm32f10x.h"


/*
********************************************************************************
                                   宏定义
********************************************************************************
*/
#define CAPTURE_F                 100           //脉宽捕获频率KHz
#define ZERO_RANGE                50            //在原点值基础上+-这个值就是原点范围，在这个范围内都认为是远点
#define PULSE_VEL_ZERO            1530          //us速度控制，<1500是向后加速 >1500是向前加速 +_500
#define PULSE_VEL                 (500-ZERO_RANGE)          //速度调节范围
#define PULSE_DIR_ZERO            1460          //us方向控制  >1500++是左转，<1500--是向右转 +_500
#define PULSE_DIR                 (500-ZERO_RANGE)           //反向调节范围
#define PULSE3_ZERO               1500         
#define RATIO_VEL                 0.5          //倒车速度系数，前进速度的倍数

#define DIR_ANGLE_MAX             90            //转弯最大角度

#define SPEED_MAX                  10000         //电机驱动最大速度，rpm/min
#define SPEED_RATE                 10           //速度倍率，CAN发来的速度数据

#define PI 3.14159265

#define LENGTH_L               0.652 //车轮轴间距
#define LENGTH_B               1     //虚拟前轮与后轮垂直距离

#define RS_ONLINE_CNT          6     //在线检测计数值，发送信息计数，

#define HEART_TIME1            20    //1秒，此值是根据定时器中断计数，若定时器改变测需要改变
#define HEART_TIME2            60    //3秒，此值是根据定时器中断计数，若定时器改变测需要改变

#define POWER_RATE1             11.0    //单个电池电压倍率
#define POWER_RATE2             21.0    //两个电池电压倍率
#define POWER_RATE3             (2.0*(1.0/5.0)*(250.0/0.5))    //液位深度转换单位米

#define PWOER_DEFAULT         (3.8*6)    //电池电压值
#define PWOER_MSG             100        //电池电压值系数，0-10000表示0-100，
        

#define RS_232_CTR             0//速度是否为232控制，0否1是
/*
*状态1->2，条件vel_flag=0 && angle_flag=0 && 按键触发。2->1按键处罚 || vel_flag
*/
typedef struct {
  volatile uint8_t vel_flag;     //油门的标志，0为速度为零，1为速度>0.
  volatile uint8_t angle_flag;   //方向盘标志，0为角度零，1为角度>0.
  volatile uint8_t online_flag;  //在线标志，即是控制手柄掉电或关机,0掉线1在线
  volatile uint8_t state;        //模式，0为正常模式，1为旋转模式 
  
  
}StateCondition;


typedef struct{
  uint8_t  revBuf[20]; //接收缓存
  uint8_t  revFlag;   //串口接收标志,1接收完整数据      
  
}UartGroup;

typedef struct{        //ADC采样
  double  powerVal1;    //电源24V 
  double  powerVal2;    //电源48V
  double  waterLevel;   //液位传感器
  
}ADC_Sampling;


typedef enum 
{
  TURN_OFF=0,
  TURN_ON
  
}SwitchState;

typedef enum{    //状态
  busControl=0,     //指令控制状态
  handleControl=1,  //遥控器状态
  urgentStop=2      //急停状态
  
}StateMachine;

typedef struct{  //消息事件 为1时表示事件发生0表示没发生
  uint8_t event_motorAlarm;     //电机报警1左电机2右电机
  uint8_t event_xx;            //
  uint8_t event_orderStop;     //急停
  uint8_t event_noPower;       //电池电量低
  uint8_t event_orderHandle;   //进入遥控器模式
  uint8_t event_orderSpeed;    //速度指令 
  uint8_t event_orderCAN;      //总线控制
  uint8_t event_engineOFF;     //关发动机 1关发动机继电器2开发动机继电器
  
}MSG_Group;

typedef struct{ 
  
  signed short  can_leftSpeed;  //can接收到的速度
  signed short  can_rightSpeed;
  
  int            leftSpeed;   //电机当前速度
  int            rightSpeed;
  //前进
  uint8_t        motor_dirL;   //电机lift速度方向标志0逆时针1顺时针
  uint8_t        motor_dirR;   //电机right速度方向标志
  
}MotorCtrGroup;
/*
********************************************************************************
                                   全局变量
********************************************************************************
*/

extern SwitchState      ENGINE_state;   //发送机继电器转台
extern SwitchState      LED_state;
extern SwitchState      MOTOR_state;  //伺服电机使能状态

extern StateMachine     STATE_machine; //状态机
extern MSG_Group        MSG_Event;    //消息事件   

extern StateCondition   WORK_condition;
extern UartGroup        UART1_group;  //速度指令返回
extern UartGroup        UART4_group;

extern ADC_Sampling     POWER_val;


extern  MotorCtrGroup MOTOR_control;

extern  uint8_t DEBUG_err;






/*
********************************************************************************
                            函数申明
********************************************************************************
*/

void            StateTransformation       (void);
void            SetCarSpeed               (double angle,uint8_t dir,uint32_t vel);
void            StateMachineSwitching     (void);
StateMachine    EventProcessing_e0        (StateMachine state);
StateMachine    EventProcessing_e1        (StateMachine state);

void            UART_DataCommunication         (void);
void            CAN_DataCommunication         (void);

#endif 
