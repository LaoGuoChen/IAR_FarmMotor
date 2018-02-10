/*
********************************************************************************
                                
�ļ�����app_conf.h
�汾��
����Ա��Tank_CG

********************************************************************************
*/

#ifndef  __APP_CONF_H
#define  __APP_CONF_H


/*
********************************************************************************
                               ͷ�ļ�        
********************************************************************************
*/
#include "stm32f10x.h"


/*
********************************************************************************
                                   �궨��
********************************************************************************
*/
#define CAPTURE_F                 100           //������Ƶ��KHz
#define ZERO_RANGE                50            //��ԭ��ֵ������+-���ֵ����ԭ�㷶Χ���������Χ�ڶ���Ϊ��Զ��
#define PULSE_VEL_ZERO            1530          //us�ٶȿ��ƣ�<1500�������� >1500����ǰ���� +_500
#define PULSE_VEL                 (500-ZERO_RANGE)          //�ٶȵ��ڷ�Χ
#define PULSE_DIR_ZERO            1460          //us�������  >1500++����ת��<1500--������ת +_500
#define PULSE_DIR                 (500-ZERO_RANGE)           //������ڷ�Χ
#define PULSE3_ZERO               1500         
#define RATIO_VEL                 0.5          //�����ٶ�ϵ����ǰ���ٶȵı���

#define DIR_ANGLE_MAX             90            //ת�����Ƕ�

#define SPEED_MAX                  10000         //�����������ٶȣ�rpm/min
#define SPEED_RATE                 10           //�ٶȱ��ʣ�CAN�������ٶ�����

#define PI 3.14159265

#define LENGTH_L               0.652 //��������
#define LENGTH_B               1     //����ǰ������ִ�ֱ����

#define RS_ONLINE_CNT          6     //���߼�����ֵ��������Ϣ������

#define HEART_TIME1            20    //1�룬��ֵ�Ǹ��ݶ�ʱ���жϼ���������ʱ���ı����Ҫ�ı�
#define HEART_TIME2            60    //3�룬��ֵ�Ǹ��ݶ�ʱ���жϼ���������ʱ���ı����Ҫ�ı�

#define POWER_RATE1             11.0    //������ص�ѹ����
#define POWER_RATE2             21.0    //������ص�ѹ����
#define POWER_RATE3             (2.0*(1.0/5.0)*(250.0/0.5))    //Һλ���ת����λ��

#define PWOER_DEFAULT         (3.8*6)    //��ص�ѹֵ
#define PWOER_MSG             100        //��ص�ѹֵϵ����0-10000��ʾ0-100��
        

#define RS_232_CTR             0//�ٶ��Ƿ�Ϊ232���ƣ�0��1��
/*
*״̬1->2������vel_flag=0 && angle_flag=0 && ����������2->1�������� || vel_flag
*/
typedef struct {
  volatile uint8_t vel_flag;     //���ŵı�־��0Ϊ�ٶ�Ϊ�㣬1Ϊ�ٶ�>0.
  volatile uint8_t angle_flag;   //�����̱�־��0Ϊ�Ƕ��㣬1Ϊ�Ƕ�>0.
  volatile uint8_t online_flag;  //���߱�־�����ǿ����ֱ������ػ�,0����1����
  volatile uint8_t state;        //ģʽ��0Ϊ����ģʽ��1Ϊ��תģʽ 
  
  
}StateCondition;


typedef struct{
  uint8_t  revBuf[20]; //���ջ���
  uint8_t  revFlag;   //���ڽ��ձ�־,1������������      
  
}UartGroup;

typedef struct{        //ADC����
  double  powerVal1;    //��Դ24V 
  double  powerVal2;    //��Դ48V
  double  waterLevel;   //Һλ������
  
}ADC_Sampling;


typedef enum 
{
  TURN_OFF=0,
  TURN_ON
  
}SwitchState;

typedef enum{    //״̬
  busControl=0,     //ָ�����״̬
  handleControl=1,  //ң����״̬
  urgentStop=2      //��ͣ״̬
  
}StateMachine;

typedef struct{  //��Ϣ�¼� Ϊ1ʱ��ʾ�¼�����0��ʾû����
  uint8_t event_motorAlarm;     //�������1����2�ҵ��
  uint8_t event_xx;            //
  uint8_t event_orderStop;     //��ͣ
  uint8_t event_noPower;       //��ص�����
  uint8_t event_orderHandle;   //����ң����ģʽ
  uint8_t event_orderSpeed;    //�ٶ�ָ�� 
  uint8_t event_orderCAN;      //���߿���
  uint8_t event_engineOFF;     //�ط����� 1�ط������̵���2���������̵���
  
}MSG_Group;

typedef struct{ 
  
  signed short  can_leftSpeed;  //can���յ����ٶ�
  signed short  can_rightSpeed;
  
  int            leftSpeed;   //�����ǰ�ٶ�
  int            rightSpeed;
  //ǰ��
  uint8_t        motor_dirL;   //���lift�ٶȷ����־0��ʱ��1˳ʱ��
  uint8_t        motor_dirR;   //���right�ٶȷ����־
  
}MotorCtrGroup;
/*
********************************************************************************
                                   ȫ�ֱ���
********************************************************************************
*/

extern SwitchState      ENGINE_state;   //���ͻ��̵���ת̨
extern SwitchState      LED_state;
extern SwitchState      MOTOR_state;  //�ŷ����ʹ��״̬

extern StateMachine     STATE_machine; //״̬��
extern MSG_Group        MSG_Event;    //��Ϣ�¼�   

extern StateCondition   WORK_condition;
extern UartGroup        UART1_group;  //�ٶ�ָ���
extern UartGroup        UART4_group;

extern ADC_Sampling     POWER_val;


extern  MotorCtrGroup MOTOR_control;

extern  uint8_t DEBUG_err;






/*
********************************************************************************
                            ��������
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
