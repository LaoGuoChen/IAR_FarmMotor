/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "stm32f10x_it.h"
/*
****************************�û����뿪ʼ****************************************
*/   
    
#include <stdio.h>
#include <math.h>
#include "bsp_inputCapture.h"
#include "app_conf.h"
#include "io_output.h"
#include "bsp_uart1.h"
#include "bsp_uart4.h"
#include "crc16_modbus.h"
#include "bsp_adc.h"
#include "bsp_can1.h"

/*
********************************************************************************
                               ȫ�ֱ���      
********************************************************************************
*/
uint8_t HeartTime = 0;


/*
********************************************************************************
                               �ֲ�����      
********************************************************************************
*/
//static uint8_t   uart3_recevieStart = 0;
//static uint8_t   uart3_dataCount = 0;   //�������ݼ���

//static uint8_t   uart4_recevieStart = 0;
//static uint8_t   uart4_dataCount = 0;   //�������ݼ���

//������
static uint16_t  captrue_value1[3] = {0,0,0};
static uint16_t  captrue_value2[3] = {0,0,0};
static uint8_t   captrueFlag[3] = {0,0,0};
static uint16_t  d_value[3] = {0,0,0};



static uint8_t switch_flag = 0;  //���ر�־���л�ģʽ��־

static uint32_t  motor_vel=0;
signed short  canOrder_Lvel=0;
signed short  canOrder_Rvel=0;

static CanRxMsg RxMessage;


/*
********************************************************************************
                     void USB_LP_CAN1_RX0_IRQHandler(void)

������     CAN1�����ж�
������     
����ֵ��    ��
********************************************************************************
*/
void USB_LP_CAN1_RX0_IRQHandler(void)
{

 // NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn); //ʧ��CAN1��Ϣ�����ж�
	
  if(CAN_GetITStatus(CAN1, CAN_IT_FOV0))
  {
     CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0); 
	 
  }else if(CAN_GetITStatus(CAN1, CAN_IT_FMP0))
  {
  	
    CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);

    CAN_Receive(CAN1,CAN_FIFO0,&RxMessage); 
   
    
    switch(RxMessage.StdId)
    {
    case USER_STDID_ADR1:
      if(0 == RxMessage.Data[0]) //����ָ�����״̬
      {
        MSG_Event.event_orderCAN=1;
      }else if(1 == RxMessage.Data[0]) //����ң����״̬
      {
        MSG_Event.event_orderHandle=1;
      }else if(2 == RxMessage.Data[0]) //���뼱ͣ״̬
      {
        MSG_Event.event_orderStop=1;
   //     printf("ָ�ͣ");
      }
  
      HeartTime = HEART_TIME2-4;
      break;
    case USER_STDID_ADR2:
      
      if(0 == RxMessage.Data[0]) //�ط��������̵���ָ��
      {
        MSG_Event.event_engineOFF=1;  
        
      }else if(1 == RxMessage.Data[0])              //���������̵���ָ��
      {
        MSG_Event.event_engineOFF=2;  
      }
 
      HeartTime = HEART_TIME2-4; 
      
      break;
    case USER_STDID_ADR3:
     
   //   MOTOR_control.can_leftSpeed = (signed short)(RxMessage.Data[0] | (RxMessage.Data[1]<<8));
   //   MOTOR_control.can_rightSpeed = (signed short)(RxMessage.Data[2] | (RxMessage.Data[3]<<8));
  
   //   printf("CAN�ٶ�ָ�� ����%d �ҵ��%d \n",MOTOR_control.can_leftSpeed,MOTOR_control.can_rightSpeed);
      
    //  MSG_Event.event_orderSpeed = 1;
    
      break;
  
    }
  }
}

/*
********************************************************************************
                         void USART3_IRQHandler(void)

������     USART3�����ж�
������     
����ֵ��    ��
********************************************************************************
*/

void USART3_IRQHandler(void)
{
  
   if(USART_GetITStatus(USART3,USART_IT_RXNE) != RESET)
  { 
    USART_ClearITPendingBit(USART3,USART_IT_RXNE); //����жϱ�־
    
    uint8_t data = USART_ReceiveData(USART3);  

   
  } 
}


/*
********************************************************************************
                         void UART5_IRQHandler(void)

������     UART5�����ж�
������     
����ֵ��    ��
********************************************************************************
*/

void UART4_IRQHandler(void)
{
  
  if(USART_GetITStatus(UART4,USART_IT_RXNE) != RESET)
  { 
    USART_ClearITPendingBit(UART4,USART_IT_RXNE); //����жϱ�־
    uint8_t data = USART_ReceiveData(UART4); 
    //printf("%x ",data);
    
  }
}



/*
********************************************************************************
                         void TIM5_IRQHandler(void)

������       ��ʱ���ж�50msһ���жϣ�
������     
����ֵ��    ��
********************************************************************************
*/

void TIM5_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM5,TIM_IT_Update) != RESET)  //����жϷ������
  {
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);      //����жϱ�־ 
    
    TIM_Cmd(TIM5, DISABLE);  
    
    CAN_DataCommunication();
    HeartTime++; //CAN���ݷ�����ʱ
    if(HeartTime > HEART_TIME2)
    {
      HeartTime = 0;
    }
    
    static uint8_t led_time=0;//ADC��������
    led_time++;
    if(led_time == 10)
    {
      led_time=0;
      
      if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_12) == 0)
      {
     //   GPIO_SetBits(GPIOE,GPIO_Pin_1);
        GPIO_SetBits(GPIOE,GPIO_Pin_12);
      //  GPIO_SetBits(GPIOE,GPIO_Pin_13);
      //  GPIO_SetBits(GPIOE,GPIO_Pin_14);
      //  GPIO_SetBits(GPIOE,GPIO_Pin_15);
      }else
      {
     //   GPIO_ResetBits(GPIOE,GPIO_Pin_1);
        GPIO_ResetBits(GPIOE,GPIO_Pin_12);
     //   GPIO_ResetBits(GPIOE,GPIO_Pin_13);
     //   GPIO_ResetBits(GPIOE,GPIO_Pin_14);
     //   GPIO_ResetBits(GPIOE,GPIO_Pin_15);
      }
    }
    
    //********��������ͣ���*******/
  //   uint8_t bit = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_3);
    if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_3))
    {
      ENGINE_state = 1;
    }else
    {
      ENGINE_state = 0;
    }

    //************ң�������߼��start**********************//
    if(0 == WORK_condition.online_flag) 
    {
      if(STATE_machine == handleControl)
      {
         MOTOR_control.leftSpeed  = MOTOR_control.rightSpeed = motor_vel = 0;
      }

    } 
    WORK_condition.online_flag = 0;
   
    //************ң�������߼��end**********************//
   
 
    //50ms����һ�Σ��ɼ�ADC_DMA_LEN�μ���һ��ƽ��ֵ
    ADC_Cmd(ADC1, ENABLE); 
    ADC_SoftwareStartConvCmd(ADC1, ENABLE); 
  
   
    TIM_Cmd(TIM5, ENABLE);  
  }
}

/*
********************************************************************************
                         vDMA1_Channel1_IRQHandler(void)

������     ADC�����жϽ����ж�
������     
����ֵ��    ��
********************************************************************************
*/
void  DMA1_Channel1_IRQHandler(void)  
{  
  if(DMA_GetITStatus(DMA1_IT_TC1)!=RESET)
  {  
    DMA_ClearITPendingBit(DMA1_IT_TC1); 
    
    
   // ADC_Cmd(ADC1, DISABLE); 
  // ADC_SoftwareStartConvCmd(ADC1, DISABLE); 
    //���ݴ洢��ȫͨ�������Դ洢������3��ͨ����xx[0]ch1,xx[1]ch2,xx[2]ch3,xx[3]ch1,xx[4]ch2,xx[5]ch3��

    for(u8 i=0;i<ADC_DMA_CH*ADC_DMA_LEN ;i++)
    {
      if(i%ADC_DMA_CH == 0)
      {
         POWER_val.waterLevel +=ADC_ConvertedValue[i];
         
      }else if(i%ADC_DMA_CH == 1)
      {
         POWER_val.powerVal1 +=ADC_ConvertedValue[i];
         
      }else if(i%ADC_DMA_CH == 2)
      {
        
        POWER_val.powerVal2 +=ADC_ConvertedValue[i];
     //   printf("%d ",ADC_ConvertedValue[i]);
       
        
      }
      
    }


    
//    printf("��ص�����%0.3f %0.3f %0.3f\n",((POWER_val.powerVal1/4095)/ADC_DMA_LEN)*3.3,
//((POWER_val.powerVal2/4095)/ADC_DMA_LEN)*3.3,((POWER_val.waterLevel/4095)/ADC_DMA_LEN)*3.3*2.0*(1.0/5.0));
    
    
   // POWER_val.powerVal1  = ((POWER_val.powerVal1/4095)/ADC_DMA_LEN)*3.3*POWER_RATE1;
  //  POWER_val.powerVal2  = ((POWER_val.powerVal2/4095)/ADC_DMA_LEN)*3.3*POWER_RATE2;
  //  POWER_val.waterLevel = ((POWER_val.waterLevel/4095)/ADC_DMA_LEN)*3.3*POWER_RATE3;
    
    //�µ�·����ֵ
    
    POWER_val.powerVal1  = ((POWER_val.powerVal1/ADC_DMA_LEN)/4095)*VOLTAGE_REFERENCE;
    POWER_val.powerVal2  = ((POWER_val.powerVal2/ADC_DMA_LEN)/4095)*VOLTAGE_REFERENCE*POWER_PART_RATE;  
    POWER_val.waterLevel = ((POWER_val.waterLevel/ADC_DMA_LEN)/4095)*VOLTAGE_REFERENCE*LEVEL_RATE;
    
  //  printf("��ص�����%0.3f %0.3f %0.3f\n",POWER_val.powerVal1,POWER_val.powerVal2,POWER_val.waterLevel);
    
    static uint8_t power_check_cnt1=0;
    static uint8_t power_check_cnt2=0;
    static uint8_t level_check_cnt=0;
    
    //��ص�ѹ���ͽ��뼱ͣ
    if(POWER_val.powerVal1 < PWOER_DEFAULT)
      //    if(POWER_val.powerVal1 < 24)
    {
      power_check_cnt1++;
      if(power_check_cnt1>=POWER_CHECK_CNT){
        MSG_Event.event_noPower = 1;
      }
      //    printf("��ع���1��%d\n",MSG_Event.event_noPower );
      
    }else
    {
       power_check_cnt1=0;
    }
    //��ص�ѹ���ͽ��뼱ͣ
    if(POWER_val.powerVal2 > POWER_val.powerVal1)
    {
      if((POWER_val.powerVal2 - POWER_val.powerVal1) < PWOER_DEFAULT)
      {
        power_check_cnt2++;
        if(power_check_cnt2>=POWER_CHECK_CNT){
          MSG_Event.event_noPower =1;
        }
        //      printf("��ع���2��%0.3f\n",POWER_val.powerVal2 - POWER_val.powerVal1 );
      }
    }else
    {
      power_check_cnt2=0;
    }
    //ҩ�����㣬�ر�
    if(POWER_val.waterLevel<LEVEL_DEFAUL)
    {
      level_check_cnt++;
      if(level_check_cnt>=LEVEL_CHECK_CNT)//�ط�����
      {
        EngineRelay(ENGINE_relayState=TURN_OFF); 
      }
    }else
    {
      level_check_cnt=0;
    }
    
  }  
}
/*
********************************************************************************
                      TIM3_IRQHandler()

������     ���벶���жϣ���Ƕ���ư���JP10 ��������
������     
����ֵ��    ��
********************************************************************************
*/
void TIM3_IRQHandler(void)
{ 
  if(TIM_GetITStatus(TIM3,TIM_IT_CC1) != RESET)  //����жϷ������
  { 
      
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);      //����жϱ�־ 
      if(0 == captrueFlag[0])
      {
        captrueFlag[0] = 1;
        captrue_value1[0] = TIM_GetCapture1(TIM3);
        
        
        TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Falling); //�����½���
      }else
      {
        captrueFlag[0] = 0; 
        captrue_value2[0] = TIM_GetCapture1(TIM3);
        
        /*--------------�����ֵ-----------------*/
        if(captrue_value2[0] > captrue_value1[0])
        {
          d_value[0] = captrue_value2[0] - captrue_value1[0];
        }else
        {
          d_value[0] = (0xffff -captrue_value1[0]) + captrue_value2[0];
        }

        WORK_condition.vel_flag = 1;
        
        if(d_value[0] <= 2500)
        {
          if((d_value[0] - ZERO_RANGE) > PULSE_VEL_ZERO) //����
          {
            WORK_condition.state = 0;
            DriveDirection(DIR_BACK); 
            motor_vel = (uint32_t)(RATIO_VEL*SPEED_MAX*((double)(d_value[0] - 
                                                                 PULSE_VEL_ZERO - ZERO_RANGE)/PULSE_VEL));//�ٶȼ��㣬�����Ƶ�ʼ���
            if(motor_vel>(RATIO_VEL*SPEED_MAX))
            {
              motor_vel = RATIO_VEL*SPEED_MAX;
            }
            
          }else if((d_value[0] + ZERO_RANGE) < PULSE_VEL_ZERO) //ǰ��
          {
            WORK_condition.state = 0;
            DriveDirection(DIR_FORWARD);
            motor_vel = (uint32_t)(SPEED_MAX*((double)(PULSE_VEL_ZERO - 
                                                       d_value[0] - ZERO_RANGE)/PULSE_VEL));//�ٶȼ��㣬�����Ƶ�ʼ���  
            if(motor_vel>(SPEED_MAX))
            {
              motor_vel = SPEED_MAX;
            }
          }else            //ֹͣ
          {     
            motor_vel = 0;          
            WORK_condition.vel_flag = 0;
          } 
        }
        TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Rising); //����������
        
      }
      
  }else if(TIM_GetITStatus(TIM3,TIM_IT_CC2) != RESET)  //����жϷ������
  { 
        
    WORK_condition.online_flag = 1; 
    
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);      //����жϱ�־ 
      
      if(0 == captrueFlag[1])
      {
        captrueFlag[1] = 1;
        captrue_value1[1] = TIM_GetCapture2(TIM3);
        TIM_OC2PolarityConfig(TIM3,TIM_ICPolarity_Falling); //�����½���
      }else
      {
        captrueFlag[1] = 0;
        captrue_value2[1] = TIM_GetCapture2(TIM3);
        
        /*--------------�����ֵ-----------------*/
        if(captrue_value2[1] > captrue_value1[1])
        {
          d_value[1] = captrue_value2[1] - captrue_value1[1];
        }else
        {
          d_value[1] = (0xffff -captrue_value1[1]) + captrue_value2[1];
        }
        
   //     printf("����:%d %d\n",d_value[0],d_value[1]);
        double dir_angle = 0;
        
        if(d_value[1] <= 2500)
        {
            
          if(1 == WORK_condition.state)
          {         
            
            if((d_value[1] - ZERO_RANGE) > PULSE_VEL_ZERO) //˳ʱ����ת
            {
              
              motor_vel = (uint32_t)(RATIO_VEL*SPEED_MAX*((double)(d_value[1] - PULSE_VEL_ZERO - ZERO_RANGE)/PULSE_VEL));//�ٶȼ��㣬�����Ƶ�ʼ���

              if(motor_vel>(RATIO_VEL*SPEED_MAX))
              {
                motor_vel = RATIO_VEL*SPEED_MAX;
              }
                
              MOTOR_control.rightSpeed  = MOTOR_control.leftSpeed = motor_vel;            
   
           //   SetCarSpeed(0,3,motor_vel);
              
              DriveDirection(DIR_RIGHT);         
              
            }else if((d_value[1] + ZERO_RANGE) < PULSE_VEL_ZERO) //��ʱ����ת
            {
              
              motor_vel = (uint32_t)(RATIO_VEL*SPEED_MAX*((double)(PULSE_VEL_ZERO - d_value[1] - ZERO_RANGE)/PULSE_VEL));//�ٶȼ��㣬�����Ƶ�ʼ���                      
              
              if(motor_vel>(RATIO_VEL*SPEED_MAX))
              {
                motor_vel = RATIO_VEL*SPEED_MAX;
              }
              
               MOTOR_control.rightSpeed  = MOTOR_control.leftSpeed = motor_vel;
           //   SetCarSpeed(0,4,motor_vel);            
              DriveDirection(DIR_LEFT);
              
            }else            //ֹͣ
            {
              MOTOR_control.leftSpeed  = MOTOR_control.rightSpeed = motor_vel = 0;         
            }               
            
         //    printf(" �ٶ�1-%d ",motor_vel);
            
          }else
          {
            
            if(motor_vel !=0)
            {
              if((d_value[1] - ZERO_RANGE) > PULSE_DIR_ZERO)      //��ת
              {
                
                dir_angle = (DIR_ANGLE_MAX*((double)(d_value[1] - PULSE_DIR_ZERO - ZERO_RANGE)/PULSE_DIR))*PI/180;               
                SetCarSpeed(dir_angle,TURN_RIGHT,motor_vel);
                
              }else if((d_value[1] + ZERO_RANGE) < PULSE_DIR_ZERO) //��ת
              {              
                dir_angle = (DIR_ANGLE_MAX*((double)(PULSE_DIR_ZERO -d_value[1]-ZERO_RANGE)/PULSE_DIR))*PI/180;  
                SetCarSpeed(dir_angle,TURN_LEFT,motor_vel);               
                
              }else
              {                         
                SetCarSpeed(0,RUN_NOAMAL ,motor_vel);                                     
              }   
            }else
            {
              
              SetCarSpeed(0,RUN_NOAMAL ,motor_vel); 
              
              WORK_condition.angle_flag = 1;
              
              if(d_value[1] > PULSE_DIR_ZERO)
              {
                if((d_value[1] - PULSE_DIR_ZERO) < ZERO_RANGE)
                {
                  WORK_condition.angle_flag = 0;
                }
                
              }else
              {
                if((PULSE_DIR_ZERO - d_value[1]) < ZERO_RANGE)
                {
                  WORK_condition.angle_flag = 0;
                }
              }        
            }
            
          }
        }
        TIM_OC2PolarityConfig(TIM3,TIM_ICPolarity_Rising); //����������
        
      }
    
  }else if(TIM_GetITStatus(TIM3,TIM_IT_CC3) != RESET)  //����жϷ������
  { 
    
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);      //����жϱ�־ 
    
    
      if(0 == captrueFlag[2])
      {
        captrueFlag[2] = 1;
        captrue_value1[2] = TIM_GetCapture3(TIM3);
        TIM_OC3PolarityConfig(TIM3,TIM_ICPolarity_Falling); //�����½���
      }else
      {
        captrueFlag[2] = 0;
        captrue_value2[2] = TIM_GetCapture3(TIM3);
        
        /*--------------�����ֵ-----------------*/
        if(captrue_value2[2] > captrue_value1[2])
        {
          d_value[2] = captrue_value2[2] - captrue_value1[2];
        }else
        {
          d_value[2] = (0xffff -captrue_value1[2]) + captrue_value2[2];
        } 
      
        if(d_value[2] < 2500)
        {
                        
       //   printf("B:%d %d\n",d_value[2],switch_flag);
          if(d_value[2] > PULSE3_ZERO)
          {
            
            if(0 == switch_flag)
            {
              
              switch_flag = 1;
              StateTransformation();            
            }       
            
          }else
          {
            if(1 == switch_flag)
            {
            
              switch_flag = 0;
              StateTransformation();
              
            }
          }
        }
        
        TIM_OC3PolarityConfig(TIM3,TIM_ICPolarity_Rising); //����������
        
      }
    
  }
}


/*
****************************�û��������****************************************
*/


/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/


/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}


/**
  * @brief  EXTI1_IRQHandler
  *         This function handles External line 1 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI1_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line1) != RESET)
  {
  
      EXTI_ClearITPendingBit(EXTI_Line1);
  }
}
/**
  * @brief  OTG_FS_IRQHandler
  *          This function handles USB-On-The-Go FS global interrupt request.
  *          requests.
  * @param  None
  * @retval None
  */
#ifdef USE_USB_OTG_FS
void OTG_FS_IRQHandler(void)
#else
void OTG_HS_IRQHandler(void)
#endif
{

}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/



/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/


