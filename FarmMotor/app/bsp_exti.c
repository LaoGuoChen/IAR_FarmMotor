/*
********************************************************************************
                               �ⲿ�ж�
�ļ�����bsp_exti.c
�汾��
����Ա��Tank_CG
********************************************************************************
˵����
********************************************************************************
*/

/*
********************************************************************************
                               ͷ�ļ�       
********************************************************************************
*/

#include "bsp_exti.h"

/*
********************************************************************************
                               ��������       
********************************************************************************
*/

static void EXTI_RCC_Configuration(void);
static void EXTI_Configuration(void);
static void EXTI_GPIO_Configuration(void);
static void EXTI_NVIC_Configuration(void);

/*
********************************************************************************
                        void EXTI_Init(void)

������     ��ʼ�ⲿ�ж�
������     ��
����ֵ��   ��
********************************************************************************
*/

void BSP_EXTI_Init(void)
{
  
  EXTI_Configuration();
  EXTI_GPIO_Configuration();
  EXTI_NVIC_Configuration();
  EXTI_RCC_Configuration();

}


/*
********************************************************************************
                     EXTI_Configuration()

������     
������     ��
����ֵ��   ��
********************************************************************************
*/
static void EXTI_Configuration()
{
  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_DeInit();
  EXTI_InitStructure.EXTI_Line = EXTI_Line13;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  

}


/*
********************************************************************************
                       EXTI_GPIO_Configuration()

������     GPIO����
������     ��
����ֵ��   ��
********************************************************************************
*/
static void EXTI_GPIO_Configuration()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_SetBits(GPIOC,GPIO_Pin_3);
  
  //�ܽ�ӳ�䵽�ж�
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource3);

}

/*
********************************************************************************
                  void EXTI_NVIC_Configuration(void)

������     �ж�����
������     ��
����ֵ��   ��
********************************************************************************
*/
static void EXTI_NVIC_Configuration(void)
{

  
  NVIC_InitTypeDef NVIC_InitStructure;    
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn; //PPP�ⲿ�ж���
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  
}

/*
********************************************************************************
                   void EXTI_RCC_Configuration(void)

������     RCC����
������     ��
����ֵ��   ��
********************************************************************************
*/
static void EXTI_RCC_Configuration(void)
{
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
}
