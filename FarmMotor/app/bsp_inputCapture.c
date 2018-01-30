/*
********************************************************************************
                                ���ݴ洢
�ļ�����bsp_inputCapture.c
�汾��
����Ա��TK_CG
********************************************************************************
˵��������STM32F107����ʱ�����벶�񣬲���������ģ�鷵��ֵ
@ ���ö�ʱ�������Ȳ��������أ�Ȼ���л��������½��أ��Ӷ���������С
@ ʹ�ù̼�����ͨ������ÿ��ͨ����������
********************************************************************************
*/

/*
********************************************************************************
                               ͷ�ļ�       
********************************************************************************
*/
#include "bsp_inputCapture.h"


/*
********************************************************************************
                               �궨��       
********************************************************************************
*/
#define INPUT_TIMER_ALL_PSC    72    //��Ƶ 1MHz

/*
********************************************************************************
                               ��������       
********************************************************************************
*/
static void INPUT_RCC_Configuration     (void);
static void INPUT_GPIO_Configuration    (void);
static void INPUT_NVIC_Configuration    (void);
static void INPUT_TIM_Configuration     (void);


/*
********************************************************************************
                        void BSP_InputCatrueInit(void)

���������벶���ʼ��
��������
����ֵ����
********************************************************************************
*/
void BSP_InputCatrueInit(void)
{
  INPUT_RCC_Configuration();
  INPUT_GPIO_Configuration();
  INPUT_NVIC_Configuration();
  INPUT_TIM_Configuration();
  
}


/*
********************************************************************************
                          void TIM_Configuration(void)

��������ʱ������
��������
����ֵ����
********************************************************************************
*/

static void INPUT_TIM_Configuration(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  //��ʱ��3 
  TIM_DeInit(TIM3);   
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;                    //����    
  TIM_TimeBaseStructure.TIM_Prescaler = INPUT_TIMER_ALL_PSC-1;  //��Ƶ
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;      //����ʱ�ӷָ�:TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 
  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;       //������
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;   
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0; 
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;       //
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;   
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0; 
  TIM_ICInit(TIM3, &TIM_ICInitStructure);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;       //
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;   
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0; 
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
  
  TIM_Cmd(TIM3, ENABLE);
 // TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3, ENABLE);
  
}


/*
********************************************************************************
                            void INPUT_RCC_Configuration(void)

������������Ҫʹ�õ�ʱ��
��������
����ֵ����
********************************************************************************
*/
static void INPUT_RCC_Configuration(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
}

/*
********************************************************************************
                            void INPUT_RCC_Configuration(void)

�������ܽ�����
��������
����ֵ����
********************************************************************************
*/
static void INPUT_GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM3,ENABLE);   //��ȫ��ӳ��   
  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/*
********************************************************************************
                            void INPUT_RCC_Configuration(void)

�������ж�����
��������
����ֵ����
********************************************************************************
*/
static void INPUT_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}
