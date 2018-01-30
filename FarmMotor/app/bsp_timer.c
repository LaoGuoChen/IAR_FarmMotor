/*
********************************************************************************
                                ��ʱ��
�ļ�����bsp_timer.c
�汾��
����Ա��TK_CG
********************************************************************************
*/

/*
********************************************************************************
                               ͷ�ļ�        
********************************************************************************
*/

#include "bsp_timer.h"

/*
********************************************************************************
                               �궨��        
********************************************************************************
*/

#define TIMER_ALL_ARR 1000          //50ms
#define TIMER_ALL_PSC 7200         //��Ƶ10KHz


/*
********************************************************************************
                               ��������       
********************************************************************************
*/

static void TIM_RCC_Configuration(void);
static void TIM_Configuration(void);
static void TIM_NVIC_Configuration(void);

/*
********************************************************************************
                        void Timer_Init(void)

������     ��ʼ��ʱ��
������     ��
����ֵ��   ��
********************************************************************************
*/
void Timer_Init(void)
{
  TIM_RCC_Configuration();
  TIM_Configuration();
  TIM_NVIC_Configuration();

}

/*
********************************************************************************
                            TIM_RCC_Configuration(void)

������     TIMʱ��
������     ��
����ֵ��   ��
********************************************************************************
*/
static void TIM_RCC_Configuration(void)
{
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);   
}


/*
********************************************************************************
                          TIM_Configuration()

������     TIM����
������     ��
����ֵ��   ��
********************************************************************************
*/
static void TIM_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    //��ʱ����ʼ��
    TIM_TimeBaseStructure.TIM_Period = TIMER_ALL_ARR-1;             //����    
    TIM_TimeBaseStructure.TIM_Prescaler = TIMER_ALL_PSC-1;          //��Ƶ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;      //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);             
  
}


/*
********************************************************************************
                        TIM_NVIC_Configuration()

������     TIM�ж�����
������     ��
����ֵ��   ��
********************************************************************************
*/
static void TIM_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    //�ж�����
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ����
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         //�����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure);  

    TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE ); 
    TIM_Cmd(TIM5, ENABLE);  
}
