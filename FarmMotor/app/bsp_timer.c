/*
********************************************************************************
                                计时器
文件名：bsp_timer.c
版本：
程序员：TK_CG
********************************************************************************
*/

/*
********************************************************************************
                               头文件        
********************************************************************************
*/

#include "bsp_timer.h"

/*
********************************************************************************
                               宏定义        
********************************************************************************
*/

#define TIMER_ALL_ARR 1000          //50ms
#define TIMER_ALL_PSC 7200         //分频10KHz


/*
********************************************************************************
                               函数申明       
********************************************************************************
*/

static void TIM_RCC_Configuration(void);
static void TIM_Configuration(void);
static void TIM_NVIC_Configuration(void);

/*
********************************************************************************
                        void Timer_Init(void)

描述：     初始化时钟
参数：     无
返回值：   无
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

描述：     TIM时钟
参数：     无
返回值：   无
********************************************************************************
*/
static void TIM_RCC_Configuration(void)
{
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);   
}


/*
********************************************************************************
                          TIM_Configuration()

描述：     TIM配置
参数：     无
返回值：   无
********************************************************************************
*/
static void TIM_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    //定时器初始化
    TIM_TimeBaseStructure.TIM_Period = TIMER_ALL_ARR-1;             //计数    
    TIM_TimeBaseStructure.TIM_Prescaler = TIMER_ALL_PSC-1;          //分频
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;      //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);             
  
}


/*
********************************************************************************
                        TIM_NVIC_Configuration()

描述：     TIM中断配置
参数：     无
返回值：   无
********************************************************************************
*/
static void TIM_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    //中断设置
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         //从优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);  

    TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE ); 
    TIM_Cmd(TIM5, ENABLE);  
}
