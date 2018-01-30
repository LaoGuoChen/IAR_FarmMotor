/*
********************************************************************************
                                数据存储
文件名：bsp_inputCapture.c
版本：
程序员：TK_CG
********************************************************************************
说明：基于STM32F107，定时器输入捕获，测量超声波模块返回值
@ 采用定时器捕获，先捕获上升沿，然后切换到捕获下降沿，从而获得脉宽大小
@ 使用固件开启通道必须每个通道单独配置
********************************************************************************
*/

/*
********************************************************************************
                               头文件       
********************************************************************************
*/
#include "bsp_inputCapture.h"


/*
********************************************************************************
                               宏定义       
********************************************************************************
*/
#define INPUT_TIMER_ALL_PSC    72    //分频 1MHz

/*
********************************************************************************
                               函数申明       
********************************************************************************
*/
static void INPUT_RCC_Configuration     (void);
static void INPUT_GPIO_Configuration    (void);
static void INPUT_NVIC_Configuration    (void);
static void INPUT_TIM_Configuration     (void);


/*
********************************************************************************
                        void BSP_InputCatrueInit(void)

描述：输入捕获初始化
参数：无
返回值：无
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

描述：定时器配置
参数：无
返回值：无
********************************************************************************
*/

static void INPUT_TIM_Configuration(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  //定时器3 
  TIM_DeInit(TIM3);   
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;                    //计数    
  TIM_TimeBaseStructure.TIM_Prescaler = INPUT_TIMER_ALL_PSC-1;  //分频
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;      //设置时钟分割:TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 
  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;       //上升沿
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

描述：配置需要使用的时钟
参数：无
返回值：无
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

描述：管脚配置
参数：无
返回值：无
********************************************************************************
*/
static void INPUT_GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM3,ENABLE);   //完全重映射   
  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/*
********************************************************************************
                            void INPUT_RCC_Configuration(void)

描述：中断配置
参数：无
返回值：无
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
