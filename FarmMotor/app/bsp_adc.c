/*
********************************************************************************
                                AD采样
文件名：bsp_adc.c
版本：
程序员：Tank_CG
********************************************************************************
*/

/*
********************************************************************************
                               头文件       
********************************************************************************
*/

#include "bsp_adc.h"
#include "app_conf.h"


/*
********************************************************************************
                               全局变量      
********************************************************************************
*/
uint16_t  ADC_ConvertedValue[ADC_DMA_CH*ADC_DMA_LEN]; 
/*
********************************************************************************
                               局部变量      
********************************************************************************
*/

/*
********************************************************************************
                               函数申明       
********************************************************************************
*/

static void ADC_RCC_Configuration(void);
static void ADC_Configuration(void);
static void ADC_GPIO_Configuration(void);
static void ADC_NVIC_Configuration(void);
static void ADC_DMA_Config(void);  



/*
********************************************************************************
                        void ADC_Init(void)

描述：     初始AD采样
参数：     无
返回值：   无
********************************************************************************
*/

void BSP_ADCInit(void)
{
  ADC_RCC_Configuration();
  ADC_GPIO_Configuration();
  ADC_Configuration();
  ADC_DMA_Config();
  ADC_NVIC_Configuration();

}



/*
********************************************************************************
                        ADC_RCC_Configuration(void)

描述：     ADC采样配置
参数：     无
返回值：   无
********************************************************************************
*/
static void ADC_Configuration(void)
{
  ADC_InitTypeDef ADC_InitStructure;

  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;                   //独立的转换模式
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;                         //多通道
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                  //单次模式
 // ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                  //
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  //由软件来触发
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;               //右对齐
  ADC_InitStructure.ADC_NbrOfChannel = ADC_DMA_CH;                              //开启通道个数
  ADC_Init(ADC1, &ADC_InitStructure);
  

  ADC_RegularChannelConfig(ADC1, ADC_Channel_1,  1, ADC_SampleTime_239Cycles5);  //对采样时间没要求，设计周期大采样精准
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2,  2, ADC_SampleTime_239Cycles5);  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4,  3, ADC_SampleTime_239Cycles5);  

  
  ADC_DMACmd(ADC1, ENABLE);                      //使能 ADC1 DMA
  ADC_Cmd(ADC1, ENABLE);                         //使能 ADC1 

 
  ADC_ResetCalibration(ADC1);                  //重置ADC标准寄存器
  while(ADC_GetResetCalibrationStatus(ADC1)); //检查重置，直到完成

  ADC_StartCalibration(ADC1);                  //校准ADC
  while(ADC_GetCalibrationStatus(ADC1));      //检查校准，知道完成
     
  //ADC_SoftwareStartConvCmd(ADC1, ENABLE);    //这里不启动，在定时器中断定时启动
  
}

/*
********************************************************************************
                        ADC_DMA_Config(void)

描述：     DMA配置
参数：     无
返回值：   无
********************************************************************************
*/
static void ADC_DMA_Config(void)
{
  DMA_InitTypeDef DMA_InitStructure;
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;           // ADC数据寄存器地址,ADC硬件地址+ADC寄存器数据偏移地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_ConvertedValue;  //内存地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                    //SRC 模式从外设搬运到内存  
  DMA_InitStructure.DMA_BufferSize = ADC_DMA_CH*ADC_DMA_LEN; //DMA缓存大小
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;      //数据传输后外设地址不变
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;               //数据传输后内存地址改变
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;   //外设数据宽度为16位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;           //内存数据宽度16位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                               //循环模式 ,只需启动一次
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                         //优先级 中
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  
  DMA_Cmd(DMA1_Channel1, ENABLE);                //使能 DMA1 
  DMA_ITConfig(DMA1_Channel1,DMA_IT_TC, ENABLE);//使能传输完成中断  
  

}

/*
********************************************************************************
                       ADC_NVIC_Configuration(void)

描述：     中断配置
参数：     无
返回值：   无
********************************************************************************
*/
static void ADC_NVIC_Configuration(void)
{
   NVIC_InitTypeDef NVIC_InitStructure;    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);  
    NVIC_InitStructure.NVIC_IRQChannel =DMA1_Channel1_IRQn;    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;   
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    NVIC_Init(&NVIC_InitStructure); 
}


/*
********************************************************************************
                        GPIO_Configuration(void)

描述：     GPIO配置
参数：     无
返回值：   无
********************************************************************************
*/
static void ADC_GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*
********************************************************************************
                        ADC_RCC_Configuration(void)

描述：     RCC配置
参数：     无
返回值：   无
********************************************************************************
*/
static void ADC_RCC_Configuration(void)
{
  /*ADC采样时钟，72M/6 ADC时钟不得超过14M*/
  RCC_ADCCLKConfig(RCC_PCLK2_Div6); 

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE); 
}

