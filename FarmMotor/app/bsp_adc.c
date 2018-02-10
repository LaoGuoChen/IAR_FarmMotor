/*
********************************************************************************
                                AD����
�ļ�����bsp_adc.c
�汾��
����Ա��Tank_CG
********************************************************************************
*/

/*
********************************************************************************
                               ͷ�ļ�       
********************************************************************************
*/

#include "bsp_adc.h"
#include "app_conf.h"


/*
********************************************************************************
                               ȫ�ֱ���      
********************************************************************************
*/
uint16_t  ADC_ConvertedValue[ADC_DMA_CH*ADC_DMA_LEN]; 
/*
********************************************************************************
                               �ֲ�����      
********************************************************************************
*/

/*
********************************************************************************
                               ��������       
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

������     ��ʼAD����
������     ��
����ֵ��   ��
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

������     ADC��������
������     ��
����ֵ��   ��
********************************************************************************
*/
static void ADC_Configuration(void)
{
  ADC_InitTypeDef ADC_InitStructure;

  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;                   //������ת��ģʽ
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;                         //��ͨ��
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                  //����ģʽ
 // ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                  //
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  //�����������
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;               //�Ҷ���
  ADC_InitStructure.ADC_NbrOfChannel = ADC_DMA_CH;                              //����ͨ������
  ADC_Init(ADC1, &ADC_InitStructure);
  

  ADC_RegularChannelConfig(ADC1, ADC_Channel_1,  1, ADC_SampleTime_239Cycles5);  //�Բ���ʱ��ûҪ��������ڴ������׼
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2,  2, ADC_SampleTime_239Cycles5);  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4,  3, ADC_SampleTime_239Cycles5);  

  
  ADC_DMACmd(ADC1, ENABLE);                      //ʹ�� ADC1 DMA
  ADC_Cmd(ADC1, ENABLE);                         //ʹ�� ADC1 

 
  ADC_ResetCalibration(ADC1);                  //����ADC��׼�Ĵ���
  while(ADC_GetResetCalibrationStatus(ADC1)); //������ã�ֱ�����

  ADC_StartCalibration(ADC1);                  //У׼ADC
  while(ADC_GetCalibrationStatus(ADC1));      //���У׼��֪�����
     
  //ADC_SoftwareStartConvCmd(ADC1, ENABLE);    //���ﲻ�������ڶ�ʱ���ж϶�ʱ����
  
}

/*
********************************************************************************
                        ADC_DMA_Config(void)

������     DMA����
������     ��
����ֵ��   ��
********************************************************************************
*/
static void ADC_DMA_Config(void)
{
  DMA_InitTypeDef DMA_InitStructure;
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;           // ADC���ݼĴ�����ַ,ADCӲ����ַ+ADC�Ĵ�������ƫ�Ƶ�ַ
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_ConvertedValue;  //�ڴ��ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                    //SRC ģʽ��������˵��ڴ�  
  DMA_InitStructure.DMA_BufferSize = ADC_DMA_CH*ADC_DMA_LEN; //DMA�����С
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;      //���ݴ���������ַ����
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;               //���ݴ�����ڴ��ַ�ı�
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;   //�������ݿ��Ϊ16λ
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;           //�ڴ����ݿ��16λ
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                               //ѭ��ģʽ ,ֻ������һ��
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                         //���ȼ� ��
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  
  DMA_Cmd(DMA1_Channel1, ENABLE);                //ʹ�� DMA1 
  DMA_ITConfig(DMA1_Channel1,DMA_IT_TC, ENABLE);//ʹ�ܴ�������ж�  
  

}

/*
********************************************************************************
                       ADC_NVIC_Configuration(void)

������     �ж�����
������     ��
����ֵ��   ��
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

������     GPIO����
������     ��
����ֵ��   ��
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

������     RCC����
������     ��
����ֵ��   ��
********************************************************************************
*/
static void ADC_RCC_Configuration(void)
{
  /*ADC����ʱ�ӣ�72M/6 ADCʱ�Ӳ��ó���14M*/
  RCC_ADCCLKConfig(RCC_PCLK2_Div6); 

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE); 
}

