/*
********************************************************************************
                                UART3����ͨ��

�ļ���    ��bsp_uart3.c
�汾      ��
����Ա    ��Tank_CG
********************************************************************************
˵����
********************************************************************************
*/


/*
********************************************************************************
                               ͷ�ļ�        
********************************************************************************
*/
#include "bsp_uart3.h"

/*
********************************************************************************
                               ��������      
********************************************************************************
*/
static void UART3_RCC_Configuration(void);
static void UART3_GPIO_Configuration(void);
static void UART3_UART_Configuration(void);
static void UART3_NVIC_Configuration(void);

/*
********************************************************************************
                                void UART1_Init(void)

������     UART1��ʼ��
������     ��
����ֵ��   ��
********************************************************************************
*/
void UART3_Init(void){
  
  UART3_RCC_Configuration();
  UART3_GPIO_Configuration();
  UART3_NVIC_Configuration();
  UART3_UART_Configuration();
  
}


/*
********************************************************************************
                        UART3_sendData (uint8_t length,uint8_t data[])

������     UART3��������
������     length ���ݳ��ȣ�data��Ҫ���͵�����
����ֵ��   �ű�־��1��ʾ�ɹ���0xee��ʾʧ��
********************************************************************************
*/
uint8_t UART3_sendData (uint8_t length,uint8_t data[])
{
  if(length >= 1)
  {

    for(uint8_t i=0;i<length;i++)
    {
      USART_SendData(USART3,*data++);
      while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET){}
    }
    return 1;
  }
  return 0xee;
}

/*
********************************************************************************
                                UART_Configuration(void)

������     UART1����
������     ��
����ֵ��   ��
********************************************************************************
*/
static void UART3_UART_Configuration(void){
  
  USART_InitTypeDef USART_InitStructure;
  USART_DeInit(USART3);
  //UART1����ͨ������
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  
  USART_Init(USART3, &USART_InitStructure);

  //USART���ʹ��
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);    
  
  USART_Cmd(USART3, ENABLE);
}

/*
********************************************************************************
                                void RCC_Configuration(void)

������     RCC����
������     ��
����ֵ��   ��
********************************************************************************
*/
static void UART3_RCC_Configuration(void){  
 //����UARTʱ�ӣ�ʹ��AFIO���ܵ�ʱ�ӡ�
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); 
                      
}
/*
********************************************************************************
                                void GPIO_Configuration(void)

������     GPIO����
������     ��
����ֵ��   ��
********************************************************************************
*/

static void UART3_GPIO_Configuration(void){
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  GPIO_PinRemapConfig(GPIO_FullRemap_USART3, ENABLE);
  
 // GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
  //UART_TX���ţ��������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  
  //UART_RX���ţ�
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}
/*
********************************************************************************
                                void NVIC_Configuration(void)

������     UART3�ж�����
������     ��
����ֵ��   ��
********************************************************************************
*/
static void UART3_NVIC_Configuration(void){
  
  NVIC_InitTypeDef NVIC_InitStructure;

  /* �ж����ȼ�*/  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* �ⲿ�ж� */
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}






