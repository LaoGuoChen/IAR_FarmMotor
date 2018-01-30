/*
********************************************************************************
                                UART1����ͨ��

�ļ���    ��bsp_uart1.c
�汾      ��
����Ա    ��TK_CG
********************************************************************************
˵��������Tank��ͨ�����˿��ư�
********************************************************************************
*/


/*
********************************************************************************
                               ͷ�ļ�        
********************************************************************************
*/
#include "bsp_uart1.h"

/*
********************************************************************************
                               ��������      
********************************************************************************
*/
static void UART1_RCC_Configuration(void);
static void UART1_GPIO_Configuration(void);
static void UART1_UART_Configuration(void);
static void UART1_NVIC_Configuration(void);

/*
********************************************************************************
                                void UART1_Init(void)

������     UART1��ʼ��
������     ��
����ֵ��   ��
********************************************************************************
*/
void UART1_Init(void){
  
  UART1_RCC_Configuration();
  UART1_GPIO_Configuration();
  UART1_NVIC_Configuration();
  UART1_UART_Configuration();
  
}


/*
********************************************************************************
                        UART1_sendData (uint8_t length,uint8_t data[])

������     UART1��������
������     length ���ݳ��ȣ�data��Ҫ���͵�����
����ֵ��   �ű�־��1��ʾ�ɹ���0xee��ʾʧ��
********************************************************************************
*/
uint8_t UART1_sendData (uint8_t length,uint8_t data[])
{
  if(length >= 1)
  {

    for(uint8_t i=0;i<length;i++)
    {
      USART_SendData(USART1,data[i]);
      while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){}
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
static void UART1_UART_Configuration(void){
  
  USART_InitTypeDef USART_InitStructure;
  USART_DeInit(USART1);
  //UART1����ͨ������
  USART_InitStructure.USART_BaudRate = 38400;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  
  USART_Init(USART1, &USART_InitStructure);

  //USART1���ʹ��
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);    
  
  USART_Cmd(USART1, ENABLE);
}

/*
********************************************************************************
                                void RCC_Configuration(void)

������     RCC����
������     ��
����ֵ��   ��
********************************************************************************
*/
static void UART1_RCC_Configuration(void){  
 //����UART1ʱ�ӣ�ʹ��AFIO���ܵ�ʱ�ӡ�
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1,ENABLE);
                      
}
/*
********************************************************************************
                                void GPIO_Configuration(void)

������     GPIO����
������     ��
����ֵ��   ��
********************************************************************************
*/

static void UART1_GPIO_Configuration(void){
  
  GPIO_InitTypeDef  GPIO_InitStructure;
 

  //UART_TX���ţ��������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  
  //UART_RX���ţ�
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}
/*
********************************************************************************
                                void NVIC_Configuration(void)

������     UART1�ж�����
������     ��
����ֵ��   ��
********************************************************************************
*/
static void UART1_NVIC_Configuration(void){
  
  NVIC_InitTypeDef NVIC_InitStructure;

  /* �ж����ȼ�*/  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* �ⲿ�ж� */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}






