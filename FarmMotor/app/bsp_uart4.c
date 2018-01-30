/*
********************************************************************************
                                UART4����ͨ��

�ļ���    ��bsp_uart4.c
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
#include "bsp_uart4.h"
/*
********************************************************************************
                               ��������      
********************************************************************************
*/
static void UART4_RCC_Configuration(void);
static void UART4_GPIO_Configuration(void);
static void UART4_UART_Configuration(void);
static void UART4_NVIC_Configuration(void);

/*
********************************************************************************
                                void UART4_Init(void)

������     UART4��ʼ��
������     ��
����ֵ��   ��
********************************************************************************
*/
void UART4_Init(void){
  
  UART4_RCC_Configuration();
  UART4_GPIO_Configuration();
  UART4_NVIC_Configuration();
  UART4_UART_Configuration();
  
}


/*
********************************************************************************
                        UART4_sendData (uint8_t length,uint8_t data[])

������     UART4��������
������     length ���ݳ��ȣ�data��Ҫ���͵�����
����ֵ��   �ű�־��1��ʾ�ɹ���0xee��ʾʧ��
********************************************************************************
*/
uint8_t UART4_sendData (uint8_t length,uint8_t data[])
{
  if(length >= 1)
  {

    for(uint8_t i=0;i<length;i++)
    {
      USART_SendData(UART4,data[i]);
      while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET){}
    }
    return 1;
  }
  return 0xee;
}

/*
********************************************************************************
                                UART4_Configuration(void)

������     UART4����
������     ��
����ֵ��   ��
********************************************************************************
*/
static void UART4_UART_Configuration(void){
  
  USART_InitTypeDef USART_InitStructure;
  USART_DeInit(UART4);
  //UART4����ͨ������
  USART_InitStructure.USART_BaudRate = 38400;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  
  USART_Init(UART4, &USART_InitStructure);

  //USART4���ʹ��
  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);    
  
  USART_Cmd(UART4, ENABLE);
}

/*
********************************************************************************
                                void RCC_Configuration(void)

������     RCC����
������     ��
����ֵ��   ��
********************************************************************************
*/
static void UART4_RCC_Configuration(void){  
 //����UART1ʱ�ӣ�ʹ��AFIO���ܵ�ʱ�ӡ�
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
                      
}
/*
********************************************************************************
                                void GPIO_Configuration(void)

������     GPIO����
������     ��
����ֵ��   ��
********************************************************************************
*/

static void UART4_GPIO_Configuration(void){
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  //UART_TX���ţ��������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  
  //UART_RX���ţ�
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}
/*
********************************************************************************
                                void NVIC_Configuration(void)

������     UART4�ж�����
������     ��
����ֵ��   ��
********************************************************************************
*/
static void UART4_NVIC_Configuration(void){
  
  NVIC_InitTypeDef NVIC_InitStructure;

  /* �ж����ȼ�*/  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* �ⲿ�ж� */
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}






