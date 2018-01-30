/*
********************************************************************************
                                UART2����ͨ��

�ļ���    ��bsp_uart2.c
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
#include "bsp_uart2.h"

/*
********************************************************************************
                               ��������      
********************************************************************************
*/
static void UART2_RCC_Configuration(void);
static void UART2_GPIO_Configuration(void);
static void UART2_UART_Configuration(void);
static void UART2_NVIC_Configuration(void);

/*
********************************************************************************
                                void UART2_Init(void)

������     UART2��ʼ��
������     ��
����ֵ��   ��
********************************************************************************
*/
void UART2_Init(void){
  
  UART2_RCC_Configuration();
  UART2_GPIO_Configuration();
  UART2_UART_Configuration();
  UART2_NVIC_Configuration();
  
}

/*
********************************************************************************
                        UART2_sendData (uint8_t length,uint8_t data[])

������     UART2��������
������     length ���ݳ��ȣ�data��Ҫ���͵�����
����ֵ��   �ű�־��1��ʾ�ɹ���0xee��ʾʧ��
********************************************************************************
*/
uint8_t UART2_sendData (uint8_t length,uint8_t data[])
{
  if(length >= 1)
  {

    for(uint8_t i=0;i<length;i++)
    {
      USART_SendData(USART2,data[i]);
      while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){}
    }
    return 1;
  }
  return 0xee;
}

/*
********************************************************************************
                                UART_Configuration(void)

������     UART����
������     ��
����ֵ��   ��
********************************************************************************
*/
static void UART2_UART_Configuration(void){
  
  USART_InitTypeDef USART_InitStructure;
  //UART2����ͨ������
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(USART2, &USART_InitStructure);

  //USART2���ʹ��
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);    
  USART_Cmd(USART2, ENABLE);
}

/*
********************************************************************************
                                void RCC_Configuration(void)

������     RCC����
������     ��
����ֵ��   ��
********************************************************************************
*/
static void UART2_RCC_Configuration(void){  
 //����UART2ʱ�ӣ�ʹ��AFIO���ܵ�ʱ�ӡ�
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 
}
/*
********************************************************************************
                                void GPIO_Configuration(void)

������     GPIO����
������     ��
����ֵ��   ��
********************************************************************************
*/

static void UART2_GPIO_Configuration(void){
  GPIO_InitTypeDef  GPIO_InitStructure;
 
  //��ӳ����
  GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE); 
  //UART_TX���ţ��������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  //UART_RX���ţ�
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}
/*
********************************************************************************
                                void NVIC_Configuration(void)

������     UART2�ж�����
������     ��
����ֵ��   ��
********************************************************************************
*/
static void UART2_NVIC_Configuration(void){
  
  NVIC_InitTypeDef NVIC_InitStructure;

  /* �ж����ȼ�*/  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* �ⲿ�ж� */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}






