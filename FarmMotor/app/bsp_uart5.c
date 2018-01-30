/*
********************************************************************************
                                UART����ͨ��

�ļ���    ��bsp_uart5.c
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
#include "bsp_uart5.h"

/*
********************************************************************************
                               ��������      
********************************************************************************
*/
static void UART5_RCC_Configuration(void);
static void UART5_GPIO_Configuration(void);
static void UART5_UART_Configuration(void);
static void UART5_NVIC_Configuration(void);




/*
********************************************************************************
                                void UART5_Init(void)

������     UART5��ʼ��
������     ��
����ֵ��   ��
********************************************************************************
*/
void UART5_Init(void){
  
  UART5_RCC_Configuration();
  UART5_GPIO_Configuration();
  UART5_UART_Configuration();
  UART5_NVIC_Configuration();
  
}

/*
********************************************************************************
                        UART5_sendData (uint8_t_t length,uint8_t_t data[])

������     UART5��������
������     length ���ݳ��ȣ�data��Ҫ���͵�����
����ֵ��   �ű�־��1��ʾ�ɹ���0xee��ʾʧ��
********************************************************************************
*/
uint8_t UART5_sendData (uint8_t length,uint8_t data[])
{
  if(length >= 1)
	{

		uint8_t i;
    for(i=0;i<length;i++)
    {
      USART_SendData(UART5,data[i]);
      while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET){}
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
static void UART5_UART_Configuration(void){
  
  USART_InitTypeDef USART_InitStructure;
  //UART5����ͨ������
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  
  USART_Init(UART5, &USART_InitStructure);

  //���ʹ��
  USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);    
  USART_Cmd(UART5, ENABLE);
}

/*
********************************************************************************
                                void RCC_Configuration(void)

������     RCC����
������     ��
����ֵ��   ��
********************************************************************************
*/
static void UART5_RCC_Configuration(void){  
 //����UART5ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC,ENABLE);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); 
}
/*
********************************************************************************
                                void GPIO_Configuration(void)

������     GPIO����
������     ��
����ֵ��   ��
********************************************************************************
*/

static void UART5_GPIO_Configuration(void){
  GPIO_InitTypeDef  GPIO_InitStructure;
 
  //��ӳ����
//  GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE); 
  
  //UART_TX���ţ��������
  GPIO_InitStructure.GPIO_Pin = BSP_UART5_TX; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  //UART_RX���ţ�
  GPIO_InitStructure.GPIO_Pin = BSP_UART5_RX; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}
/*
********************************************************************************
                       void NVIC_Configuration(void)

������     UART�ж�����
������     ��
����ֵ��   ��
********************************************************************************
*/
static void UART5_NVIC_Configuration(void){
  
  NVIC_InitTypeDef NVIC_InitStructure;

  /* �ж����ȼ�*/  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* �ⲿ�ж� */
  NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}






