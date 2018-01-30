/*
********************************************************************************
                                UART4串口通信

文件名    ：bsp_uart4.c
版本      ：
程序员    ：TK_CG
********************************************************************************
说明：基于Tank交通机器人控制板
********************************************************************************
*/


/*
********************************************************************************
                               头文件        
********************************************************************************
*/
#include "bsp_uart4.h"
/*
********************************************************************************
                               函数申明      
********************************************************************************
*/
static void UART4_RCC_Configuration(void);
static void UART4_GPIO_Configuration(void);
static void UART4_UART_Configuration(void);
static void UART4_NVIC_Configuration(void);

/*
********************************************************************************
                                void UART4_Init(void)

描述：     UART4初始化
参数：     无
返回值：   无
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

描述：     UART4发送数据
参数：     length 数据长度，data需要发送的数据
返回值：   放标志，1表示成功，0xee表示失败
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

描述：     UART4配置
参数：     无
返回值：   无
********************************************************************************
*/
static void UART4_UART_Configuration(void){
  
  USART_InitTypeDef USART_InitStructure;
  USART_DeInit(UART4);
  //UART4串口通信配置
  USART_InitStructure.USART_BaudRate = 38400;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  
  USART_Init(UART4, &USART_InitStructure);

  //USART4相关使能
  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);    
  
  USART_Cmd(UART4, ENABLE);
}

/*
********************************************************************************
                                void RCC_Configuration(void)

描述：     RCC配置
参数：     无
返回值：   无
********************************************************************************
*/
static void UART4_RCC_Configuration(void){  
 //开启UART1时钟，使能AFIO功能的时钟。
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
                      
}
/*
********************************************************************************
                                void GPIO_Configuration(void)

描述：     GPIO配置
参数：     无
返回值：   无
********************************************************************************
*/

static void UART4_GPIO_Configuration(void){
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  //UART_TX引脚，推免输出
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  
  //UART_RX引脚，
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}
/*
********************************************************************************
                                void NVIC_Configuration(void)

描述：     UART4中断配置
参数：     无
返回值：   无
********************************************************************************
*/
static void UART4_NVIC_Configuration(void){
  
  NVIC_InitTypeDef NVIC_InitStructure;

  /* 中断优先级*/  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* 外部中断 */
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}






