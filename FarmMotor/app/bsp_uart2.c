/*
********************************************************************************
                                UART2串口通信

文件名    ：bsp_uart2.c
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
#include "bsp_uart2.h"

/*
********************************************************************************
                               函数申明      
********************************************************************************
*/
static void UART2_RCC_Configuration(void);
static void UART2_GPIO_Configuration(void);
static void UART2_UART_Configuration(void);
static void UART2_NVIC_Configuration(void);

/*
********************************************************************************
                                void UART2_Init(void)

描述：     UART2初始化
参数：     无
返回值：   无
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

描述：     UART2发送数据
参数：     length 数据长度，data需要发送的数据
返回值：   放标志，1表示成功，0xee表示失败
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

描述：     UART配置
参数：     无
返回值：   无
********************************************************************************
*/
static void UART2_UART_Configuration(void){
  
  USART_InitTypeDef USART_InitStructure;
  //UART2串口通信配置
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(USART2, &USART_InitStructure);

  //USART2相关使能
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);    
  USART_Cmd(USART2, ENABLE);
}

/*
********************************************************************************
                                void RCC_Configuration(void)

描述：     RCC配置
参数：     无
返回值：   无
********************************************************************************
*/
static void UART2_RCC_Configuration(void){  
 //开启UART2时钟，使能AFIO功能的时钟。
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 
}
/*
********************************************************************************
                                void GPIO_Configuration(void)

描述：     GPIO配置
参数：     无
返回值：   无
********************************************************************************
*/

static void UART2_GPIO_Configuration(void){
  GPIO_InitTypeDef  GPIO_InitStructure;
 
  //重映引脚
  GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE); 
  //UART_TX引脚，推免输出
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  //UART_RX引脚，
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}
/*
********************************************************************************
                                void NVIC_Configuration(void)

描述：     UART2中断配置
参数：     无
返回值：   无
********************************************************************************
*/
static void UART2_NVIC_Configuration(void){
  
  NVIC_InitTypeDef NVIC_InitStructure;

  /* 中断优先级*/  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* 外部中断 */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}






