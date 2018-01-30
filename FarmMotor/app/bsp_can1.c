/*
********************************************************************************
                                UART2串口通信

文件名    ：bsp_can1.c
版本      ：
程序员    ：
********************************************************************************
*/


/*
********************************************************************************
                               包含的头文件        
********************************************************************************
*/
#include "bsp_can1.h"



/*
********************************************************************************
                               函数申明      
********************************************************************************
*/
void CAN1_RCC_Configuration(void);
void CAN1_GPIO_Configuration(void);
void CAN1_NVIC_Configuration(void);
void CAN1_Configuration(void);




/*
********************************************************************************
                     void BSP_CAN1_Init(void)

描述：     CAN1初始化
参数：     无
返回值：   无
********************************************************************************
*/
void BSP_CAN1_Init(void){
  
  CAN1_RCC_Configuration();
  CAN1_GPIO_Configuration();
  CAN1_Configuration();
  CAN1_NVIC_Configuration();
  
}
/*
********************************************************************************
                              void CAN1_RCC_Configuration(void);  

描述：     时钟配置
参数：     无
返回值：   无
********************************************************************************
*/
void CAN1_RCC_Configuration(void){
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);
}
/*
********************************************************************************
                              void CAN1_GPIO_Configuration(void);
  

描述：     管脚配置
参数：     无
返回值：   无
********************************************************************************
*/
void CAN1_GPIO_Configuration(void){
  GPIO_InitTypeDef GPIO_InitStructure;

  
   //端口功能复用
  GPIO_PinRemapConfig(GPIO_Remap2_CAN1,ENABLE);
  
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
  GPIO_Init(GPIOD,&GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;//TX
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
  GPIO_Init(GPIOD,&GPIO_InitStructure);
  

}
/*
********************************************************************************
                             void CAN1_NVIC_Configuration(void)   

描述：     中断配置
参数：     无
返回值：   无
********************************************************************************
*/

void CAN1_NVIC_Configuration(void){
  NVIC_InitTypeDef NVIC_InitStructure; 

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 

  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn; 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStructure);
}
/*
********************************************************************************
                               void CAN1_Configuration(void) 

描述：     CAN配置
参数：     无
返回值：   无
********************************************************************************
*/
void CAN1_Configuration(void){
    CAN_InitTypeDef CAN_InitStructure;                                
    CAN_FilterInitTypeDef CAN_FilterInitStructure;       
    
    
    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);
    /************************************************************************
                        CAN 配置
    *************************************************************************/
    CAN_InitStructure.CAN_TTCM = DISABLE;                     //禁止时间戳
    CAN_InitStructure.CAN_ABOM = DISABLE;                     //离线模式由软件实现
    CAN_InitStructure.CAN_AWUM = DISABLE;                     //软件唤醒
    CAN_InitStructure.CAN_TTCM = DISABLE;                     //禁止时间触发通信模式
    CAN_InitStructure.CAN_NART = DISABLE;                     //禁止自动重传
    CAN_InitStructure.CAN_TXFP = DISABLE;                     //优先级由报文的标识符来决定
    CAN_InitStructure.CAN_RFLM = DISABLE;                     //接受溢出时FIFO不锁定，下一个收到的报文覆盖原有报文     
    
  //  CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack;           //CAN硬件工作环回模式
    
   CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;           //CAN硬件工作模式
    //波特率设置
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;                  //重新同步跳跃宽度为2个时间单位
    CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;                  //时间段为6个时间单位
    CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;                  //时间段为5个时间单位
    CAN_InitStructure.CAN_Prescaler = 6;                      //(pclk1/((1+6+5)*6)) = 36Mhz/（12*6） = 0.5Mbits设定了一个时间单位的长度12  
    CAN_Init(CAN1, &CAN_InitStructure);
    
    /************************************************************************
                        过滤器配置，列表模式
    *************************************************************************/
    CAN_FilterInitStructure.CAN_FilterNumber=0;                                 //设置初始化过滤组编号为0
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;             //设定过滤器组为屏蔽位模式
//	  CAN_FilterInitStructure.CAN_FilterMode =  CAN_FilterMode_IdMask; 
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;            //过滤器位宽为16位过滤器一个
    CAN_FilterInitStructure.CAN_FilterIdHigh = USER_STDID_ADR1<<5;               //设定过滤器16位为第一个
    CAN_FilterInitStructure.CAN_FilterIdLow =  USER_STDID_ADR2<<5;               //设定过滤器16位为第二个
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh= USER_STDID_ADR3<<5;            //设定过滤器16位为第三个
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=  USER_STDID_ADR4<<5;            //设定过滤器16位为第四个
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;        //过滤器FIFO0指向过滤器0
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;                      //使能过滤器
    CAN_FilterInit(&CAN_FilterInitStructure);  
    
    CAN_FilterInitStructure.CAN_FilterNumber=1;                                 //设置初始化过滤组编号为0
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;             //设定过滤器组为屏蔽位模式
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;            //过滤器位宽为16位过滤器一个
    CAN_FilterInitStructure.CAN_FilterIdHigh = USER_STDID_ADR5<<5;               //设定过滤器16位为第一个
    CAN_FilterInitStructure.CAN_FilterIdLow =  USER_STDID_ADR6<<5;               //设定过滤器16位为第二个
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh= USER_STDID_ADR7<<5;            //设定过滤器16位为第三个
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=  USER_STDID_ADR8<<5;            //设定过滤器16位为第四个
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;        //过滤器FIFO0指向过滤器0
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;                      //使能过滤器
    CAN_FilterInit(&CAN_FilterInitStructure);  

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE); //FIF0消息挂号中断允许
    
    
}

