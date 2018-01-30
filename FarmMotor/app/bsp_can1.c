/*
********************************************************************************
                                UART2����ͨ��

�ļ���    ��bsp_can1.c
�汾      ��
����Ա    ��
********************************************************************************
*/


/*
********************************************************************************
                               ������ͷ�ļ�        
********************************************************************************
*/
#include "bsp_can1.h"



/*
********************************************************************************
                               ��������      
********************************************************************************
*/
void CAN1_RCC_Configuration(void);
void CAN1_GPIO_Configuration(void);
void CAN1_NVIC_Configuration(void);
void CAN1_Configuration(void);




/*
********************************************************************************
                     void BSP_CAN1_Init(void)

������     CAN1��ʼ��
������     ��
����ֵ��   ��
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

������     ʱ������
������     ��
����ֵ��   ��
********************************************************************************
*/
void CAN1_RCC_Configuration(void){
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);
}
/*
********************************************************************************
                              void CAN1_GPIO_Configuration(void);
  

������     �ܽ�����
������     ��
����ֵ��   ��
********************************************************************************
*/
void CAN1_GPIO_Configuration(void){
  GPIO_InitTypeDef GPIO_InitStructure;

  
   //�˿ڹ��ܸ���
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

������     �ж�����
������     ��
����ֵ��   ��
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

������     CAN����
������     ��
����ֵ��   ��
********************************************************************************
*/
void CAN1_Configuration(void){
    CAN_InitTypeDef CAN_InitStructure;                                
    CAN_FilterInitTypeDef CAN_FilterInitStructure;       
    
    
    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);
    /************************************************************************
                        CAN ����
    *************************************************************************/
    CAN_InitStructure.CAN_TTCM = DISABLE;                     //��ֹʱ���
    CAN_InitStructure.CAN_ABOM = DISABLE;                     //����ģʽ�����ʵ��
    CAN_InitStructure.CAN_AWUM = DISABLE;                     //�������
    CAN_InitStructure.CAN_TTCM = DISABLE;                     //��ֹʱ�䴥��ͨ��ģʽ
    CAN_InitStructure.CAN_NART = DISABLE;                     //��ֹ�Զ��ش�
    CAN_InitStructure.CAN_TXFP = DISABLE;                     //���ȼ��ɱ��ĵı�ʶ��������
    CAN_InitStructure.CAN_RFLM = DISABLE;                     //�������ʱFIFO����������һ���յ��ı��ĸ���ԭ�б���     
    
  //  CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack;           //CANӲ����������ģʽ
    
   CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;           //CANӲ������ģʽ
    //����������
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;                  //����ͬ����Ծ���Ϊ2��ʱ�䵥λ
    CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;                  //ʱ���Ϊ6��ʱ�䵥λ
    CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;                  //ʱ���Ϊ5��ʱ�䵥λ
    CAN_InitStructure.CAN_Prescaler = 6;                      //(pclk1/((1+6+5)*6)) = 36Mhz/��12*6�� = 0.5Mbits�趨��һ��ʱ�䵥λ�ĳ���12  
    CAN_Init(CAN1, &CAN_InitStructure);
    
    /************************************************************************
                        ���������ã��б�ģʽ
    *************************************************************************/
    CAN_FilterInitStructure.CAN_FilterNumber=0;                                 //���ó�ʼ����������Ϊ0
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;             //�趨��������Ϊ����λģʽ
//	  CAN_FilterInitStructure.CAN_FilterMode =  CAN_FilterMode_IdMask; 
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;            //������λ��Ϊ16λ������һ��
    CAN_FilterInitStructure.CAN_FilterIdHigh = USER_STDID_ADR1<<5;               //�趨������16λΪ��һ��
    CAN_FilterInitStructure.CAN_FilterIdLow =  USER_STDID_ADR2<<5;               //�趨������16λΪ�ڶ���
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh= USER_STDID_ADR3<<5;            //�趨������16λΪ������
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=  USER_STDID_ADR4<<5;            //�趨������16λΪ���ĸ�
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;        //������FIFO0ָ�������0
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;                      //ʹ�ܹ�����
    CAN_FilterInit(&CAN_FilterInitStructure);  
    
    CAN_FilterInitStructure.CAN_FilterNumber=1;                                 //���ó�ʼ����������Ϊ0
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;             //�趨��������Ϊ����λģʽ
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;            //������λ��Ϊ16λ������һ��
    CAN_FilterInitStructure.CAN_FilterIdHigh = USER_STDID_ADR5<<5;               //�趨������16λΪ��һ��
    CAN_FilterInitStructure.CAN_FilterIdLow =  USER_STDID_ADR6<<5;               //�趨������16λΪ�ڶ���
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh= USER_STDID_ADR7<<5;            //�趨������16λΪ������
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=  USER_STDID_ADR8<<5;            //�趨������16λΪ���ĸ�
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;        //������FIFO0ָ�������0
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;                      //ʹ�ܹ�����
    CAN_FilterInit(&CAN_FilterInitStructure);  

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE); //FIF0��Ϣ�Һ��ж�����
    
    
}

