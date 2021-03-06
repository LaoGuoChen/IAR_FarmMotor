/*
********************************************************************************
                               CAN1通信
文件名：bsp_can1.h
版本：
程序员：
********************************************************************************
*/

#ifndef  BSP_CAN1_H
#define  BSP_CAN1_H

/*
********************************************************************************
                                    包含的头文件
********************************************************************************
*/

#include "stm32f10x.h"

/*
********************************************************************************
                                    宏定义
********************************************************************************
*/

/* 过滤器列表地址 */
#define  USER_STDID_ADR1    0x010 	//状态切换指令地址
#define  USER_STDID_ADR2    0x020 	//发动机开关地址
#define  USER_STDID_ADR3    0x030 	//速度指令地址
#define  USER_STDID_ADR4    0x040 	//状态机反馈地址
#define  USER_STDID_ADR5    0x050 	//发动机继电器状态反馈地址
#define  USER_STDID_ADR6    0x060 	//伺服报警信息地址
#define  USER_STDID_ADR7    0x070 	//水箱水位地址
#define  USER_STDID_ADR8    0x080 	//电池电量地址
#define  USER_STDID_ADR9    0x090 	//负载曲线地址
#define  USER_STDID_ADR10   0x0A0 	//错误信息地址
#define  USER_STDID_ADR11   0x0B0 	//发动机状态地址
#define  USER_STDID_ADR12   0x0C0 	//伺服使能地址

/*
********************************************************************************
                                    函数申明
********************************************************************************
*/

void BSP_CAN1_Init(void);


#endif
