/*
********************************************************************************
                                IO控制

文件名    ：io_output.h
版本      ：
程序员    ：Tank_CG
********************************************************************************
说明：
********************************************************************************
*/

#ifndef __IO_OUTPUT_H
#define __IO_OUTPUT_H

/*
********************************************************************************
                               头文件        
********************************************************************************
*/

#include "stm32f10x.h"
#include "app_conf.h"
/*
********************************************************************************
                               宏定义      
********************************************************************************
*/


#define         DIR_FORWARD     0   //前进  
#define         DIR_BACK        1  //倒车
#define         DIR_LEFT        2  //逆时针旋转
#define         DIR_RIGHT       3  //顺时针旋转

#define	        LED_ALARM       1   //报警指示灯	
#define         RUN_NOAMAL      0
#define         TURN_LEFT       1
#define         TURN_RIGHT      2

#define         RUN_LED         1 //正常运行led 闪烁正常运行
#define         ENGINE_LED      2 //发动机启停 启动led常亮
#define         NO_LEVEL_LED    3 //没药led常亮
#define         NO_POWER_LED    4 //电量不足led常亮
#define         STOP_LED        5 //进入急停led常亮




/*
********************************************************************************
                               函数申明      
********************************************************************************
*/

void    OutPutInit       (void);
void    EngineRelay      (SwitchState state);
void    DriveDirection   (uint8_t dir);
void    RunMessage       (uint8_t whatMsg,SwitchState state);


#endif
