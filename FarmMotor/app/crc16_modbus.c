
/*
********************************************************************************
                                CRC校验
文件名：crc_modbus.h
版本：
程序员：Tank_CG
********************************************************************************
*/


/*
********************************************************************************
                               头文件        
********************************************************************************
*/

#include "crc16_modbus.h"

/*
********************************************************************************
                               函数申明   
********************************************************************************
*/  


void InvertUint16(unsigned short *dBuf,unsigned short *srcBuf);
void InvertUint8(unsigned char *dBuf,unsigned char *srcBuf);




/*
********************************************************************************
  unsigned short CRC16_MODBUS(unsigned char *puchMsg, unsigned int usDataLen)  

描述：     CRC16_MODBUS协议校验
参数：     puchMsg,需要校验的数据的指针
           usDataLen，需要校验的数据长度
返回值：   校验值
********************************************************************************
*/
unsigned short CRC16_MODBUS(unsigned char *puchMsg, unsigned int usDataLen)  
{  
  unsigned short wCRCin = 0xFFFF;  
  unsigned short wCPoly = 0x8005;  
  unsigned char wChar = 0;  
    
  while (usDataLen--)     
  {  
        wChar = *(puchMsg++);  
        InvertUint8(&wChar,&wChar);  
        wCRCin ^= (wChar << 8);  
        for(int i = 0;i < 8;i++)  
        {  
          if(wCRCin & 0x8000)  
            wCRCin = (wCRCin << 1) ^ wCPoly;  
          else  
            wCRCin = wCRCin << 1;  
        }  
  }  
  InvertUint16(&wCRCin,&wCRCin);  
  return (wCRCin) ;  
}



void InvertUint8(unsigned char *dBuf,unsigned char *srcBuf)
{
  int i;
  unsigned char tmp[4];
  tmp[0] = 0;
  for(i=0;i< 8;i++)
  {
    if(srcBuf[0]& (1 << i))
      tmp[0]|=1<<(7-i);
  }
  dBuf[0] = tmp[0];
}


void InvertUint16(unsigned short *dBuf,unsigned short *srcBuf)  
{  
    int i;  
    unsigned short tmp[4];  
    tmp[0] = 0;  
    for(i=0;i< 16;i++)  
    {  
      if(srcBuf[0]& (1 << i))  
        tmp[0]|=1<<(15 - i);  
    }  
    dBuf[0] = tmp[0];  
}
