#ifndef __SBUS_H
#define __SBUS_H

#include "system.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//LED驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
#define USART6_MAX_RECV_LEN		25					//最大接收缓存字节数


void Sbus_GetByte(void);
void SBUS_Init(void);//初始化		 				    
#endif
