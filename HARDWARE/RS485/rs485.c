#include "sys.h"		    
#include "rs485.h"	 
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//RS485驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/7
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 
u8 sendaskbuf[8]={0x01,0x03,0x06,0x06,0x00,0x08,0xA4,0x85};
u8 sendclearbuf[13]={0x01,0x10,0x06,0x2A,0x00,0x02,0x04,0x00,0x00,0x00,0x01,0x9B,0xA8};
u8 crc16(uint8_t *addr, uint8_t num);
#if EN_USART2_RX   		//如果使能了接收   	  
//接收缓存区 	
u8 RS485_RX_BUF[64];  	//接收缓冲,最大64个字节.
//接收到的数据长度
u8 RS485_RX_CNT=0;  
u8 receivestate=0;
u8 receiveflag=0;
void USART2_IRQHandler(void)
{
	u8 res;	 
  u8 len=0;
	u8 crcres=0;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)//接收到数据
	{	 	
	  res =USART_ReceiveData(USART2);//;读取接收到的数据USART2->DR
		if(receiveflag==0)
		{
			RS485_RX_BUF[RS485_RX_CNT]=res;		//记录接收到的值
			if(RS485_RX_BUF[0]==0x01)
			{
					receivestate=1;	
			}
			if((receivestate==1)&&(RS485_RX_CNT>0))
			{
				//查询
				if(RS485_RX_BUF[1]==0x03)
				{
					 if(RS485_RX_CNT==8) 
					 {
						 len=RS485_RX_BUF[2];
						 
						 //CRC校验
						 crcres=crc16(RS485_RX_BUF,len);
						 if(crcres==1)
						 {
							 receiveflag=1;
						 }
						 else
						 {
							 RS485_RX_CNT=0;
							 receivestate=0;
						 }
						 //校验成功说明一帧结束，否则推出重新开始
					 }
					
					 
				}
				else 		if(RS485_RX_BUF[1]==0x10)//清零
				{
					 if(RS485_RX_CNT==7) 
					 {
						 len=RS485_RX_BUF[2];
						 
						 //CRC校验
						 //校验成功说明一帧结束  ，否则推出重新开始
						  //CRC校验
						 crcres=crc16(RS485_RX_BUF,len);
						 if(crcres==1)
						 {
							 receiveflag=1;
						 }
						 else
						 {
							 RS485_RX_CNT=0;
							 receivestate=0;
						 }
					 }
					
				}
				else
				{
					 RS485_RX_CNT=0;
					 receivestate=0;
				}
			}
			if(receivestate==1)
			{
				RS485_RX_CNT++;						//接收数据增加1 
			}	
	 }
	}  											 
} 
#endif										 
//初始化IO 串口2
//bound:波特率	  
void RS485_Init(u32 bound)
{  	 
	
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟
	
  //串口2引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2复用为USART2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3复用为USART2
	
	//USART2    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA2与GPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA2，PA3
	
	//PG8推挽输出，485模式控制  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //GPIOG8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOG,&GPIO_InitStructure); //初始化PG8
	

   //USART2 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口2
	
  USART_Cmd(USART2, ENABLE);  //使能串口 2
	
	USART_ClearFlag(USART2, USART_FLAG_TC);
	
#if EN_USART2_RX	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启接受中断

	//Usart2 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

#endif	
	
	RS485_TX_EN=0;				//默认为接收模式	
}

//RS485发送len个字节.
//buf:发送区首地址
//len:发送的字节数(为了和本代码的接收匹配,这里建议不要超过64个字节)
void RS485_Send_Data(u8 *buf,u8 len)
{
	u8 t;
	RS485_TX_EN=1;			//设置为发送模式
  	for(t=0;t<len;t++)		//循环发送数据
	{
	  while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET); //等待发送结束		
    USART_SendData(USART2,buf[t]); //发送数据
	}	 
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET); //等待发送结束		
	RS485_RX_CNT=0;	  
	RS485_TX_EN=0;				//设置为接收模式	
}
//RS485查询接收到的数据
//buf:接收缓存首地址
//len:读到的数据长度
void RS485_Receive_Data(u8 *buf,u8 *len)
{
	u8 rxlen=RS485_RX_CNT;
	u8 i=0;
	*len=0;				//默认为0
	delay_ms(10);		//等待10ms,连续超过10ms没有接收到一个数据,则认为接收结束
	if(rxlen==RS485_RX_CNT&&rxlen)//接收到了数据,且接收完成了
	{
		for(i=0;i<rxlen;i++)
		{
			buf[i]=RS485_RX_BUF[i];	
		}		
		*len=RS485_RX_CNT;	//记录本次数据长度
		RS485_RX_CNT=0;		//清零
	}
}

//询问重量
void RS485_Send_Ask_Weight(void)
{
	u8 t;
	RS485_TX_EN=1;			//设置为发送模式
  for(t=0;t<8;t++)		//循环发送数据
	{
	  while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);//等待发送结束		
    USART_SendData(USART2,sendaskbuf[t]); //发送数据
	}	 
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);  //等待发送结束		
	RS485_RX_CNT=0;	  
	RS485_TX_EN=0;				//设置为接收模式	
}

//询问重量
void RS485_Send_clear_Weight(void)
{
	u8 t;
	RS485_TX_EN=1;			//设置为发送模式
  for(t=0;t<13;t++)		//循环发送数据
	{
	  while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);//等待发送结束		
    USART_SendData(USART2,sendclearbuf[t]); //发送数据
	}	 
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET); //等待发送结束		
	RS485_RX_CNT=0;	  
	RS485_TX_EN=0;				//设置为接收模式	
}

u8 crc16(uint8_t *addr, uint8_t num) 
{
		uint16_t crc = 0xFFFF;
	  u8 crch,crcl;
		for (int i = 0; i < num-2; i++) 
	  {
			crc ^= *addr++;
			for (int j = 0; j < 8; j++) 
			{
				if (crc & 0x0001) 
				{
						crc = (crc >> 1) ^ 0xA001;
				} 
				else 
				{
						crc >>= 1;
				 }
			}
		}
		crch=(crc & 0xff00)>>8;
		crcl=crc & 0x00ff;
		if((crcl==addr[num-1])&&(crch==addr[num-2])) return 1;
		else return 0;
}
