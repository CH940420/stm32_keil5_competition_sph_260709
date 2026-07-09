#include "system.h" 
vu16 USART6_RX_STA=0;   
u8 datatmp[25];



void SBUS_Init(void)
{    	 
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
 
	USART_DeInit(USART6);  //复位串口3
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOB时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//使能USART3时钟
	
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //GPIO_PuPd_UP;//
	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化GPIOB11，和GPIOB10
	
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); //GPIOB11复用为USART3

	
    USART_InitStructure.USART_BaudRate = 100000;       
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
    USART_InitStructure.USART_StopBits = USART_StopBits_2;   
    USART_InitStructure.USART_Parity = USART_Parity_Even;    
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
    USART_InitStructure.USART_Mode = USART_Mode_Rx; 
    USART_Init ( USART6, &USART_InitStructure );

    USART_ITConfig ( USART6, USART_IT_RXNE, ENABLE );
  

    USART_Cmd ( USART6, ENABLE );
	
 
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	TIM7_Int_Init(100-1,8400-1);	//10ms中断一次   是否应更改为5ms一次
	
  TIM_Cmd(TIM7, DISABLE); //关闭定时器7
	
	USART6_RX_STA=0;				//清零 

}

u16 Rc_Sbus_In[16];
u8 sbus_flag;

/*
sbus flags的结构如下所示：
flags：
bit7 = ch17 = digital channel (0x80)
bit6 = ch18 = digital channel (0x40)
bit5 = Frame lost, equivalent red LED on receiver (0x20)
bit4 = failsafe activated (0x10) b: 0001 0000
bit3 = n/a
bit2 = n/a
bit1 = n/a
bit0 = n/a

*/
 void Sbus_GetByte(void)
{
	u8 i;
//  char buf[200];
	static u8 cnt = 0;
	
	cnt=USART6_RX_STA&0xff;
	
	if(cnt == 25)
	{//0X0F
		if(datatmp[0] == 0x0F && ((datatmp[24] == 0x00)||(datatmp[24] == 0x04)||(datatmp[24] == 0x14)||(datatmp[24] == 0x24)||(datatmp[24] == 0x34)))
		{
			cnt = 0;
			Rc_Sbus_In[0] = (s16)(datatmp[2] & 0x07) << 8 | datatmp[1];
			Rc_Sbus_In[1] = (s16)(datatmp[3] & 0x3f) << 5 | (datatmp[2] >> 3);
			Rc_Sbus_In[2] = (s16)(datatmp[5] & 0x01) << 10 | ((s16)datatmp[4] << 2) | (datatmp[3] >> 6);
			Rc_Sbus_In[3] = (s16)(datatmp[6] & 0x0F) << 7 | (datatmp[5] >> 1);
			Rc_Sbus_In[4] = (s16)(datatmp[7] & 0x7F) << 4 | (datatmp[6] >> 4);
			Rc_Sbus_In[5] = (s16)(datatmp[9] & 0x03) << 9 | ((s16)datatmp[8] << 1) | (datatmp[7] >> 7);
			Rc_Sbus_In[6] = (s16)(datatmp[10] & 0x1F) << 6 | (datatmp[9] >> 2);
			Rc_Sbus_In[7] = (s16)datatmp[11] << 3 | (datatmp[10] >> 5);
			
			Rc_Sbus_In[8] = (s16)(datatmp[13] & 0x07) << 8 | datatmp[12];
			Rc_Sbus_In[9] = (s16)(datatmp[14] & 0x3f) << 5 | (datatmp[13] >> 3);
			Rc_Sbus_In[10] = (s16)(datatmp[16] & 0x01) << 10 | ((s16)datatmp[15] << 2) | (datatmp[14] >> 6);
			Rc_Sbus_In[11] = (s16)(datatmp[17] & 0x0F) << 7 | (datatmp[16] >> 1);
			Rc_Sbus_In[12] = (s16)(datatmp[18] & 0x7F) << 4 | (datatmp[17] >> 4);
			Rc_Sbus_In[13] = (s16)(datatmp[20] & 0x03) << 9 | ((s16)datatmp[19] << 1) | (datatmp[18] >> 7);
			Rc_Sbus_In[14] = (s16)(datatmp[21] & 0x1F) << 6 | (datatmp[20] >> 2);
			Rc_Sbus_In[15] = (s16)datatmp[22] << 3 | (datatmp[21] >> 5);
			
//			sprintf(buf,"%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n\r",Rc_Sbus_In[0],Rc_Sbus_In[1],Rc_Sbus_In[2],Rc_Sbus_In[3],Rc_Sbus_In[4],Rc_Sbus_In[5],Rc_Sbus_In[6],Rc_Sbus_In[7],Rc_Sbus_In[8],Rc_Sbus_In[9],Rc_Sbus_In[10],Rc_Sbus_In[11],Rc_Sbus_In[12],Rc_Sbus_In[13],Rc_Sbus_In[14],Rc_Sbus_In[15]);
//      printf(buf);
			sbus_flag = datatmp[23];
			
			//user
			//
			if(sbus_flag & 0x08)
			{
				//如果有数据且能接收到有失控标记，则不处理，转嫁成无数据失控。
			}
			else
			{
				//否则有数据就喂狗
				for(i = 0;i < 8;i++)//原RC接收程序只设计了8个通道
				{
				//	ch_watch_dog_feed(i); //20241223
				}
			}
		}
		else
		{
			for(i=0; i<24;i++)
				datatmp[i] = datatmp[i+1];
			cnt = 24;
		}
	}
}



	 
void USART6_IRQHandler(void)
{
	u8 res;	    
   
	if(USART_GetFlagStatus(USART6, USART_FLAG_RXNE) != RESET)//接收到数据
	{	 
		res=USART6->DR;
		if((USART6_RX_STA&(1<<15))==0)//接收完的一批数据,还没有被处理,则不再接收其他数据
		{ 
			if(USART6_RX_STA<USART6_MAX_RECV_LEN)	//还可以接收数据
			{
				TIM_SetCounter(TIM7,0);       				//计数器清空
				if(USART6_RX_STA==0) 				//使能定时器7的中断 
				{
					TIM_Cmd(TIM7, ENABLE); 	    			//使能定时器7
				}
				datatmp[USART6_RX_STA++]=res;	//记录接收到的值	 
			}else 
			{
				USART6_RX_STA|=1<<15;				//强制标记接收完成
				TIM_Cmd(TIM7, DISABLE); 	    			//使能定时器7
			} 
		}
		
	}  											 
										 
} 
