#ifndef __UART4_H
#define __UART4_H

#include "system.h"
#include "delay.h"

// 485 DE/RE控制引脚 PG8
#define RS485_DE_PIN    GPIO_Pin_8
#define RS485_DE_PORT   GPIOG
#define RS485_SEND()    GPIO_SetBits(RS485_DE_PORT, RS485_DE_PIN)
#define RS485_RECV()    GPIO_ResetBits(RS485_DE_PORT, RS485_DE_PIN)

#define UART4_RX_BUF_LEN  64

// 串口接收缓存
extern u8  uart4_rx_buf[UART4_RX_BUF_LEN];
extern u16 uart4_rx_len;
extern u8  poll_slave_idx;

// 传感器全局数据
extern u16 sensor_do;
extern u16 sensor_ph;
extern u16 gps_reg1, gps_reg2;
extern float DO_val;
extern float PH_val;

// 函数声明
u16 Modbus_CRC16(u8 *data, u16 len);
void RS485_Read_HoldReg(u8 slave_addr, u16 reg_addr, u16 reg_num);
void UART4_Clear_RxBuf(void);
void RS485_Poll_Task(void);
void UART4_Init(u32 baudrate);

#endif