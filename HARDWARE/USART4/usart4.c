#include "usart4.h"
#include "string.h"

// 接收缓存
u8  uart4_rx_buf[UART4_RX_BUF_LEN] = {0};
u16 uart4_rx_len = 0;

// 轮询从站：0x01 DO、0x02 pH、0x50 GPS
u8 poll_slave_list[] = {0x01, 0x02, 0x50};
u8 poll_slave_idx = 0;
#define SLAVE_CNT (sizeof(poll_slave_list) / sizeof(u8))

// 全局传感器存储
u16 sensor_do = 0;
u16 sensor_ph = 0;
u16 gps_reg1 = 0, gps_reg2 = 0;
float DO_val = 0.0f;
float PH_val = 0.0f;

/**
 * @brief Modbus CRC16计算
 */
u16 Modbus_CRC16(u8 *data, u16 len)
{
    u16 crc = 0xFFFF;
    u16 i, j;
    for(i = 0; i < len; i++)
    {
        crc ^= data[i];
        for(j = 0; j < 8; j++)
        {
            if(crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Modbus 03功能码 读取保持寄存器
 */
void RS485_Read_HoldReg(u8 slave_addr, u16 reg_addr, u16 reg_num)
{
    u8 send_buf[8] = {0};
    u16 crc_val;
    send_buf[0] = slave_addr;
    send_buf[1] = 0x03;
    send_buf[2] = (reg_addr >> 8) & 0xFF;
    send_buf[3] = reg_addr & 0xFF;
    send_buf[4] = (reg_num >> 8) & 0xFF;
    send_buf[5] = reg_num & 0xFF;

    crc_val = Modbus_CRC16(send_buf, 6);
    send_buf[6] = crc_val & 0xFF;
    send_buf[7] = (crc_val >> 8) & 0xFF;

    RS485_SEND();
    for(u16 i = 0; i < 8; i++)
    {
        while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
        USART_SendData(UART4, send_buf[i]);
    }
    delay_ms(1);
    RS485_RECV();
}

/**
 * @brief 清空接收缓存
 */
void UART4_Clear_RxBuf(void)
{
    memset(uart4_rx_buf, 0, UART4_RX_BUF_LEN);
    uart4_rx_len = 0;
}

/**
 * @brief 485轮询总任务，自动切换从站、解析数据更新全局变量
 */
void RS485_Poll_Task(void)
{
    u8 cur_slave = poll_slave_list[poll_slave_idx];
    UART4_Clear_RxBuf();
    RS485_Read_HoldReg(cur_slave, 0x0000, 0x0002);
    delay_ms(40);

    // 校验应答帧
    if(uart4_rx_len >= 9 && uart4_rx_buf[0] == cur_slave)
    {
        u16 recv_crc = (uart4_rx_buf[uart4_rx_len-1] << 8) | uart4_rx_buf[uart4_rx_len-2];
        u16 calc_crc = Modbus_CRC16(uart4_rx_buf, uart4_rx_len - 2);
        if(recv_crc == calc_crc)
        {
            u16 reg_data1 = (uart4_rx_buf[3] << 8) | uart4_rx_buf[4];
            u16 reg_data2 = (uart4_rx_buf[5] << 8) | uart4_rx_buf[6];
            switch(cur_slave)
            {
                case 0x01:
										sensor_do = reg_data1;
										DO_val = sensor_do / 100.0f;
										printf("DO原始寄存器值:%d\r\n",sensor_do);
										break;
								case 0x02:
										sensor_ph = reg_data1;
										PH_val = sensor_ph / 100.0f;
										printf("PH原始寄存器值:%d\r\n",sensor_ph);
										break;
								case 0x50:
										gps_reg1 = reg_data1;
										gps_reg2 = reg_data2;
										printf("GPS reg1:%d reg2:%d\r\n",gps_reg1,gps_reg2);
										break;
						}
        }
    }

    poll_slave_idx++;
    if(poll_slave_idx >= SLAVE_CNT)
        poll_slave_idx = 0;
}

/**
 * @brief UART4初始化 PC10 TX / PC11 RX  PG8 485收发控制
 */
void UART4_Init(u32 baudrate)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOG, ENABLE);

    // PC10 PC11 复用UART4
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    // PG8 DE/RE
    GPIO_InitStruct.GPIO_Pin = RS485_DE_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(RS485_DE_PORT, &GPIO_InitStruct);
    RS485_RECV();

    // 串口参数
    USART_InitStruct.USART_BaudRate = baudrate;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART4, &USART_InitStruct);

    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
    USART_Cmd(UART4, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

/**
 * @brief UART4接收中断
 */
void UART4_IRQHandler(void)
{
    u8 dat;
    if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
    {
        dat = USART_ReceiveData(UART4);
        if(uart4_rx_len < UART4_RX_BUF_LEN)
        {
            uart4_rx_buf[uart4_rx_len++] = dat;
        }
        USART_ClearITPendingBit(UART4, USART_IT_RXNE);
    }
}