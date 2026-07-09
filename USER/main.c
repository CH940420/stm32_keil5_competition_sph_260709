#ifndef __MAIN_H
#define __MAIN_H

#include "system.h"
#include "string.h"
#include "usart4.h" 

// 调度函数声明
void Loop_check(void);
void main_loop(void);

#endif

// ===================== WiFi + MQTT =====================
#define WIFI_SSID     "ch"
#define WIFI_PWD      "wxh20041101"
#define TB_HOST       "192.168.43.249"
#define TB_PORT       1883
#define CLIENT_ID     "admin"
#define TB_TOKEN      "Q34ndygRjCLg8jTOw4sr"
#define TB_TOPIC      "v1/devices/me/telemetry"




// 导航规划
typedef struct{
	u32 pos_x;
	u32 pos_y;
	u32 action;
} Map;

typedef struct {
	Map elem[MAXSIZE];
	int length;
} List;

// 船体位置状态
typedef struct{
	u32 pos_x_begin;
	u32 pos_y_begin;
	int pos_list;
	u32 pos_x_end;
	u32 pos_y_end;
	double angle;
	double len;
	u32 pos_action;
	int command;
	int status;
} Ship;

// ===================== 小船位置宏定义 =====================
#define SHIP_BIGIN      0
#define SHIP_CON        1
#define SHIP_ACT1       1
#define SHIP_ACT2       2
#define SHIP_END        3

char st = 'A';
Ship shipa;
List LA;
Map e;

u8	key_num=0;
u8 USART1_TX_BUF[USART3_MAX_RECV_LEN];

double bearing;
double angle=0.0;
float	latitude1,longitude1,latitude2,longitude2;
float mylon,mylat;
float lat2[2];
float lon2[2];
double len=0;
float yaw1=0.0;
float pitch,roll,yaw;
short temp_mpu=0;

u8 flag_auto_tl=0,flag_auto_ps=0;
u8 flag_au_en=0;
u8 flag_au_doing=0;
u8 flag_au_doing_jieduan=0;
u32 gps_lat,gps_lon;

u8 uart_buf[128];    // 接收缓冲区
u32 uart_len = 0;    // 接收长度
u8 uart_recv_flag = 0; // 接收完成标志

//////////////////////////////////////////
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;


const uint32_t c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
//////////////////////////////////////////

void InitList();
Map GetElem(int i);
void ListInsert(Map ee);
void ListOutput(List L);
void Gps_Msg_Show(void);
void model_change(void);
double Length(double lat1, double lon1, double lat2, double lon2);
double toRadians(double angdeg);
double toDegrees(double angrad);
double calculateBearing(void);
void InsertLANode(int act);

u8 TEXT_Buffer[1000]={0};
#define SIZE sizeof(TEXT_Buffer)
u32 FLASH_SIZE;

int ListLength(List L);




loop_t loop;
u32 time[10],time_sum;

// ===================== 函数声明 =====================
void main_loop(void);
void Loop_check(void);
void Duty_2ms();
void Duty_4ms();
void Duty_6ms();
void Duty_10ms();
void Duty_20ms();
void Duty_50ms();
void Duty_1000ms();
void RC_Analy(void);
void CopeCmdData(unsigned char ucData);
void ShowHelp(void);
void CmdProcess(void);
void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
void Delayms(uint16_t ucMs){ delay_ms(ucMs); };
void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
void ESP_SendCmd(char *cmd);
void System_Init_ESP(void);
void MQTT_Report_Temp(float temp);
void USART3_SendString(char *str);

// ===================== 多任务调度递加 =====================
void Loop_check()
{
	loop.cnt_2ms++;
	loop.cnt_4ms++;
	loop.cnt_6ms++;
	loop.cnt_10ms++;
	loop.cnt_20ms++;
	loop.cnt_50ms++;
	loop.cnt_1000ms++;

	if( loop.check_flag >= 1) loop.err_flag++;
	else loop.check_flag = 1;
}

void main_loop()
{
	if( loop.check_flag )
	{
		if( loop.cnt_2ms >= 1 )  
    { 
		   loop.cnt_2ms = 0; 
		   Duty_2ms(); 
		}
		if( loop.cnt_4ms >= 2 ) 
    { 
		   loop.cnt_4ms = 0; 
		   Duty_4ms(); 
		}
		if( loop.cnt_6ms >= 3 )  
    { 
		   loop.cnt_6ms = 0; 
		   Duty_6ms(); 
		}
		if( loop.cnt_10ms >=5 )  
    { 
		  loop.cnt_10ms= 0; 
		  Duty_10ms();
		}
		if( loop.cnt_20ms >=10 ) 
    { 
		   loop.cnt_20ms=0; 
		    Duty_20ms();
		}
		if( loop.cnt_50ms >=250 ) { loop.cnt_50ms=0; Duty_50ms();}  //25>250放大10 500ms
		if( loop.cnt_1000ms>=500){loop.cnt_1000ms=0;Duty_1000ms();}
		loop.check_flag = 0;
	}
}

// ===================== 2ms  =====================
void Duty_2ms()
{
	CmdProcess();
	if(s_cDataUpdate)
	{
		float fAcc[3], fGyro[3], fAngle[3];
		float fLon,fLat;
		int iSVNUM;
		
		for(int i=0;i<3;i++){
			fAcc[i] = sReg[AX+i]/32768.0f*16;
			fGyro[i] = sReg[GX+i]/32768.0f*2000;
			fAngle[i] = sReg[Roll+i]/32768.0f*180;
		}
		fLon=((sReg[LonH]<<16)+sReg[LonL])/10000000.0;
		fLat=((sReg[LatH]<<16)+sReg[LatL])/10000000.0;
		iSVNUM = sReg[SVNUM];

		if(s_cDataUpdate & ANGLE_UPDATE){
			yaw = fAngle[2];
			s_cDataUpdate &= ~ANGLE_UPDATE;
		}
		if(s_cDataUpdate & READ_UPDATE){
			latitude1 =fLat;
			longitude1 = fLon;
			s_cDataUpdate &= ~READ_UPDATE;
		}
	}
	  model_change();
		TIM_SetCompare2(TIM12,pwm_right_now);
		TIM_SetCompare2(TIM9,pwm_left_now);
		
		
}

// ===================== 4ms  =====================

void Duty_4ms()
{
  // MQTT_Report_Temp();
}

// ===================== 6ms 读取GPS =====================
void Duty_6ms()
{

}

// ===================== 10ms 遥控数据解析 =====================
void Duty_10ms()
{
	RC_Analy();
}

void Duty_20ms(){}
//500ms暂时
void Duty_50ms()
{
	RS485_Poll_Task();
  WitReadReg(YYMM, 0x30);
	printf("左桨速度:%d，右桨速度:%d\r\n",pwm_left_now,pwm_right_now);
	printf("经度:%.7f  ，纬度:%.7f\r\n",longitude1,latitude1);
	printf("DO:%.2f  PH:%.2f  GPS_R1:%d GPS_R2:%d\r\n",DO_val,PH_val,gps_reg1,gps_reg2);
}
	
// ===================== 1000ms MQTT上报 =====================
void Duty_1000ms()
{

	MQTT_Report_Temp(18.5f);   // 每秒上报一次温度
	
}

// ===================== 遥控数据解析 =====================
void RC_Analy()
{
    if(USART6_RX_STA&0X8000)
	  {
			YY_ON_Flag=0;
			Remote_ON_Flag=1;
			ROS_ON_Flag=0;
			Usart1_ON_Flag=0;
			Usart5_ON_Flag=0;
			Sbus_GetByte();
			ship_sbus_control();
			motor_control();
			Remote_ON_Flag=1;
			USART6_RX_STA=0;
		//	TIM_SetCompare2(TIM12,pwm_right_now/2);
		//	TIM_SetCompare2(TIM9,pwm_left_now);
		}
}

// ===================== MAIN =====================
int main(void)
{
	u16 i;
	u8 datatemp[SIZE];
	u8 readflash_lengh=0;
	Map MAP_read;

	FLASH_SIZE=16*1024*1024;
	systemInit();
	InitList(LA);

	uart_init(115200);
	
	printf("GPS初始化\r\n");
	Usart2Init(9600);
	UART4_Init(9600);    //新增UART4 485初始化
	
	//new gps
	WitInit(WIT_PROTOCOL_MODBUS, 0x50);
	WitSerialWriteRegister(SensorUartSend);
	WitRegisterCallBack(CopeSensorData);
	WitDelayMsRegister(Delayms);
	
	// ========== ESP8266 MQTT ==========
	printf("ESP8266 MQTT初始化\r\n");
	usart3_init(115200);			//初始化串口3波特率为38400  wifi
	System_Init_ESP();
	
	shipa.pos_x_begin=0;
	shipa.pos_y_begin=0;
	shipa.pos_list=0;
	shipa.pos_x_end=0;
	shipa.pos_y_end=0;
	shipa.pos_action=0;
	shipa.angle=0;
	shipa.len=0;
	shipa.command=0;
	shipa.status=SHIP_BIGIN;
	shipa.command=1;

	W25QXX_Read(datatemp,0,1000);
	flag_au_en=datatemp[0];
	if(flag_au_en==1)
	{
	   readflash_lengh=datatemp[1];
		 for(i=0;i<readflash_lengh;i++)
		 {
		  memcpy(&MAP_read,&datatemp[i*sizeof(MAP_read)+2],sizeof(MAP_read));
			ListInsert(MAP_read);
		 }
	}
	ListOutput(LA);
  printf("程序开始\r\n");
	
   while(1)
	{
		RS485_Poll_Task(); // 自动轮询DO/pH/GPS，刷新DO_val/PH_val/gps_reg1/gps_reg2
    delay_ms(50);
		main_loop();
		
		printf("DO:%.2f  PH:%.2f  GPS_R1:%d GPS_R2:%d\r\n",DO_val,PH_val,gps_reg1,gps_reg2);
		
		if(uart_recv_flag)
        {
            printf("收到: %s", uart_buf);
            uart_len = 0;
            uart_recv_flag = 0;
            memset(uart_buf, 0, sizeof(uart_buf));
        }
		
	}
}

// ===================== 遥控模式切换 =====================
void model_change(void)
{
	int i=0;
	float angledec=0;
	u8 runoverflag=0;
	Map MAP_write;

	switch(model)
	{
		case 0:
			ship_speed_up();
			shipa.pos_list=0;
			break;

		case 1:
			if(study==0)
			{
				if(flag_au_en && shipa.pos_list==0){
					flag_au_doing=1;
					flag_au_doing_jieduan=0;
				}
				if(flag_au_doing)
				{
					if(flag_au_doing_jieduan==0){
						shipa.pos_list++;
						flag_au_doing_jieduan=1;
					}
					if(shipa.pos_list <= LA.length)
					{
						e = GetElem(shipa.pos_list);
						shipa.pos_x_end = e.pos_x;
						shipa.pos_y_end = e.pos_y;
						latitude2 = shipa.pos_x_end / 10000000.0;


						longitude2 = shipa.pos_y_end / 10000000.0;
						shipa.pos_action = e.action;
						shipa.status = SHIP_CON;

						len = Length(latitude1, longitude1, latitude2, longitude2);
						angle = calculateBearing();
						angledec = angle - yaw;

						if(angledec > 3) turn_right();
						if(angledec < -3) turn_left();

						if(angledec>-3 && angledec<3){
							if(len>20) ship_speed_down(0);
							else ship_speed_down(1);
						}
						if(len <= 1){
							ship_stop();
							runoverflag=1;
						}
						if(runoverflag)
						{
							if(shipa.pos_action == 1)
							{
								if(flag_auto_tl == 0){
									GPIO_ResetBits(GPIOE,GPIO_Pin_8);
									flag_auto_tl=1;
									TIM_Cmd(TIM5,ENABLE);
									TIM_SetCounter(TIM5,0);
								}
								if(flag_auto_tl == 2){
									flag_auto_tl=0; runoverflag=0; flag_au_doing_jieduan=0;
								}
							}
							else if(shipa.pos_action == 2)
							{
								if(flag_auto_ps == 0){
									GPIO_ResetBits(GPIOE,GPIO_Pin_9);
									flag_auto_ps=1;
									TIM_Cmd(TIM5,ENABLE);
									TIM_SetCounter(TIM5,0);
								}
								if(flag_auto_ps == 2){
									flag_auto_ps=0; runoverflag=0; flag_au_doing_jieduan=0;
								}
							}
							else { runoverflag=0; flag_au_doing_jieduan=0; }
						}
					}
					if(shipa.pos_list > LA.length)
					{
						flag_au_doing=0;
					}
				}
			}
			break;
		default:break;
	}

	if(study == 1)
	{
		if(model_old == 0){
			LA.length=0;
			InsertLANode(0);
		}
		model_old = 1;
	}
	else if(study == 0)
	{
		if(model_old == 1)
		{
			InsertLANode(0);
			flag_au_en=1;
			TEXT_Buffer[0]=flag_au_en;
			TEXT_Buffer[1]=LA.length;
			for(i=0;i<LA.length;i++){
				MAP_write=GetElem(i+1);
				memcpy(&TEXT_Buffer[2+i*sizeof(MAP_write)],&MAP_write,sizeof(MAP_write));
			}
			W25QXX_Write(TEXT_Buffer,0,SIZE);
			model_old=0;
		}
	}
}

// ===================== 点位距离计算 =====================
double Length(double lat1, double lon1, double lat2, double lon2)
{
	 double EARTH_RADIUS = 6371;
	 double radLat1 = toRadians(lat1);
   double radLat2 = toRadians(lat2);
	 double a = radLat1 - radLat2;
   double b = toRadians(lon1) - toRadians(lon2);
   double s = 2 * asin(sqrt(pow(sin(a/2),2)+cos(radLat1)*cos(radLat2)*pow(sin(b/2),2)));
   s = s * EARTH_RADIUS*1000;
   return s;
}

void InitList(){ LA.length=0; }
int ListLength(List L){ return L.length; }

Map GetElem(int i)
{
	Map e1;
	e1.pos_x = LA.elem[i-1].pos_x;
	e1.pos_y = LA.elem[i-1].pos_y;
	e1.action = LA.elem[i-1].action;
	return e1;
}

void ListInsert(Map ee)
{
	LA.elem[LA.length].pos_x = ee.pos_x;
	LA.elem[LA.length].pos_y = ee.pos_y;
	LA.elem[LA.length].action = ee.action;
	LA.length++;
}

void ListOutput(List L)
{
	for(int i=0;i<L.length;i++){
		printf("%d %d %d\r\n",L.elem[i].pos_x,L.elem[i].pos_y,L.elem[i].action);
	}
}

double toRadians(double angdeg){ return angdeg/180.0*PI; }
double toDegrees(double angrad){ return angrad*180.0/PI; }

double calculateBearing(void)
{
    double dlon = longitude2 - longitude1;
    double y = sin(dlon) * cos(latitude2);
    double x = cos(latitude1)*sin(latitude2) - sin(latitude1)*cos(latitude2)*cos(dlon);
    return toDegrees(atan2(y,x));
}

// ===================== 点位储存 =====================
void InsertLANode(int act)
{
	Map MAP_write;
	MAP_write.pos_x = latitude1 * 10000000;
	MAP_write.pos_y = longitude1 * 10000000;
	MAP_write.action = act;
	ListInsert(MAP_write);
}



void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
	GPIO_SetBits(GPIOG, GPIO_Pin_8);
	Uart2Send(p_data, uiSize);
	GPIO_ResetBits(GPIOG, GPIO_Pin_8);
}

//static void Delayms(uint16_t ucMs)
//{
//	delay_ms(ucMs);
//}

void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}

//void AutoScanSensor(void)
//{
//	int i, iRetry;
//	
//	for(i = 1; i < 10; i++)
//	{
//		Usart2Init(c_uiBaud[i]);
//		iRetry = 2;
//		do
//		{
//			s_cDataUpdate = 0;
//			WitReadReg(AX, 3);
//			delay_ms(100);
//			if(s_cDataUpdate != 0)
//			{
//				printf("%d baud find sensor\r\n\r\n", c_uiBaud[i]);
//				ShowHelp();
//				return ;
//			}
//			iRetry--;
//		}while(iRetry);		
//	}
//	printf("can not find sensor\r\n");
//	printf("please check your connection\r\n");
//}

void CmdProcess(void)
{
	switch(s_cCmd)
	{
		case 'a':	
			if(WitStartAccCali() != WIT_HAL_OK) 
				printf("\r\nSet AccCali Error\r\n");
			break;
		case 'm':	
			if(WitStartMagCali() != WIT_HAL_OK) 
				printf("\r\nSet MagCali Error\r\n");
			break;
		case 'e':	
			if(WitStopMagCali() != WIT_HAL_OK)
				printf("\r\nSet MagCali Error\r\n");
			break;
		case 'u':	
			if(WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK) 
				printf("\r\nSet Bandwidth Error\r\n");
			break;
		case 'U':	
			if(WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) 
				printf("\r\nSet Bandwidth Error\r\n");
			break;
		case 'B':	
			if(WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK) 
				printf("\r\nSet Baud Error\r\n");
			//else
				//Usart2Init(c_uiBaud[WIT_BAUD_115200]);
			break;
		case 'b':	
			if(WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK)
				printf("\r\nSet Baud Error\r\n"); 
			//else 
				//Usart2Init(c_uiBaud[WIT_BAUD_9600]);
			break;
		case 'h':	
			//  ShowHelp()
			break;
		default :
			return;
	}
	s_cCmd = 0xff;
}

void CopeCmdData(unsigned char ucData)
{
	static unsigned char s_ucData[50], s_ucRxCnt = 0;
	
	s_ucData[s_ucRxCnt++] = ucData;
	if(s_ucRxCnt<3)return;
	if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
	if(s_ucRxCnt >= 3)
	{
		if((s_ucData[1] == '\r') && (s_ucData[2] == '\n'))
		{
			s_cCmd = s_ucData[0];
			memset(s_ucData,0,50);
			s_ucRxCnt = 0;
		}
		else 
		{
			s_ucData[0] = s_ucData[1];
			s_ucData[1] = s_ucData[2];
			s_ucRxCnt = 2;
		}
	}
}

// ================= 唯一的ESP_SendCmd函数 =================
// 发送AT指令
void ESP_SendCmd(char *cmd)
{
    USART3_SendString(cmd);
    delay_ms(100);
}

// ESP8266初始化(WiFi+MQTT)
void System_Init_ESP(void)
{
		// 1. 复位ESP8266
    ESP_SendCmd("AT+RST\r\n");
    delay_ms(500);
		printf("ESP8266 OK\r\n");
	
		// 2. 设置Station模式
    ESP_SendCmd("AT+CWMODE=1\r\n");
    delay_ms(500);
		printf("Station OK\r\n");
	
		// 3. 关闭自动连接
    ESP_SendCmd("AT+CWAUTOCONN=0\r\n");
    delay_ms(500);
		printf("自动连接关闭OK\r\n");

		// 4. 连接WiFi
    char wifi_cmd[64];
    sprintf(wifi_cmd, "AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI_SSID, WIFI_PWD);
    ESP_SendCmd(wifi_cmd);
    delay_ms(1000);
		printf("WIFI OK\r\n");
	
		// 5. MQTT用户配置
    char mqtt_cfg[128];
    sprintf(mqtt_cfg, "AT+MQTTUSERCFG=0,1,\"%s\",\"%s\",\"\",0,0,\"\"\r\n", CLIENT_ID, TB_TOKEN);
    ESP_SendCmd(mqtt_cfg);
    delay_ms(1000);
		printf("MQTT_配置 OK\r\n");
		
		// 6. 连接MQTT服务器
    char mqtt_conn[64];
    sprintf(mqtt_conn, "AT+MQTTCONN=0,\"%s\",%d,0\r\n", TB_HOST, TB_PORT);
    ESP_SendCmd(mqtt_conn);
    delay_ms(1000);
		printf("MQTT_服务器 OK\r\n");

		// 7. 订阅属性主题
    ESP_SendCmd("AT+MQTTSUB=0,\"v1/devices/me/attributes\",0\r\n");
    delay_ms(1000);
		printf("MQTT_主题 OK\r\n");
    
    printf("ESP8266 + MQTT 初始化完成\r\n");
}

// MQTT上报船的数据含位置、方向、温度,到地点再上报水参数，预留
void MQTT_Report_Temp(float temp)
{
    char pub_cmd[255];
	char pub1[120];
	//"{\"Temp\":%u,\"Humi\":%u,\"Led\":%s}"
	sprintf(pub1,"{\"lati\":%.7f,\"longi\":%.7f,\"yaw\":%.2f,\"temp\":%.1f,\"DO\":%.2f,\"PH\":%.2f,\"gps1\":%d,\"gps2\":%d}\r\n",
        latitude1,longitude1,yaw,temp,DO_val,PH_val,gps_reg1,gps_reg2);
	sprintf(pub_cmd, "AT+MQTTPUBRAW=0,\"%s\",%d,0,0\r\n",TB_TOPIC,strlen(pub1));  
	
	ESP_SendCmd(pub_cmd);
	delay_ms(100);
	ESP_SendCmd(pub1);
}

void USART3_SendString(char *str)
{
	while(*str)
	{
		while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
		USART_SendData(USART3, *str++);
	}
}
