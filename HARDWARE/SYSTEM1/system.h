#ifndef __SYSTEM_H
#define __SYSTEM_H

// Refer to all header files you need

//FreeRTOS related header files

#include "stm32f4xx.h"

//The associated header file for the peripheral 
//外设的相关头文件
#include "sys.h"
#include "delay.h"
#include "usart.h"


#include "ship.h"



#include "timer.h"
#include "SBUS.h"


#include "usart3.h"
#include "math.h"
#include "rs485.h"
#include "spi.h"
#include "w25qxx.h" 

#include "wit_c_sdk.h"
//#include "UART1.h"
#include "UART2.h"

#define DATA_STK_SIZE   512 
#define DATA_TASK_PRIO  4

//要写入到STM32 FLASH的字符串数组
//const u8 TEXT_Buffer[]={"STM32 FLASH TEST"};
//#define TEXT_LENTH sizeof(TEXT_Buffer)	 		  	//数组长度	
//#define SIZE TEXT_LENTH/4+((TEXT_LENTH%4)?1:0)
//#define FLASH_SAVE_ADDR  0X0800C004 	//设置FLASH 保存地址(必须为偶数，且所在扇区,要大于本代码所占用到的扇区.
//										//否则,写操作的时候,可能会导致擦除整个扇区,从而引起部分程序丢失.引起死机.

#define FRAME_HEADER_CAR 0X7B //控制底盘运动的数据帧头
#define FRAME_TAIL_CAR 0X7D //控制底盘运动的数据帧尾
#define FRAME_HEADER_MOVEIT 0XAA //控制机械臂运动的数据帧头
#define FRAME_TAIL_MOVEIT 0XBB //控制机械臂运动的数据帧头
#define SEND_DATA_SIZE    24
#define RECEIVE_DATA_SIZE 11
#define follower 2
#define default_mode 1
#define MAXSIZE 20
#define SHIP_BIGIN 0
#define SHIP_ACT1 1
#define SHIP_ACT2 2
#define SHIP_END  3
#define SHIP_CON  4
#define EN_PRINT  0
// Enumeration of car types
//小车型号的枚举定义
typedef enum 
{
	Mec_Car = 0, 
	FourWheel_Car, 
	Tank_Car
} CarMode;

//Motor speed control related parameters of the structure
//电机速度控制相关参数结构体
typedef struct  
{
	float Encoder;     //Read the real time speed of the motor by encoder //编码器数值，读取电机实时速度
	float Motor_Pwm;   //Motor PWM value, control the real-time speed of the motor //电机PWM数值，控制电机实时速度
	float Target;      //Control the target speed of the motor //电机目标速度值，控制电机目标速度
	float Velocity_KP; //Speed control PID parameters //速度控制PID参数
	float	Velocity_KI; //Speed control PID parameters //速度控制PID参数
}Motor_parameter;

//Smoothed the speed of the three axes
//平滑处理后的三轴速度
typedef struct  
{
	float VX;
	float VY;
	float VZ;
}Smooth_Control;

// ===================== 多任务调度框架 =====================
typedef struct
{
	u8 check_flag;
	u16 err_flag;
	s16 cnt_2ms;
	s16 cnt_4ms;
	s16 cnt_6ms;
	s16 cnt_10ms;
	s16 cnt_20ms;
	s16 cnt_50ms;
	s16 cnt_1000ms;
}loop_t;


/*****A structure for storing triaxial data of a gyroscope accelerometer*****/
/*****用于存放陀螺仪加速度计三轴数据的结构体*********************************/
typedef struct __Mpu6050_Data_ 
{
	short X_data; //2 bytes //2个字节
	short Y_data; //2 bytes //2个字节
	short Z_data; //2 bytes //2个字节
}Mpu6050_Data;

/*******The structure of the serial port sending data************/
/*******串口发送数据的结构体*************************************/
typedef struct _SEND_DATA_  
{
	unsigned char buffer[SEND_DATA_SIZE];
	struct _Sensor_Str_
	{
		unsigned char Frame_Header; //1个字节
		short X_speed;	            //2 bytes //2个字节
		short Y_speed;              //2 bytes //2个字节
		short Z_speed;              //2 bytes //2个字节
		short Power_Voltage;        //2 bytes //2个字节
		Mpu6050_Data Accelerometer; //6 bytes //6个字节
		Mpu6050_Data Gyroscope;     //6 bytes //6个字节	
		unsigned char Frame_Tail;   //1 bytes //1个字节
	}Sensor_Str;
}SEND_DATA;

typedef struct _RECEIVE_DATA_  
{
	unsigned char buffer[RECEIVE_DATA_SIZE];
	struct _Control_Str_
	{
		unsigned char Frame_Header; //1 bytes //1个字节
		float X_speed;	            //4 bytes //4个字节
		float Y_speed;              //4 bytes //4个字节
		float Z_speed;              //4 bytes //4个字节
		unsigned char Frame_Tail;   //1 bytes //1个字节
	}Control_Str;
}RECEIVE_DATA;

/****** external variable definition. When system.h is referenced in other C files, 
        other C files can also use the variable defined by system.c           ******/
/****** 外部变量定义，当其它c文件引用system.h时，也可以使用system.c定义的变量 ******/
extern u8 Flag_Stop;
extern int Divisor_Mode;
extern u8 Car_Mode;
extern int Servo;
extern float RC_Velocity;
extern float Move_X, Move_Y, Move_Z; 
extern float Velocity_KP, Velocity_KI;	
extern Smooth_Control smooth_control;
extern Motor_parameter MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;
extern float Encoder_precision;
extern float Wheel_perimeter;
extern float Wheel_spacing; 
extern float Axle_spacing; 
extern float Omni_turn_radiaus; 
extern u8 YY_ON_Flag, ROS_ON_Flag, Remote_ON_Flag, Usart1_ON_Flag, Usart5_ON_Flag;
extern u8 Remote_SPEED_ON_Flag, Remote_SPEED_SET_Flag; 
extern u8 Flag_Left, Flag_Right, Flag_Direction, Turn_Flag; 
extern u8 PID_Send;
extern u8 car_A_steer_flag;
extern int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;
extern int Check, Checking, Checked, CheckCount, CheckPhrase1, CheckPhrase2;
extern long int ErrorCode; 
extern float Moveit_Angle1,Moveit_Angle2,Moveit_Angle3,Moveit_Angle4;
extern int  Moveit_PWM1,Moveit_PWM2,Moveit_PWM3,Moveit_PWM4;
extern vu16 USART6_RX_STA;
extern u16 Rc_Sbus_In[16];
extern int L_Remoter_Ch1,L_Remoter_Ch2,L_Remoter_Ch3,L_Remoter_Ch4;
extern int Remoter_Ch1,Remoter_Ch2,Remoter_Ch3,Remoter_Ch4;
extern u8 flag_send_per;
extern u16 pwm_right_now;
extern u16 pwm_left_now;
extern u16 ch_wal_now[16];
extern int model,study;
extern int model_old;
extern u8 flag_auto_tl,flag_auto_ps;

//extern u16 ch_wal_old[16];
void systemInit(void);
void ship_up(void);
int UART5_IRQHandler(void);
void motor_control(void);
void mpu_dmp_init(void);
u8 mpu_dmp_get_data(float *pitch,float *roll,float *yaw);
void  InsertLANode(int act);
void Loop_check(void);

/***Macros define***/ /***宏定义***/
//After starting the car (1000/100Hz =10) for seconds, it is allowed to control the car to move
//开机(1000/100hz=10)秒后才允许控制小车进行运动
#define CONTROL_DELAY		1000
//The number of robot types to determine the value of Divisor_Mode. There are currently 6 car types
//机器人型号数量，决定Divisor_Mode的值，目前有6种小车类型
#define CAR_NUMBER    3      
#define RATE_1_HZ		  1
#define RATE_5_HZ		  5
#define RATE_10_HZ		10
#define RATE_20_HZ		20
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000
/***Macros define***/ /***宏定义***/

//C library function related header file
//C库函数的相关头文件
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"
#endif 
