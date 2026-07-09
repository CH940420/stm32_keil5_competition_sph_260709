#ifndef _SHIP_H
#define _SHIP_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//定时器 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/6/16
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
/*--------Motor_A control pins--------*/
#define PWM_PORTE  GPIOA			 //PWMA
#define PWM_PIN_E GPIO_Pin_6 //PWMA
#define PWME 	  TIM9->CCR2	 //PWMA


/*--------Motor_B control pins--------*/
#define PWM_PORTB GPIOB			 //PWMB
#define PWM_PIN_B GPIO_Pin_15 //PWMB
#define PWMB 	  TIM12->CCR2	 //PWMB



#define  PWM_INIT_VAL 1500

#define PI 3.1415f  //PI //圆周率

void Set_Pwm(int motor_a,int motor_b,int servo);
void Limit_Pwm(int amplitude);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);
void Drive_Motor(float Vx,float Vy,float Vz);
void Remote_Control(void);
void ship_speed_up(void);
void ship_sbus_control(void);
void turn_right(void);
void turn_left(void);
void ship_stop(void);
void motor_control(void);
void ship_init(void);
void ship_speed_down(int flag);
#endif
