#include "system.h"
#include "delay.h"

//速度用整数表示，以便与遥控器值一致
u16 pwm_right_now=1500,pwm_right_old=1500;
u16 pwm_left_now=1500,pwm_left_old=1500;
u8 flag_ps=0,flag_tl=0;
int model=0,study=0;
int model_old=0;//记录上一次模式，以便记录开始学习和结束学习
float ship_angle=0.0;
//串口数据
u16 ch_wal_now[16];
u16 ch_wal_old[16];

//自动/手动切换，用于sbus遥控
void ship_sbus_control(void)
{
	ch_wal_now[9]=Rc_Sbus_In[9];          //手动自动模式切换按键设置
	  if(ch_wal_now[9]<500)
			model=0;
		else
			model=1;
		
	ch_wal_now[8]=Rc_Sbus_In[8];
		if(ch_wal_now[8]<500)
			{
				study=0;
			}
		else if(ch_wal_now[8]>500)
			{
				study=1;
			}
}


//停止
void ship_stop(void)
{

		pwm_left_now=1500;
	

		pwm_right_now=1500;


}
int set_pwm_min_max(int pwm_value,int line,int Pulse,int min,int max)
{
	int addvalue=(Pulse-line)/2.5;
	int i=pwm_value+addvalue;
	if (i<min)
		return min;
	if (i>max)
		return max;
	return i;
}

//加速
void ship_speed_up(void)
{
	ch_wal_now[1]=Rc_Sbus_In[1];
	ch_wal_now[2]=Rc_Sbus_In[2];
	  
	if(ch_wal_now[1]>1100)
	{
		if(ch_wal_old[1] != ch_wal_now[1])
		{
			if(ch_wal_old[1] - ch_wal_now[1] <-5 || ch_wal_old[1] - ch_wal_now[1] >5)
			{
				pwm_right_now=set_pwm_min_max(1500,1100,ch_wal_now[1],1500,1700); 
			}
			ch_wal_old[1] = ch_wal_now[1];
		}
		if(pwm_right_now!=pwm_right_old)
		{
			pwm_right_old=pwm_right_now;
		}
		delay_ms(10); //延时
	}
	else if(ch_wal_now[1]<900)
	{
		if(ch_wal_old[1] != ch_wal_now[1])
		{
			if(ch_wal_old[1] - ch_wal_now[1] <-5 || ch_wal_old[1] - ch_wal_now[1] >5)
			{
				pwm_right_now=set_pwm_min_max(1500,900,ch_wal_now[1],1300,1500); 
			}
			ch_wal_old[1] = ch_wal_now[1];
		}
	
		if(pwm_right_now!=pwm_right_old)
		{
			pwm_right_old=pwm_right_now;
		}
		delay_ms(10); //延时
	}
	else
	{
		pwm_right_now=1500;
		if(pwm_right_now!=pwm_right_old)
		{
			pwm_right_old=pwm_right_now;
		}
		delay_ms(10); //延时
	}

//	if(ch_wal_now[2]>1300)
//	{
//	//	pwm_left_now=set_pwm_min_max(pwm_left_old,50,900,1500);
//		if(pwm_left_now!=pwm_left_old)
//		{
//			pwm_left_old=pwm_left_now;
//		}
//		delay_ms(10); //延时
//	}
//	else if(ch_wal_now[2]<500)
//	{
//	//	pwm_left_now=set_pwm_min_max(pwm_left_old,-50,900,1500);
//		if(pwm_left_now!=pwm_left_old)
//		{
//			pwm_left_old=pwm_left_now;
//		}
//		delay_ms(10); //延时
//	}

	if(ch_wal_now[2]>1100)
	{
		if(ch_wal_old[2] != ch_wal_now[2])
		{
			if(ch_wal_old[2] - ch_wal_now[2] <-5 || ch_wal_old[2] - ch_wal_now[2] >5)
			{
				pwm_left_now=set_pwm_min_max(1500,-1100,-ch_wal_now[2],1300,1500); 
			}
			ch_wal_old[2] = ch_wal_now[2];
		}
		if(pwm_left_now!=pwm_left_old)
		{
			pwm_left_old=pwm_left_now;
		}
		delay_ms(10); //延时
	}
	else if(ch_wal_now[2]<900)
	{
		if(ch_wal_old[2] != ch_wal_now[2])
		{
			if(ch_wal_old[2] - ch_wal_now[2] <-5 || ch_wal_old[2] - ch_wal_now[2] >5)
			{
				pwm_left_now=set_pwm_min_max(1500,-900,-ch_wal_now[2],1500,1700); 
			}
			ch_wal_old[2] = ch_wal_now[2];
		}
	
		if(pwm_left_now!=pwm_left_old)
		{
			pwm_left_old=pwm_left_now;
		}
		delay_ms(10); //延时
	}
	else
	{
		pwm_left_now=1500;
		if(pwm_left_now!=pwm_left_old)
		{
			pwm_left_old=pwm_left_now;
		}
		delay_ms(10); //延时
	}
}
//减速
void ship_speed_down(int flag)
{
	if(flag==1) //减速
	{
		if(pwm_right_now>1550)
			pwm_right_now-=10;
		else if(pwm_right_now<1550)
			pwm_right_now+=10;
		
		if(pwm_left_now>1450)
			pwm_left_now-=10;
		else if(pwm_left_now<1450)
			pwm_left_now+=10;

  }
	else if (flag==0)
	{
		pwm_left_now=1350;
		pwm_right_now=1650;
	}
}
//转弯
//右转
void turn_right(void)
{
	pwm_left_now=1450;
	pwm_right_now=1500;

}
//左转
void turn_left(void)
{
	pwm_left_now=1500;
	pwm_right_now=1550;

}

//后退
void ship_back(void)
{
	 if(pwm_right_now!=pwm_left_now)
	 {
	 
	 }
	 else
	 {
		
	 }
}

// 定速
void ship_speed_on(void)
{

}
//经纬度 Latitude and longitude
void ship_move(float latx,float longx, float laty,float longy)
{


}
//定位中
void ship_pos_on(int flagposfind,int num_pos)
{
	
}


//饲料机，喷水泵
void motor_control(void)
{
	ch_wal_now[5]=Rc_Sbus_In[5];
	ch_wal_now[6]=Rc_Sbus_In[6];
	
	
	if(ch_wal_now[5]<500)
	{
		if(model==0 && study==0)
		{
		GPIO_SetBits(GPIOE,GPIO_Pin_8);

//		printf("A high 投料关闭\r\n");
		}
		if(study==1)//学习模式
		{
			GPIO_SetBits(GPIOE,GPIO_Pin_8);
		  flag_tl=1; //投料关闭
		}
	}
	else if(ch_wal_now[5]>500)
	{
		if(model==0 && study==0)
		{
		GPIO_ResetBits(GPIOE,GPIO_Pin_8);
//		printf("A low  投料打开\r\n");
		}
		if(study==1)//学习模式
		{
		  if(flag_tl==1) //投料打开
			{
				//添加链表
				GPIO_ResetBits(GPIOE,GPIO_Pin_8);
        InsertLANode(1);
				printf("添加列表\r\n");
				flag_tl=0;
				
			}
		}
	}
	
	if(ch_wal_now[6]<500)
	{
		if(model==0 && study==0)
		{
		GPIO_SetBits(GPIOE,GPIO_Pin_9);
//		printf("B high 喷水关闭\r\n");
		}
		if(study==1)//学习模式
		{
			GPIO_SetBits(GPIOE,GPIO_Pin_9);
		  flag_ps=1; //喷水关闭
		}
	}
	else if(ch_wal_now[6]>500)
	{
		if(model==0 && study==0)
		{
		GPIO_ResetBits(GPIOE,GPIO_Pin_9);
//		printf("B low  喷水打开\r\n");
		}
		if(study==1)
		{
		 if(flag_ps==1) //喷水打开
			{
				//添加链表
				GPIO_ResetBits(GPIOE,GPIO_Pin_9);
				InsertLANode(2);
				printf("添加列表\r\n");
				flag_ps=0;
				
			}
		}
	}
}
/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
函数功能：赋值给PWM寄存器，控制推进器转速与方向
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int servo)
{
	//Forward and reverse control of motor
	//电机正反转控制
	if(motor_a<0)			PWME=1500+motor_a;
	else 	            PWME=1500+motor_a;
	
	//Forward and reverse control of motor
	//电机正反转控制	
	if(motor_b<0)			PWMB=1500+motor_b;
	else 	            PWMB=1500+motor_b;

	//Servo control
	//舵机控制
	//Servo_PWM =servo;
}
/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
函数功能：限制PWM值 
入口参数：幅值
返回  值：无
**************************************************************************/
void Limit_Pwm(int amplitude)
{	
	    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
		  
}	    
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
函数功能：限幅函数
入口参数：幅值
返回  值：无
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}

/**************************************************************************
Function: The remote control command of model aircraft is processed
Input   : none
Output  : none
函数功能：对航模遥控控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Remote_Control(void)
{
	  //Data within 1 second after entering the model control mode will not be processed
	  //对进入航模控制模式后1秒内的数据不处理
    static u8 thrice=100; 
    int Threshold=100; //Threshold to ignore small movements of the joystick //阈值，忽略摇杆小幅度动作

	  //limiter //限幅
    int LX,LY,RY,RX,Remote_RCvelocity; 
	  Remoter_Ch1=Rc_Sbus_In[1];
	  Remoter_Ch2=Rc_Sbus_In[2];
		Remoter_Ch3=Rc_Sbus_In[3];
	  Remoter_Ch4=Rc_Sbus_In[4];
	
	  //这里需要对Remoter_Ch1进行判断，如果没有接收遥控的信号，应该默认输出1500
		Remoter_Ch1=target_limit_int(Remoter_Ch1,500,1500);
		Remoter_Ch2=target_limit_int(Remoter_Ch2,500,1500);
		Remoter_Ch3=target_limit_int(Remoter_Ch3,500,1500);
		Remoter_Ch4=target_limit_int(Remoter_Ch4,500,1500);

	  // Front and back direction of left rocker. Control forward and backward.
	  //左摇杆前后方向。控制前进后退。
    LX=Remoter_Ch3-1000; 
	
	  //Left joystick left and right.Control left and right movement. Only the wheelie omnidirectional wheelie will use the channel.
	  //Ackerman trolleys use this channel as a PWM output to control the steering gear
	  //左摇杆左右方向。控制左右移动。麦轮全向轮才会使用到改通道。阿克曼小车使用该通道作为PWM输出控制舵机
    LY=Remoter_Ch4-1000;

    //Front and back direction of right rocker. Throttle/acceleration/deceleration.
		//右摇杆前后方向。油门/加减速。
	  RX=Remoter_Ch2-1000;

    //Right stick left and right. To control the rotation. 
		//右摇杆左右方向。控制自转。
    RY=Remoter_Ch1-1000; 
		
    //阈值，忽略摇杆小幅度动作
    if(LX>-Threshold&&LX<Threshold)LX=0;
    if(LY>-Threshold&&LY<Threshold)LY=0;
    if(RX>-Threshold&&RX<Threshold)RX=0;
	  if(RY>-Threshold&&RY<Threshold)RY=0;
		
		//Throttle related //油门相关
		Remote_RCvelocity=RC_Velocity+RX;
	  if(Remote_RCvelocity<0)Remote_RCvelocity=0;
		
		//The remote control command of model aircraft is processed
		//对航模遥控控制命令进行处理
    Move_X= LX*Remote_RCvelocity/500; 
		Move_Y=-LY*Remote_RCvelocity/500;
		Move_Z=-RY*(PI/2)/500;      
			 
		//Z轴数据转化
		//if(Car_Mode==Diff_Car)
		//{
			if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
			Move_Z=Move_Z*Remote_RCvelocity/500;
		//}
		
	  //Unit conversion, mm/s -> m/s
    //单位转换，mm/s -> m/s	
		Move_X=Move_X/1000;       
    Move_Y=Move_Y/1000;      
		Move_Z=Move_Z;
		
	  //Data within 1 second after entering the model control mode will not be processed
	  //对进入航模控制模式后1秒内的数据不处理
    if(thrice>0) Move_X=0,Move_Z=0,thrice--;
				
		//Control target value is obtained and kinematics analysis is performed
	  //得到控制目标值，进行运动学分析
		Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
函数功能：运动学逆解，根据三轴目标速度计算各车轮目标转速
入口参数：X和Y、Z轴方向的目标运动速度
返回  值：无
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
		float amplitude=3.5; //Wheel target speed limit //车轮目标速度限幅
	
	  //Speed smoothing is enabled when moving the omnidirectional trolley
	  //全向移动小车才开启速度平滑处理
		
		//Differential car
		//差速小车
		//if (Car_Mode==Diff_Car) 
		//{
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target  = Vx - Vz * Wheel_spacing / 2.0f; //计算出左轮的目标速度
		  MOTOR_B.Target =  Vx + Vz * Wheel_spacing / 2.0f; //计算出右轮的目标速度
			
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		  MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
	    MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 

		//}
		
    //最后需要Set_Pwm控制PWM
	
}


/**************************************************************************
Function: Click the user button to update gyroscope zero
Input   : none
Output  : none
函数功能：单击用户按键更新陀螺仪零点
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{	
//	u8 tmp;
	//tmp=click_N_Double_MPU6050(50); 
//	if(tmp==2)memcpy(Deviation_gyro,Original_gyro,sizeof(gyro)),memcpy(Deviation_accel,Original_accel,sizeof(accel));
}


/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
函数功能：对APP通过串口2发送过来的命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;
	float step=0.003;
	if(car_A_steer_flag==1)
	{
	if(Car_Mode==Mec_Car) //The omnidirectional wheel moving trolley can move laterally //全向轮运动小车可以进行横向移动
	{
	 switch(Flag_Direction)  //Handle direction control commands //处理方向控制命令
	 { 
			case 1:      Move_X=RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 2:      Move_X=RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 3:      Move_X=0;      		     Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 4:      Move_X=-RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1;    break;
			case 5:      Move_X=-RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 6:      Move_X=-RC_Velocity;  	 Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 7:      Move_X=0;     	 		     Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 8:      Move_X=RC_Velocity; 	   Move_Y=RC_Velocity;   Flag_Move=1;    break; 
			default:     Move_X=0;               Move_Y=0;             Flag_Move=0;    break;
	 }
	 if(Flag_Move==0)		
	 {	
		 //If no direction control instruction is available, check the steering control status
		 //如果无方向控制指令，检查转向控制状态
		 if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500); //left rotation  //左自转  
		 else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500); //right rotation //右自转
		 else 		               Move_Z=0;                       //stop           //停止
	 }
	}	
	else //Non-omnidirectional moving trolley //非全向移动小车
	{
	 switch(Flag_Direction) //Handle direction control commands //处理方向控制命令
	 { 
			case 1:      Move_X=+RC_Velocity;  	 Move_Z=0;         break;
			case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/2;   	 break;
			case 3:      Move_X=0;      				 Move_Z=-PI/2;   	 break;	 
			case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/2;     break;		 
			case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;         break;	 
			case 6:      Move_X=-RC_Velocity;  	 Move_Z=+PI/2;     break;	 
			case 7:      Move_X=0;     	 			 	 Move_Z=+PI/2;     break;
			case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/2;     break; 
			default:     Move_X=0;               Move_Z=0;         break;
	 }
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //左自转 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //右自转	
	}
	
 if(Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
	{
	  if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
		Move_Z=Move_Z*RC_Velocity/500;
	}
}	
	else if (car_A_steer_flag==2)
	{
	 switch(Flag_Direction) //Handle direction control commands //处理方向控制命令
	 { 
			case 1:      Moveit_Angle1=Moveit_Angle1+step;    Flag_Move=1;    break;
			case 2:      Moveit_Angle2=Moveit_Angle2+step;    Flag_Move=1;    break;
			case 3:      Moveit_Angle3=Moveit_Angle3+step;    Flag_Move=1;    break;
			case 4:      Moveit_Angle4=Moveit_Angle4+step;    Flag_Move=1;    break;
			case 5:      Moveit_Angle1=Moveit_Angle1-step;    Flag_Move=1;    break;
			case 6:      Moveit_Angle2=Moveit_Angle2-step;    Flag_Move=1;    break;
			case 7:      Moveit_Angle3=Moveit_Angle3-step;    Flag_Move=1;    break;
			case 8:      Moveit_Angle4=Moveit_Angle4-step;    Flag_Move=1;    break;
	 }
		 Moveit_Angle1=target_limit_float(Moveit_Angle1,-1.57,1.57);
		 Moveit_Angle2=target_limit_float(Moveit_Angle2,-1.57,1.57);
		 Moveit_Angle3=target_limit_float(Moveit_Angle3,-1.57,1.57);
		 Moveit_Angle4=target_limit_float(Moveit_Angle4,-1.57,1.57);
		
	}
	//Unit conversion, mm/s -> m/s
  //单位转换，mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//Control target value is obtained and kinematics analysis is performed
	//得到控制目标值，进行运动学分析
	Drive_Motor(Move_X,Move_Y,Move_Z);
}


void ship_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//使能GPIOA,GPIOE时钟
 
  //GPIOB8,B9初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
 // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化
	GPIO_SetBits(GPIOE,GPIO_Pin_8 | GPIO_Pin_9);
	
	
}
