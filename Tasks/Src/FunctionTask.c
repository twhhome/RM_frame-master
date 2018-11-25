/**
  ******************************************************************************
  * File Name          : FunctionTask.c
  * Description        : 用于记录机器人独有的功能
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

float rotate_speed = 0;
KeyboardMode_e KeyboardMode = NO_CHANGE;
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   	//斜坡函数
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;
ChassisSpeed_Ref_t ChassisSpeedRef; 

int32_t auto_counter=0;		//用于准确延时的完成某事件

int16_t channelrrow = 0;
int16_t channelrcol = 0;
int16_t channellrow = 0;
int16_t channellcol = 0;

int flag = 0;

void delay_1us(uint16_t x)//粗略延时
{
	uint8_t i=3;
	x=i*x;
	while(x--);
}

//初始化
void FunctionTaskInit()
{
	LRSpeedRamp.SetScale(&LRSpeedRamp, MOUSE_LR_RAMP_TICK_COUNT);
	FBSpeedRamp.SetScale(&FBSpeedRamp, MOUSR_FB_RAMP_TICK_COUNT);
	LRSpeedRamp.ResetCounter(&LRSpeedRamp);
	FBSpeedRamp.ResetCounter(&FBSpeedRamp);
	
	ChassisSpeedRef.forward_back_ref = 0.0f;
	ChassisSpeedRef.left_right_ref = 0.0f;
	ChassisSpeedRef.rotate_ref = 0.0f;
	
	KeyboardMode=NO_CHANGE;
}

void Limit_and_Synchronization()
{
	//demo
	MINMAX(UD1.TargetAngle,-900,270);//limit
	UD2.TargetAngle=-UD1.TargetAngle;//sychronization
	//demo end
}

//******************
//遥控器模式功能编写
//******************
void RemoteControlProcess(Remote *rc)
{
	if(WorkState <= 0) return;
	//max=297
	channelrrow = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channelrcol = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellrow = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellcol = (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	if(WorkState == NORMAL_STATE) // up
	{	
		// 上下
		if(channelrcol > 300) { // 上
			if(HAL_GPIO_ReadPin(GATE_2_GPIO_Port, GATE_2_Pin) != GPIO_PIN_SET) {
				HAL_GPIO_WritePin(INA_GPIO_Port, INA_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(INB_GPIO_Port, INB_Pin, GPIO_PIN_RESET);
			}
		} else if(channelrcol < -300) { // 下
			if(HAL_GPIO_ReadPin(GATE_1_GPIO_Port, GATE_1_Pin) != GPIO_PIN_SET) {
				HAL_GPIO_WritePin(INA_GPIO_Port, INA_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(INB_GPIO_Port, INB_Pin, GPIO_PIN_SET);
			}
		} else {
			// 电机不转
			HAL_GPIO_WritePin(INA_GPIO_Port, INA_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(INB_GPIO_Port, INB_Pin, GPIO_PIN_RESET);
		}
		
		// 前后
		if(channelrrow > 300) { // 前
			if(HAL_GPIO_ReadPin(GATE_4_GPIO_Port, GATE_4_Pin) != GPIO_PIN_SET) {
				HAL_GPIO_WritePin(INC_GPIO_Port, INC_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IND_GPIO_Port, IND_Pin, GPIO_PIN_RESET);
			}
		} else if(channelrrow < -300) { // 后
			if(HAL_GPIO_ReadPin(GATE_3_GPIO_Port, GATE_3_Pin) != GPIO_PIN_SET) {
				HAL_GPIO_WritePin(INC_GPIO_Port, INC_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IND_GPIO_Port, IND_Pin, GPIO_PIN_SET);
			}
		} else {
			// 电机不转
			HAL_GPIO_WritePin(INC_GPIO_Port, INC_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IND_GPIO_Port, IND_Pin, GPIO_PIN_RESET);
		}
		
		// 舵机1(机械爪)
		if(channellcol > 300) { // 打开机械爪
			HAL_TIM_PWM_Start(SERVO_TIM, SERVO_1_CHANNEL);
			__HAL_TIM_SET_COMPARE(SERVO_TIM, SERVO_1_CHANNEL, SERVO_1_OPEN);
		} else if(channellcol < -300) { // 关闭机械爪
			HAL_TIM_PWM_Start(SERVO_TIM, SERVO_1_CHANNEL);
			__HAL_TIM_SET_COMPARE(SERVO_TIM, SERVO_1_CHANNEL, SERVO_1_CLOSE);
		}
		
		// 钩子
		if(rc->s2 == 1) { // 右开关在上状态
			HAL_TIM_PWM_Start(SERVO_TIM, SERVO_3_CHANNEL);
			__HAL_TIM_SET_COMPARE(SERVO_TIM, SERVO_3_CHANNEL, SERVO_3_OPEN);
		} else if(rc->s2 == 3) { // 右开关在中间状态
			HAL_TIM_PWM_Start(SERVO_TIM, SERVO_3_CHANNEL);
			__HAL_TIM_SET_COMPARE(SERVO_TIM, SERVO_3_CHANNEL, SERVO_3_CLOSE);
		}
	}
	if(WorkState == ADDITIONAL_STATE_ONE) // mid
	{
		ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
		ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF/2;
		rotate_speed = -channellrow * RC_ROTATE_SPEED_REF;
		
		// 电机停转
		HAL_GPIO_WritePin(INA_GPIO_Port, INA_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(INB_GPIO_Port, INB_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(INC_GPIO_Port, INC_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IND_GPIO_Port, IND_Pin, GPIO_PIN_RESET);
		
		// 舵机1(机械爪)
		if(channellcol > 300) { // 打开机械爪
			HAL_TIM_PWM_Start(SERVO_TIM, SERVO_1_CHANNEL);
			__HAL_TIM_SET_COMPARE(SERVO_TIM, SERVO_1_CHANNEL, SERVO_1_OPEN);
		} else if(channellcol < -300) { // 关闭机械爪
			HAL_TIM_PWM_Start(SERVO_TIM, SERVO_1_CHANNEL);
			__HAL_TIM_SET_COMPARE(SERVO_TIM, SERVO_1_CHANNEL, SERVO_1_CLOSE);
		}
		
		// 钩子
		if(rc->s2 == 1) { // 右开关在上状态
			HAL_TIM_PWM_Start(SERVO_TIM, SERVO_3_CHANNEL);
			__HAL_TIM_SET_COMPARE(SERVO_TIM, SERVO_3_CHANNEL, SERVO_3_OPEN);
		} else if(rc->s2 == 3) { // 右开关在中间状态
			HAL_TIM_PWM_Start(SERVO_TIM, SERVO_3_CHANNEL);
			__HAL_TIM_SET_COMPARE(SERVO_TIM, SERVO_3_CHANNEL, SERVO_3_CLOSE);
		}
	}
	if(WorkState == ADDITIONAL_STATE_TWO) // down
	{
		ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
		ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF/2;
		rotate_speed = -channellrow * RC_ROTATE_SPEED_REF;
		
		// 电机停转
		HAL_GPIO_WritePin(INA_GPIO_Port, INA_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(INB_GPIO_Port, INB_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(INC_GPIO_Port, INC_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IND_GPIO_Port, IND_Pin, GPIO_PIN_RESET);
		
		// 舱门
		if(channellcol > 300) {
			HAL_TIM_PWM_Start(SERVO_TIM, SERVO_2_CHANNEL);
			__HAL_TIM_SET_COMPARE(SERVO_TIM, SERVO_2_CHANNEL, SERVO_2_CLOSE);
		} else if(channellcol < -300) {
			HAL_TIM_PWM_Start(SERVO_TIM, SERVO_2_CHANNEL);
			__HAL_TIM_SET_COMPARE(SERVO_TIM, SERVO_2_CHANNEL, SERVO_2_OPEN);
		}
		
		// 钩子
		if(rc->s2 == 1) { // 右开关在上状态
			HAL_TIM_PWM_Start(SERVO_TIM, SERVO_3_CHANNEL);
			__HAL_TIM_SET_COMPARE(SERVO_TIM, SERVO_3_CHANNEL, SERVO_3_OPEN);
		} else if(rc->s2 == 3) { // 右开关在中间状态
			HAL_TIM_PWM_Start(SERVO_TIM, SERVO_3_CHANNEL);
			__HAL_TIM_SET_COMPARE(SERVO_TIM, SERVO_3_CHANNEL, SERVO_3_CLOSE);
		}
	}
	Limit_and_Synchronization();
}


uint16_t KM_FORWORD_BACK_SPEED 	= NORMAL_FORWARD_BACK_SPEED;
uint16_t KM_LEFT_RIGHT_SPEED  	= NORMAL_LEFT_RIGHT_SPEED;
void KeyboardModeFSM(Key *key);

//****************
//键鼠模式功能编写
//****************
void MouseKeyControlProcess(Mouse *mouse, Key *key)
{	
	if(WorkState <= 0) return;
	
	MINMAX(mouse->x, -150, 150); 
	MINMAX(mouse->y, -150, 150); 

	KeyboardModeFSM(key);
	
	switch (KeyboardMode)
	{
		case SHIFT_CTRL:		//State control
		{
			
			break;
		}
		case CTRL:				//slow
		{
			
		}//DO NOT NEED TO BREAK
		case SHIFT:				//quick
		{
			
		}//DO NOT NEED TO BREAK
		case NO_CHANGE:			//normal
		{//CM Movement Process
			if(key->v & KEY_W)  		//key: w
				ChassisSpeedRef.forward_back_ref =  KM_FORWORD_BACK_SPEED* FBSpeedRamp.Calc(&FBSpeedRamp);
			else if(key->v & KEY_S) 	//key: s
				ChassisSpeedRef.forward_back_ref = -KM_FORWORD_BACK_SPEED* FBSpeedRamp.Calc(&FBSpeedRamp);
			else
			{
				ChassisSpeedRef.forward_back_ref = 0;
				FBSpeedRamp.ResetCounter(&FBSpeedRamp);
			}
			if(key->v & KEY_D)  		//key: d
				ChassisSpeedRef.left_right_ref =  KM_LEFT_RIGHT_SPEED * LRSpeedRamp.Calc(&LRSpeedRamp);
			else if(key->v & KEY_A) 	//key: a
				ChassisSpeedRef.left_right_ref = -KM_LEFT_RIGHT_SPEED * LRSpeedRamp.Calc(&LRSpeedRamp);
			else
			{
				ChassisSpeedRef.left_right_ref = 0;
				LRSpeedRamp.ResetCounter(&LRSpeedRamp);
			}
		}
	}
	Limit_and_Synchronization();
}

void KeyboardModeFSM(Key *key)
{
	if((key->v & 0x30) == 0x30)//Shift_Ctrl
	{
		KM_FORWORD_BACK_SPEED=  LOW_FORWARD_BACK_SPEED;
		KM_LEFT_RIGHT_SPEED = LOW_LEFT_RIGHT_SPEED;
		KeyboardMode=SHIFT_CTRL;
	}
	else if(key->v & KEY_SHIFT)//Shift
	{
		KM_FORWORD_BACK_SPEED=  HIGH_FORWARD_BACK_SPEED;
		KM_LEFT_RIGHT_SPEED = HIGH_LEFT_RIGHT_SPEED;
		KeyboardMode=SHIFT;
	}
	else if(key->v & KEY_CTRL)//Ctrl
	{
		KM_FORWORD_BACK_SPEED=  LOW_FORWARD_BACK_SPEED;
		KM_LEFT_RIGHT_SPEED = LOW_LEFT_RIGHT_SPEED;
		KeyboardMode=CTRL;
	}
	else
	{
		KM_FORWORD_BACK_SPEED=  NORMAL_FORWARD_BACK_SPEED;
		KM_LEFT_RIGHT_SPEED = NORMAL_LEFT_RIGHT_SPEED;
		KeyboardMode=NO_CHANGE;
	}	
}
