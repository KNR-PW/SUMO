#include "motors.h"


//this values if set to yes means that motor is rotating the way that robot will move forward
bool leftMotorForward = true;
bool rightMotorForward = true;
//-------------------------------------------------------------
//"technical functions" should not be modified unless reasonable reason
void move_at_speed(int speed,int motor_id)
{
	if(speed > 100)speed = 100;
	switch(motor_id)
		{
		case 1:
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, speed);
			break;
		case 2:
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, speed);
			break;
		}
}


void sumoUpdateDirection()
{
	if(LEFT_MOTOR == 1 && RIGTH_MOTOR == 2)
	{
		if(leftMotorForward)
		{
			HAL_GPIO_WritePin(DIR_MOT_1_L_GPIO_Port, DIR_MOT_1_L_Pin,SET);
			HAL_GPIO_WritePin(DIR_MOT_1_R_GPIO_Port, DIR_MOT_1_R_Pin,RESET);
		}
		else{
			HAL_GPIO_WritePin(DIR_MOT_1_L_GPIO_Port, DIR_MOT_1_L_Pin,RESET);
			HAL_GPIO_WritePin(DIR_MOT_1_R_GPIO_Port, DIR_MOT_1_R_Pin,SET);
		}
		if(rightMotorForward)
		{
			HAL_GPIO_WritePin(DIR_MOT_2_L_GPIO_Port, DIR_MOT_2_L_Pin,SET);
			HAL_GPIO_WritePin(DIR_MOT_2_R_GPIO_Port, DIR_MOT_2_R_Pin,RESET);
		}
		else{
			HAL_GPIO_WritePin(DIR_MOT_2_L_GPIO_Port, DIR_MOT_2_L_Pin,RESET);
			HAL_GPIO_WritePin(DIR_MOT_2_R_GPIO_Port, DIR_MOT_2_R_Pin,SET);
		}
	}

	else if(LEFT_MOTOR == 2 && RIGTH_MOTOR == 1)
	{
		if(leftMotorForward)
		{
			HAL_GPIO_WritePin(DIR_MOT_2_L_GPIO_Port, DIR_MOT_2_L_Pin,SET);
			HAL_GPIO_WritePin(DIR_MOT_2_R_GPIO_Port, DIR_MOT_2_R_Pin,RESET);
		}
		else{
			HAL_GPIO_WritePin(DIR_MOT_2_L_GPIO_Port, DIR_MOT_2_L_Pin,RESET);
			HAL_GPIO_WritePin(DIR_MOT_2_R_GPIO_Port, DIR_MOT_2_R_Pin,SET);
		}
		if(rightMotorForward)
		{
			HAL_GPIO_WritePin(DIR_MOT_1_L_GPIO_Port, DIR_MOT_1_L_Pin,SET);
			HAL_GPIO_WritePin(DIR_MOT_1_R_GPIO_Port, DIR_MOT_1_R_Pin,RESET);
		}
		else{
			HAL_GPIO_WritePin(DIR_MOT_1_L_GPIO_Port, DIR_MOT_1_L_Pin,RESET);
			HAL_GPIO_WritePin(DIR_MOT_1_R_GPIO_Port, DIR_MOT_1_R_Pin,SET);
		}
	}
}

void change_direction(int motor_id)
{
	if(LEFT_MOTOR == 1 && RIGTH_MOTOR == 2)
		switch (motor_id) {
			case 1:
				leftMotorForward = !leftMotorForward;
				break;
			case 2:
				rightMotorForward = !rightMotorForward;
			default:
				break;
		}
	else if(LEFT_MOTOR == 2 && RIGTH_MOTOR == 1){
		switch (motor_id) {
			case 1:
				rightMotorForward = !rightMotorForward;
				break;
			case 2:
				leftMotorForward = !leftMotorForward;
			default:
				break;
		}
	}
	sumoUpdateDirection();
}

void motorDirection(bool direction, int motor_id)
{
	switch (motor_id) {
		case 1:
			if(LEFT_MOTOR == 1 && RIGTH_MOTOR == 2){
				leftMotorForward = direction;
			}
			else if(LEFT_MOTOR == 2 && RIGTH_MOTOR == 1){
				rightMotorForward = direction;
			}
			break;
		case 2:
			if(LEFT_MOTOR == 1 && RIGTH_MOTOR == 2){
				rightMotorForward = direction;
			}
			else if(LEFT_MOTOR == 2 && RIGTH_MOTOR == 1){
				leftMotorForward = direction;
			}
			break;
		default:
			break;
	}
	sumoUpdateDirection();
}


//--------------------------------------------------------------------------------------
//the user API
void moveForward()
{
	leftMotorForward = true;
	rightMotorForward = true;
	sumoUpdateDirection();
}

void moveBackwards()
{
	leftMotorForward = false;
	rightMotorForward = false;
	sumoUpdateDirection();
}

void turnLeft()
{
	leftMotorForward = false;
	rightMotorForward = true;
	sumoUpdateDirection();
}

void turnRight()
{
	leftMotorForward = true;
	rightMotorForward = false;
	sumoUpdateDirection();
}
