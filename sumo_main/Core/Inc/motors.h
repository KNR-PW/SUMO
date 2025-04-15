#ifndef MOTORS_H
#define MOTORS_H

#include "main.h"
#include <stdbool.h>

//this values should be configured based on hardware GPIO connections
//if you switched the pins that controls digestion of motor switch them
//this should makes motors rotates the same way as before
//	(switch DIR_MOT_1 with DIR_MOR_2 pins)
#define RIGTH_MOTOR 1
#define LEFT_MOTOR 2

#define FORWARD true
#define BACKWARDS false

//tim 12 handler
extern TIM_HandleTypeDef htim12;

//sets the speed of chosen motor
void move_at_speed(int speed,int motor_id);

//checks if the current diretion for the motors (the bool parameters) and sets the control output
//pins pase on this
void sumoUpdateDirection();

//changes rotation direction of chosen motor
void change_direction(int motor_id);

//changes rotation direction of chosen motor
void motorDirection(bool direction, int motor_id);

#endif //MOTORS_H
