#ifndef LIDAR_UART_H
#define LIDAR_UART_H

#include "main.h"

//uart handler
extern UART_HandleTypeDef huart3;

//read data from lidar (using uart protocol)
//if some flags occurs or the connection isnt established returns 0xFFFF
uint16_t readLidar4();

#endif //LIDAR_UART_H
