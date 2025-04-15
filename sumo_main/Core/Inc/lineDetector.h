#ifndef LINE_DETECTOR_H
#define LINE_DETECTOR_H

#include "main.h"

#define LINE_SENSOR_1 1
#define LINE_SENSOR_2 2
#define LINE_SENSOR_3 3
#define LINE_SENSOR_4 4

//ADC handler
extern ADC_HandleTypeDef hadc1;

//resturns value form 0 to ~2500 the bigger the value the darker the object
uint32_t LineDetectorMessure(uint8_t LineSensor_ID);

#endif //LIDAR_I2C_H
