#include "lineDetector.h"

uint32_t LineDetectorMessure(uint8_t LineSensor_ID)
{
	switch (LineSensor_ID) {
		case LINE_SENSOR_1:
			ADC_SetActiveChannel(&hadc1, ADC_CHANNEL_5);
			break;
		case LINE_SENSOR_2:
			ADC_SetActiveChannel(&hadc1, ADC_CHANNEL_10);
			break;
		case LINE_SENSOR_3:
			ADC_SetActiveChannel(&hadc1, ADC_CHANNEL_11);
			break;
		case LINE_SENSOR_4:
			ADC_SetActiveChannel(&hadc1, ADC_CHANNEL_15);
			break;
		default:
			break;
	}
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint32_t value = HAL_ADC_GetValue(&hadc1);
}
