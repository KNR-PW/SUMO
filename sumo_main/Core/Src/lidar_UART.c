#include "lidar_UART.h"

uint16_t readLidar4()
{
	uint8_t value[8];  // Buffer for LiDAR data
	uint8_t command[] = {0x55, 0xAA, 0x81, 0x00, 0xFA};
	HAL_UART_Transmit(&huart3, command, sizeof(command), HAL_MAX_DELAY);
	if (HAL_UART_Receive(&huart3, value, 8, 10) == HAL_OK){
		if ((value[0] == 0x55) && (value[1] == 0xAA) &&
			(value[2] == 0x81) && (value[7] == 0xFA))
		{
		  if (value[3] == 3)  // Sprawdzamy, czy długość danych wynosi 3 bajty
		  {
			// Parsowanie odległości: 2 bajty (big-endian)
			uint16_t distance = (value[4] << 8) | value[5];
			uint8_t status = value[6];
			return distance;
		  }else return 0xFFFF;
		}
		else return 0xFFFF;
	}else return 0xFFFF;
}
