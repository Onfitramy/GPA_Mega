#include "spark.h"

void spark_sendCommand(DataPacket_t *packet)
{
  // Send the command over SPI
  HAL_GPIO_WritePin(EXT2_CS_GPIO_Port, EXT2_CS_Pin, GPIO_PIN_RESET); // Activate CS
  HAL_SPI_Transmit(&hspi2, (uint8_t *)packet, sizeof(DataPacket_t), HAL_MAX_DELAY);
  HAL_GPIO_WritePin(EXT2_CS_GPIO_Port, EXT2_CS_Pin, GPIO_PIN_SET); // Deactivate CS
}
