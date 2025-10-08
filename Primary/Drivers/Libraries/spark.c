#include "spark.h"

DataPacket_t spark_rx_buffer;

DataPacket_t spark_data;

void ProcessSparkData(DataPacket_t *packet) {
  // Extract SPARK-specific data from the received packet
  spark_data.Packet_ID = PACKET_ID_SPARK;
  spark_data.timestamp = packet->timestamp;
  spark_data.Data.spark.magAngle = packet->Data.spark.magAngle;
  spark_data.Data.spark.posDeviation = packet->Data.spark.posDeviation;
  spark_data.Data.spark.voltage_driver = packet->Data.spark.voltage_driver;
  spark_data.Data.spark.temperature_NTC1 = packet->Data.spark.temperature_NTC1;
  spark_data.Data.spark.temperature_NTC2 = packet->Data.spark.temperature_NTC2;
  spark_data.Data.spark.sparkStatus = packet->Data.spark.sparkStatus;
  spark_data.crc = packet->crc;
}

void spark_sendCommand(DataPacket_t *packet)
{
  // Send the command over SPI
  
  //taskENTER_CRITICAL();
  HAL_GPIO_WritePin(EXT2_CS_GPIO_Port, EXT2_CS_Pin, GPIO_PIN_RESET); // Activate CS
  HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)packet, (uint8_t *)&spark_rx_buffer, sizeof(DataPacket_t), HAL_MAX_DELAY);
  HAL_GPIO_WritePin(EXT2_CS_GPIO_Port, EXT2_CS_Pin, GPIO_PIN_SET); // Deactivate CS
  //taskEXIT_CRITICAL();

  // Process the received SPARK data
  if (spark_rx_buffer.Packet_ID == PACKET_ID_SPARK) {
    ProcessSparkData(&spark_rx_buffer);
  }
}
