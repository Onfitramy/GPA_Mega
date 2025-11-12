#include "spark.h"
#include "cli_app.h"
#include "InterBoardCom.h"

DataPacket_t spark_rx_buffer;

DataPacket_t spark_data;

void ProcessSPARKData(DataPacket_t *packet) {
    if (packet->crc != getCRC(packet)) {
        // CRC mismatch, discard packet
        return;
    }
    // Extract SPARK-specific data from the received packet
    spark_data.Packet_ID = PACKET_ID_SPARK;
    spark_data.timestamp = packet->timestamp;
    spark_data.Data.spark.magAngle = packet->Data.spark.magAngle;
    spark_data.Data.spark.magSpeed = packet->Data.spark.magSpeed;
    spark_data.Data.spark.posDeviation = packet->Data.spark.posDeviation;
    spark_data.Data.spark.voltage_driver = packet->Data.spark.voltage_driver;
    spark_data.Data.spark.temperature_driver = packet->Data.spark.temperature_driver;
    spark_data.Data.spark.temperature_converter = packet->Data.spark.temperature_converter;
    spark_data.Data.spark.sparkStatus = packet->Data.spark.sparkStatus;
    spark_data.crc = packet->crc;
}

void SPARK_sendCommand(DataPacket_t *packet)
{
    if (cli_target_mode == CLI_TARGET_MODE_EXTERNAL) {
        // Send command to other board via InterBoardCom
        InterBoardCom_SendDataPacket(INTERBOARD_OP_CMD | INTERBOARD_TARGET_RADIO, packet);
        return;
    }
    // Send the command over SPI
  
    //taskENTER_CRITICAL();
    HAL_GPIO_WritePin(EXT2_CS_GPIO_Port, EXT2_CS_Pin, GPIO_PIN_RESET); // Activate CS
    HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)packet, (uint8_t *)&spark_rx_buffer, sizeof(DataPacket_t), HAL_MAX_DELAY);
    HAL_GPIO_WritePin(EXT2_CS_GPIO_Port, EXT2_CS_Pin, GPIO_PIN_SET); // Deactivate CS
    //taskEXIT_CRITICAL();

    // Process the received SPARK data
    if (spark_rx_buffer.Packet_ID == PACKET_ID_SPARK) {
        ProcessSPARKData(&spark_rx_buffer);
    }
}

void SPARK_ReadData() {
    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SPARK, COMMAND_ID_SPARK_READ_DATA, NULL, 0);
    SPARK_sendCommand(&packet);
}

/* SPARK Commands */

void SPARK_SetAngle(float angle_deg) {
    uint8_t parameters[sizeof(float)];
    memcpy(parameters, &angle_deg, sizeof(float));
    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SPARK, COMMAND_ID_SPARK_SET_ANGLE, parameters, sizeof(parameters));
    SPARK_sendCommand(&packet);
}

void SPARK_SetSpeed(float speed_deg_s) {
    uint8_t parameters[sizeof(float)];
    memcpy(parameters, &speed_deg_s, sizeof(float));
    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SPARK, COMMAND_ID_SPARK_SET_SPEED, parameters, sizeof(parameters));
    SPARK_sendCommand(&packet);
}

void SPARK_ExitMode() {
    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SPARK, COMMAND_ID_SPARK_EXIT_MODE, NULL, 0);
    SPARK_sendCommand(&packet);
}

void SPARK_ZeroStepper() {
    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SPARK, COMMAND_ID_SPARK_ZERO_STEPPER, NULL, 0);
    SPARK_sendCommand(&packet);
}

void SPARK_FindMax() {
    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SPARK, COMMAND_ID_SPARK_FIND_MAX, NULL, 0);
    SPARK_sendCommand(&packet);
}

void SPARK_TargetPositionMode(uint8_t torque_16) {
    uint8_t parameters[1];
    parameters[0] = torque_16;
    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SPARK, COMMAND_ID_SPARK_MODE_TARGET_POSITION, parameters, sizeof(parameters));
    SPARK_sendCommand(&packet);
}

void SPARK_TargetSpeedMode(uint8_t torque_16) {
    uint8_t parameters[1];
    parameters[0] = torque_16;
    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SPARK, COMMAND_ID_SPARK_MODE_TARGET_SPEED, parameters, sizeof(parameters));
    SPARK_sendCommand(&packet);
}

void SPARK_Reset() {
    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SPARK, COMMAND_ID_SPARK_RESET, NULL, 0);
    SPARK_sendCommand(&packet);
}

void ACS_SetAngle(float angle_deg) {
    if (cli_target_mode == CLI_TARGET_MODE_EXTERNAL) {
        // Forward packet with ACS target angle
        uint8_t parameters[sizeof(float)];
        memcpy(parameters, &angle_deg, sizeof(float));
        DataPacket_t packet;
        CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SPARK, COMMAND_ID_SPARK_SET_ACS_ANGLE, parameters, sizeof(parameters));
        SPARK_sendCommand(&packet);
        return;
    }
    // convert ACS angle to stepper position and set SPARK angle
    acs_target_angle_deg = angle_deg;
    StepperPositionFromACSAngle(acs_target_angle_deg, &stepper_target_position);
    StepperAngleFromPosition(stepper_target_position, stepper_zero_position, &stepper_target_angle_deg);
    SPARK_SetAngle(stepper_target_angle_deg);
}