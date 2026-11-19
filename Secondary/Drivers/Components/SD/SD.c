#include "SD.h"
#include "fatfs.h"
#include <stdio.h>
#include <string.h>

FATFS FatFs; 	//Fatfs handle
FIL fil; 		//File handle
FRESULT fres; //Result after operations

DWORD free_clusters, free_sectors, total_sectors;

#define BUFFER_SIZE 3072 // 3*1024 bytes,

uint8_t sd_buffer1[BUFFER_SIZE]; // Buffer for SD operations
uint8_t sd_buffer2[BUFFER_SIZE]; // Using double buffering
uint8_t* sd_buffer = sd_buffer1; // Pointer to current buffer
uint32_t sd_buffer_index = 0; // Current index in the buffer

uint8_t SD_SaveBuffer(char *filename){
  uint32_t write_size = sd_buffer_index;
  sd_buffer_index = 0; // Reset buffer index
  uint8_t* save_buffer =  sd_buffer; // Pointer to current buffer, which will be saved
  sd_buffer = (sd_buffer == sd_buffer1) ? sd_buffer2 : sd_buffer1; // Switch buffers

  if (write_size == 0) {
    return 1; // Nothing to save
  }
  FRESULT open = SD_Open(filename, FA_WRITE | FA_OPEN_APPEND);
  FRESULT write = SD_Write(save_buffer, write_size);
  FRESULT close = SD_Close();
  return 0; // Success
}

uint8_t SD_AppendDataPacketToBuffer(DataPacket_t* packet) {
    char text_buffer[384] = {0}; // Temporary buffer for saving SD text
    uint32_t text_size = 0;

    if (packet == NULL) {
      return 1; // Error: Null pointer
    }

    int length;
    switch(packet->Packet_ID) {
    case PACKET_ID_IMU:
        length = snprintf(text_buffer, sizeof(text_buffer), "\nID:%d, TS:%lu, accelX:%ld, accelY:%ld, accelZ:%ld, gyroX:%d, gyroY:%d, gyroZ:%d, magX:%d, magY:%d, magZ:%d, flight_state:%u",
                (int)packet->Packet_ID, (unsigned long)packet->timestamp,
                (long)packet->Data.imu.accelX,
    (long)packet->Data.imu.accelY,
    (long)packet->Data.imu.accelZ,
    (int)packet->Data.imu.gyroX,
    (int)packet->Data.imu.gyroY,
    (int)packet->Data.imu.gyroZ,
    (int)packet->Data.imu.magX,
    (int)packet->Data.imu.magY,
    (int)packet->Data.imu.magZ,
    (unsigned int)packet->Data.imu.flight_state);
        break;
    case PACKET_ID_ATTITUDE:
        length = snprintf(text_buffer, sizeof(text_buffer), "\nID:%d, TS:%lu, phi:%.9g, theta:%.9g, psi:%.9g, gyro_biasX:%.9g, gyro_biasY:%.9g, gyro_biasZ:%.9g",
                (int)packet->Packet_ID, (unsigned long)packet->timestamp,
                (double)packet->Data.attitude.phi,
    (double)packet->Data.attitude.theta,
    (double)packet->Data.attitude.psi,
    (double)packet->Data.attitude.gyro_biasX,
    (double)packet->Data.attitude.gyro_biasY,
    (double)packet->Data.attitude.gyro_biasZ);
        break;
    case PACKET_ID_GPS:
        length = snprintf(text_buffer, sizeof(text_buffer), "\nID:%d, TS:%lu, latitude:%ld, longitude:%ld, altitude:%ld, EKF2_Heigth:%.9g, EKF2_vel:%.9g, EKF2_refPres:%.9g, numSats:%u, gpsFix:%u",
                (int)packet->Packet_ID, (unsigned long)packet->timestamp,
                (long)packet->Data.gps.latitude,
    (long)packet->Data.gps.longitude,
    (long)packet->Data.gps.altitude,
    (double)packet->Data.gps.EKF2_Heigth,
    (double)packet->Data.gps.EKF2_vel,
    (double)packet->Data.gps.EKF2_refPres,
    (unsigned int)packet->Data.gps.numSats,
    (unsigned int)packet->Data.gps.gpsFix);
        break;
    case PACKET_ID_POSITION:
        length = snprintf(text_buffer, sizeof(text_buffer), "\nID:%d, TS:%lu, posX:%ld, posY:%ld, posZ:%ld, velX:%ld, velY:%ld, velZ:%ld",
                (int)packet->Packet_ID, (unsigned long)packet->timestamp,
                (long)packet->Data.position.posX,
    (long)packet->Data.position.posY,
    (long)packet->Data.position.posZ,
    (long)packet->Data.position.velX,
    (long)packet->Data.position.velY,
    (long)packet->Data.position.velZ);
        break;
    case PACKET_ID_POWER:
        length = snprintf(text_buffer, sizeof(text_buffer), "\nID:%d, TS:%lu, M1_5V_bus:%lu, M1_BAT_bus_volt:%lu, M2_bus_5V:%u, M2_bus_GPA_bat_volt:%u, PU_bat_bus_volt:%u, PU_curr:%u, PU_pow:%u, pu_state:%u",
                (int)packet->Packet_ID, (unsigned long)packet->timestamp,
                (unsigned long)packet->Data.power.M1_5V_bus,
    (unsigned long)packet->Data.power.M1_BAT_bus_volt,
    (unsigned int)packet->Data.power.M2_bus_5V,
    (unsigned int)packet->Data.power.M2_bus_GPA_bat_volt,
    (unsigned int)packet->Data.power.PU_bat_bus_volt,
    (unsigned int)packet->Data.power.PU_curr,
    (unsigned int)packet->Data.power.PU_pow,
    (unsigned int)packet->Data.power.pu_state);
        break;
    case PACKET_ID_TEMPERATURE:
        length = snprintf(text_buffer, sizeof(text_buffer), "\nID:%d, TS:%lu, M1_ADC:%d, M1_BMP:%d, M1_DTS:%d, M1_IMU1:%d, M1_MAG:%d, M2_3V3:%d, M2_XBee:%d, M1_IMU2:%d, PU_bat:%d, pressure_static:%.9g, state_timestamp:%lu",
                (int)packet->Packet_ID, (unsigned long)packet->timestamp,
                (int)packet->Data.temperature.M1_ADC,
    (int)packet->Data.temperature.M1_BMP,
    (int)packet->Data.temperature.M1_DTS,
    (int)packet->Data.temperature.M1_IMU1,
    (int)packet->Data.temperature.M1_MAG,
    (int)packet->Data.temperature.M2_3V3,
    (int)packet->Data.temperature.M2_XBee,
    (int)packet->Data.temperature.M1_IMU2,
    (int)packet->Data.temperature.PU_bat,
    (double)packet->Data.temperature.pressure_static,
    (unsigned long)packet->Data.temperature.state_timestamp);
        break;
    case PACKET_ID_STATUS:
        length = snprintf(text_buffer, sizeof(text_buffer), "\nID:%d, TS:%lu, status_flags:%ld, sensor_status_flags:%ld, error_flags:%ld, State:%d",
                (int)packet->Packet_ID, (unsigned long)packet->timestamp,
                (long)packet->Data.status.status_flags,
    (long)packet->Data.status.sensor_status_flags,
    (long)packet->Data.status.error_flags,
    (int)packet->Data.status.State);
        break;
    case PACKET_ID_KALMANMATRIX:
        length = snprintf(text_buffer, sizeof(text_buffer), "\nID:%d, TS:%lu, P11:%.9g, P22:%.9g, P33:%.9g",
                (int)packet->Packet_ID, (unsigned long)packet->timestamp,
                (double)packet->Data.kalman.P11,
    (double)packet->Data.kalman.P22,
    (double)packet->Data.kalman.P33);
        break;
    case PACKET_ID_MPC_INFO:
        length = snprintf(text_buffer, sizeof(text_buffer), "\nID:%d, TS:%lu, MPC_freq:%.9g, Airbreak_deflection:%.9g, Target_Angle:%.9g, t_solve:%.9g, t_setup:%.9g, num_iterations:%u, exit_code:%u",
                (int)packet->Packet_ID, (unsigned long)packet->timestamp,
                (double)packet->Data.mpc_info.MPC_freq,
    (double)packet->Data.mpc_info.Airbreak_deflection,
    (double)packet->Data.mpc_info.Target_Angle,
    (double)packet->Data.mpc_info.t_solve,
    (double)packet->Data.mpc_info.t_setup,
    (unsigned int)packet->Data.mpc_info.num_iterations,
    (unsigned int)packet->Data.mpc_info.exit_code);
        break;
    case PACKET_ID_SPARK:
        length = snprintf(text_buffer, sizeof(text_buffer), "\nID:%d, TS:%lu, magAngle:%.9g, magSpeed:%.9g, posDeviation:%.9g, voltage_driver:%.9g, temperature_driver:%.9g, temperature_converter:%.9g, sparkStatus:%u, as5600_status:%u",
                (int)packet->Packet_ID, (unsigned long)packet->timestamp,
                (double)packet->Data.spark.magAngle,
    (double)packet->Data.spark.magSpeed,
    (double)packet->Data.spark.posDeviation,
    (double)packet->Data.spark.voltage_driver,
    (double)packet->Data.spark.temperature_driver,
    (double)packet->Data.spark.temperature_converter,
    (unsigned int)packet->Data.spark.sparkStatus,
    (unsigned int)packet->Data.spark.as5600_status);
        break;
    case PACKET_ID_STATE:
        length = snprintf(text_buffer, sizeof(text_buffer), "\nID:%d, TS:%lu, flight_state:%u, timestamp_ms:%lu",
                (int)packet->Packet_ID, (unsigned long)packet->timestamp,
                (unsigned int)packet->Data.state.flight_state,
    (unsigned long)packet->Data.state.timestamp_ms);
        break;
      default:
        return 2; // Unsupported packet type
    }
    if (length < 0 || (size_t)length >= sizeof(text_buffer)) return 2;
    text_size = (uint32_t)length;

    // Check if there is enough space in the buffer
    if (text_size > BUFFER_SIZE - sd_buffer_index) {
      text_size = sizeof("\nBuffer full") - 1;
      sprintf(text_buffer, "\nBuffer full");
      if(text_size <= BUFFER_SIZE - sd_buffer_index){
        memcpy(&sd_buffer[sd_buffer_index], text_buffer, text_size);
        sd_buffer_index += text_size;
        return 3; // Error: Not enough space in buffer
      }
      return 4; // Error: Not enough space in buffer even for "Buffer full"
    }

    // Copy the DataPacket_t into the buffer
    memcpy(&sd_buffer[sd_buffer_index], text_buffer, text_size);
    sd_buffer_index += text_size;

    return 0; // Success
}

void SD_ResetBufferIndex() {
  sd_buffer_index = 0;
}

uint8_t SD_Mount(void)
{
  fres = f_mount(&FatFs, "", 1);
  if (fres != FR_OK) {
    return 1;  // Error
  }

  FATFS* getFreeFS;
  fres = f_getfree("", &free_clusters, &getFreeFS);
  if (fres != FR_OK) {
    return 1;  // Error
  }

  total_sectors = (getFreeFS->n_fatent - 2) * getFreeFS->csize;
  free_sectors = free_clusters * getFreeFS->csize;
  return 0;  // Success
}

uint8_t SD_Unmount(void)
{
  fres = f_mount(NULL, "", 0);
  if (fres != FR_OK) {
    return 1;  // Error
  }
  return 0;  // Success
}

uint8_t SD_Open(char* filename, uint8_t mode)
{
  fres = f_open(&fil, filename, mode);
  if (fres != FR_OK) {
    return 1;  // Error
  }
  return 0;  // Success
}

uint8_t SD_Read(char* buffer, uint32_t bytes_to_read)
{
  TCHAR* rres = f_gets(buffer, bytes_to_read, &fil);
  if (rres == NULL) {
    return 1;  // Error
  }
  return 0;  // Success
}

uint8_t SD_Write(char* buffer, uint32_t bytes_to_write)
{
  UINT bytesWrote;
  fres = f_write(&fil, buffer, bytes_to_write, &bytesWrote);
  if (fres != FR_OK) {
    return 1;  // Error
  }
  return 0;  // Success
}

uint8_t SD_Close(void)
{
  fres = f_close(&fil);
  if (fres != FR_OK) {
    return 1;  // Error
  }
  return 0;  // Success
}

uint8_t SD_SelfTest(void) {
  FRESULT mount = SD_Mount();
  if (mount == FR_OK) return 1; // Success
  return 0; // Error
}
