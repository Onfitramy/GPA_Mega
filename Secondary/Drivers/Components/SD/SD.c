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
    char text_buffer[256] = {0}; // Temporary buffer for saving SD text
    uint32_t text_size = 0;

    if (packet == NULL) {
      return 1; // Error: Null pointer
    }

    switch(packet->Packet_ID) {
      case PACKET_ID_STATUS:
        sprintf(text_buffer, "\nID:%d, TS:%lu, StatFl:%lu, SensFl:%lu, ErrFl:%lu, STATE:%d",
                packet->Packet_ID, packet->timestamp,
                packet->Data.status.status_flags, packet->Data.status.sensor_status_flags, packet->Data.status.error_flags, packet->Data.status.State);
        text_size = strlen(text_buffer);
        break;
      case PACKET_ID_POWER:
        sprintf(text_buffer, "\nID:%d, TS:%lu, M25VBUS:%u, M2BATV:%u, PUPOW:%u, PUCURR:%u, PUBATV:%u",
                packet->Packet_ID, packet->timestamp,
                packet->Data.power.M2_bus_5V, packet->Data.power.M2_bus_GPA_bat_volt,
                packet->Data.power.PU_pow, packet->Data.power.PU_curr, packet->Data.power.PU_bat_bus_volt);
        text_size = strlen(text_buffer);
        break;
      case PACKET_ID_GPS:
        sprintf(text_buffer, "\nID:%d, TS:%lu, Lat:%ld, Lon:%ld, Alt:%ld, Spd:%d, Crs:%d",
                packet->Packet_ID, packet->timestamp,
                packet->Data.gps.latitude, packet->Data.gps.longitude, packet->Data.gps.altitude,
                packet->Data.gps.speed, packet->Data.gps.course);
        text_size = strlen(text_buffer);
        break;
      case PACKET_ID_IMU:
        sprintf(text_buffer, "\nID:%d, TS:%lu, GyrX:%d, GyrY:%d, GyrZ:%d, AccX:%ld, AccY:%ld, AccZ:%ld, MagX:%d, MagY:%d, MagZ:%d",
                packet->Packet_ID, packet->timestamp,
                packet->Data.imu.gyroX, packet->Data.imu.gyroY, packet->Data.imu.gyroZ,
                packet->Data.imu.accelX, packet->Data.imu.accelY, packet->Data.imu.accelZ,
                packet->Data.imu.magX, packet->Data.imu.magY, packet->Data.imu.magZ);
        text_size = strlen(text_buffer);
        break;
      case PACKET_ID_TEMPERATURE:
        sprintf(text_buffer, "\nID:%d, TS:%lu, M1DTS:%d, M1ADC:%d, M1BMP:%d, M1IMU1:%d, M1IMU2:%d, M1MAG:%d, M23V3:%d, M2XBee:%d, PUBAT:%d, Pressure:%.6f",
                packet->Packet_ID, packet->timestamp,
                packet->Data.temperature.M1_DTS, packet->Data.temperature.M1_ADC,
                packet->Data.temperature.M1_BMP, packet->Data.temperature.M1_IMU1,
                packet->Data.temperature.M1_IMU2, packet->Data.temperature.M1_MAG,
                packet->Data.temperature.M2_3V3, packet->Data.temperature.M2_XBee,
                packet->Data.temperature.PU_bat, packet->Data.temperature.pressure);
        text_size = strlen(text_buffer);
        break;

      case PACKET_ID_ATTITUDE:
        sprintf(text_buffer, "\nID:%d, TS:%lu, phi:%f, theta:%f, psi:%f",
                packet->Packet_ID, packet->timestamp,
                packet->Data.attitude.phi, packet->Data.attitude.theta, packet->Data.attitude.psi);
        text_size = strlen(text_buffer);
        break;
      case PACKET_ID_KALMANMATRIX:
        sprintf(text_buffer, "\nID:%d, TS:%lu, P11:%f, P22:%f, P33:%f, EKF2_Height:%f, EKF2_Vel:%f, EKF2_RefPres:%f",
                packet->Packet_ID, packet->timestamp,
                packet->Data.kalman.P11, packet->Data.kalman.P22, packet->Data.kalman.P33,
                packet->Data.kalman.EKF2_Heigth, packet->Data.kalman.EKF2_vel, packet->Data.kalman.EKF2_refPres);
        text_size = strlen(text_buffer);
        break;
      case PACKET_ID_STATE:
        sprintf(text_buffer, "\nID:%d, TS:%lu, STATE:%d, STATE_TS:%ld", packet->Packet_ID, packet->timestamp, packet->Data.state.flight_state, packet->Data.state.timestamp_us);
        break;
      default:
        return 2; // Error: Invalid Packet ID
    }

    // Check if there is enough space in the buffer
    if (text_size > BUFFER_SIZE - sd_buffer_index) {
      text_size = 13; // Length of "\nBuffer full"
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
