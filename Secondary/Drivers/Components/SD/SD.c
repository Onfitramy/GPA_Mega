#include "SD.h"
#include "fatfs.h"
#include <stdio.h>
#include <string.h>

FATFS FatFs; 	//Fatfs handle
FIL fil; 		//File handle
FRESULT fres; //Result after operations

DWORD free_clusters, free_sectors, total_sectors;

#define BUFFER_SIZE 2048

uint8_t sd_buffer1[BUFFER_SIZE]; // Buffer for SD operations, maximum of 2048 bytes
uint8_t sd_buffer2[BUFFER_SIZE]; // Using double buffering
uint8_t* sd_buffer = sd_buffer1; // Pointer to current buffer
uint32_t sd_buffer_index = 0; // Current index in the buffer

uint8_t SD_SaveBuffer(void){
  uint32_t write_size = sd_buffer_index;
  sd_buffer_index = 0; // Reset buffer index
  uint8_t* save_buffer =  sd_buffer; // Pointer to current buffer, which will be saved
  sd_buffer = (sd_buffer == sd_buffer1) ? sd_buffer2 : sd_buffer1; // Switch buffers

  if (write_size == 0) {
    return 1; // Nothing to save
  }
  SD_Open("LOG.txt", FA_WRITE | FA_OPEN_APPEND);
  SD_Write(save_buffer, write_size);
  SD_Close();
  return 0; // Success
}

uint8_t SD_AppendDataPacketToBuffer(DataPacket_t* packet) {
    char text_buffer[128] = {0}; // Temporary buffer for saving SD text
    uint32_t text_size = 0;

    if (packet == NULL) {
        return 1; // Error: Null pointer
    }

    switch(packet->Packet_ID) {
        case PACKET_ID_Status:
          sprintf(text_buffer, "\nID:%d, TS:%lu, StatFl:%lu, SensFl:%lu, ErrFl:%lu",
                  packet->Packet_ID, packet->timestamp,
                  packet->Data.status.status_flags, packet->Data.status.sensor_status_flags, packet->Data.status.error_flags);
          text_size = strlen(text_buffer);
          break;
        case PACKET_ID_BATTERY:
          sprintf(text_buffer, "\nID:%d, TS:%lu, V5V:%lu, VBAT:%lu",
                  packet->Packet_ID, packet->timestamp,
                  packet->Data.battery.voltage5V0bus, packet->Data.battery.voltageBATbus);
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
