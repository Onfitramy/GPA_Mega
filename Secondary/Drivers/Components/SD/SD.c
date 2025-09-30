#include "SD.h"
#include "fatfs.h"

FATFS FatFs; 	//Fatfs handle
FIL fil; 		//File handle
FRESULT fres; //Result after operations

DWORD free_clusters, free_sectors, total_sectors;

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
