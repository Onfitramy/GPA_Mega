/*
 *  W25Q1 Flash Chip
 *
 * 
 */

#include "main.h"
#include "W25Q1.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"

W25QPage0_config_t W25Q_FLASH_CONFIG = {
	.ID = {0x41, 0x45}, // AP in hex
	.curr_configPage = CONFIG_PAGE, // Start at the configuration page
	.curr_configOffset = 0, // Start at the beginning of the configuration page
	.curr_logPage = LOG_PAGE, // Start at the log page
	.curr_logOffset = 0, // Start at the beginning of the log page
	.write_logs = true,
};

//Saves the data to the log page
void W25Q_SaveToLog(uint8_t *data, uint32_t size)
{
	if (size == 0 || !W25Q_FLASH_CONFIG.write_logs) return; // nothing to save

	// Check if we have enough space in the current log page
	if (W25Q_FLASH_CONFIG.curr_logOffset + size > 256) {
		W25Q_FLASH_CONFIG.curr_logPage += 1; // Move to next page
		W25Q_FLASH_CONFIG.curr_logOffset = 0; // Reset offset
	}

	W25Q_Write_Cleared(W25Q_FLASH_CONFIG.curr_logPage, W25Q_FLASH_CONFIG.curr_logOffset, size, data);

	W25Q_FLASH_CONFIG.curr_logOffset += size;
	if (W25Q_FLASH_CONFIG.curr_logOffset >= 256) {
		W25Q_FLASH_CONFIG.curr_logOffset %= 256;
		W25Q_FLASH_CONFIG.curr_logPage += 1;
	}

	W25Q_Write_Page(0, 0, sizeof(W25QPage0_config_t), (uint8_t *)&W25Q_FLASH_CONFIG);
}


uint8_t flash_buffer_index = 0;
DataPacket_t flash_packet_buffer[8];

void W25Q_AddFlashBufferPacket(const DataPacket_t *data_packet) {
	if (flash_buffer_index < 8) {
		flash_packet_buffer[flash_buffer_index++] = *data_packet;
	}

	// TODO: Add logic to determine if sector should be cleared.
	if (flash_buffer_index == 8) {
		W25Q_Erase_Sector(W25Q_FLASH_CONFIG.curr_logPage / 16);
		W25Q_SaveToLog((uint8_t*)flash_packet_buffer, 256);
		flash_buffer_index = 0;
	}
}

void W25Q_LoadFromLog(uint8_t *data, uint32_t size, uint32_t log_page, uint32_t log_offset)
{
	// Load data from the log page
	if (size == 0) return; // nothing to load

	W25Q_Read(log_page, log_offset, size, data);
}

void csLOW(void) 
{
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
}

void csHIGH(void) 
{
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
}

uint32_t bytestowrite (uint32_t size, uint16_t offset)
{
	if ((size+offset)<256) return size;
	else return 256-offset;
}

uint32_t bytestomodify (uint32_t size, uint16_t offset)
{
	if ((size+offset)<4096) return size;
	else return 4096-offset;
}

void W25Q1_Reset(void)
{
    uint8_t tData[2];
    tData[0] = 0x66;    //Enable reset
    tData[1] = 0x99;    //Reset
    csLOW();
    HAL_SPI_Transmit(&W25Q1_SPI , &tData[0], 1, 1000);
    HAL_SPI_Transmit(&W25Q1_SPI , &tData[1], 1, 1000);
    csHIGH();
	
    vTaskDelay(20); // Wait for the chip to reset
}

uint32_t W25Q1_ReadID(void)
{
    uint8_t tData = 0x9F; //Read JEDEC ID
    uint8_t rData[3];
    csLOW();
    HAL_SPI_Transmit(&W25Q1_SPI , &tData, 1, 1000);
    HAL_SPI_Receive(&W25Q1_SPI , rData, 3, 1000);
    csHIGH();

    return ((rData[0]<<16)|(rData[1]<<8)|rData[2]);
}

void W25Q_Read (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData)
{
	uint8_t tData[5];
	uint32_t memAddr = (startPage*256) + offset;

    if (numBLOCK<512)   // Chip Size<256Mb
	{
		tData[0] = 0x03;  // enable Read
		tData[1] = (memAddr>>16)&0xFF;  // MSB of the memory Address
		tData[2] = (memAddr>>8)&0xFF;
		tData[3] = (memAddr)&0xFF; // LSB of the memory Address
	}

    csLOW();
    if (numBLOCK<512)
    {
        HAL_SPI_Transmit(&W25Q1_SPI , tData, 4, 100);
    }
    
    HAL_SPI_Receive(&W25Q1_SPI, rData, size, 100);
    csHIGH();
}

void W25Q_FastRead (uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData)
{
	uint8_t tData[6];
	uint32_t memAddr = (startPage*256) + offset;

	if (numBLOCK<512)   // Chip Size<256Mb
	{
		tData[0] = 0x0B;  // enable Fast Read
		tData[1] = (memAddr>>16)&0xFF;  // MSB of the memory Address
		tData[2] = (memAddr>>8)&0xFF;
		tData[3] = (memAddr)&0xFF; // LSB of the memory Address
		tData[4] = 0;  // Dummy clock
	}

	csLOW();  // pull the CS Low
	if (numBLOCK<512)
	{
		HAL_SPI_Transmit(&W25Q1_SPI, tData, 5, 100);  // send read instruction along with the 24 bit memory address
	}

    uint8_t tempRx[3000];
    memset(tempRx, 0, 3000);
    HAL_SPI_TransmitReceive(&W25Q1_SPI, tempRx, rData, size, 100);
	//HAL_SPI_Receive(&W25Q1_SPI, rData, size, 100);  // Read the data
	csHIGH();  // pull the CS High
}

uint8_t W25Q_Read_Byte (uint32_t Addr)
{
	uint8_t tData[5];
	uint8_t rData;

	if (numBLOCK<512)   // Chip Size<256Mb
	{
		tData[0] = 0x03;  // enable Read
		tData[1] = (Addr>>16)&0xFF;  // MSB of the memory Address
		tData[2] = (Addr>>8)&0xFF;
		tData[3] = (Addr)&0xFF; // LSB of the memory Address
	}
	else  // we use 32bit memory address for chips >= 256Mb
	{
		tData[0] = 0x13;  // Read Data with 4-Byte Address
		tData[1] = (Addr>>24)&0xFF;  // MSB of the memory Address
		tData[2] = (Addr>>16)&0xFF;
		tData[3] = (Addr>>8)&0xFF;
		tData[4] = (Addr)&0xFF; // LSB of the memory Address
	}

	csLOW();  // pull the CS Low
	if (numBLOCK<512)
	{
		HAL_SPI_Transmit(&W25Q1_SPI, tData, 4, 100);  // send read instruction along with the 24 bit memory address
	}
	else
	{
		HAL_SPI_Transmit(&W25Q1_SPI, tData, 5, 100);  // send read instruction along with the 32 bit memory address
	}

	HAL_SPI_Receive(&W25Q1_SPI, &rData, 1, 100);  // Read the data
	csHIGH();  // pull the CS High

	return rData;
}

void enable_write (void)
{
	uint8_t tData = 0x06;  // enable Write
	csLOW();  // pull the CS LOW
	HAL_SPI_Transmit(&W25Q1_SPI, &tData, 1, 100);
	csHIGH();  // pull the HIGH
	vTaskDelay(5);  // Write cycle delay (5ms)
}

void disable_write (void)
{
	uint8_t tData = 0x04;  // disable Write
	csLOW();  // pull the CS LOW
	HAL_SPI_Transmit(&W25Q1_SPI, &tData, 1, 100);
	csHIGH();  // pull the HIGH
	vTaskDelay(5);  // Write cycle delay (5ms)
}

void W25Q_Erase_Sector (uint16_t numsector)
{
	uint8_t tData[6];
	uint32_t memAddr = numsector*16*256;   // Each sector contains 16 pages * 256 bytes

	enable_write();

	if (numBLOCK<512)   // Chip Size<256Mb
	{
	  tData[0] = 0x20;  // Erase sector
	  tData[1] = (memAddr>>16)&0xFF;  // MSB of the memory Address
	  tData[2] = (memAddr>>8)&0xFF;
	  tData[3] = (memAddr)&0xFF; // LSB of the memory Address

	  csLOW();
	  HAL_SPI_Transmit(&W25Q1_SPI, tData, 4, 100);
	  csHIGH();
	}

	vTaskDelay(450);  // 450ms delay for sector erase

	disable_write();

}

void W25Q_Write_Page (uint32_t page, uint16_t offset, uint32_t size, uint8_t *data)
{
	uint8_t tData[266];
	uint32_t startPage = page;
	uint32_t endPage  = startPage + ((size+offset-1)/256);
	uint32_t numPages = endPage-startPage+1;

    uint16_t startSector  = startPage/16;
	uint16_t endSector  = endPage/16;
	uint16_t numSectors = endSector-startSector+1;

    for (uint16_t i=0; i<numSectors; i++)
	{
		W25Q_Erase_Sector(startSector++);
	}

    uint32_t dataPosition = 0;

	// write the data
	for (uint32_t i=0; i<numPages; i++)
	{
		uint32_t memAddr = (startPage*256)+offset;
		uint16_t bytesremaining  = bytestowrite(size, offset);

		enable_write();
                uint32_t indx = 0;
		if (numBLOCK<512)   // Chip Size<256Mb
		{
			tData[0] = 0x02;  // page program
			tData[1] = (memAddr>>16)&0xFF;  // MSB of the memory Address
			tData[2] = (memAddr>>8)&0xFF;
			tData[3] = (memAddr)&0xFF; // LSB of the memory Address

			indx = 4;
		}

        uint16_t bytestosend  = bytesremaining + indx;

		for (uint16_t i=0; i<bytesremaining; i++)
		{
			tData[indx++] = data[i+dataPosition];
		}

		csLOW();
		HAL_SPI_Transmit(&W25Q1_SPI, tData, bytestosend, 100);
		csHIGH();

		startPage++;
		offset = 0;
		size = size-bytesremaining;
		dataPosition = dataPosition+bytesremaining;

        vTaskDelay(4);
		disable_write();
    }
}

void W25Q_Write (uint32_t page, uint16_t offset, uint32_t size, uint8_t *data)
{
	uint16_t prevDSize = 64;

	uint16_t startSector  = page/16;
	uint16_t endSector  = (page + ((size+offset-1)/256))/16;
	uint16_t numSectors = endSector-startSector+1;

    uint8_t previousData[prevDSize];
	uint32_t sectorOffset = ((page%16)*256)+offset;
	uint32_t dataindx = 0;

    for (uint16_t i=0; i<numSectors; i++)
	{
		uint32_t startPage = startSector*16;
		W25Q_FastRead(startPage, 0, prevDSize, previousData);
		uint16_t bytesRemaining = bytestomodify(size, sectorOffset);

		for (uint16_t i=0; i<bytesRemaining; i++)
		{
			previousData[i+sectorOffset] = data[i+dataindx];
		}

		W25Q_Write_Page(startPage, 0, prevDSize, previousData);

		startSector++;
		sectorOffset = 0;
		dataindx = dataindx+bytesRemaining;
		size = size-bytesRemaining;
    }
}

void W25Q_Write_Byte (uint32_t Addr, uint8_t data)
{
	uint8_t tData[6];
	uint8_t indx;

	if (numBLOCK<512)   // Chip Size<256Mb
	{
		tData[0] = 0x02;  // page program
		tData[1] = (Addr>>16)&0xFF;  // MSB of the memory Address
		tData[2] = (Addr>>8)&0xFF;
		tData[3] = (Addr)&0xFF; // LSB of the memory Address
		tData[4] = data;
		indx = 5;
	}


	if (W25Q_Read_Byte(Addr) == 0xFF)
	{
		enable_write();
		csLOW();
		HAL_SPI_Transmit(&W25Q1_SPI, tData, indx, 100);
		csHIGH();

		vTaskDelay(5);
		disable_write();
	}
}

void float2Bytes(uint8_t * ftoa_bytes_temp,float float_variable)
{
    union {
      float a;
      uint8_t bytes[4];
    } thing;

    thing.a = float_variable;

    for (uint8_t i = 0; i < 4; i++) {
      ftoa_bytes_temp[i] = thing.bytes[i];
    }

}

float Bytes2float(uint8_t * ftoa_bytes_temp)
{
    union {
      float a;
      uint8_t bytes[4];
    } thing;

    for (uint8_t i = 0; i < 4; i++) {
    	thing.bytes[i] = ftoa_bytes_temp[i];
    }

   float float_variable =  thing.a;
   return float_variable;
}

uint8_t tempBytes[4];
void W25Q_Write_NUM (uint32_t page, uint16_t offset, float data)
{
	float2Bytes(tempBytes, data);

	/* Write using sector update function */
	W25Q_Write(page, offset, 4, tempBytes);
}

float W25Q_Read_NUM (uint32_t page, uint16_t offset)
{
	uint8_t rData[4];
	W25Q_Read(page, offset, 4, rData);
	return (Bytes2float(rData));
}

void W25Q_Write_32B (uint32_t page, uint16_t offset, uint32_t size, uint32_t *data)
{
	uint8_t data8[size*4];
	uint32_t indx = 0;

	for (uint32_t i=0; i<size; i++)
	{
		data8[indx++] = data[i]&0xFF;   // extract LSB
		data8[indx++] = (data[i]>>8)&0xFF;
		data8[indx++] = (data[i]>>16)&0xFF;
		data8[indx++] = (data[i]>>24)&0xFF;
	}

	W25Q_Write(page, offset, indx, data8);
}

void W25Q_Read_32B (uint32_t page, uint16_t offset, uint32_t size, uint32_t *data)
{
	uint8_t data8[size*4];
	uint32_t indx = 0;

	W25Q_FastRead(page, offset, size*4, data8);

	for (uint32_t i=0; i<size; i++)
	{
		data[i] = (data8[indx++]) | (data8[indx++]<<8) | (data8[indx++]<<16) | (data8[indx++]<<24);
	}
}

void W25Q_Chip_Erase (void)
{
    uint8_t tData[1];
    tData[0] = 0xC7;    //Chip erase
	W25Q_FLASH_CONFIG.curr_logPage = LOG_PAGE;	//Reset Log offset to zero
	W25Q_FLASH_CONFIG.curr_logOffset = 0;
    
    enable_write();
    csLOW();
    HAL_SPI_Transmit(&W25Q1_SPI, tData, 1, 100);
    csHIGH();

    vTaskDelay(40000); //should be 40s - 200s
	disable_write();

	W25Q_Write_Page(0, 0, sizeof(W25QPage0_config_t), (uint8_t *)&W25Q_FLASH_CONFIG);
}


void W25Q_GetConfig()
{
	uint8_t tempConfig[256];
	// Copy the config on the chip to a local variable
	W25Q_Read(0, 0, sizeof(W25QPage0_config_t), tempConfig);
	// Copy the data to the W25Q_FLASH_CONFIG structure if ID is correct
	if (tempConfig[0] == W25Q_FLASH_CONFIG.ID[0] && tempConfig[1] == W25Q_FLASH_CONFIG.ID[1])
	{
		memcpy(&W25Q_FLASH_CONFIG, tempConfig, sizeof(W25QPage0_config_t));
	}
	else
	{
		// If the ID does not match, save the default config
		W25Q_Write_Page(0, 0, sizeof(W25QPage0_config_t), (uint8_t *)&W25Q_FLASH_CONFIG);
	}
}

/**
  * @brief Writes Pre cleared memory if the memory hasnt been cleard nothing is written in that position
  * @param None
  * @retval None
  */
void W25Q_Write_Cleared(uint32_t page, uint16_t offset, uint32_t size, uint8_t *data)
{
	uint8_t tData[266];
	uint32_t startPage = page;
	uint32_t endPage  = startPage + ((size+offset-1)/256);
	uint32_t numPages = endPage-startPage+1;

    uint16_t startSector  = startPage/16;
	uint16_t endSector  = endPage/16;
	uint16_t numSectors = endSector-startSector+1;

    uint32_t dataPosition = 0;

	// write the data
	for (uint32_t i=0; i<numPages; i++)
	{
		uint32_t memAddr = (startPage*256)+offset;
		uint16_t bytesremaining  = bytestowrite(size, offset);

		enable_write();
        uint32_t indx = 0;
		if (numBLOCK<512)   // Chip Size<256Mb
		{
			tData[0] = 0x02;  // page program
			tData[1] = (memAddr>>16)&0xFF;  // MSB of the memory Address
			tData[2] = (memAddr>>8)&0xFF;
			tData[3] = (memAddr)&0xFF; // LSB of the memory Address

			indx = 4;
		}

        uint16_t bytestosend  = bytesremaining + indx;

		for (uint16_t i=0; i<bytesremaining; i++)
		{
			tData[indx++] = data[i+dataPosition];
		}

		csLOW();
		HAL_SPI_Transmit(&W25Q1_SPI, tData, bytestosend, 100);
		csHIGH();

		startPage++;
		offset = 0;
		size = size-bytesremaining;
		dataPosition = dataPosition+bytesremaining;

        vTaskDelay(4);
		disable_write();
    }
}
