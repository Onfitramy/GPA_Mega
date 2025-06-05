/*
 *  W25Q1 Flash Chip
 *
 * 
 */

#include "main.h"
#include "W25Q1.h"

CIRC_BBUF_DEF(event_circ_buf, 8192);

int circ_bbuf_push(circ_bbuf_t *c, uint8_t data)
{
    int next;

    next = c->head + 1;  // next is where head will point to after this write.
    if (next >= c->maxlen)
        next = 0;

    if (next == c->tail)  // if the head + 1 == tail, circular buffer is full
        return -1;

    c->buffer[c->head] = data;  // Load data and then move
    c->head = next;             // head to next data offset.
    return 0;  // return success to indicate successful push.
}

int circ_bbuf_pop(circ_bbuf_t *c, uint8_t *data)
{
    int next;

    if (c->head == c->tail)  // if the head == tail, we don't have any data
        return -1;

    next = c->tail + 1;  // next is where tail will point to after this read.
    if(next >= c->maxlen)
        next = 0;

    *data = c->buffer[c->tail];  // Read data and then move
    c->tail = next;              // tail to next offset.
    return 0;  // return success to indicate successful push.
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
    HAL_Delay(20);
}

uint32_t W25Q1_ReadID(void)
{
    uint8_t tData = 0x9F; //Read JEDEC ID
    uint8_t rData[3];
    csLOW();
    HAL_SPI_Transmit(&W25Q1_SPI , &tData, 1, 1000);
    HAL_SPI_Receive(&W25Q1_SPI , rData, 3, 1000);
    csHIGH();

    /*uint8_t tData[4];
    tData[0] = 0xAB;    //Device ID
    tData[1] = 0xFF;
    tData[2] = 0xFF;
    tData[3] = 0xFF;
    uint8_t iData;
    csLOW();
    HAL_SPI_Transmit(&W25Q1_SPI , tData, 4, 1000);
    HAL_SPI_Receive(&W25Q1_SPI , iData, 1, 1000);
    csHIGH();*/

    return ((rData[0]<<16)|(rData[1]<<8)|rData[2]);
    //return iData;
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
    
    uint8_t tempRx[3000];
    memset(tempRx, 0, 3000);
    HAL_SPI_TransmitReceive(&W25Q1_SPI, tempRx, rData, size, 100);
    //HAL_SPI_Receive(&W25Q1_SPI, rData, size, 100); 
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
	HAL_Delay(5);  // Write cycle delay (5ms)
}

void disable_write (void)
{
	uint8_t tData = 0x04;  // disable Write
	csLOW();  // pull the CS LOW
	HAL_SPI_Transmit(&W25Q1_SPI, &tData, 1, 100);
	csHIGH();  // pull the HIGH
	HAL_Delay(5);  // Write cycle delay (5ms)
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

	HAL_Delay(450);  // 450ms delay for sector erase

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

        HAL_Delay(4);
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

		HAL_Delay(5);
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

uint32_t curr_logPage = 1;
uint16_t curr_logOffset = 0;

void W25Q_Chip_Erase (void)
{
    uint8_t tData[1];
    tData[0] = 0xC7;    //Chip erase
	curr_logPage = 1;	//Reset Log offset to zero
	curr_logOffset = 0;
    
    enable_write();
    csLOW();
    HAL_SPI_Transmit(&W25Q1_SPI, tData, 1, 100);
    csHIGH();

    HAL_Delay(40000); //should be 40s - 200s
	disable_write();

	uint32_t tData32[2] = {curr_logPage, curr_logOffset};

	W25Q_Write_32B(0, 0, 2, tData32); //Save current log page and offset at 0

}

/**
  * @brief Reads the current log Position from Page 0, offset 0 in memory
  * @param None
  * @retval None
  */
void W25Q_readLogPos(void){
	uint32_t LogPos[2];

	W25Q_Read_32B(0, 0, 2, LogPos);

	curr_logPage = LogPos[0];
	curr_logOffset = LogPos[1];
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

        HAL_Delay(4);
		disable_write();
    }
}

void W25Q_SaveLog(void)
{
	uint8_t tData[4096];
	uint8_t out_data;
	uint32_t size = 0;
	for(int i=0 ;circ_bbuf_pop(&event_circ_buf, &out_data)!=-1; i++){
		size += 1;
		tData[i] = out_data;
	}

	W25Q_Write_Cleared(curr_logPage, curr_logOffset, size, tData);

	if (curr_logOffset + size >= 256){
		curr_logPage += 1;
		size -= 256;
		while (curr_logOffset + size >= 256){
			curr_logPage += 1;
			size -= 256;
		}
	}
	curr_logOffset += size;

	//uint32_t tData32[2] = {curr_logPage, curr_logOffset};

	//W25Q_Write_32B(0, 0, 2, tData32); //Save current log page and offset at 0

}

/**
  * @brief Append the LOG with a event , eventsize is alway wscode shown - 1
  * @param None
  * @retval None
  */
uint8_t W25Q_AppendLog(uint8_t *event, uint32_t eventSize, float data1, float data2, float data3)
{
	uint32_t timestamp = HAL_GetTick();
	uint8_t tData[37];
	uint32_t size = 6; //initial size only \n and  timestamp 
	tData[0] = 0x5C; 
	tData[1] = 0x6E;	// backslash n
	tData[2] = (timestamp>>24)&0xFF;   // add timestamp
	tData[3] = (timestamp>>16)&0xFF;
	tData[4] = (timestamp>>8)&0xFF;
	tData[5] = timestamp&0xFF;
	
	if (eventSize>16){
		return 0; //to long event description
	}else{
		size += eventSize + 1;
		tData[6] = 0x3A; //":"
		for (int i=0; i<eventSize; i++)
		{
			tData[7+i] = event[i];   //add event
		}
	}

	if (data1 != 0){
		tData[size] = 0x3A; //":"
		float2Bytes(tempBytes, data1);
		tData[size+1] = tempBytes[0];
		tData[size+2] = tempBytes[1];
		tData[size+3] = tempBytes[2];
		tData[size+4] = tempBytes[3];
		size += 5;
	}

	if (data2 != 0){
		tData[size] = 0x3A; //":"
		float2Bytes(tempBytes, data2);
		tData[size+1] = tempBytes[0];
		tData[size+2] = tempBytes[1];
		tData[size+3] = tempBytes[2];
		tData[size+4] = tempBytes[3];
		size += 5;
	}

	if (data3 != 0){
		tData[size] = 0x3A; //":"
		float2Bytes(tempBytes, data3);
		tData[size+1] = tempBytes[0];
		tData[size+2] = tempBytes[1];
		tData[size+3] = tempBytes[2];
		tData[size+4] = tempBytes[3];
		size += 5;
	}

	for (int i=0; i<size; i++){
		circ_bbuf_push(&event_circ_buf, tData[i]);
	}
}