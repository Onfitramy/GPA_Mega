#include "u_ubx_protocol.h"
#include "SAM-M8Q.h"
#include "string.h"

HAL_StatusTypeDef GPS_I2C_status;

/*Random Read Access*/
void I2C_RRC(uint8_t reg, uint8_t *data, uint8_t length) {
    uint8_t testdata = 0xFF;
    GPS_I2C_status = HAL_I2C_Master_Transmit(&GPS_I2C, GPS_I2C_ADDR, &testdata, 1, 100);
    //GPS_I2C_status = HAL_I2C_Mem_Read(&GPS_I2C, GPS_I2C_ADDR, reg, 1, data, length, 100);
    uint8_t test = 1;
}


uint16_t ublox_ReadLength(void)
{
  uint8_t data[2];
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&GPS_I2C, GPS_I2C_ADDR, 0xFD, 1, data, sizeof(data), 100);
  if (status != HAL_OK){
    status = HAL_I2C_GetError(&GPS_I2C);
  }

  return(((uint16_t)data[0] << 8) + (uint16_t)data[1]);
}

void ublox_Write(int size, uint8_t *data) {
    
    //HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&GPS_I2C, GPS_I2C_ADDR, 0xFF, 1, data, size, 100);
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&GPS_I2C, GPS_I2C_ADDR, data, size, 100);
    if (status != HAL_OK) {
        status = HAL_I2C_GetError(&GPS_I2C);
    }else if (status == HAL_OK) {
        //HAL_GPIO_TogglePin(M1_LED_GPIO_Port, M1_LED_Pin);
    }
}

uint8_t GPS_VER_CHECK(void) {
    uint8_t UBX_MessageSend[8];
    uint8_t UBX_MessageReturn[200];
    int32_t messageClass;
    int32_t messageId;

    static uint8_t buffer[1000];

    uUbxProtocolEncode(0x0A, 0x04, NULL, 0, UBX_MessageSend);


    ublox_Write(8, UBX_MessageSend);

    uint16_t length = ublox_ReadLength();
      //printf("uBlox Length %5d %04X\n", length, length);
      if (length)
      {
        uint16_t len = (length < sizeof(buffer)) ? length : sizeof(buffer);
        HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&GPS_I2C, GPS_I2C_ADDR, 0xFF, 1, buffer, len, 300);
        if (status == HAL_OK){
            uUbxProtocolDecode(buffer, len, &messageClass, &messageId, UBX_MessageReturn, sizeof(UBX_MessageReturn), NULL);
            for (size_t i = 0; i <= length; i++) {
                buffer[i] = 0;
            }
        }else{
          return 0;//Read Fail
        }
        if((strncmp((char *)UBX_MessageReturn, "ROM CORE 3.01 (107888)", 22) == 0)){
            return 1; //Read Success
        }else{return 0;} //Wrong Data
      }
}