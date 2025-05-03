#include "u_ubx_protocol.h"
#include "SAM-M8Q.h"
#include "string.h"

HAL_StatusTypeDef GPS_I2C_status;

static uint8_t buffer[1000]; //Main Buffer used for many different things. Never use it for longer than inside a single function!! Try to clear/reset it after use

bool GPSnotConfig = true;

/**
 * @brief Reads the length of data available from the u-blox GPS module.
 * 
 * This function communicates with the u-blox GPS module over I2C to read
 * a 16-bit length value from a specific register (0xFD). The length value
 * indicates the amount of data available to be read from the GPS module.
 * 
 * @note The function uses the HAL_I2C_Mem_Read function to perform the I2C
 *       memory read operation. If the read operation fails, the error status
 *       is retrieved using HAL_I2C_GetError.
 * 
 * @return uint16_t The 16-bit length value read from the GPS module. The
 *         value is constructed by combining two bytes read from the module,
 *         with the first byte as the most significant byte (MSB) and the
 *         second byte as the least significant byte (LSB).
 */
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


/**
 * @brief Reads output from the u-blox GPS module and decodes the UBX message.
 *
 * This function reads the length of the data available from the GPS module,
 * retrieves the data via I2C, and decodes it into a UBX message structure.
 * It also performs a validation check on the decoded message.
 *
 * @param UBX_MessageReturn A pointer to a character buffer where the decoded
 *                          UBX message body will be stored.
 * @return UBX_MessageType  A structure containing the decoded UBX message
 *                          details, including message class, ID, and body.
 *
 * @note The function assumes that the buffer used for I2C communication is
 *       globally defined and accessible. It also clears the buffer after
 *       reading the data.
 *
 * @warning Ensure that the buffer size is sufficient to hold the data read
 *          from the GPS module. The function uses the smaller of the available
 *          data length or the buffer size.
 *
 * @warning The function does not handle cases where the length of the data
 *          exceeds the buffer size. Ensure proper buffer management to avoid
 *          data loss or corruption.
 */
UBX_MessageType ublox_ReadOutput(char* UBX_MessageReturn) {
  UBX_MessageType UBX_Message = {0, 0, UBX_MessageReturn, 0}; //Initialize the message structure to zero
    
  uint16_t length = ublox_ReadLength();
  length = 1000;
  //printf("uBlox Length %5d %04X\n", length, length);
  if (length)
  {
    uint16_t len = (length < sizeof(buffer)) ? length : sizeof(buffer);
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&GPS_I2C, GPS_I2C_ADDR, 0xFF, 1, buffer, len, 300);
    if (status == HAL_OK){
        uUbxProtocolDecode((char*)buffer, len, &UBX_Message.messageClass, &UBX_Message.messageId, UBX_Message.messageBody, len, NULL);
        UBX_Message.status = 1; //Read Success
        for (size_t i = 0; i <= length; i++) {
            buffer[i] = 0;
        }
    }else{
      return UBX_Message;//Read Fail
    }
    return UBX_Message; //Read Success
  }
}

void GPS_Init(void){
  uint8_t UBX_MessageSend[32];
  char UBX_MessageReturn[32];
  char MessageBody[1] = {0x00};
  
  /*First Set Port Messages to UBX and deactive NMEA Messages, to do that set bit 14 to 1*/
  if(GPSnotConfig == true){
    int len = uUbxProtocolEncode(0x06, 0x00, MessageBody, 1, UBX_MessageSend);
    ublox_Write(len, UBX_MessageSend);

    UBX_MessageType UBX_CFG_PRT = ublox_ReadOutput(UBX_MessageReturn);
    if (UBX_CFG_PRT.messageClass == 6){
      UBX_CFG_PRT.messageBody[14] = 0x01; //Set bit 14 to 1 to disable NMEA messages
      int len = uUbxProtocolEncode(0x06, 0x00, UBX_CFG_PRT.messageBody, 19, UBX_MessageSend);

      ublox_Write(len, UBX_MessageSend);

      GPSnotConfig = false; //Set to false to not configure again
    }
  }else{
    GPS_ReadSensorData();
  }
}

ubx_nav_posllh_t GPS_ReadSensorData(void){
  uint8_t UBX_MessageSend[16];
  char UBX_MessageReturn[32];
  int len = uUbxProtocolEncode(0x01, 0x02, NULL, 0, UBX_MessageSend);
  ublox_Write(len, UBX_MessageSend);
  UBX_MessageType UBX_NAV_POSLLH  = ublox_ReadOutput(UBX_MessageReturn);
  
  ubx_nav_posllh_t posllh;
  memcpy(&posllh, UBX_NAV_POSLLH.messageBody, sizeof(ubx_nav_posllh_t));
  return posllh; //Return the position data
}

uint8_t GPS_VER_CHECK(void) {
    uint8_t UBX_MessageSend[8];
    char UBX_MessageReturn[200];
    int32_t messageClass;
    int32_t messageId;

    uUbxProtocolEncode(0x0A, 0x04, NULL, 0, UBX_MessageSend);

    ublox_Write(8, UBX_MessageSend);

    UBX_MessageType UBX_MON_VER = ublox_ReadOutput(UBX_MessageReturn);

    if(UBX_MON_VER.status != 1){
      return 0; //Read Fail
    }
    if((strncmp(UBX_MON_VER.messageBody, "ROM CORE 3.01 (107888)", 22) == 0)){
        return 1; //Read Success
    }
}