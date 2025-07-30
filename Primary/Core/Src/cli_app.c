#ifndef CLI_COMMANDS_H
#define CLI_COMMANDS_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "FreeRTOS_CLI.h"
#include "stdbool.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "usbd_cdc_if.h"
#include "ws2812.h"
#include "LSM6DSR.h"
#include "freeRTOS.h"

#define MAX_INPUT_LENGTH 50
#define USING_VS_CODE_TERMINAL 0
#define USING_OTHER_TERMINAL 1 // e.g. Putty, TerraTerm

char cOutputBuffer[configCOMMAND_INT_MAX_OUTPUT_SIZE], pcInputString[MAX_INPUT_LENGTH];
extern const CLI_Command_Definition_t xCommandList[];
extern StreamBufferHandle_t xStreamBuffer;
int8_t cRxedChar;
const char * cli_prompt = "\r\ncli> ";
/* CLI escape sequences*/
uint8_t backspace[] = "\b \b";
uint8_t backspace_tt[] = " \b";

extern LSM6DSR_Data_t imu1_data;

int _write(int file, char *data, int len)
{
    UNUSED(file);
    // Transmit data using USB
    while(CDC_Transmit_HS(data, len)==USBD_BUSY);
}
//*****************************************************************************
BaseType_t cmd_clearScreen(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    /* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
    (void)pcCommandString;
    (void)xWriteBufferLen;
    memset(pcWriteBuffer, 0x00, xWriteBufferLen);
    printf("\033[2J\033[1;1H");
    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_set_led(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString; // comntains the command string
    (void)xWriteBufferLen; // contains the length of the write buffer

    const char *pcParameter;
    BaseType_t xParameterStringLength;

    int r, g, b;

    // Get the first parameter (R)
    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    if (pcParameter == NULL) {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing parameter 1 (R)\r\n");
        return pdFALSE;
    }
    r = atoi(pcParameter); // Convert to integer

    // Get the second parameter (G)
    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength);
    if (pcParameter == NULL) {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing parameter 2 (G)\r\n");
        return pdFALSE;
    }
    g = atoi(pcParameter); // Convert to integer

    // Get the third parameter (B)
    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 3, &xParameterStringLength);
    if (pcParameter == NULL) {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing parameter 3 (B)\r\n");
        return pdFALSE;
    }
    b = atoi(pcParameter); // Convert to integercCommandString, 1, &xParameterStringLength );
    
    /* Toggle the LED */
    Set_LED(r, g, b);
    WS2812_Send();

    
    /* Write the response to the buffer */
    uint8_t string[] = "LED set\r\n";
    strcpy(pcWriteBuffer, (char *)string);
    
    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_read_IMU1(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    /* Write the response to the buffer */
    snprintf(pcWriteBuffer, 100, "Accelerometer: %d, %d, %d Gyroscope: %d, %d, %d\r\n", imu1_data.accel[0], imu1_data.accel[1], imu1_data.accel[2], imu1_data.gyro[0], imu1_data.accel[1], imu1_data.gyro[3]);

    return pdFALSE;
}

BaseType_t cmd_Flash(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    const char *pcParameter;

    const char *mode;
    uint32_t page;
    uint32_t size;
    BaseType_t xParameterStringLength;
    char *endPtr;  // Pointer to track invalid characters
    
    //Read Page number
    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength);
    if (pcParameter == NULL) { //Handle to missing Input
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing parameter 2 (B)\r\n");
        return pdFALSE;
    }
    page = (uint32_t)strtoul(pcParameter, &endPtr, 10);

    //Read Size
    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 3, &xParameterStringLength);
    if (pcParameter == NULL) { //Handle to missing Input
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing parameter 3 (B)\r\n");
        return pdFALSE;
    }
    size = (uint32_t)strtoul(pcParameter, &endPtr, 10);

    uint8_t data[size];

    //Set mode, reset/read/write
    mode = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    if (strncmp(mode, "Reset", 5) == 0){
        //W25Q1_Reset();
        uint8_t string[] = "FLASH Reset\r\n";
        strcpy(pcWriteBuffer, (char *)string);
    } else if (strncmp(mode, "Read ", 5) == 0)
    {
        //W25Q_Read(page, 0, size, data);
        strcpy(pcWriteBuffer, (char *)data);
    } else if (strncmp(mode, "Write", 5) == 0){
        pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 4, &xParameterStringLength);
        memcpy(data, pcParameter, size);
        //W25Q_Write(page, 0, size, data);
        
        uint8_t string[] = "Data Written\r\n";
        strcpy(pcWriteBuffer, (char *)string);
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing/Wrong parameter 1 (B)\r\n"); 
    }

    /* Write the response to the buffer */

    return pdFALSE;
}

const CLI_Command_Definition_t xCommandList[] = {
    {
        .pcCommand = "cls", /* The command string to type. */
        .pcHelpString = "cls: Clears screen\r\n\r\n",
        .pxCommandInterpreter = cmd_clearScreen, /* The function to run. */
        .cExpectedNumberOfParameters = 0 /* No parameters are expected. */
    },
    {
        .pcCommand = "set_LED", /* The command string to type. */
        .pcHelpString = "set_LED[R,G,B]: Set LED to a color(max 255)\r\n\r\n",
        .pxCommandInterpreter = cmd_set_led, /* The function to run. */
        .cExpectedNumberOfParameters = 3 /* No parameters are expected. */
    },
    {
        .pcCommand = "read_IMU1", /* The command string to type. */
        .pcHelpString = "read_IMU1: Read IMU1\r\n\r\n",
        .pxCommandInterpreter = cmd_read_IMU1, /* The function to run. */
        .cExpectedNumberOfParameters = 0 /* No parameters are expected. */
    },
    {
        .pcCommand = "Flash", /* The command string to type. */
        .pcHelpString = "Flash[Mode, Param1, Param2, Param3]: Reset, Read, Write data from at Page Param1 of size Param2, data is Param3\r\n\r\n",
        .pxCommandInterpreter = cmd_Flash, /* The function to run. */
        .cExpectedNumberOfParameters = 4 /* No parameters are expected. */
    },
    {
        .pcCommand = NULL /* simply used as delimeter for end of array*/
    }
};

void vRegisterCLICommands(void){
    //itterate thourgh the list of commands and register them
    for (int i = 0; xCommandList[i].pcCommand != NULL; i++)
    {
        FreeRTOS_CLIRegisterCommand(&xCommandList[i]);
    }
}
/*************************************************************************************************/
void cliWrite(const char *str)
{
    printf("%s", str);
    // flush stdout
    fflush(stdout);
}
/*************************************************************************************************/
/*Continous Data Output to log signals*/
void continousSignal(void *argument){

        cliWrite("Test");
}

void handleNewline(const char *const pcInputString, char *cOutputBuffer, uint8_t cInputIndex)
{
    cliWrite("\r\n");

    BaseType_t xMoreDataToFollow;
    do
    {     
        xMoreDataToFollow = FreeRTOS_CLIProcessCommand(pcInputString, cOutputBuffer, configCOMMAND_INT_MAX_OUTPUT_SIZE);
        cliWrite(cOutputBuffer);
    } while (xMoreDataToFollow != pdFALSE);

    cliWrite(cli_prompt);
}
/*************************************************************************************************/
void handleBackspace(uint8_t *cInputIndex, char *pcInputString)
{
    if (*cInputIndex > 0)
    {
        (*cInputIndex)--;
        pcInputString[*cInputIndex] = '\0';

#if USING_VS_CODE_TERMINAL
        cliWrite((char *)backspace);
#elif USING_OTHER_TERMINAL
        cliWrite((char *)backspace_tt);
#endif
    }
    else
    {
#if USING_OTHER_TERMINAL
        uint8_t right[] = "\x1b\x5b\x43";
        cliWrite((char *)right);
#endif
    }
}
/*************************************************************************************************/
void handleCharacterInput(uint8_t *cInputIndex, char *pcInputString)
{
    if (cRxedChar == '\r')
    {
        return;
    }
    else if (cRxedChar == (uint8_t)0x08 || cRxedChar == (uint8_t)0x7F)
    {
        handleBackspace(cInputIndex, pcInputString);
    }
    else
    {
        if (*cInputIndex < MAX_INPUT_LENGTH)
        {
            pcInputString[*cInputIndex] = cRxedChar;
            (*cInputIndex)++;
        }
    }
}
/*************************************************************************************************/
void vCommandConsoleTask(void *pvParameters)
{
    uint8_t cInputIndex = 0; // simply used to keep track of the index of the input string
    char receivedData[50];; // used to store the received value from the notification
    UNUSED(pvParameters);
    vRegisterCLICommands();
    
    for (;;)
    {

       size_t bytesRead = xStreamBufferReceive(xStreamBuffer, receivedData, sizeof(receivedData), portMAX_DELAY);
        if (bytesRead > 0) {
            cliWrite(receivedData);
            receivedData[bytesRead-1] = 0x00; //Strip of \r for analysis
            handleNewline(receivedData, cOutputBuffer, strlen(receivedData));
            xStreamBufferReset(xStreamBuffer);
        }
    }
}
#endif /* CLI_COMMANDS_H */