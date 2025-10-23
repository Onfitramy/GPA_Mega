#ifndef CLI_COMMANDS_H
#define CLI_COMMANDS_H

#include "cli_app.h"
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
#include "IMUS.h"
#include "FreeRTOS.h"
#include "Packets.h"
#include "InterBoardCom.h"

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

extern IMU_Data_t imu1_data;

//Internal commands are executed on this board, external commands are sent via radio to the other board
CLI_TargetMode_t cli_target_mode = CLI_TARGET_MODE_INTERNAL;

int _write(int file, char *data, int len)
{
    UNUSED(file);
    // Transmit data using USB
    while(CDC_Transmit_HS(data, len)==USBD_BUSY);
}

int sendcmdToTarget(DataPacket_t *packet) {
    if (cli_target_mode == CLI_TARGET_MODE_INTERNAL) {
        // Execute command locally (sends to secondary, then evaluated like normaly, potentially forwarded back to primary)
        InterBoardCom_SendDataPacket(INTERBOARD_OP_CMD | INTERBOARD_TARGET_MCU, packet);
        return 0; // Assume success for sending
    } else if (cli_target_mode == CLI_TARGET_MODE_EXTERNAL) {
        // Send command to other board via InterBoardCom
        InterBoardCom_SendDataPacket(INTERBOARD_OP_CMD | INTERBOARD_TARGET_RADIO, packet);
        return 0; // Assume success for sending
    }
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
BaseType_t cmd_switchCLIMode(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    const char *pcParameter;
    BaseType_t xParameterStringLength;

    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    if (pcParameter == NULL) {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing parameter 1\r\n");
        return pdFALSE;
    }

    if (strncmp(pcParameter, "internal", xParameterStringLength) == 0) {
        cli_target_mode = CLI_TARGET_MODE_INTERNAL;
        snprintf(pcWriteBuffer, xWriteBufferLen, "Switched to Internal CLI Mode\r\n");
    } else if (strncmp(pcParameter, "external", xParameterStringLength) == 0) {
        cli_target_mode = CLI_TARGET_MODE_EXTERNAL;
        snprintf(pcWriteBuffer, xWriteBufferLen, "Switched to External CLI Mode\r\n");
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Invalid parameter 1\r\n");
        return pdFALSE;
    }

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_resetPrimary(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SPECIAL, COMMAND_ID_PRIMARY_RESET, NULL, 0);
    sendcmdToTarget(&packet);

    /* Write the response to the buffer */
    snprintf(pcWriteBuffer, 30, "Resetting Primary MCU...\r\n");

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_resetSecondary(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SPECIAL, COMMAND_ID_SECONDARY_RESET, NULL, 0);
    sendcmdToTarget(&packet);

    /* Write the response to the buffer */
    snprintf(pcWriteBuffer, 30, "Resetting Secondary MCU...\r\n");

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_Camera_Power(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    const char *pcParameter;
    BaseType_t xParameterStringLength;
    char *endPtr;  // Pointer to track invalid characters

    uint8_t parameters[1];

    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    if (pcParameter == NULL) { //Handle to missing Input
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing parameter 1\r\n");
        return pdFALSE;
    }
    parameters[0] = (uint32_t)strtoul(pcParameter, &endPtr, 10);

    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_CAMERA, COMMAND_ID_CAMERA_POWER, parameters, sizeof(parameters));
    sendcmdToTarget(&packet);

    /* Write the response to the buffer */
    if (parameters[0] == 0)
        snprintf(pcWriteBuffer, 30, "Turning Camera OFF...\r\n");
    else
        snprintf(pcWriteBuffer, 30, "Turning Camera ON...\r\n");

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_Camera_Recording(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    const char *pcParameter;
    BaseType_t xParameterStringLength;
    char *endPtr;  // Pointer to track invalid characters

    uint8_t parameters[1];

    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    if (pcParameter == NULL) { //Handle to missing Input
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing parameter 1\r\n");
        return pdFALSE;
    }
    parameters[0] = (uint32_t)strtoul(pcParameter, &endPtr, 10);

    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_CAMERA, COMMAND_ID_CAMERA_RECORD, parameters, sizeof(parameters));
    sendcmdToTarget(&packet);

    /* Write the response to the buffer */
    if (parameters[0] == 0)
        snprintf(pcWriteBuffer, 30, "Stopping Video Recording...\r\n");
    else
        snprintf(pcWriteBuffer, 30, "Starting Video Recording...\r\n");

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_Camera_SkipDate(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;
    
    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_CAMERA, COMMAND_ID_CAMERA_SKIPDATE, NULL, 0);
    sendcmdToTarget(&packet);

    /* Write the response to the buffer */
    snprintf(pcWriteBuffer, 30, "Skipping Camera Date...\r\n");

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_Camera_Wifi(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    const char *pcParameter;
    BaseType_t xParameterStringLength;
    char *endPtr;  // Pointer to track invalid characters

    uint8_t parameters[1];

    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    if (pcParameter == NULL) { //Handle to missing Input
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing parameter 1\r\n");
        return pdFALSE;
    }
    parameters[0] = (uint32_t)strtoul(pcParameter, &endPtr, 10);

    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_CAMERA, COMMAND_ID_CAMERA_WIFI, parameters, sizeof(parameters));
    sendcmdToTarget(&packet);

    /* Write the response to the buffer */
    if (parameters[0] == 0)
        snprintf(pcWriteBuffer, 30, "Turning Camera WiFi OFF...\r\n");
    else
        snprintf(pcWriteBuffer, 30, "Turning Camera WiFi ON...\r\n");

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_State_Force(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    const char *pcParameter;
    BaseType_t xParameterStringLength;
    char *endPtr;  // Pointer to track invalid characters

    uint8_t parameters[1];

    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    if (pcParameter == NULL) { //Handle to missing Input
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing parameter 1\r\n");
        return pdFALSE;
    }
    parameters[0] = (uint32_t)strtoul(pcParameter, &endPtr, 10);

    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_STATE, COMMAND_ID_STATE_FORCE, parameters, sizeof(parameters));
    sendcmdToTarget(&packet);

    /* Write the response to the buffer */
    snprintf(pcWriteBuffer, 30, "Forcing Flight State %d...\r\n", parameters[0]);

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_SimulateEvent(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    const char *pcParameter;
    BaseType_t xParameterStringLength;
    char *endPtr;  // Pointer to track invalid characters

    uint8_t parameters[1];

    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    if (pcParameter == NULL) { //Handle to missing Input
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing parameter 1\r\n");
        return pdFALSE;
    }
    parameters[0] = (uint32_t)strtoul(pcParameter, &endPtr, 10);

    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_STATE, COMMAND_ID_STATE_SIMULATE_EVENT, parameters, sizeof(parameters));
    sendcmdToTarget(&packet);

    /* Write the response to the buffer */
    snprintf(pcWriteBuffer, 30, "Simulating Flight Event %d...\r\n", parameters[0]);

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_Logging_FlightDataOut(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    const char *pcParameter;
    BaseType_t xParameterStringLength;
    char *endPtr;  // Pointer to track invalid characters

    uint8_t parameters[1];

    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    if (pcParameter == NULL) { //Handle to missing Input
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing parameter 1\r\n");
        return pdFALSE;
    }
    parameters[0] = (uint32_t)strtoul(pcParameter, &endPtr, 10);

    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_LOGGING, 0, parameters, sizeof(parameters));
    sendcmdToTarget(&packet);

    /* Write the response to the buffer */
    if (parameters[0] == 0)
        snprintf(pcWriteBuffer, 50, "Turning Flight Data Output Logging OFF...\r\n");
    else
        snprintf(pcWriteBuffer, 50, "Turning Flight Data Output Logging ON...\r\n");

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_SPARK_SetAngle(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    const char *pcParameter;
    BaseType_t xParameterStringLength;
    char *endPtr;  // Pointer to track invalid characters

    float parameter;

    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    if (pcParameter == NULL) { //Handle to missing Input
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing parameter 1\r\n");
        return pdFALSE;
    }
    parameter = strtof(pcParameter, &endPtr);
    uint8_t parameters[sizeof(float)];
    memcpy(parameters, &parameter, sizeof(float));

    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SPARK, COMMAND_ID_SPARK_SET_ANGLE, parameters, sizeof(parameters));
    sendcmdToTarget(&packet);

    /* Write the response to the buffer */
    snprintf(pcWriteBuffer, 50, "Setting SPARK Target Angle to %.0f°...\r\n", parameter);

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_SPARK_SetSpeed(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    const char *pcParameter;
    BaseType_t xParameterStringLength;
    char *endPtr;  // Pointer to track invalid characters

    float parameter;

    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    if (pcParameter == NULL) { //Handle to missing Input
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing parameter 1\r\n");
        return pdFALSE;
    }
    parameter = strtof(pcParameter, &endPtr);
    uint8_t parameters[sizeof(float)];
    memcpy(parameters, &parameter, sizeof(float));

    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SPARK, COMMAND_ID_SPARK_SET_SPEED, parameters, sizeof(parameters));
    sendcmdToTarget(&packet);

    /* Write the response to the buffer */
    snprintf(pcWriteBuffer, 50, "Setting SPARK Target Speed to %.0f°/s...\r\n", parameter);

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_SPARK_ExitMode(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SPARK, COMMAND_ID_SPARK_EXIT_MODE, NULL, 0);
    sendcmdToTarget(&packet);

    /* Write the response to the buffer */
    snprintf(pcWriteBuffer, 30, "Exiting current mode...\r\n");

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_SPARK_ZeroStepper(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SPARK, COMMAND_ID_SPARK_ZERO_STEPPER, NULL, 0);
    sendcmdToTarget(&packet);

    /* Write the response to the buffer */
    snprintf(pcWriteBuffer, 30, "Activating SPARK Stepper Zero function...\r\n");

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_PU_toggleCAMPower(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    const char *pcParameter;
    BaseType_t xParameterStringLength;
    char *endPtr;  // Pointer to track invalid characters

    uint8_t parameters[1];

    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    if (pcParameter == NULL) { //Handle to missing Input
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing parameter 1\r\n");
        return pdFALSE;
    }
    parameters[0] = (uint32_t)strtoul(pcParameter, &endPtr, 10);

    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_POWERUNIT, COMMAND_ID_PU_POWER_CAM, parameters, sizeof(parameters));
    sendcmdToTarget(&packet);

    /* Write the response to the buffer */
    if (parameters[0] == 0)
        snprintf(pcWriteBuffer, 50, "Turning Camera power OFF...\r\n");
    else
        snprintf(pcWriteBuffer, 50, "Turning Camera power ON...\r\n");

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_PU_toggleRECPower(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    const char *pcParameter;
    BaseType_t xParameterStringLength;
    char *endPtr;  // Pointer to track invalid characters

    uint8_t parameters[1];

    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    if (pcParameter == NULL) { //Handle to missing Input
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing parameter 1\r\n");
        return pdFALSE;
    }
    parameters[0] = (uint32_t)strtoul(pcParameter, &endPtr, 10);

    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_POWERUNIT, COMMAND_ID_PU_POWER_RECOVERY, parameters, sizeof(parameters));
    sendcmdToTarget(&packet);

    /* Write the response to the buffer */
    if (parameters[0] == 0)
        snprintf(pcWriteBuffer, 50, "Turning Recovery power OFF...\r\n");
    else
        snprintf(pcWriteBuffer, 50, "Turning Recovery power ON...\r\n");

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_PU_toggleACSPower(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    const char *pcParameter;
    BaseType_t xParameterStringLength;
    char *endPtr;  // Pointer to track invalid characters

    uint8_t parameters[1];

    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    if (pcParameter == NULL) { //Handle to missing Input
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing parameter 1\r\n");
        return pdFALSE;
    }
    parameters[0] = (uint32_t)strtoul(pcParameter, &endPtr, 10);

    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_POWERUNIT, COMMAND_ID_PU_POWER_ACS, parameters, sizeof(parameters));
    sendcmdToTarget(&packet);

    /* Write the response to the buffer */
    if (parameters[0] == 0)
        snprintf(pcWriteBuffer, 50, "Turning ACS power OFF...\r\n");
    else
        snprintf(pcWriteBuffer, 50, "Turning ACS power ON...\r\n");

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
        .pcCommand = "switchCLIMode", /* The command string to type. */
        .pcHelpString = "switchCLIMode <mode>: Switches the CLI mode (internal/external)\r\n\r\n",
        .pxCommandInterpreter = cmd_switchCLIMode, /* The function to run. */
        .cExpectedNumberOfParameters = 1 /* One parameter is expected. */
    },
    {
        .pcCommand = "RESET_PRIMARY", /* The command string to type. */
        .pcHelpString = "RESET_PRIMARY: Resets the Primary MCU on the flight computer\r\n\r\n",
        .pxCommandInterpreter = cmd_resetPrimary, /* The function to run. */
        .cExpectedNumberOfParameters = 0
    },
    {
        .pcCommand = "RESET_SECONDARY", /* The command string to type. */
        .pcHelpString = "RESET_SECONDARY: Resets the Secondary MCU on the flight computer\r\n\r\n",
        .pxCommandInterpreter = cmd_resetSecondary, /* The function to run. */
        .cExpectedNumberOfParameters = 0
    },
    {
        .pcCommand = "Camera_Power", /* The command string to type. */
        .pcHelpString = "Camera_Power <1/0>: Turns the camera power on or off\r\n\r\n",
        .pxCommandInterpreter = cmd_Camera_Power, /* The function to run. */
        .cExpectedNumberOfParameters = 1
    },
    {
        .pcCommand = "Camera_Recording", /* The command string to type. */
        .pcHelpString = "Camera_Recording <1/0>: Turns the camera recording on or off\r\n\r\n",
        .pxCommandInterpreter = cmd_Camera_Recording, /* The function to run. */
        .cExpectedNumberOfParameters = 1
    },
    {
        .pcCommand = "Camera_SkipDate", /* The command string to type. */
        .pcHelpString = "Camera_SkipDate: Skips the current date for the camera\r\n\r\n",
        .pxCommandInterpreter = cmd_Camera_SkipDate, /* The function to run. */
        .cExpectedNumberOfParameters = 0
    },
    {
        .pcCommand = "Camera_Wifi", /* The command string to type. */
        .pcHelpString = "Camera_Wifi <1/0>: Turns the camera WiFi on or off\r\n\r\n",
        .pxCommandInterpreter = cmd_Camera_Wifi, /* The function to run. */
        .cExpectedNumberOfParameters = 1
    },
    {
        .pcCommand = "State_Force", /* The command string to type. */
        .pcHelpString = "State_Force <x>: Force the Statemachine into state x\r\n\r\n",
        .pxCommandInterpreter = cmd_State_Force, /* The function to run. */
        .cExpectedNumberOfParameters = 1
    },
    {
        .pcCommand = "SimulateEvent", /* The command string to type. */
        .pcHelpString = "SimulateEvent <event>: Simulate state machine event <event>\r\n\r\n",
        .pxCommandInterpreter = cmd_SimulateEvent, /* The function to run. */
        .cExpectedNumberOfParameters = 1
    },
    {
        .pcCommand = "Logging_FlightDataOut", /* The command string to type. */
        .pcHelpString = "Logging_FlightDataOut <Enable/Disable>: Enables or disables flight data output logging to PC\r\n\r\n",
        .pxCommandInterpreter = cmd_Logging_FlightDataOut, /* The function to run. */
        .cExpectedNumberOfParameters = 1
    },
    {
        .pcCommand = "SPARK_SetAngle", /* The command string to type. */
        .pcHelpString = "SPARK_SetAngle <float>: Sets target angle of the stepper motor\r\n\r\n",
        .pxCommandInterpreter = cmd_SPARK_SetAngle, /* The function to run. */
        .cExpectedNumberOfParameters = 1
    },
    {
        .pcCommand = "SPARK_SetSpeed", /* The command string to type. */
        .pcHelpString = "SPARK_SetSpeed <float>: Sets target speed of the stepper motor\r\n\r\n",
        .pxCommandInterpreter = cmd_SPARK_SetSpeed, /* The function to run. */
        .cExpectedNumberOfParameters = 1
    },
    {
        .pcCommand = "SPARK_ExitMode", /* The command string to type. */
        .pcHelpString = "SPARK_ExitMode: Exits the current target mode\r\n\r\n",
        .pxCommandInterpreter = cmd_SPARK_ExitMode, /* The function to run. */
        .cExpectedNumberOfParameters = 0
    },
    {
        .pcCommand = "SPARK_ZeroStepper", /* The command string to type. */
        .pcHelpString = "SPARK_ZeroStepper: Resets the stepper motor position\r\n\r\n",
        .pxCommandInterpreter = cmd_SPARK_ZeroStepper, /* The function to run. */
        .cExpectedNumberOfParameters = 0
    },
    {
        .pcCommand = "PU_setCAMPower", /* The command string to type. */
        .pcHelpString = "PU_setCAMPower <1/0>: toggles Camera power\r\n\r\n",
        .pxCommandInterpreter = cmd_PU_toggleCAMPower, /* The function to run. */
        .cExpectedNumberOfParameters = 1
    },
    {
        .pcCommand = "PU_setRecoveryPower", /* The command string to type. */
        .pcHelpString = "PU_setRecoveryPower <1/0>: toggles Recovery power\r\n\r\n",
        .pxCommandInterpreter = cmd_PU_toggleRECPower, /* The function to run. */
        .cExpectedNumberOfParameters = 1
    },
    {
        .pcCommand = "PU_setACSPower", /* The command string to type. */
        .pcHelpString = "PU_setACSPower <1/0>: toggles ACS power\r\n\r\n",
        .pxCommandInterpreter = cmd_PU_toggleACSPower, /* The function to run. */
        .cExpectedNumberOfParameters = 1
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