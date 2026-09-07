#ifndef CLI_COMMANDS_H
#define CLI_COMMANDS_H

#include "cli_app.h"

#include "_components.h"
#include "_libraries.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "FreeRTOS_CLI.h"
#include "stdbool.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "usbd_cdc_if.h"
#include "tests_app.h"

#define MAX_INPUT_LENGTH 50
#define USING_VS_CODE_TERMINAL 0
#define USING_OTHER_TERMINAL 1 // e.g. Putty, TerraTerm

#define FLASH_CLI_MAX_SIZE 1024u

#define ARRAY_LEN(array) (sizeof(array) / sizeof((array)[0]))
static BaseType_t register_list_index = -1;

/**
  ******************************************************************************
  * File Description : 
  * This file implements a command-line interface (CLI) for the embedded system, allowing users to interact with the system through a terminal. 
  * It defines various commands that can be executed to perform actions such as clearing the screen, switching CLI modes, and changing output schedules. 
  * The CLI supports both internal commands that are executed on the local board and external commands that are sent to another board via radio communication.
  ******************************************************************************
  */

char cOutputBuffer[configCOMMAND_INT_MAX_OUTPUT_SIZE];
extern const CLI_Command_Definition_t xCommandList[];
extern StreamBufferHandle_t xStreamBuffer;
int8_t cRxedChar;
const char * cli_prompt = "\r\ncli> ";
/* CLI escape sequences*/
uint8_t backspace[] = "\b \b";
uint8_t backspace_tt[] = " \b";

extern IMU_Data_t imu1_data;
uint32_t system_version = 0x00009500; // Version 0.9.5
uint32_t empty_reg = 0;

//Function prototypes for register write and read callbacks
bool reg_write_radio_mode(const void *value);

static const reg_descriptor_t registers[] = {
    {
        .name = "system.version",
        .description = "System version number",
        .address = (void *)&system_version,
        .type = REG_TYPE_U32,
        .access = REG_ACCESS_READ
    },
    {
        .name = "system.FHPlotter.out",
        .description = "FH Plotter output 1/0",
        .address = (void *)&signalPlotterSend,
        .type = REG_TYPE_BOOL,
        .access = REG_ACCESS_READ|REG_ACCESS_WRITE
    },
    {
        .name = "system.messageSchedule",
        .description = "Set message schedule 0-6",
        .address = (void *)&empty_reg,
        .type = REG_TYPE_U32,
        .access = REG_ACCESS_WRITE,
        .min = 0,
        .max = 7,
        .custom_write = SetComSchedule
    },
    {
        .name = "system.radioMode",
        .description = "Set radio mode 0-2",
        .address = (void *)&empty_reg,
        .type = REG_TYPE_U32,
        .access = REG_ACCESS_WRITE,
        .min = 0,
        .max = 2,
        .custom_write = reg_write_radio_mode
    },
    {
        .name = "system.CLITargetMode",
        .description = "Set CLI target 0=internal, 1=external",
        .address = (void *)&cli_target_mode,
        .type = REG_TYPE_BOOL,
        .access = REG_ACCESS_READ|REG_ACCESS_WRITE,
    },
    {
        .name = "sensor.imu1.accel.x",
        .description = "IMU 1 X-axis acceleration",
        .address = (void *)&imu1_data.accel[0],
        .type = REG_TYPE_FLOAT,
        .access = REG_ACCESS_READ
    },
    {
        .name = "sensor.imu1.accel.y",
        .description = "IMU 1 Y-axis acceleration",
        .address = (void *)&imu1_data.accel[1],
        .type = REG_TYPE_FLOAT,
        .access = REG_ACCESS_READ
    },
    {
        .name = "sensor.imu1.accel.z",
        .description = "IMU 1 Z-axis acceleration",
        .address = (void *)&imu1_data.accel[2],
        .type = REG_TYPE_FLOAT,
        .access = REG_ACCESS_READ
    },
    {
        .name = "sensor.imu1.gyro.x",
        .description = "IMU 1 X-axis gyroscope",
        .address = (void *)&imu1_data.gyro[0],
        .type = REG_TYPE_FLOAT,
        .access = REG_ACCESS_READ
    },
    {
        .name = "sensor.imu1.gyro.y",
        .description = "IMU 1 Y-axis gyroscope",
        .address = (void *)&imu1_data.gyro[1],
        .type = REG_TYPE_FLOAT,
        .access = REG_ACCESS_READ
    },
    {
        .name = "sensor.imu1.gyro.z",
        .description = "IMU 1 Z-axis gyroscope",
        .address = (void *)&imu1_data.gyro[2],
        .type = REG_TYPE_FLOAT,
        .access = REG_ACCESS_READ
    },
};

//Internal commands are executed on this board, external commands are sent via radio to the other board
CLI_TargetMode_t cli_target_mode = CLI_TARGET_MODE_INTERNAL;

int _write(int file, char *data, int len)
{
    UNUSED(file);
    // Transmit data using USB
    while(CDC_Transmit_HS((uint8_t*)data, len)==USBD_BUSY){
        vTaskDelay(1);
    };
    return len;
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
    return -1; // Invalid target mode
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
BaseType_t cmd_regList(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    if (register_list_index == -1) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Registered Registers:\r\n");

        register_list_index = 0;
        return pdTRUE;
    }

    snprintf(pcWriteBuffer, xWriteBufferLen,
             "  %s: %s\r\n",
             registers[register_list_index].name,
             registers[register_list_index].description);

    register_list_index++;

    if ((size_t)register_list_index < ARRAY_LEN(registers)) {
        return pdTRUE;  // FreeRTOS CLI calls again
    }

    register_list_index = -1;
    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_regGet(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    const char *pcParameter;
    BaseType_t xParameterStringLength;

    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    if (pcParameter == NULL) {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing register name\r\n");
        return pdFALSE;
    }

    for (int i = 0; i < ARRAY_LEN(registers); i++) {
        if (strlen(registers[i].name) == (size_t)xParameterStringLength &&
            strncmp(pcParameter, registers[i].name, xParameterStringLength) == 0) {
            if (registers[i].access & REG_ACCESS_READ) {
                if (registers[i].custom_read) {
                    if (!registers[i].custom_read(pcWriteBuffer)) {
                        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Custom read failed\r\n");
                    }
                } else {
                    switch (registers[i].type) {
                        case REG_TYPE_U32:
                            snprintf(pcWriteBuffer, xWriteBufferLen, "%u\r\n", *(uint32_t *)registers[i].address);
                            break;
                        case REG_TYPE_I32:
                            snprintf(pcWriteBuffer, xWriteBufferLen, "%d\r\n", *(int32_t *)registers[i].address);
                            break;
                        case REG_TYPE_FLOAT:
                            snprintf(pcWriteBuffer, xWriteBufferLen, "%.6f\r\n", *(float *)registers[i].address);
                            break;
                        case REG_TYPE_BOOL:
                            snprintf(pcWriteBuffer, xWriteBufferLen, "%s\r\n", (*(bool *)registers[i].address) ? "true" : "false");
                            break;
                        default:
                            snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Unknown register type\r\n");
                            break;
                    }
                }
            } else {
                snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Register is not readable\r\n");
            }
            return pdFALSE;
        }
    }
    snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Unknown register\r\n");
    return pdFALSE;
}

#define ARRAY_LEN(x) (sizeof(x) / sizeof((x)[0]))

static BaseType_t cmd_regSet(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    BaseType_t name_len, value_len; 
    const char *name = FreeRTOS_CLIGetParameter(pcCommandString, 1, &name_len);
    const char *text = FreeRTOS_CLIGetParameter(pcCommandString, 2, &value_len);

    if (!name || !text || value_len >= 32) {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Usage: reg-set <name> <value>\r\n");
        return pdFALSE;
    }

    const reg_descriptor_t *reg = NULL;

    for (size_t i = 0; i < ARRAY_LEN(registers); i++) {
        if (strlen(registers[i].name) == (size_t)name_len &&
            strncmp(registers[i].name, name, name_len) == 0) {
            reg = &registers[i];
            break;
        }
    }

    if (!reg) {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Unknown register: %.*s\r\n",
                 (int)name_len, name);
        return pdFALSE;
    }

    if (!(reg->access & REG_ACCESS_WRITE)) {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Register is read-only\r\n");
        return pdFALSE;
    }

    char buffer[32];
    memcpy(buffer, text, value_len);
    buffer[value_len] = '\0';

    char *end;
    float value = strtof(buffer, &end);
    if (buffer == end) {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Invalid number\r\n");
        return pdFALSE; // Invalid number error
    }

    if (reg->min != 0.0f || reg->max != 0.0f) { //Check if min and max are set (not 0.0f)
        if (*end || !isfinite(value) ||
            value < reg->min || value > reg->max) {
            snprintf(pcWriteBuffer, xWriteBufferLen, "Invalid value [%.3f, %.3f]\r\n",
                    reg->min, reg->max);
            return pdFALSE;
        }
    }

    union {
        uint32_t u32;
        int32_t  i32;
        float    f32;
        bool     boolean;
    } converted;

    const void *src;
    size_t size;

    switch (reg->type) {
        case REG_TYPE_U32:
            converted.u32 = (uint32_t)value;
            src = &converted.u32;
            size = sizeof(converted.u32);
            break;

        case REG_TYPE_I32:
            converted.i32 = (int32_t)value;
            src = &converted.i32;
            size = sizeof(converted.i32);
            break;

        case REG_TYPE_FLOAT:
            converted.f32 = value;
            src = &converted.f32;
            size = sizeof(converted.f32);
            break;

        case REG_TYPE_BOOL:
            if (value != 0.0f && value != 1.0f) {
                snprintf(pcWriteBuffer, xWriteBufferLen, "Boolean must be 0 or 1\r\n");
                return pdFALSE;
            }

            converted.boolean = value != 0.0f;
            src = &converted.boolean;
            size = sizeof(converted.boolean);
            break;

        default:
            snprintf(pcWriteBuffer, xWriteBufferLen, "Unsupported register type\r\n");
            return pdFALSE;
    }

    bool success;

    if (reg->custom_write) {
        success = reg->custom_write(src);
    } else if (reg->address) {
        taskENTER_CRITICAL();
        memcpy(reg->address, src, size);
        taskEXIT_CRITICAL();
        success = true;
    } else {
        success = false;
    }

    snprintf(pcWriteBuffer, xWriteBufferLen, success
             ? "%s written\r\n"
             : "Failed to write %s\r\n",
             reg->name);

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
    snprintf(pcWriteBuffer, 33, "Simulating Flight Event %d...\r\n", parameters[0]);

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

    SPARK_SetAngle(parameter);

    /* Write the response to the buffer */
    snprintf(pcWriteBuffer, 50, "Setting SPARK Target Angle to %.2f°...\r\n", parameter);

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
    
    SPARK_SetSpeed(parameter);

    /* Write the response to the buffer */
    snprintf(pcWriteBuffer, 50, "Setting SPARK Target Speed to %.2f°/s...\r\n", parameter);

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_SPARK_ExitMode(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    SPARK_ExitMode();

    /* Write the response to the buffer */
    snprintf(pcWriteBuffer, 30, "Exiting current mode...\r\n");

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_SPARK_ZeroStepper(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    SPARK_ZeroStepper();

    /* Write the response to the buffer */
    snprintf(pcWriteBuffer, 50, "Activating SPARK Stepper Zero function...\r\n");

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_SPARK_FindMax(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    SPARK_FindMax();

    /* Write the response to the buffer */
    snprintf(pcWriteBuffer, 50, "Activating SPARK Stepper Find Max function...\r\n");

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_SPARK_TargetPositionMode(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
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

    SPARK_TargetPositionMode(parameters[0]);

    /* Write the response to the buffer */
    snprintf(pcWriteBuffer, 60, "Activating SPARK Target Position mode with %d/16 TRQ...\r\n", parameters[0]);

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_SPARK_TargetSpeedMode(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
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

    SPARK_TargetSpeedMode(parameters[0]);

    /* Write the response to the buffer */
    snprintf(pcWriteBuffer, 60, "Activating SPARK Target Speed mode with %d/16 TRQ...\r\n", parameters[0]);

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_SPARK_Reset(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    SPARK_Reset();

    /* Write the response to the buffer */
    snprintf(pcWriteBuffer, 50, "Resetting SPARK...\r\n");

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_ACS_SetAngle(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    const char *pcParameter;
    BaseType_t xParameterStringLength;
    char *endPtr;  // Pointer to track invalid characters

    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    if (pcParameter == NULL) { //Handle to missing Input
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing parameter 1\r\n");
        return pdFALSE;
    }
    acs_target_angle_deg = strtof(pcParameter, &endPtr);

    ACS_SetAngle(acs_target_angle_deg);

    /* Write the response to the buffer */
    snprintf(pcWriteBuffer, 50, "Setting ACS Target Angle to %.2f°...\r\n", acs_target_angle_deg);

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_SPARK_SetNeutralAngle(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    const char *pcParameter;
    BaseType_t xParameterStringLength;
    char *endPtr;  // Pointer to track invalid characters

    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    if (pcParameter == NULL) { //Handle to missing Input
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing parameter 1\r\n");
        return pdFALSE;
    }
    stepper_neutral_angle = strtof(pcParameter, &endPtr);

    StepperPositionFromACSAngle(0.f, &stepper_zero_position);
    stepper_zero_position += stepper_neutral_angle / 360.f * ROD_SLOPE;

    /* Write the response to the buffer */
    snprintf(pcWriteBuffer, 50, "Setting Stepper Neutral Angle to %.2f°...\r\n", stepper_neutral_angle);

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
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SECONDARY, COMMAND_ID_PU_POWER_CAM, parameters, sizeof(parameters));
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
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SECONDARY, COMMAND_ID_PU_POWER_RECOVERY, parameters, sizeof(parameters));
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
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SECONDARY, COMMAND_ID_PU_POWER_ACS, parameters, sizeof(parameters));
    sendcmdToTarget(&packet);

    /* Write the response to the buffer */
    if (parameters[0] == 0)
        snprintf(pcWriteBuffer, 50, "Turning ACS power OFF...\r\n");
    else
        snprintf(pcWriteBuffer, 50, "Turning ACS power ON...\r\n");

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_Buzzer_PlayNote(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    const char *pcParameter1;
    const char *pcParameter2;
    BaseType_t xParameterStringLength1;
    BaseType_t xParameterStringLength2;
    char *endPtr;  // Pointer to track invalid characters

    uint8_t parameters[6];

    pcParameter1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1);
    pcParameter2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength2);
    if (pcParameter1 == NULL) { //Handle to missing Input
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing parameter 1\r\n");
        return pdFALSE;
    }
    if (pcParameter2 == NULL) { //Handle to missing Input
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing parameter 2\r\n");
        return pdFALSE;
    }

    uint16_t delay_ms = (uint32_t)strtoul(pcParameter2, &endPtr, 10);
    parameters[0] = (uint8_t)(delay_ms >> 8);
    parameters[1] = (uint8_t)delay_ms;
    parameters[2] = xParameterStringLength1;
    parameters[3] = pcParameter1[0];
    parameters[4] = pcParameter1[1];
    if (xParameterStringLength1 == 3) {
        parameters[5] = pcParameter1[2];
    }

    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SECONDARY, COMMAND_ID_BUZZER_PLAYNOTE, parameters, sizeof(parameters));
    sendcmdToTarget(&packet);

    /* Write the response to the buffer */
    snprintf(pcWriteBuffer, 50, "Playing Note %c%c for %.3f seconds...\r\n", parameters[0], parameters[1], (float)delay_ms / 1000.f);

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_Buzzer_PlaySong(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
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
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SECONDARY, COMMAND_ID_BUZZER_PLAYSONG, parameters, sizeof(parameters));
    sendcmdToTarget(&packet);

    /* Write the response to the buffer */
    snprintf(pcWriteBuffer, 30, "Playing Song %d...\r\n", parameters[0]);

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_Buzzer_PlaySongRepeat(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    const char *pcParameter1;
    const char *pcParameter2;
    BaseType_t xParameterStringLength1;
    BaseType_t xParameterStringLength2;
    char *endPtr;  // Pointer to track invalid characters

    uint8_t parameters[3];

    pcParameter1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1);
    pcParameter2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength2);
    if (pcParameter1 == NULL) { //Handle to missing Input
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing parameter 1\r\n");
        return pdFALSE;
    }
    if (pcParameter2 == NULL) { //Handle to missing Input
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing parameter 2\r\n");
        return pdFALSE;
    }

    uint16_t delay_ms = (uint32_t)strtoul(pcParameter2, &endPtr, 10);
    parameters[0] = (uint8_t)(delay_ms >> 8);
    parameters[1] = (uint8_t)delay_ms;
    parameters[2] = (uint32_t)strtoul(pcParameter1, &endPtr, 10);

    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SECONDARY, COMMAND_ID_BUZZER_PLAYSONGREPEAT, parameters, sizeof(parameters));
    sendcmdToTarget(&packet);

    /* Write the response to the buffer */
    snprintf(pcWriteBuffer, 50, "Playing Song %d ON REPEAT every %.2f seconds!!!\r\n", parameters[2], (float)delay_ms / 1000.f);

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_Buzzer_Stop(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;
    
    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_SECONDARY, COMMAND_ID_BUZZER_STOPALL, NULL, 0);
    sendcmdToTarget(&packet);

    /* Write the response to the buffer */
    snprintf(pcWriteBuffer, 50, "Stopping annoying buzzing activities...\r\n");

    return pdFALSE;
}

bool reg_write_radio_mode(const void *value)
{
    uint8_t mode = *(const uint8_t *)value;
    if (mode != 1 && mode != 2) {
        return false;
    }
    
    uint8_t parameters[1];
    parameters[0] = mode;

    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_RADIO, COMMAND_ID_RADIO_SWITCH, parameters, sizeof(parameters));
    sendcmdToTarget(&packet);
    return true;
}

//*****************************************************************************
BaseType_t cmd_Storage_FlashToSD(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    const char *pcParameter;
    BaseType_t xParameterStringLength;
    char *endPtr;  // Pointer to track invalid characters

    uint8_t parameters[3];

    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    uint32_t parameter;
    if (pcParameter == NULL) { //Handle to missing Input
        parameter = 0;
    } else {
        parameter = (uint32_t)strtoul(pcParameter, &endPtr, 10);

        if (parameter < 1024 || parameter > 65535) {
            snprintf(pcWriteBuffer, xWriteBufferLen, "Page must me in the range between 1024 and 65535\r\n");
            return pdFALSE;
        }
    }
    uint16_t parameter_16 = (uint16_t)parameter;
    memcpy(parameters, &parameter_16, sizeof(parameter_16));

    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_STORAGE, COMMAND_ID_STORAGE_FLASH_TO_SD, parameters, sizeof(parameters));
    sendcmdToTarget(&packet);

    /* Write the response to the buffer */
    if (parameter == 0) {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Transferring data from FLASH to SD card up to current config page\r\n");
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Transferring data from FLASH to SD card up to page %d \r\n", (int)parameter);
    }

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_Storage_LogsToSerial(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{

    (void)pcCommandString;
    (void)xWriteBufferLen;

    const char *pcParameter;
    BaseType_t xParameterStringLength;
    char *endPtr;  // Pointer to track invalid characters

    uint8_t parameters[3];

    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    uint32_t parameter;
    if (pcParameter == NULL) { //Handle to missing Input
        parameter = 0;
    } else {
        parameter = (uint32_t)strtoul(pcParameter, &endPtr, 10);

        if (parameter < 1024 || parameter > 65535) {
            snprintf(pcWriteBuffer, xWriteBufferLen, "Page must me in the range between 1024 and 65535\r\n");
            return pdFALSE;
        }
    }
    uint16_t parameter_16 = (uint16_t)parameter;
    memcpy(parameters, &parameter_16, sizeof(parameter_16));

    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_STORAGE, COMMAND_ID_STORAGE_FLASH_TO_SERIAL, parameters, sizeof(parameters));
    sendcmdToTarget(&packet);

    /* Write the response to the buffer */
    if (parameter == 0) {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Transferring data from FLASH to serial interface up to current config page\r\n");
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Transferring data from FLASH to serial interface up to page %d\r\n");
    }

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_Storage_FlashErase(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;
    
    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_STORAGE, COMMAND_ID_STORAGE_FLASH_ERASE, NULL, 0);
    sendcmdToTarget(&packet);

    /* Write the response to the buffer */
    snprintf(pcWriteBuffer, 50, "Erasing FLASH memory...\r\n");

    return pdFALSE;
}

//*****************************************************************************
BaseType_t cmd_Storage_FlashWrite(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
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

    /* Write the response to the buffer */
    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_STORAGE, COMMAND_ID_STORAGE_FLASH_WRITE, parameters, sizeof(parameters));
    sendcmdToTarget(&packet);

    snprintf(pcWriteBuffer, 50, "Flash Saving set to %d\r\n", parameters[0]);

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

    (void)page; // To avoid unused variable warning, will be assigned before use
    
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

    if (size == 0u || size > FLASH_CLI_MAX_SIZE) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                "Error: size must be 1..%u\r\n",
                (unsigned)FLASH_CLI_MAX_SIZE);
        return pdFALSE;
    }

    uint8_t data[FLASH_CLI_MAX_SIZE];

    //Set mode, reset/read/write
    mode = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    if (strncmp(mode, "Reset", 5) == 0){
        //W25Q1_Reset();
        snprintf(pcWriteBuffer, xWriteBufferLen, "FLASH Reset\r\n");
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

//*****************************************************************************
BaseType_t cmd_TestRun(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    BaseType_t xParameterStringLength;
    //char *endPtr;  // Pointer to track invalid characters

    const char *test; // Test name
    test = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    if (test == NULL) { //Handle to missing Input
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing parameter 1\r\n");
        return pdFALSE;
    }

    test_result_t result = single_test_run(test);

    if (result.test_status == TEST_STATUS_PASS) {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Test %s PASSED\r\n", test);
    } else if (result.test_status == TEST_STATUS_NOT_FOUND) {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Test %s NOT FOUND\r\n", test);
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Test %s FAILED with code %d\r\n", test, result.test_status);
    }

    return pdFALSE;
}


//*****************************************************************************
BaseType_t cmd_TestSuiteRun(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    BaseType_t xParameterStringLength;
    //char *endPtr;  // Pointer to track invalid characters

    const char *suite_name; // Test name
    suite_name = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    if (suite_name == NULL) { //Handle to missing Input
        snprintf(pcWriteBuffer, xWriteBufferLen, "Error: Missing parameter 1\r\n");
        return pdFALSE;
    }

    test_suite_result_t result = test_suite_run(suite_name);

    if (result.overall_status == TEST_STATUS_PASS) {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Test suite %s PASSED\r\n", suite_name);
    } else if (result.overall_status == TEST_STATUS_NOT_FOUND) {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Test suite %s NOT FOUND\r\n", suite_name);
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Test suite %s FAILED with code %d\r\n", suite_name, result.overall_status);
    }

    return pdFALSE;
}


//*****************************************************************************
BaseType_t cmd_Storage_SDUnmount(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    (void)pcCommandString;
    (void)xWriteBufferLen;

    DataPacket_t packet;
    CreateCommandPacket(&packet, HAL_GetTick(), COMMAND_TARGET_STORAGE, COMMAND_ID_STORAGE_SD_UNMOUNT, NULL, 0);
    sendcmdToTarget(&packet);

    /* Write the response to the buffer */
    snprintf(pcWriteBuffer, 50, "Unmounting SD card...\r\n");

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
        .pcCommand = "reg.list", /* The command string to type. */
        .pcHelpString = "reg.list: Lists all registers\r\n\r\n",
        .pxCommandInterpreter = cmd_regList, /* The function to run. */
        .cExpectedNumberOfParameters = 0 /* No parameters are expected. */
    },
    {
        .pcCommand = "reg.get", /* The command string to type. */
        .pcHelpString = "reg.get <reg_name>: Gets the value of a register\r\n\r\n",
        .pxCommandInterpreter = cmd_regGet, /* The function to run. */
        .cExpectedNumberOfParameters = 1 /* One parameter is expected. */
    },
    {
        .pcCommand = "reg.set", /* The command string to type. */
        .pcHelpString = "reg.set <reg_name> <value>: Sets the value of a register\r\n\r\n",
        .pxCommandInterpreter = cmd_regSet, /* The function to run. */
        .cExpectedNumberOfParameters = 2 /* Two parameters are expected. */
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
        .pcHelpString = "SPARK_ZeroStepper: Finds minimum position of Stepper\r\n\r\n",
        .pxCommandInterpreter = cmd_SPARK_ZeroStepper, /* The function to run. */
        .cExpectedNumberOfParameters = 0
    },
    {
        .pcCommand = "SPARK_FindMax", /* The command string to type. */
        .pcHelpString = "SPARK_FindMax: Finds maximum position of Stepper\r\n\r\n",
        .pxCommandInterpreter = cmd_SPARK_FindMax, /* The function to run. */
        .cExpectedNumberOfParameters = 0
    },
    {
        .pcCommand = "SPARK_TargetPositionMode", /* The command string to type. */
        .pcHelpString = "SPARK_TargetPositionMode <int>: SPARK enters Target Position mode with x/16 torque\r\n\r\n",
        .pxCommandInterpreter = cmd_SPARK_TargetPositionMode, /* The function to run. */
        .cExpectedNumberOfParameters = 1
    },
    {
        .pcCommand = "SPARK_TargetSpeedMode", /* The command string to type. */
        .pcHelpString = "SPARK_TargetSpeedMode <int>: SPARK enters Target Speed mode with x/16 torque\r\n\r\n",
        .pxCommandInterpreter = cmd_SPARK_TargetSpeedMode, /* The function to run. */
        .cExpectedNumberOfParameters = 1
    },
    {
        .pcCommand = "SPARK_Reset", /* The command string to type. */
        .pcHelpString = "SPARK_Reset: Resets SPARK MCU\r\n\r\n",
        .pxCommandInterpreter = cmd_SPARK_Reset, /* The function to run. */
        .cExpectedNumberOfParameters = 0
    },
    {
        .pcCommand = "ACS_SetAngle", /* The command string to type. */
        .pcHelpString = "ACS_SetAngle <float>: Sets ACS Angle\r\n\r\n",
        .pxCommandInterpreter = cmd_ACS_SetAngle, /* The function to run. */
        .cExpectedNumberOfParameters = 1
    },
    {
        .pcCommand = "SPARK_SetNeutralAngle", /* The command string to type. */
        .pcHelpString = "SPARK_SetNeutralAngle <float>: Sets Stepper Neutral Angle\r\n\r\n",
        .pxCommandInterpreter = cmd_SPARK_SetNeutralAngle, /* The function to run. */
        .cExpectedNumberOfParameters = 1
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
        .pcCommand = "Buzzer_PlayNote", /* The command string to type. */
        .pcHelpString = "Buzzer_PlayNote <Note> <duration>: Plays Note from C0 to B8\r\n\r\n",
        .pxCommandInterpreter = cmd_Buzzer_PlayNote, /* The function to run. */
        .cExpectedNumberOfParameters = 2
    },
    {
        .pcCommand = "Buzzer_PlaySong", /* The command string to type. */
        .pcHelpString = "Buzzer_PlaySong <Song>: Plays Song from Playlist\r\n\r\n",
        .pxCommandInterpreter = cmd_Buzzer_PlaySong, /* The function to run. */
        .cExpectedNumberOfParameters = 1
    },
    {
        .pcCommand = "Buzzer_PlaySongRepeat", /* The command string to type. */
        .pcHelpString = "Buzzer_PlaySongRepeat <Song> <Period>: Plays Song from Playlist on repeat each period\r\n\r\n",
        .pxCommandInterpreter = cmd_Buzzer_PlaySongRepeat, /* The function to run. */
        .cExpectedNumberOfParameters = 2
    },
    {
        .pcCommand = "Buzzer_Stop", /* The command string to type. */
        .pcHelpString = "Buzzer_Stop: Stops annoying buzzing activities\r\n\r\n",
        .pxCommandInterpreter = cmd_Buzzer_Stop, /* The function to run. */
        .cExpectedNumberOfParameters = 0
    },
    {
        .pcCommand = "Storage_FlashToSD", /* The command string to type. */
        .pcHelpString = "Storage_FlashToSD <?page>: Transfers data from FLASH to SD card. "
                        "Logs are copied up to the current log page in the config, if no page is explicitly specified.\r\n\r\n",
        .pxCommandInterpreter = cmd_Storage_FlashToSD, /* The function to run. */
        .cExpectedNumberOfParameters = -1
    },
    {
        .pcCommand = "Storage_LogsToSerial", /* The command string to type. */
        .pcHelpString = "Storage_LogsToSerial: Write the log data from FLASH to the serial interface\r\n\r\n",
        .pxCommandInterpreter = cmd_Storage_LogsToSerial, /* The function to run. */
        .cExpectedNumberOfParameters = -1
    },
    {
        .pcCommand = "Storage_FlashErase", /* The command string to type. */
        .pcHelpString = "Storage_FlashErase: Erases the entire FLASH memory\r\n\r\n",
        .pxCommandInterpreter = cmd_Storage_FlashErase, /* The function to run. */
        .cExpectedNumberOfParameters = 0
    },
    {
        .pcCommand = "Storage_FlashWrite", /* The command string to type. */
        .pcHelpString = "Storage_FlashWrite <1/0>: Enables data saving to FLASH memory\r\n\r\n",
        .pxCommandInterpreter = cmd_Storage_FlashWrite, /* The function to run. */
        .cExpectedNumberOfParameters = 1
    },
    {
        .pcCommand = "Storage_SDUnmount", /* The command string to type. */
        .pcHelpString = "Storage_SDUnmount: Unmounts the SD card\r\n\r\n",
        .pxCommandInterpreter = cmd_Storage_SDUnmount, /* The function to run. */
        .cExpectedNumberOfParameters = 0
    },
    {
        .pcCommand = "test.run", /* The command string to type. */
        .pcHelpString = "test.run <TestName>: Runs the specified test and outputs the result\r\n\r\n",
        .pxCommandInterpreter = cmd_TestRun, /* The function to run. */
        .cExpectedNumberOfParameters = 1
    },
    {
        .pcCommand = "test.suite", /* The command string to type. */
        .pcHelpString = "test.suite <SuiteName>: Runs the specified test suite and outputs the results\r\n\r\n",
        .pxCommandInterpreter = cmd_TestSuiteRun, /* The function to run. */
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
void continousSignal(){

        cliWrite("Test");
}

void handleNewline(const char *const pcInputString, char *cOutputBuffer)
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
    char receivedData[50];; // used to store the received value from the notification
    UNUSED(pvParameters);
    vRegisterCLICommands();
    
    for (;;)
    {

       size_t bytesRead = xStreamBufferReceive(xStreamBuffer, receivedData, sizeof(receivedData), portMAX_DELAY);
        if (bytesRead > 0) {
            cliWrite(receivedData);
            receivedData[bytesRead-1] = 0x00; //Strip of \r for analysis
            handleNewline(receivedData, cOutputBuffer);
            xStreamBufferReset(xStreamBuffer);
        }
    }
}
#endif /* CLI_COMMANDS_H */