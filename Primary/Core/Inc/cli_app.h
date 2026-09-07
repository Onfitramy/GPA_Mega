#ifndef CLI_APP_H
#define CLI_APP_H

#include "packets.h"
#include "stdint.h"

typedef enum {
    CLI_TARGET_MODE_INTERNAL = 0,
    CLI_TARGET_MODE_EXTERNAL = 1,
} CLI_TargetMode_t;

typedef enum {
    REG_TYPE_U8,
    REG_TYPE_U16,
    REG_TYPE_U32,
    REG_TYPE_I8,
    REG_TYPE_I16,
    REG_TYPE_I32,
    REG_TYPE_FLOAT,
    REG_TYPE_BOOL,
} reg_type_t;

typedef enum {
    REG_ACCESS_READ  = 1 << 0,
    REG_ACCESS_WRITE = 1 << 1,
} reg_access_t;

typedef struct {
    const char *name;
    const char * const description;
    bool hide_from_list; // If true, this register will not be shown in the list command
    reg_type_t type;
    reg_access_t access;
    void *address;

    float min;
    float max;

    bool (*custom_read)(void *dst);
    bool (*custom_write)(const void *src);
} reg_descriptor_t;

#define IMU1_ENTRY(member_, accessName_, hide_, type_) {.name = "sensor.imu1." #accessName_,.description = "IMU 1 " #member_,.hide_from_list = hide_,.type = type_,.access = REG_ACCESS_READ,.address = &imu1_data.member_,}
#define GPS_ENTRY(member_, hide_, type_) {.name = "sensor.gps." #member_,.description = "GPS " #member_,.hide_from_list = hide_,.type = type_,.access = REG_ACCESS_READ,.address = &gps_data.member_,}

extern CLI_TargetMode_t cli_target_mode;

int sendcmdToTarget(DataPacket_t *packet);
void handleNewline(const char *const pcInputString, char *cOutputBuffer);
void handleCharacterInput(uint8_t *cInputIndex, char *pcInputString);
void vRegisterCLICommands(void);
void vCommandConsoleTask(void *pvParameters);
#endif // CLI_APP_H
