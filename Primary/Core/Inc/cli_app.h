#ifndef CLI_APP_H
#define CLI_APP_H

#include "packets.h"
#include "stdint.h"

typedef enum {
    CLI_TARGET_MODE_INTERNAL = 0,
    CLI_TARGET_MODE_EXTERNAL = 1,
} CLI_TargetMode_t;

extern CLI_TargetMode_t cli_target_mode;

int sendcmdToTarget(DataPacket_t *packet);
void handleNewline(const char *const pcInputString, char *cOutputBuffer, uint8_t cInputIndex);
void handleCharacterInput(uint8_t *cInputIndex, char *pcInputString);
void vRegisterCLICommands(void);
void vCommandConsoleTask(void *pvParameters);
#endif // CLI_APP_H
