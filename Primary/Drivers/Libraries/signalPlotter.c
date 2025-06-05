#include "signalPlotter.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdint.h>
#include <string.h>

typedef struct {
  float value;
  uint8_t sendFlag;
  char name[17];
} data_t;

data_t data[32];

uint32_t TimeMeasureTime;

uint32_t TimeMeasureTick = 0;

uint32_t TimerOverflow = 0;

void signalPlotter_setSignalName(uint8_t id, char *name) {
  strncpy(data[id].name, name, 16);
}

void signalPlotter_sendData(uint8_t id, float value) {
  data[id].value = value;
  data[id].sendFlag = 1;
}

// FHAC, 32-bitfield, n x float, 3 x \x00, crc32

void signalPlotter_executeTransmission(uint32_t millisTime) {

  if (1) {
    static uint8_t nameSendPosition = 0;

    uint8_t buffer[150] = {0};

    uint32_t bitfieldSendFlags = 0;

    for (int id = 0; id < 32; id++) {
      if (data[id].sendFlag) {
        bitfieldSendFlags += 1 << id;
      }
    }

    if (bitfieldSendFlags) {

      char header[] = "FHAC";
      
      memcpy(&buffer[0], header, 4);

      memcpy(&buffer[4], &millisTime, 4);

      memcpy(&buffer[8], &bitfieldSendFlags, 4);

      int maxid = 0;
      for (int id = 0; id < 32; id++) {
        if (data[id].sendFlag) {
          data[id].sendFlag = 0;

          memcpy(&buffer[12+id*4], &data[id].value, 4);
          maxid++;

        }
      }

      uint8_t nameSendBuffer[3];
      nameSendBuffer[0] = nameSendPosition;
      nameSendBuffer[1] = data[nameSendPosition / 8].name[2 * (nameSendPosition % 8)];
      nameSendBuffer[2] = data[nameSendPosition / 8].name[2 * (nameSendPosition % 8) + 1];
      nameSendPosition++;

      memcpy(&buffer[12+maxid*4], nameSendBuffer, 3);
      CDC_Transmit_HS(buffer, 12+maxid*4+3);
    }
  }
}

void TimeMeasureStart(void) {
  TimeMeasureTick = HAL_GetTickUS();
}

void TimeMeasureStop(void) {
  TimeMeasureTime = HAL_GetTickUS() - TimeMeasureTick;
  if (TimeMeasureTime > 1000) {
    TimerOverflow += 1;
  }
  signalPlotter_sendData(0, (float)TimeMeasureTime / 1000.0f);
}