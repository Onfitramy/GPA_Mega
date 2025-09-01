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

void signalPlotter_init(void) {
  #ifdef SIGNAL_PLOTTER_OUT_1 // position ekf testing
  signalPlotter_setSignalName(0, "delta Time");
  signalPlotter_setSignalName(1, "NRF Data");
  signalPlotter_setSignalName(2, "a_x world");
  signalPlotter_setSignalName(3, "a_y world");
  signalPlotter_setSignalName(4, "a_z world");
  signalPlotter_setSignalName(5, "a_x offset");
  signalPlotter_setSignalName(6, "a_y offset");
  signalPlotter_setSignalName(7, "a_z offset");
  signalPlotter_setSignalName(8, "velocity var");
  signalPlotter_setSignalName(9, "horizontal var");
  signalPlotter_setSignalName(10, "pressure");
  signalPlotter_setSignalName(11, "phi");
  signalPlotter_setSignalName(12, "theta");
  signalPlotter_setSignalName(13, "psi");
  signalPlotter_setSignalName(14, "phi fix");
  signalPlotter_setSignalName(15, "theta fix");
  signalPlotter_setSignalName(16, "psi fix");
  signalPlotter_setSignalName(17, "vel x");
  signalPlotter_setSignalName(18, "vel y");
  signalPlotter_setSignalName(19, "vel z");
  signalPlotter_setSignalName(20, "pos x");
  signalPlotter_setSignalName(21, "pos y");
  signalPlotter_setSignalName(22, "pos z");
  signalPlotter_setSignalName(23, "vel x fix");
  signalPlotter_setSignalName(24, "vel y fix");
  signalPlotter_setSignalName(25, "vel z fix");
  signalPlotter_setSignalName(26, "pos x fix");
  signalPlotter_setSignalName(27, "pos y fix");
  signalPlotter_setSignalName(28, "pos z fix");
  #endif

  #ifdef SIGNAL_PLOTTER_OUT_2 // raw sensor data
  signalPlotter_setSignalName(0, "delta Time");
  signalPlotter_setSignalName(1, "NRF Data");
  signalPlotter_setSignalName(2, "MAG_X");
  signalPlotter_setSignalName(3, "MAG_Y");
  signalPlotter_setSignalName(4, "MAG_Z");
  signalPlotter_setSignalName(5, "ACC_X");
  signalPlotter_setSignalName(6, "ACC_Y");
  signalPlotter_setSignalName(7, "ACC_Z");
  signalPlotter_setSignalName(8, "GYR_X");
  signalPlotter_setSignalName(9, "GYR_Y");
  signalPlotter_setSignalName(10, "GYR_Z");
  signalPlotter_setSignalName(11, "pressure");
  signalPlotter_setSignalName(12, "temperature");
  signalPlotter_setSignalName(13, "GPS_fix");
  signalPlotter_setSignalName(14, "numSV");
  signalPlotter_setSignalName(15, "hAcc");
  signalPlotter_setSignalName(16, "vAcc");
  signalPlotter_setSignalName(17, "sAcc");
  signalPlotter_setSignalName(18, "lat");
  signalPlotter_setSignalName(19, "lon");
  signalPlotter_setSignalName(20, "height");
  signalPlotter_setSignalName(21, "velN");
  signalPlotter_setSignalName(22, "velE");
  signalPlotter_setSignalName(23, "velD");
  #endif

  #ifdef SIGNAL_PLOTTER_OUT_3 // orientation ekf testing
  signalPlotter_setSignalName(0, "delta Time");
  signalPlotter_setSignalName(1, "NRF Data");
  signalPlotter_setSignalName(2, "quat phi");
  signalPlotter_setSignalName(3, "quat theta");
  signalPlotter_setSignalName(4, "quat psi");
  signalPlotter_setSignalName(5, "gx exp");
  signalPlotter_setSignalName(6, "gy exp");
  signalPlotter_setSignalName(7, "gz exp");
  signalPlotter_setSignalName(8, "mx exp");
  signalPlotter_setSignalName(9, "my exp");
  signalPlotter_setSignalName(10, "mz exp");
  signalPlotter_setSignalName(11, "ax meas");
  signalPlotter_setSignalName(12, "ay meas");
  signalPlotter_setSignalName(13, "az meas");
  signalPlotter_setSignalName(14, "mx meas");
  signalPlotter_setSignalName(15, "my meas");
  signalPlotter_setSignalName(16, "mz meas");
  signalPlotter_setSignalName(17, "euler phi");
  signalPlotter_setSignalName(18, "euler theta");
  signalPlotter_setSignalName(19, "euler psi");
  signalPlotter_setSignalName(20, "gyro x");
  signalPlotter_setSignalName(21, "gyro y");
  signalPlotter_setSignalName(22, "gyro z");
  signalPlotter_setSignalName(23, "gyro x bias");
  signalPlotter_setSignalName(24, "gyro y bias");
  signalPlotter_setSignalName(25, "gyro z bias");
  #endif
}