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

uint32_t TimeMeasureTick = 0;

uint32_t TimerOverflow = 0;

uint32_t dt_1000Hz;

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

uint32_t TimeMeasureStop(void) {
  uint32_t TimeMeasureTime;
  TimeMeasureTime = HAL_GetTickUS() - TimeMeasureTick;
  if (TimeMeasureTime > 1000) {
    TimerOverflow += 1;
  }
  return TimeMeasureTime;
}

void signalPlotter_init(void) {
  #ifdef SIGNAL_PLOTTER_OUT_1 // imu testing
  signalPlotter_setSignalName(0, "delta Time");
  signalPlotter_setSignalName(1, "NRF Data");
  signalPlotter_setSignalName(2, "IMU1_ACC_X");
  signalPlotter_setSignalName(3, "IMU1_ACC_Y");
  signalPlotter_setSignalName(4, "IMU1_ACC_Z");
  signalPlotter_setSignalName(5, "IMU2_ACC_X");
  signalPlotter_setSignalName(6, "IMU2_ACC_Y");
  signalPlotter_setSignalName(7, "IMU2_ACC_Z");
  signalPlotter_setSignalName(8, "AVG_ACC_X");
  signalPlotter_setSignalName(9, "AVG_ACC_Y");
  signalPlotter_setSignalName(10, "AVG_ACC_Z");
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
  signalPlotter_setSignalName(2, "phi");
  signalPlotter_setSignalName(3, "theta");
  signalPlotter_setSignalName(4, "psi");
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
  signalPlotter_setSignalName(17, "gyro x");
  signalPlotter_setSignalName(18, "gyro y");
  signalPlotter_setSignalName(19, "gyro z");
  signalPlotter_setSignalName(20, "gyro x bias");
  signalPlotter_setSignalName(21, "gyro y bias");
  signalPlotter_setSignalName(22, "gyro z bias");
  signalPlotter_setSignalName(23, "FlightState");
  signalPlotter_setSignalName(24, "Entry_Timestamp");
  #endif

  #ifdef SIGNAL_PLOTTER_OUT_4 // raw sensor data
  signalPlotter_setSignalName(0, "delta_Time");
  signalPlotter_setSignalName(1, "NRF_Data");
  signalPlotter_setSignalName(2, "phi");
  signalPlotter_setSignalName(3, "theta");
  signalPlotter_setSignalName(4, "psi");
  signalPlotter_setSignalName(5, "ACC_X");
  signalPlotter_setSignalName(6, "ACC_Y");
  signalPlotter_setSignalName(7, "ACC_Z");
  signalPlotter_setSignalName(8, "AWORLD_X");
  signalPlotter_setSignalName(9, "AWORLD_Y");
  signalPlotter_setSignalName(10, "AWORLD_Z");
  signalPlotter_setSignalName(11, "pressure");
  signalPlotter_setSignalName(12, "temperature");
  signalPlotter_setSignalName(13, "height_baro");
  signalPlotter_setSignalName(14, "GPS_fix");
  signalPlotter_setSignalName(15, "gnss_numSV");
  signalPlotter_setSignalName(16, "gnss_hAcc");
  signalPlotter_setSignalName(17, "gnss_vAcc");
  signalPlotter_setSignalName(18, "gnss_sAcc");
  signalPlotter_setSignalName(19, "gnss_lat");
  signalPlotter_setSignalName(20, "gnss_lon");
  signalPlotter_setSignalName(21, "gnss_height");
  signalPlotter_setSignalName(22, "gnss_velN");
  signalPlotter_setSignalName(23, "gnss_velE");
  signalPlotter_setSignalName(24, "gnss_velD");
  signalPlotter_setSignalName(25, "EKF_height");
  signalPlotter_setSignalName(26, "EKF_vel_Z");
  signalPlotter_setSignalName(27, "EKF_p_ref");
  signalPlotter_setSignalName(28, "height_comp");
  signalPlotter_setSignalName(29, "velZ_comp");
  signalPlotter_setSignalName(30, "FlightState");
  signalPlotter_setSignalName(31, "Entry_Timestamp");
  #endif

  #ifdef SIGNAL_PLOTTER_OUT_5 // variable testing data
  signalPlotter_setSignalName(0, "delta_Time");
  signalPlotter_setSignalName(1, "NRF_Data");
  signalPlotter_setSignalName(2, "FlightState");
  signalPlotter_setSignalName(3, "Entry_Timestamp");
  #endif

  #ifdef SIGNAL_PLOTTER_OUT_GROUND // ground station data
  signalPlotter_setSignalName(0, "delta_Time");
  signalPlotter_setSignalName(1, "phi");
  signalPlotter_setSignalName(2, "theta");
  signalPlotter_setSignalName(3, "psi");
  signalPlotter_setSignalName(4, "x_Accel");
  signalPlotter_setSignalName(5, "y_Accel");
  signalPlotter_setSignalName(6, "z_Accel");
  #endif
}

void signalPlotter_sendAll(void) {
  #ifdef SIGNAL_PLOTTER_OUT_1
  signalPlotter_sendData(0, (float)dt_1000Hz / 1000.0f);
  signalPlotter_sendData(1, (float)nrf_timeout);
  signalPlotter_sendData(2, imu1_data.accel[0]);
  signalPlotter_sendData(3, imu1_data.accel[1]);
  signalPlotter_sendData(4, imu1_data.accel[2]);
  signalPlotter_sendData(5, imu2_data.accel[0]);
  signalPlotter_sendData(6, imu2_data.accel[1]);
  signalPlotter_sendData(7, imu2_data.accel[2]);
  signalPlotter_sendData(8, average_imu_data.accel[0]);
  signalPlotter_sendData(9, average_imu_data.accel[1]);
  signalPlotter_sendData(10, average_imu_data.accel[2]);
  #endif


  #ifdef SIGNAL_PLOTTER_OUT_2 // signal plotter outputs raw sensor data
  signalPlotter_sendData(0, (float)dt_1000Hz / 1000.0f);
  signalPlotter_sendData(1, (float)nrf_timeout);
  signalPlotter_sendData(2, mag_data.field[0]);
  signalPlotter_sendData(3, mag_data.field[1]);
  signalPlotter_sendData(4, mag_data.field[2]);
  signalPlotter_sendData(5, imu2_data.accel[0]);
  signalPlotter_sendData(6, imu2_data.accel[1]);
  signalPlotter_sendData(7, imu2_data.accel[2]);
  signalPlotter_sendData(8, imu2_data.gyro[0]);
  signalPlotter_sendData(9, imu2_data.gyro[1]);
  signalPlotter_sendData(10, imu2_data.gyro[2]);
  signalPlotter_sendData(11, pressure);
  signalPlotter_sendData(12, temperature);
  signalPlotter_sendData(13, (float)gps_data.gpsFix);
  signalPlotter_sendData(14, (float)gps_data.numSV);
  signalPlotter_sendData(15, (float)gps_data.hAcc/1000.f);
  signalPlotter_sendData(16, (float)gps_data.vAcc/1000.f);
  signalPlotter_sendData(17, (float)gps_data.sAcc/1000.f);
  signalPlotter_sendData(18, (float)gps_data.lat*1e-7);
  signalPlotter_sendData(19, (float)gps_data.lon*1e-7);
  signalPlotter_sendData(20, (float)gps_data.height/1000.f);
  signalPlotter_sendData(21, (float)gps_data.velN/1000.f);
  signalPlotter_sendData(22, (float)gps_data.velE/1000.f);
  signalPlotter_sendData(23, (float)gps_data.velD/1000.f);
  #endif

  #ifdef SIGNAL_PLOTTER_OUT_3 // signal plotter outputs quaternion ekf testing
  signalPlotter_sendData(0, (float)dt_1000Hz / 1000.0f);
  signalPlotter_sendData(1, (float)nrf_timeout);
  signalPlotter_sendData(2, euler_from_q[0]);
  signalPlotter_sendData(3, euler_from_q[1]);
  signalPlotter_sendData(4, euler_from_q[2]);
  signalPlotter_sendData(5, h3_corr1[0]);
  signalPlotter_sendData(6, h3_corr1[1]);
  signalPlotter_sendData(7, h3_corr1[2]);
  signalPlotter_sendData(8, h3_corr1[3]);
  signalPlotter_sendData(9, h3_corr1[4]);
  signalPlotter_sendData(10, h3_corr1[5]);
  signalPlotter_sendData(11, average_imu_data.accel[0]);
  signalPlotter_sendData(12, average_imu_data.accel[1]);
  signalPlotter_sendData(13, average_imu_data.accel[2]);
  signalPlotter_sendData(14, mag_data.field[0]);
  signalPlotter_sendData(15, mag_data.field[1]);
  signalPlotter_sendData(16, mag_data.field[2]);
  signalPlotter_sendData(17, average_imu_data.gyro[0]);
  signalPlotter_sendData(18, average_imu_data.gyro[1]);
  signalPlotter_sendData(19, average_imu_data.gyro[2]);
  signalPlotter_sendData(20, x3[4]);
  signalPlotter_sendData(21, x3[5]);
  signalPlotter_sendData(22, x3[6]);
  signalPlotter_sendData(23, (float)flight_sm.currentFlightState);
  signalPlotter_sendData(24, (float)flight_sm.timestamp_us);
  #endif

  #ifdef SIGNAL_PLOTTER_OUT_4 // signal plotter outputs height ekf testing
  signalPlotter_sendData(0, (float)dt_1000Hz / 1000.0f);
  signalPlotter_sendData(1, (float)nrf_timeout);
  signalPlotter_sendData(2, euler_from_q[0]);
  signalPlotter_sendData(3, euler_from_q[1]);
  signalPlotter_sendData(4, euler_from_q[2]);
  signalPlotter_sendData(5, average_imu_data.accel[0]);
  signalPlotter_sendData(6, average_imu_data.accel[1]);
  signalPlotter_sendData(7, average_imu_data.accel[2]);
  signalPlotter_sendData(8, a_WorldFrame[0]);
  signalPlotter_sendData(9, a_WorldFrame[1]);
  signalPlotter_sendData(10, a_WorldFrame[2]);
  signalPlotter_sendData(11, pressure);
  signalPlotter_sendData(12, temperature);
  signalPlotter_sendData(13, height_baro);
  signalPlotter_sendData(14, (float)gps_data.gpsFix);
  signalPlotter_sendData(15, (float)gps_data.numSV);
  signalPlotter_sendData(16, (float)gps_data.hAcc/1000.f);
  signalPlotter_sendData(17, (float)gps_data.vAcc/1000.f);
  signalPlotter_sendData(18, (float)gps_data.sAcc/1000.f);
  signalPlotter_sendData(19, (float)gps_data.lat*1e-7);
  signalPlotter_sendData(20, (float)gps_data.lon*1e-7);
  signalPlotter_sendData(21, (float)gps_data.height/1000.f);
  signalPlotter_sendData(22, (float)gps_data.velN/1000.f);
  signalPlotter_sendData(23, (float)gps_data.velE/1000.f);
  signalPlotter_sendData(24, (float)-gps_data.velD/1000.f);
  signalPlotter_sendData(25, EKF2.x[0]);
  signalPlotter_sendData(26, EKF2.x[1]);
  signalPlotter_sendData(27, EKF2.x[2]);
  signalPlotter_sendData(28, gnss_height_corr);
  signalPlotter_sendData(29, gnss_velZ_corr);
  signalPlotter_sendData(30, (float)flight_sm.currentFlightState);
  signalPlotter_sendData(31, (float)flight_sm.timestamp_us);
  #endif

  #ifdef SIGNAL_PLOTTER_OUT_5 // signal plotter outputs testing data
  signalPlotter_sendData(0, (float)dt_1000Hz / 1000.0f);
  signalPlotter_sendData(1, (float)nrf_timeout);
  signalPlotter_sendData(2, (float)flight_sm.currentFlightState);
  signalPlotter_sendData(3, (float)flight_sm.timestamp_us);
  #endif
}