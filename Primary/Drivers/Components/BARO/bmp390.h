#ifndef BMP390_H_
#define BMP390_H_

#include "stdbool.h"

#include "main.h"  // Falls du Debugging per LED machst

typedef struct {
    // Original
    uint16_t t1;
    int16_t t2;
    int8_t t3;

    int16_t p1;
    int16_t p2;
    int8_t p3;
    int8_t p4;
    uint16_t p5;
    uint16_t p6;
    int8_t p7;
    int8_t p8;
    int16_t p9;
    int8_t p10;
    int8_t p11;

    // Umgerechnet
    float par_t1, par_t2, par_t3;
    float par_p1, par_p2, par_p3, par_p4, par_p5, par_p6, par_p7, par_p8, par_p9, par_p10, par_p11;

    float t_lin;
} bmp390_handle_t;

typedef struct {
    float pressure;
    float temperature;
    float height;
} bmp390_data_t;

// I²C-Handle extern deklarieren
extern I2C_HandleTypeDef hi2c3;  // Falls du I²C1 benutzt, sonst anpassen
#define BMP_I2C        hi2c3      // Das richtige I²C-Interface setzen
#define BMP390_I2C_ADDR  (0x76 << 1)  // BMP390 I²C-Adresse (STM32 nutzt 8-Bit-Adresse)

extern bmp390_handle_t bmp_handle;
extern bmp390_data_t bmp_data;

// Register-Adressen
#define BMP390_CHIP_ID_REG  0x00  // WHO_AM_I Register (sollte 0x60 zurückgeben)
#define BMP_390_OSR_REG 0x1C //Oversampling Rate
#define BMP_390_ODR_REG 0x1D //Output Data Rate
#define BMP_390_IIR_REG 0x1F //IIR Filter
#define BMP_390_PWR_CTRL_REG 0x1B //Power Control


#define PRESS_MSB_23_16 0x06
#define PRESS_LSB_15_8 0x05
#define PRESS_XLSB_7_0 0x04
#define TEMP_MSB_23_16 0x09
#define TEMP_LSVB_15_8 0x08
#define TEMP_XLSB_7_0 0x07

#define BMP390_REG_NVM_PAR_T1_L        0x31        /**< NVM PAR T1 low register */
#define BMP390_REG_NVM_PAR_T1_H        0x32        /**< NVM PAR T1 high register */
#define BMP390_REG_NVM_PAR_T2_L        0x33        /**< NVM PAR T2 low register */
#define BMP390_REG_NVM_PAR_T2_H        0x34        /**< NVM PAR T2 high register */
#define BMP390_REG_NVM_PAR_T3          0x35        /**< NVM PAR T3 register */
#define BMP390_REG_NVM_PAR_P1_L        0x36        /**< NVM PAR P1 low register */
#define BMP390_REG_NVM_PAR_P1_H        0x37        /**< NVM PAR P1 high register */
#define BMP390_REG_NVM_PAR_P2_L        0x38        /**< NVM PAR P2 low register */
#define BMP390_REG_NVM_PAR_P2_H        0x39        /**< NVM PAR P2 high register */
#define BMP390_REG_NVM_PAR_P3          0x3A        /**< NVM PAR P3 register */
#define BMP390_REG_NVM_PAR_P4          0x3B        /**< NVM PAR P4 register */
#define BMP390_REG_NVM_PAR_P5_L        0x3C        /**< NVM PAR P5 low register */
#define BMP390_REG_NVM_PAR_P5_H        0x3D        /**< NVM PAR P5 high register */
#define BMP390_REG_NVM_PAR_P6_L        0x3E        /**< NVM PAR P6 low register */
#define BMP390_REG_NVM_PAR_P6_H        0x3F        /**< NVM PAR P6 high register */
#define BMP390_REG_NVM_PAR_P7          0x40        /**< NVM PAR P7 register */
#define BMP390_REG_NVM_PAR_P8          0x41        /**< NVM PAR P8 register */
#define BMP390_REG_NVM_PAR_P9_L        0x42        /**< NVM PAR P9 low register */
#define BMP390_REG_NVM_PAR_P9_H        0x43        /**< NVM PAR P9 high register */
#define BMP390_REG_NVM_PAR_P10         0x44        /**< NVM PAR P10 register */
#define BMP390_REG_NVM_PAR_P11         0x45        /**< NVM PAR P11 register */


// Funktionsdeklarationen
uint8_t BMP_SelfTest(void);
uint8_t BMP_enable(void);
void BMP_Read_Calibration_Params(bmp390_handle_t *handle);
uint8_t BMP_GetRawData(uint32_t *pressure_raw, uint32_t *temperature_raw);
uint8_t BMP_readData(float *pressure, float *height, float *temperature);
float bmp390_compensate_pressure(uint32_t uncomp_press, bmp390_handle_t *handle);
float bmp390_compensate_temperature(uint32_t uncomp_temp, bmp390_handle_t *handle);



#endif /* BMP390_H_ */