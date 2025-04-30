#ifndef BMP390_H_
#define BMP390_H_

#include "stm32h7xx_hal.h"
#include "stdbool.h"

#include "main.h"  // Falls du Debugging per LED machst

// I²C-Handle extern deklarieren
extern I2C_HandleTypeDef hi2c3;  // Falls du I²C1 benutzt, sonst anpassen

#define BMP_I2C        hi2c3      // Das richtige I²C-Interface setzen
#define BMP390_I2C_ADDR  (0x76 << 1)  // BMP390 I²C-Adresse (STM32 nutzt 8-Bit-Adresse)

// Register-Adressen
#define BMP390_CHIP_ID_REG  0x00  // WHO_AM_I Register (sollte 0x60 zurückgeben)

// Funktionsdeklarationen
uint8_t BMP390_SelfTest(void);
void BMP_write_reg(uint8_t reg, uint8_t data);
void BMP_read_reg(uint8_t reg, uint8_t *data);
uint8_t BMP_enable(void);
uint8_t BMP_GetPressureRaw(uint32_t *pressure);


#endif /* BMP390_H_ */