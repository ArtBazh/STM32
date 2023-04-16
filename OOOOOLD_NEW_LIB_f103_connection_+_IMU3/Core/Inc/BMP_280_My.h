/*
 * BMP_280_My.h
 *  Created on: Mar 26, 2023
 */

#ifndef INC_BMP_280_MY_H_
#define INC_BMP_280_MY_H_

#include "stm32f1xx_hal.h"
#include <string.h>
#include <math.h>

#endif /* INC_BMP_280_MY_H_ */

#define BMP280_SPI_MASK_WRITE   0b01111111
#define BMP280_REG_RESET        0xE0
#define BMP280_RESET_VALUE      0xB6
#define BMP280_REGISTER_STATUS  0XF3
#define BMP280_STATUS_MEASURING 0X08
#define BMP280_STATUS_IM_UPDATE 0X01
#define BMP280_REG_CALIB        0x88
#define BMP280_REG_CONFIG       0xF5
#define BMP280_REG_CTRL_MEAS    0xF4
#define BMP280_REG_CTRL         0xF4
#define BMP280_REG_DATA         0xF7




void BMP280_Init(void);
uint8_t read_BMP_Register(uint8_t);
uint8_t write_BMP_Register(uint8_t, uint8_t);
uint8_t spi_BMP_ReadWrite(uint8_t);
void BMP280_reset(void);
uint8_t BMP280_readStatus(void);
void readCompensationParameters(void);
void readMBRegister(uint8_t address, uint8_t *values, uint8_t length);
void setPressureOversampling(void);
void setTemperatureOversampling(void);
void setPowerMode(void);
void setFilterCoefficient(void);
void setStandbyTime(void);
float measure(void);
int32_t compensate_temperature(int32_t uncomp_temp);
uint32_t compensate_pressure(int32_t uncomp_pres);
float setReferencePressure(void);
float get_diff_preasure(void);

//------------------------------------------------
typedef struct
{
  uint16_t dig_t1;
  int16_t dig_t2;
  int16_t dig_t3;
  uint16_t dig_p1;
  int16_t dig_p2;
  int16_t dig_p3;
  int16_t dig_p4;
  int16_t dig_p5;
  int16_t dig_p6;
  int16_t dig_p7;
  int16_t dig_p8;
  int16_t dig_p9;
} BMP280_CalibData;
//------------------------------------------------


