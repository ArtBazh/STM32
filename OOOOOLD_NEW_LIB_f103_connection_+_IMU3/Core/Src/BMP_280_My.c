/*
 * BMP_280_My.c
 *  Created on: Mar 26, 2023
 */
#include "BMP_280_My.h"
//------------------------------------------------
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;
extern int32_t t_fine = 0;
extern float p_reference = 0;
BMP280_CalibData compensationParameters;
//------------------------------------------------

/*
Пример как юзать
float pressure = 0;
uint8_t press[30];

pressure = get_diff_preasure();
sprintf(press, "%0.7f;\n", pressure);
HAL_UART_Transmit_IT(&huart1, (uint8_t*)press, strlen(press));
 */

void BMP280_Init(void)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
	HAL_Delay(10);
	BMP280_reset();
	while (BMP280_readStatus() & BMP280_STATUS_IM_UPDATE)
	;
	setPressureOversampling();
	setTemperatureOversampling();
	setPowerMode();
	setFilterCoefficient();
	setStandbyTime();
	readCompensationParameters();
	setReferencePressure();
}


// Считать 1 байт из BMP280 Через SPI
uint8_t read_BMP_Register(uint8_t address)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET); // Кидаем ноль на пин чип-селект чтоб обратиться к BMP280, а не к MPU9250
	spi_BMP_ReadWrite(address);
	uint8_t value = spi_BMP_ReadWrite(0);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); // Кидаем VDD на пин чип-селект чтоб остановить общение с BMP280
	return value;
}

// Записать 1 байт в BMP280 Через SPI. Принимает адрес и значение для записи, смотреть что етсь что по даташиту
uint8_t write_BMP_Register(uint8_t address, uint8_t value)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET); // Кидаем ноль на пин чип-селект чтоб обратиться к BMP280, а не к MPU9250
	spi_BMP_ReadWrite(address & BMP280_SPI_MASK_WRITE);
	spi_BMP_ReadWrite(value);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); // Кидаем VDD на пин чип-селект чтоб остановить общение с BMP280
}

// Вызов SPI изнутри двух предидущих функций
uint8_t spi_BMP_ReadWrite(uint8_t tx_message)
{
	uint8_t rx_message = 255;
	HAL_SPI_TransmitReceive(&hspi1, &tx_message, &rx_message, 1, HAL_MAX_DELAY);
	return rx_message;
}


// Чтение мульти байтовых регистров. Принимает адрес регистра, указатель на массив для записи и кол-во байт для чтения
void readMBRegister(uint8_t address, uint8_t *values, uint8_t length)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET); // Кидаем ноль на пин чип-селект чтоб обратиться к BMP280, а не к MPU9250
	spi_BMP_ReadWrite(address);
	while (length--)
	{
		*values++ = spi_BMP_ReadWrite(0);
	}
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); // Кидаем VDD на пин чип-селект чтоб остановить общение с BMP280
}


void BMP280_reset(void)
{
	write_BMP_Register(BMP280_REG_RESET, BMP280_RESET_VALUE);
}

uint8_t BMP280_readStatus(void)
{
	uint8_t result = read_BMP_Register(BMP280_REGISTER_STATUS)&0x09;
	return result;
}

void readCompensationParameters(void)
{
	uint8_t buf[24] = {6};
	readMBRegister(BMP280_REG_CALIB, buf, 24);
	compensationParameters.dig_t1 = ((buf[1] << 8) | buf[0]);
	compensationParameters.dig_t2 = ((buf[3] << 8) | buf[2]);
	compensationParameters.dig_t3 = ((buf[5] << 8) | buf[4]);
	compensationParameters.dig_p1 = ((buf[7] << 8) | buf[6]);
	compensationParameters.dig_p2 = ((buf[9] << 8) | buf[8]);
	compensationParameters.dig_p3 = ((buf[11] << 8) | buf[10]);
	compensationParameters.dig_p4 = ((buf[13] << 8) | buf[12]);
	compensationParameters.dig_p5 = ((buf[15] << 8) | buf[14]);
	compensationParameters.dig_p6 = ((buf[17] << 8) | buf[16]);
	compensationParameters.dig_p7 = ((buf[19] << 8) | buf[18]);
	compensationParameters.dig_p8 = ((buf[21] << 8) | buf[20]);
	compensationParameters.dig_p9 = ((buf[23] << 8) | buf[22]);
}

void setPressureOversampling(void)
{
	uint8_t ctrl = read_BMP_Register(BMP280_REG_CTRL_MEAS);
	ctrl = (ctrl & 0b11100011) | (0b101 << 2);
	write_BMP_Register(BMP280_REG_CTRL, ctrl);
}

void setTemperatureOversampling(void)
{
	uint8_t ctrl = read_BMP_Register(BMP280_REG_CTRL_MEAS);
	ctrl = (ctrl & 0b00011111) | (0b101 << 5);
	write_BMP_Register(BMP280_REG_CTRL, ctrl);
}

void setPowerMode(void)
{
	uint8_t ctrl = read_BMP_Register(BMP280_REG_CTRL_MEAS);
	ctrl = (ctrl & 0b11111100) | 0b11;
	write_BMP_Register(BMP280_REG_CTRL, ctrl);
}

void setFilterCoefficient(void)
{
	uint8_t conf = read_BMP_Register(BMP280_REG_CONFIG);
	conf = (conf & 0b11100011) | (0b100 << 2);
	write_BMP_Register(BMP280_REG_CONFIG, conf);
}

void setStandbyTime(void)
{
	uint8_t conf = read_BMP_Register(BMP280_REG_CONFIG);
	conf = (conf & 0b00011111) | (0b000 << 5);
	write_BMP_Register(BMP280_REG_CONFIG, conf);
}


float measure(void)
{
	uint8_t data[6];
	float temperature = 0;
	float pressure = 0;
	readMBRegister(BMP280_REG_DATA, data, 6);

	int32_t adc_P = data[0] << 12 | data[1] << 4 | data[2] >> 4;
	int32_t adc_T = data[3] << 12 | data[4] << 4 | data[5] >> 4;

	temperature = (float) compensate_temperature(adc_T) / 100.0;
	pressure = (float) compensate_pressure(adc_P) / 256.0;

	return pressure;
}

float get_diff_preasure(void)
{
	float diff_measure = measure()- p_reference;
	return diff_measure;
}

int32_t compensate_temperature(int32_t uncomp_temp)
{
	int32_t var1, var2;
	var1 =
			((((uncomp_temp / 8)
					- ((int32_t) compensationParameters.dig_t1 << 1)))
					* ((int32_t) compensationParameters.dig_t2)) / 2048;
	var2 = (((((uncomp_temp / 16) - ((int32_t) compensationParameters.dig_t1))
			* ((uncomp_temp / 16) - ((int32_t) compensationParameters.dig_t1)))
			/ 4096) * ((int32_t) compensationParameters.dig_t3)) / 16384;
	t_fine = var1 + var2;
	return (t_fine * 5 + 128) / 256;
}

uint32_t compensate_pressure(int32_t uncomp_pres)
{
	int64_t var1, var2, p;

	var1 = ((int64_t) (t_fine)) - 128000;
	var2 = var1 * var1 * (int64_t) compensationParameters.dig_p6;
	var2 = var2 + ((var1 * (int64_t) compensationParameters.dig_p5) * 131072);
	var2 = var2 + (((int64_t) compensationParameters.dig_p4) * 34359738368);
	var1 = ((var1 * var1 * (int64_t) compensationParameters.dig_p3) / 256)
			+ ((var1 * (int64_t) compensationParameters.dig_p2) * 4096);
	var1 = ((INT64_C(0x800000000000) + var1)
			* ((int64_t) compensationParameters.dig_p1)) / 8589934592;
	if (var1 == 0)
	{
		return 0;
	}
	p = 1048576 - uncomp_pres;
	p = (((((p * 2147483648U)) - var2) * 3125) / var1);
	var1 = (((int64_t) compensationParameters.dig_p9) * (p / 8192) * (p / 8192))
			/ 33554432;
	var2 = (((int64_t) compensationParameters.dig_p8) * p) / 524288;
	p = ((p + var1 + var2) / 256)
			+ (((int64_t) compensationParameters.dig_p7) * 16);
	return (uint32_t) p;
}

float setReferencePressure(void)
{
	uint16_t samples = 20;
	HAL_Delay(100);
	float sum = 0;
	for (char i = 0; i < samples; i++)
	{
		sum += measure();
		HAL_Delay(100);
	}
	p_reference = sum / samples;
}
