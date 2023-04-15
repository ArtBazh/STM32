/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU9250.h"
#include "vector.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
  extern uint8_t flag_tx;
  extern uint8_t flag_CALLBACK;
  uint8_t TX_data[100] = {};

  extern uint8_t cnt;
    Vector3f MagShift,AccShift,GyroShift;
    Matrix3f MagCalibr, AccCalibr;
    Vector3f Mag, Acc, Mag0, Gyr;
    Vector3f MagR, AccR;
    Vector3f Mag, Acc;
    int16_t AccData[3]={1,1,1};
    int16_t MagData[3]={1,1,1};
    int16_t GyroData[3]={1,1,1};



    Vector3f vX, vY, vZ;
    Matrix3f M0, M1, M2, M3;
    Vector3f Euler;
    Vector3f EulerF;
    Vector3f EulerF2;

    int optimer=0;
    uint16_t Code;

    uint8_t bufArd[30];
    char str0[100];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void UART_to_Computer (uint8_t data[])
{
	//uint16_t size = sizeof(data)/sizeof(uint8_t);
	uint16_t size = strlen(data);

	uint16_t ind = 0;
	while (ind<=size)
	  {
		while (!LL_USART_IsActiveFlag_TXE(USART1))
		;
		LL_USART_TransmitData8(USART1, (data[ind]));
		ind++;
	  }
}

void UART_TX_to_computer_DMA(unsigned char data[])
	{
	  //uint16_t size = sizeof(data)+1;
	  uint16_t size = strlen(data);
	  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
	  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, size);
	  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
	  while (!flag_tx) {;}
	  flag_tx=0;
	}

void DMA1_TransmitComplete(void)
{
  flag_tx = 1;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4); // Отключаем каналы DMA перед настройкой
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);
  LL_DMA_ClearFlag_TC4(DMA1); // Явно сбрасываем все флаги
  LL_DMA_ClearFlag_TE4(DMA1);
  LL_USART_EnableDMAReq_TX(USART1); // Разрешаем DMA работать с USART1
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4); // Разрешаем прерывания DMA на завершение передачи
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_4); // Разрешаем прерывания DMA на ошибку
  LL_DMA_ClearFlag_GI4(DMA1); // Сбрасываем флаги которые могли подняться после предидущих двух строчек
  LL_DMA_ClearFlag_TC4(DMA1);
  LL_DMA_ClearFlag_TE4(DMA1);
  // Дальше даём DMA указатель на буффер из которого пересылать данные
  LL_DMA_ConfigAddresses(DMA1,
		  	  	  	  	  LL_DMA_CHANNEL_4,
						  &TX_data,
						  LL_USART_DMA_GetRegAddr(USART1),
						  LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4));
  	int st;
	st = MPU_begin();
	HAL_Delay(100);
	enableDataReadyInterrupt();
	HAL_Delay(100);
	setSrd(0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	 // LL_mDelay(1);
	 // readSensor3(AccData, MagData, GyroData);
	 // sprintf(str1,"%08d;%08d;%08d;%08d;%08d;%08d;%08d;%08d;%08d\r\n", (int16_t)AccData[1], (int16_t)AccData[2], (int16_t)AccData[3], (int16_t)MagData[1], (int16_t)MagData[2], (int16_t)MagData[3], (int16_t)GyroData[1], (int16_t)GyroData[2], (int16_t)GyroData[3]);
		 //CDC_Transmit_FS((unsigned char*)str1, strlen(str1));
	  //UART_TX_to_computer_DMA(str1);
	  /*
	  UART_TX_to_computer_DMA(TX_data);
	  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
	  LL_mDelay(500);
	  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
	  LL_mDelay(500);
	  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
	  LL_mDelay(500);
	  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
	  */
	// HAL_UART_Transmit(&huart1, TX_data, sizeof(TX_data), 0xFFFF);
	// UART_to_Computer(TX_data);


	  //sprintf(z, "%08d\r\n", (int16_t)readSensor());
	  //snprintf (TX_data, sizeof(), "%08d", readSensor());

	//  UART_to_Computer(TX_data);
	//  LL_mDelay(2000);
	// LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
	// LL_mDelay(200);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(72000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void ProcessHeadTracking()
{
	readSensor3(AccData, MagData, GyroData);
	calibrateData();
	processEulerMatrix();
	processArduinoReport();
}

void SendTestData()
{
	readSensor3(AccData, MagData, GyroData);
	calibrateData();
	processEulerMatrix();
	 sprintf(str0,"%08d;%08d;%08d;%08d;%08d;%08d\r\n", (int16_t)Euler[0], (int16_t)Euler[1], (int16_t)Euler[2], (int16_t)Gyr[0], (int16_t)Gyr[1], (int16_t)Gyr[2]);
 	 CDC_Transmit_FS((unsigned char*)str0, strlen(str0));
}


void calibrateData()
  {
	  MagR[0]=(float)MagData[0]-MagShift[0];
	  MagR[1]=(float)MagData[1]-MagShift[1];
	  MagR[2]=(float)MagData[2]-MagShift[2];
	  V3fTransform(MagR, MagCalibr, Mag0);
	  Mag[0] = Mag0[1];
	  Mag[1] = Mag0[0];
	  Mag[2] = -Mag0[2];
	  AccR[0]=(float)AccData[0]-AccShift[0];
	  AccR[1]=(float)AccData[1]-AccShift[1];
	  AccR[2]=(float)AccData[2]-AccShift[2];
	  V3fTransform(AccR, AccCalibr, Acc);
	  Gyr[0]=(float)GyroData[0]-GyroShift[0];
	  Gyr[1]=(float)GyroData[1]-GyroShift[1];
	  Gyr[2]=(float)GyroData[2]-GyroShift[2];
  }

void processEulerMatrix()
{
	  V3fReject(Mag, Acc, vX);
	  V3fNormalizeSelf(vX);
	  V3fNormalize(Acc, vZ);

	  V3fCross(vZ,vX,vY);
	  M3fSetRow(M1,vX,0);
	  M3fSetRow(M1,vY,1);
	  M3fSetRow(M1,vZ,2);

	  uint8_t ttt= M3fInvert(M1, M3);

	  M3fMultiply(M3, M0, M2);
	  M3fGetEuler(M2, Euler);
	  V3fMult(Euler, 50);

	  EulerF[0]=((EulerF[0]-(float)GyroData[0]*0.0000065f)*0.9995f) + (Euler[0]*0.0005f);
	  EulerF[1]=((EulerF[1]-(float)GyroData[1]*0.0000065f)*0.9995f) + (Euler[1]*0.0005f);
	  EulerF[2]=((EulerF[2]-(float)GyroData[2]*0.0000065f)*0.9995f) + (Euler[2]*0.0005f);
}

void TransmitSensorData()
{
	 optimer++;
	 if (optimer>10000) {optimer=0;}
	 readSensor();
	 if (optimer%50==0)
	  {
		 //Передача данных в ком порт 20 раз в сек, а сбор по прерыванию 1000 раз в сек
		 sprintf(TX_data,"c = %08d;%08d;%08d;%08d;%08d;%08d;%08d;%08d;%08d\r\n", (int16_t)_axcounts, (int16_t)_aycounts, (int16_t)_azcounts, (int16_t)_hxcounts, (int16_t)_hycounts, (int16_t)_hzcounts, (int16_t)_gxcounts, (int16_t)_gycounts, (int16_t)_gzcounts);
		 //CDC_Transmit_FS((unsigned char*)str1, strlen(str1));
		 UART_TX_to_computer_DMA(TX_data);
	  }
}

void processArduinoReport()
  {
	  optimer++;
	  if (optimer>10000) {optimer=0;}

	  EulerF2[0]=EulerF2[0]*0.9f+EulerF[0]*0.1f;
	  EulerF2[1]=EulerF2[1]*0.9f+EulerF[1]*0.1f;
	  EulerF2[2]=EulerF2[2]*0.9f+EulerF[2]*0.1f;

	  if (optimer%1==0)
	  {
		  Code++;
		  if (Code>999)
			  Code=0;

			bufArd[0]=(uint8_t)(0xAA);
			bufArd[1]=(uint8_t)(0xAA);
			bufArd[2]=(uint8_t)Code;
			bufArd[3]=(uint8_t)(Code>>8);

			bufArd[4]=(uint8_t)0x00;
			bufArd[5]=(uint8_t)0x00;
			bufArd[6]=(uint8_t)0x00;
			bufArd[7]=(uint8_t)0x00;

			bufArd[8]=(uint8_t)0x00;
			bufArd[9]=(uint8_t)0x00;
			bufArd[10]=(uint8_t)0x00;
			bufArd[11]=(uint8_t)0x00;

			bufArd[12]=(uint8_t)0x00;
			bufArd[13]=(uint8_t)0x00;
			bufArd[14]=(uint8_t)0x00;
			bufArd[15]=(uint8_t)0x00;

			bufArd[16]=(uint8_t) 0x00;
			bufArd[17]=(uint8_t) 0x00;
			bufArd[18]=(uint8_t) 0x00;
			bufArd[19]=(uint8_t) 0x00;

			bufArd[20]=(uint8_t) 0x00;
			bufArd[21]=(uint8_t) 0x00;
			bufArd[22]=(uint8_t) 0x00;
			bufArd[23]=(uint8_t) 0x00;

			bufArd[24]=(uint8_t) 0x00;
			bufArd[25]=(uint8_t) 0x00;
			bufArd[26]=(uint8_t) 0x00;
			bufArd[27]=(uint8_t) 0x00;

			bufArd[28]=(uint8_t)(0x55);
			bufArd[29]=(uint8_t)(0x55);

			floatToByteArray(EulerF2[0],bufArd,4);
			floatToByteArray(EulerF2[1],bufArd,8);
			floatToByteArray(EulerF2[2],bufArd,12);

			CDC_Transmit_FS(bufArd,30);
	  }

  }


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		//ProcessHeadTracking();
		//SendTestData();
		TransmitSensorData();
		cnt++;
		if (cnt>=250)
		{
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			cnt=0;
		}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
