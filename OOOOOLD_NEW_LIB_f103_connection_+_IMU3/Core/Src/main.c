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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU9250.h"
#include "vector.h"
#include <string.h>
#include "BMP_280_My.h"
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
//******FOR_BMP280******************

float temp = 0;
int16_t Pressure = 0;

//******FOR_MPU9250******************
uint8_t cnt=0;
Vector3f MagShift,AccShift,GyroShift;
Matrix3f MagCalibr, AccCalibr;
Vector3f Mag, Acc, Mag0, Gyr;
Vector3f MagR, AccR;

int16_t AccData[3];
int16_t MagData[3];
int16_t GyroData[3];

Vector3f Mag, Acc;

Vector3f vX, vY, vZ;
Matrix3f M0, M1, M2, M3;
Vector3f Euler;
Vector3f EulerF;
Vector3f EulerF2;
int optimer=0;
uint16_t Code=0;

uint8_t bufArd[30];
uint8_t testerspec[30];
char str0[100];
char str1[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */


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
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  int st;

  	MagShift[0] = 41.616649f;
  	MagShift[1] = 39.388807f;
  	MagShift[2] = -164.968190f;

  	MagCalibr[0][0] = 1.008692f;
  	MagCalibr[0][1] = 0.009086f;
  	MagCalibr[0][2] = -0.002774f;

  	MagCalibr[1][0] = 0.009086f;
  	MagCalibr[1][1] = 1.055844f;
  	MagCalibr[1][2] = 0.017625f;

  	MagCalibr[2][0] = -0.002774f;
  	MagCalibr[2][1] = 0.017625f;
  	MagCalibr[2][2] = 0.998143f;

	AccShift[0]=18.204330f;
	AccShift[1]=1.946967f;
	AccShift[2]=25.956847f;


	AccCalibr[0][0]=1.023314f;
	AccCalibr[0][1]=-0.004549f;
	AccCalibr[0][2]=0.001297f;

	AccCalibr[1][0]=-0.004549f;
	AccCalibr[1][1]=0.997855f;
	AccCalibr[1][2]=-0.000489f;

	AccCalibr[2][0]=0.001297f;
	AccCalibr[2][1]=-0.000489f;
	AccCalibr[2][2]=1.018328f;

  	GyroShift[0]=90.37f;
  	GyroShift[1]=34.80f;
  	GyroShift[2]=-19.44f;

  	uint8_t flag_for_data_capturing = 0;



	st = MPU_begin();
	HAL_Delay(100);
	setSrd(0);
	HAL_Delay(100);

	// BMP280 INIT
	BMP280_Init();
	GPIOC->ODR ^= 1<<13;
	HAL_Delay(200);
	GPIOC->ODR ^= 1<<13;
	HAL_TIM_Base_Start_IT(&htim1);
	uint8_t buff[200] = {0,};
	extern uint8_t Y_Lable;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (GPIOB->IDR & (1<<1))
	  {
		  Y_Lable = 0;
		  GPIOC->ODR |= (1<<13);
	  }
	  else
	  {
		  Y_Lable = 1;
		  GPIOC->ODR &= ~(1<<13);
	  }

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
		  {
	  	  	  readSensor3(AccData, MagData, GyroData);
	  	  	  calibrateData();

	  	  	  Pressure = (int16_t)(get_diff_preasure()*100);

	  	  	  snprintf(str1, 100, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.f,%d",
			  	  	  	  (float)Acc[0], (float)Acc[1], (float)Acc[2],
						  (float)Gyr[0], (float)Gyr[1], (float)Gyr[2],
						  (float)Mag[0], (float)Mag[1], (float)Mag[2],
						  (float)Pressure, (uint8_t)Y_Lable);

	  	  	HAL_UART_Transmit_IT(&huart1, (char*)str1, strlen(str1));
		  }

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
	sprintf(str0, "%08d;%08d;%08d\r\n", (int16_t)Euler[0], (int16_t)Euler[1], (int16_t)Euler[2]);
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)str0, strlen(str0));
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

	  V3fMult(Euler, 1);

	  EulerF[0]=((EulerF[0]-(float)GyroData[0]*0.0000065f)*0.9995f) + (Euler[0]*0.0005f);
	  EulerF[1]=((EulerF[1]-(float)GyroData[1]*0.0000065f)*0.9995f) + (Euler[1]*0.0005f);
	  EulerF[2]=((EulerF[2]-(float)GyroData[2]*0.0000065f)*0.9995f) + (Euler[2]*0.0005f);

}

void TransmitSensorData()
{
	  readSensor();

	  snprintf(str1, 100, "%08d;%08d;%08d;%08d;%08d;%08d;%08d;%08d;%08d\n",
			  	  	  	  (int16_t)_axcounts, (int16_t)_aycounts, (int16_t)_azcounts,
						  (int16_t)_hxcounts, (int16_t)_hycounts, (int16_t)_hzcounts,
						  (int16_t)_gxcounts, (int16_t)_gycounts, (int16_t)_gzcounts);

	  HAL_UART_Transmit_IT(&huart1, (uint8_t*)str1, strlen(str1));
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
	  		if (Code>999) Code=0;

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

			//HAL_UART_Transmit_IT(&huart1, (uint8_t*)bufArd, 30);

	      	  sprintf(testerspec, "%0.4f;%0.4f;%0.4ff\n", EulerF2[0], EulerF2[1], EulerF2[2]);

	      	  HAL_UART_Transmit_IT(&huart1, (uint8_t*)testerspec, strlen(testerspec));

	  	 }
  }


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	M3fDupe(M1, M0);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
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
