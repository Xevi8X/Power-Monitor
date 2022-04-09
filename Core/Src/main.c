/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CHANNELS 3
#define OVERSAMPLING 8
#define BUFFERSIZE 128
#define EXPECTEDFREQ 50
#define CALIBRATIONPERIOD 128
#define CURRENTSCALE 362.2
#define VOLTAGESCALE 34.8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t ADC_Buffer[2*CHANNELS];
uint32_t* halfOfADC_Buffer = ADC_Buffer + CHANNELS;
int16_t data[BUFFERSIZE][CHANNELS*2];
uint32_t time[BUFFERSIZE];
uint16_t indexCircBuffer;
uint8_t oversamplingIndex;
uint16_t calibZeros[CHANNELS*2] = {0};
uint64_t RMS[6] = {0};
//0,1,2 - V
//3,4,5 - A
uint8_t correctionRMS;
uint16_t calibCounter;
int64_t P[3] = {0};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ADC_Start(void);
void takeData(uint32_t* buffer);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);
int __io_putchar(int ch);
uint32_t getCurrentMicros(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void ADC_Start(void)
{
	while(HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK);
	while(HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK);
	HAL_Delay(10);
	HAL_ADC_Start(&hadc2);
	HAL_ADCEx_MultiModeStart_DMA(&hadc1, ADC_Buffer, (uint32_t)2 * CHANNELS);
}

void CalcRMScorection()
{
		while(indexCircBuffer!= 0);
		__disable_irq();
		uint32_t timeOfBufforing = time[BUFFERSIZE-1]- time[0];
		uint32_t halfPhase = 1000000/EXPECTEDFREQ/2;
		uint16_t halfPeriods = timeOfBufforing/halfPhase;

		while(time[BUFFERSIZE-1-correctionRMS] > time[0] + halfPhase*halfPeriods) correctionRMS++;
		__enable_irq();
}

void CalibrateZero()
{

	printf("Starting calibration...\n");
	//printf("Press button when voltage and current is equal to 0\n");
	while(indexCircBuffer!= 0);
	__disable_irq();
	//while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) != GPIO_PIN_RESET);

	int32_t sum[CHANNELS*2] = {0};
	for(uint16_t i = correctionRMS; i < BUFFERSIZE;i++)
	{
		for(uint8_t j = 0; j < CHANNELS*2;j++)
		{
			sum[j] += data[i][j];
			data[i][j] = 0;
		}
	}
	for(uint8_t j = 0; j < CHANNELS*2;j++)
	{
		calibZeros[j] += (sum[j]/(BUFFERSIZE-correctionRMS));
		data[0][j] = -calibZeros[j];
		RMS[j] = 0;
	}
	for(uint8_t j = 0; j < CHANNELS;j++)
	{
		P[j] = 0;
	}


	printf("Calibration completed\n");
	__enable_irq();
}

void takeData(uint32_t* buffer)
{
	if(oversamplingIndex == OVERSAMPLING)
	{
		oversamplingIndex = 0;
		time[indexCircBuffer] = getCurrentMicros();
		for(uint8_t i = 0; i < CHANNELS*2;i++)
		{
			RMS[i] += data[indexCircBuffer][i]*data[indexCircBuffer][i];
		}
		for(uint8_t i = 0; i < CHANNELS;i++)
		{
			P[i] += data[indexCircBuffer][2*i]*data[indexCircBuffer][2*i+1];
		}
		indexCircBuffer++;
		if(indexCircBuffer == BUFFERSIZE)
		{
			indexCircBuffer = 0;
			calibCounter++;
			if(calibCounter == CALIBRATIONPERIOD)
			{
				calibCounter = 0;
				CalibrateZero();
			}
			//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		}
		for(uint8_t i = 0; i < CHANNELS;i++)
		{
			P[i] -= data[(indexCircBuffer+correctionRMS)% BUFFERSIZE][2*i]*data[(indexCircBuffer+correctionRMS)% BUFFERSIZE][2*i+1];
		}
		for(uint8_t i = 0; i < CHANNELS*2;i++)
		{
			RMS[i] -= data[(indexCircBuffer+correctionRMS)% BUFFERSIZE][i]*data[(indexCircBuffer+ correctionRMS)% BUFFERSIZE][i];
			data[indexCircBuffer][i] = -calibZeros[i];
		}

	}
	for(uint8_t i = 0; i < CHANNELS;i++)
	{
		data[indexCircBuffer][2*i] += (uint16_t) buffer[i];
		data[indexCircBuffer][2*i+1] += (uint16_t) (buffer[i] >> 16);
	}
	oversamplingIndex++;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc)
	{
		takeData(ADC_Buffer);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc)
	{
		takeData(halfOfADC_Buffer);
	}
}


int __io_putchar(int ch)
{
  if (ch == '\n') {
    __io_putchar('\r');
  }

  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

  return 1;
}

static inline uint32_t LL_SYSTICK_IsActiveCounterFlag(void)
{
  return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}

uint32_t getCurrentMicros(void)
{
  /* Ensure COUNTFLAG is reset by reading SysTick control and status register */
  LL_SYSTICK_IsActiveCounterFlag();
  uint32_t m = HAL_GetTick();
  const uint32_t tms = SysTick->LOAD + 1;
  __IO uint32_t u = tms - SysTick->VAL;
  if (LL_SYSTICK_IsActiveCounterFlag()) {
    m = HAL_GetTick();
    u = tms - SysTick->VAL;
  }
  return (m * 1000 + (u * 1000) / tms);
}



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
  indexCircBuffer = 0;
  oversamplingIndex = 0;
  correctionRMS = 1;
  calibCounter = 0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  ADC_Start();
  HAL_Delay(1500);
  CalibrateZero();
  CalcRMScorection();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	HAL_Delay(1000);
	float V1 = sqrt((float)RMS[0]/(BUFFERSIZE-correctionRMS))/VOLTAGESCALE;
	float A1 = sqrt((float)RMS[1]/(BUFFERSIZE-correctionRMS))/CURRENTSCALE;
	float S1 = V1*A1;
	float P1 = -P[0];
	P1 /= ((BUFFERSIZE-correctionRMS)*(VOLTAGESCALE*CURRENTSCALE));
	float Q1 = sqrt(S1*S1-P1*P1);

	printf("RMS: V: %.1f,  A:%.2f,  P:%.2f,  Q:%.2f,  S:%.2f\n",V1 ,A1, P1, Q1, S1);


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
