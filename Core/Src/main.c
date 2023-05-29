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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050.h"
#include <string.h>
#include <stdio.h>
#include "ACS711EX.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t bufforADC[3];
uint16_t bufforCalib[3];

char buffor[70];
char bufforStartStop[1];
volatile uint8_t cmplt = 1;
volatile uint8_t xD = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	MPU6050_Config_TypeDef mpu6050;volatile int SAMPLE = 0 ;
	float accBuffor[3],gyroBuffor[3];uint16_t bufforADC[3];float bufforADCOUput[3];
	//char buffor[100];
	struct Data {


			 int sample;

			 float ax;
			 float ay;
			 float az;
			 float gx;
			 float gy;
			 float gz;
			 float  phasea;
			 float phaseb;
			 float phasec;



	};


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  mpu6050.ACC_RANGE = MPU6050_ACC_AFS_2G;
  mpu6050.CLOCK = MPU6050_CLOCK_SOURCE_0;
  mpu6050.FILTER = MPU6050_LOW_PASS_FILTER_0;
  mpu6050.GYRO_RANGE = MPU6050_GYRO_FS_500;
  mpu6050.TEMP_ON_OFF = ENABLE;

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(15);
  MPU6050_Init(&hi2c1, &mpu6050);

  HAL_UART_Receive_DMA(&huart2, (uint8_t*)bufforStartStop, 1);

  //HAL_TIM_Base_Start_IT(&htim3);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)bufforCalib, 3);
  HAL_Delay(1);

  HAL_TIM_Base_Start_IT(&htim10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {




    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)bufforADC, 3);
    SAMPLE +=1;
    MPU6050_Get_Acc_Value(&hi2c1,&mpu6050,accBuffor);
	MPU6050_Get_Gyro_Value(&hi2c1,&mpu6050,gyroBuffor);


	ACS711EX_Convert_To_mA(bufforADC, bufforADCOUput);

//	struct Data  DataToSend = {
//
//
//
//			.sample = SAMPLE,
//
//			.ax = accBuffor[0],
//			.ay = accBuffor[1],
//			.az = accBuffor[2],
//			.gx = gyroBuffor[0],
//			.gy = gyroBuffor[1],
//			.gz = gyroBuffor[2],
//			.phasea = bufforADCOUput[0],
//			.phaseb = bufforADCOUput[1],
//			.phasec = bufforADCOUput[2],
//
//
//
//	};
//
//	HAL_UART_Transmit(&huart2, (uint8_t*)"S", sizeof("S"), 1);
//	HAL_UART_Transmit(&huart2, (uint8_t*)&DataToSend,sizeof(DataToSend),1000);
//	HAL_UART_Transmit(&huart2, (uint8_t*)"E", sizeof("E"), 1);


	snprintf(buffor,100,"%d;%0.3f;%0.3f;%0.3f;%0.3f;%0.3f;%0.3f;%0.3f;%0.3f;%0.3f\n",SAMPLE,accBuffor[0],accBuffor[1],accBuffor[2],gyroBuffor[0],gyroBuffor[1],gyroBuffor[2],bufforADCOUput[0],bufforADCOUput[1],bufforADCOUput[2]);
	HAL_UART_Transmit(&huart2, (uint8_t*)buffor	,sizeof(buffor),10000);



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	//MPU6050_Get_Acc_Value(&hi2c1, &mpu6050, buffor);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//
//
//{
//
//	//if(htim==&htim10)
//		///cmplt = 1;
//
//			//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//	;
//}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

	//cmplt = 0;
	//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, RESET);
	memset(buffor,0,sizeof(buffor));
    cmplt = 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart==&huart2)
  {
    if (huart->Instance->DR == 's')
    {
    	HAL_TIM_Base_Start_IT(&htim3);
      HAL_UART_Receive_DMA(&huart2, (uint8_t*)bufforStartStop, 1);
    }
    else if (huart->Instance->DR == 'e')
    {
    	HAL_TIM_Base_Stop_IT(&htim3);
      HAL_UART_Receive_DMA(&huart2, (uint8_t*)bufforStartStop, 1);
    }
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
