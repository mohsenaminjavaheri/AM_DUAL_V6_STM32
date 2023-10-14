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
/*
	I2C Register Table
		
	Function			F. Mode			Byte 0			Byte1
	
	Start PWM			Rec.			0				1
	Stop PWM			Rec.			0				2
	Change Relay		Rec.			1				1
	Send Relay State	Trs.			2				1
	Auto Tune			Rec.			2				2
	Send Tune Value		Trs.			4				1

*/

	
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

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

uint8_t I2C_Address = 20;

uint8_t ReceiveData[2]={0.0};

uint16_t Tune[100];
uint8_t state=0 , MaxTuneIdx=0 , AdcConvCplt=0 ,AdcConvStart=0;
bool Autochange=false , completeSearch=false;
uint32_t bin, gray, temp1, count, count2=0, temp2;

uint8_t i=0,I2C;

uint8_t receivingBuffer[5] = {0};

float TuneVoltage3,TuneVoltage5,TuneCurrent,MaxTune;
float SumTuneCurrent , avgTune , AvgTuneCurrent;
uint16_t TransmitData , MaxTuneValue , TuneValue , SumTune;

uint16_t mVperAmp = 100; // use 100 for 20A Module and 66 for 30A Module
uint16_t ACSoffset = 2500; 
float adcFilter;
char temp[50],len;

uint32_t debounce_delay=300,last_press_tick,PRESSED_KEY;

uint8_t	BUZcount,INTcount,AlarmCount;

uint32_t pulse;
uint16_t maxTune;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

int bintogray(int n);
void swap(int* xp, int* yp);
void selectionSort(int arr[], int n);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


uint8_t DelayStateMachine(uint16_t Delay)
{
	static uint32_t prevTick = 0;
	uint32_t currentTick = HAL_GetTick();
	if(currentTick - prevTick > Delay)
	{
		prevTick = currentTick;
		return 1;
	}
	else
		return 0;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == ZCD_Pin)
	{
		HAL_GPIO_TogglePin(ZC_Out_GPIO_Port, ZC_Out_Pin);			
	}
	
//	if(GPIO_Pin == TX_Pin)
//	{
//		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
////			NVIC_SystemReset();
//	}
	
	
	if(GPIO_Pin == Key_Pin)
	{
		last_press_tick = HAL_GetTick();
		PRESSED_KEY=1;
//		HAL_TIM_Base_Start_IT(&htim16);
		

	}
		
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim1)
	{
		pulse++;
		
		if(pulse >= 70)
		{			
			HAL_GPIO_WritePin(Tx_En_GPIO_Port,Tx_En_Pin , 1);
			HAL_TIM_PWM_Stop_IT(&htim1 , TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop_IT(&htim1 , TIM_CHANNEL_1);
//			HAL_ADC_Stop_DMA(&hadc);
//			HAL_ADC_Start_DMA(&hadc,(uint32_t *)Tune,100);
			AdcConvStart = 0;
			pulse = 0;
		}

	}
	
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	//=================Read From I2C - 1MHz===================//
	if(htim == &htim3)
	{
		HAL_I2C_Slave_Receive_DMA(&hi2c1,ReceiveData,2);		
	}


	//=================Timer For Chenge Relay in Auto Tune - 0.5 Hz===================//
	if(htim == &htim14)
	{
	//		Autochange = true;
		if(Autochange == true)
		{

			if(state > 32)
			{
				Autochange = false;
				completeSearch = true;
				HAL_GPIO_WritePin(GPIOB , 0x1F<<3 , 0);
				HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,1);
				state=0;		
			}
			else
			{
				temp1=bintogray(state);// ^ bintogray((state+1)%32);	
				HAL_GPIO_WritePin(GPIOB , 0x1F<<3 , 0);
				HAL_GPIO_WritePin(GPIOB,	temp1<<3 , 1);
				state++;
			}
		}
		else if(completeSearch == true)
		{			
			if(i>5)
			{
				i=0;
				completeSearch = false;
				state = MaxTuneIdx;
				HAL_TIM_Base_Stop_IT(&htim14);
			}
			else
			{
				temp2 = (MaxTuneIdx >> i) & 0x01;
				HAL_GPIO_WritePin(GPIOB,	temp2<<(3+i) , 1);
				i++;
			}
		}
	}
	
	if(htim == &htim16)
	{

		AlarmCount++;
		if(AlarmCount > 29)
		{
			AlarmCount = 0;
			HAL_GPIO_WritePin(Alarm_GPIO_Port,Alarm_Pin , 0);
			HAL_TIM_Base_Stop_IT(&htim16);
		}
	}
}

//========================I2C Receive Callback Functions=====================//
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	receivingBuffer[ReceiveData[0]] = ReceiveData[1];
//	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	//==================Change Tune Relay=====================//
	if(ReceiveData[0] == 2)
	{
		if(ReceiveData[1] == 128)
		{
			HAL_GPIO_WritePin(GPIOB , 0x1F<<3 , 0);
			state++;
			HAL_GPIO_WritePin(GPIOB,bintogray(state)<<3 , 1);
			ReceiveData[0] = 0;
			MaxTune = 0;
		}
		else
		{	
			HAL_GPIO_WritePin(GPIOB , 0x1F<<3 , 0);
			state = ReceiveData[1];
			HAL_GPIO_WritePin(GPIOB,state<<3 , 1);
			ReceiveData[0] = 0;
			MaxTune = 0;
		}
	}
	//==================Send Tune Value======================//
	if(ReceiveData[0] == 4)
	{
		AdcConvStart = 1;
		HAL_I2C_Slave_Transmit_DMA(&hi2c1 , (uint8_t *)&MaxTuneValue , 1);
		ReceiveData[0] = 0;
		ReceiveData[1] = 0;
//		MaxTune = 0;
		receivingBuffer[4] = 0;				
	}	

	//==================Enable Tx Signal======================//
	if(receivingBuffer[1] == 1)
	{
		HAL_GPIO_WritePin(Tx_En_GPIO_Port,Tx_En_Pin , 0);
		HAL_TIM_PWM_Start_IT(&htim1 , TIM_CHANNEL_1);
		HAL_TIMEx_PWMN_Start_IT(&htim1 , TIM_CHANNEL_1);			
		ReceiveData[0] = 0;
		ReceiveData[1] = 0;
		HAL_ADC_Start_DMA(&hadc,(uint32_t *)Tune,100);
		AdcConvStart = 1;
		receivingBuffer[1] = 0;		
	}
	//=========================Alarm=========================//
	if(receivingBuffer[3] == 1)
	{
		HAL_GPIO_WritePin(Alarm_GPIO_Port,Alarm_Pin , 1);
		HAL_TIM_Base_Start_IT(&htim16);
		receivingBuffer[3] = 0;
	}
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
//	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	AdcConvCplt = 1;	
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
	


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_ADCEx_Calibration_Start(&hadc);
	
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
//		if(DelayStateMachine(200) == 1)
//		{
//			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//		}
		
		
		//================Send Tune Relay Value===================//
//		if(receivingBuffer[2] == 1)
//		{
////			TransmitData = MaxTuneIdx;
//			HAL_I2C_Slave_Transmit_DMA(&hi2c1 , (uint8_t *)&state , 1);
//			receivingBuffer[2] = 0;
////			TransmitData = 0;
//		}
		
		//=======================Auto Tune========================//
//		if(receivingBuffer[2] == 2)
//		{
//			HAL_GPIO_WritePin(GPIOB , 0x1F<<3 , 0);
//			state = 0;
//			Autochange = 1;
//			receivingBuffer[2] = 0;
//		}
		
//		//=========================Alarm=========================//
//		if(receivingBuffer[3] == 1)
//		{
//			HAL_TIM_Base_Start_IT(&htim16);
//			receivingBuffer[3] = 0;
//		}


		

		
//		if(HAL_GetTick() - last_press_tick > debounce_delay  && PRESSED_KEY == 1)		
//		{
//			if(HAL_GPIO_ReadPin(Key_GPIO_Port,Key_Pin) == 1)
//			{		
////				HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
////				I2C ^= 1;
//				HAL_GPIO_WritePin(GPIOB , 0x1F<<3 , 0);
//				if(state >= 32)
//					state = 0;
//				else
//					state++;
//				HAL_GPIO_WritePin(GPIOB,bintogray(state)<<3 , 1);
//			}
//			PRESSED_KEY = 0;
//		}
		
//		if(I2C == 1)
//			HAL_TIM_Base_Start_IT(&htim3);
//		else
//			HAL_TIM_Base_Stop_IT(&htim3);

		
		if(Autochange)
		{			
			HAL_TIM_Base_Start_IT(&htim14);
			if(AdcConvStart == 1)
				HAL_ADC_Start_DMA(&hadc,(uint32_t *)Tune,50);
			
			if(AdcConvCplt == 1)
			{

//				selectionSort((int *)Tune , 50);			
				
				for(int i = 0; i<50 ; i++)
					adcFilter = 0.7*adcFilter + 0.3*Tune[25];	
				
				TuneVoltage3 = (adcFilter / 4095.0) * 3300.0;				//voltage in mV
				TuneVoltage5 = TuneVoltage3 * 5000.0 / 3300.0;
				TuneCurrent = (TuneVoltage5 - ACSoffset) / mVperAmp;	
				TuneValue = (uint8_t)TuneCurrent;
				
				if(TuneCurrent < 0)
					TuneCurrent = TuneCurrent*(-1);

				
				if(TuneCurrent > MaxTune)
				{
					MaxTune = TuneCurrent;
					MaxTuneIdx = state-1;	
				}
				
				AdcConvCplt = 0;
			}
		}	
		
		else
		{
//			if(AdcConvStart == 1)
//				HAL_ADC_Start_DMA(&hadc,(uint32_t *)Tune,100);
			
			if(AdcConvCplt == 1)
			{
//				selectionSort((int *)Tune , 100);			
				
//				for(int i = 0; i<100 ; i++)
//					adcFilter = 0.7*adcFilter + 0.3*Tune[50];	
				
//				adcFilter = Tune[50];
				maxTune = 0;
				for(int i = 0; i<100 ; i++)
				{
					if(Tune[i] > 3000)
						Tune[i] = 0;
					adcFilter += Tune[i];
				}

				
				adcFilter = adcFilter/100;
				
				TuneVoltage5 = (adcFilter / 4095.0) * 5000.0;
				TuneCurrent = (TuneVoltage5 - ACSoffset) / mVperAmp;	
				TuneValue = (uint8_t)TuneCurrent;
				
				if(TuneCurrent < 0)
					TuneCurrent = TuneCurrent*(-1);
				
//				if(TuneCurrent > 3)
//				{
//					HAL_GPIO_WritePin(Tx_En_GPIO_Port,Tx_En_Pin , 1);
//				}
				
				
				SumTuneCurrent += TuneCurrent;
				SumTune++;
//				
				if(SumTune >= 20)
				{
					AvgTuneCurrent = SumTuneCurrent/20;
					MaxTune = AvgTuneCurrent*10;
					
					MaxTuneIdx = state;
					MaxTuneValue = (uint16_t)MaxTune;
					SumTuneCurrent = 0;
					SumTune = 0;
				}

	
				AdcConvCplt = 0;
			}
				
		}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

int bintogray(int n)
{
    return n ^ (n >> 1);
}

void swap(int* xp, int* yp)
{
    int temp = *xp;
    *xp = *yp;
    *yp = temp;
}
void selectionSort(int arr[], int n)
{
    int i, j, min_idx;
 
    // One by one move boundary of unsorted subarray
    for (i = 0; i < n - 1; i++) {
 
        // Find the minimum element in unsorted array
        min_idx = i;
        for (j = i + 1; j < n; j++)
            if (arr[j] < arr[min_idx])
                min_idx = j;
 
        // Swap the found minimum element
        // with the first element
        swap(&arr[min_idx], &arr[i]);
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
