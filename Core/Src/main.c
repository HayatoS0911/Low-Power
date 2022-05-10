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
#include "rtc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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
uint32_t timerCounter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config_UseMSI(void);
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
  uint8_t mode = 0;
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
  MX_RTC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // select HSI as system clock source after Wake-Up from Stop mode
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG( RCC_STOP_WAKEUPCLOCK_HSI );

  while (1)
  {
    /* USER CODE END WHILE */

	  switch(mode)
	  {
	  	  case 0: // Run
	  		  while (1)
	  		  {

	  		  }
	  		  break;
	  	  case 1: // LPRun
	  		  HAL_SuspendTick();													// Stop Systick timer

	  		  SystemClock_Config_UseMSI();											// Use System Clock MSI
	  		  HAL_PWREx_EnableLowPowerRunMode();									// Enter Low Power Run
	  		  HAL_Delay(3000);														// Wait 3000ms
	  		  HAL_PWREx_DisableLowPowerRunMode();									// Disable Low Power Run
	  		  SystemClock_Config();													// Use Sysmtem Clock HSI

	  		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);				// LD2 ON
	  		  HAL_ResumeTick();														// Resume Systick timer
	  		  HAL_Delay(1000);														// Wait 1000ms
	  		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);			// LD2 OFF
	  		  break;
	  	  case 2: // HAL_Delay
	  		  while(1)
	  		  {
	  			  HAL_Delay(10000);													// Wait 10000ms
	  		  }
	  		  break;
	  	  case 3: // Sleep
	  		  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);								// Disable Wakeup Timer
	  		  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,10,RTC_WAKEUPCLOCK_CK_SPRE_16BITS);	// Set wake up timer with intterupt RTC
	  		  HAL_SuspendTick();													// Stop Systick timer

	  		  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON,PWR_SLEEPENTRY_WFE);		// Enter Sleep Mode, WakeUpEvent WFE

	  		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);				// LD2 ON
	  		  HAL_ResumeTick();														// Resume Systick timer
	  		  HAL_Delay(1000);														// Wait 1000ms
	  		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);			// LD2 OFF
	  		  break;
	  	  case 4: // LPSleep
	  		  SystemClock_Config_UseMSI();											// Use System Clock MSI
	  		  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);								// Disable Wakeup Timer
	  		  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,10,RTC_WAKEUPCLOCK_CK_SPRE_16BITS);	// Set wake up timer with intterupt RTC
	  		  HAL_SuspendTick();													// Stop Systick timer

	  		  HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON,PWR_SLEEPENTRY_WFE);	// Enter Low Power Sleep Mode, WakeUpEvent WFE
	  		  HAL_PWREx_DisableLowPowerRunMode();										// Disable Low Power Run
	  		  SystemClock_Config();													// Use Sysmtem Clock HSI

	  		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);				// LD2 ON
	  		  HAL_ResumeTick();														// Resume Systick timer
	  		  HAL_Delay(1000);														// Wait 1000ms
	  		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);			// LD2 OFF
	  		  break;
	  	  case 5: // Stop0
	  		  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);								// Disable Wakeup Timer
	  		  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,10,RTC_WAKEUPCLOCK_CK_SPRE_16BITS);	// Set wake up timer with intterupt RTC
	  		  HAL_SuspendTick();													// Stop Systick timer

	  		  HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE);		// Enter Stop0 Mode, Wake Up Event
	  		  HAL_PWREx_DisableLowPowerRunMode();									// Disable Low Power Run
	  		  SystemClock_Config();													// Use Sysmtem Clock HSI

	  		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);				// LD2 ON
	  		  HAL_ResumeTick();														// Resume Systick timer
	  		  HAL_Delay(1000);														// Wait 1000ms
			  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);			// LD2 OFF
			  break;
	  	  case 6: // stop1
	  		  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);								// Disable Wakeup Timer
	  		  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,10,RTC_WAKEUPCLOCK_CK_SPRE_16BITS);	// Set wake up timer with intterupt RTC
	  		  HAL_SuspendTick();													// Stop Systick timer

	  		  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFE);	// Enter Stop1 Mode, Wake Up Event
	  		  HAL_PWREx_DisableLowPowerRunMode();									// Disable Low Power Run
	  		  SystemClock_Config();													// Use Sysmtem Clock HSI

	  		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);				// LD2 ON
	  		  HAL_ResumeTick();														// Resume Systick timer
	  		  HAL_Delay(1000);														// Wait 1000ms
	  		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);			// LD2 OFF
	  		  break;
	  	  case 7: // stop2
	  		  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);								// Disable Wakeup Timer
	  		  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,10,RTC_WAKEUPCLOCK_CK_SPRE_16BITS);	// Set wake up timer with intterupt RTC
	  		  HAL_SuspendTick();													// Stop Systick timer

	  		  HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFE);							// Enter Stop2 Mode, Wake Up Event
	  		  HAL_PWREx_DisableLowPowerRunMode();									// Disable Low Power Run
	  		  SystemClock_Config();													// Use Sysmtem Clock HSI

	  		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);				// LD2 ON
	  		  HAL_ResumeTick();														// Resume Systick timer
	  		  HAL_Delay(1000);														// Wait 1000ms
	  		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);			// LD2 OFF
	  		  break;
	  	  case 8:	// Stanby Mode
	  		  // Check if the SB flag is set
	  		  if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
	  		  {
	  			  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);								// Clear Stanby Flag

	  			  for(int i = 0; i < 5;i++)
	  			  {
	  				  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);					// Toggle LD2
	  				  HAL_Delay(100);												// Wait 100ms
	  			  }
	  			  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);		// LD2 OFF
	  		  }

	  		  while (1)
	  		  {
	  			  HAL_SuspendTick();												// Stop Systick timer
	  			  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);							// RTC timer wake up timer stop
	  			  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,10,RTC_WAKEUPCLOCK_CK_SPRE_16BITS);	// Set wake up timer with intterupt RTC
	  			  HAL_PWR_EnterSTANDBYMode();												// Enter Stanby Mode
	  		  }
	  	  case 9: // Shutdown
	  		  while (1)
	  		  {
	  			  HAL_SuspendTick();												// Stop Systick timer
	  			  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);							// RTC timer wake up timer stop
	  			  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,10,RTC_WAKEUPCLOCK_CK_SPRE_16BITS);	// Set wake up timer with intterupt RTC
	  			  HAL_PWREx_EnterSHUTDOWNMode();									// Enter Shutdown Mode
	  		  }
	  		  break;
}
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief System Clock Configuration Use MSI
  * @retval None
  */
void SystemClock_Config_UseMSI(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief TIM Period Elapsed Callback
  * @param  htim: call back tim
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2)																// timer "htim2" ?
    {
    	if(timerCounter > 0)														// Check the number of timer counters remaining
    	{
    		timerCounter--;															// timer counter -1
    		HAL_TIM_Base_Start_IT(&htim2);											// Start TIM2
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
