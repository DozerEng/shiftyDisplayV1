/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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

COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_TIM3_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  printf("\n\rStarting NUCLEO-STM32G491RE...\n\n\r");

  const uint32_t MIN_TIM_PERIOD = 65535;
  const uint32_t MAX_TIM_PERIOD = 2360;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  /*
   * Initialize Shift Register Outputs
   */
  // Initialize control registers as appropriate
  HAL_GPIO_WritePin(row_SR_nOE_GPIO_Port, row_SR_nOE_Pin, 0); // Row is disabled to start, will be enabled when column values are set
  HAL_GPIO_WritePin(row_SR_RCLK_GPIO_Port, row_SR_RCLK_Pin, 0);
  HAL_GPIO_WritePin(row_SR_SRCLK_GPIO_Port, row_SR_SRCLK_Pin, 0);
  HAL_GPIO_WritePin(row_SR_nSRCLR_GPIO_Port, row_SR_nSRCLR_Pin, 0);

  HAL_GPIO_WritePin(col_SR_nOE_GPIO_Port, col_SR_nOE_Pin, 1);
  HAL_GPIO_WritePin(col_SR_RCLK_GPIO_Port, col_SR_RCLK_Pin, 0);
  HAL_GPIO_WritePin(col_SR_SRCLK_GPIO_Port, col_SR_SRCLK_Pin, 0);
  HAL_GPIO_WritePin(col_SR_nSRCLR_GPIO_Port, col_SR_nSRCLR_Pin, 0);

  // Remove shift register clear
  HAL_GPIO_WritePin(row_SR_nSRCLR_GPIO_Port, row_SR_nSRCLR_Pin, 1);
  HAL_GPIO_WritePin(col_SR_nSRCLR_GPIO_Port, col_SR_nSRCLR_Pin, 1);

  // Set all shift register serial outputs to 0
  HAL_GPIO_WritePin(row_SR_SER_GPIO_Port, row_SR_SER_Pin, 0);

  HAL_GPIO_WritePin(col_SR_SER0_GPIO_Port, col_SR_SER0_Pin, 0);
  HAL_GPIO_WritePin(col_SR_SER1_GPIO_Port, col_SR_SER1_Pin, 0);
  HAL_GPIO_WritePin(col_SR_SER2_GPIO_Port, col_SR_SER2_Pin, 0);
  HAL_GPIO_WritePin(col_SR_SER3_GPIO_Port, col_SR_SER3_Pin, 0);
  HAL_GPIO_WritePin(col_SR_SER4_GPIO_Port, col_SR_SER4_Pin, 0);
  HAL_GPIO_WritePin(col_SR_SER5_GPIO_Port, col_SR_SER5_Pin, 0);
  HAL_GPIO_WritePin(col_SR_SER6_GPIO_Port, col_SR_SER6_Pin, 0);



  const uint8_t CLK_DELAY = 1;
  uint8_t row = 0;
  #define ROW_COUNT 18
  #define SR_COUNT 7

//  typedef struct frame {
//	  uint8_t row[ROW_COUNT][SR_COUNT];
//  } Frame;

  const uint8_t frame_hello_world[ROW_COUNT][SR_COUNT] = {
	  {0, 0, 0, 0, 0, 0, 0},
	  {8, 190, 130, 7, 0, 0, 0},
	  {8, 160, 130, 8, 128, 0, 0},
	  {8, 160, 130, 8, 128, 65, 0},
	  {15, 184, 130, 8, 128, 65, 0},
	  {8, 160, 130, 8, 128, 65, 0},
	  {8, 160, 130, 8, 128, 65, 0},
	  {8, 190, 251, 231, 0, 65, 0},
	  {0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 2, 0, 32},
	  {8, 39, 60, 131, 194, 0, 32},
	  {8, 40, 162, 130, 34, 0, 32},
	  {9, 40, 162, 130, 33, 0, 64},
	  {9, 40, 188, 130, 32, 128, 128},
	  {9, 40, 162, 130, 32, 127, 0},
	  {9, 40, 162, 130, 32, 0, 0},
	  {6, 199, 34, 251, 192, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0},
	  };

  const uint8_t frame_all_on[ROW_COUNT][SR_COUNT] = {
	  {255, 255, 255, 255, 255,	255, 255},
	  {255,	255, 255, 255, 255,	255, 255},
	  {255,	255, 255, 255, 255,	255, 255},
	  {255,	255, 255, 255, 255,	255, 255},
	  {255,	255, 255, 255, 255,	255, 255},
	  {255,	255, 255, 255, 255,	255, 255},
	  {255,	255, 255, 255, 255,	255, 255},
	  {255,	255, 255, 255, 255,	255, 255},
	  {255,	255, 255, 255, 255,	255, 255},
	  {255,	255, 255, 255, 255,	255, 255},
	  {255,	255, 255, 255, 255,	255, 255},
	  {255,	255, 255, 255, 255,	255, 255},
	  {255,	255, 255, 255, 255,	255, 255},
	  {255,	255, 255, 255, 255,	255, 255},
	  {255,	255, 255, 255, 255,	255, 255},
	  {255,	255, 255, 255, 255,	255, 255},
	  {255,	255, 255, 255, 255,	255, 255},
	  {255,	255, 255, 255, 255,	255, 255}
  	  };

  const uint8_t frame_list_of_numbers[ROW_COUNT][SR_COUNT] = {
	  {0, 0, 0, 0, 0, 0, 0},
	  {7, 0, 64, 56, 14, 0, 128},
	  {8, 128, 192, 68, 17, 1, 128},
	  {8, 129, 64, 4, 1, 2, 128},
	  {8, 128, 64, 24, 14, 4, 128},
	  {8, 128, 64, 32, 1, 7, 192},
	  {8, 128, 64, 64, 17, 0, 128},
	  {7, 1, 240, 124, 14, 0, 128},
	  {0, 0, 0, 0, 0, 0, 0},
	  {0, 0, 0, 0, 0, 0, 0},
	  {15, 128, 224, 124, 14, 3, 128},
	  {8, 1, 16, 4, 17, 4, 64},
	  {15, 1, 0, 8, 17, 4, 64},
	  {0, 129, 224, 16, 14, 3, 192},
	  {0, 129, 16, 16, 17, 0, 64},
	  {8, 129, 16, 16, 17, 4, 64},
	  {7, 0, 224, 16, 14, 3, 128},
	  {0, 0, 0, 0, 0, 0, 0},
	  };


//  uint8_t currentFrame[ROW_COUNT][SR_COUNT];
//  memcpy(currentFrame, frame_helloWorld, ROW_COUNT*SR_COUNT);

  uint8_t (*current_frame)[SR_COUNT];
  uint8_t (*next_frame)[SR_COUNT];
  current_frame = frame_hello_world;
  next_frame = NULL;
  // Array of different frames and index to track
  uint8_t frame_index = 0;
  #define FRAME_ARRAY_COUNT 3
  uint8_t *frame_array[] = {
	  frame_hello_world,
	  frame_list_of_numbers,
	  frame_all_on
  };

  while (1)
  {
	  // Check if the bottom row has been displayed
	  if(row >= ROW_COUNT - 1) {
		  // Reset row index to the top row
		  row = 0;
		  HAL_GPIO_WritePin(row_SR_SER_GPIO_Port, row_SR_SER_Pin, 1);
		  // Check if a new frame is available, and if so, update current_frame
		  if(next_frame != NULL) {
			  current_frame = next_frame;
			  next_frame = NULL;
		  }
	  } else {
		  // Increment row
		  row++;
		  HAL_GPIO_WritePin(row_SR_SER_GPIO_Port, row_SR_SER_Pin, 0);
	  }

	  /*
	   * Shift each LED value  into the column shift registers
	   */
	  for(uint8_t bit = 0; bit < 8; bit ++) {
		  // Set serial output to each shift register
		  HAL_GPIO_WritePin(col_SR_SER0_GPIO_Port, col_SR_SER0_Pin, (current_frame[row][0] >> bit) & 0x01);
		  HAL_GPIO_WritePin(col_SR_SER1_GPIO_Port, col_SR_SER1_Pin, (current_frame[row][1] >> bit) & 0x01);
		  HAL_GPIO_WritePin(col_SR_SER2_GPIO_Port, col_SR_SER2_Pin, (current_frame[row][2] >> bit) & 0x01);
		  HAL_GPIO_WritePin(col_SR_SER3_GPIO_Port, col_SR_SER3_Pin, (current_frame[row][3] >> bit) & 0x01);
		  HAL_GPIO_WritePin(col_SR_SER4_GPIO_Port, col_SR_SER4_Pin, (current_frame[row][4] >> bit) & 0x01);
		  HAL_GPIO_WritePin(col_SR_SER5_GPIO_Port, col_SR_SER5_Pin, (current_frame[row][5] >> bit) & 0x01);
		  HAL_GPIO_WritePin(col_SR_SER6_GPIO_Port, col_SR_SER6_Pin, (current_frame[row][6] >> bit) & 0x01);

		  // Increment shift register storage location
		  HAL_GPIO_WritePin(col_SR_SRCLK_GPIO_Port, col_SR_SRCLK_Pin, 1);
		  HAL_GPIO_WritePin(col_SR_SRCLK_GPIO_Port, col_SR_SRCLK_Pin, 0);
	  }
	  // Disable row so output is off while transitioning to the next row
	  HAL_GPIO_WritePin(col_SR_nOE_GPIO_Port, col_SR_nOE_Pin, 1);

	  // Latch shift registers
	  HAL_GPIO_WritePin(col_SR_RCLK_GPIO_Port, col_SR_RCLK_Pin, 1);
	  HAL_GPIO_WritePin(col_SR_RCLK_GPIO_Port, col_SR_RCLK_Pin, 0);

	  // Increment row shift registers
	  HAL_GPIO_WritePin(row_SR_SRCLK_GPIO_Port, row_SR_SRCLK_Pin, 1);
	  HAL_GPIO_WritePin(row_SR_RCLK_GPIO_Port, row_SR_RCLK_Pin, 1);

	  HAL_GPIO_WritePin(row_SR_SRCLK_GPIO_Port, row_SR_SRCLK_Pin, 0);
	  HAL_GPIO_WritePin(row_SR_RCLK_GPIO_Port, row_SR_RCLK_Pin, 0);

	  // Enable row
	  HAL_GPIO_WritePin(col_SR_nOE_GPIO_Port, col_SR_nOE_Pin, 0);


	  /*
	   *  Enter config mode if button pressed
	   */
	  if(BSP_PB_GetState(BUTTON_USER) == BUTTON_PRESSED) {
		  // Wait for button to be released
		  HAL_Delay(50); // De-bounce
		  BSP_LED_On(LED_GREEN);
		  HAL_GPIO_WritePin(col_SR_nOE_GPIO_Port, col_SR_nOE_Pin, 1);
		  printf("Entering configuration mode\n\r");
		  while(BSP_PB_GetState(BUTTON_USER) == BUTTON_PRESSED);
		  HAL_Delay(50); // De-bounce

		  // Increment frame
		  if(frame_index < FRAME_ARRAY_COUNT - 1 ) {
			  frame_index ++;
		  } else {
			  frame_index = 0;
		  }

		  next_frame = frame_array[frame_index];

		  /* This section is not current being used

		  // Do this while waiting for another button push
		  uint32_t previousADCValue = 4097; // Should be larger than the maximum ADC value so output is updated on first iteration
		  while(BSP_PB_GetState(BUTTON_USER) == BUTTON_RELEASED) {
			  // Start ADC and sample
			  HAL_ADC_Start(&hadc2);
			  HAL_ADC_PollForConversion(&hadc2, 100);
			  uint32_t newADCValue = HAL_ADC_GetValue(&hadc2);
			  // Check if there is a big enough difference between new and old ADC values
			  uint32_t threshold = 200;
			  uint32_t difference = 0;
			  if (newADCValue > previousADCValue) {
				  difference = newADCValue - previousADCValue;
			  } else {
				  difference = previousADCValue - newADCValue;
			  }
//			  printf("difference: %lu\n\r", difference);
//			  previousADCValue = newADCValue;

			  // Print ADC reading as ratio of full scale (12-bit)
			  if ( difference > threshold) {
				  double adcPercentage = newADCValue/4096.0;
				  printf("ADC Value: %0.3f\n\r", adcPercentage);
				  uint32_t newTIMPeriod = (uint32_t)((MIN_TIM_PERIOD - MAX_TIM_PERIOD) * adcPercentage) + MAX_TIM_PERIOD;
				  htim3.Instance->ARR = newTIMPeriod;
				  htim3.Instance->CCR1 = (uint32_t)(newTIMPeriod * 0.5);
				  printf("New TIM Period: %lu\n\r", newTIMPeriod);
				  previousADCValue = newADCValue;
			  }
			  HAL_Delay(100);
		  }
		  // Wait for button to be released
		  while(BSP_PB_GetState(BUTTON_USER) == BUTTON_PRESSED);
		  printf("Exiting configuration mode\n\r");
		  BSP_LED_Off(LED_GREEN);
		  HAL_GPIO_WritePin(col_SR_nOE_GPIO_Port, col_SR_nOE_Pin, 0);

		  */


	  } // EO config mode






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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
