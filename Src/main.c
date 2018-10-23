
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "uart.h"
#include "string.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t currentMISO;
uint8_t currentMOSI;
uint8_t bitsFilled = 0;

uint8_t resetCaused = 0;
uint8_t overSized = 0;

#define HIDE_FF 0 // Hide 0xff values (set to 2 to make them blue)
#define DISPLAYSIZE 16 // Amount of data to display by default
#define STORESIZE 3 * 16 // Amount of data to store
uint8_t miso[STORESIZE]; // Safety margin for memory allocation
uint8_t mosi[STORESIZE];
volatile uint16_t displayPosition = 0;

uint8_t nssTrigger[STORESIZE] = { 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
inline static void printOneChar(uint8_t character, uint8_t isMOSI) {
#if (HIDE_FF == 1)
	if (character == 0xff) {
		UART_printf("  ");
	} else {
		UART_printf("%02x", character);
	}
#elif (HIDE_FF == 2)
	if (character == 0xff) {
		if (isMOSI) {
			UART_printf("\e[34m%02x\e[33m", character);
		} else {
			UART_printf("\e[34m%02x\e[36m", character);
		}
	} else {
		UART_printf("%02x", character);
	}
#else
	UART_printf("%02x", character);
#endif

}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	volatile uint8_t state = HAL_GPIO_ReadPin(NSS_GPIO_Port, NSS_Pin);
	if (GPIO_Pin == NSS_Pin) {
		// NSS changed
		nssTrigger[displayPosition] |= 0x08 | (1 << state);
		// Store the final state
		if (state) {
			nssTrigger[displayPosition] |= 0x04;
		} else {
			nssTrigger[displayPosition] &= ~0x04;
		}
	} else if (GPIO_Pin == CLK_Pin) {
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
		// CLK asserted
		currentMISO = ((currentMISO << 1) & 0xff) | HAL_GPIO_ReadPin(MISO_GPIO_Port, MISO_Pin);
		currentMOSI = ((currentMOSI << 1) & 0xff) | HAL_GPIO_ReadPin(MOSI_GPIO_Port, MOSI_Pin);

		if (++bitsFilled == 8) {
			// An entire byte has been filled. Push it back on the display arrays
			miso[displayPosition] = currentMISO;
			mosi[displayPosition] = currentMOSI;
			displayPosition++;
			bitsFilled = 0;

			if (displayPosition >= STORESIZE) {
				// Back to the beginning
				displayPosition = 0;
				overSized = 1;
			}
		}
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
	} else {
		Error_Handler();
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  UART_printf("Welcome to SPI-Sniffer\r\n");

  // Reset the terminal formatting
  UART_printf("\e[0m");

  uint32_t lastTime = HAL_GetTick();
  uint32_t lastSeparatorTime = HAL_GetTick();
  uint8_t separatorShown = 0;

  uint8_t misoPrint[STORESIZE]; // Safety margin for memory allocation
  uint8_t mosiPrint[STORESIZE];
  uint8_t  nssPrint[STORESIZE];

  // Force a calculation
  HAL_GPIO_EXTI_Callback(NSS_Pin);
  for (int i = 0; i < 8; i++) HAL_GPIO_EXTI_Callback(CLK_Pin);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	if (displayPosition >= DISPLAYSIZE || HAL_GetTick() - lastTime >= 100) {
  		uint16_t oldDisplayPosition = displayPosition; // Make sure it doesn't change
  		// Copy the data so no one writes on it
  		memcpy(misoPrint, miso, oldDisplayPosition);
  		memcpy(mosiPrint, mosi, oldDisplayPosition);
  		memcpy(nssPrint, nssTrigger, oldDisplayPosition);
  		memset(nssTrigger, 0, oldDisplayPosition);

  		if (displayPosition != oldDisplayPosition) {
  			UART_printf("!!!COW\r\n"); // copy overwrite
  		}

  		displayPosition = 0;
  		if (oldDisplayPosition > 0) {
  			// Data exists
  			UART_printf("\e[33m");

				for (int i = 0; i < oldDisplayPosition; i++) {
					printOneChar(mosiPrint[i], 1);

					// NSS assertion
					if (nssPrint[i] & 0x08) {
						if (nssPrint[i] & 0x03) {
							if (nssPrint[i] & 0x04) {
								UART_printf("\e[31m\u292f\e[33m");
							} else {
								UART_printf("\e[32m\u2930\e[33m");
							}
						} else if (nssPrint[i] & 0x01) {
							UART_printf("\e[32m\u2924\e[33m");
						} else if (nssPrint[i] & 0x02) {
							UART_printf("\e[31m\u2925\e[33m");
						}
					} else {
						UART_printf(" ");
					}
				}
				UART_printf("\e[39m\r\n\e[36m");
				for (int i = 0; i < oldDisplayPosition; i++) {
					printOneChar(misoPrint[i], 0);

					// NSS assertion
					if (nssPrint[i] & 0x08) {
						if (nssPrint[i] & 0x03) {
							if (nssPrint[i] & 0x04) {
								UART_printf("\e[31m\u292f\e[36m");
							} else {
								UART_printf("\e[32m\u2930\e[36m");
							}
						} else if (nssPrint[i] & 0x01) {
							UART_printf("\e[32m\u2924\e[36m");
						} else if (nssPrint[i] & 0x02) {
							UART_printf("\e[31m\u2925\e[36m");
						}
						nssPrint[i] = 0; // Reset for next packet
					} else {
						UART_printf(" ");
					}

				}
				UART_printf("\e[39m\r\n");

				separatorShown = 0;
				lastSeparatorTime = HAL_GetTick();
  		}

  		if (HAL_GetTick() - lastTime >= 100){
  			if (oldDisplayPosition > 0) {
  				UART_printf("\r\n"); // Print a newline to separate old data
  			}
  			if (bitsFilled != 0) {
  				UART_printf("!!!IR\r\n");
  				bitsFilled = 0;
  			}
  		}
  		if (resetCaused) {
  			UART_printf("!!!RR\r\n");
  			resetCaused = 0;
  		}
  		if (overSized) {
  			UART_printf("!!!OS\r\n");
  			overSized = 0;
  		}

  		if (!separatorShown && HAL_GetTick() - lastSeparatorTime >= 1000) {
  			// A lot of time has passed. Print a separator.
				UART_printf("\e[1;39m====================================================================================\e[0;39m\r\n\r\n");
				separatorShown = 1;
  		}

			lastTime = HAL_GetTick();

			__WFI(); // Power saving
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 2000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOSI_Pin MISO_Pin */
  GPIO_InitStruct.Pin = MOSI_Pin|MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_Pin */
  GPIO_InitStruct.Pin = CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NSS_Pin */
  GPIO_InitStruct.Pin = NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NSS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
