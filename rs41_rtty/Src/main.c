
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
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>

#define DIT dit();
#define DAH dah();
#define DIT_T 50
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void TX_BUS_Init(void);
void tx_letter(char zeichen);
void tx_string_morse(char const string[]);
void dit(void);
void dah(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char uart_message[20], tx_string[100];
	uint8_t spi2_data[15];
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
  MX_USART3_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	strcpy(uart_message, "\x0d\x0aHallo, ich bin eine Sonde.\x0d\x0a");
	HAL_UART_Transmit(&huart3, (uint8_t *) uart_message, strlen(uart_message), HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */
		HAL_GPIO_WritePin(GPIOB, LED_green_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, LED_red_Pin, GPIO_PIN_SET);
		
		//Configure the transmitter (Si4032)
		HAL_GPIO_WritePin(GPIOC, SS_RF_TX_Pin, GPIO_PIN_RESET);
		
		//Write to the register with the adress 0x6E (Si4032 auto increments the register adress when reading or writing)
		spi2_data[0] = (0x6E | 0x80);
		
		//Set the data rate (26400 bits/s --> 12 Symbols for 2200Hz and 22 Symbols for 1200Hz)
		//Data to be written to 0x6E
		//High byte
		spi2_data[1] = 0xD8;
		//Data to be written to 0x6F
		//Low byte
		spi2_data[2] = 0x45;
		//Data to be written to 0x70
		//Data rate prescaler (txdtrtscale = 1)
		spi2_data[3] = 0x2C;
		
		//Set modulation type
		//Data to be written to 0x71 (bin 00010001, SDI --> modulation data in, modulation --> FSK)
		spi2_data[4] = 0x11;
		
		//Set frequnecy deviation (+/- 2500Hz)
		//Data to be written to 0x72
		spi2_data[5] = 0x01;
		
		//Set transmit frequnecy (432.470MHz)
		//Frequency offset (+37.8125kHz)
		//Data to be written to 0x73
		spi2_data[6] = 0x79;
		//Data to be written to 0x74
		spi2_data[7] = 0x00;
		//Band select
		//Data to be written to 0x75
		spi2_data[8] = 0x20;
		//Carrier Frequnecy
		//Data to be written to 0x76
		//High Byte
		spi2_data[9] = 0xED;
		//Data to be written to 0x77
		//Low Byte
		spi2_data[10] = 0x10;
		
		HAL_SPI_Transmit(&hspi2, spi2_data, 11, HAL_MAX_DELAY);
		HAL_Delay(10);
		HAL_GPIO_WritePin(GPIOC, SS_RF_TX_Pin, GPIO_PIN_SET);
		
		HAL_Delay(1);
		
		//The Crystal Oscillator Load Capacitance should be set to
		//0 since an external TCXO is used.
		HAL_GPIO_WritePin(GPIOC, SS_RF_TX_Pin, GPIO_PIN_RESET);
		//Write to the register 0x09
		spi2_data[0] = (0x09 | 0x80);
		spi2_data[1] = 0x00;
		HAL_SPI_Transmit(&hspi2, spi2_data, 2, HAL_MAX_DELAY);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOC, SS_RF_TX_Pin, GPIO_PIN_SET);
		
		//TX
		HAL_GPIO_WritePin(GPIOC, SS_RF_TX_Pin, GPIO_PIN_RESET);
		//Write to the register 0x07
		spi2_data[0] = (0x07 | 0x80);
		spi2_data[1] = 0x09;
		HAL_SPI_Transmit(&hspi2, spi2_data, 2, HAL_MAX_DELAY);
		HAL_Delay(1);
		
		//Deinit SPI2
		HAL_SPI_DeInit(&hspi2);
		//Init the SPI2 Pins as standart Input/Output
		TX_BUS_Init();
		
		HAL_GPIO_WritePin(GPIOC, SS_RF_TX_Pin, GPIO_PIN_SET);
		
		while(1)
		{
		tx_string_morse("oe3tec testing my wx station");
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
		HAL_Delay(1000);
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SS_RF_TX_GPIO_Port, SS_RF_TX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_green_Pin|LED_red_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SS_RF_TX_Pin */
  GPIO_InitStruct.Pin = SS_RF_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SS_RF_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_green_Pin LED_red_Pin */
  GPIO_InitStruct.Pin = LED_green_Pin|LED_red_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//Configures the SPI2 Pins as standard Input/Output for direct modulation
//HAL_SPI_DeInit() should be called before calling this function
static void TX_BUS_Init(void)
{
	GPIO_InitTypeDef GPIO_Init_Struct;
	
	/*Configure MOSI pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	
	/*Configure MOSI pin : GPIO_PIN_15 */
	GPIO_Init_Struct.Pin = GPIO_PIN_15;
	GPIO_Init_Struct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_Init_Struct.Pull = GPIO_NOPULL;
	GPIO_Init_Struct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_Init_Struct);
	
	/*Configure MISO pin : GPIO_PIN_14 */
	GPIO_Init_Struct.Pin = GPIO_PIN_14;
	GPIO_Init_Struct.Mode = GPIO_MODE_INPUT;
	GPIO_Init_Struct.Pull = GPIO_NOPULL;
	GPIO_Init_Struct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_Init_Struct);
	
}
void tx_letter(char zeichen)
{
	switch(zeichen) {
		case 'a': DIT DAH break;
		case 'b': DAH DIT DIT break;
		case 'c': DAH DIT DAH DIT break;
		case 'd': DAH DIT DIT break;
		case 'e': DIT break;
		case 'f': DIT DIT DAH DIT break;
		case 'g': DAH DAH DIT break;
		case 'h': DIT DIT DIT DIT break;
		case 'i': DIT DIT break;
		case 'j': DIT DAH DAH DAH break;
		case 'k': DAH DIT DAH break;
		case 'l': DIT DAH DIT DIT break;
		case 'm': DAH DAH break;
		case 'n': DAH DIT break;
		case 'o': DAH DAH DAH break;
		case 'p': DIT DAH DAH DIT break;
		case 'q': DAH DAH DIT DAH break;
		case 'r': DIT DAH DIT break;
		case 's': DIT DIT DIT break;
		case 't': DAH break;
		case 'u': DIT DIT DAH break;
		case 'v': DIT DIT DAH break;
		case 'w': DIT DAH DAH break;
		case 'x': DAH DIT DIT DAH break;
		case 'y': DAH DIT DAH DAH break;
		case 'z': DAH DAH DIT DIT break;
		
		case '1': DIT DAH DAH DAH DAH break;
		case '2': DIT DIT DAH DAH DAH break;
		case '3': DIT DIT DIT DAH DAH break;
		case '4': DIT DIT DIT DIT DAH break;
		case '5': DIT DIT DIT DIT DIT break;
		case '6': DAH DIT DIT DIT DIT break;
		case '7': DAH DAH DIT DIT DIT break;
		case '8': DAH DAH DAH DIT DIT break;
		case '9': DAH DAH DAH DAH DIT break;
		case '0': DAH DAH DAH DAH DAH break;
		
		case ' ': HAL_Delay(DIT_T*6); break;
	}
}
void tx_string_morse(char const string[])
{
	int i=0;
	
	while(string[i] != 0x00)
	{
		tx_letter(string[i]);
		HAL_Delay(DIT_T*2);
		i++;
	}
}
void dit(void)
{
	//Red LED is on when the transmitter is on
	//TX on
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, LED_red_Pin, GPIO_PIN_RESET);
	HAL_Delay(DIT_T);
	//Tx off
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, LED_red_Pin, GPIO_PIN_SET);
	HAL_Delay(DIT_T);
}
void dah(void)
{
	//Red LED is on when the transmitter is on
	//TX on
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, LED_red_Pin, GPIO_PIN_RESET);
	HAL_Delay(DIT_T*3);
	//TX off
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, LED_red_Pin, GPIO_PIN_SET);
	HAL_Delay(DIT_T);
}
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
