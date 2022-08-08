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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

	#include "stdio.h"
	#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

	#define	BUFFERSIZE		8
	#define LOCAL_DELAY		400

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

	#define BIT_CHECK( byte , pos )		((byte) &   (1UL << (pos)))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

	char DataChar[0xFF];
	int cnt_i=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

	uint16_t BufferCmp(uint8_t *pBuffer1, uint8_t *pBuffer2, uint16_t BufferLength);
	void SendSPI(uint8_t *spi_buffer_u8, uint16_t spi_size_u16 );
	uint8_t InverseOrderInByte (uint8_t input);
	void LocalDelayUs ( uint32_t _delay_u32 );

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
  /* USER CODE BEGIN 2 */

	sprintf(DataChar,"\r\n\r\n\tSPI+DMA MASTER for VIY.UA\r\n" );
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

	#define DATE_as_int_str 	(__DATE__)
	#define TIME_as_int_str 	(__TIME__)
	sprintf(DataChar,"\tBuild: %s. Time: %s." , DATE_as_int_str , TIME_as_int_str ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

	sprintf(DataChar,"\r\n\tfor debug: UART1 115200/8-N-1\r\n" ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, SET);
	HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, SET);
	HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	uint8_t aTxBuffer[BUFFERSIZE] = "SPI_DMA7" ;
	uint8_t aRxBuffer[BUFFERSIZE] = "SPI_DMA7" ;

	sprintf(DataChar,"1Tx: " ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	snprintf(DataChar, BUFFERSIZE + 1 , "%s", (char*)aTxBuffer ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	sprintf(DataChar,"\r\n" ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

	sprintf(DataChar,"1Rx: " ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	snprintf(DataChar, BUFFERSIZE + 1 , "%s", (char*)aRxBuffer ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	sprintf(DataChar,"\r\n" ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

	while (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) != GPIO_PIN_RESET) {
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		sprintf(DataChar,"  press the button %d \r" , cnt_i++ ) ;
		HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
		HAL_Delay(100);
	}
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
	sprintf(DataChar,"\r\nButton pressed.\r\n" ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

	sprintf(DataChar,"SendSPI\r\n" ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, RESET);

	SendSPI( aTxBuffer, BUFFERSIZE ) ;

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
	sprintf(DataChar,"\r\nMaster Transfer Completed.\r\n" ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

	sprintf(DataChar,"Finaly HAL_Delay(1000)\r\n\r\n" ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	HAL_Delay(1000);

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

/* USER CODE BEGIN 4 */

uint16_t BufferCmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength) {
	BufferLength++;
	while (BufferLength--) {
		if((*pBuffer1) != *pBuffer2) {
			return BufferLength;
		}
			pBuffer1++;
			pBuffer2++;
	}
	return 0;
}
//-------------------------------------------------------

void SendSPI(uint8_t *spi_buffer_u8, uint16_t spi_size_u16 ) {
	HAL_GPIO_WritePin( NSS_GPIO_Port, NSS_Pin, RESET );
	LocalDelayUs(LOCAL_DELAY);

	HAL_GPIO_WritePin (MOSI_GPIO_Port, MOSI_Pin, RESET );
	LocalDelayUs(LOCAL_DELAY);
	for	(int byte = 0; byte < spi_size_u16; byte++) {
//		if (byte == 4) { byte = 6; }
		uint8_t byte_to_Send = InverseOrderInByte( spi_buffer_u8[byte] );
		for ( int bit=0; bit<8; bit++) {
			HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, RESET);
			uint8_t mosi_bit = BIT_CHECK( byte_to_Send, bit );
			if (mosi_bit == 0 ) {
				HAL_GPIO_WritePin (MOSI_GPIO_Port, MOSI_Pin, RESET );
			} else {
				HAL_GPIO_WritePin( MOSI_GPIO_Port, MOSI_Pin, SET );
			}
			LocalDelayUs(LOCAL_DELAY);
			HAL_GPIO_WritePin( CLK_GPIO_Port, CLK_Pin, SET );
			LocalDelayUs(LOCAL_DELAY);
		}
	}
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, SET);
}
//-------------------------------------------------------

uint8_t InverseOrderInByte (uint8_t input) {
    uint8_t var_u8 =((input & 0x01) << 7) |
					((input & 0x02) << 5) |
					((input & 0x04) << 3) |
					((input & 0x08) << 1) |
					((input & 0x10) >> 1) |
					((input & 0x20) >> 3) |
					((input & 0x40) >> 5) |
					((input & 0x80) >> 7) ;
    return var_u8 ;
}
/***************************************************************************************/

void LocalDelayUs ( uint32_t _delay_u32 ) {
	for ( ; _delay_u32 > 0; _delay_u32-- ) {
		__asm( "nop" ) ;
	}
}
/***************************************************************************************/

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
