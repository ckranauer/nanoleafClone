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
#include "math.h"
#include <string.h>


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch2;

UART_HandleTypeDef huart2;



/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint8_t data_received_flag = 0;
uint8_t uart_buffer_rx[4];  // max 12  255_255_255_100_
uint8_t color_number[3];		// R


uint8_t input_red=0;
uint8_t input_green=0;
uint8_t input_blue=0;
uint8_t input_brightness=0;

#define MAX_LED 60
#define USE_BRIGHTNESS 1



uint8_t LED_data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];	// for brightness

int datasentflag = 0;  // set once the DMA stopped

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_2);
	datasentflag = 1;  // the flag ensures that the DMA dont send another data while the first data is still transmitted
}

void Set_LED (int LEDnum, int Red, int Green, int Blue){
	LED_data[LEDnum][0] = LEDnum;
	LED_data[LEDnum][1] = Green;
	LED_data[LEDnum][2] = Red;
	LED_data[LEDnum][3] = Blue;
}

# define PI 3.14159265

void Set_Brightness (int brightness){
#if USE_BRIGHTNESS

	if (brightness > 45) brightness = 45;
	for(int i=0; i<MAX_LED; i++){
		LED_Mod[i][0] = LED_data[i][0];
		for(int j=1; j<4;j++){
			float angle = 90 - brightness; // in degrees
			angle = angle*PI / 180; // in rad
			LED_Mod[i][j] = (LED_data[i][j])/(tan(angle));
		}
	}

#endif
}

// compare the two scaling algorithm
void Set_Brightness2 (int brightness){
#if USE_BRIGHTNESS
	if (brightness > 100) brightness = 100;
	for(int i=0; i<MAX_LED; i++){
		for(int j=1; j<4;j++){
		    double scaledValue = (double)brightness/100;
		    double logValue = log10(1 + 9 * scaledValue) / log10(10);
			LED_Mod[i][j] = LED_data[i][j] * logValue;
		}
	}
#endif
}


//24
// 23

//uint16_t pwmData[24];
uint16_t pwmData[(24*MAX_LED)+50];


//uint16_t pwmData[24];

void send (int Green, int Red, int Blue){
	  uint32_t data = (Green<<16) | (Red<<8) | Blue;

	  for (int i = 23; i>=0; i--){
		  if(data&(1<<i)) pwmData[i] = 60;
		  else pwmData[i] = 30;
	  }

	  HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_2, (uint32_t *)pwmData, 24);
}


  // the extra 50 is to store the reset code

void WS2812B_Send(void){
	uint32_t indx = 0;
	uint32_t color;

	for(int i=0; i<MAX_LED; i++){
		color = ((LED_Mod[i][1]<<16) | (LED_Mod[i][2]<< 8) | (LED_Mod[i][3]));

		for(int i=23; i>=0; i--){
			if(color&(1<<i)){
				pwmData[indx] = 60;  	//  2/3 of 90  (datasheet timing)
			}
			else pwmData[indx] = 30;	//  1/3 of 90  (datasheet timing)
			indx++;
		}
	}
	for(int i=0; i<50; i++){  // from datasheet info  50 zeros -> reset code   50usec delay in the PWM output
		pwmData[indx] =0;
		indx++;
	}
	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_2, (uint32_t *)pwmData, indx);
	while(!datasentflag){};
	datasentflag = 0;
}






void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){


}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==USART2){
		data_received_flag = 1;
		//HAL_UART_Receive_IT(&huart2, buffer_rx, 10);

		// if yes we can do something   or  __NOP();
		// in my exampl i have to show the main loop that i received datas, i need a flag
	}

	// in case we have multiple usarts:
	//if(huart->Instance==USART1){
			// ez akkor kell ha van másik UART ról is megszakítás
	//}
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
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
 // HAL_SPI_Transmit(&hspi2, spi_buffer_tx, 9, 100);
  /*
  for(int i=0; i<8;i++){
	  setLED(i, 0, 0, 0);
  }
  WS2812_Send();*/
  //B R G
  //send(5, 255, 101);
  /*
  for(int i=0; i<30;i++){
	  Set_LED(i,255, 100,0);
  }
  for(int i=30; i<59;i++){
  	  Set_LED(i,255,0,100);
    }
    */
  //Set_LED(0,255, 0,0);
  //Set_LED(1,0, 255,0);
  //Set_LED(2,0, 0,255);

  //Set_LED(3, 46, 89, 128);
  //send(0, 0, 255);
  //send(255, 0, 0);
  //Set_Brightness(45);
  //WS2812B_Send();

  // i have to start the reception before transmittion ??  dont forget
  //HAL_UART_Receive_IT(&huart2, (uint8_t *)uart_buffer_rx, 10);
  //HAL_UART_Transmit_IT(&huart2, buffer_tx, 10);
  //HAL_UART_Receive_IT(&huart2, buffer_rx, 10);
  HAL_UART_Receive_IT(&huart2, (uint8_t *)uart_buffer_rx, 4);
  //HAL_UART_Transmit_IT(&huart2, buffer_tx, 10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(data_received_flag){
		  HAL_UART_Receive_IT(&huart2, (uint8_t *)uart_buffer_rx, 4);
		  input_red = uart_buffer_rx[0];
		  input_green = uart_buffer_rx[1];
		  input_blue = uart_buffer_rx[2];


		  //void byteToRGB();
		  for(int i=0; i<30;i++){
		  	  Set_LED(i,input_red, input_green, input_blue);
		  }
		  for(int i=30; i<59;i++){
		    Set_LED(i,input_red, input_green, input_blue);
		  }
		  Set_Brightness(uart_buffer_rx[3]);
		  WS2812B_Send();



		  //HAL_UART_Transmit_IT(&huart2, uart_buffer_rx, 15);

		  data_received_flag = 0;
	  }
	  /*
	   *

	  for(int i=0; i<8;i++){
	  	  setLED(i, 255, 0, 0);
	    }
	    WS2812_Send();
*/

	// Apres l'allume le unit, it need to open the last saved led pattern if there is one,
	 // if exist then open and send to LED, if not then do switch off and on all of the leds some diag light
	// Saving mo22re patterns
	// switching between patters with a button
	// saving a pattern with a long push
	// do a blink with the current pattern indicate that the pattern is saved
	 // set the SPI speed to 800kHz

	//HAL_SPI_Transmit(&hspi2, spi_buffer_tx, 10, 100);
	//HAL_UART_Transmit_IT(&huart2, buffer_tx, 10);
	  //HAL_SPI_Transmit(&hspi2, spi2_buffer_tx, 3, 100);

	 // updateLEDs(spi_buffer_tx, size);
	    /*
	if(data_received_flag == 1){
		for(int i = 0; i<10;i++){
			spi_buffer_tx[i] = uart_buffer_rx[i];
		}
		updateLEDs(spi_buffer_tx, 10);
		HAL_UART_Transmit_IT(&huart2, spi_buffer_tx, 10);
		//HAL_SPI_Transmit(&hspi2, spi_buffer_tx, 10, 100);
		//spi_buffer_tx[i] =
		//uart_buffer_rx
		//do something
		// update the LED output
		data_received_flag = 0;
	}
	*/
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 90-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
