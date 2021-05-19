/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define  UartHaberUzulugu 50
#define RxStart 0xAB
#define I2CDataUzunluk 8
#define DS3231_ADDRESS 0xD0
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


uint8_t UartHaberlesmeBaslat=0;


uint8_t DataTx[UartHaberUzulugu];

uint8_t DataRx[UartHaberUzulugu];


uint8_t DataRxDogrulama=0;

uint8_t UartParemetre=0;

uint8_t UartHaberlesmeAdrsi=0xAB;

uint32_t TimeCount=0;

uint8_t UartCount=0;

uint32_t RealTime=0;
uint8_t ChecksumKontrol=0;
uint8_t GelenChecksum=0;
uint8_t GelenDataDogru=0;



 

///////////////////////i2c////////////////
uint8_t status = 0;
uint8_t I2CAdres=0x82;
uint8_t I2CData[1];
uint8_t I2CDataTx[2]={0x010,0x05};

uint8_t SetEt=0;
uint8_t yil=0;
uint8_t ay=0;
uint8_t gun=0;
uint8_t hafta=0;
uint8_t saat=0;
uint8_t dakika=0;
uint8_t saniye=0;
uint8_t Sor=0;
uint8_t nerden_geldi=0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t decToBcd(int val)
{
  return (uint8_t)( (val/10*16) + (val%10) );
}
// Convert binary coded decimal to normal decimal numbers
int bcdToDec(uint8_t val)
{
  return (int)( (val/16*10) + (val%16) );
}

typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hour;
	uint8_t dayofweek;
	uint8_t dayofmonth;
	uint8_t month;
	uint8_t year;
} TIME;

TIME time;

// function to set time

void Set_Time (uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year)
{
	uint8_t set_time[7];
	set_time[0] = decToBcd(sec);
	set_time[1] = decToBcd(min);
	set_time[2] = decToBcd(hour);
	set_time[3] = decToBcd(dow);
	set_time[4] = decToBcd(dom);
	set_time[5] = decToBcd(month);
	set_time[6] = decToBcd(year);

	HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDRESS, 0x00, 1, set_time, 7, 1000);
}

void Get_Time (void)
{
	uint8_t get_time[7];
	HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDRESS, 0x00, 1, get_time, 7, 1000);
	time.seconds = bcdToDec(get_time[0]);
	time.minutes = bcdToDec(get_time[1]);
	time.hour = bcdToDec(get_time[2]);
	time.dayofweek = bcdToDec(get_time[3]);
	time.dayofmonth = bcdToDec(get_time[4]);
	time.month = bcdToDec(get_time[5]);
	time.year = bcdToDec(get_time[6]);
}

float Get_Temp (void)
{
	uint8_t temp[2];

	HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDRESS, 0x11, 1, temp, 2, 1000);
	return ((temp[0])+(temp[1]>>6)/4.0);
}

void force_temp_conv (void)
{
	uint8_t status=0;
	uint8_t control=0;
	HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDRESS, 0x0F, 1, &status, 1, 100);  // read status register
	if (!(status&0x04))
	{
		HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDRESS, 0x0E, 1, &control, 1, 100);  // read control register
		HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDRESS, 0x0E, 1, (uint8_t *)(control|(0x20)), 1, 100);
	}
}

float TEMP;
char buffer[15];

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
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
		HAL_UART_Receive_IT(&huart1, DataRx, UartHaberUzulugu);
	
	__HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
	HAL_TIM_Base_Start_IT(&htim7);	//50Ms
	DataTx[0]=UartHaberlesmeAdrsi;
	DataTx[1]=0x01;
	DataTx[UartHaberUzulugu-3]=0x10;
	DataTx[UartHaberUzulugu-4]=0x11;
	DataTx[UartHaberUzulugu-4]=0xF1;
  while (1)
  {
	  
			

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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 36;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_ENABLE;
  hi2c1.Init.OwnAddress2 = 40;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 14;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 59999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, Count_Pin|Tx_Pin|Rx_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TxStart_Pin */
  GPIO_InitStruct.Pin = TxStart_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(TxStart_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Count_Pin Tx_Pin Rx_Pin */
  GPIO_InitStruct.Pin = Count_Pin|Tx_Pin|Rx_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
 if ((hi2c->Instance->CR1 & I2C_CR1_PE) != I2C_CR1_PE)
{
nerden_geldi=1;
}
 else
{
nerden_geldi=2;
}

}
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
if ((hi2c->Instance->CR1 & I2C_CR1_PE) != I2C_CR1_PE)
{
nerden_geldi=3;
}
else
{
nerden_geldi=4;
}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)	//RPi
	{

	
		HAL_UART_Receive_IT(&huart1, DataRx, UartHaberUzulugu);
		HAL_GPIO_TogglePin(GPIOG,Rx_Pin);
		ChecksumKontrol=1;
		
//		if(DataRx[UartParemetre]==0xAB)UartParemetre++;
//		else
//		{
//		UartParemetre=0;
//		}
//		if(UartParemetre>=5)UartParemetre=0;
	}
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	
	//HAL_UART_Stop(&huart1);		// ?
	//dummy=huart1.Instance->RDR;
	//HAL_UART_Receive_DMA(&huart1,rx_buff,25);

  __HAL_UART_DISABLE_IT(huart, UART_IT_TC);
  __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);

  /* Check if a receive process is ongoing or not */
  if(huart->gState == HAL_UART_STATE_BUSY_TX_RX)
  {
    huart->gState = HAL_UART_STATE_BUSY_RX;
  }
  else
  {
    huart->gState = HAL_UART_STATE_READY;
  }
	//  HAL_UART_RxCpltCallback(huart);
}

unsigned checksum(uint8_t *buffer, int len)
{  
	  buffer[len-1]=0;
		for(int i=0;i<len-1;i++)
	{ 
		buffer[len-1]=buffer[i]+buffer[len-1];
	}
      return buffer[UartHaberUzulugu-1];
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	__HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
	
	status=HAL_I2C_Slave_Receive(&hi2c1,I2CData,2,100);
	if(I2CData[0]==4 ){
  HAL_I2C_Slave_Transmit(&hi2c1,I2CDataTx,2,10);
  I2CData[0]=0;
		status=2;
	}
//	HAL_I2C_Mem_Read(&hi2c1, 0x12, 0x03, 1, I2CData, 7, 1000);
if(SetEt==1)
{SetEt=0;
	 Set_Time(saniye, dakika, saat,hafta, gun, ay, yil);
}
	if(ChecksumKontrol==1)
	{
		ChecksumKontrol=0;
		
		 if(checksum(DataRx,UartHaberUzulugu) == DataRx[UartHaberUzulugu-1] && RxStart == DataRx[0] )
		 { 
			 GelenDataDogru=1;
		 }
	}
	
	
	 
	if(UartHaberlesmeBaslat==0&&TimeCount==5)
	{ 


	  DataTx[UartHaberUzulugu-1]=checksum(DataTx,UartHaberUzulugu);
		DataTx[UartHaberUzulugu-1]=DataTx[UartHaberUzulugu-1] & 0xFF;
		TimeCount=0;
		UartHaberlesmeBaslat=1;
		HAL_UART_Transmit_IT(&huart1,(uint8_t *)DataTx,UartHaberUzulugu);
		HAL_GPIO_TogglePin(GPIOG,Tx_Pin);
	}
	
	 if(HAL_GPIO_ReadPin(GPIOA,TxStart_Pin)==1)
 { DataTx[4]=0xF1;
	 HAL_UART_Transmit_IT(&huart1,(uint8_t *)DataTx,5);
	 UartHaberlesmeBaslat=0;
 }
	
	TimeCount++;
	if(TimeCount>20)
		{
		TimeCount=0;
 	  UartHaberlesmeBaslat=0;
			
 
	//	Get_Time();	
	}

	HAL_GPIO_TogglePin(GPIOG,Count_Pin);
	RealTime=TIM7->CNT;

}



// Convert binary coded decimal to normal decimal numbers

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
