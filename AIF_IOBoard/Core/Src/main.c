/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CANID_AMT21_WHEEL  0x012
#define CANID_AMT21_ANGLE  0x013
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
static HAL_StatusTypeDef amt21_read(UART_HandleTypeDef *huart, uint16_t *count, uint8_t cmd, uint32_t timeout_ms);
HAL_StatusTypeDef CAN1_Send_U16_StdId(uint16_t std_id, uint16_t value);
void callback_amt21_1(void);
void callback_amt21_2(void);
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  const int amt21_1_ms = 100;
  uint32_t time_amt21_1 = 0;

  const int amt21_2_ms = 100;
  uint32_t time_amt21_2 = 0;

  const int cycle_led_ms = 1000;
  uint32_t time_cycle_led = 0;



  printf("Boot\n");

  HAL_CAN_Start(&hcan1);

  //エンコー???��?��??��?��?set_zero
//  uint8_t tx_enc_reset[2];
//  tx_enc_reset[0] = tx_add | 0x02;
////  tx_enc_reset[1] = 0x75;//reset
//  tx_enc_reset[1] = 0x5e;//set_zero
//  HAL_UART_Transmit_IT(&huart1, tx_enc_reset, 2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	if(HAL_GetTick() - time_amt21_1 > amt21_1_ms){
  		time_amt21_1 = HAL_GetTick();
  		callback_amt21_1();
  	}
  	if(HAL_GetTick() - time_amt21_2 > amt21_2_ms){
  		time_amt21_2 = HAL_GetTick();
			callback_amt21_2();
  	}
  	if(HAL_GetTick() - time_cycle_led > cycle_led_ms){
  		time_cycle_led = HAL_GetTick();
  		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);//cycle_LED
  	}

//		HAL_Delay(10); // 少し?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?つ?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��??????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 256;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  int DataIdx;
  for(DataIdx=0; DataIdx<len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if (huart->Instance == USART1) {
//
//	} else if (huart->Instance == USART2) {
////			rx2_done = 1;
//	}
//}

static HAL_StatusTypeDef amt21_read(UART_HandleTypeDef *huart, uint16_t *count, uint8_t cmd, uint32_t timeout_ms){
	uint8_t buff[2];

	if (HAL_UART_Transmit(huart, (uint8_t*)&cmd, 1, timeout_ms) != HAL_OK) return HAL_ERROR;
	HAL_StatusTypeDef st = HAL_UART_Receive (huart, buff, 2, timeout_ms);
	if (st != HAL_OK){
		return HAL_ERROR;
	}


	const uint16_t observed_count_1 = (buff[0] | (buff[1] << 8));
	//チェ?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?クサ?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?
	bool binaryArray_1[16];
	for(int i = 0; i < 16; i++) binaryArray_1[i] = (0x01) & (observed_count_1 >> (i));

	if ((binaryArray_1[15] == !(binaryArray_1[13] ^ binaryArray_1[11] ^ binaryArray_1[9] ^ binaryArray_1[7] ^ binaryArray_1[5] ^ binaryArray_1[3] ^ binaryArray_1[1]))
		&& (binaryArray_1[14] == !(binaryArray_1[12] ^ binaryArray_1[10] ^ binaryArray_1[8] ^ binaryArray_1[6] ^ binaryArray_1[4] ^ binaryArray_1[2] ^ binaryArray_1[0])))
	{
		*count = observed_count_1 & 0x3FFF;

		//12bit解像度のエンコー?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?は位置をシフトする
//			count = count >> 2;
	}else{
		return HAL_ERROR;
	}
	return HAL_OK;
}

HAL_StatusTypeDef CAN1_Send_U16_StdId(uint16_t std_id, uint16_t value){
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
		return HAL_BUSY;
	}
	CAN_TxHeaderTypeDef txHeader;
	uint8_t txData[2];
	uint32_t txMailbox;

	// LSB -> MSB の順（あなたの元コードと同じ）
	txData[0] = (uint8_t)(value & 0x00FF);
	txData[1] = (uint8_t)((value >> 8) & 0x00FF);

	txHeader.StdId = (std_id & 0x7FF);   // 11bitにマスク
	txHeader.IDE   = CAN_ID_STD;
	txHeader.RTR   = CAN_RTR_DATA;
	txHeader.DLC   = 2;
	txHeader.TransmitGlobalTime = DISABLE; // FDCANでなくCANなら無視されることもあります

	return HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}

void callback_amt21_1(void){//14bit
	uint8_t tx_add_amt21_1 = 0x54;   // AMT21 position request
	uint16_t count_1 = 0;
	int16_t count_signed_1 = 0;
	//====================USART1====================
	if (amt21_read(&huart1, &count_1, tx_add_amt21_1, 10) != HAL_OK) return;


	printf("Position_1 = %u\r\n", count_1);

	//countを＋－に変換
	if(count_1 < 8192){
		count_signed_1 = count_1 * -1;
	}else{
		count_signed_1 = (count_1 - 16384) * -1;
	}
	printf("Position_Signd_1 = %d\r\n", count_signed_1);

	CAN1_Send_U16_StdId(CANID_AMT21_ANGLE, (uint16_t)count_signed_1);
}

void callback_amt21_2(void){//12bit
	uint8_t tx_add_amt21_2 = 0x54;   // AMT21 position request
	uint16_t count_2 = 0;
//	int16_t count_signed_2 = 0;
	//====================USART1====================
	if (amt21_read(&huart2, &count_2, tx_add_amt21_2, 10) != HAL_OK) return;

	count_2 = count_2 >> 2;//because 12bit model
	printf("Position_2 = %u\r\n", count_2);

	CAN1_Send_U16_StdId(CANID_AMT21_WHEEL, (uint16_t)count_2);
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
