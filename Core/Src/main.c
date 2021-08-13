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
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "printf_SWO.h"

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
CAN_TxHeaderTypeDef pHeader;
//
HAL_StatusTypeDef status;
uint8_t aData[8];
uint8_t aData_motor_of[8];
uint8_t aData_motor_on[8];
CAN_RxHeaderTypeDef pHeader_rd;
uint8_t aData_rd[8];
uint32_t pTxMailbox;
CAN_FilterTypeDef sFilterConfig;
//
uint8_t Motor_ID;
//bool flag = 0;



//

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t TIM2_prev = 0; // предыдущ. значение
uint8_t pTxData[200];

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
  MX_TIM6_Init();
  MX_UART5_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	sprintf((char *)pTxData, "CAN_motor\r\n");
	status = HAL_UART_Transmit (&huart5, pTxData, strlen((char *)pTxData), 200);
	 status = HAL_TIM_Encoder_Start (&htim2, TIM_CHANNEL_ALL);
  if(HAL_CAN_Start(&hcan1) != HAL_OK) // включили CAN
  {
		sprintf((char *)pTxData, "HAL_CAN_start Err\r\n");
		status = HAL_UART_Transmit (&huart5, pTxData, strlen((char *)pTxData), 200);
		 status = HAL_TIM_Encoder_Start (&htim2, TIM_CHANNEL_ALL);

  }
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if(HAL_CAN_ConfigFilter (&hcan1, &sFilterConfig) != HAL_OK)
  {
	  printf("HAL_CAN_start_filter Err\r\n");
  }

  if(HAL_CAN_ActivateNotification (&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
	  printf("HAL_CAN_ActivateNotofication Err\r\n");
  }

  Motor_ID = 2;
  pHeader.DLC = 8;
  pHeader.StdId = 0x140 + Motor_ID;
  pHeader.IDE = 0; //Standard identifier.
  pHeader.RTR = 0; //Data frame


 status =  HAL_CAN_AddTxMessage (&hcan1, &pHeader, aData_motor_of, &pTxMailbox);
//  status = HAL_CAN_AddTxMessage (&hcan1, &pHeader, aData, &pTxMailbox);
aData_motor_on[0] = 0xA1;
aData_motor_on[1] = 0x00;
aData_motor_on[2] = 0x00;
aData_motor_on[3] = 0x00;
aData_motor_on[4] = 0xA;
aData_motor_on[5] = 0xA;
aData_motor_on[6] = 0x00;
aData_motor_on[7] = 0x00;

aData_motor_of[0] = 0x80;
aData_motor_of[1] = 0x00;
aData_motor_of[2] = 0x00;
aData_motor_of[3] = 0x00;
aData_motor_of[4] = 0x00;
aData_motor_of[5] = 0x00;
aData_motor_of[6] = 0x00;
aData_motor_of[7] = 0x00;

aData[0] = 0x88;
aData[1] = 0x00;
aData[2] = 0x00;
aData[3] = 0x00;
aData[4] = 0x00;
aData[5] = 0x00;
aData[6] = 0x00;
aData[7] = 0x00;
//    status_motor_of = HAL_CAN_AddTxMessage(&hcan1, &pHeader, aData_motor_of, &pTxMailbox);
status = HAL_TIM_Encoder_Start (&htim2, TIM_CHANNEL_ALL);
  //printf("HAL_CAN_AddTxMessage - status - %d\r\n",status);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
	  if(TIM2->CNT != TIM2_prev)
	  	  	  	 {
	  	  	  	  TIM2_prev = TIM2->CNT; // предыдущ. значение
	  	  	  	  	sprintf((char *)pTxData, "%ld\r\n", TIM2_prev);
	  	  	  		status = HAL_UART_Transmit (&huart5, pTxData, strlen((char *)pTxData), 200);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

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
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART5;
  PeriphClkInitStruct.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback (CAN_HandleTypeDef * hcan)
{
	uint8_t i;
	if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &pHeader_rd, aData_rd) != HAL_OK)
	{
		printf("HAL_CAN_GetRxMessage Error\r\n");

	}
	else
	{
		printf("%lx: ", pHeader_rd.StdId);
		for (i = 0; i <pHeader_rd.DLC; i++)
		{
			printf(" %x ", aData_rd[i]);

		}
		printf("\r\n");
	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_11)
	{
//	HAL_CAN_AddTxMessage (&hcan1, &pHeader, aData, &pTxMailbox);
	HAL_CAN_AddTxMessage (&hcan1, &pHeader, aData_motor_of, &pTxMailbox);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

	}
	else if (GPIO_Pin == GPIO_PIN_10)
	{
		HAL_CAN_AddTxMessage (&hcan1, &pHeader, aData_motor_on, &pTxMailbox);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
}

	else
	{
		__NOP();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
