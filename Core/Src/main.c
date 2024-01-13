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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_config.h"
#include "remote_rc.h"
#include "pid.h"
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
CAN_Feedback_t Can_Feedback[2];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

dataBuffer_t dataBuffer;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	DecodeData(rxBuffer,&dataBuffer);
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
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //UART_InitDMAReceive();
	HAL_UART_Receive_DMA(&huart3, (uint8_t *)rxBuffer, rxBufferLen);
	can_filter_init();
	Pid_init();
	 HAL_TIM_Base_Start_IT(&htim3);
	HAL_Delay(200);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
/*
	PID_ERR=target_Speed[0]-Can_Feedback[0].Speed;
	Interal+=PID_ERR;
	PID_OUT = Kp * PID_ERR + Ki * Interal + Kd * (Can_Feedback[0].Speed-Can_Feedback[1].Last_speed);
	if(PID_OUT>=25000){
		PID_OUT=25000;
	}
	else if(PID_OUT<=-25000){
		PID_OUT = -25000;
	}
		*/

		/*
	CAN_ctrl(PID_OUT,dataBuffer.Channel_2 * MOTOR_RATIO);
		HAL_Delay(1);*/
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
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


/*
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan==&hcan1)
	{
		CAN_RxHeaderTypeDef CAN_rx;
		uint8_t data_rx[8];
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&CAN_rx,data_rx);
		if(CAN_rx.StdId==0x205)
		{
			for(int i=FilterLength-1;i>0;i--)
			{
				Can_Feedback[0].Last_speed[i]=Can_Feedback[0].Last_speed[i-1];
			}
			Can_Feedback[0].Mechnical_angle=((uint16_t) data_rx[0] <<8) | (uint16_t)data_rx[1];
			Can_Feedback[0].Speed=((uint16_t) data_rx[2] <<8) | (uint16_t)data_rx[3];
			Can_Feedback[0].Last_speed[0]=Can_Feedback[0].Speed;
			Can_Feedback[0].Torque_current=((uint16_t) data_rx[4] <<8) | (uint16_t)data_rx[5];
			Can_Feedback[0].Temprature=data_rx[6];
		}
		else if (CAN_rx.StdId==0x206)
		{
			Can_Feedback[1].Last_speed[0]=Can_Feedback[1].Speed;
			Can_Feedback[1].Mechnical_angle=((uint16_t) data_rx[0] <<8) | (uint16_t)data_rx[1];
			Can_Feedback[1].Speed=((uint16_t) data_rx[2] <<8) | (uint16_t)data_rx[3];
			Can_Feedback[1].Torque_current=((uint16_t) data_rx[4] <<8) | (uint16_t)data_rx[5];
			Can_Feedback[1].Temprature=data_rx[6];
		}
	}
	return;
}
*/


/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim3)
	{
	target_Speed[0]= (float)dataBuffer.Channel_1/33*16;
	target_Speed[1]= (float)dataBuffer.Channel_2/33*16;
	target_Angle[0]= (float)dataBuffer.Channel_1*5+4096;
	uint16_t Current_Speed=0;
		
	for(int i=0;i<FilterLength-1;i++)  //filter
		{
			Current_Speed+=Can_Feedback[0].Last_speed[i]/FilterLength;
		}
		
	PID.ERR=target_Speed[0]-Current_Speed;
	Interal+=PID.ERR;
	PID.OUT = PID.Kp * (float)PID.ERR + PID.Ki * (float)Interal + PID.Kd * (PID.ERR-PID.Last_ERR);
	if(PID.OUT>=25000){
		PID.OUT=25000;
	}
	else if(PID.OUT<=-25000){
		PID.OUT = -25000;
	}
	PID.Last_ERR=PID.ERR;
	CAN_ctrl(PID.OUT,dataBuffer.Channel_2 * MOTOR_RATIO);
	}
}


*/

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
