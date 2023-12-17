/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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

#define Polling_Enable    1
#define Polling_Disable   0


unsigned char data_Tx[100]={1,0,0,0,0,0,0,0};
unsigned char *data_frame_Tx= data_Tx;

unsigned char data_Rx[100];
unsigned char *data_frame_Rx= data_Rx;
int x=5;
//unsigned char *data_frame = "abcdefghjklmnopq";
//static int Frames_counter;

uint32_t Rx_Id=0;
uint8_t Rx_dlc;

//Flags
int CAN_FINISHED = 0 ;
int Process_OnData_Finished=1;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
//CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//std_ID
//data frame
void CAN_Tx(uint32_t STD_ID, uint8_t DLC, uint8_t* payload,uint8_t POLLING_Enable )
{
	uint8_t No_Free_MailBoxes = 0 ;
	uint32_t pTxMailbox=0;
	CAN_TxHeaderTypeDef pHeader ;
	pHeader.DLC = DLC;
	pHeader.StdId = STD_ID;
	pHeader.IDE = CAN_ID_STD;
	pHeader.RTR = CAN_RTR_DATA;
	/*(++) Monitor the Tx mailboxes availability until at least one Tx
	                 mailbox is free, using HAL_CAN_GetTxMailboxesFreeLevel().*/
	No_Free_MailBoxes = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
	if(No_Free_MailBoxes)
	{
		//	 (++) Then request transmission of a message using
		//                 HAL_CAN_AddTxMessage().
		if (HAL_CAN_AddTxMessage(&hcan, &pHeader,payload,&pTxMailbox) != HAL_OK)
		{
			Error_Handler();
		};
		//wait untill mail box is transmitted
		//		(++) HAL_CAN_IsTxMessagePending() to check if a message is pending
		//		                 in a Tx mailbox.
		if(POLLING_Enable){
			while(HAL_CAN_IsTxMessagePending(&hcan,pTxMailbox));
		}

	}
}


void CAN_Filter(uint16_t std_filterID, uint16_t std_filterMASK)
{
	CAN_FilterTypeDef sFilterConfig;
	sFilterConfig.FilterMaskIdHigh= std_filterMASK<<5;
	sFilterConfig.FilterMaskIdLow=0x0000;
	sFilterConfig.FilterIdHigh=std_filterID<<5;
	sFilterConfig.FilterIdLow= 0x0000;
	sFilterConfig.FilterActivation=CAN_FILTER_ENABLE;
	sFilterConfig.FilterBank=0;
	sFilterConfig.FilterMode =CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterFIFOAssignment =CAN_FILTER_FIFO0;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.SlaveStartFilterBank =0;
	//	Configure the reception filters using the following configuration
	//	          functions:
	//	            (++) HAL_CAN_ConfigFilter()
	if (HAL_CAN_ConfigFilter(&hcan,&sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}

}



void CAN_Rx(uint32_t* STD_ID, uint8_t* DLC, uint8_t* payload,uint8_t POLLING_Enable )
{
	CAN_RxHeaderTypeDef pHeade;
	//	(#) Reception:
	//	   (++) Monitor reception of message using HAL_CAN_GetRxFifoFillLevel()
	//	             until at least one message is received.
	//	      (++) Then get the message using HAL_CAN_GetRxMessage().

	//wait unitll message recieved
	if(POLLING_Enable){
		while(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) == 0);
	}

	if(HAL_CAN_GetRxMessage(&hcan,CAN_RX_FIFO0,&pHeade,payload)!=HAL_OK){
		Error_Handler();
	}
	*STD_ID = pHeade.StdId;
	*DLC =pHeade.DLC;

	if(pHeade.DLC == 3)
	{
		x=8;
	}

}




//IRQ call backs

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	//data_frame += 8;
	//HAL_CAN_Stop(hcan);
	//	Frames_counter++;
	//
	//	if (Frames_counter == 40 )
	//	{
	//		CAN_FINISHED = 1;
	//		Frames_counter = 0;
	//		HAL_CAN_Stop(hcan);
	//	}else
	//	{
	//		data_frame += 8;
	//	}
	//Error_Handler();
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{

	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
	//	Frames_counter++;
	//
	//		if (Frames_counter == 344 )
	//		{
	//			CAN_FINISHED = 1;
	//			Frames_counter = 0;
	//			HAL_CAN_Stop(hcan);
	//		}else
	//		{
	//			data_frame+=8;
	//		}


}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	//CAN_Rx(&Rx_Id,&Rx_dlc,data_Rx,Polling_Disable);

}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	Error_Handler();
}
/* USER CODE END PV */


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//std_ID
//data frame








/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
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
	MX_CAN_Init();
	/* USER CODE BEGIN 2 */

	//configure The filter for Receiver
	CAN_Filter(0X3FF, 0x000);

	//interrupt notification, Activate Rx interrupt
	if(HAL_CAN_ActivateNotification(&hcan,  CAN_IT_RX_FIFO0_MSG_PENDING  ) != HAL_OK)
	{
		Error_Handler();
	};

	//start CAN
	if(HAL_CAN_Start(&hcan) != HAL_OK)
	{Error_Handler();}




	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{


		//CAN_Tx(0X3, 8, data_Tx,Polling_Enable);
		CAN_Rx(&Rx_Id,&Rx_dlc,data_Rx,Polling_Enable);
		//		if(data_Rx[0]==1)
		//			{
		//				x=20;
		//			}else if(data_Rx[0] == 2)
		//			{
		//				x=30;
		//
		//			}else if(data_Rx[0] == 3)
		//			{
		//				x=40;
		//			}
		//		if(data_Rx[0] ==1){
		//			x=1;
		//		}else if(data_Rx[0] ==0)
		//		{
		//			x=0;
		//		}
		//		data_frame_Tx +=8;
		//CAN_Rx(&Rx_Id,&Rx_dlc,data_frame_Rx,Polling_Enable);
		//		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
		//		CAN_Rx(&Rx_Id,&Rx_dlc,data_frame_Rx,Polling_Enable);
		//		if(data_Rx[0] == 90 ){
		//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
		//		}
		//data_frame_Rx++;
		//data_frame_Rx = data_frame_Rx + 8;
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);


		//		CAN_Tx(0X222, 8, data_frame_Tx,Polling_Enable);
		//		data_frame_Tx +=8;

		//		if(Process_OnData_Finished == 1) //this flag set by the algorithm of Parsing
		//		{
		//			//	if(HAL_CAN_Start(&hcan) != HAL_OK)
		//			//	{Error_Handler();}
		//			CAN_FINISHED = 0 ;
		//			CAN_Rx(&Rx_Id,&Rx_dlc,data_frame,Polling_Disable);
		//		}else if(Process_OnData_Finished == 0)
		//		{
		//			//HAL_CAN_Stop(&hcan);  or just do nothing
		//		}

		//CAN_Tx(0X3AB, 8, data_frame,Polling_Disable);
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

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void)
{

	/* USER CODE BEGIN CAN_Init 0 */

	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 8;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */

	/* USER CODE END CAN_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA1 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB1 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
