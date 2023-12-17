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

#define Polling_Enable    1
#define Polling_Disable   0

char data_Tx[25]={0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,'i','B','Y','D','E','A','D','B','F'};
char *data_frame= &data_Tx;

char data_Tx_mo[25]={0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
char *data_frame_mo= &data_Tx_mo;

char *Calc_ptr = &data_Tx;
int current_record_size;
char volatile NumOf_currRecord_Frames;
char volatile RemandingBytes_forCurrRecord;

char Rx_data[100];
char *data_Tx_ptr= &Rx_data;
static int Frames_counter;

//Flags
int CAN_FINISHED = 0 ;
int Process_OnData_Finished=1;
int UART_Received_Finished =1;
int CAN_Awake = 1;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define Polling_Enable    1
#define Polling_Disable   0


uint16_t Rx_Id=0;
uint16_t Rx_dlc;
int i_Tx_Data=0;
static int Frames_counter;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define Polling_Enable    1
#define Polling_Disable   0



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */
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
		while(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_FILTER_FIFO0) == 0);
	}

	if(HAL_CAN_GetRxMessage(&hcan,CAN_RX_FIFO0,&pHeade,payload)!=HAL_OK){
		Error_Handler();
	}
	*STD_ID = pHeade.StdId;
	*DLC =pHeade.DLC;

}




void Calculate_RecodFrames()
{
	for(int i=0; i<25 ;i++)
	{
		current_record_size++;
		Calc_ptr++;

		if(*Calc_ptr == 'F' )
		{
			if(*(--Calc_ptr) == 'B')
			{
				if(*(--Calc_ptr) == 'D')
				{
					if(*(--Calc_ptr) == 'A')
					{
						if(*(--Calc_ptr) == 'E')
						{
							if(*(--Calc_ptr) == 'D')
							{
								current_record_size -= 5;
								NumOf_currRecord_Frames = current_record_size / 8;
								RemandingBytes_forCurrRecord = current_record_size % 8;
								break;

							}
						}
					}

				}

			}

		}
	}
}

void CAN_Tx_CallBack(CAN_HandleTypeDef *hcan)
{
	//HAL_CAN_Stop(hcan);
		Frames_counter++;
//		if(Frames_counter == 50)
//		{
//			HAL_CAN_Stop(hcan);
//		}else{
//			data_frame ++;
//		}
		if (Frames_counter == NumOf_currRecord_Frames )
		{
			if(RemandingBytes_forCurrRecord > 0)
			{

				CAN_Tx(0X30, RemandingBytes_forCurrRecord,data_frame,Polling_Disable);
				Frames_counter = 0;
				HAL_CAN_Stop(hcan);
			}
			CAN_FINISHED = 1;
			Frames_counter = 0;
			//HAL_CAN_Stop(hcan);
			CAN_Awake = 0 ;
			//HAL_CAN_RequestSleep(hcan);
			HAL_CAN_Stop(hcan);
		}else
		{
			data_frame +=8;
		}
}




//IRQ call backs

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{

	//test
	//data_frame +=8;
	//New version  (record version)
	CAN_Tx_CallBack(hcan);

	/* Old version
	Frames_counter++;

	if (Frames_counter == 344 )
	{
		CAN_FINISHED = 1;
		Frames_counter = 0;
		//HAL_CAN_Stop(hcan);
		CAN_Awake = 0 ;
		HAL_CAN_RequestSleep(hcan);
	}else
	{
		i = i+8;
		data_frame = &data_Tx[i] ;
	}
	*/
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
	Error_Handler();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	//CAN_Rx(&Rx_Id, &Rx_dlc, data_Tx_ptr, 0);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	Error_Handler();
}
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
  CAN_Filter(0x100, 0x000);
	//Calculate_RecodFrames();
  	//interrupt notification
//  	if(HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING ) != HAL_OK)
//  	{
//  		Error_Handler();
//  	};


  	//start CAN
  	if(HAL_CAN_Start(&hcan) != HAL_OK)
  	{Error_Handler();}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  CAN_Tx(0X003, 8,data_frame_mo,Polling_Enable);

	  //CAN_Rx(&Rx_Id, &Rx_dlc, data_Tx_ptr, Polling_Enable);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hcan.Init.Prescaler = 1;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
