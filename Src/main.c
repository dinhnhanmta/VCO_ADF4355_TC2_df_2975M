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
#include <math.h>

#define         _CLK_HIGH									HAL_GPIO_WritePin(VCO_CLK_GPIO_Port,VCO_CLK_Pin,1)
#define 		_CLK_LOW									HAL_GPIO_WritePin(VCO_CLK_GPIO_Port,VCO_CLK_Pin,0)

#define         _LE_HIGH									HAL_GPIO_WritePin(VCO_LE_GPIO_Port,VCO_LE_Pin,1)
#define 		_LE_LOW									  HAL_GPIO_WritePin(VCO_LE_GPIO_Port,VCO_LE_Pin,0)

#define         _CE_HIGH									HAL_GPIO_WritePin(VCO_CE_GPIO_Port,VCO_CE_Pin,1)
#define 		_CE_LOW									  HAL_GPIO_WritePin(VCO_CE_GPIO_Port,VCO_CE_Pin,0)

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// for CAN
//Filter
#define	ID_IDE_Filter     			1   // Filter 29 bit
#define	ID_RTR_Filter     			1   // filter remote Msg
#define	Mask_IDE								1   // care
#define	Mask_RTR				  			1   // care

#define	ID_STD_Filter     			0  // filter 11 bit
#define	ID_DATA_Filter        	0  // filter data Msg
#define	NO_Mask_IDE				    	0  //dont care
#define	NO_Mask_RTR				    	0  //dont care

uint8_t RX_DATA[8] = {0,0,0,0,0,0,0,0};			// luu goi thong tin CAN_RX
/* cau truc 8 Bytes nhu sau:
[1,2,3]. 	Header: 3 Byte: "DAT"
[4].			phan loai lenh: 	'K': dieu khien cho cac kenh
														'F': dieu khien cho VCO tao thay doi tan so
[5]			  Chi so Kenh: int: [0..31]
[6] 			Pha_bin[8..1]
[7]				Pha_OPT[0]
[8]				ATT_Bin [6..0]
*/

uint8_t volatile CAN_FlAG = 0;


#define  MOD1  					16777216
#define  MOD2  					16383

#define  F_ref  					50  	// Mhz
#define  F_PFD  					100  // Mhz

float F_VCO;
float N, FRAC, ti_so_Frac2Mode2;
uint32_t FRAC1, FRAC2, INT_Value;



uint8_t ID_0,ID_1,ID_2,ID_3,ID_4;
uint8_t ID_BOARD = 0;
uint32_t ADDR_VCO_BOARD = 0;     //ADDR_VCO_BOARD = BASE_ADDR + ID_BOARD
#define		BASE_ADDR			100



// FOR VCO
float  Freq_out;
uint32_t volatile register_ADF4355[13] = {
     0x1050C, 					// R12
     0x81200B,
     0x60003EBA,
     0x34067F99,
     0x1A69A6B8,
     0x100002E7,
		 0x35248076,              // 76: +5dBm, 66: + 2 dBm, 56: -1dBm, 46: -4 dBm 
     0x800025,
     0x34008984,
     0x3,
     0x3FFF2,
     0x1,
     0x3A0 						// R0
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void CACULATOR_ADF4355(float F_out);						// tinhtoan va update du lieu moi vao thanh ghi
void CONFIG_REG_VCO_ADF4355(uint32_t REG_VALUE)	;							// thuc hien cau hinh cho ADF4355
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	CanRxMsgTypeDef					RX_Msg;
	CAN_FilterConfTypeDef  	RX_Msg_Filter;

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
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

// Lay ID Board: du lieu tu Switch
	// 
	ID_0 = HAL_GPIO_ReadPin(ID_Board_0_GPIO_Port,ID_Board_0_Pin);   
	ID_1 = HAL_GPIO_ReadPin(ID_Board_1_GPIO_Port,ID_Board_1_Pin);
	ID_2 = HAL_GPIO_ReadPin(ID_Board_2_GPIO_Port,ID_Board_2_Pin);			
	ID_3 = HAL_GPIO_ReadPin(ID_Board_3_GPIO_Port,ID_Board_3_Pin);
	ID_4 = HAL_GPIO_ReadPin(ID_Board_4_GPIO_Port,ID_Board_4_Pin);
	ID_BOARD = ID_0 | (ID_1 << 1 )|(ID_2 << 2 )|(ID_3 << 3 )|(ID_4 << 4 );

	ADDR_VCO_BOARD = BASE_ADDR + ID_BOARD;   // dia chi CAN cua board



// THIET LAP TRANG THAI DAU CHO vco
				_CE_HIGH;
				
				
				HAL_Delay(100);
				_LE_HIGH;
				_CLK_LOW;
				HAL_GPIO_WritePin(VCO_DATA_GPIO_Port,VCO_DATA_Pin,0);
				

// Thiet lap CAN Bus: RX_ONLY ..............................................
	// 1. cho phep chay trong khi Debug
	hcan.Instance->MCR = 0x60;   
	
	
	// 2.set up FILTERs for RX
	//RX_Msg_Filter.SlaveStartFilterBank = 14;    // chi danh cho cac MCU co 2 CAN tro len: stm32f4...
	RX_Msg_Filter.FilterNumber = 0;
	RX_Msg_Filter.BankNumber   = 14;  						// CAN2SB , CAN2 Start slaver bank: STM32f103: trong F103 do chi co 1 CAn nen tham so nay khong co y nghia
	RX_Msg_Filter.FilterMode   = CAN_FILTERMODE_IDMASK;//CAN_FILTERMODE_IDLIST ;//; CAN_FILTERMODE_IDMASK; ;
	
  
	// ###  Che do SCALER 32 bit cho cac Filter
	RX_Msg_Filter.FilterScale  = CAN_FILTERSCALE_32BIT;//CAN_FILTERSCALE_16BIT;
	//	ADDR: 29 bit
	// Khong quan tam 2 bit cuoi cung cua dia chi: de cho 4 kenh tren 1 board: 0x1FFFFFFC
	uint32_t Mask_Filter 						= (0x1FFFFFFC 			<< 3)| Mask_IDE |Mask_RTR;   // 1: bit do duoc so sanh, 0: bo qua khong duoc so sanh: loc ca kieu dia chi va kieu du lieu: data or remote
	uint32_t ID_Filter							= (ADDR_VCO_BOARD   << 3)| ID_IDE_Filter |ID_DATA_Filter;  // loc kieu 29 bit va loai DATA
	RX_Msg_Filter.FilterIdHigh  		= (ID_Filter &0xFFFF0000) >> 16; 		// 11 bit cao trong thanh ghi phan cao 16 bit: 
	RX_Msg_Filter.FilterIdLow				= (ID_Filter &0x0000FFFF) ;
	RX_Msg_Filter.FilterMaskIdHigh 	= (Mask_Filter &0xFFFF0000) >> 16 ;  // tai nhung bit == 1 se dc kiem tra va mask thi se nhan data vao fifo || chi co y nghia khi dung che do Mask.
	RX_Msg_Filter.FilterMaskIdLow  	= (Mask_Filter &0x0000FFFF) ;
	//
	// Nhan du lieu qua FIFO_0
	RX_Msg_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;				// luu vao fifo 0
	RX_Msg_Filter.FilterActivation = ENABLE;
	
	hcan.pRxMsg = &RX_Msg;   	// phai co lenh nay de tranh con tro co gia tri NULL gay loi ngat cung
	
 // Thuc hien cau hinh FILTER	
	 if (HAL_CAN_ConfigFilter(&hcan, &RX_Msg_Filter) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }
	
	// Cho phep ngat RX_FIFO_0
	hcan.Instance->IER |= (1<<1); //  cho phep ngat RX FIFO_0: 1<< 1 ; fifo_1: 1<<4

	
	
	HAL_GPIO_WritePin(LED_User_GPIO_Port,LED_User_Pin,1);		// bat
	HAL_Delay(1000);
	HAL_GPIO_WritePin(LED_User_GPIO_Port,LED_User_Pin,0);		// bat

	
	
	
	// cau hinh mac dinh
	CAN_FlAG = 1;
	Freq_out = 1160;    // tan so mac dinh
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */


		if (CAN_FlAG == 1)
		{
				CAN_FlAG = 0;
				CACULATOR_ADF4355(Freq_out);   										// tinh toan va update gia tri thanh ghi

					CONFIG_REG_VCO_ADF4355(register_ADF4355[0]);   //R12
					CONFIG_REG_VCO_ADF4355(register_ADF4355[1]);		// R11
					CONFIG_REG_VCO_ADF4355(register_ADF4355[2]);		//R10
			
					CONFIG_REG_VCO_ADF4355(register_ADF4355[12-4]); // R4
			
					CONFIG_REG_VCO_ADF4355(register_ADF4355[3]);  //R9
					CONFIG_REG_VCO_ADF4355(register_ADF4355[4]);
					CONFIG_REG_VCO_ADF4355(register_ADF4355[5]);
					CONFIG_REG_VCO_ADF4355(register_ADF4355[6]);
					CONFIG_REG_VCO_ADF4355(register_ADF4355[7]);
					CONFIG_REG_VCO_ADF4355(register_ADF4355[8]);
					CONFIG_REG_VCO_ADF4355(register_ADF4355[9]);
					CONFIG_REG_VCO_ADF4355(register_ADF4355[10]);
					CONFIG_REG_VCO_ADF4355(register_ADF4355[11]);   //R1

					HAL_Delay(1);   // wait for ADC set: it nhat la 0.204 (ms)
					CONFIG_REG_VCO_ADF4355(register_ADF4355[12] | (1 << 21));		//R0   : auto cal = enable
					
					CONFIG_REG_VCO_ADF4355(register_ADF4355[8]);    // R4
					CONFIG_REG_VCO_ADF4355(register_ADF4355[10]);		// R2
					CONFIG_REG_VCO_ADF4355(register_ADF4355[11]);		// R1
					CONFIG_REG_VCO_ADF4355(register_ADF4355[12]);		// R0  : : auto cal = Disable
					
				// dua ve trang thai mac dinh
				_LE_HIGH;
				_CLK_LOW;
				HAL_GPIO_WritePin(VCO_DATA_GPIO_Port,VCO_DATA_Pin,0);
			// LED bao
			 HAL_GPIO_WritePin(LED_User_GPIO_Port,LED_User_Pin,1);		// bat
			 HAL_Delay(500);
			 HAL_GPIO_WritePin(LED_User_GPIO_Port,LED_User_Pin,0);		// tat		
			
		}
		
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
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

/* CAN init function */
static void MX_CAN_Init(void)
{

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_3TQ;
  hcan.Init.BS2 = CAN_BS2_5TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = ENABLE;
  hcan.Init.AWUM = ENABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_User_GPIO_Port, LED_User_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VCO_CLK_Pin|VCO_DATA_Pin|VCO_LE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VCO_CE_GPIO_Port, VCO_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_User_Pin */
  GPIO_InitStruct.Pin = LED_User_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_User_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ID_Board_4_Pin ID_Board_3_Pin ID_Board_2_Pin ID_Board_1_Pin */
  GPIO_InitStruct.Pin = ID_Board_4_Pin|ID_Board_3_Pin|ID_Board_2_Pin|ID_Board_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ID_Board_0_Pin */
  GPIO_InitStruct.Pin = ID_Board_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ID_Board_0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : VCO_CLK_Pin VCO_DATA_Pin VCO_LE_Pin */
  GPIO_InitStruct.Pin = VCO_CLK_Pin|VCO_DATA_Pin|VCO_LE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VCO_CE_Pin */
  GPIO_InitStruct.Pin = VCO_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(VCO_CE_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan) 
{
	uint32_t phan_thphan = 0;
	
	CAN_FlAG = 0;
	RX_DATA[0] = hcan->pRxMsg->Data[0];
	RX_DATA[1] = hcan->pRxMsg->Data[1];
	RX_DATA[2] = hcan->pRxMsg->Data[2];
	RX_DATA[3] = hcan->pRxMsg->Data[3];
	
	RX_DATA[4] = hcan->pRxMsg->Data[4];
	RX_DATA[5] = hcan->pRxMsg->Data[5];
	RX_DATA[6] = hcan->pRxMsg->Data[6];
	RX_DATA[7] = hcan->pRxMsg->Data[7];
	// kiem tra Header
	if ((RX_DATA[0] =='D')&&(RX_DATA[1] =='A')&& (RX_DATA[2] =='T')&&(RX_DATA[3] =='F'))
	{
		CAN_FlAG = 1;
		// gan gia tri tuong ung .............
		// gai tri tan so can thiet lap duoc luu trong 4 byte: 2 bytes phan nguyen, 2 bytes phan thap phan
		//  "[4][5]. [6][7]"
		Freq_out = (RX_DATA[4] << 8) + RX_DATA[5];   // lay phan nguyen
		phan_thphan = (RX_DATA[6] << 8) + RX_DATA[7]; // lay phan thap phan
		if (phan_thphan < 10) Freq_out = Freq_out + 0.1 * phan_thphan;
		else if (phan_thphan < 100) Freq_out = Freq_out + 0.01 * phan_thphan;
				 else if (phan_thphan < 1000) Freq_out = Freq_out + 0.001 * phan_thphan;
							else if (phan_thphan < 10000) Freq_out = Freq_out + 0.0001 * phan_thphan;
										else if (phan_thphan < 100000) Freq_out = Freq_out + 0.00001 * phan_thphan;
		
	}
	hcan->Instance->RF0R |= (1 <<5);   	// giai phong mail khoi FIFO
	hcan->Instance->IER 	= (1<<1);  			// cho phep tiep tuc ngat
}




// FOR ADF4355
void CACULATOR_ADF4355(float F_out)						// tinhtoan va update du lieu moi vao thanh ghi
{
// thuc hien tinh toan va update du lieu vao thanh ghi
uint32_t	RF_dev_Sel = 0;
uint32_t	N_Dev_out = 1;
	F_VCO = N_Dev_out *F_out;
while (F_VCO < 3300.0) 
	{
		N_Dev_out = 2* N_Dev_out;
		F_VCO = N_Dev_out* F_out;
		RF_dev_Sel = RF_dev_Sel + 1;
	}
	
	
	
	// tinh toan cac tham so khac
	N 				= F_VCO/F_PFD; 
	INT_Value = floor(N);
	FRAC			=	N - INT_Value;	
	FRAC1 		=	floor(FRAC*	MOD1);
	
	ti_so_Frac2Mode2 = FRAC*	MOD1 - FRAC1;
	FRAC2 		= round (ti_so_Frac2Mode2 * MOD2);
	
	
	// thiet lap he so chia RF out
	register_ADF4355[12-6] = (register_ADF4355[12-6] & 0xFF1FFFFF) | ((RF_dev_Sel & 0x07) << 21);
	
	// he so chia VCO
	register_ADF4355[12-0] = (INT_Value << 4) + 0;   							//R0
	register_ADF4355[12-1] = (FRAC1 << 4) 		+ 1;   							//R1
	register_ADF4355[12-2] = (FRAC2 << 18) + (MOD2 << 4)  + 2 ;   //R2
	
}


void CONFIG_REG_VCO_ADF4355(uint32_t REG_VALUE)							// thuc hien cau hinh cho ADF4355
{
    uint32_t i = 0;
    _LE_LOW;
		HAL_Delay(1);
    for( i = 0; i < 32; i++)    
		{ 
			HAL_GPIO_WritePin(VCO_DATA_GPIO_Port,VCO_DATA_Pin,(REG_VALUE >>(31-i)) & 1);
			HAL_Delay(1);
			_CLK_HIGH;
			HAL_Delay(1);
			_CLK_LOW;
			HAL_Delay(1);
    }
    _LE_HIGH;
		HAL_Delay(1);
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
