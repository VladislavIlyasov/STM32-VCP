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
#include "usb_device.h"

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
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

  char Start_Menu_1[]= "\r\n STM32F103C8T6 \r\n v - version\r\n a - ADC start\r\n l - LED ON/OFF";
  char Led_Off[] = "\r\n LED OFF";
  char Led_On[] = "\r\n LED ON";

  char trans_str[64] = {0,};

  volatile uint16_t adc_2[2] = {0,}; // у нас два канала поэтому массив из двух элементов

  char str[16];
  volatile uint8_t flag = 0;
  // volatile uint8_t FlagInterADC = 0;
  // volatile uint8_t	FlagStartADC =0;
  uint16_t adc_buf[20];
  int btn_cnt =0;

  uint8_t Rec_Data[4]={0};   // 8
  uint32_t Rec_Len=0;
  uint32_t Sum_Len=0;

  short Ovf=0;



  uint32_t Rec_Stack[16]={0};  // 32 64

 int ReceiveFlag = 0;
 uint8_t BusyFlag = 0;
 short Ovf_Flag = 0;

 int i =0;


 char Output_Stack[256];
 int Output_Len[64];
 int Output_Len_Sum;

 int OutRead = 0;




//const char first[11] = __DATE__;
//const char second[8] = __TIME__;
 	   // char *third =  __DATE__  __TIME__;

        char str_V[23] = {0};
 	// int xx =  sprintf (str_temp, "\r\n %s %s", __DATE__ , __TIME__);

// extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void IfReceivedV(void);
void IfReceivedA(void);
void IfReceivedL(void);

void ADC_Result(void);
void StartMenu(void);
void ADC_Start(void);


extern uint8_t BusyCheck (void);
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

void ReceiveHandler(void);

void StackHandler(void);
void OutputHandler(void);

void OutputStack(char TransMes[], uint8_t Len);
//uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

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
  MX_USB_DEVICE_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

 // StartMenu();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //StartMenu();


  while(BusyFlag == USBD_BUSY){
      	  BusyFlag = BusyCheck();
      	 // if(BusyFlag == USBD_OK ){--i;}
        }

  CDC_Transmit_FS(Start_Menu_1,sizeof(Start_Menu_1));

  sprintf (str_V, "\r\n %s %s", __DATE__ , __TIME__);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(ReceiveFlag == 1){

			  StackHandler();
			  ReceiveFlag = 0;
		  }




		//  if(BusyFlag == USBD_OK && Sum_Len!=i ){
	      if(Sum_Len!=i ){
	    	  ReceiveHandler();
	    	  i++;
	    	//  if(BusyFlag == USBD_BUSY && i!=0){i--;}
	      }

	      while(BusyFlag == USBD_BUSY){
	         	  BusyFlag = BusyCheck();
	         	 // if(BusyFlag == USBD_OK ){--i;}
	           }
	      if(BusyFlag == USBD_OK && Output_Len_Sum!=0 ){
	    	  OutputHandler();
	      }


	      }


  }



  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 11999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

int ii = 0;
int iii = 0;
void StackHandler(){

	if(Rec_Len>4){Rec_Len =4;}  // 4

	/// sum len - zapisannie. i - obrabotannie

	if (Sum_Len + Rec_Len >= 16 && Ovf_Flag == 0 ){
		Ovf = Sum_Len;
		Ovf_Flag =1;
		iii = 0;
	}


	if(Ovf_Flag ==1&& (i>Rec_Len+ii || i==0)){ ///!!!
		// ii указатель перезаиси
		 iii =0;
		 ii = Sum_Len-Ovf;    // можно переписать на фор / +1



	      for(iii =0; iii< Rec_Len; iii++){
			Rec_Stack[ii]= Rec_Data[iii]; // тут не ии в рек дате а иии то есть 123
			ii++;
           // iii++;
		}


		Sum_Len += Rec_Len;
       // iii=0;


	}



	if (Sum_Len + Rec_Len < 16){

		Ovf_Flag =0; ///  в обработчик сброса оверфлова
        ii = 0;


      for(iii =0; iii< Rec_Len; iii++){
		Rec_Stack[Sum_Len+iii]=Rec_Data[iii];
      }
      Sum_Len += Rec_Len;
	}

}

int ooo =0;
void OutputStack(char TransMes[], uint8_t Len){


	if(Output_Len_Sum + Len < 256){


		  for(ooo =0; ooo< Len; ooo++){
					Output_Stack[Output_Len_Sum+ooo]=TransMes[ooo];
			      }
			      Output_Len_Sum += Len;
                //  ooo =0;



	//BusyFlag = CDC_Transmit_FS(TransMes,Len);


	}

}



void OutputHandler(){

	BusyFlag = CDC_Transmit_FS(Output_Stack,Output_Len_Sum);

	Output_Len_Sum = 0;
	}

void ReceiveHandler(){
  /// len istead of i
	// int i = 0;

	if (i == Ovf && i!=0){
			i=0;
			Sum_Len = Sum_Len - Ovf;


			Ovf_Flag =0; ///  в обработчик сброса оверфлова
			        ii = 0;
		}


	//  CDC_Transmit_FS(Rec_Len,sizeof(Rec_Len));


	if (Rec_Stack[i]=='l'){
				IfReceivedL();
			} else if (Rec_Stack[i]=='v'){
			IfReceivedV();
		} else if (Rec_Stack[i]=='a'){
				IfReceivedA();
			} else if (Rec_Stack[i]=='e'){
					StartMenu();
				}else if (Rec_Stack[i]>0 && Rec_Stack[i]<4099){
					StartMenu(); //&
				}
			else {
				StartMenu();

			}

//}


 //  Rec_Len =0;
 //  memset(Rec_Data,0,128);



}

void StartMenu(void){


	 OutputStack( Start_Menu_1,sizeof(Start_Menu_1) );

	//BusyFlag = CDC_Transmit_FS(Start_Menu_1,sizeof(Start_Menu_1));
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_TIM_Base_Stop_IT(&htim2);


}

void IfReceivedV(void){

	//BusyFlag = CDC_Transmit_FS("\r\n"__DATE__ " " __TIME__ ,sizeof("\r\n"__DATE__ " " __TIME__ ));

	//BusyFlag = CDC_Transmit_FS(str_V,23);

	OutputStack(str_V,23 );

}
void IfReceivedA(void){

	//HAL_ADC_Start_DMA(&hadc1, (uint16_t*) &adc_2, 2);

			//FlagStartADC =1;

	HAL_TIM_Base_Start_IT(&htim2);

}

void IfReceivedL(void){

	   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	   if( HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET ) {


		   //BusyFlag =   CDC_Transmit_FS("\r\n LED OFF",sizeof("\r\n LED OFF"));

		   OutputStack(Led_Off,10 );
	   }
	   else {

		   //BusyFlag =  CDC_Transmit_FS("\r\n LED ON",sizeof("\r\n LED ON"));
		   OutputStack(Led_On,9 );
	   }

}


int count = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if(hadc->Instance == ADC1 ){


			//HAL_TIM_Base_Start_IT(&htim2);
			//     HAL_ADC_Start_DMA(&hadc1, (uint16_t*)&adc_2, 2);


		HAL_ADC_Stop_DMA(&hadc1);

			adc_buf[count]= adc_2[0];
			adc_buf[count+1]= adc_2[1];

			count ++;
			count ++;

		if(count==20) {
			HAL_ADC_Stop_DMA(&hadc1); ADC_Result(); count =0;
		}


	 }
}
void ADC_Start(void){

	HAL_ADC_Start_DMA(&hadc1, (uint16_t*) &adc_2, 2);
}


void ADC_Result(void){

	HAL_ADC_Stop_DMA(&hadc1); // это необязательно
	HAL_TIM_Base_Stop_IT(&htim2);

	int AdcResult = 0;

	for (int a =0; a<20; a++){

		AdcResult += adc_buf[a];


	}

	AdcResult = AdcResult/20;

	 //  sprintf (str, "\r\n ADC %d %d ", adc_2[0], adc_2[1]);
	sprintf (str, "\r\n ADC %d ", AdcResult);
				  		   adc_2[0]=0;
				  		   adc_2[1]=0;



	 // BusyFlag = CDC_Transmit_FS(str,11);
	  OutputStack(str,11);

	 str[16]= " ";

	 HAL_TIM_Base_Start_IT(&htim2);
	  //	 HAL_ADC_Start_DMA(&hadc1, (uint16_t*)&adc_2, 2);
}


void button_state(void){
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);


while(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_SET){
	btn_cnt++;
	if( btn_cnt==12000){           /// TIME = 15 ms
		 while(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_SET){};
		 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

		 if( HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET )
		 	   {

			  // BusyFlag =   CDC_Transmit_FS("\r\n LED OFF",sizeof("\r\n LED OFF"));
			    OutputStack(Led_Off,10 );
		 	   }

		 else {
			   //BusyFlag =  CDC_Transmit_FS("\r\n LED ON",sizeof("\r\n LED ON"));
			   OutputStack(Led_On,9 );
		 	  }

		 	break;
			}


	}

     btn_cnt=0;
	 HAL_NVIC_EnableIRQ(EXTI0_IRQn);
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
