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
#include "../LogicH/CircBuff.h"
#include "../LogicH/CircBuff.h"

#include "InitPeriphery.h"
#include "Func.h"
#include "Stack.h"
//#include "CircBuff.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define max(x, y) ( (x) > (y) ? (x) : (y) )
#define min(x, y) ( (x) < (y) ? (x) : (y) )
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */



// char str_V[23] = {0};

  char trans_str[64] = {0,};

 // volatile uint16_t adc_2[2] = {0,}; // у нас два канала поэтому массив из двух элементов



 // char str[16];



  volatile uint8_t flag = 0;
  // volatile uint8_t FlagInterADC = 0;
  // volatile uint8_t	FlagStartADC =0;


  // uint16_t adc_buf[20];




 // int btn_cnt =0;

  uint8_t Rec_Data[4]={0};   // 8
  uint32_t Rec_Len=0;



  //uint32_t Sum_Len=0;

  short Ovf=0;



  uint32_t Rec_Stack[16]={0};  // 32 64

 int ReceiveFlag = 0;
 uint8_t BusyFlag = 0;


 int i =0;



 int OutRead = 0;




//const char first[11] = __DATE__;
//const char second[8] = __TIME__;
 	   // char *third =  __DATE__  __TIME__;


 	// int xx =  sprintf (str_temp, "\r\n %s %s", __DATE__ , __TIME__);

// extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_DMA_Init(void);
//static void MX_ADC1_Init(void);
//static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

// void IfReceivedA(void);
//void IfReceivedL(void);

//void ADC_Result(void);

// void StartMenu(void);

// void ADCstop(void);
// void ADC_Start(void);


extern uint8_t BusyCheck (void);
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

extern uint8_t  USBD_CDC_EP0_RxReady(USBD_HandleTypeDef *pdev);

void ReceiveHandler(void);

void StackHandler(uint8_t* Data, uint32_t Len);

int OutputHandler(void);

void OutputStack(char TransMes[], uint8_t Len);
//uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */







extern struct queue queueIn, queueOut;











/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	queueIn.buffer      = malloc(128);
		queueIn.buffer_size = 128;
		queueIn.head        = 0;
		queueIn.tail        = 0;
		queueIn.bytes_avail = 0;

		queueOut.buffer      = malloc(256);
		queueOut.buffer_size = 256;
		queueOut.head        = 0;
		queueOut.tail        = 0;
		queueOut.bytes_avail = 0;

	//put(&queue, "hello ", 6);
	//put(&queue, "world\n", 7);

	char s[13];

	//get(&queue, (uint8_t *) s, 13);



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
  MX_ADC1_Init(&hadc1);
  MX_TIM2_Init(&htim2);
  /* USER CODE BEGIN 2 */

 // StartMenu();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //StartMenu();





 // StartMenu();

 // OutputStack( Start_Menu_1,sizeof(Start_Menu_1) );  // first line is not displayed


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//  if(ReceiveFlag == 1){

			 // StackHandler();
		//	  ReceiveFlag = 0;
		//  }




		//  if(BusyFlag == USBD_OK && Sum_Len!=i ){
	      if(queueIn.tail != queueIn.head ){
	    	  ReceiveHandler();
	    	  i++;
	    	//  if(BusyFlag == USBD_BUSY && i!=0){i--;}
	      }


	     // if(BusyFlag == USBD_OK && queueOut.bytes_avail!=0 ){
	      if(queueOut.bytes_avail!=0 ){
	    	  OutputHandler();
	      }


	      }

  }



  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */


/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */


/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */


/**
  * Enable DMA controller clock
  */

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */


/* USER CODE BEGIN 4 */

short OvfIn;
void StackHandler(uint8_t* Data, uint32_t Len){

	if(Len>4){Len =4;}

	OvfIn = put(&queueIn, Data, Len);

     if(OvfIn==0){

    	 while(BusyFlag != USBD_OK){
    	      BusyFlag = BusyCheck();
    	      if(BusyFlag == USBD_OK)
    	      {BusyFlag = CDC_Transmit_FS("\r\n Input Stack Overflow",23);
    	         break;}
    	   }

     }


}




char Out[8];
int OutputHandler(){
    int Cond =0;
    BusyFlag = USBD_BUSY;
     	 Cond = get(&queueOut,(uint8_t *) Out, 8);// нужна переменная  8!! check if you have 8
      if (Cond == 0){
    	  short LenOfMess = queueOut.bytes_avail;
    	  get(&queueOut,(uint8_t *) Out,LenOfMess);


    	  /// busy check here
    	  while(BusyFlag != USBD_OK){
    	  	         	  BusyFlag = BusyCheck();
                  if(BusyFlag == USBD_OK){BusyFlag = CDC_Transmit_FS(Out,LenOfMess);
                  break;}
    	  	           }
    	 // BusyFlag = CDC_Transmit_FS(Out,LenOfMess);// bytes avail =0
    	  return 0;
      }

      while(BusyFlag != USBD_OK){
         	  	         	  BusyFlag = BusyCheck();
         if(BusyFlag == USBD_OK){BusyFlag = CDC_Transmit_FS(Out,8);
         break;}
         	  	           }
	//BusyFlag = CDC_Transmit_FS(Out,8);



	return 1;
	}

char InputSymb[2];
void ReceiveHandler(){



	//  CDC_Transmit_FS(Rec_Len,sizeof(Rec_Len));

	get(&queueIn, (uint8_t *) InputSymb, 1);



	switch( InputSymb[0] )
	{
	    case 'l':{
	    	IfReceivedL();
	    	break;
	    }
	    case 'v':{
	    	IfReceivedV();
	    	break;
	    }
	    case 'a':{
	    	IfReceivedA(&htim2);
	    	break;
	    }
	    case 'e':{
	    		ADCstop(&hadc1, &htim2);
	  	    	StartMenu();
	  	    	break;
	  	    }
	    default :
	    	StartMenu();
	}



}






int OvfOut;
void OutputStack(char TransMes[], uint8_t Len){

        // ovf
		OvfOut =put(&queueOut, TransMes, Len);
		if (OvfOut==0)
		{
			queueIn.bytes_avail++;
			queueIn.head = (queueIn.head--) % queueIn.buffer_size;
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
