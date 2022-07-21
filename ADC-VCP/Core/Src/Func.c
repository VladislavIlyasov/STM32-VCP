 #include "main.h"
 // #include "Func.h"

#include <stdio.h>

char Start_Menu_1[]= "\r\n STM32F103C8T6 \r\n v - version\r\n a - ADC start\r\n l - LED ON/OFF";


void StartMenu(void){


	 OutputStack( Start_Menu_1,sizeof(Start_Menu_1) );

	//BusyFlag = CDC_Transmit_FS(Start_Menu_1,sizeof(Start_Menu_1));


}


void IfReceivedV(void){


	char str_V[23] = {0};
	sprintf (str_V, "\r\n %s %s", __DATE__ , __TIME__);
	//BusyFlag = CDC_Transmit_FS("\r\n"__DATE__ " " __TIME__ ,sizeof("\r\n"__DATE__ " " __TIME__ ));

	//BusyFlag = CDC_Transmit_FS(str_V,23);

	OutputStack(str_V,23 );

}


void IfReceivedA(TIM_HandleTypeDef* htim2){

	//HAL_ADC_Start_DMA(&hadc1, (uint16_t*) &adc_2, 2);

			//FlagStartADC =1;

	HAL_TIM_Base_Start_IT(htim2);

}



void IfReceivedL(void){
	  char Led_Off[] = "\r\n LED OFF";
	  char Led_On[] = "\r\n LED ON";

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





void button_state(void){
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	 int btn_cnt =0;

	 char Led_Off[] = "\r\n LED OFF";
	 	  char Led_On[] = "\r\n LED ON";

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




