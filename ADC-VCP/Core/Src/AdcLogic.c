#include "main.h"

 uint16_t adc_buf[20];
 volatile uint16_t adc_2[2] = {0,}; // у нас два канала поэтому массив из двух элементов
 char str[16];
 int count = 0;

//  ADC_HandleTypeDef hadc1;
// TIM_HandleTypeDef* htim2;



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1) {
	if(hadc1->Instance == ADC1 ){


			//HAL_TIM_Base_Start_IT(&htim2);
			//     HAL_ADC_Start_DMA(&hadc1, (uint16_t*)&adc_2, 2);


		HAL_ADC_Stop_DMA(&hadc1);

			adc_buf[count]= adc_2[0];
			adc_buf[count+1]= adc_2[1];

			count ++;
			count ++;

		if(count==20) {
			HAL_ADC_Stop_DMA(hadc1); ADC_Result(hadc1); count =0;
		}


	 }
}
void ADC_Start(ADC_HandleTypeDef* hadc1){

	HAL_ADC_Start_DMA(hadc1, (uint16_t*) &adc_2, 2);
}


void ADC_Result(ADC_HandleTypeDef* hadc1){

	HAL_ADC_Stop_DMA(hadc1); // это необязательно
//HAL_TIM_Base_Stop_IT(&htim2);

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

//	 HAL_TIM_Base_Start_IT(&htim2);
	  //	 HAL_ADC_Start_DMA(&hadc1, (uint16_t*)&adc_2, 2);
}




void ADCstop(ADC_HandleTypeDef* hadc1,TIM_HandleTypeDef* htim2 ){

	HAL_ADC_Stop_DMA(hadc1);
    HAL_TIM_Base_Stop_IT(htim2);


}
