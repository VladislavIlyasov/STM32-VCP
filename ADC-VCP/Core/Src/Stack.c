#include "main.h"

#include "usbd_def.h"
#include "../LogicH/CircBuff.h"

extern queue_t queueIn, queueOut;
extern uint8_t BusyFlag;

//extern USBD_HandleTypeDef hUsbDeviceFS;
extern USBD_StatusTypeDef;


extern TIM_HandleTypeDef htim2;
extern ADC_HandleTypeDef hadc1;

uint8_t BusyFlag = 0;

void StackHandler(uint8_t* Data, uint32_t Len){
	short OvfIn;
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


void ReceiveHandler(){

	char InputSymb[2];


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



//////






int OutputHandler(){
	char Out[8];
    int Cond =0;  // переименовать
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




void OutputStack(char TransMes[], uint8_t Len){
	int OvfOut;
        // ovf
		OvfOut =put(&queueOut, TransMes, Len);
		if (OvfOut==0)
		{
			queueIn.bytes_avail++;
			queueIn.head = (queueIn.head--) % queueIn.buffer_size;
		} // else  вынести сообщение о переполнении выходного стека


}


///



void LogicReadInpStack(){


	  if(queueIn.tail != queueIn.head ){
		    	  ReceiveHandler();

		    	//  if(BusyFlag == USBD_BUSY && i!=0){i--;}
		      }
}




void LogicOutpStackCheck(){

	if(queueOut.bytes_avail!=0 ){
		    	  OutputHandler();
		      }
}
