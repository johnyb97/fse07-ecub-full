#include "Wheel_speed.h"
static uint32_t WSRA; //posicion for encoder DMA data i hope
static uint32_t WSRB; //posicion for encoder DMA data i hope
static uint32_t WSLA; //posicion for encoder DMA data i hope
static uint32_t WSLB; //posicion for encoder DMA data i hope
static uint32_t wsr_data[8]; //data from measurement for averaging
static uint32_t wsl_data[8]; //data from measurement for averaging
static uint16_t wsr_poss; //posicion of new data in wsl_data
static uint16_t wsl_poss; //posicion of new data in wsl_data
static uint32_t summ; //used for averaging data
static uint32_t time_last_can; //time since last measurement
static uint8_t wsr_number_data; //number of new data since last can message (max 8)
static uint8_t wsl_number_data; //number of new data since last can message (max 8)
static ECUB_Wheelspeed_t Wheel_can; //can structure
int32_t i, key, j; //for sortInsertion function
uint32_t *temp; //only used in compute_average fuction
uint32_t average; //only used in compute_average fuction
uint16_t number_good_data; //only used in compute_average fuction
uint32_t limit; //maximal difference between data
#define DIFF_LIMIT 50 //maximal difference between data in mille...now its 5%

void start_WS_measure(TIM_HandleTypeDef *WSR,TIM_HandleTypeDef *WSL) //should start dma measurement of wheel speed
{ 
	HAL_TIM_Encoder_Start_DMA(WSR,TIM_CHANNEL_ALL,&WSRA,&WSRB,4); //configuration for wheelspeed right
	HAL_TIM_Encoder_Start_DMA(WSL,TIM_CHANNEL_ALL,&WSLA,&WSLB,4); //configuration for wheelspeed left
	WSRA = 0; //starting speed
	WSRB = 0; //starting speed
	WSLA = 0; //starting speed
	WSLB = 0; //starting speed
	wsr_poss = 0; //starting possicion
	wsl_poss = 0; //starting possicion
}

void stop_WS_measure(TIM_HandleTypeDef *WSR,TIM_HandleTypeDef *WSL) //should stop dma measurement of wheel speed
{ 
	HAL_TIM_Encoder_Stop_DMA(WSR,TIM_CHANNEL_ALL); //configuration for wheelspeed right
	HAL_TIM_Encoder_Stop_DMA(WSL,TIM_CHANNEL_ALL); //configuration for wheelspeed left
}

void sortInsertion(uint32_t *array, uint16_t size) //function for sorting data array
{
   for (i = 1; i < size; i++)
   {
       key = array[i];
       j = i-1;
       while (j >= 0 && array[j] > key)
       {
           array[j+1] = array[j];
           j = j-1;
       }
       array[j+1] = key;
   }
}

uint32_t compute_average(uint32_t *data,uint16_t number_of_data,uint16_t poss) //simple function to compute average form given array
{
	number_good_data = 0; //number of filtered data
	for(int i=0; i < number_of_data; i++) //just creates new temp array
   {
		 if (poss >= i){
			temp[i] = data[poss-i]; //creates temp array because HAL_TIM_IC_CaptureCallback can rewrite data during the compution 
		 }else{
			temp[i] = data[8+(poss-i)];
		 }
   }
	sortInsertion(temp,number_of_data); //sorts given data
	if(number_of_data % 2)
  {
    average = temp[number_of_data >> 1]; //finds average
  }
  else
  {
    average = ((temp[number_of_data>>1] + temp[(number_of_data>>1)-1]) >> 1); //finds average
  }
	limit = (average*DIFF_LIMIT)/1000; //maximum difference allowed between data
	for (int x = 0;x<number_of_data;++x){ //goes througt all data values
		if (temp[x]<average){
			if(average-temp[x] < limit){ //if data are close enough to average
					summ+=temp[x]; // adds all values together
					number_good_data++; //for finding average
			}
		}else{
			if(temp[x]-average < limit){ //if data are close enough to average
					summ+=temp[x]; // adds all values together
					number_good_data++; //for finding average
			}
		}
	}
	summ /= number_good_data; //final average
	return summ;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(HAL_GPIO_ReadPin(LED4_GPIO_Port,LED4_Pin)==GPIO_PIN_RESET){ //debug LED
		HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_SET); //debug LED
	}else{
		HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_RESET); //debug LED
	}
	if(htim->Instance == TIM3){ //if callback for left wheel encoder
		wsl_data[wsl_poss] = (WSLA+WSLB)>>1; //calculate average from 2 values of encoder
		wsl_poss = (wsl_poss+1)%8; //change position
		wsl_number_data = (wsl_number_data+1)%9; //number of new data since last can message
	} 
	if(htim->Instance == TIM5){ //if callback for left wheel encoder
		wsr_data[wsr_poss] = (WSRA+WSRB)>>1; //calculate average from 2 values of encoder
		wsr_poss = (wsr_poss+1)%8; //change position
		wsr_number_data = (wsr_number_data+1)%9; //number of new data since last can message
	}
}
void Can_WheelSpeed(CAN_HandleTypeDef *hcan){
 if(ECUB_Wheelspeed_need_to_send()){ //if can message needs to be send
		Wheel_can.WhR = compute_average(wsr_data,wsr_number_data,wsr_poss); //aritmetic average i hope
		Wheel_can.WhL = compute_average(wsl_data,wsl_number_data,wsl_poss); //aritmetic average i hope
		wsr_number_data = 0; //number of new data since last can message
		wsl_number_data = 0; //number of new data since last can message
		Wheel_can.Timestamp = HAL_GetTick()-time_last_can; //time since last message
		//error needs to be aded :(
		//error needs to be aded :(
		ECUB_send_Wheelspeed_s(&Wheel_can); //sends can message
		time_last_can = HAL_GetTick(); //time of new message
	 	HAL_CAN_Receive_IT(hcan,CAN_FIFO0);
		Wheel_can.SEQ++;
 }
}

