#include "Wheel_speed.h"
static uint32_t WSRA; //posicion for encoder DMA data i hope
static uint32_t WSRB; //posicion for encoder DMA data i hope
static uint32_t WSLA; //posicion for encoder DMA data i hope
static uint32_t WSLB; //posicion for encoder DMA data i hope
static uint32_t wsr_data[8]; //data from measurement for averaging
static uint32_t wsl_data[8]; //data from measurement for averaging
static uint16_t wsr_poss; //posicion of new data in wsl_data
static uint16_t wsl_poss; //posicion of new data in wsl_data
static uint32_t time_last_can; //time since last measurement
static uint8_t wsr_number_data; //number of new data since last can message (max 8)
static uint8_t wsl_number_data; //number of new data since last can message (max 8)
static ECUB_Wheelspeed_t Wheel_can; //can structure
uint32_t total_spins; //number of total number of rotation of wheel
uint32_t spins_array[255]; //number of spins for past can messagess
uint32_t times_array[255]; //times for past can messagess
double speeds_array[255]; //estimated speed for past can messagess
double estimated_array_speed; //speed estimated from previous arrays
uint8_t posicion_for_arrays; //pointer to current possicion for spins array,times array and speeds array
int32_t i, key, j; //for sortInsertion function
uint32_t *temp; //only used in compute_average fuction
double average; //only used in compute_average fuction
double circuit; //circuit of wheels
#define PI 3.14159265359 //pi constant
#define RADIUS 0.5 //radius of wheels

void start_WS_measure(TIM_HandleTypeDef *WSR,TIM_HandleTypeDef *WSL) //should start dma measurement of wheel speed
{ 
	HAL_TIM_Encoder_Start_IT(WSR,TIM_CHANNEL_ALL); //configuration for wheelspeed right
	HAL_TIM_Encoder_Start_IT(WSL,TIM_CHANNEL_ALL); //configuration for wheelspeed left
	WSRA = 0; //starting speed
	WSRB = 0; //starting speed
	WSLA = 0; //starting speed
	WSLB = 0; //starting speed
	wsr_poss = 0; //starting possicion
	wsl_poss = 0; //starting possicion
	total_spins = 0; //starting number of wheel rotations
	posicion_for_arrays = 0; //starting possicion in arrays
	spins_array[posicion_for_arrays] = 0; //sets starting values
	times_array[posicion_for_arrays] = 0; //sets starting values
	speeds_array[posicion_for_arrays] = 0; //sets starting values
	circuit = 2*PI*RADIUS; //circuit of wheels
}

void stop_WS_measure(TIM_HandleTypeDef *WSR,TIM_HandleTypeDef *WSL) //should stop dma measurement of wheel speed
{ 
	HAL_TIM_Encoder_Stop_DMA(WSR,TIM_CHANNEL_ALL); //configuration for wheelspeed right
	HAL_TIM_Encoder_Stop_DMA(WSL,TIM_CHANNEL_ALL); //configuration for wheelspeed left
}

void sortInsertion(uint32_t *array, uint16_t size) //function for sorting data array
{
   for (i = 1; i < size; i++){ //goes through all data
       key = array[i]; // current data are compared
       j = i-1; //it is compared with previous data in array 
       while (j >= 0 && array[j] > key){ //while not begining of array and current data are larger than key
           array[j+1] = array[j]; //change prev and current data
           j = j-1; //moves possicion closer to begining 
       }
       array[j+1] = key; //puts key to last possicion
   }
}

uint32_t compute_average(uint32_t *data,uint16_t number_of_data,uint16_t poss) //simple function to compute average form given array
{
	if (!number_of_data){ //if none rotation accured since last can message
		average = ((speeds_array[(posicion_for_arrays+254)%255] + 2*speeds_array[posicion_for_arrays]))/4; //just some correction
		return average; //estimated speed
	}
	
	for(int i=0; i < number_of_data; i++){ //just creates new temp array
			temp[i] = data[(poss+(8-i))%8]; //creates temp array because HAL_TIM_IC_CaptureCallback can rewrite data during the compution 
   }
	
	sortInsertion(temp,number_of_data); //sorts given data
	if(number_of_data % 2){ //number of data is even
    average = (temp[number_of_data >> 1])*(circuit); //finds average
  }else{
    average = ((temp[number_of_data>>1] + temp[(number_of_data>>1)-1]) >> 1)*(circuit); //finds average
  }
	
	average = ((speeds_array[(posicion_for_arrays+254)%255] + 2*speeds_array[posicion_for_arrays])+average)/4; //just some correction
	return average; //estimated speed
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	total_spins++; //number of spins since start
	
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
	  posicion_for_arrays = (posicion_for_arrays+1)%255; //moves possicion pointer
	 	spins_array[posicion_for_arrays] = total_spins; //saves new values
		times_array[posicion_for_arrays] = HAL_GetTick(); //time of new message
		speeds_array[posicion_for_arrays] = ((total_spins-spins_array[(posicion_for_arrays+254)%255])/(HAL_GetTick()-times_array[(posicion_for_arrays+254)%255]))*(circuit); //saves new speeds values
		
	 
	  Wheel_can.WhR = compute_average(wsr_data,wsr_number_data,wsr_poss); //aritmetic average i hope
		Wheel_can.WhL = compute_average(wsl_data,wsl_number_data,wsl_poss); //aritmetic average i hope
		wsr_number_data = 0; //number of new data since last can message
		wsl_number_data = 0; //number of new data since last can message
		Wheel_can.Timestamp = HAL_GetTick()-time_last_can; //time since last message
		//error needs to be aded :(
		//error needs to be aded :(
	 
	 
		ECUB_send_Wheelspeed_s(&Wheel_can); //sends can message
		time_last_can = HAL_GetTick(); //time of new message
	 	HAL_CAN_Receive_IT(hcan,CAN_FIFO0); //just to fix Hal can
		Wheel_can.SEQ++;	//sends check that the message is changing
 }
}

