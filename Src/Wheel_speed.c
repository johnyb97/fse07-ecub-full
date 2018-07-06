#include "Wheel_speed.h"

static uint32_t WSR; //posicion for encoder DMA data i hope
static uint32_t WSL; //posicion for encoder DMA data i hope

static uint32_t time_last_can; //time since last measurement

static ECUB_Wheelspeed_t Wheel_can; //can structure
int32_t i, key, j; //for sortInsertion function
uint32_t *temp; //only used in compute_average fuction
double average; //only used in compute_average fuction
double perimeter; //circuit of wheels
#define PI 3.14159265359 //pi constant
#define RADIUS 0.5 //radius of wheels

void start_WS_measure(TIM_HandleTypeDef *tim_WSR,TIM_HandleTypeDef *tim_WSL) //should start dma measurement of wheel speed
{ 
	HAL_TIM_Base_Start_IT(tim_WSR); //configuration for wheelspeed right
	HAL_TIM_Base_Start_IT(tim_WSL); //configuration for wheelspeed left
	HAL_TIM_IC_Start_IT(tim_WSR,TIM_CHANNEL_1); //configuration for wheelspeed left
	HAL_TIM_IC_Start_IT(tim_WSL,TIM_CHANNEL_1); //configuration for wheelspeed left
	perimeter = 2*PI*RADIUS; //circuit of wheels
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM3)
	{
		HAL_GPIO_TogglePin(LED7_GPIO_Port,LED7_Pin);
	}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM3)
	{
		TIM3->CNT = 0;
	}
}


void stop_WS_measure(TIM_HandleTypeDef *WSR,TIM_HandleTypeDef *WSL) //should stop dma measurement of wheel speed
{ 
	//HAL_TIM_Encoder_Stop_DMA(WSR,TIM_CHANNEL_ALL); //configuration for wheelspeed right
	//HAL_TIM_Encoder_Stop_DMA(WSL,TIM_CHANNEL_ALL); //configuration for wheelspeed left
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
	for(int i=0; i < number_of_data; i++){ //just creates new temp array
			temp[i] = data[(poss+(8-i))%8]; //creates temp array because HAL_TIM_IC_CaptureCallback can rewrite data during the compution 
   }
	
	sortInsertion(temp,number_of_data); //sorts given data
	if(number_of_data % 2){ //number of data is even
    average = (temp[number_of_data >> 1])*(perimeter); //finds average
  }else{
    average = ((temp[number_of_data>>1] + temp[(number_of_data>>1)-1]) >> 1)*(perimeter); //finds average
  }	
	return average; //estimated speed
}

void Can_WheelSpeed(CAN_HandleTypeDef *hcan)
{

	
}

