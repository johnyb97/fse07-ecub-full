#include "BrakeLight.h"

void brake_set(GPIO_PinState state){ //function for changing state of brakelight
	if (state != HAL_GPIO_ReadPin(BrakeLight_GPIO_Port,BrakeLight_Pin)){ //if current state is diferent than demanded
		HAL_GPIO_WritePin(BrakeLight_GPIO_Port,BrakeLight_Pin,state); //change light state
	}
}
void  brakelightprocess(ECUP_Status_t data,ECUB_Status_t* ECUB_Status){
	if(!ECUP_get_Status(&data)){ //if can message has not been recived correctly
		change_to_NOT_READY(ECUB_Notready_reason_TIMEOUT_ECUP); //change carstate to NOT READY
	}	
	if (data.BrakeActive){ //if brake is pressed
			brake_set(GPIO_PIN_SET); //light on
			ECUB_Status->BrakeLight_EN = 1; //can message brake light on
		}else{
			brake_set(GPIO_PIN_RESET); //light off
			ECUB_Status->BrakeLight_EN = 0; //can message brake light off
		}
}
