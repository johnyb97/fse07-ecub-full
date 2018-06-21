#include "cooling_circuit.h"

extern ECUB_Cooling_t ECUB_COOL;
static  int pwm_check_bool; //can message
static TIM_OC_InitTypeDef fan_values; //can message
static TIM_OC_InitTypeDef pump_values; //can message



void start_PWM(TIM_HandleTypeDef* pump,TIM_HandleTypeDef* fan, ECUB_Status_t *ECUB_Status){ //start pwm
	
	fan_values.OCMode = TIM_OCMODE_PWM1; //copied from main...
  fan_values.OCPolarity = TIM_OCPOLARITY_HIGH; //copied from main...
  fan_values.OCFastMode = TIM_OCFAST_DISABLE; //copied from main...
	
	pump_values.OCMode = TIM_OCMODE_PWM1; //copied from main...
  pump_values.OCPolarity = TIM_OCPOLARITY_HIGH; //copied from main...
  pump_values.OCFastMode = TIM_OCFAST_DISABLE; //copied from main...
	
	HAL_TIM_PWM_Start(fan,1); //start for fan channel 1
	HAL_TIM_PWM_Start(fan,2); //start for fan channel 2
	HAL_TIM_PWM_Start(fan,3); //start for fan channel 3
	HAL_TIM_PWM_Start(pump,1); //start for water pump channel 1
	HAL_TIM_PWM_Start(pump,2); //start for water pump channel 2
	ECUB_Status->PWR_FAN1_EN = 1; //can message
	ECUB_Status->PWR_FAN2_EN = 1; //can message
	ECUB_Status->PWR_FAN3_EN = 1; //can message
	ECUB_Status->PWR_WP1_EN = 1; //can message 
	ECUB_Status->PWR_WP2_EN = 1; //can message
	pump_pwm_process(pump,100);//pumps must be set to 100% all the time for some reason
	fan_pwm_process(fan,0);
}

void stop_PWM(TIM_HandleTypeDef* pump,TIM_HandleTypeDef* fan, ECUB_Status_t *ECUB_Status){ //end pwm
	HAL_TIM_PWM_Stop(fan,1); //end for fan channel 1
	HAL_TIM_PWM_Stop(fan,2); //end for fan channel 2
	HAL_TIM_PWM_Stop(fan,3); //end for fan channel 3
	HAL_TIM_PWM_Stop(pump,1); //end for water pump channel 1
	HAL_TIM_PWM_Stop(pump,2); //end for water pump channel 2
	ECUB_Status->PWR_FAN1_EN = 0; //can message
	ECUB_Status->PWR_FAN2_EN = 0; //can message
	ECUB_Status->PWR_FAN3_EN = 0; //can message
	ECUB_Status->PWR_WP1_EN = 0; //can message 
	ECUB_Status->PWR_WP2_EN = 0; //can message
}

void fan_pwm_process(TIM_HandleTypeDef* fan, int value){ //set PWM for Fan1, Fan2 and Fan3
	if(HAL_GPIO_ReadPin(LED2_GPIO_Port,LED2_Pin)==GPIO_PIN_RESET){ //debug LED
		HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET); //debug LED
	}else{
		HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET); //debug LED
	}
	fan_values.Pulse = value*10; //accuracy possible in numbers from 0-1000...10% is 100 value :) 
	HAL_TIM_OC_ConfigChannel(fan,&fan_values,1); //set PWM for air conditioning....pilots need to stay cool :)
	HAL_TIM_OC_ConfigChannel(fan,&fan_values,2); //set PWM for air conditioning....pilots need to stay cool :)
	HAL_TIM_OC_ConfigChannel(fan,&fan_values,3); //set PWM for air conditioning....pilots need to stay cool :)
	ECUB_COOL.FAN1 = value; //can message
	ECUB_COOL.FAN2 = value; //can message
	ECUB_COOL.FAN3 = value; //can message
}


void pump_pwm_process(TIM_HandleTypeDef* pump, int value){ //set PWM for Pump1 and Pump2
	if(HAL_GPIO_ReadPin(LED3_GPIO_Port,LED3_Pin)==GPIO_PIN_RESET){ //debug LED
		HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_SET); //debug LED
	}else{
		HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET); //debug LED
	}
	pump_values.Pulse = value*10; //accuracy possible in numbers from 0-1000...10% is 100 value :)
	HAL_TIM_OC_ConfigChannel(pump,&pump_values,1); //set PWM for water pump
	HAL_TIM_OC_ConfigChannel(pump,&pump_values,2); //set PWM for water pump
	ECUB_COOL.WP1 = value/10; //can message...devided 10...can factor == 10
	ECUB_COOL.WP2 = value/10; //can message...devided 10...can factor == 10
}

int pwm_check(ECUB_Status_t *ECUB_Status,CAN_HandleTypeDef *hcan){
	pwm_check_bool = 0; //any error present
	if (HAL_GPIO_ReadPin(Fault_Fan1_2_GPIO_Port,Fault_Fan1_2_Pin)){ //check if error on fan1,2 pin
		if(!pwm_check_bool){ // if no error detectet so far
			pwm_check_bool = 1; //change error
		}
		ECUB_Status->FT_PWR5_OT = 1; //can message for fault of fan1,2
	}
	if (HAL_GPIO_ReadPin(Fault_EM__Fan3_GPIO_Port,Fault_EM__Fan3_Pin)){ //check if error on fan3...or Energy meter
		if(!pwm_check_bool){ // if no error detectet so far
			pwm_check_bool = 1; //change error
		}
		ECUB_Status->FT_PWR4_OT = 1; //can message for fault of fan 3 or Energy meter
	}
	if (HAL_GPIO_ReadPin(Fault_WP_GPIO_Port,Fault_WP_Pin)){ //check if error on Water Pump 1,2
		if(!pwm_check_bool){ // if no error detectet so far
			pwm_check_bool = 1; //change error
		}
		ECUB_Status->FT_PWR5_OT = 1; //can message for fault of Water Pump 1,2
	}
	if	(ECUB_Cooling_need_to_send()){ //if can message is needed to be send
		ECUB_send_Cooling_s(&ECUB_COOL); //send can message
		HAL_CAN_Receive_IT(hcan,CAN_FIFO0); //just fix HAL can config
	}
	return pwm_check_bool; //returns if any error present
}
