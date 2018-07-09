#include "cooling_circuit.h"

ECUB_Cooling_t ECUB_COOL;
static  int pwm_check_bool; //can message
static TIM_OC_InitTypeDef fan_values; //can message
static TIM_OC_InitTypeDef pump_values; //can message
MCR_ThermalMeasuresA_t MCR_Engine_A; //can structure for engine temperatures rear engine (left??)
MCR_ThermalMeasuresB_t MCR_Engine_B; //can structure for engine temperatures rear engine (right??)
MCF_ThermalMeasuresA_t MCF_Engine_A; //can structure for engine temperatures front engine (left??)
MCF_ThermalMeasuresB_t MCF_Engine_B; //can structure for engine temperatures front engine (right??)
uint32_t max_engine_temperature; //maximal current engines temperature
uint32_t max_sensors_after_engine_temperature; //maximal current sensors after engine temperature
uint32_t min_sensors_before_engine_temperature; //minimal current sensors before engine temperature
uint32_t new_pwm_value; //value of fans pwm signal
uint32_t additional_pwm_value; //value added to pwm value because of some growing difference betwen temperatures
uint64_t last_temp_check_time; //check for gradualy changing temperature
uint32_t last_temp_check_SensorsSensors_diff; //temperature during last temperature check
uint32_t last_temp_check_EngineSensors_diff; //temperature during last temperature check

#define MAX_ENGINE_TEMP 110 //maximal alowed engines temperature
#define MOTORS_COOL 40 //temperature below what we shut down cooling circuit
#define TEMP_LIMIT 90 //temp when all goes to max and warnings are sended via can message
#define CAN_TEMP_OFSET 40 //ofset of temperature in can message
#define TEMP_CHECK_TIME_DIFERENCE 1000 //ms time between two checks of temperature


void start_PWM(TIM_HandleTypeDef* pump,TIM_HandleTypeDef* fan, ECUB_Status_t *ECUB_Status){ //start pwm
	HAL_TIM_PWM_Start(fan,TIM_CHANNEL_1); //start for fan channel 1
	HAL_TIM_PWM_Start(fan,TIM_CHANNEL_2); //start for fan channel 2
	HAL_TIM_PWM_Start(fan,TIM_CHANNEL_3); //start for fan channel 3
	HAL_TIM_PWM_Start(pump,TIM_CHANNEL_3); //start for water pump channel 1
	HAL_TIM_PWM_Start(pump,TIM_CHANNEL_4); //start for water pump channel 2
	ECUB_Status->PWR_FAN1_EN = 1; //can message
	ECUB_Status->PWR_FAN2_EN = 1; //can message
	ECUB_Status->PWR_FAN3_EN = 1; //can message
	ECUB_Status->PWR_WP1_EN = 1; //can message 
	ECUB_Status->PWR_WP2_EN = 1; //can message
	pump_pwm_process(pump,0);
	fan_pwm_process(fan,0);
}

void stop_PWM(TIM_HandleTypeDef* pump,TIM_HandleTypeDef* fan, ECUB_Status_t *ECUB_Status){ //end pwm
	HAL_TIM_PWM_Stop(fan,TIM_CHANNEL_1); //end for fan channel 1
	HAL_TIM_PWM_Stop(fan,TIM_CHANNEL_2); //end for fan channel 2
	HAL_TIM_PWM_Stop(fan,TIM_CHANNEL_3); //end for fan channel 3
	HAL_TIM_PWM_Stop(pump,TIM_CHANNEL_3); //end for water pump channel 1
	HAL_TIM_PWM_Stop(pump,TIM_CHANNEL_4); //end for water pump channel 2
	ECUB_Status->PWR_FAN1_EN = 0; //can message
	ECUB_Status->PWR_FAN2_EN = 0; //can message
	ECUB_Status->PWR_FAN3_EN = 0; //can message
	ECUB_Status->PWR_WP1_EN = 0; //can message 
	ECUB_Status->PWR_WP2_EN = 0; //can message
}

void fan_pwm_process(TIM_HandleTypeDef* fan, int value){ //set PWM for Fan1, Fan2 and Fan3
	if(value){
	HAL_GPIO_WritePin(DRV_reset_GPIO_Port,DRV_reset_Pin,GPIO_PIN_SET);
	}else{
	HAL_GPIO_WritePin(DRV_reset_GPIO_Port,DRV_reset_Pin,GPIO_PIN_RESET);
	}
	__HAL_TIM_SET_COMPARE(fan,TIM_CHANNEL_1,(value*10));
	__HAL_TIM_SET_COMPARE(fan,TIM_CHANNEL_2,(value*10));
	__HAL_TIM_SET_COMPARE(fan,TIM_CHANNEL_3,(value*10));
	ECUB_COOL.FAN1 = value; //can message
	ECUB_COOL.FAN2 = value; //can message
	ECUB_COOL.FAN3 = value; //can message
}


void pump_pwm_process(TIM_HandleTypeDef* pump, int value){ //set PWM for Pump1 and Pump2
	if(value){
	HAL_GPIO_WritePin(DRV_reset_GPIO_Port,DRV_reset_Pin,GPIO_PIN_SET);
	}else{
	HAL_GPIO_WritePin(DRV_reset_GPIO_Port,DRV_reset_Pin,GPIO_PIN_RESET);
	}
	__HAL_TIM_SET_COMPARE(pump,TIM_CHANNEL_3,(value*10));
	__HAL_TIM_SET_COMPARE(pump,TIM_CHANNEL_4,(value*10));
	ECUB_COOL.WP1 = value/10; //can message...devided 10...can factor == 10
	ECUB_COOL.WP2 = value/10; //can message...devided 10...can factor == 10
}

int pwm_check(ECUB_Status_t *ECUB_Status,CAN_HandleTypeDef *hcan){
	pwm_check_bool = 0; //any error present
	if	(ECUB_Cooling_need_to_send()){ //if can message is needed to be send
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
		HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);//led blinking
		ECUB_send_Cooling_s(&ECUB_COOL); //send can message
		HAL_CAN_Receive_IT(hcan,CAN_FIFO0); //just fix HAL can config
	}
	return pwm_check_bool; //returns if any error present
}

void Cooling_process_intern(TIM_HandleTypeDef *fans,TIM_HandleTypeDef *pumps, uint32_t temp_left_before, uint32_t temp_left_after, uint32_t temp_right_before, uint32_t temp_right_after)
{
	if(!MCR_get_ThermalMeasuresA(&MCR_Engine_A)){ //if can message not recived
		fan_pwm_process(fans,100); //sets fans to 100%
	}else{
		if (MCR_Engine_A.TMOTSEN > (MAX_ENGINE_TEMP + CAN_TEMP_OFSET)){ //if we are all about to blow up
			fan_pwm_process(fans,100); //sets fans to 100%
			ECUB_COOL.FT_MOT_RR_OT = 1; //can messsage temperature is too high for engines to work
		}
		if (MCR_Engine_A.TMOTSEN > (TEMP_LIMIT + CAN_TEMP_OFSET)){ //if the motor tempemature is close to dangerous level 
			ECUB_COOL.WARN_MOT_RR_TEMP = 1;
			fan_pwm_process(fans,100); //sets fans to 100%
		}		
	}
	if(!MCR_get_ThermalMeasuresB(&MCR_Engine_B)){ //if can message not recived
		fan_pwm_process(fans,100); //sets fans to 100%
	}else{
		if (MCR_Engine_B.TMOTSEN > (MAX_ENGINE_TEMP + CAN_TEMP_OFSET)){ //if we are all about to blow up
			ECUB_COOL.FT_MOT_RL_OT = 1; //can messsage temperature is too high for engines to work
			fan_pwm_process(fans,100); //sets fans to 100%
		}
		if (MCR_Engine_B.TMOTSEN > (TEMP_LIMIT + CAN_TEMP_OFSET)){ //if the motor tempemature is close to dangerous level
			ECUB_COOL.WARN_MOT_RL_TEMP = 1; //sends warning that overtemperature can occur soon 
			fan_pwm_process(fans,100); //sets fans to 100%
		}		
	}
	if(!MCF_get_ThermalMeasuresA(&MCF_Engine_A)){ //if can message not recived
		fan_pwm_process(fans,100); //sets fans to 100%
	}else{
		if (MCF_Engine_A.TMOTSEN > (MAX_ENGINE_TEMP + CAN_TEMP_OFSET)){ //if we are all about to blow up
			fan_pwm_process(fans,100); //sets fans to 100%
			ECUB_COOL.FT_MOT_FR_OT = 1; //can messsage temperature is too high for engines to work
		}
		if (MCF_Engine_A.TMOTSEN > (TEMP_LIMIT + CAN_TEMP_OFSET)){ //if the motor tempemature is close to dangerous level
			ECUB_COOL.WARN_MOT_FR_TEMP = 1; //sends warning that overtemperature can occur soon
			fan_pwm_process(fans,100); //sets fans to 100%
		}		
	}
	if(!MCF_get_ThermalMeasuresB(&MCF_Engine_B)){ //if can message not recived
		fan_pwm_process(fans,100); //sets fans to 100%
	}else{
		if (MCF_Engine_B.TMOTSEN > (MAX_ENGINE_TEMP + CAN_TEMP_OFSET)){ //if we are all about to blow up
			fan_pwm_process(fans,100); //sets fans to 100%
			ECUB_COOL.FT_MOT_FL_OT = 1; //can messsage temperature is too high for engines to work
		}
		if (MCF_Engine_B.TMOTSEN > (TEMP_LIMIT + CAN_TEMP_OFSET)){ //if the motor tempemature is close to dangerous level
			ECUB_COOL.WARN_MOT_FL_TEMP = 1; //sends warning that overtemperature can occur soon
			fan_pwm_process(fans,100); //sets fans to 100%
		}		
	}
	
	//finding maximal engine temperature
	max_engine_temperature = MCR_Engine_A.TMOTSEN; //sets starting temperature
	if(MCR_Engine_B.TMOTSEN > max_engine_temperature){ //if new temp is greater than current maximal
		max_engine_temperature = MCR_Engine_B.TMOTSEN; //change current maximal temperature
	}
	if(MCF_Engine_A.TMOTSEN > max_engine_temperature){ //if new temp is greater than current maximal
		max_engine_temperature = MCF_Engine_A.TMOTSEN; //change current maximal temperature
	}
	if(MCF_Engine_B.TMOTSEN > max_engine_temperature){ //if new temp is greater than current maximal
		max_engine_temperature = MCF_Engine_B.TMOTSEN; //change current maximal temperature
	}
	max_engine_temperature -= CAN_TEMP_OFSET; //deals with can message ofset 
	//end of finding maximal engine temperature

	if ((max_engine_temperature<TEMP_LIMIT)&&(max_engine_temperature>MOTORS_COOL)){ //if motors are in temperature for partial cooling circuit activity
		new_pwm_value = (max_engine_temperature-MOTORS_COOL) *100/(TEMP_LIMIT-MOTORS_COOL); //chooses the pwm value according to linear mapping between defined temperatures MOTORS_COOL and TEMP_LIMIT
		if (last_temp_check_time< HAL_GetTick()){
			
			//finding min a max values of temperature sensors
			if (temp_left_before<temp_right_before){  //if left is smaller than right
				min_sensors_before_engine_temperature = temp_left_before; //sets required temp to left
			}else{
				min_sensors_before_engine_temperature = temp_right_before; //sets required temp to right
			}
			if (temp_left_after>temp_right_after){ //if left is bigger than right
				max_sensors_after_engine_temperature = temp_left_after; //sets required temp to left
			}else{
				max_sensors_after_engine_temperature = temp_right_after; //sets required temp to right
			}
			//end of finding min a max values of temperature sensors
			if (min_sensors_before_engine_temperature<max_sensors_after_engine_temperature){ //if heat given to engines
				if((max_sensors_after_engine_temperature-min_sensors_before_engine_temperature)>last_temp_check_SensorsSensors_diff){ //if diference between sensors is rising(more heat is given to engines)
					additional_pwm_value++; //add more to value of pwm signal
				}else{
					if(additional_pwm_value>0){
					additional_pwm_value--; //reduce value of pwm signal
					}
				}
				if((max_engine_temperature-max_sensors_after_engine_temperature)>last_temp_check_EngineSensors_diff){ //if diference between sensors and engines is rising(more heat created by engines)
					additional_pwm_value++; //add more to value of pwm signal
				}else{
					if(additional_pwm_value>0){
					additional_pwm_value--; //reduce value of pwm signal
					}
				}
			}
			last_temp_check_SensorsSensors_diff = (max_sensors_after_engine_temperature-min_sensors_before_engine_temperature); //change differences to current differences
			last_temp_check_EngineSensors_diff = (max_engine_temperature-max_sensors_after_engine_temperature); //change differences to current differences
			last_temp_check_time = HAL_GetTick()+TEMP_CHECK_TIME_DIFERENCE; //sets countdown for next check
		}
		if ((new_pwm_value+additional_pwm_value)<100){
			new_pwm_value+=additional_pwm_value;
		}else{
			new_pwm_value = 100;
		}
		fan_pwm_process(fans,new_pwm_value);
	}
	if (max_engine_temperature<MOTORS_COOL){ //if temperature of engines is low enagth to shut down the cooling circute
		fan_pwm_process(fans,0); //stops fans
		pump_pwm_process(pumps,0); //stops pumps
	}
	if(ECUB_COOL.FAN1>0){
		pump_pwm_process(pumps,100);
	}

}
