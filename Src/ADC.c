#include "ADC.h"
#include "cooling_circuit.h"

#define ref_voltage 3300 //value for ADC equation from 0 to 5V in mV
#define TEMPERATURE_OUTSIDE 25; //outside temperature

ECUB_TEMPAux_t ECUB_TEMP_AUX; //can structure for sending data
volatile uint32_t ntc0_value;
volatile uint32_t ntc1_value;
volatile uint32_t ntc2_value;
volatile uint32_t ntc3_value;
volatile uint32_t ecua_u_value;
volatile uint32_t lv_bat_u_value;
volatile uint32_t service_box_u_value;
volatile uint16_t adc_measurement[9];

void start_ADC(ADC_HandleTypeDef* ADC_handle)
{
	HAL_ADC_Start_DMA(ADC_handle,(uint32_t*)adc_measurement,9); //start DMA...it saves data into adc_measurement
}

void stop_newADC(ADC_HandleTypeDef* ADC_handle)
{
	HAL_ADC_Stop_DMA(ADC_handle); //end DMA communication
}

void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* hadc)
{
	ntc0_value = ((uint32_t)adc_measurement[3] * ref_voltage / 4095); //calculation voltage from raw ADC values...4095 range of value from ADC 
	ntc1_value = ((uint32_t)adc_measurement[4] * ref_voltage / 4095); //calculation voltage from raw ADC values...4095 range of value from ADC 
	ntc2_value = ((uint32_t)adc_measurement[5] * ref_voltage / 4095); //calculation voltage from raw ADC values...4095 range of value from ADC 
	ntc3_value = ((uint32_t)adc_measurement[6] * ref_voltage / 4095); //calculation voltage from raw ADC values...4095 range of value from ADC 
	ecua_u_value = ((uint32_t)adc_measurement[7] * 124 * ref_voltage / 40950); //calculation voltage from raw ADC values...4095 range of value from ADC...12 is acording to hardware conection...rezistence devider
	lv_bat_u_value = ((uint32_t)adc_measurement[2] * 124 * ref_voltage / 40950); //calculation voltage from raw ADC values...4095 range of value from ADC...12 is acording to hardware conection...rezistence devider
	service_box_u_value = ((uint32_t)adc_measurement[1] * 124 * ref_voltage / 40950); //calculation voltage from raw ADC values...4095 range of value from ADC...12 is acording to hardware conection...rezistence devider
}
int LV_voltage_recive(void){
	return (int)lv_bat_u_value;
}

uint32_t Chip_temperature(void){
	return TEMPERATURE_OUTSIDE;
}
void cooling_poccess(TIM_HandleTypeDef *fans, TIM_HandleTypeDef *pumps,CAN_HandleTypeDef* hcan){
	ECUF_Dashboard_t dash =  get_dash();
	if(dash.WP_ON != 1){
		if((*get_state())>=ECUB_CarState_TS_ON){ //cooling only if not on LV battery
			Cooling_process_intern(fans,pumps,ntc0_value,ntc1_value,ntc2_value,ntc3_value); //intern function for cooling circuit logit
		}else{
			fan_pwm_process(fans,0); //stops fans
			pump_pwm_process(pumps,0); //stops pumps
		}
	}else{
		fan_pwm_process(fans,100); //stops fans
		pump_pwm_process(pumps,100); //stops pumps
	}
	if	(ECUB_TEMPAux_need_to_send()){ //if can message needed to be send
		ECUB_TEMP_AUX.Cooling1_NTC = (((2540-ntc0_value)/24) * 2) + 25; //fills can message
		ECUB_TEMP_AUX.Cooling2_NTC = (((2540-ntc1_value)/24) * 2) + 25; //fills can message
		ECUB_TEMP_AUX.Cooling3_NTC = (((2540-ntc2_value)/24) * 2) + 25; //fills can message
		ECUB_TEMP_AUX.Cooling4_NTC = (((2540-ntc3_value)/24) * 2) + 25; //fills can message
		ECUB_send_TEMPAux_s(&ECUB_TEMP_AUX); //sends can message
		HAL_CAN_Receive_IT(hcan,CAN_FIFO0); //fixes hal can structure
	}
}
