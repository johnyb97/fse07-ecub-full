#include "ADC.h"
#include "cooling_circuit.h"

#define ref_voltage 5000 //value for ADC equation from 0 to 5V in mV

ECUB_TEMPAux_t ECUB_TEMP_AUX; //can structure for sending data
volatile uint32_t ntc0_value;
volatile uint32_t ntc1_value;
volatile uint32_t ntc2_value;
volatile uint32_t ntc3_value;
volatile uint32_t ecua_u_value;
volatile uint32_t lv_bat_u_value;
volatile uint32_t service_box_u_value;
volatile uint16_t adc_measurement[8];

void start_ADC(ADC_HandleTypeDef* ADC_handle)
{
	HAL_ADC_Start_DMA(ADC_handle,(uint32_t*)adc_measurement,8); //start DMA...it saves data into adc_measurement
}

void stop_newADC(ADC_HandleTypeDef* ADC_handle)
{
	HAL_ADC_Stop_DMA(ADC_handle); //end DMA communication
}

void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* hadc)
{
	if(HAL_GPIO_ReadPin(LED1_GPIO_Port,LED1_Pin)==GPIO_PIN_RESET){ //debug LED
		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET); //debug LED
	}else{
		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET); //debug LED
	}
	ntc0_value = ((uint32_t)adc_measurement[3] * ref_voltage / 4095); //calculation voltage from raw ADC values...4095 range of value from ADC 
	ntc1_value = ((uint32_t)adc_measurement[4] * ref_voltage / 4095); //calculation voltage from raw ADC values...4095 range of value from ADC 
	ntc2_value = ((uint32_t)adc_measurement[5] * ref_voltage / 4095); //calculation voltage from raw ADC values...4095 range of value from ADC 
	ntc3_value = ((uint32_t)adc_measurement[6] * ref_voltage / 4095); //calculation voltage from raw ADC values...4095 range of value from ADC 
	ecua_u_value = ((uint32_t)adc_measurement[7] * 12 * ref_voltage / 4095); //calculation voltage from raw ADC values...4095 range of value from ADC...12 is acording to hardware conection...rezistence devider
	lv_bat_u_value = ((uint32_t)adc_measurement[2] * 12 * ref_voltage / 4095); //calculation voltage from raw ADC values...4095 range of value from ADC...12 is acording to hardware conection...rezistence devider
	service_box_u_value = ((uint32_t)adc_measurement[1] * 12 * ref_voltage / 4095); //calculation voltage from raw ADC values...4095 range of value from ADC...12 is acording to hardware conection...rezistence devider
}
void cooling_poccess(TIM_HandleTypeDef *fan, TIM_HandleTypeDef *pumps,CAN_HandleTypeDef* hcan){
	Cooling_procces_intern(fan,pumps,ntc0_value,ntc1_value,ntc2_value,ntc3_value); //intern function for cooling circuit logit
	if	(ECUB_TEMPAux_need_to_send()){ //if can message needed to be send
		ECUB_TEMP_AUX.Cooling1_NTC = ntc0_value; //fills can message
		ECUB_TEMP_AUX.Cooling2_NTC = ntc1_value; //fills can message
		ECUB_TEMP_AUX.Cooling3_NTC = ntc2_value; //fills can message
		ECUB_TEMP_AUX.Cooling4_NTC = ntc3_value; //fills can message
		ECUB_send_TEMPAux_s(&ECUB_TEMP_AUX); //sends can message
		HAL_CAN_Receive_IT(hcan,CAN_FIFO0); //fixes hal can structure
	}
}