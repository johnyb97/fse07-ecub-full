#include "ADC.h"

#define ref_voltage 5000 //value for ADC equation from 0 to 5V in mV

volatile uint32_t ntc0_value;
volatile uint32_t ntc1_value;
volatile uint32_t ntc2_value;
volatile uint32_t ntc3_value;
volatile uint32_t ecua_u_value;
volatile uint32_t lv_bat_u_value;
volatile uint32_t service_box_u_value;
volatile uint16_t adc_measurement[8];
//verze 1
/*
void start_ADC(ADC_HandleTypeDef* ADC_handle){//zahajeni ADC komunikace
	HAL_ADC_Start(ADC_handle);
}

void read_ADC(ADC_HandleTypeDef* ADC_handle){ //ziskat hodnotu z ADC mereni
	HAL_ADC_ConvCpltCallback(ADC_handle); //cekej do konce smycky mereni
	HAL_ADC_GetValue(ADC_handle); //ziskat data z mereni
	ntc0_value = ADC_CHANNEL_12; //overit
	ntc1_value = ADC_CHANNEL_13; //overit
	ntc2_value = ADC_CHANNEL_2; //overit
	ntc3_value = ADC_CHANNEL_3; //overit
	ecua_u_value = (ADC_CHANNEL_15 *12); //nasobeno 12 podle sch�matu //overit
	lv_bat_u_value = (ADC_CHANNEL_9 * 12); //nasobeno 12 podle sch�matu //overit
	service_box_u_value = (ADC_CHANNEL_8 * 12); //nasobeno 12 podle sch�matu //overit
}
void stop_ADC(ADC_HandleTypeDef* ADC_handle){ //ukonceni komunikace
	HAL_ADC_Stop(ADC_handle);
}
*/
//verze 2
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
