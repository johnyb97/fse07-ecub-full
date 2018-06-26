#ifndef __ADC_H__
#define __ADC_H__
#include "stm32f105xc.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_adc.h"

void start_ADC(ADC_HandleTypeDef* ADC_handle);
void stop_ADC(ADC_HandleTypeDef* ADC_handle);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void cooling_poccess(TIM_HandleTypeDef *fan, TIM_HandleTypeDef *pumps,CAN_HandleTypeDef* hcan);
#endif
