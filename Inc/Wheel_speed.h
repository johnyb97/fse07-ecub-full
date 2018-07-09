#ifndef __WHEEL_H__
#define __WHEEL_H__
#include "stm32f105xc.h"
#include "stm32f1xx_hal.h"
#include "can_ECUB.h"

void start_WS_measure(TIM_HandleTypeDef *WSR,TIM_HandleTypeDef *WSL);
void stop_WS_measure(TIM_HandleTypeDef *WSR,TIM_HandleTypeDef *WSL);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
uint32_t compute_average(uint32_t *data,uint16_t number_of_data,uint16_t poss);
void Can_WheelSpeed(CAN_HandleTypeDef *hcan,TIM_HandleTypeDef *htim3,TIM_HandleTypeDef *htim5);
#endif
