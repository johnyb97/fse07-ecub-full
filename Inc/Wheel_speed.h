#ifndef __WHEEL_H__
#define __WHEEL_H__
#include "stm32f105xc.h"
#include "stm32f1xx_hal.h"
#include "can_ECUB.h"

#define PERIOD_BUF_LEN  20

typedef struct
{
    int32_t len;
    int32_t win_len;
    int32_t index;

    int32_t arr[PERIOD_BUF_LEN];
} MEDIAN_TypeDef;

typedef struct
{
    int32_t counter;
    int32_t last_counter;
    int16_t input_prescale; // input prescaler to be used in current calculation
    uint8_t flag_overflow; // coutner overflow flag
    int32_t filt_period;

    MEDIAN_TypeDef filter;

} SENS_DATA_TypeDef;

typedef struct
{
    int zerospeed;
    int16_t freq; // Hz
    float speed; // meters per second
    int16_t speed_mms; // milimeters per second
    float rot_speed; // revolution per second
    int16_t rot_speed_mrps;  // 1000x revolution per second
    int16_t rot_speed_drpm; // 10 x revolution per minute

    SENS_DATA_TypeDef data;

} SPEED_SENS_TypeDef;


void start_WS_measure(TIM_HandleTypeDef *WSR,TIM_HandleTypeDef *WSL);
void stop_WS_measure(TIM_HandleTypeDef *WSR,TIM_HandleTypeDef *WSL);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
uint32_t compute_average(uint32_t *data,uint16_t number_of_data,uint16_t poss);
void Can_WheelSpeed(CAN_HandleTypeDef *hcan,TIM_HandleTypeDef *htim3,TIM_HandleTypeDef *htim5);
#endif
