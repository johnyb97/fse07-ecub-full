#ifndef __CARSTATE_H__
#define __CARSTATE_H__

#include <string.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_spi.h"
#include "stm32f1xx_hal_tim.h"
#include "BrakeLight.h"
#include "can_ECUB.h"
#include "periphery_set.h"
#include "cooling_circuit.h"
#include "ADC.h"

ECUF_Dashboard_t get_dash(void);
enum ECUB_CarState *get_state(void);
ECUB_Status_t* get_can_state(void);
int measure_SDC(SPI_HandleTypeDef * SPI_handle);
int	carstate_init(void);
void check_mess(void);
void carstate_process(SPI_HandleTypeDef * SPI_handle,CAN_HandleTypeDef* hcan);//mising values shit
void change_to_NOT_READY(enum ECUB_Notready_reason reason);
void set_latch_countdown(enum ECUB_Notready_reason reason);
void blink_led(uint32_t period);

#endif
