#ifndef __PERIPHERY_SET_H__
#define __PERIPHERY_SET_H__

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"

#include "can_ECUB.h"
#include <string.h>

uint32_t play_RTDS(void);
uint32_t units_set(GPIO_PinState state,ECUB_Status_t *ECUB_Status);
uint32_t aux_set(GPIO_PinState state,ECUB_Status_t *ECUB_Status);
void battery_state_process(uint32_t battery_voltage);
void set_SDB_led(GPIO_PinState state,ECUB_Status_t *ECUB_Status);
#endif
