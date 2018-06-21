#ifndef __BRAKELIGHT_H__
#define __BRAKELIGHT_H__

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "carstate.h"
#include "can_ECUB.h"
#include "main.h"

void brake_set(GPIO_PinState state);
void brakelightprocess(ECUP_Status_t data,  ECUB_Status_t*	ECUB_Status);

#endif
