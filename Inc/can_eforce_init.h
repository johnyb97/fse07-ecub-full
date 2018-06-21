#ifndef EFORCE_CAN_INIT
#define EFORCE_CAN_INIT

#include "main.h"
#include "string.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_can.h"

int CAN_eforce(CAN_HandleTypeDef *hcan,CAN_TypeDef *Instance,CanRxMsgTypeDef*canRxMsg);

#endif
