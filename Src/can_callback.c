#include "tx2/ringbuf.h"
#include "stm32f1xx_hal.h"
#include "tx2/can.h"
#include "can_ECUB.h"
#include <string.h>
CanTxMsgTypeDef txMsg;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

uint32_t txGetTimeMillis(void)
{
    return (uint32_t)HAL_GetTick();
}
int txHandleCANMessage(uint32_t timestamp, int bus, CAN_ID_t id, const void* data, size_t length)
{
    return 0;
}

int txSendCANMessage(int bus, CAN_ID_t id, const void* data, size_t length)
{

    txMsg.StdId = id;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = length;
    memcpy(txMsg.Data, data, length);
    if (bus == bus_CAN1_powertrain){
        hcan1.pTxMsg = &txMsg;
        return HAL_CAN_Transmit_IT(&hcan1);
    }
    if (bus == bus_CAN2_aux){
        hcan2.pTxMsg = &txMsg;
        return HAL_CAN_Transmit_IT(&hcan2);
    }
    return HAL_CAN_Transmit_IT(&hcan1);
}
