#include <eforce/tx.h>
//#include <tx.h>
#if defined(STM32F1)
#include <stm32f1xx.h>
#include <stm32f1xx_hal.h>
#include <stm32f1xx_hal_can.h>
#elif defined(STM32F2)
#include <stm32f2xx_hal.h>
#include <stm32f2xx_hal_can.h>
#endif

#include <string.h>
#include "stm32f105xc.h"
#include "stm32f1xx_hal_can.h"

void canInitPeripheral(CAN_TypeDef* instance, int loopback, int prescaler, int bs1, int bs2, int irqn) {
	__HAL_RCC_CAN1_CLK_ENABLE();

	CAN_HandleTypeDef hcan;
	hcan.Instance = instance;
	hcan.Init.Prescaler = prescaler;
	hcan.Init.Mode = CAN_MODE_NORMAL | (loopback ? CAN_MODE_SILENT_LOOPBACK : 0);
	hcan.Init.SJW = CAN_SJW_1TQ;
	hcan.Init.BS1 = bs1;
	hcan.Init.BS2 = bs2;
	hcan.Init.TTCM = DISABLE;
	hcan.Init.ABOM = ENABLE;
	hcan.Init.AWUM = DISABLE;
	hcan.Init.NART = DISABLE;
	hcan.Init.RFLM = DISABLE;
	hcan.Init.TXFP = DISABLE;

	HAL_CAN_Init(&hcan);

	CAN_FilterConfTypeDef filter;
	filter.FilterIdLow = 0;
	filter.FilterIdHigh = 0;
	filter.FilterMaskIdLow = 0;
	filter.FilterMaskIdHigh = 0;
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterNumber = 0;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_16BIT;
	filter.FilterActivation = ENABLE;
	filter.BankNumber = 0;
	HAL_CAN_ConfigFilter(&hcan, &filter);

	if (irqn) {
		HAL_NVIC_SetPriority(irqn, 2, 1);
		HAL_NVIC_EnableIRQ(irqn);
		instance->IER |= CAN_IER_FMPIE0 | CAN_IER_FMPIE1;
	}
}

static void WriteMailbox(CAN_TypeDef* instance, uint8_t mailbox, uint16_t sid, const uint8_t* data, size_t length) {
	instance->sTxMailBox[mailbox].TIR = (sid << 21);
	instance->sTxMailBox[mailbox].TDTR = length;

	uint32_t regs[2];
	memcpy(regs, data, length);
	instance->sTxMailBox[mailbox].TDLR = regs[0];
	instance->sTxMailBox[mailbox].TDHR = regs[1];
}

int canSendMessage(CAN_TypeDef* instance, CAN_ID_t id, const void* data, size_t length) {
	int mailbox=0;
	uint16_t sid = id;

	for (;;) {
		if ((instance->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
			mailbox = 0;
			break;
		}
		else if ((instance->TSR & CAN_TSR_TME1) == CAN_TSR_TME1) {
			mailbox = 1;
			break;
		}
		else if ((instance->TSR & CAN_TSR_TME2) == CAN_TSR_TME2) {
			mailbox = 2;
			break;
		}
	}

	instance->sTxMailBox[mailbox].TIR &= CAN_TI0R_TXRQ;
	WriteMailbox(instance, mailbox, sid, data, length);
	instance->sTxMailBox[mailbox].TIR |= CAN_TI0R_TXRQ;

	return 0;
}

int canReceiveMessage(CAN_TypeDef* instance) {
	CAN_ID_t id;
	size_t length;
	uint8_t data[8];

	if (canTryReceive(instance, &id, data, &length))
		return txReceiveCANMessage(id, data, length);
	else
		return 0;
}

int canTryReceive(CAN_TypeDef* instance, CAN_ID_t* id_out, void* data_out, size_t* length_out) {
	if (instance->RF0R & CAN_RF0R_FMP0) {
		if (instance->sFIFOMailBox[0].RIR & CAN_RI0R_IDE)
			*id_out = EXT_ID((instance->sFIFOMailBox[0].RIR >> 3) & 0x1fffffff);
		else
			*id_out = STD_ID((instance->sFIFOMailBox[0].RIR >> 21) & 0x7ff);

		*length_out = (instance->sFIFOMailBox[0].RDTR & 0x0f);

		uint32_t regs[2];
		regs[0] = instance->sFIFOMailBox[0].RDLR;
		regs[1] = instance->sFIFOMailBox[0].RDHR;
		memcpy(data_out, regs, sizeof(regs));

		instance->RF0R |= CAN_RF0R_RFOM0;
		return 1;
	}
	else
		return 0;
}

int canInitFiltersSTD(CAN_TypeDef* instance, const uint16_t* sids, size_t count) {
	CAN_HandleTypeDef hcan;
	hcan.Instance = instance;

	// TODO: check if not too many needed
	const int numFilters = (count + 3) / 4;

	for (int i = 0; i < numFilters; i++) {
		int id0 = sids[i * 4];
		int id1 = (i * 4 + 1 < count) ? sids[i * 4 + 1] : 0;
		int id2 = (i * 4 + 2 < count) ? sids[i * 4 + 2] : 0;
		int id3 = (i * 4 + 3 < count) ? sids[i * 4 + 3] : 0;

		CAN_FilterConfTypeDef filter;
		filter.FilterIdLow = 			(id0 << 5);
		filter.FilterIdHigh =			(id1 << 5);
		filter.FilterMaskIdLow =		(id2 << 5);
		filter.FilterMaskIdHigh =		(id3 << 5);
		filter.FilterFIFOAssignment =	CAN_FILTER_FIFO0;
		filter.FilterNumber =			i;
		filter.FilterMode =				CAN_FILTERMODE_IDLIST;
		filter.FilterScale =			CAN_FILTERSCALE_16BIT;
		filter.FilterActivation =		ENABLE;
		filter.BankNumber =				0;
		HAL_CAN_ConfigFilter(&hcan, &filter);
	}
}
