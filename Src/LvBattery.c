#include "ltc6804.h"
#include "main.h"
#include "can_ECUB.h"

extern SPI_HandleTypeDef hspi2;
uint8_t recive_data;
enum ECUB_Batt_code battcode;
ECUB_GLV_AMS_t ECUB_GLV; //can structure for sending data
LTC_ConfigGroupTypeDef LV_Config_struct;
LTC_CellVoltageListTypeDef VoltageList;

void LTC_DRV_SPI_CSLow(void){
 HAL_GPIO_WritePin(Bat_cs_GPIO_Port,Bat_cs_Pin,GPIO_PIN_RESET);
}
void LTC_DRV_SPI_CSHigh(void){
 HAL_GPIO_WritePin(Bat_cs_GPIO_Port,Bat_cs_Pin,GPIO_PIN_SET);
}

uint8_t LTC_DRV_SPI_Read(void){
	HAL_SPI_Receive(&hspi2,(uint8_t*)&recive_data,1,5);
	return recive_data;
}
void LTC_DRV_SPI_Write(uint8_t data){
	HAL_SPI_Transmit(&hspi2,(uint8_t*)&data,1,5);
}

void LV_init(void){
	LTC_Init();
	LTC_WakeUp();
	if (LTC_ReadConfigGroup(&LV_Config_struct) != LTC_OK) {
		Error_Handler();
	}
}

void LV_process(CAN_HandleTypeDef* hcan){
	if(ECUB_GLV_AMS_need_to_send()){
		LTC_WakeUp();
		LTC_StartCellVoltageConversion(LTC_ADCMODE_NORMAL, LTC_DISCHARGE_PERMITTED, LTC_CELLMEASURE_ALL);
		LTC_WakeUp();
		if (LTC_ReadCellVoltageGroups(&VoltageList) != LTC_OK) {
			  Error_Handler();
		}
		ECUB_send_GLV_AMS_s(&ECUB_GLV);
		HAL_CAN_Receive_IT(hcan,CAN_FIFO0);
	}

}