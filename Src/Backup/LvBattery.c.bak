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
 HAL_GPIO_WritePin(Bat_cs_GPIO_Port,Bat_cs_Pin,GPIO_PIN_SET);
}
void LTC_DRV_SPI_CSHigh(void){
 HAL_GPIO_WritePin(Bat_cs_GPIO_Port,Bat_cs_Pin,GPIO_PIN_RESET);
}

uint8_t LTC_DRV_SPI_Read(void){
	HAL_SPI_Receive(&hspi2,(uint8_t*)&recive_data,1,5);
	return recive_data;
}
void LTC_DRV_SPI_Write(uint8_t data){
	HAL_SPI_Transmit(&hspi2,(uint8_t*)&data,1,5);
}
/*
Configurates SPI comunication accordig to whitch chip should be comunicated with 
Entire code if copied from cubemx configuration
Witch == 1 is LV battery, witch == 0 is SDC
*/
void config_spi_LV(uint8_t witch,SPI_HandleTypeDef * hspi) 
{
	hspi->Instance = SPI2;
  hspi->Init.Mode = SPI_MODE_MASTER;
  hspi->Init.Direction = SPI_DIRECTION_2LINES;
	if(witch == 1){
		hspi->Init.DataSize = SPI_DATASIZE_8BIT;
		hspi->Init.CLKPhase = SPI_PHASE_2EDGE;
		hspi->Init.CLKPolarity = SPI_POLARITY_HIGH;
	}
	if(witch == 0){
		hspi->Init.DataSize = SPI_DATASIZE_16BIT;
		hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
		hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
	}
  hspi->Init.NSS = SPI_NSS_SOFT;
  hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi->Init.TIMode = SPI_TIMODE_DISABLE;
  hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi->Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(hspi) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}


void LV_init(void){
	HAL_GPIO_WritePin(Bat_cs_GPIO_Port,Bat_cs_Pin,GPIO_PIN_SET);
	config_spi_LV(1,&hspi2);
	LTC_Init();
	LTC_WakeUp();
	if (LTC_ReadConfigGroup(&LV_Config_struct) != LTC_OK) {
		//Error_Handler();
	}
	HAL_GPIO_WritePin(Charge_en_GPIO_Port,Charge_en_Pin,GPIO_PIN_SET);//starts charging
	config_spi_LV(1,&hspi2);
}

void LV_process(CAN_HandleTypeDef* hcan){
	if(HAL_GPIO_ReadPin(Charge_status_GPIO_Port,Charge_status_Pin)==GPIO_PIN_RESET){ //debug for charging
		HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_SET); //debug LED for charging
	}else{
		HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_RESET); //debug LED for charging
	}
	if(HAL_GPIO_ReadPin(Charge_pow_ok_GPIO_Port,Charge_pow_ok_Pin)==GPIO_PIN_SET){ //debug for charging
		HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin,GPIO_PIN_SET); //debug LED for charging
	}else{
		HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin,GPIO_PIN_RESET); //debug LED for charging
	}
	if(ECUB_GLV_AMS_need_to_send()){
		LTC_WakeUp();
		LTC_StartCellVoltageConversion(LTC_ADCMODE_NORMAL, LTC_DISCHARGE_PERMITTED, LTC_CELLMEASURE_ALL);
		LTC_WakeUp();
		if (LTC_ReadCellVoltageGroups(&VoltageList) != LTC_OK) {
			  //Error_Handler();
		}
		ECUB_send_GLV_AMS_s(&ECUB_GLV);
		HAL_CAN_Receive_IT(hcan,CAN_FIFO0);
	}

}
