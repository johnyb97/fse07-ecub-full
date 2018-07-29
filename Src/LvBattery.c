#include "ltc6804.h"
#include "LvBattery.h"
#include "main.h"
#include "can_ECUB.h"
#include "ADC.h"

extern SPI_HandleTypeDef hspi2;
static uint8_t recive_data;
static ECUB_GLV_AMS_t ECUB_GLV; //can structure for sending data
static LTC_ConfigGroupTypeDef LV_Config_struct;
static LTC_CellVoltageListTypeDef VoltageList;
static LTC_AuxiliaryGroupTypeDef AuxiliaryGroup;
static LTC_CommGroupTypeDef LV_Config_com;
static uint32_t array_possicion;
static volatile int errorcount = 0;
	

static BMS_State_t state = BMS_Fault;
static int start_time;
	
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
	HAL_SPI_Transmit(&hspi2,(uint8_t*)&data,1,5);//martin.....
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
		hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
		hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
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
	__disable_irq();
	LV_Config_struct.ADC_ModeOption = LTC_ADC_Option_27_7_26;
	LV_Config_struct.DischargeTime = LTC_DCTO_0_5_Min;
	LV_Config_struct.CellDischarge = 0;//LTC_CELL_12;
	LV_Config_struct.GPIO_Pins = 0x1F;
	LV_Config_struct.Overvoltage = 0x0800;
	LV_Config_struct.Undervoltage = 0x0400;
	LV_Config_struct.ReferencePwrUp = LTC_Reference_Enable;
	
	LV_Config_com.Data0ICOM = LTC_SPI_ICOM_CSBLOW;
	LV_Config_com.Data0 = 0xFE;
	LV_Config_com.Data0FCOM = LTC_SPI_FCOM_CSBHIGH;
	LV_Config_com.Data1ICOM = LTC_SPI_ICOM_NOTRANSMIT;
	LV_Config_com.Data1 = 0x22;
	LV_Config_com.Data1FCOM = LTC_SPI_FCOM_CSBHIGH;
	LV_Config_com.Data2ICOM = LTC_SPI_ICOM_NOTRANSMIT;
	LV_Config_com.Data2 = 0x33;
	LV_Config_com.Data2FCOM = LTC_SPI_FCOM_CSBHIGH;
	
	config_spi_LV(1,&hspi2);
	LTC_Init();
	LTC_SetCommandAddress(LTC_ADDRESS_BROADCAST); //emil
	LTC_WakeUp(); //kondenzátoru
	LTC_WakeUp(); //kondenzátoru
	LTC_WriteConfigGroup(&LV_Config_struct);
	LTC_Microdelay(LTC_tREFUP);
	if (LTC_ReadConfigGroup(&LV_Config_struct) != LTC_OK) {
		errorcount++;
	}
	else {
		state = BMS_Ready;
		start_time = HAL_GetTick();
	}
	__enable_irq();
	HAL_GPIO_WritePin(Charge_en_GPIO_Port,Charge_en_Pin,GPIO_PIN_SET);//starts charging
	config_spi_LV(0,&hspi2);
	ECUB_GLV.CellID = 0;
	array_possicion = HAL_GetTick()%100;
}

static void BMS_Process(void) {
	switch (state) {
		case BMS_Fault:
			break; // pico
		case BMS_Ready:
			// need to start another measurement?
			if (HAL_GetTick() > start_time + BMS_CONVERSION_INTERVAL_MS) {
				config_spi_LV(1,&hspi2);
				__disable_irq();
				LTC_WakeUp();
				//LTC_StartStatusGroupConversion(LTC_ADCMODE_FILTERED,LTC_STATUSMEASURE_ALL);
				//LTC_StartCellVoltageConversion(LTC_ADCMODE_NORMAL, LTC_DISCHARGE_PERMITTED, LTC_CELLMEASURE_ALL);
				LTC_StartCombinedConversion(LTC_ADCMODE_NORMAL, LTC_DISCHARGE_NOT_PERMITTED);
				__enable_irq();
				config_spi_LV(0,&hspi2);

				state = BMS_CombinedConversion;
				start_time = HAL_GetTick();
			}
			break;
		case BMS_CombinedConversion:
			// conversion finished?
			if (HAL_GetTick() > start_time + BMS_COMBINED_CONVERSION_MS) {
				config_spi_LV(1,&hspi2);
				__disable_irq();
				LTC_WakeUp();
				LTC_WriteCommGroup(&LV_Config_com);	
				LTC_StartCommunication(1);
				LTC_WakeUp();
				if (LTC_ReadCellVoltageGroups(&VoltageList) != LTC_OK) {
						errorcount++;
				}
				if (LTC_ReadAuxiliaryGroups(&AuxiliaryGroup) != LTC_OK) {
					errorcount++;
				}
				__enable_irq();
				config_spi_LV(0,&hspi2);

				state = BMS_Ready;
				start_time = HAL_GetTick();
			}
			break;
	}
}

#include <math.h>
int GetTemperature(int index) {
	int raw = AuxiliaryGroup.GPIO_AnalogVoltage[index];
	int Vgpio = raw;		// * 0.1 mV, i guess

	int Rhigh = 5600;

	// divider: Vgpio = 5 V * Rntc / (Rntc + Rhigh)
	// Rhigh = 5.6 kOhm
	int Rntc = Vgpio * Rhigh / (50000 - Vgpio);

	const float T_ref = 273.15f + 25.0f;
	const float B = 3790.0f;
	const float R_ref = 10000;
	
	float temp = B / (B / T_ref - log(R_ref) + log(Rntc));
	
	return (int)(temp - 273.15f);
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

	BMS_Process();
	
	if(ECUB_GLV_AMS_need_to_send()){
		ECUB_GLV.CellID = (ECUB_GLV.CellID+1)%6;
		if (ECUB_GLV.CellID < 3){
			if(ECUB_GLV.CellID == 1){
				ECUB_GLV.Volt_cell = (uint16_t)(VoltageList.Cell[2]/10);
			}else{
				ECUB_GLV.Volt_cell = (uint16_t)(VoltageList.Cell[ECUB_GLV.CellID]/10);
			}
		}else{
			ECUB_GLV.Volt_cell = (uint16_t)(VoltageList.Cell[ECUB_GLV.CellID+3]/10);
		}
		ECUB_GLV.Volt = LV_voltage_recive();
		ECUB_GLV.Temp_cell = (GetTemperature(ECUB_GLV.CellID % 2)+25)*2;
		array_possicion = (array_possicion+1)%100;
		ECUB_send_GLV_AMS_s(&ECUB_GLV);
		HAL_CAN_Receive_IT(hcan,CAN_FIFO0);
		config_spi_LV(0,&hspi2);
	}

}
