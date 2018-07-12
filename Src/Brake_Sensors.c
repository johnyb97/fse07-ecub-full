#include "Brake_Sensors.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_spi.h"
#include "can_ECUB.h"

#include "main.h"

uint8_t config_register_write[2]; //config register
uint8_t recived_data[8]; //for recived data
volatile int temperatureleft; //temperature of left brake sensor
volatile int temperatureright; //temperature of right brake sensor
ECUB_TEMPSuspR_t ECUB_TEMP; //can structure for sending data

/*
Configurates SPI comunication accordig to whitch chip should be comunicated with 
Entire code if copied from cubemx configuration
Witch == 1 is brake sensor, witch == 0 is SDC
*/
void config_spi(uint8_t witch,SPI_HandleTypeDef * hspi) 
{
	hspi->Instance = SPI2;
  hspi->Init.Mode = SPI_MODE_MASTER;
  hspi->Init.Direction = SPI_DIRECTION_2LINES;
	if(witch == 1){
		hspi->Init.DataSize = SPI_DATASIZE_8BIT;
		hspi->Init.CLKPhase = SPI_PHASE_1EDGE;

	}
	if(witch == 0){
		hspi->Init.DataSize = SPI_DATASIZE_16BIT;
		hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
	}
	hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
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

/*
Configuration for brake temperature measurements
*/
void brake_sens_init(SPI_HandleTypeDef * hspi){
	config_spi(1,hspi); //configurates given SPI to current chip (only necessary because harware is wrong)
	config_register_write[0] = 0x80; //whitch register is written into
	config_register_write[1] = 0xC3;//config data
	HAL_GPIO_WritePin(PTRCS_GPIO_Port,PTRCS_Pin,GPIO_PIN_RESET); //select chip
	HAL_Delay(10);
	HAL_SPI_Transmit(hspi,config_register_write,2,60); //sends config data
	HAL_GPIO_WritePin(PTRCS_GPIO_Port,PTRCS_Pin,GPIO_PIN_SET); //select chip 
	
	config_spi(1,hspi); //configurates given SPI to current chip (only necessary because harware is wrong)

	HAL_GPIO_WritePin(PTLCS_GPIO_Port,PTLCS_Pin,GPIO_PIN_RESET); //select chip
	HAL_Delay(10);	
	HAL_SPI_Transmit(hspi,config_register_write,2,60); //sends config data
	HAL_GPIO_WritePin(PTLCS_GPIO_Port,PTLCS_Pin,GPIO_PIN_SET); //select chip
	config_spi(0,hspi); //configurates given SPI back to SDC chip (only necessary because harware is wrong)
}

/*
Function for measuring temperatures on brakes
Uses SPI comunication
Hardware is wrongly connected therefore configuration of SPI is needed to be changed (integrated into function)
*/
void brake_sens_process(SPI_HandleTypeDef * hspi,CAN_HandleTypeDef* hcan){
	if(ECUB_TEMPSuspR_need_to_send()){ //if can message is needed to be send
		config_spi(1,hspi); //configurates given SPI to current chip (only necessary because harware is wrong)
		
		HAL_GPIO_WritePin(PTRCS_GPIO_Port,PTRCS_Pin,GPIO_PIN_RESET); //select chip
		HAL_SPI_Transmit(hspi,(uint8_t[]){0x00},1,60); //transmit signal to transmit measurement
		HAL_SPI_Receive(hspi,(uint8_t*)&recived_data,8, 5);		
		HAL_GPIO_WritePin(PTRCS_GPIO_Port,PTRCS_Pin,GPIO_PIN_SET); //select chip 
		temperatureright = (((4000*(recived_data[1]*256)+recived_data[2])/65535)-1015)/4; //transformation to acurate temperature
		
		config_spi(1,hspi); //configurates given SPI to current chip (only necessary because harware is wrong)

		HAL_GPIO_WritePin(PTLCS_GPIO_Port,PTLCS_Pin,GPIO_PIN_RESET); //select chip
		HAL_SPI_Transmit(hspi,(uint8_t[]){0x00},1,60); //transmit signal to transmit measurement
		HAL_SPI_Receive(hspi,(uint8_t*)&recived_data,8, 5);		
		HAL_GPIO_WritePin(PTLCS_GPIO_Port,PTLCS_Pin,GPIO_PIN_SET); //select chip 
		temperatureleft = (((4000*(recived_data[1]*256)+recived_data[2])/65535)-1015)/4; //transformation to acurate temperature
		
		config_spi(0,hspi); //configurates given SPI back to SDC chip (only necessary because harware is wrong)
		
		ECUB_TEMP.BrakeCal_RL = temperatureleft/2; //saves measured temperature to CAN structure
		ECUB_TEMP.BrakeCal_RR = temperatureright/2; //saves measured temperature to CAN structure
		ECUB_send_TEMPSuspR_s(&ECUB_TEMP); //sends CAN message
		HAL_CAN_Receive_IT(hcan,CAN_FIFO0); //fixes HAL lib configuration
	}
}
