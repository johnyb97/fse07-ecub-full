/**
  ******************************************************************************
  * @file    ltc6804.c
  * @author  Jan Sixta
  * @version V0.1
  * @date    30-July-2014
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the Linear Technology LTC6804 battery management
	*          integrated circuit
  *           + Initialization and Configuration 
	*           + Support both daisy-chained and addressed devices
  *           + Reading and writing internal registers
	*           + Calculating PEC15 checksums
  @verbatim
 ===============================================================================
                     ##### How to use this driver #####
 ===============================================================================
    
		#1 - As the LTC6804 is a quite complex beast, the library should be con-
		figured first to meet the application demands. The configuration is done 
		through the ltc6804_conf.h file. A library mode should be set to specify
		the LTC6804 subversion: -1 or -2 (These differ with communication inter-
		faces and some command implementations. Please refer to device datasheet
		for more information.) When LTC6804-1 devices with isoSPI are used 
		in a daisy-chained configuration, the number of chained devices should 
		also be defined in the ltc6804_conf.h file.
		
		#2 - User must define the four low level functions for handling communication
		with the LTC6804 devices through an SPI. The four functions needed are:
		 + void LTC_DRV_SPI_Write(uint8_t data);
     + uint8_t LTC_DRV_SPI_Read(void);
     + void LTC_DRV_SPI_CSLow(void);
     + void LTC_DRV_SPI_CSHigh(void);
		These functions are called internally by the LTC6804 library. Any operations
		with the LTC6804 are handled by these four functions. The ChipSelect signal
		must be handled separately by thy library.
		
		#3 - To initialize the LTC6804 library, user should:
		 + Initialize the SPI used by the four functions mentioned in #1
		 + Call a LTC_Init() function.
		 
		#4 - For handling the complex protocols and register settings, this library
		offers a set of structures and associated functions to do the tasks.
		
		#5 - When initiating communication with the LTC6804, the device must first
		be woken up from the low-power mode by using the LTC_WakeUp() function.
		The wakeup process needs LTC_tWAKE time multiplied by the number of chained 
    devices (if applicable). The device then keeps the +5V power supply on for
		about 2000ms period of time. But after about 5.5ms the device goes back to 
		sleep mode. If pausing the communication for more than 5ms, new wakep
		event is needed, otherwise the communication gets lost.
		
		#6 - General notes for users
		NOTE: Please be aware, that if the device chain length is larger than 1, 
		the data reading functions use an array of structures (which quite makes
		sense, but I am jsut mentioning...) 
		NOTE: Keep in mind, that most commands (especially ADC conversions) take
		some time to finish, so please do not forget to wait the required time,
		or use the LTC_PollStatusADC() function (only possible with LTC6804-2).
		NOTE: The device goes back to sleep mode after about 5ms.
		NOTE: LTC6804-1 does support only broadcast (LTC_ADDRESS_BROADCAST) commads.
		NOTE: Switching on the main reference voltage source takes LTC_tREFUP time.
		NOTE: If the +5V regulator is loaded by more than 1uF capacitance, longer
		wakeup time (LTC_tWAKE) is to be needed.
		
		#7 - Example codes.
		All examples expect the user has already defined the low-level communication
		handling functions.
		Example 1: Consider 6 daisy-chained LTC6804-1 devices, LTC_CHAIN_LENGTH=6. 
		See the code example reading configuration register group of all devices:
		
		  LTC_ConfigGroupTypeDef LTC_ConfigStruct[6];
		  ...
		  LTC_Init();
		  ...
		  LTC_WakeUp();
			if (LTC_ReadConfigGroup(LTC_ConfigStruct) != LTC_OK) {
			  Error_Handler();
			}
		
		Example 2: Measuring cell voltages with three chained devices.
		
		  LTC_CellVoltageListTypeDef VoltageList[3];
			...
			LTC_Init();
			...
			LTC_WakeUp();
			LTC_StartCellVoltageConversion(
			  LTC_ADCMODE_NORMAL, LTC_DISCHARGE_PERMITTED, LTC_CELLMEASURE_ALL);
			** Wait required time for the conversion **
			LTC_WakeUp(); ** if conversion time > 5ms **
			if (LTC_ReadCellVoltageGroups(VoltageList) != LTC_OK) {
			  Error_Handler();
			}
		
 ===============================================================================
                      ##### Library Limitations #####
 ===============================================================================		
 
     Current library version limitations and thing to be done:
 
      + LTC_WakeUp() does always wait full tWAKEUP (multiplied by chain length
			  if applicable), regardless the STANDBY or SLEEP modes. (tREADY should 
				be enough if STANDBY)
			+ LTC_PollStatusADC() is not implemented yet
			+ Any register reading function returns LTC_PECERROR and quits when PEC15 
			  error occurs. Due to this limitation, if any error occurs during reading
				of chained devices, the reading task is not finished.
			+ The library is not thread safe. Can be used only by a single task. 
			  No locking mechanisms are present. Global buffers are used.
		
    @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 JAN SIXTA </center></h2>
  *
  * Licensed under LICENSE (To Be Done ...)
  *
  ******************************************************************************
  */ 


/* Includes -------------------------------------------------------------------*/
#include "ltc6804.h"

/* Private macros -------------------------------------------------------------*/
#define CRC15_POLY     (uint16_t)0x4599

/* Transmitt and receive buffers 
   4 bytes header length (2 bytes command, 2 bytes PEC)
	 8 bytes data packet per device (6 bytes of register data, 2 bytes PEC)
*/
#define HEADER_LENGTH    4
#define PAYLOAD_LENGTH   8

/* Private typedef ------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------*/
uint16_t pec15Table[256];

uint8_t txbuffer[HEADER_LENGTH+PAYLOAD_LENGTH];
uint8_t rxbuffer[PAYLOAD_LENGTH];

/* Variable holding command the actual address set */
uint16_t CmdAddress = (uint16_t)LTC_ADDRESS_BROADCAST << 8;

/* Private function prototypes ------------------------------------------------*/
void init_PEC15_Table(void);
void transmit_packet(uint8_t length);
void receive_packet(uint8_t length);
void transceive_data(uint8_t txlen, uint8_t rxlen);
void store_command_with_pec(uint16_t cmd);
uint8_t pec_is_ok(void);

/* Functions ------------------------------------------------------------------*/

/**
  * @brief  Initialization of the LTC6804 library
  * @param  None
  * @retval None
  */
void LTC_Init(void)
{	
	init_PEC15_Table();	
}


/**
  * @brief  Wakes up device connected to the bus. Sends a dummy byte on the bus
            and waits the time of chain length multipled by tWake (100us typ, 300us max).
  * @param  none
  * @retval None
  */
void LTC_WakeUp(void)
{
  LTC_DRV_SPI_CSLow();
	LTC_DRV_SPI_Write(0x00);
	LTC_DRV_SPI_CSHigh();	
		
	LTC_Microdelay(LTC_tWAKE*LTC_CHAIN_LENGTH);
}

/**
  * @brief  Writes Configuration Register Group of LTC6804
  * @param  config: Pointer to the ConfigGroupStructure array
  * @retval None
  */
LTC_ResultTypeDef LTC_WriteConfigGroup(LTC_ConfigGroupTypeDef *config)
{
  uint16_t pec;
	uint8_t i,j;
	
	store_command_with_pec(CmdAddress | LTC_CC_WRCFG);
	
	LTC_DRV_SPI_CSLow();
	
	transmit_packet(4);	
	
	for (i=0; i<LTC_CHAIN_LENGTH; i++) {
		#if (LTC_CHAIN_WRITEMODE==LTC_WRMODE_ALLSAME)
		  j = 0;
		#else
		  j = i;
		#endif
	  txbuffer[0] = ((config[j].GPIO_Pins)<<3)|(config[j].ReferencePwrUp)|(config[j].ADC_ModeOption);
	  txbuffer[1] = (uint8_t)(config[j].Undervoltage);
	  txbuffer[2] = (uint8_t)(((config[j].Undervoltage)>>8)&0x000F) | (uint8_t)((((uint16_t)(config[j].Overvoltage))<<4)&0x00F0);
	  txbuffer[3] = (uint8_t)(((config[j].Overvoltage)>>4)&0x00FF);
	  txbuffer[4] = (uint8_t)(config[j].CellDischarge); 
	  txbuffer[5] = (uint8_t)(((config[j].CellDischarge)>>8)&0x000F) | (config[j].DischargeTime);
	  pec = pec15(&txbuffer[0], 6);
	  txbuffer[6] = (uint8_t)(pec>>8);
	  txbuffer[7] = (uint8_t)(pec);
		
		transmit_packet(8);
	} 
		
	LTC_DRV_SPI_CSHigh();
	
	return LTC_OK;
}


/**
  * @brief  Reads Configuration Register Group of LTC6804
  * @param  config: Pointer to the ConfigGroupStructure array
  * @retval LTC_OK when everything is OK, error otherwise
  */
LTC_ResultTypeDef LTC_ReadConfigGroup(LTC_ConfigGroupTypeDef *config)
{
	uint8_t j;
	
	store_command_with_pec(CmdAddress | LTC_CC_RDCFG);
	
	LTC_DRV_SPI_CSLow();
	transmit_packet(4);
	
	for (j=0; j<LTC_CHAIN_LENGTH; j++) {
		
		receive_packet(8);
		
	  if (!pec_is_ok()) return LTC_PECFAIL;
	
	  config[j].GPIO_Pins = (rxbuffer[0] & 0xF8) >> 3;	
	  config[j].ADC_ModeOption = rxbuffer[0] & LTC_ADC_Option_14_3_2;
	  config[j].ReferencePwrUp = rxbuffer[0] & LTC_Reference_Enable;
	  config[j].SWTEN_PinStatus = rxbuffer[0] & LTC_SWTEN_1;
	  config[j].Undervoltage = ((uint16_t)rxbuffer[1] | ((uint16_t)rxbuffer[2] << 8)) & 0x0FFF;
	  config[j].Overvoltage = (((rxbuffer[2] >> 4) & 0x0F) | ((uint16_t)rxbuffer[3] << 4)) & 0x0FFF;
	  config[j].CellDischarge = (rxbuffer[4] | ((uint16_t)rxbuffer[5] << 8)) & 0x0FFF;
	  config[j].DischargeTime = rxbuffer[5] & 0xF0;
	}
	
	LTC_DRV_SPI_CSHigh();
	
	return LTC_OK;
}


/**
  * @brief  Reads Cell Voltage Groups A B C D
  * @param  list: Pointer to structure with array of 12 cell voltages in 100uV increments
  *         When more than 1 devices connected in a chain, the pointer should address 
  *         an array of lists with the length same as the chain.
  * @retval LTC_OK when everything is OK, error otherwise
  */
LTC_ResultTypeDef LTC_ReadCellVoltageGroups(LTC_CellVoltageListTypeDef *list)
{
  uint8_t i;
	
	/* Read Group A */
	store_command_with_pec(CmdAddress | LTC_CC_RDCVA);	
	LTC_DRV_SPI_CSLow();
	transmit_packet(4);	
	for (i=0; i<LTC_CHAIN_LENGTH; i++) {
	  receive_packet(8);
		if (!pec_is_ok()) return LTC_PECFAIL;
	  list[i].Cell[0] = rxbuffer[0] | (rxbuffer[1] << 8);
	  list[i].Cell[1] = rxbuffer[2] | (rxbuffer[3] << 8);
	  list[i].Cell[2] = rxbuffer[4] | (rxbuffer[5] << 8);
	}	
	LTC_DRV_SPI_CSHigh();
	
	LTC_Microdelay(1);
	
	/* Read Group B */
	store_command_with_pec(CmdAddress | LTC_CC_RDCVB);	
	LTC_DRV_SPI_CSLow();
	transmit_packet(4);	
	for (i=0; i<LTC_CHAIN_LENGTH; i++) {
		receive_packet(8);
	  if (!pec_is_ok()) return LTC_PECFAIL;
	  list[i].Cell[3] = rxbuffer[0] | (rxbuffer[1] << 8);
	  list[i].Cell[4] = rxbuffer[2] | (rxbuffer[3] << 8);
	  list[i].Cell[5] = rxbuffer[4] | (rxbuffer[5] << 8);
	}
	LTC_DRV_SPI_CSHigh();
	
	LTC_Microdelay(1);
	
	/* Read Group C */
	store_command_with_pec(CmdAddress | LTC_CC_RDCVC);	
	LTC_DRV_SPI_CSLow();
	transmit_packet(4);	
	for (i=0; i<LTC_CHAIN_LENGTH; i++) {
	  receive_packet(8);
		if (!pec_is_ok()) return LTC_PECFAIL;
	  list[i].Cell[6] = rxbuffer[0] | (rxbuffer[1] << 8);
	  list[i].Cell[7] = rxbuffer[2] | (rxbuffer[3] << 8);
	  list[i].Cell[8] = rxbuffer[4] | (rxbuffer[5] << 8);
	}	
	LTC_DRV_SPI_CSHigh();
	
	LTC_Microdelay(1);
	
	/* Read Group D */
	store_command_with_pec(CmdAddress | LTC_CC_RDCVD);	
	LTC_DRV_SPI_CSLow();
	transmit_packet(4);	
	for (i=0; i<LTC_CHAIN_LENGTH; i++) {
	  receive_packet(8);
		if (!pec_is_ok()) return LTC_PECFAIL;
	  list[i].Cell[9] = rxbuffer[0] | (rxbuffer[1] << 8);
	  list[i].Cell[10] = rxbuffer[2] | (rxbuffer[3] << 8);
	  list[i].Cell[11] = rxbuffer[4] | (rxbuffer[5] << 8);
	}	
	LTC_DRV_SPI_CSHigh();
	
	return LTC_OK;
}

/**
  * @brief  Reads Auxiliary Register Groups A and B
  * @param  auxgroup: pointer to the structure holding 5 GPIO/analog voltages
            and voltage measured across second voltage reference, both 100uV 
            counts and 16bit precision
  * @retval LTC_OK when everything is OK, error otherwise
  */
LTC_ResultTypeDef LTC_ReadAuxiliaryGroups(LTC_AuxiliaryGroupTypeDef *auxgroup)
{
	uint8_t i;
	
 	/* Read Aux Group A */
	store_command_with_pec(CmdAddress | LTC_CC_RDAUXA);	
	LTC_DRV_SPI_CSLow();
	transmit_packet(4);	
  for (i=0; i<LTC_CHAIN_LENGTH; i++) {
	  receive_packet(8);	
    if (!pec_is_ok()) return LTC_PECFAIL;	
	  auxgroup[i].GPIO_AnalogVoltage[0] = rxbuffer[0] | (rxbuffer[1] << 8);
	  auxgroup[i].GPIO_AnalogVoltage[1] = rxbuffer[2] | (rxbuffer[3] << 8);
	  auxgroup[i].GPIO_AnalogVoltage[2] = rxbuffer[4] | (rxbuffer[5] << 8);
	}
	LTC_DRV_SPI_CSHigh();
	
	LTC_Microdelay(1);
	
	/* Read Aux Group B */
	store_command_with_pec(CmdAddress | LTC_CC_RDAUXB);	
	LTC_DRV_SPI_CSLow();
	transmit_packet(4);	
	for (i=0; i<LTC_CHAIN_LENGTH; i++) {
	  receive_packet(8);
    if (!pec_is_ok()) return LTC_PECFAIL;	
	  auxgroup[i].GPIO_AnalogVoltage[3] = rxbuffer[0] | (rxbuffer[1] << 8);
	  auxgroup[i].GPIO_AnalogVoltage[4] = rxbuffer[2] | (rxbuffer[3] << 8);
	  auxgroup[i].SecondReference = rxbuffer[4] | (rxbuffer[5] << 8);
	}
	LTC_DRV_SPI_CSHigh();
	
	return LTC_OK;
}

/**
  * @brief  Reads Auxiliary Register Group A
  * @param  auxgroup: pointer to the structure holding 5 GPIO/analog voltages
            and voltage measured across second voltage reference, both 100uV 
            counts and 16bit precision
  * @retval LTC_OK when everything is OK, error otherwise
  */
LTC_ResultTypeDef LTC_ReadAuxiliaryGroupA(LTC_AuxiliaryGroupTypeDef *auxgroup)
{
 uint8_t i;
	
 	/* Read Aux Group A */
	store_command_with_pec(CmdAddress | LTC_CC_RDAUXA);	
	LTC_DRV_SPI_CSLow();
	transmit_packet(4);	
  for (i=0; i<LTC_CHAIN_LENGTH; i++) {
	  receive_packet(8);	
    if (!pec_is_ok()) return LTC_PECFAIL;	
	  auxgroup[i].GPIO_AnalogVoltage[0] = rxbuffer[0] | (rxbuffer[1] << 8);
	  auxgroup[i].GPIO_AnalogVoltage[1] = rxbuffer[2] | (rxbuffer[3] << 8);
	  auxgroup[i].GPIO_AnalogVoltage[2] = rxbuffer[4] | (rxbuffer[5] << 8);
	}
	LTC_DRV_SPI_CSHigh();
	
	return LTC_OK;
}


/**
  * @brief  Reads Auxiliary Register Group B
  * @param  auxgroup: pointer to the structure holding 5 GPIO/analog voltages
            and voltage measured across second voltage reference, both 100uV 
            counts and 16bit precision
  * @retval LTC_OK when everything is OK, error otherwise
  */
LTC_ResultTypeDef LTC_ReadAuxiliaryGroupB(LTC_AuxiliaryGroupTypeDef *auxgroup)
{
  uint8_t i;
	
	/* Read Aux Group B */
	store_command_with_pec(CmdAddress | LTC_CC_RDAUXB);	
	LTC_DRV_SPI_CSLow();
	transmit_packet(4);	
	for (i=0; i<LTC_CHAIN_LENGTH; i++) {
	  receive_packet(8);
    if (!pec_is_ok()) return LTC_PECFAIL;	
	  auxgroup[i].GPIO_AnalogVoltage[3] = rxbuffer[0] | (rxbuffer[1] << 8);
	  auxgroup[i].GPIO_AnalogVoltage[4] = rxbuffer[2] | (rxbuffer[3] << 8);
	  auxgroup[i].SecondReference = rxbuffer[4] | (rxbuffer[5] << 8);
	}
	LTC_DRV_SPI_CSHigh();
	
	return LTC_OK;
}


/**
  * @brief  Reads Status Register Groups A and B
  * @param  status: pointer to the status group
  * @retval LTC_OK when everything is OK, error otherwise
  */
LTC_ResultTypeDef LTC_ReadStatusGroups(LTC_StatusGroupTypeDef *status)
{
	uint8_t i;
	
	/* Read Status Group A */
	store_command_with_pec(CmdAddress | LTC_CC_RDSTATA);	
	LTC_DRV_SPI_CSLow();
	transmit_packet(4);
	for (i=0; i<LTC_CHAIN_LENGTH; i++) {
	  receive_packet(8);
	  if (!pec_is_ok()) return LTC_PECFAIL;
	  status[i].SumOfCells = rxbuffer[0] | (rxbuffer[1] << 8);
	  status[i].DieTemperature = rxbuffer[2] | (rxbuffer[3] << 8);
	  status[i].AnalogSupplyVoltage = rxbuffer[4] | (rxbuffer[5] << 8);
	}
	LTC_DRV_SPI_CSHigh();
	
	LTC_Microdelay(1);	
	
	/* Read Status Group B */
	store_command_with_pec(CmdAddress | LTC_CC_RDSTATB);	
	LTC_DRV_SPI_CSLow();
	transmit_packet(4);
	for (i=0; i<LTC_CHAIN_LENGTH; i++) {
		receive_packet(8);
    if (!pec_is_ok()) return LTC_PECFAIL;	
	  status[i].DigitalSupplyVoltage = rxbuffer[0] | (rxbuffer[1] << 8);
	  status[i].UVOV_StatusFlags = rxbuffer[2] | (rxbuffer[3] << 8) | (rxbuffer[4] << 16);
	  status[i].RevisionCode = (rxbuffer[5] & 0xF0) >> 4;
	  status[i].MuxFail = rxbuffer[5] & 0x02;
	  status[i].ThermalShutdown = rxbuffer[5] & 0x01;
	}
	LTC_DRV_SPI_CSHigh();
		
	return LTC_OK;
}


/**
  * @brief  Reads Status Register Group A
  * @param  status: pointer to the status group
  * @retval LTC_OK when everything is OK, error otherwise
  */
LTC_ResultTypeDef LTC_ReadStatusGroupA(LTC_StatusGroupTypeDef *status)
{
  uint8_t i;
	
	/* Read Status Group A */
	store_command_with_pec(CmdAddress | LTC_CC_RDSTATA);	
	LTC_DRV_SPI_CSLow();
	transmit_packet(4);
	for (i=0; i<LTC_CHAIN_LENGTH; i++) {
	  receive_packet(8);
	  if (!pec_is_ok()) return LTC_PECFAIL;
	  status[i].SumOfCells = rxbuffer[0] | (rxbuffer[1] << 8);
	  status[i].DieTemperature = rxbuffer[2] | (rxbuffer[3] << 8);
	  status[i].AnalogSupplyVoltage = rxbuffer[4] | (rxbuffer[5] << 8);
	}
	LTC_DRV_SPI_CSHigh();
	
	return LTC_OK;
}


/**
  * @brief  Reads Status Register Group B
  * @param  status: pointer to the status group
  * @retval LTC_OK when everything is OK, error otherwise
  */
LTC_ResultTypeDef LTC_ReadStatusGroupB(LTC_StatusGroupTypeDef *status)
{
  uint8_t i;
	
	/* Read Status Group B */
	store_command_with_pec(CmdAddress | LTC_CC_RDSTATB);	
	LTC_DRV_SPI_CSLow();
	transmit_packet(4);
	for (i=0; i<LTC_CHAIN_LENGTH; i++) {
		receive_packet(8);
    if (!pec_is_ok()) return LTC_PECFAIL;	
	  status[i].DigitalSupplyVoltage = rxbuffer[0] | (rxbuffer[1] << 8);
	  status[i].UVOV_StatusFlags = rxbuffer[2] | (rxbuffer[3] << 8) | (rxbuffer[4] << 16);
	  status[i].RevisionCode = (rxbuffer[5] & 0xF0) >> 4;
	  status[i].MuxFail = rxbuffer[5] & 0x02;
	  status[i].ThermalShutdown = rxbuffer[5] & 0x01;
	}
	LTC_DRV_SPI_CSHigh();
	
	return LTC_OK;
}


/**
  * @brief  Clears All Cell Voltage Register Groups.
  * @param  None
  * @retval LTC_OK when everything is OK, error otherwise
  */
LTC_ResultTypeDef LTC_ClearCellVoltageGroups(void)
{
  store_command_with_pec(CmdAddress | LTC_CC_CLRCELL);
	LTC_DRV_SPI_CSLow();
	transmit_packet(4);
	LTC_DRV_SPI_CSHigh();
	return LTC_OK;
}


/**
  * @brief  Clears All Auxiliary Register Groups.
  * @param  None
  * @retval LTC_OK when everything is OK, error otherwise
  */
LTC_ResultTypeDef LTC_ClearAuxiliaryGroups(void)
{
  store_command_with_pec(CmdAddress | LTC_CC_CLRAUX);
	LTC_DRV_SPI_CSLow();
	transmit_packet(4);
	LTC_DRV_SPI_CSHigh();
	return LTC_OK;
}


/**
  * @brief  Clears All Status Register Groups.
  * @param  None
  * @retval LTC_OK when everything is OK, error otherwise
  */
LTC_ResultTypeDef LTC_ClearStatusGroups(void)
{
  store_command_with_pec(CmdAddress | LTC_CC_CLRSTAT);
	LTC_DRV_SPI_CSLow();
	transmit_packet(4);
	LTC_DRV_SPI_CSHigh();
	return LTC_OK;
}


/**
  * @brief  Starts measurement of cell voltages.
  * @param  ADC_Mode: LTC_ADCMODE_x (FAST, NORMAL, FILTERED)
  * @param  Discharge: LTC_DISCHARGE_x (PERMITTED or NOT_PERMITTED)
  * @param  CellMeasure: LTC_CELLMEASURE_x
  * @retval LTC_OK when everything is OK, error otherwise
  */
LTC_ResultTypeDef LTC_StartCellVoltageConversion(
	     uint16_t ADC_Mode, 
       uint16_t Discharge, 
       uint16_t CellMeasure)
{
  store_command_with_pec(CmdAddress | LTC_CC_ADCV | ADC_Mode | Discharge | CellMeasure);
	LTC_DRV_SPI_CSLow();
	transmit_packet(4);
	LTC_DRV_SPI_CSHigh();
	return LTC_OK;
}

/**
  * @brief  Starts selftest measurement of cell voltages.
  * @param  ADC_Mode: LTC_ADCMODE_x (FAST, NORMAL, FILTERED)
  * @param  Selftest: LTC_SELFTEST_x (1 or 2)
  * @retval LTC_OK when everything is OK, error otherwise
  */
LTC_ResultTypeDef LTC_StartSelfTestCellConversion(uint16_t ADC_Mode, uint16_t Selftest)
{
  store_command_with_pec(CmdAddress | LTC_CC_CVST | ADC_Mode | Selftest);
	LTC_DRV_SPI_CSLow();
	transmit_packet(4);
	LTC_DRV_SPI_CSHigh();
	return LTC_OK;
}


/**
  * @brief  Starts measurement of GPIO voltages.
  * @param  ADC_Mode: LTC_ADCMODE_x (FAST, NORMAL, FILTERED)
  * @param  GpioMeasure: LTC_GPIOMEASURE_x (ALL, 1..5, 2nd reference)
  * @retval LTC_OK when everything is OK, error otherwise
  */
LTC_ResultTypeDef LTC_StartGpioVoltageConversion(uint16_t ADC_Mode, uint16_t GpioMeasure)
{
  store_command_with_pec(CmdAddress | LTC_CC_ADAX | ADC_Mode | GpioMeasure);
	LTC_DRV_SPI_CSLow();
	transmit_packet(4);
	LTC_DRV_SPI_CSHigh();
	return LTC_OK;
}



/**
  * @brief  Starts selftest measurement of GPIO voltages.
  * @param  ADC_Mode: LTC_ADCMODE_x (FAST, NORMAL, FILTERED)
  * @param  Selftest: LTC_SELFTEST_x (1 or 2)
  * @retval LTC_OK when everything is OK, error otherwise
  */
LTC_ResultTypeDef LTC_StartSelfTestGpioConversion(uint16_t ADC_Mode, uint16_t Selftest)
{
  store_command_with_pec(CmdAddress | LTC_CC_AXST | ADC_Mode | Selftest);
	LTC_DRV_SPI_CSLow();
	transmit_packet(4);
	LTC_DRV_SPI_CSHigh();
	return LTC_OK;
}



/**
  * @brief  Starts measurement of StatusGroup voltages.
  * @param  ADC_Mode: LTC_ADCMODE_x (FAST, NORMAL, FILTERED)
  * @param  StatusMeasure: 
  * @retval LTC_OK when everything is OK, error otherwise
  */
LTC_ResultTypeDef LTC_StartStatusGroupConversion(uint16_t ADC_Mode, uint16_t StatusMeasure)
{
  store_command_with_pec(CmdAddress | LTC_CC_ADSTAT | ADC_Mode | StatusMeasure);
	LTC_DRV_SPI_CSLow();
	transmit_packet(4);
	LTC_DRV_SPI_CSHigh();
	return LTC_OK;
}



/**
  * @brief  Starts selftest measurement of status group.
  * @param  ADC_Mode: LTC_ADCMODE_x (FAST, NORMAL, FILTERED)
  * @param  Selftest: LTC_SELFTEST_x (1 or 2)
  * @retval LTC_OK when everything is OK, error otherwise
  */
LTC_ResultTypeDef LTC_StartSelfTestStatusConversion(uint16_t ADC_Mode, uint16_t Selftest)
{
  store_command_with_pec(CmdAddress | LTC_CC_STATST | ADC_Mode | Selftest);
	LTC_DRV_SPI_CSLow();
	transmit_packet(4);
	LTC_DRV_SPI_CSHigh();
	return LTC_OK;
}


/**
  * @brief  Polls for ADC status (waits for conversion end).
  *         This function (PLADC command) is not supported on LTC6804-1
  * @param  None
  * @retval LTC_OK when everything is OK, error otherwise
  */
LTC_ResultTypeDef LTC_PollStatusADC(void)
{
  #if (LTC_LIBRARY_MODE==LTC_LIBMODE_1)
	  /* LTC6804-1 does not support ADC polling! */
	  while (1) {}	  
	#else 
	  /* Current version of library does not support this command */
		while (1) {}	  
	#endif 
}



/**
  * @brief  Starts combined measurement of cell voltages and GPIO1,2 voltages.
  * @param  ADC_Mode: LTC_ADCMODE_x (FAST, NORMAL, FILTERED)
  * @param  Discharge: LTC_DISCHARGE_x (PERMITTED or NOT_PERMITTED)
  * @retval LTC_OK when everything is OK, error otherwise
  */
LTC_ResultTypeDef LTC_StartCombinedConversion(uint16_t ADC_Mode, uint16_t Discharge)
{
  store_command_with_pec(CmdAddress | LTC_CC_ADCVAX | ADC_Mode | Discharge);
	LTC_DRV_SPI_CSLow();
	transmit_packet(4);
	LTC_DRV_SPI_CSHigh();
	return LTC_OK;
}



/**
  * @brief  Writes group of communication registers (COMM).
  * @param  CommGroup: Group of communication registers, see LTC6804 datasheet.
  * @retval LTC_OK when everything is OK, error otherwise
  */
LTC_ResultTypeDef LTC_WriteCommGroup(LTC_CommGroupTypeDef *CommGroup)
{
  uint16_t pec;
	
	store_command_with_pec(CmdAddress | LTC_CC_WRCOMM);
	
	txbuffer[4] = CommGroup->Data0ICOM | (CommGroup->Data0 >> 4);
	txbuffer[5] = CommGroup->Data0FCOM | (uint8_t)(CommGroup->Data0 << 4);
	txbuffer[6] = CommGroup->Data1ICOM | (CommGroup->Data1 >> 4);
	txbuffer[7] = CommGroup->Data1FCOM | (uint8_t)(CommGroup->Data1 << 4);
	txbuffer[8] = CommGroup->Data2ICOM | (CommGroup->Data2 >> 4);
	txbuffer[9] = CommGroup->Data2FCOM | (uint8_t)(CommGroup->Data2 << 4);
	pec = pec15(&txbuffer[4], 6);
	txbuffer[10] = (uint8_t)(pec>>8);
	txbuffer[11] = (uint8_t)(pec);
	
	LTC_DRV_SPI_CSLow();
	transmit_packet(12);	
	LTC_DRV_SPI_CSHigh();
	
	return LTC_OK;
}


/**
  * @brief  Reads group of communication registers (COMM).
  * @param  CommGroup: Group of communication registers, see LTC6804 datasheet.
  * @retval LTC_OK when everything is OK, error otherwise
  */
LTC_ResultTypeDef LTC_ReadCommGroup(LTC_CommGroupTypeDef *CommGroup)
{
  store_command_with_pec(CmdAddress | LTC_CC_RDCOMM);
	
	transceive_data(4, 8);
	if (!pec_is_ok()) return LTC_PECFAIL;
	
	CommGroup->Data0ICOM = rxbuffer[0] & 0xF0;	
	CommGroup->Data0 = ((rxbuffer[0] & 0x0F) << 4) | ((rxbuffer[1] & 0xF0) >> 4);
	CommGroup->Data0FCOM = rxbuffer[1] & 0x0F;
	CommGroup->Data1ICOM = rxbuffer[2] & 0xF0;
	CommGroup->Data1 = ((rxbuffer[2] & 0x0F) << 4) | ((rxbuffer[3] & 0xF0) >> 4);
	CommGroup->Data1FCOM = rxbuffer[3] & 0x0F;
	CommGroup->Data2ICOM = rxbuffer[4] & 0xF0;
	CommGroup->Data2 = ((rxbuffer[4] & 0x0F) << 4) | ((rxbuffer[5] & 0xF0) >> 4);
	CommGroup->Data2FCOM = rxbuffer[5] & 0x0F;
	
	return LTC_OK;
}


/**
  * @brief  This function executes the communications via GPIOs of LTC6804, I2C or SPI
  *         as configured via the COMM register group using the LTC_WriteCommGroup function.
  * @param  BytesToTransmit: Number of bytes to transmit: 1 to 3. (24 clocks generated per each)
  * @retval LTC_OK when everything is OK, error otherwise
  */
LTC_ResultTypeDef LTC_StartCommunication(uint8_t BytesToTransmit)
{
  uint8_t i;
	
	store_command_with_pec(CmdAddress | LTC_CC_STCOMM);
	
	LTC_DRV_SPI_CSLow();
	
	transmit_packet(4);
	
	for (i=0; i<=3*BytesToTransmit; i++) {
	  LTC_DRV_SPI_Write(0x00);
	}
	
	LTC_DRV_SPI_CSHigh();
	
	return LTC_OK;
	
}

/**
  * @brief  Starts open wire test measurement of cell voltages.
  * @param  ADC_Mode: LTC_ADCMODE_x (FAST, NORMAL, FILTERED)
  * @param  Discharge: LTC_DISCHARGE_x (PERMITTED or NOT_PERMITTED)
  * @param  PUpDown: LTC_PUPDN_x (LTC_PUPDN_UP or LTC_PUPDN_DOWN)
  * @param  CellMeasure: LTC_CELLMEASURE_x
  * @retval LTC_OK when everything is OK, error otherwise
  */
LTC_ResultTypeDef LTC_StartOpenWireConversion(
	     uint16_t ADC_Mode, 
       uint16_t Discharge,
       uint16_t PUpDown,
       uint16_t CellMeasure)
{
  store_command_with_pec(CmdAddress | LTC_CC_ADCV | ADC_Mode | \
	                       Discharge | PUpDown | CellMeasure);
	LTC_DRV_SPI_CSLow();
	transmit_packet(4);
	LTC_DRV_SPI_CSHigh();
	
	return LTC_OK;
}

/**
  * @brief  Selects the address for the following commands to be transmitted to. This 
  *         function does not initiate any communication with the LTC6804, only preselects
  *         target address for the next command function call. The default address after library
  *         initialization is BROADCAST. Note that LTC6804-1 does support only BROADCAST commands.           
  * @param  Address: LTC_ADDRESS_x (BROADCAST, 0..15 or you can use the LTC_ADDRESS_VAL
  *         macro to calculate the address from a variable containing a value from 0 to 15)
  * @retval None
  */
void LTC_SetCommandAddress(uint8_t Address)
{
  CmdAddress = (uint16_t)Address << 8;
}


/**
  * @brief  Transmits the packet via the SPI
  * @param  length: number of bytes to send
  * @retval None
  */
void transmit_packet(uint8_t length)
{
  uint8_t i;
	
	for (i=0; i<length; i++) {
		LTC_DRV_SPI_Write(txbuffer[i]);
	}
}


/**
  * @brief  Receives the packet via the SPI
  * @param  length: number of bytes to read
  * @retval None
  */
void receive_packet(uint8_t length)
{
  uint8_t i;

	for(i=0; i<length; i++) {
		rxbuffer[i] = LTC_DRV_SPI_Read();
	}
}

/**
  * @brief  Transmits packet via SPI and then receives answer. Handles CS signal.
  * @param  txlen: number of bytes to send in txbuffer
  * @param  rxlen: number of bytes to receive into rxbuffer
  * @retval None
  */
void transceive_data(uint8_t txlen, uint8_t rxlen)
{
	LTC_DRV_SPI_CSLow();
	transmit_packet(txlen);	
	receive_packet(rxlen);
	LTC_DRV_SPI_CSHigh();
}


/**
  * @brief  Prepares a command with PEC15 in the TX buffer
  * @param  cmd: Command code
  * @retval None
  */
void store_command_with_pec(uint16_t cmd)
{
  uint16_t pec;	
	txbuffer[0] = (uint8_t)(cmd>>8);
	txbuffer[1] = (uint8_t)(cmd);
	pec = pec15(&txbuffer[0], 2);
	txbuffer[2] = (uint8_t)(pec>>8);
	txbuffer[3] = (uint8_t)(pec);
}


/**
  * @brief  Checks the PEC15 of data in RX buffer.
  * @param  None
  * @retval Returns positive on success, zero otherwise
  */
uint8_t pec_is_ok(void)
{
  uint16_t pec, pecreceived;
	pec = pec15(&rxbuffer[0], 6);
	pecreceived = (uint16_t)rxbuffer[6]<<8 | rxbuffer[7];
	return (pec!=pecreceived)?0:1;
}


/**
  * @brief  This function waits four times the specified number of ticks.
  * @note   Note that the 3 clock per specified tick count applies only to Cortex M3.   
  * @param  ticks: Number of clock ticks to delay the MCU
  * @retval None
  */
__asm void ltc_tickdelay(uint32_t ticks)
{
			SUBS R0, R0, #1
			BNE .-2
			BX LR
}


/* User defined WEAK functions -----------------------------------------------*/


__weak void LTC_DRV_SPI_CSLow(void) 
{
  /* NOTE: This function should not be modified.
	   The LTC_DRV_SPI_CSLow() function should be implemented in the user code.*/
}


__weak void LTC_DRV_SPI_CSHigh(void)
{
  /* NOTE: This function should not be modified.
	   The LTC_DRV_SPI_CSHigh() function should be implemented in the user code.*/
}

__weak void LTC_DRV_SPI_Write(uint8_t data) 
{
  /* NOTE: This function should not be modified.
	   The LTC_DRV_SPI_Write(..) function should be implemented in the user code.*/
}


__weak uint8_t LTC_DRV_SPI_Read(void)
{
  /* NOTE: This function should not be modified.
	   The LTC_DRV_SPI_Read() function should be implemented in the user code.*/
	return 0;
}



/* Disclaimer for the following PEC15 functions */
/**********************************************************
Copyright 2012 Linear Technology Corp. (LTC)
Permission to freely use, copy, modify, and distribute this software for any
purpose with or without fee is hereby granted, provided that the above
copyright notice and this permission notice appear in all copies:
THIS SOFTWARE IS PROVIDED “AS IS” AND LTC DISCLAIMS ALL WARRANTIES
INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO
EVENT SHALL LTC BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM ANY USE OF SAME, INCLUDING
ANY LOSS OF USE OR DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE
OR OTHER TORTUOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
PERFORMANCE OF THIS SOFTWARE.
***********************************************************/

void init_PEC15_Table()
{
  uint32_t remainder;
	uint16_t i, bit;
	
	for (i=0; i<256; i++) {
    remainder = i << 7;
    for (bit=8; bit>0; --bit) {
      if (remainder & 0x00004000) {
        remainder = (remainder << 1);
        remainder = (remainder ^ CRC15_POLY);
      }
      else {
        remainder = (remainder << 1);
      }
    }
    pec15Table[i] = remainder & 0x0000FFFF;
  }
}

uint16_t pec15(uint8_t *data , uint16_t len)
{
  uint32_t remainder, address;
	uint16_t i;
	
  remainder = 16;  /* PEC seed */
  for (i=0; i<len; i++) {
    address = ((remainder >> 7) ^ data[i]) & 0xFF; /* calculate PEC table address */
    remainder = (remainder << 8) ^ pec15Table[address];
  }
  return (remainder*2); /* The CRC15 has a 0 in the LSB so the final value must be multiplied by 2 */
}

/* END OF FILE */
