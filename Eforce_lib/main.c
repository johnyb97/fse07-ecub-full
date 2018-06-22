

/* Includes -----------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "ltc6804.h"

/* Private macros -----------------------------------------------------------*/
/* SPI peripheral used for communication */
#define LTC_SPI_PERIPHERAL           SPI1

/* Chipselect pin definition */
#define LTC_SPI_CS_GPIO              GPIOB
#define LTC_SPI_CS_GPIO_PIN          GPIO_Pin_2


/* Private variables --------------------------------------------------------*/
__IO uint32_t errorcount=0;

/* Private function prototypes ----------------------------------------------*/
void Init_SPI(void);

/* Private functions --------------------------------------------------------*/

void SystemInit(void)
{
  /* FPU settings -----------------------------------------------------------*/
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    /* set CP10 and CP11 Full Access */
	  SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2)); 
  #endif
	
	RCC_HSEConfig(RCC_HSE_ON);
	while (!(RCC->CR & RCC_CR_HSERDY)) {}
	
	FLASH_SetLatency(FLASH_Latency_5);
	FLASH_PrefetchBufferCmd(ENABLE);
	FLASH_InstructionCacheCmd(ENABLE);
	FLASH_DataCacheCmd(ENABLE);
	
	RCC_HCLKConfig(RCC_SYSCLK_Div1);
	RCC_PCLK1Config(RCC_HCLK_Div4);
	RCC_PCLK2Config(RCC_HCLK_Div2);
		
	/* 8MHz HSE Crystal,
		 M = 5
		 N = 210 
		 P = 2 (168MHz SYSCLK)
		 Q = 7 (48MHz USBCLK)
	*/	
	RCC_PLLConfig(RCC_PLLSource_HSE, 5, 210, 2, 7);
	RCC_PLLCmd(ENABLE);
	while (!(RCC->CR & RCC_CR_PLLRDY)) {}
		
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	while (RCC_GetSYSCLKSource() != 0x08) {}		
}


LTC_ConfigGroupTypeDef LTC_ConfigGroupStruct[2];
LTC_CellVoltageListTypeDef VoltageList[2];
LTC_AuxiliaryGroupTypeDef AuxGroup[2];
LTC_StatusGroupTypeDef StatusGroup[2];
LTC_CommGroupTypeDef LTC_CommGroupStruct1, LTC_CommGroupStruct2;


/* MAIN ---------------------------------------------------------------------*/
int main(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	uint32_t i;
	
	/* Chipselect pin init */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/* SPI peripheral Init */
	Init_SPI();
	
	/* LTC6804 library internal init */
	LTC_Init();		
	
	
	LTC_ConfigGroupStruct[0].ADC_ModeOption = LTC_ADC_Option_27_7_26;
	LTC_ConfigGroupStruct[0].DischargeTime = LTC_DCTO_0_5_Min;
	LTC_ConfigGroupStruct[0].CellDischarge = 0;//LTC_CELL_12;
	LTC_ConfigGroupStruct[0].GPIO_Pins = 0x1F;
	LTC_ConfigGroupStruct[0].Overvoltage = 0x0800;
	LTC_ConfigGroupStruct[0].Undervoltage = 0x0400;
	LTC_ConfigGroupStruct[0].ReferencePwrUp = LTC_Reference_Enable;
	
	LTC_CommGroupStruct1.Data0ICOM = LTC_SPI_ICOM_CSBLOW;
	LTC_CommGroupStruct1.Data0 = 0xFE;
	LTC_CommGroupStruct1.Data0FCOM = LTC_SPI_FCOM_CSBHIGH;
	LTC_CommGroupStruct1.Data1ICOM = LTC_SPI_ICOM_NOTRANSMIT;
	LTC_CommGroupStruct1.Data1 = 0x22;
	LTC_CommGroupStruct1.Data1FCOM = LTC_SPI_FCOM_CSBHIGH;
	LTC_CommGroupStruct1.Data2ICOM = LTC_SPI_ICOM_NOTRANSMIT;
	LTC_CommGroupStruct1.Data2 = 0x33;
	LTC_CommGroupStruct1.Data2FCOM = LTC_SPI_FCOM_CSBHIGH;
	
	LTC_SetCommandAddress(LTC_ADDRESS_BROADCAST);
  LTC_WakeUp();
	LTC_WriteConfigGroup(&LTC_ConfigGroupStruct[0]);	
	LTC_Microdelay(LTC_tREFUP);
	if (LTC_ReadConfigGroup(LTC_ConfigGroupStruct) != LTC_OK) {
	  //while (1) {}
		errorcount++;
	}
	//LTC_ReadConfigGroup(&LTC_ConfigGroupStruct);
  //LTC_WriteCommGroup(&LTC_CommGroupStruct1);	
	//LTC_ReadCommGroup(&LTC_CommGroupStruct2);
	//LTC_ClearStatusGroups();
	
  while (1) {
		
		LTC_WakeUp();
		
		//LTC_StartCellVoltageConversion(LTC_ADCMODE_FILTERED,LTC_DISCHARGE_NOT_PERMITTED,LTC_CELLMEASURE_ALL);
		//LTC_StartSelfTestCellConversion(LTC_ADCMODE_FILTERED, LTC_SELFTEST_2);
		//LTC_StartGpioVoltageConversion(LTC_ADCMODE_FILTERED, LTC_GPIOMEASURE_ALL);
		//LTC_StartSelfTestGpioConversion(LTC_ADCMODE_FILTERED, LTC_SELFTEST_1);
		LTC_StartStatusGroupConversion(LTC_ADCMODE_FILTERED, LTC_STATUSMEASURE_ALL);
		
		LTC_Microdelay(400000);
		LTC_WakeUp();
		LTC_WriteCommGroup(&LTC_CommGroupStruct1);	
		LTC_StartCommunication(1);
		//if (LTC_ReadConfigGroup(LTC_ConfigGroupStruct) != LTC_OK) errorcount++;
		//if (LTC_ReadCellVoltageGroups(VoltageList) != LTC_OK) errorcount++;
		//if (LTC_ReadAuxiliaryGroups(AuxGroup) != LTC_OK) errorcount++;
		if (LTC_ReadStatusGroups(StatusGroup) != LTC_OK) errorcount++;
			
		for (i=0; i<10000000; i++) {}	
	
	}
}



void Init_SPI(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);  /* MOSI */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);  /* MISO */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);  /* SCK */
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
  SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
  SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStruct.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStruct);

  SPI_Cmd(SPI1, ENABLE);
}

void LTC_DRV_SPI_CSLow(void) 
{
	GPIOB->BSRRH = GPIO_Pin_2;
}

void LTC_DRV_SPI_CSHigh(void)
{
  GPIOB->BSRRL = GPIO_Pin_2;
}

void LTC_DRV_SPI_Write(uint8_t data)
{
	/* Wait for transmit buffer empty */
  while (!SPI_I2S_GetFlagStatus(LTC_SPI_PERIPHERAL, SPI_I2S_FLAG_TXE)) {}  
	/* Send the byte through SPI */
	SPI_I2S_SendData(LTC_SPI_PERIPHERAL, data);
	/* Wait while SPI busy */
	while (SPI_I2S_GetFlagStatus(LTC_SPI_PERIPHERAL, SPI_I2S_FLAG_BSY)) {}
}

uint8_t LTC_DRV_SPI_Read(void)
{
	/* Flush the receive FIFO */
	while (SPI_I2S_GetFlagStatus(LTC_SPI_PERIPHERAL, SPI_I2S_FLAG_RXNE)) {
	  LTC_SPI_PERIPHERAL->DR;
	}
	/* Wait until transmit buffer empty */
  while (!SPI_I2S_GetFlagStatus(LTC_SPI_PERIPHERAL, SPI_I2S_FLAG_TXE)) {}
	/* Send dummy data */
	SPI_I2S_SendData(LTC_SPI_PERIPHERAL, 0x00);		
	/* Wait until receive data ready  */
	while (!SPI_I2S_GetFlagStatus(LTC_SPI_PERIPHERAL, SPI_I2S_FLAG_RXNE)) {}
	/* Return the received data */
	return (uint8_t)SPI_I2S_ReceiveData(LTC_SPI_PERIPHERAL);
}


/* END OF FILE */
