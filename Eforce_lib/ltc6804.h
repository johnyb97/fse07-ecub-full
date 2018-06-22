/**
  ******************************************************************************
  * @file    ltc6804.h
  * @author  Jan Sixta
  * @version V0.1
  * @date    30-July-2015
  * @brief   This file contains all the functions prototypes for the LTC6804
  *          firmware library.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 JAN SIXTA </center></h2>
  *
  * Licensed under LICENSE (To Be Done)
	*
  *
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LTC6804_H
#define __LTC6804_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes -------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "ltc6804_conf.h"

/* Public macros --------------------------------------------------------------*/

/* Library compile time cofig constanst - see ltc6804_conf.h */	 
#define LTC_LIBMODE_1           1
#define LTC_LIBMODE_2           2
#define LTC_WRMODE_ALLSAME      0
#define LTC_WRMODE_INDEPENDENT  255
	 
/* Timing in microseconds
   Typical values: 
     tWAKE  = 100us
		 tREFUP = 3500us
	   tREADY = 10us
	   tIDLE  = 5500us
*/
#define LTC_tWAKE      (uint32_t)220
#define LTC_tREFUP     (uint32_t)3500
#define LTC_tREADY     (uint32_t)10
#define LTC_tIDLE      (uint32_t)5500

/* LTC6804 Command Codes */
#define LTC_CC_WRCFG       (uint16_t)0x0001
#define LTC_CC_RDCFG       (uint16_t)0x0002
#define LTC_CC_RDCVA       (uint16_t)0x0004
#define LTC_CC_RDCVB       (uint16_t)0x0006
#define LTC_CC_RDCVC       (uint16_t)0x0008
#define LTC_CC_RDCVD       (uint16_t)0x000A
#define LTC_CC_RDAUXA      (uint16_t)0x000C
#define LTC_CC_RDAUXB      (uint16_t)0x000E
#define LTC_CC_RDSTATA     (uint16_t)0x0010
#define LTC_CC_RDSTATB     (uint16_t)0x0012
#define LTC_CC_ADCV        (uint16_t)0x0260
#define LTC_CC_ADOW        (uint16_t)0x0228
#define LTC_CC_CVST        (uint16_t)0x0207
#define LTC_CC_ADAX        (uint16_t)0x0460
#define LTC_CC_AXST        (uint16_t)0x0407
#define LTC_CC_ADSTAT      (uint16_t)0x0468
#define LTC_CC_STATST      (uint16_t)0x040F
#define LTC_CC_ADCVAX      (uint16_t)0x046F
#define LTC_CC_CLRCELL     (uint16_t)0x0711
#define LTC_CC_CLRAUX      (uint16_t)0x0712
#define LTC_CC_CLRSTAT     (uint16_t)0x0713
#define LTC_CC_PLADC       (uint16_t)0x0714
#define LTC_CC_DIAGN       (uint16_t)0x0715
#define LTC_CC_WRCOMM      (uint16_t)0x0721
#define LTC_CC_RDCOMM      (uint16_t)0x0722
#define LTC_CC_STCOMM      (uint16_t)0x0723

#define LTC_GPIO_1         (uint8_t)0x01
#define LTC_GPIO_2         (uint8_t)0x02
#define LTC_GPIO_3         (uint8_t)0x04
#define LTC_GPIO_4         (uint8_t)0x08
#define LTC_GPIO_5         (uint8_t)0x10
#define LTC_GPIO_ALL       (uint8_t)0x1F
#define LTC_GPIO_NONE      (uint8_t)0x00

#define LTC_Reference_Enable     (uint8_t)0x04
#define LTC_Reference_Disable    (uint8_t)0x00

#define LTC_SWTEN_0              (uint8_t)0x00
#define LTC_SWTEN_1              (uint8_t)0x02

#define LTC_ADC_Option_27_7_26   (uint8_t)0x00
#define LTC_ADC_Option_14_3_2    (uint8_t)0x01

#define LTC_CELL_1         (uint16_t)0x0001
#define LTC_CELL_2         (uint16_t)0x0002
#define LTC_CELL_3         (uint16_t)0x0004
#define LTC_CELL_4         (uint16_t)0x0008
#define LTC_CELL_5         (uint16_t)0x0010
#define LTC_CELL_6         (uint16_t)0x0020
#define LTC_CELL_7         (uint16_t)0x0040
#define LTC_CELL_8         (uint16_t)0x0080
#define LTC_CELL_9         (uint16_t)0x0100
#define LTC_CELL_10        (uint16_t)0x0200
#define LTC_CELL_11        (uint16_t)0x0400
#define LTC_CELL_12        (uint16_t)0x0800
#define LTC_CELL_ALL       (uint16_t)0x0FFF

/* Discharge timer in minutes */
#define LTC_DCTO_DISABLED   (uint8_t)0x00
#define LTC_DCTO_0_5_Min    (uint8_t)0x10
#define LTC_DCTO_1_Min      (uint8_t)0x20
#define LTC_DCTO_2_Min      (uint8_t)0x30
#define LTC_DCTO_3_Min      (uint8_t)0x40
#define LTC_DCTO_4_Min      (uint8_t)0x50
#define LTC_DCTO_5_Min      (uint8_t)0x60
#define LTC_DCTO_10_Min     (uint8_t)0x70
#define LTC_DCTO_15_Min     (uint8_t)0x80
#define LTC_DCTO_20_Min     (uint8_t)0x90
#define LTC_DCTO_30_Min     (uint8_t)0xA0
#define LTC_DCTO_40_Min     (uint8_t)0xB0
#define LTC_DCTO_60_Min     (uint8_t)0xC0
#define LTC_DCTO_75_Min     (uint8_t)0xD0
#define LTC_DCTO_90_Min     (uint8_t)0xE0
#define LTC_DCTO_120_Min    (uint8_t)0xF0

/* Undervoltage and overvoltage status flags */
#define LTC_CELL1_UV        (uint32_t)0x00000001
#define LTC_CELL1_OV        (uint32_t)0x00000002
#define LTC_CELL2_UV        (uint32_t)0x00000004
#define LTC_CELL2_OV        (uint32_t)0x00000008
#define LTC_CELL3_UV        (uint32_t)0x00000010
#define LTC_CELL3_OV        (uint32_t)0x00000020
#define LTC_CELL4_UV        (uint32_t)0x00000040
#define LTC_CELL4_OV        (uint32_t)0x00000080
#define LTC_CELL5_UV        (uint32_t)0x00000100
#define LTC_CELL5_OV        (uint32_t)0x00000200
#define LTC_CELL6_UV        (uint32_t)0x00000400
#define LTC_CELL6_OV        (uint32_t)0x00000800
#define LTC_CELL7_UV        (uint32_t)0x00001000
#define LTC_CELL7_OV        (uint32_t)0x00002000
#define LTC_CELL8_UV        (uint32_t)0x00004000
#define LTC_CELL8_OV        (uint32_t)0x00008000
#define LTC_CELL9_UV        (uint32_t)0x00010000
#define LTC_CELL9_OV        (uint32_t)0x00020000
#define LTC_CELL10_UV       (uint32_t)0x00040000
#define LTC_CELL10_OV       (uint32_t)0x00080000
#define LTC_CELL11_UV       (uint32_t)0x00100000
#define LTC_CELL11_OV       (uint32_t)0x00200000
#define LTC_CELL12_UV       (uint32_t)0x00400000
#define LTC_CELL12_OV       (uint32_t)0x00800000

/* ADC Modes (measurement speeds) */
#define LTC_ADCMODE_FAST      (uint16_t)0x0080
#define LTC_ADCMODE_NORMAL    (uint16_t)0x0100
#define LTC_ADCMODE_FILTERED  (uint16_t)0x0180

/* Discharge permission during ADC conversion */
#define LTC_DISCHARGE_PERMITTED      (uint16_t)0x0010
#define LTC_DISCHARGE_NOT_PERMITTED  (uint16_t)0x0000

/* ADCV cell voltage measure command - cell select */
#define LTC_CELLMEASURE_ALL   (uint16_t)0x0000
#define LTC_CELLMEASURE_1_7   (uint16_t)0x0001
#define LTC_CELLMEASURE_2_8   (uint16_t)0x0002
#define LTC_CELLMEASURE_3_9   (uint16_t)0x0003
#define LTC_CELLMEASURE_4_10  (uint16_t)0x0004
#define LTC_CELLMEASURE_5_11  (uint16_t)0x0005
#define LTC_CELLMEASURE_6_12  (uint16_t)0x0006

/* Selftest selection */
#define LTC_SELFTEST_1        (uint16_t)0x0020
#define LTC_SELFTEST_2        (uint16_t)0x0040

/* GPIO measurement selection */
#define LTC_GPIOMEASURE_ALL   (uint16_t)0x0000
#define LTC_GPIOMEASURE_1     (uint16_t)0x0001
#define LTC_GPIOMEASURE_2     (uint16_t)0x0002
#define LTC_GPIOMEASURE_3     (uint16_t)0x0003
#define LTC_GPIOMEASURE_4     (uint16_t)0x0004
#define LTC_GPIOMEASURE_5     (uint16_t)0x0005
#define LTC_GPIOMEASURE_REF2  (uint16_t)0x0006

/* Status Group mesaurement selection */
#define LTC_STATUSMEASURE_ALL  (uint16_t)0x0000
#define LTC_STATUSMEASURE_SOC  (uint16_t)0x0001
#define LTC_STATUSMEASURE_ITMP (uint16_t)0x0002
#define LTC_STATUSMEASURE_VA   (uint16_t)0x0003
#define LTC_STATUSMEASURE_VD   (uint16_t)0x0004

/* Communication control through GPIO pins of LTC6804 */
#define LTC_SPI_ICOM_CSBLOW       (uint8_t)0x80
#define LTC_SPI_ICOM_CSBHIGH      (uint8_t)0x90
#define LTC_SPI_ICOM_NOTRANSMIT   (uint8_t)0xF0
#define LTC_SPI_ICOM_READ         (uint8_t)0x70
#define LTC_SPI_FCOM_CSBLOW       (uint8_t)0x08
#define LTC_SPI_FCOM_CSBHIGH      (uint8_t)0x09
#define LTC_SPI_FCOM_READ         (uint8_t)0x0F

#define LTC_I2C_ICOM_START         (uint8_t)0x60
#define LTC_I2C_ICOM_STOP          (uint8_t)0x10
#define LTC_I2C_ICOM_BLANK         (uint8_t)0x00
#define LTC_I2C_ICOM_NOTRANSMIT    (uint8_t)0x70
#define LTC_I2C_ICOM_SDALOW        (uint8_t)0x00
#define LTC_I2C_ICOM_SDAHIGH       (uint8_t)0x70
#define LTC_I2C_FCOM_MASTERACK     (uint8_t)0x00
#define LTC_I2C_FCOM_MASTERNACK    (uint8_t)0x08
#define LTC_I2C_FCOM_MSTRNACKSTOP  (uint8_t)0x09
#define LTC_I2C_FCOM_ACKFROMSLAVE  (uint8_t)0x07
#define LTC_I2C_FCOM_NACKFROMSLAVE (uint8_t)0x0F
#define LTC_I2C_FCOM_ACKSLAVESTOP  (uint8_t)0x01
#define LTC_I2C_FCOM_NACKSLAVESTOP (uint8_t)0x09

/* Open wire measurement pull up/down constants */
#define LTC_PUPDN_UP               (uint16_t)0x0040
#define LTC_PUPDN_DOWN             (uint16_t)0x0000

/* Command addresses definition constants */
#define LTC_ADDRESS_BROADCAST      (uint8_t)0x00
#if (LTC_LIBRARY_MODE==LTC_LIBMODE_2)
  #define LTC_ADDRESS_0            (uint8_t)(0x80|(0<<3))
  #define LTC_ADDRESS_1            (uint8_t)(0x80|(1<<3))
  #define LTC_ADDRESS_2            (uint8_t)(0x80|(2<<3))
  #define LTC_ADDRESS_3            (uint8_t)(0x80|(3<<3))
  #define LTC_ADDRESS_4            (uint8_t)(0x80|(4<<3))
  #define LTC_ADDRESS_5            (uint8_t)(0x80|(5<<3))
  #define LTC_ADDRESS_6            (uint8_t)(0x80|(6<<3))
  #define LTC_ADDRESS_7            (uint8_t)(0x80|(7<<3))
  #define LTC_ADDRESS_8            (uint8_t)(0x80|(8<<3))
  #define LTC_ADDRESS_9            (uint8_t)(0x80|(9<<3))
  #define LTC_ADDRESS_10           (uint8_t)(0x80|(10<<3))
  #define LTC_ADDRESS_11           (uint8_t)(0x80|(11<<3))
  #define LTC_ADDRESS_12           (uint8_t)(0x80|(12<<3))
  #define LTC_ADDRESS_13           (uint8_t)(0x80|(13<<3))
  #define LTC_ADDRESS_14           (uint8_t)(0x80|(14<<3))
  #define LTC_ADDRESS_15           (uint8_t)(0x80|(15<<3))
	#define LTC_ADDRESS_VAL(adr)     (uint8_t)(0x80|((adr)<<3))
#endif


/* Microsecond delay macro */
#define LTC_Microdelay(us)  ltc_tickdelay((us)*(uint64_t)F_HCLK/6000000UL)

/* Public typedef -------------------------------------------------------------*/

/* LTC6804 result type definition */
typedef enum {

	LTC_OK=0,                   /* Everything is OK, everyone is happy */
	LTC_ERROR,                  /* General error occured */
	LTC_PECFAIL,                /* PEC calculation failed - eg wrong data received */
	LTC_TIMEOUT                 /* Ooops, time is out */               
	
} LTC_ResultTypeDef;             

/* LTC6804 Configuration Register Group */
typedef struct {  
	
	uint8_t GPIO_Pins;          /* Read or write state of GPIO pins LTC_GPIO_x */
	uint8_t ReferencePwrUp;	    /* Reference power up: LTC_Reference_Enable or Disable */
	uint8_t SWTEN_PinStatus;    /* Readonly SWTEN pin status: LTC_SWTEN_0 or LTC_SWTEN_1 */
	uint8_t ADC_ModeOption;     /* Selects ADC bandwidth: LTC_ADC_Option_27_7_26 or LTC_ADC_Option_14_3_2 */
	uint16_t Undervoltage;      /* Sets undervoltage threshold (VUV+1)*16*100uV (default 0x000, max 0xFFF) */
	uint16_t Overvoltage;       /* Sets undervoltage threshold VOV*16*100uV (default 0x000, max 0xFFF) */
	uint16_t CellDischarge;     /* Selects cells to be discharged: LTC_CELL_x */
	uint8_t  DischargeTime;     /* Discharge timer select: LTC_DCTO_x */
	
} LTC_ConfigGroupTypeDef;


/* LTC6804 Cell Voltage List */
typedef struct {
  
	uint16_t Cell[12];         /* Array of 12 cell voltages in counts of 100uV, 16bit precision */
	
} LTC_CellVoltageListTypeDef;


/* LTC6804 Auxiliary Group Register List */
typedef struct {
	
	uint16_t GPIO_AnalogVoltage[5];  /* Array of 5 analog voltages present on GPIO1 to GPIO5 represented
	                                    in 100uV increments, 16bit precision */
	uint16_t SecondReference;        /* Voltage of second voltage reference in 100uV increments, 16bit precision 
	                                    Normal range is within 2.985V to 3.015V */

} LTC_AuxiliaryGroupTypeDef;


/* LTC6804 Status Group Register List */
typedef struct {

	uint16_t SumOfCells;           /* Sum of all cell voltages in 2mV counts, 16bit precision */
	uint16_t DieTemperature;       /* 16-Bit ADC Measurement Value of Internal Die Temperature
	                                  Degrees C = DieTemperature*100uV/7.5mV–273 */
	uint16_t AnalogSupplyVoltage;  /* 16-Bit ADC measurement value of analog power supply voltage, 100uV counts,
		                                Normal range is within 4.5V to 5.5V */
	uint16_t DigitalSupplyVoltage; /* 16-Bit ADC measurement Value of digital power supply voltage, 100uV counts,
		                                Normal range is within 2.7V to 3.6V */
	uint32_t UVOV_StatusFlags;     /* Holds 12 undervoltage and 12 overvoltage flags for cells 1 to 12 
	                                  Use together with LTC_CELLx_UV and LTC_CELLx_OV */
	uint8_t RevisionCode;          /* 4bit revision  code of the LTC6804 chip */
	uint8_t MuxFail;               /* Multiplexer self-test result. Positive if failed, zero otherwise */
	uint8_t ThermalShutdown;       /* Positive number if thermal shutdown occured, zero otherwise */
	
} LTC_StatusGroupTypeDef;


/* LTC6804 GPIO Communication Group */
typedef struct {
	
	uint8_t Data0ICOM;             /* Data byte #0 initial communication control */
	uint8_t Data0;                 /* Data byte #0 to be read or written to the COMM register group*/
	uint8_t Data0FCOM;             /* Data byte #0 final communication control setting */
	uint8_t Data1ICOM;             /* Data byte #1 initial communication control */
	uint8_t Data1;                 /* Data byte #1 to be read or written to the COMM register group*/
	uint8_t Data1FCOM;             /* Data byte #1 final communication control setting */
	uint8_t Data2ICOM;             /* Data byte #2 initial communication control */
	uint8_t Data2;                 /* Data byte #2 to be read or written to the COMM register group*/
	uint8_t Data2FCOM;             /* Data byte #2 final communication control setting */

} LTC_CommGroupTypeDef;


/* Public variables -----------------------------------------------------------*/

/* Public functions -----------------------------------------------------------*/

/* The abbreviation in the comment after function prototype 
   denotes the corresponding LTC6804 command the function uses */

void LTC_Init(void);
void LTC_WakeUp(void);
LTC_ResultTypeDef LTC_WriteConfigGroup(LTC_ConfigGroupTypeDef *config); /* WRCFG */
LTC_ResultTypeDef LTC_ReadConfigGroup(LTC_ConfigGroupTypeDef *config); /* RDCFG */
LTC_ResultTypeDef LTC_ReadCellVoltageGroups(LTC_CellVoltageListTypeDef *list); /* RDCVA..RDCVD */
LTC_ResultTypeDef LTC_ReadAuxiliaryGroups(LTC_AuxiliaryGroupTypeDef *auxgroup); /* RDAUXA, RDAUXB */
LTC_ResultTypeDef LTC_ReadAuxiliaryGroupA(LTC_AuxiliaryGroupTypeDef *auxgroup); /* RDAUXA */
LTC_ResultTypeDef LTC_ReadAuxiliaryGroupB(LTC_AuxiliaryGroupTypeDef *auxgroup); /* RDAUXB */
LTC_ResultTypeDef LTC_ReadStatusGroups(LTC_StatusGroupTypeDef *status); /* RDSTATA, RDSTATB */
LTC_ResultTypeDef LTC_ReadStatusGroupA(LTC_StatusGroupTypeDef *status); /* RDSTATA */
LTC_ResultTypeDef LTC_ReadStatusGroupB(LTC_StatusGroupTypeDef *status); /* RDSTATB */
LTC_ResultTypeDef LTC_ClearCellVoltageGroups(void); /* CLRCELL */
LTC_ResultTypeDef LTC_ClearAuxiliaryGroups(void); /* CLRAUX */
LTC_ResultTypeDef LTC_ClearStatusGroups(void); /* CLRSTAT */
LTC_ResultTypeDef LTC_StartCellVoltageConversion(uint16_t ADC_Mode, uint16_t Discharge, uint16_t CellMeasure); /* ADCV */
LTC_ResultTypeDef LTC_StartSelfTestCellConversion(uint16_t ADC_Mode, uint16_t Selftest); /* CVST */
LTC_ResultTypeDef LTC_StartGpioVoltageConversion(uint16_t ADC_Mode, uint16_t GpioMeasure); /* ADAX */
LTC_ResultTypeDef LTC_StartSelfTestGpioConversion(uint16_t ADC_Mode, uint16_t Selftest); /* AXST */
LTC_ResultTypeDef LTC_StartStatusGroupConversion(uint16_t ADC_Mode, uint16_t StatusMeasure); /* ADSTAT  */
LTC_ResultTypeDef LTC_StartSelfTestStatusConversion(uint16_t ADC_Mode, uint16_t Selftest); /* STATST */
LTC_ResultTypeDef LTC_PollStatusADC(void); /* PLADC */
LTC_ResultTypeDef LTC_StartCombinedConversion(uint16_t ADC_Mode, uint16_t Discharge); /* ADCVAX */
LTC_ResultTypeDef LTC_WriteCommGroup(LTC_CommGroupTypeDef *CommGroup); /* WRCOMM */
LTC_ResultTypeDef LTC_ReadCommGroup(LTC_CommGroupTypeDef *CommGroup); /* RDCOMM */
LTC_ResultTypeDef LTC_StartCommunication(uint8_t BytesToTransmit); /* STCOMM */
LTC_ResultTypeDef LTC_StartOpenWireConversion(uint16_t ADC_Mode, 
	                  uint16_t Discharge, uint16_t PUpDown, uint16_t CellMeasure); /* ADOW */
void LTC_SetCommandAddress(uint8_t Address);

/* Low level function handling communication with LTC6804 through SPI */
/* Note: These functions should be defined by the user! */
void LTC_DRV_SPI_Write(uint8_t data);
uint8_t LTC_DRV_SPI_Read(void);
void LTC_DRV_SPI_CSLow(void);
void LTC_DRV_SPI_CSHigh(void);

/* Utility functions */
uint16_t pec15(uint8_t *data, uint16_t len);
__asm void ltc_tickdelay(uint32_t ticks);


#ifdef __cplusplus
}
#endif

#endif /*__LTC6804_H */

/* END OF FILE */

