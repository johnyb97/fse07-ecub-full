/**
  ******************************************************************************
  * @file    ltc6804_conf.h
  * @author  Jan Sixta
  * @version V0.1
  * @date    31-July-2015
  * @brief   This file contains configuration of the LTC6804 firmware library.
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

#include "stm32f1xx_hal.h"
#include "ltc6804.h"

/* Cortex M3 core frequency in Hz */
#define F_HCLK                  168000000UL

/* Library mode configuration:
     LTC_LIBMODE_1 for LTC6804-1 devices 
	   LTC_LIBMODE_2 for LTC6804-2 devices
	 If the library is in mode 2, the LTC_CHAIN_LENGTH should be configured to 1
*/
#define LTC_LIBRARY_MODE             LTC_LIBMODE_1

/* Number of daisy-chained devices. If the library is in mode 2, the chain
   length should be set to 1. Otherwise it won't work! */
#define LTC_CHAIN_LENGTH             1

/* If the library is in the 6804-1 mode (daisy-chained devices), you can select
   in between two modes of writing data into the device chain: 
	  + Mode LTC_WRMODE_ALLSAME where all devices in the chain are written 
		  the same data. That means the functions writing data to the chain require
			only pointer to a single structure of data to be written.
		+ Mode LTC_WRMODE_INDEPENDENT where each device of the chain can be written
		  with independent data. Every data writing function therefore needs
			an array of data structures. The array is of the length as the chain.
*/
#define LTC_CHAIN_WRITEMODE          LTC_WRMODE_ALLSAME


/* END OF FILE */
