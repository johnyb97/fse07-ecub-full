#include "carstate.h"

#define CS_NOT_READY_LOCK_TIME			3000 // ms time to do nothing after error(notready)
#define CS_BSPD_LATCH_WRITING_DELAY		200 // ms countdown after crititcal error
#define LED_blink 100 //ms for bling led
#define SDC_ECUF_OFFSET 8 //bit ofset for SDC data ECUF (front)
#define SDC_SDBL_OFFSET 9 //bit ofset for SDC data SDBL (shut down buton left)
#define SDC_SDBR_OFFSET 10 //bit ofset for SDC data SDBR (shut down buton right)
#define SDC_ECUMCR_OFFSET 11 //bit ofset for SDC data ECUMCR (back motor controler)
#define SDC_ECUA_OFFSET 12 //bit ofset for SDC data ECUA (accupack)
#define SDC_HVD_OFFSET 13 //bit ofset for SDC data HVD (high voltage disconect)
#define SDC_BSPD_OFFSET 14 //bit ofset for SDC data BSPD (Brake System Plausibility Device)
#define SDC_TSMS_OFFSET 15 //bit ofset for SDC data TSMS (Tractive System Master Switch)

static ECUB_Status_t				ECUB_Status; //can message
static ECUB_Power_dist_t		ECUB_Power; // can message
static enum ECUB_CarState 					state; //car state 
static enum ECUB_Notready_reason		state_notready;  //reason for notready car state
static enum ECUB_Batt_code					batt_code; //state battery (charging)
//enum ECUB_Power_code				pow_code; //error pow now

static uint32_t			cs_not_ready_lock = 0; //time to do nothing after error (notready)
static uint32_t			cs_latch_write_delay = 0; //time to fix critical error or latch happends
static uint16_t			SDC_measure; 
static uint8_t				SDC_error;
static uint32_t			LED_blink_time; //time of last led blink
static uint32_t			number_of_bliks; //number of blinks

static ECUP_Status_t ECUP_data; //data form can status ECUP
static ECUF_Dashboard_t ECUF_Dash_data; //data from can ECUF dashboard
static ECUF_Status_t ECUF_Status_data; //data from can ECUF dashboard
static ECUA_Status_t ECUA_data; //data from can ECUA


enum ECUB_CarState* get_state(){ // gives current car state outside carstate.c
	return &state;
}

ECUB_Status_t* get_can_state(){
	return &ECUB_Status;
}

int measure_SDC(SPI_HandleTypeDef * SPI_handle){ //check SDC circuit
	HAL_GPIO_WritePin(SDC_CS_GPIO_Port,SDC_CS_Pin,GPIO_PIN_RESET); //starts comunication...enable pin
	HAL_SPI_TransmitReceive(SPI_handle,(uint8_t*)&SDC_measure,(uint8_t*)&SDC_measure,1,5); //recives 16bit number from SDC chip
	HAL_GPIO_WritePin(SDC_CS_GPIO_Port,SDC_CS_Pin,GPIO_PIN_SET); //stops comunication...enable pin
	SDC_error = 1;

	if (!(SDC_measure & (1 << SDC_ECUF_OFFSET))){ // if ECUF bit is 1
		ECUB_Status.SDC_FRONT = 0; //can message...reason for SDC disconect
		change_to_NOT_READY(ECUB_Notready_reason_SDC_FAILURE); //change state because of critical error report
		SDC_error = 1; //error find
	}else{
		ECUB_Status.SDC_FRONT = 1; //can message...reason for SDC disconect
	}
	
	if (!(SDC_measure & (1 << SDC_SDBL_OFFSET))){ // if SDBL bit is 1
		ECUB_Status.SDC_SDBL = 0; //can message...reason for SDC disconect
		change_to_NOT_READY(ECUB_Notready_reason_SDC_FAILURE); //change state because of critical error report
		SDC_error = 1; //error find
	}else{
		ECUB_Status.SDC_SDBL = 1; //can message...reason for SDC disconect
	}
	
	if (!(SDC_measure & (1 << SDC_SDBR_OFFSET))){ // if SDBR bit is 1
		ECUB_Status.SDC_SDBR = 0; //can message...reason for SDC disconect
		change_to_NOT_READY(ECUB_Notready_reason_SDC_FAILURE); //change state because of critical error report
		SDC_error = 1; //error find
	}else{
		ECUB_Status.SDC_SDBR = 1; //can message...reason for SDC disconect
	}
	
	if (!(SDC_measure & (1 << SDC_ECUMCR_OFFSET))){ // if ECUMCR bit is 1
		ECUB_Status.SDC_MCUR = 0; //can message...reason for SDC disconect
		change_to_NOT_READY(ECUB_Notready_reason_SDC_FAILURE); //change state because of critical error report
		SDC_error = 1; //error find
	}else{
		ECUB_Status.SDC_MCUR = 1; //can message...reason for SDC disconect
	}
	
	if (!(SDC_measure & (1 << SDC_ECUA_OFFSET))){ // if ECUA bit is 1
		ECUB_Status.SDC_AMS = 0; //can message...reason for SDC disconect
		change_to_NOT_READY(ECUB_Notready_reason_SDC_FAILURE); //change state because of critical error report
		SDC_error = 1; //error find
	}else{
		ECUB_Status.SDC_FRONT = 1; //can message...reason for SDC disconect
	}
	
	if (!(SDC_measure & (1 << SDC_HVD_OFFSET))){ // if HVD bit is 1
		ECUB_Status.SDC_HVD = 0; //can message...reason for SDC disconect
		change_to_NOT_READY(ECUB_Notready_reason_SDC_FAILURE); //change state because of critical error report
		SDC_error = 1; //error find
	}else{
		ECUB_Status.SDC_HVD = 1; //can message...reason for SDC disconect
	}
	
	if (!(SDC_measure & (1 << SDC_BSPD_OFFSET))){ // if BSPD bit is 1
		ECUB_Status.SDC_BSPD = 0; //can message...reason for SDC disconect
		change_to_NOT_READY(ECUB_Notready_reason_SDC_FAILURE); //change state because of critical error report
		SDC_error = 1; //error find
	}else{
		ECUB_Status.SDC_BSPD = 1; //can message...reason for SDC disconect
	}
	
	if (!(SDC_measure & (1 << SDC_TSMS_OFFSET))){ // if TSMS bit is 1
		ECUB_Status.SDC_TSMS = 0; //can message...reason for SDC disconect
		change_to_NOT_READY(ECUB_Notready_reason_SDC_FAILURE); //change state because of critical error report
		SDC_error = 1; //error find
	}else{
		ECUB_Status.SDC_TSMS = 1; //can message...reason for SDC disconect
	}
	
	if (SDC_error){ //if any error found
		return 1; //return something went wrong
	} 
	return 0; //return all ok
}


void change_to_NOT_READY(enum ECUB_Notready_reason reason){ //change car state to NOT READY and set time in witch it must stay in notready
	state_notready = reason; //duvod not ready state
	state = ECUB_CarState_NOT_READY; //set car state
	cs_not_ready_lock = HAL_GetTick() + CS_NOT_READY_LOCK_TIME; //set time in NOTREADY after error
	if (HAL_GPIO_ReadPin(Fan2_GPIO_Port,Fan2_Pin) == GPIO_PIN_SET){ // if air conditioning still running
		HAL_GPIO_WritePin(WP1_GPIO_Port,WP1_Pin,GPIO_PIN_RESET); //stops water pumps for air conditioning :(
		HAL_GPIO_WritePin(WP2_GPIO_Port,WP2_Pin,GPIO_PIN_RESET); //stops water pumps for air conditioning :(
		HAL_GPIO_WritePin(Fan1_GPIO_Port,Fan1_Pin,GPIO_PIN_RESET); //stops fans for air conditioning :(
		HAL_GPIO_WritePin(Fan2_GPIO_Port,Fan2_Pin,GPIO_PIN_RESET); //stops fans for air conditioning :(	
	}
	set_SDB_led(GPIO_PIN_RESET,&ECUB_Status); //lights SDB leds on
}


void set_latch_countdown(enum ECUB_Notready_reason reason){ //set car state to NOTREADY and set latch countdown
	state_notready = reason; //reason for critical erroe
	state = ECUB_CarState_NOT_READY; //set car state
	if (!cs_latch_write_delay){
		cs_latch_write_delay = HAL_GetTick() + CS_BSPD_LATCH_WRITING_DELAY; //set countdown to fix crititcal error 
	}
}


int	carstate_init(void){ //car state at the begining
	state = ECUB_CarState_NOT_READY; //car state at the begining
	state_notready = ECUB_Notready_reason_NONE; //no error right after start of unit	
	cs_not_ready_lock = 0;
	cs_latch_write_delay = 0;
	return 1;
}


void check_mess(void){ //check if all messages form other units are being recived
	if(!ECUA_get_Status(&ECUA_data)){ //check if recived messsage and if did, copyed into given structure
		change_to_NOT_READY(ECUB_Notready_reason_TIMEOUT_ECUA); //reason not ready state
	}
	if(!ECUF_get_Status(&ECUF_Status_data)){ //check if recived messsage and if did, copyed into given structure
		change_to_NOT_READY(ECUB_Notready_reason_TIMEOUT_ECUF); //reason not ready state
	}
	if(!ECUF_get_Dashboard(NULL)){ //check if recived messsage and if did, copyed into given structure
		change_to_NOT_READY(ECUB_Notready_reason_TIMEOUT_ECUF); //reason not ready state
	}
	if(!ECUP_get_Status(&ECUP_data)){ //check if recived messsage and if did, copyed into given structure
		change_to_NOT_READY(ECUB_Notready_reason_TIMEOUT_ECUP); //reason not ready state
	}
	if(!MCR_get_GeneralReport(NULL)){ //check if recived messsage and if did, copyed into given structure
		change_to_NOT_READY(ECUB_Notready_reason_TIMEOUT_MC); //reason not ready state
	}
	if(!VDCU_get_Status(NULL)){ //check if recived messsage and if did, copyed into given structure
		change_to_NOT_READY(ECUB_Notready_reason_TIMEOUT_VDCU); //reason not ready state
	}
	if(!MCF_get_GeneralReport(NULL)){ //check if recived messsage and if did, copyed into given structure
		change_to_NOT_READY(ECUB_Notready_reason_TIMEOUT_MC); //reason not ready state
	}
}



void blink_led(uint32_t number_of_blinks){
	if (number_of_blinks>number_of_bliks){
		if (LED_blink_time<HAL_GetTick()){
			if(HAL_GPIO_ReadPin(LED5_GPIO_Port,LED5_Pin)==GPIO_PIN_RESET){ //debug LED
				HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_SET); //debug LED
			}else{
				HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_RESET); //debug LED
			}
			LED_blink_time = HAL_GetTick() + LED_blink; //sets time in led state
			number_of_bliks++;
		}
	}else{
		LED_blink_time = HAL_GetTick() + LED_blink*10; //time of led time out 
		number_of_bliks = 0;
	}
}

void carstate_process(SPI_HandleTypeDef * SPI_handle,CAN_HandleTypeDef* hcan){ //main car state machine
	if (measure_SDC(SPI_handle)){ //measure if any error on SDC circute
		HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
	}
	
	
	check_mess(); //check that messages are being recived
	if(ECUA_data.AMSState != ECUA_StateAMS_All_OK){ //if AMS critical error report
		set_latch_countdown(ECUB_Notready_reason_LATCH_AMS); //change state because of critical error report
	}
	if(ECUP_data.BPPC_Latch){ //if BSPD critical error report
		set_latch_countdown(ECUB_Notready_reason_LATCH_BSPD); //change state because of critical error report
	}
	if (!ECUF_Status_data.SDC_SDBC){
		change_to_NOT_READY(ECUB_Notready_reason_SDC_FAILURE);
	}
	if (!ECUF_Status_data.SDC_FWIL){
		change_to_NOT_READY(ECUB_Notready_reason_SDC_FAILURE);
	}
	if (!ECUF_Status_data.SDC_Inertia){
		change_to_NOT_READY(ECUB_Notready_reason_SDC_FAILURE);
	}
	if (cs_latch_write_delay){ //in coundown for latch critical error
		if ((state_notready == ECUB_Notready_reason_LATCH_BSPD)&&(!ECUP_data.BPPC_Latch)){ //if error fixed
			cs_latch_write_delay = 0; //stop countdown
		}
		if ((state_notready == ECUB_Notready_reason_LATCH_AMS)&&(ECUA_data.AMSState == ECUA_StateAMS_All_OK)){ //if error fixed
			cs_latch_write_delay = 0; //stop countdown
		}
		if(cs_latch_write_delay<HAL_GetTick()){
			state = ECUB_CarState_LATCHED; //krizovy stav auta
			goto there_is_no_escape;
		}
	}
	switch (state){ //main state automat
		
		
		case ECUB_CarState_NOT_READY: //starting and error state
			blink_led(3); //sets to 2bliks period of debug led
			if(cs_latch_write_delay){ 
					break;//if in coundown do nothing
			}
			if(cs_not_ready_lock){ //if in countdown for error
				if(HAL_GetTick() < cs_not_ready_lock){ //if countdown not finished
					break; //do nothing
				}
				else {cs_not_ready_lock=0;} //end error state
			}				
			if(units_set(GPIO_PIN_SET,&ECUB_Status)){ //if all units have power
				state = ECUB_CarState_TS_READY; //set car state
			}
			set_SDB_led(GPIO_PIN_SET,&ECUB_Status); //lights SDB leds on
			state_notready = ECUB_Notready_reason_NONE; //error message delated 
			break;
			
			
		case ECUB_CarState_LATCHED: //critical error...only way to get from here is to shut down the car
			there_is_no_escape:
			set_SDB_led(GPIO_PIN_RESET,&ECUB_Status); //sets SDB leds off
			units_set(GPIO_PIN_RESET,&ECUB_Status); //stops power to units
			aux_set(GPIO_PIN_RESET,&ECUB_Status); //stop power to aux
			if (HAL_GPIO_ReadPin(Fan2_GPIO_Port,Fan2_Pin) == GPIO_PIN_SET){ // if air conditioning still running
				HAL_GPIO_WritePin(WP1_GPIO_Port,WP1_Pin,GPIO_PIN_RESET); //stops water pumps for air conditioning :(
				HAL_GPIO_WritePin(WP2_GPIO_Port,WP2_Pin,GPIO_PIN_RESET); //stops water pumps for air conditioning :(
				HAL_GPIO_WritePin(Fan1_GPIO_Port,Fan1_Pin,GPIO_PIN_RESET); //stops fans for air conditioning :(
				HAL_GPIO_WritePin(Fan2_GPIO_Port,Fan2_Pin,GPIO_PIN_RESET); //stops fans for air conditioning :(	
			}
			if (ECUB_Status_need_to_send()){ //if is needed to send can message
				ECUB_Status.CarState = state; //put state of car to can message
				ECUB_Status.CarState_Notready = state_notready; //put NOTREADY reason to can message
				ECUB_send_Status_s(&ECUB_Status); //sendidng can message
				ECUB_Status.SEQ++; //message just for check that this message is different from last one
			}
			goto there_is_no_escape;
		
		
		case ECUB_CarState_TS_READY: //tractive system on ready...state needed for startitng procedure
			blink_led(5); //sets to 3bliks period of debug led
			if(ECUF_get_Dashboard(&ECUF_Dash_data)){ //check if recived messsage and if did, copyed into given structure
				if (ECUF_Dash_data.TSON ==1){ //if preded button tractive system on
					HAL_GPIO_WritePin(WP1_GPIO_Port,WP1_Pin,GPIO_PIN_SET); //starts water pumps for air conditioning
					HAL_GPIO_WritePin(WP2_GPIO_Port,WP2_Pin,GPIO_PIN_SET); //starts water pumps for air conditioning
					HAL_GPIO_WritePin(Fan1_GPIO_Port,Fan1_Pin,GPIO_PIN_SET); //starts fans for air conditioning
					HAL_GPIO_WritePin(Fan2_GPIO_Port,Fan2_Pin,GPIO_PIN_SET); //starts fans for air conditioning
					state = ECUB_CarState_PRECHARGE; //set car state
				}
			}else{
				change_to_NOT_READY(ECUB_Notready_reason_TIMEOUT_ECUF); //change car state to notready
			}
			break;
			
			
		case ECUB_CarState_PRECHARGE: //precharge condenser if i got it right
			blink_led(7); //sets to 4bliks period of debug led			
			if ((!ECUA_data.FT_ANY)&&(!ECUA_data.AMSState)&&(ECUA_data.AIRsState)){ //mising precharge finished
				state = ECUB_CarState_TS_ON; //set car state
			}				
			break;
				
		case ECUB_CarState_TS_ON: //ready for start
			blink_led(9); //sets to 5bliks period of debug led
			ECUP_get_Status(&ECUP_data);
			ECUF_get_Dashboard(&ECUF_Dash_data);
			if (ECUF_Dash_data.START && ECUP_data.BrakeActive){ // must be presed start button and brake pedal in the same time
				state = ECUB_CarState_WAITING_FOR_RTDS; //set car state
			}
			break;
			
		case ECUB_CarState_WAITING_FOR_RTDS: //playing RTDS
			blink_led(11); //sets to 6bliks period of debug led
			ECUB_Status.RTDS_EN = 1; //play RDTS can message
			if(play_RTDS()){ //if RDTS stops
				ECUB_Status.RTDS_EN = 0; //RDTS don´t plat can message
					state = ECUB_CarState_STARTED; //set car state
			}
			break;
			
		case ECUB_CarState_STARTED: //car driven...end possible only using SDC or error
			blink_led(13); //sets to 7bliks period of debug led
			brakelightprocess(ECUP_data,&ECUB_Status); //check brake light
			break;
		
		
		default:
			state = ECUB_CarState_NOT_READY; //should never happen but just to be sure
			break;
		}
	
		if (ECUB_Status_need_to_send()){ //if is needed to send can message
			ECUB_Status.CarState = state; //put state of car to can message
			ECUB_Status.CarState_Notready = state_notready; //put NOTREADY reason to can message
 		  ECUB_send_Status_s(&ECUB_Status); //sendidng can message
			ECUB_Status.SEQ++; //message just for check that this message is different from last one
			HAL_CAN_Receive_IT(hcan,CAN_FIFO0);
		}
}
