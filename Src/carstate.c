#include "carstate.h"


#define CS_NOT_READY_LOCK_TIME			3000 // ms time to do nothing after error(notready)
#define CS_BSPD_LATCH_WRITING_DELAY		200 // ms countdown after crititcal error
#define LED_blink 100 //ms for bling led
#define LED_SDC_BLINK 200 //blinking led for SDC
#define SDC_ECUF_OFFSET 8 //bit ofset for SDC data ECUF (front)
#define SDC_SDBL_OFFSET 9 //bit ofset for SDC data SDBL (shut down buton left)
#define SDC_SDBR_OFFSET 10 //bit ofset for SDC data SDBR (shut down buton right)
#define SDC_ECUMCR_OFFSET 11 //bit ofset for SDC data ECUMCR (back motor controler)
#define SDC_ECUA_OFFSET 12 //bit ofset for SDC data ECUA (accupack)
#define SDC_HVD_OFFSET 13 //bit ofset for SDC data HVD (high voltage disconect)
#define SDC_BSPD_OFFSET 15 //bit ofset for SDC data BSPD (Brake System Plausibility Device)
#define SDC_TSMS_OFFSET 14 //bit ofset for SDC data TSMS (Tractive System Master Switch)

static ECUB_Status_t				ECUB_Status; //can message
static enum ECUB_CarState 					state; //car state 
static enum ECUB_Notready_reason		state_notready;  //reason for notready car state
//enum ECUB_Power_code				pow_code; //error pow now

static uint32_t			cs_not_ready_lock = 0; //time to do nothing after error (notready)
static uint32_t			cs_latch_write_delay = 0; //time to fix critical error or latch happends
static uint16_t			SDC_measure; 
static uint8_t			SDC_error;
static uint32_t			LED_blink_time; //time of last led blink
static uint32_t			number_of_bliks; //number of blinks
uint8_t							parity_check_summ; //parity check if SDC measure correctly recived  
uint8_t							parity_for; //number used in for cycle inside parity check
uint8_t							SDC_parity_control; //number used in for cycle inside parity check
uint8_t							TS_ON_presed;
uint32_t						TSMS_set_countdown;
#define							TSMS_set_timeout 1000;

uint32_t							LV_low_countdown; //countdown of LV to low error
static const uint32_t LV_low_timeout = 1000; //time for witch LV has to low voltage before error
static const uint32_t LV_low_threshold = 15000; //mV minimal voltage to run the car
static const uint32_t LV_low_dead = 2000; //LV disconected


static ECUP_Status_t ECUP_data; //data form can status ECUP
static ECUF_Dashboard_t ECUF_Dash_data; //data from can ECUF dashboard
static ECUF_Status_t ECUF_Status_data; //data from can ECUF dashboard
static ECUA_Status_t ECUA_data; //data from can ECUA
static VDCU_Status_t VDCU_data; //data from can VDCU
static MCF_ChannelMeasuresA_t MCF_measurements; //data from MCF

static uint32_t time_SDC_blink_L; //time variable for blinking led
static uint32_t time_SDC_blink_R; //time variable for blinking led

static uint32_t RTDS_countdowm = 0;
#define RTDS_time 2000 //cas hrani RTDS

/*
Configurates SPI comunication accordig to whitch chip should be comunicated with 
*/
void config_SDC_spi(SPI_HandleTypeDef * hspi) 
{
	hspi->Instance = SPI2;
  hspi->Init.Mode = SPI_MODE_MASTER;
  hspi->Init.Direction = SPI_DIRECTION_2LINES;
	hspi->Init.DataSize = SPI_DATASIZE_16BIT;
	hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
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

uint32_t play_RTDS(void) //hraje RTDS ton...snad
	{
	if(!RTDS_countdowm){ //pokud nehraje ton
		HAL_GPIO_WritePin(RTDS_GPIO_Port, RTDS_Pin, GPIO_PIN_SET); //zapnuti RTDS (ready to drive sound)
		RTDS_countdowm = HAL_GetTick() + RTDS_time; //odpocet 2s
		return 0;
	}else{
		if(RTDS_countdowm<HAL_GetTick()){
			HAL_GPIO_WritePin(RTDS_GPIO_Port, RTDS_Pin, GPIO_PIN_RESET); //vypnuti RTDS (ready to drive sound)
			RTDS_countdowm = 0; //konec odpoctu
			return 1; //vraci 1 pro zmenu stavu
		}else{
			return 0;	// vraci 0 pro zustani ve stejnem stavu
		}
	}
}
void check_voltage(void)
{
	if (LV_voltage_recive()>LV_low_dead){ //only if LV conected
		if (LV_low_countdown>0){
			if (LV_low_countdown > HAL_GetTick()){
				while(!units_set(GPIO_PIN_RESET,get_can_state())||(!aux_set(GPIO_PIN_RESET,get_can_state()))){} //gives power to all units and aux
				while (1){}
			}
		}
		if (LV_voltage_recive()< LV_low_threshold){
			if ((LV_low_countdown <= 0)){
				LV_low_countdown = HAL_GetTick()+LV_low_timeout;
			}
		}else{
			LV_low_countdown = 0;
		}
	}
	/*
	if (MCF_measurements.VDC >= 5500){
		HAL_GPIO_WritePin(TSALL_test_GPIO_Port,TSALL_test_Pin,GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(TSALL_test_GPIO_Port,TSALL_test_Pin,GPIO_PIN_RESET);
	}
	*/
}

enum ECUB_CarState* get_state(){ // gives current car state outside carstate.c
	return &state;
}

ECUF_Dashboard_t get_dash(){
	return ECUF_Dash_data;
}

ECUB_Status_t* get_can_state(){
	return &ECUB_Status;
}
/*which 1 = right which 2 = left else all set*/
void blink_SDB(ECUB_Status_t *ECUB_status,uint8_t which){
	switch (which){
		case 1:
			if (time_SDC_blink_R){
				if((time_SDC_blink_R+LED_SDC_BLINK)<HAL_GetTick()){
					time_SDC_blink_R = HAL_GetTick();
					if (HAL_GPIO_ReadPin(SDBR_GPIO_Port,SDBR_Pin)!= GPIO_PIN_SET){
						HAL_GPIO_WritePin(SDBR_GPIO_Port,SDBR_Pin,GPIO_PIN_SET);
					}else{
						HAL_GPIO_WritePin(SDBR_GPIO_Port,SDBR_Pin,GPIO_PIN_RESET);
					}
				}
			}else{
				time_SDC_blink_R = HAL_GetTick();
			}
			break;
		case 2:
			if (time_SDC_blink_L){
				if((time_SDC_blink_L+LED_SDC_BLINK<HAL_GetTick())){
					time_SDC_blink_L = HAL_GetTick();
					if (HAL_GPIO_ReadPin(SDBL_GPIO_Port,SDBL_Pin)!= GPIO_PIN_SET){
						HAL_GPIO_WritePin(SDBL_GPIO_Port,SDBL_Pin,GPIO_PIN_SET);
					}else{
						HAL_GPIO_WritePin(SDBL_GPIO_Port,SDBL_Pin,GPIO_PIN_RESET);
					}
				}
			}else{
				time_SDC_blink_L = HAL_GetTick();
			}
			break;
		default:
			set_SDB_led(GPIO_PIN_SET,ECUB_status,which);
	}
			
}

int paritycheck(uint16_t SDC_measure) //check if message is correctly recived
{
	//check first parity bit
	parity_check_summ = 0; 
	for(parity_for = 8;parity_for<16;parity_for++){
		if(SDC_measure & (1 << (parity_for))){
			parity_check_summ++;
		}
	}
	if(SDC_measure & (1 << 5)){
		parity_check_summ++;
	}
	parity_check_summ = parity_check_summ%2;
	if (!parity_check_summ){
		return 0;
	}
	//end of check first parity bit
	
	//check second parity bit
	parity_check_summ = 0; 
	for(parity_for = 16;parity_for>12;parity_for--){
		if(SDC_measure & (1 << (parity_for))){
			parity_check_summ++;
		}	
	}
	if(SDC_measure & (1 << 4)){
		parity_check_summ++;
	}
	parity_check_summ = parity_check_summ%2;
	if (!parity_check_summ){
		return 0;
	}
	//end of check second parity bit
	
	//check third parity bit
	parity_check_summ = 0; 
	for(parity_for = 8;parity_for<12;parity_for++){
		if(SDC_measure & (1 << (parity_for))){
			parity_check_summ++;
		}
	}
	if(SDC_measure & (1 << 3)){
		parity_check_summ++;
	}
	parity_check_summ = parity_check_summ%2;
	if (!parity_check_summ){
		return 0;
	}
	//end of check third parity bit
	
	//check fourth parity bit
	parity_check_summ = 0; 
	for(parity_for = 10;parity_for<14;parity_for++){
		if(SDC_measure & (1 << (parity_for))){
			parity_check_summ++;
		}
	}
	if(SDC_measure & (1 << 2)){
		parity_check_summ++;
	}
	parity_check_summ = parity_check_summ%2;
	if (!parity_check_summ){
		return 0;
	}
	//end of check fourth parity bit
	return 1; //if all correct

}

int measure_SDC(SPI_HandleTypeDef * SPI_handle){ //check SDC circuit
	config_SDC_spi(SPI_handle);
	for(SDC_parity_control = 0;SDC_parity_control<8;SDC_parity_control++){
		HAL_GPIO_WritePin(SDC_CS_GPIO_Port,SDC_CS_Pin,GPIO_PIN_RESET); //starts comunication...enable pin
		HAL_SPI_TransmitReceive(SPI_handle,(uint8_t*)&SDC_measure,(uint8_t*)&SDC_measure,1,5); //recives 16bit number from SDC chip
		HAL_GPIO_WritePin(SDC_CS_GPIO_Port,SDC_CS_Pin,GPIO_PIN_SET); //stops comunication...enable pin
		SDC_error = 0;
		if (paritycheck(SDC_measure)){
			break;
		}
	}
	
	if (!(SDC_measure & (1 << SDC_ECUF_OFFSET))){ // if ECUF bit is 0
		ECUB_Status.SDC_FRONT = 0; //can message...reason for SDC disconect
		SDC_error = 1; //error find
	}else{
		ECUB_Status.SDC_FRONT = 1; //can message...reason for SDC disconect
	}
	
	if (!(SDC_measure & (1 << SDC_SDBL_OFFSET))){ // if SDBL bit is 0
		if(ECUF_Status_data.SDC_SDBC){
			blink_SDB(&ECUB_Status,2);
		}else{
			HAL_GPIO_WritePin(SDBL_GPIO_Port,SDBL_Pin,GPIO_PIN_RESET);
		}
		change_to_NOT_READY(ECUB_Notready_reason_SDC_FAILURE);
		ECUB_Status.SDC_SDBL = 0; //can message...reason for SDC disconect
		SDC_error = 1; //error find
	}else{
		HAL_GPIO_WritePin(SDBL_GPIO_Port,SDBL_Pin,GPIO_PIN_RESET);
		ECUB_Status.SDC_SDBL = 1; //can message...reason for SDC disconect
	}
	
	if (!(SDC_measure & (1 << SDC_SDBR_OFFSET))){ // if SDBR bit is 0
		if(ECUB_Status.SDC_SDBL==0){
			HAL_GPIO_WritePin(SDBR_GPIO_Port,SDBR_Pin,GPIO_PIN_RESET);
		}else{
				blink_SDB(&ECUB_Status,1);
		}
		ECUB_Status.SDC_SDBR = 0; //can message...reason for SDC disconect
		SDC_error = 1; //error find
	}else{
		HAL_GPIO_WritePin(SDBR_GPIO_Port,SDBR_Pin,GPIO_PIN_RESET);
		ECUB_Status.SDC_SDBR = 1; //can message...reason for SDC disconect
	}
	
	if (!(SDC_measure & (1 << SDC_ECUMCR_OFFSET))){ // if ECUMCR bit is 0
		ECUB_Status.SDC_MCUR = 0; //can message...reason for SDC disconect		
		SDC_error = 1; //error find
	}else{
		ECUB_Status.SDC_MCUR = 1; //can message...reason for SDC disconect
	}
	
	if (!(SDC_measure & (1 << SDC_ECUA_OFFSET))){ // if ECUA bit is 0
		ECUB_Status.SDC_AMS = 0; //can message...reason for SDC disconect
		SDC_error = 1; //error find
	}else{
		ECUB_Status.SDC_AMS = 1; //can message...reason for SDC disconect
	}
	
	if (!(SDC_measure & (1 << SDC_HVD_OFFSET))){ // if HVD bit is 0
		ECUB_Status.SDC_HVD = 0; //can message...reason for SDC disconect
		SDC_error = 1; //error find
	}else{
		ECUB_Status.SDC_HVD = 1; //can message...reason for SDC disconect
	}
	
	if (!(SDC_measure & (1 << SDC_BSPD_OFFSET))){ // if BSPD bit is 0
		ECUB_Status.SDC_BSPD = 0; //can message...reason for SDC disconect
		SDC_error = 1; //error find
	}else{
		ECUB_Status.SDC_BSPD = 1; //can message...reason for SDC disconect
	}
	
	if (!(SDC_measure & (1 << SDC_TSMS_OFFSET))){ // if TSMS bit is 0
		ECUB_Status.SDC_TSMS = 0; //can message...reason for SDC disconect
		if (state >= ECUB_CarState_TS_ON){
			SDC_error = 1; //error find
		}
	}else{
		ECUB_Status.SDC_TSMS = 1; //can message...reason for SDC disconect
	}
	return SDC_error; //return error report 
}


void change_to_NOT_READY(enum ECUB_Notready_reason reason){ //change car state to NOT READY and set time in witch it must stay in notready
	state_notready = reason; //duvod not ready state
	state = ECUB_CarState_NOT_READY; //set car state
	cs_not_ready_lock = HAL_GetTick() + CS_NOT_READY_LOCK_TIME; //set time in NOTREADY after error
	if(ECUB_Status.RTDS_EN){
		HAL_GPIO_WritePin(RTDS_GPIO_Port, RTDS_Pin, GPIO_PIN_RESET);
		ECUB_Status.RTDS_EN = 0; //RDTS don´t plat can message
		RTDS_countdowm = 0;
	}
	if (HAL_GPIO_ReadPin(Fan2_GPIO_Port,Fan2_Pin) == GPIO_PIN_SET){ // if air conditioning still running
		HAL_GPIO_WritePin(WP1_GPIO_Port,WP1_Pin,GPIO_PIN_RESET); //stops water pumps for air conditioning :(
		HAL_GPIO_WritePin(WP2_GPIO_Port,WP2_Pin,GPIO_PIN_RESET); //stops water pumps for air conditioning :(
		HAL_GPIO_WritePin(Fan1_GPIO_Port,Fan1_Pin,GPIO_PIN_RESET); //stops fans for air conditioning :(
		HAL_GPIO_WritePin(Fan2_GPIO_Port,Fan2_Pin,GPIO_PIN_RESET); //stops fans for air conditioning :(	
	}
	if (HAL_GPIO_ReadPin(SDC_complete_GPIO_Port,SDC_complete_Pin) == GPIO_PIN_SET){
		HAL_GPIO_WritePin(SDC_complete_GPIO_Port,SDC_complete_Pin,GPIO_PIN_RESET); //disconnects HV SDC
	}
	TS_ON_presed = 0;
	//set_SDB_led(GPIO_PIN_RESET,&ECUB_Status); //lights SDB leds on
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
	LV_low_countdown = 0;
	return 1;
}


void check_mess(void){ //check if all messages form other units are being recived
	if(!ECUA_get_Status(&ECUA_data)){ //check if recived messsage and if did, copyed into given structure
		//change_to_NOT_READY(ECUB_Notready_reason_TIMEOUT_ECUA); //reason not ready state
	}
	if(!ECUF_get_Status(&ECUF_Status_data)){ //check if recived messsage and if did, copyed into given structure
		change_to_NOT_READY(ECUB_Notready_reason_TIMEOUT_ECUF); //reason not ready state
	}
	if(!ECUF_get_Dashboard(&ECUF_Dash_data)){ //check if recived messsage and if did, copyed into given structure
		change_to_NOT_READY(ECUB_Notready_reason_TIMEOUT_ECUF); //reason not ready state
	}
	if(!ECUP_get_Status(&ECUP_data)){ //check if recived messsage and if did, copyed into given structure
		change_to_NOT_READY(ECUB_Notready_reason_TIMEOUT_ECUP); //reason not ready state
	}
	if(!MCR_get_GeneralReport(NULL)){ //check if recived messsage and if did, copyed into given structure
		change_to_NOT_READY(ECUB_Notready_reason_TIMEOUT_MC); //reason not ready state
	}
	if(!VDCU_get_Status(&VDCU_data)){ //check if recived messsage and if did, copyed into given structure
		change_to_NOT_READY(ECUB_Notready_reason_TIMEOUT_VDCU); //reason not ready state
	}
	if(!MCF_get_GeneralReport(NULL)){ //check if recived messsage and if did, copyed into given structure
		change_to_NOT_READY(ECUB_Notready_reason_TIMEOUT_MC); //reason not ready state
	}
	MCF_get_ChannelMeasuresA(&MCF_measurements);
}



void blink_led(uint32_t number_of_blinks){
	if (number_of_blinks>number_of_bliks){
		if (LED_blink_time<HAL_GetTick()){
			HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin); //debug LED
			LED_blink_time = HAL_GetTick() + LED_blink; //sets time in led state
			number_of_bliks++; //counts bliks 
		}
	}else{
		LED_blink_time = HAL_GetTick() + LED_blink*10; //time of led time out 
		number_of_bliks = 0; //resets bliks 
	}
}

void carstate_process(SPI_HandleTypeDef * SPI_handle,CAN_HandleTypeDef* hcan){ //main car state machine
	if (measure_SDC(SPI_handle)){ //measure if any error on SDC circute
		change_to_NOT_READY(ECUB_Notready_reason_SDC_FAILURE); //change state because of critical error report
		HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_SET); //debug led
	}else{
		HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET); //debug led
	}
	check_mess(); //check that messages are being recived
	if (!ECUF_Status_data.SDC_SDBC){
		change_to_NOT_READY(ECUB_Notready_reason_SDC_FAILURE);
	}
	if(!ECUF_Status_data.SDC_FWIL){
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
			state = ECUB_CarState_LATCHED; //critical state of car
			goto there_is_no_escape;
		}
	}
	if (state >= ECUB_CarState_PRECHARGE){
		if (VDCU_data.State==VDCU_VDCU_State_ERROR){
			state = ECUB_CarState_TS_READY;
		}
	}
	check_voltage(); //check if LV battery have high enougth voltage
	brakelightprocess(ECUP_data,&ECUB_Status); //check brake light
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
			state_notready = ECUB_Notready_reason_NONE; //error message delated 
			break;
			
			
		case ECUB_CarState_LATCHED: //critical error...only way to get from here is to shut down the car
			there_is_no_escape:
			set_SDB_led(GPIO_PIN_RESET,&ECUB_Status,3); //sets SDB leds off
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
				HAL_CAN_Receive_IT(hcan,CAN_FIFO0);
				ECUB_Status.SEQ++; //message just for check that this message is different from last one
			}
			goto there_is_no_escape;
		
		
		case ECUB_CarState_TS_READY: //tractive system on ready...state needed for startitng procedure
			blink_led(5); //sets to 3bliks period of debug led
			if(ECUF_get_Dashboard(&ECUF_Dash_data)){ //check if recived messsage and if did, copyed into given structure
				if ((VDCU_data.State!=VDCU_VDCU_State_ERROR)&&(ECUF_Dash_data.TSON==1)){ //if preded button tractive system on
					HAL_GPIO_WritePin(SDC_complete_GPIO_Port,SDC_complete_Pin,GPIO_PIN_SET); //connects HV SDC
					HAL_GPIO_WritePin(WP1_GPIO_Port,WP1_Pin,GPIO_PIN_SET); //starts water pumps for air conditioning
					HAL_GPIO_WritePin(WP2_GPIO_Port,WP2_Pin,GPIO_PIN_SET); //starts water pumps for air conditioning
					HAL_GPIO_WritePin(Fan1_GPIO_Port,Fan1_Pin,GPIO_PIN_SET); //starts fans for air conditioning
					HAL_GPIO_WritePin(Fan2_GPIO_Port,Fan2_Pin,GPIO_PIN_SET); //starts fans for air conditioning
					state = ECUB_CarState_PRECHARGE; //set car state
					TSMS_set_countdown = HAL_GetTick() + TSMS_set_timeout;
				}
			}else{
				change_to_NOT_READY(ECUB_Notready_reason_TIMEOUT_ECUF); //change car state to notready
			}
			break;
			
			
		case ECUB_CarState_PRECHARGE: //precharge condenser if i got it right
			if((ECUB_Status.SDC_TSMS!=1)&&(HAL_GetTick()>TSMS_set_countdown)){
				state = ECUB_CarState_TS_READY;
				HAL_GPIO_WritePin(SDC_complete_GPIO_Port,SDC_complete_Pin,GPIO_PIN_RESET); //connects HV SDC
			}
			blink_led(7); //sets to 4bliks period of debug led			
			if ((!ECUA_data.FT_ANY)&&(ECUA_data.AIRsState == 9)&&(ECUB_Status.SDC_TSMS==1)){//(!ECUA_data.AMSState)&&(ECUA_data.AIRsState)){ //mising precharge finished
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
			break;
		
		
		default:
			state = ECUB_CarState_NOT_READY; //should never happen but just to be sure
			break;
		}
	
		if (ECUB_Status_need_to_send()){ //if is needed to send can message
			if(state == ECUB_CarState_STARTED){ //if state is in started SDC is measured only when can message is needed to be send
				if (measure_SDC(SPI_handle)){ //measure if any error on SDC circute
					HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_SET); //debug led
				}else{
					HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET); //debug led
				}
			}
			if (HAL_GPIO_ReadPin(Det1_GPIO_Port,Det1_Pin) == GPIO_PIN_SET){ //check if modules are present
				ECUB_Status.Det_MOD1 = 1; //module is present can message 
			}else{
				ECUB_Status.Det_MOD1 = 0; //module is not present can message
			}
			if (HAL_GPIO_ReadPin(Det2_GPIO_Port,Det2_Pin) == GPIO_PIN_SET){ //check if modules are present
				ECUB_Status.Det_MOD2 = 1; //module is present can message
			}else{
				ECUB_Status.Det_MOD2 = 0; //module is not present can message
			}
			if (HAL_GPIO_ReadPin(Det3_GPIO_Port,Det3_Pin) == GPIO_PIN_SET){ //check if modules are present
				ECUB_Status.Det_MOD3 = 1; //module is present can message
			}else{
				ECUB_Status.Det_MOD3 = 0; //module is not present can message
			}
			ECUB_Status.CarState = state; //put state of car to can message
			ECUB_Status.CarState_Notready = state_notready; //put NOTREADY reason to can message
 		  ECUB_send_Status_s(&ECUB_Status); //sendidng can message
			ECUB_Status.SEQ++; //message just for check that this message is different from last one
			HAL_CAN_Receive_IT(hcan,CAN_FIFO0); //fixes hal can structure
		}
}
