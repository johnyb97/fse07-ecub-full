#include "periphery_set.h"

static GPIO_PinState pinstate;
static uint32_t RTDS_countdowm = 0;
static uint32_t last_bat_check = 0;
static uint32_t wait_for_next = 0;

#define RTDS_time 2000 //cas hrani RTDS
#define reference_voltage_battery 5 //spravne napeti na baterce
#define pauza 10 //cas v milisekundach urceny pro odlozeni vykonani funkce

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


uint32_t units_set(GPIO_PinState state,ECUB_Status_t *ECUB_Status) //zapnuti vypnuti napajeni jednotek
	{
	pinstate = HAL_GPIO_ReadPin(ECUA_en_GPIO_Port,ECUA_en_Pin); //jaky byl stav pinu
	if (pinstate != state){ //pokud se meni pin
		HAL_GPIO_WritePin(ECUA_en_GPIO_Port,ECUA_en_Pin,pinstate); //nastav pin na state
		if (state == SET){
			ECUB_Status->PWR_ECUA_EN = 1; //can message
		}else{
			ECUB_Status->PWR_ECUA_EN = 0; //can message
		}
		wait_for_next = HAL_GetTick(); //pauza ciste pro jistotu
		return 0; //nezmenily se vsechny hodnoty
	}
	
	pinstate = HAL_GPIO_ReadPin(ECUF_en_GPIO_Port,ECUF_en_Pin); //jaky byl stav pinu
	if (pinstate != state){ //pokud se meni pin
		if(wait_for_next+pauza<HAL_GetTick()){ //pokud ubehlo dost casu od posledni pauzy
			HAL_GPIO_WritePin(ECUF_en_GPIO_Port,ECUF_en_Pin,pinstate); //nastav pin na state
			if (state == SET){
				ECUB_Status->PWR_ECUF_EN = 1; //can message
			}else{
				ECUB_Status->PWR_ECUF_EN = 0; //can message
			}
			wait_for_next = HAL_GetTick(); //pauza ciste pro jistotu
		}
		return 0; //nezmenily se vsechny hodnoty
	}	
	pinstate = HAL_GPIO_ReadPin(ECUMCR_en_GPIO_Port,ECUMCR_en_Pin); //jaky byl stav pinu
	if (pinstate != state){ //pokud se meni pin
		if(wait_for_next+pauza<HAL_GetTick()){ //pokud ubehlo dost casu od posledni pauzy
			HAL_GPIO_WritePin(ECUMCR_en_GPIO_Port,ECUMCR_en_Pin,pinstate); //nastav pin na state
			if (state == SET){
				ECUB_Status->PWR_MCUR_EN = 1; //can message
			}else{
				ECUB_Status->PWR_MCUR_EN = 0; //can message
			}
			wait_for_next = HAL_GetTick(); //pauza ciste pro jistotu
		}
		return 0; //nezmenily se vsechny hodnoty
	}
	
	pinstate = HAL_GPIO_ReadPin(ECUMCF_en_GPIO_Port,ECUMCF_en_Pin); //jaky byl stav pinu
	if (pinstate != state){ //pokud se meni pin
		if(wait_for_next+pauza<HAL_GetTick()){ //pokud ubehlo dost casu od posledni pauzy
			HAL_GPIO_WritePin(ECUMCF_en_GPIO_Port,ECUMCF_en_Pin,pinstate); //nastav pin na state
			if (state == SET){
				ECUB_Status->PWR_MCUF_EN = 1; //can message
			}else{
				ECUB_Status->PWR_MCUF_EN = 0; //can message
			}
			wait_for_next = HAL_GetTick(); //pauza ciste pro jistotu
		}
		return 0; //nezmenily se vsechny hodnoty
	}
	
	pinstate = HAL_GPIO_ReadPin(EM_en_GPIO_Port,EM_en_Pin); //jaky byl stav pinu
	if (pinstate != state){ //pokud se meni pin
		if(wait_for_next+pauza<HAL_GetTick()){ //pokud ubehlo dost casu od posledni pauzy
			HAL_GPIO_WritePin(EM_en_GPIO_Port,EM_en_Pin,pinstate); //nastav pin na state
			if (state == SET){
				ECUB_Status->PWR_EM_EN = 1; //can message
			}else{
				ECUB_Status->PWR_EM_EN = 0; //can message
			}
			wait_for_next = HAL_GetTick(); //pauza ciste pro jistotu
			return 1; //zmenily se vsechny hodnoty
		}
		return 0;
	}
	return 1;
}

uint32_t aux_set(GPIO_PinState state,ECUB_Status_t *ECUB_Status) //nastavi napajeni aux
	{
	pinstate = HAL_GPIO_ReadPin(Aux_1_GPIO_Port,Aux_1_Pin); //jaky byl stav pinu
	if (pinstate != state){ //pokud se meni pin
		HAL_GPIO_WritePin(Aux_1_GPIO_Port,Aux_1_Pin,pinstate); //nastav pin na state
		if (state == SET){
			ECUB_Status->PWR_AUX1_EN = 1; //can message
		}else{
			ECUB_Status->PWR_AUX1_EN = 0; //can message
		}
		wait_for_next = HAL_GetTick(); //pauza ciste pro jistotu
		return 0; //nezmenily se vsechny hodnoty
	}
	pinstate = HAL_GPIO_ReadPin(Aux_2_GPIO_Port,Aux_2_Pin); //jaky byl stav pinu
	if (pinstate != state){ //pokud se meni pin
		if(wait_for_next+pauza<HAL_GetTick()){ //pokud ubehlo dost casu od posledni pauzy
			HAL_GPIO_WritePin(Aux_2_GPIO_Port,Aux_2_Pin,pinstate); //nastav pin na state
			if (state == SET){
			ECUB_Status->PWR_AUX2_EN = 1; //can message
		}else{
			ECUB_Status->PWR_AUX2_EN = 0; //can message
		}
			wait_for_next = HAL_GetTick(); //pauza ciste pro jistotu
		}
		return 0; //nezmenily se vsechny hodnoty
	}
	
	pinstate = HAL_GPIO_ReadPin(Aux_3_GPIO_Port,Aux_3_Pin); //jaky byl stav pinu
	if ((pinstate != state)&&(wait_for_next+100<HAL_GetTick())){ //pokud se meni pin
		if(wait_for_next+pauza<HAL_GetTick()){ //pokud ubehlo dost casu od posledni pauzy
			HAL_GPIO_WritePin(Aux_3_GPIO_Port,Aux_3_Pin,pinstate); //nastav pin na state
			if (state == SET){
				ECUB_Status->PWR_AUX3_EN = 1; //can message
			}else{
				ECUB_Status->PWR_AUX3_EN = 0; //can message
			}
			wait_for_next = HAL_GetTick(); //pauza ciste pro jistotu
			return 1; //zmenily se vsechny hodnoty
		}
		return 0;
	}
	return 1;
}
void battery_state_process(uint32_t battery_voltage) //nabijeni baterky
	{ 
	switch(HAL_GPIO_ReadPin(Charge_en_GPIO_Port,Charge_en_Pin)){ //v jakem stavu se ma nachazet baterie
		
		case GPIO_PIN_SET: //baterie se ma nabijet
			if (battery_voltage >= reference_voltage_battery){ //pokud je dostatecne napeti na baterii
					HAL_GPIO_WritePin(Charge_en_GPIO_Port,Charge_en_Pin,GPIO_PIN_RESET); //vypni nabijeni
			}
			if (!HAL_GPIO_ReadPin(Charge_status_GPIO_Port,Charge_status_Pin)){ //hlaseni chyby nebo konce napajeni...chyba je stridani SET a RESET stavu....k tomu slouzi casovac
				if (!last_bat_check){ //pokud neni nastaven casovac
					last_bat_check = HAL_GetTick(); //nastaveni casovace periody zmeny v pripade chyby
				}else if(((HAL_GetTick()-last_bat_check)/500)%2){ //perioda zmeny..pin se nezmenil: konec nabijeni
						HAL_GPIO_WritePin(Charge_en_GPIO_Port,Charge_en_Pin,GPIO_PIN_RESET); //vypni nabijeni
					
				}
			}else{ //
				if(last_bat_check){ //pin se zmenil...jsme v prdeli mame chybu
				HAL_GPIO_WritePin(Charge_en_GPIO_Port,Charge_en_Pin,GPIO_PIN_RESET); //vypni nabijeni
				//modli se
				}
			}
			break;
		case GPIO_PIN_RESET: //baterie se nemá nabijet
			if (battery_voltage < reference_voltage_battery){ //pokud je nizke napeti na baterii
							HAL_GPIO_WritePin(Charge_en_GPIO_Port,Charge_en_Pin,GPIO_PIN_SET); //zapni nabijeni
			}
			break;
	}
}
void set_SDB_led(GPIO_PinState state,ECUB_Status_t *ECUB_Status) //nastaveni LEDky pro pro SDB
	{
	if(HAL_GPIO_ReadPin(SDBR_GPIO_Port,SDBR_Pin)!=state){ //pokud je treba zmenin stav LEDek
		HAL_GPIO_WritePin(SDBR_GPIO_Port,SDBR_Pin,state); //zmen stav prave LEDky
		HAL_GPIO_WritePin(SDBL_GPIO_Port,SDBL_Pin,state); //zmen stav leve LEDky
		if (state == SET){
				ECUB_Status->SDBL_LED_EN = 1; //can message
				ECUB_Status->SDBR_LED_EN = 1; //can message
			}else{
				ECUB_Status->SDBL_LED_EN = 0; //can message
				ECUB_Status->SDBR_LED_EN = 0; //can message
			}
	}
}
